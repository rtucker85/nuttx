/****************************************************************************
 * arch/arm/src/am263x/am263x_adc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* The on-chip 12-bit SAR ADCs, ADC0-4.
 *
 * Each ADC scans a fixed list of up to 16 channels, one SOC (start of
 * conversion) per channel, all on the same trigger: software, an RTI, or
 * an ePWM's SOCA or SOCB.  ADCINT1 fires at the end of the last SOC and
 * the handler passes every result to the upper half.  ADCINT1 reaches the
 * VIM through the CONTROLSS INTXBAR, output
 * CONFIG_AM263X_ADC_INTXBAR + adc.
 *
 * The trims come from eFuse in hardware.  Conversions are single-ended,
 * 12-bit, against the reference of the ADC's group: ADC0, ADC1/2 or
 * ADC3/4.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "hardware/am263x_adc.h"
#include "am263x_controlss.h"
#include "am263x_adc.h"

#ifdef CONFIG_AM263X_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Time for the analog circuits to power up and the reference to settle */

#define ADC_POWERUP_US        1000

/* Wait this long for the reference monitor to report good */

#define ADC_REFGOOD_US        1000

#define ADC_ACQPS             (CONFIG_AM263X_ADC_SAMPLE_WINDOW - 1)

#if CONFIG_AM263X_ADC_INTXBAR + AM263X_ADC_NINSTANCES > \
    AM263X_INTXBAR_NOUTPUTS
#  error CONFIG_AM263X_ADC_INTXBAR leaves no room for ADC4
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_adc_s
{
  struct adc_dev_s dev;                   /* Upper half's view, first */
  FAR const struct adc_callback_s *cb;    /* Upper half callbacks */
  uintptr_t base;                         /* Control registers */
  uintptr_t rbase;                        /* Result registers */
  uint8_t   adc;                          /* 0-4 */
  uint8_t   trigger;                      /* ADC_TRIGSEL_* */
  uint8_t   nchannels;
  uint8_t   channels[AM263X_ADC_NSOC];
  uint32_t  overruns;                     /* Scans lost since setup() */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                      unsigned long arg);
static int  adc_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adcops =
{
  .ao_bind     = adc_bind,
  .ao_reset    = adc_reset,
  .ao_setup    = adc_setup,
  .ao_shutdown = adc_shutdown,
  .ao_rxint    = adc_rxint,
  .ao_ioctl    = adc_ioctl,
};

/* Which ADCs Kconfig enables */

static const uint8_t g_adc_enabled =
#ifdef CONFIG_AM263X_ADC0
  (1 << 0) |
#endif
#ifdef CONFIG_AM263X_ADC1
  (1 << 1) |
#endif
#ifdef CONFIG_AM263X_ADC2
  (1 << 2) |
#endif
#ifdef CONFIG_AM263X_ADC3
  (1 << 3) |
#endif
#ifdef CONFIG_AM263X_ADC4
  (1 << 4) |
#endif
  0;

static struct am263x_adc_s g_adc[AM263X_ADC_NINSTANCES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t adc_getreg(FAR struct am263x_adc_s *priv,
                                  unsigned int offset)
{
  return getreg16(priv->base + offset);
}

static inline void adc_putreg(FAR struct am263x_adc_s *priv,
                              unsigned int offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

static inline int adc_irq(FAR struct am263x_adc_s *priv)
{
  return AM263X_IRQ_INTXBAR(CONFIG_AM263X_ADC_INTXBAR + priv->adc);
}

/****************************************************************************
 * Name: adc_reference
 *
 * Description:
 *   Turn on the reference of ADC<adc>'s group, if it is not already, and
 *   its monitor.  With CONFIG_AM263X_ADC_EXTERNAL_REF the board drives
 *   VREFHI and only the monitor is enabled.
 *
 ****************************************************************************/

static void adc_reference(int adc)
{
  uint16_t refok;
  uint16_t good;
  int refbuf;
  int i;

  if (adc == 0)
    {
      refbuf = 0;
      refok  = TOP_CTRL_ADC_REFOK_EN_ADC0;
      good   = TOP_CTRL_ADC_REF_GOOD_ADC0;
    }
  else if (adc <= 2)
    {
      refbuf = 0;
      refok  = TOP_CTRL_ADC_REFOK_EN_ADC12;
      good   = TOP_CTRL_ADC_REF_GOOD_ADC12;
    }
  else
    {
      refbuf = 1;
      refok  = TOP_CTRL_ADC_REFOK_EN_ADC34;
      good   = TOP_CTRL_ADC_REF_GOOD_ADC34;
    }

#ifndef CONFIG_AM263X_ADC_EXTERNAL_REF
  if ((getreg16(AM263X_TOP_CTRL_ADC_REFBUF_CTRL(refbuf)) &
       TOP_CTRL_ADC_REFBUF_ENABLE) != TOP_CTRL_ADC_REFBUF_ENABLE)
    {
      /* Mask the HHV monitor while the buffer starts, or it can reset the
       * device.
       */

      am263x_top_ctrl_modify16(AM263X_TOP_CTRL_MASK_ANA_ISO, 0,
                               TOP_CTRL_MASK_ANA_ISO_HHV);
      am263x_top_ctrl_modify16(AM263X_TOP_CTRL_ADC_REFBUF_CTRL(refbuf), 0,
                               TOP_CTRL_ADC_REFBUF_ENABLE);
      am263x_top_ctrl_modify16(AM263X_TOP_CTRL_MASK_ANA_ISO,
                               TOP_CTRL_MASK_ANA_ISO_HHV, 0);
    }
#else
  UNUSED(refbuf);
#endif

  am263x_top_ctrl_modify16(AM263X_TOP_CTRL_ADC_REF_COMP_CTRL, 0, refok);

  for (i = 0; i < ADC_REFGOOD_US / 10; i++)
    {
      if ((getreg16(AM263X_TOP_CTRL_ADC_REF_GOOD) & good) == good)
        {
          return;
        }

      up_udelay(10);
    }

  awarn("WARNING: ADC%d reference not good: %04x\n", adc,
        getreg16(AM263X_TOP_CTRL_ADC_REF_GOOD));
}

/****************************************************************************
 * Name: adc_bind
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
  FAR struct am263x_adc_s *priv = dev->ad_priv;

  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Called at registration.  The ADC stays unclocked until setup().
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  UNUSED(dev);
}

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Clock and power the ADC, program the scan and attach the interrupt.
 *   With a hardware trigger the ADC converts from here on; results reach
 *   the upper half once rxint() enables the interrupt.
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct am263x_adc_s *priv = dev->ad_priv;
  uint32_t soc;
  int ret;
  int i;

  am263x_css_ctrl_modify(AM263X_CSS_CTRL_ADC_CLK_GATE(priv->adc),
                         CSS_CTRL_CLK_GATED, CSS_CTRL_CLK_UNGATED);

  adc_reference(priv->adc);

  /* ADCCLK = SYSCLK / prescale; ADCINT at the end of conversion */

  adc_putreg(priv, AM263X_ADC_CTL2,
             CONFIG_AM263X_ADC_PRESCALE << ADC_CTL2_PRESCALE_SHIFT);
  adc_putreg(priv, AM263X_ADC_CTL1, ADC_CTL1_INTPULSEPOS);
  adc_putreg(priv, AM263X_ADC_CTL1,
             ADC_CTL1_INTPULSEPOS | ADC_CTL1_ADCPWDNZ);

  up_udelay(ADC_POWERUP_US);

  /* One SOC per channel, the rest off.  SOCs triggered together convert
   * in round-robin order from SOC0.
   */

  adc_putreg(priv, AM263X_ADC_INTSEL1N2, 0);
  adc_putreg(priv, AM263X_ADC_SOCPRICTL, 0);

  for (i = 0; i < AM263X_ADC_NSOC; i++)
    {
      soc = 0;
      if (i < priv->nchannels)
        {
          soc = (ADC_ACQPS << ADC_SOCCTL_ACQPS_SHIFT) |
                ((uint32_t)priv->channels[i] << ADC_SOCCTL_CHSEL_SHIFT) |
                ((uint32_t)priv->trigger << ADC_SOCCTL_TRIGSEL_SHIFT);
        }

      putreg32(soc, priv->base + AM263X_ADC_SOCCTL(i));
    }

  priv->overruns = 0;
  adc_putreg(priv, AM263X_ADC_SOCOVFCLR1, 0xffff);
  adc_putreg(priv, AM263X_ADC_INTOVFCLR, ADC_INT(1));
  adc_putreg(priv, AM263X_ADC_INTFLGCLR, ADC_INT(1));

  /* ADCINT1 on the last SOC's end of conversion */

  adc_putreg(priv, AM263X_ADC_INTSEL1N2,
             ((priv->nchannels - 1) << ADC_INTSEL_SEL_SHIFT) |
             ADC_INTSEL_E);

  ret = irq_attach(adc_irq(priv), adc_interrupt, priv);
  if (ret < 0)
    {
      adc_shutdown(dev);
      return ret;
    }

  am263x_intxbar_select(CONFIG_AM263X_ADC_INTXBAR + priv->adc,
                        INTXBAR_GROUP_ADC, INTXBAR_ADC_INT(priv->adc, 1),
                        true);

  ainfo("ADC%d: %d channels, trigger %d\n", priv->adc, priv->nchannels,
        priv->trigger);
  return OK;
}

/****************************************************************************
 * Name: adc_shutdown
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct am263x_adc_s *priv = dev->ad_priv;
  int i;

  up_disable_irq(adc_irq(priv));
  am263x_intxbar_select(CONFIG_AM263X_ADC_INTXBAR + priv->adc,
                        INTXBAR_GROUP_ADC, INTXBAR_ADC_INT(priv->adc, 1),
                        false);
  irq_detach(adc_irq(priv));

  /* Stop triggers reaching the SOCs, then power down and gate.  The
   * reference stays on: another ADC may share it.
   */

  adc_putreg(priv, AM263X_ADC_INTSEL1N2, 0);
  for (i = 0; i < AM263X_ADC_NSOC; i++)
    {
      putreg32(0, priv->base + AM263X_ADC_SOCCTL(i));
    }

  adc_putreg(priv, AM263X_ADC_INTFLGCLR, ADC_INT(1));
  adc_putreg(priv, AM263X_ADC_CTL1, 0);

  am263x_css_ctrl_modify(AM263X_CSS_CTRL_ADC_CLK_GATE(priv->adc),
                         0, CSS_CTRL_CLK_GATED);

  if (priv->overruns > 0)
    {
      awarn("WARNING: ADC%d lost %" PRIu32 " scans\n", priv->adc,
            priv->overruns);
    }
}

/****************************************************************************
 * Name: adc_rxint
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct am263x_adc_s *priv = dev->ad_priv;

  if (enable)
    {
      adc_putreg(priv, AM263X_ADC_INTOVFCLR, ADC_INT(1));
      adc_putreg(priv, AM263X_ADC_INTFLGCLR, ADC_INT(1));
      up_enable_irq(adc_irq(priv));
    }
  else
    {
      up_disable_irq(adc_irq(priv));
    }
}

/****************************************************************************
 * Name: adc_ioctl
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct am263x_adc_s *priv = dev->ad_priv;

  switch (cmd)
    {
      case ANIOC_TRIGGER:

        /* Force every SOC of the scan, whatever the trigger */

        adc_putreg(priv, AM263X_ADC_SOCFRC1, (1 << priv->nchannels) - 1);
        return OK;

      case ANIOC_GET_NCHANNELS:
        return priv->nchannels;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   End of scan: hand every result to the upper half.  If another scan
 *   finished before this one was handled (ADCINT1 overflow), its results
 *   have overwritten this one's; they are passed on all the same, and
 *   the lost scan is counted (reported at shutdown, not from here).
 *
 ****************************************************************************/

static int adc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct am263x_adc_s *priv = arg;
  uint16_t data;
  int i;

  if ((adc_getreg(priv, AM263X_ADC_INTFLG) & ADC_INT(1)) == 0)
    {
      return OK;
    }

  for (i = 0; i < priv->nchannels; i++)
    {
      data = getreg16(priv->rbase + AM263X_ADC_RESULT(i));
      if (priv->cb != NULL && priv->cb->au_receive != NULL)
        {
          priv->cb->au_receive(&priv->dev, priv->channels[i], data);
        }
    }

  if ((adc_getreg(priv, AM263X_ADC_INTOVF) & ADC_INT(1)) != 0)
    {
      adc_putreg(priv, AM263X_ADC_INTOVFCLR, ADC_INT(1));
      priv->overruns++;
    }

  adc_putreg(priv, AM263X_ADC_INTFLGCLR, ADC_INT(1));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_adc_initialize
 ****************************************************************************/

FAR struct adc_dev_s *
am263x_adc_initialize(int adc, FAR const struct am263x_adc_config_s *config)
{
  FAR struct am263x_adc_s *priv;
  int i;

  if (adc < 0 || adc >= AM263X_ADC_NINSTANCES ||
      (g_adc_enabled & (1 << adc)) == 0 || config == NULL ||
      config->nchannels < 1 || config->nchannels > AM263X_ADC_NSOC ||
      config->trigger > ADC_TRIGSEL_MAX)
    {
      aerr("ERROR: ADC%d not enabled or bad configuration\n", adc);
      return NULL;
    }

  for (i = 0; i < config->nchannels; i++)
    {
      if (config->channels[i] > ADC_CHSEL_MAX)
        {
          aerr("ERROR: ADC%d: no channel %d\n", adc, config->channels[i]);
          return NULL;
        }
    }

  priv = &g_adc[adc];
  priv->dev.ad_ops  = &g_adcops;
  priv->dev.ad_priv = priv;
  priv->base        = AM263X_ADC_BASE(adc);
  priv->rbase       = AM263X_ADC_RESULT_BASE(adc);
  priv->adc         = adc;
  priv->trigger     = config->trigger;
  priv->nchannels   = config->nchannels;

  for (i = 0; i < config->nchannels; i++)
    {
      priv->channels[i] = config->channels[i];
    }

  return &priv->dev;
}

#endif /* CONFIG_AM263X_ADC */
