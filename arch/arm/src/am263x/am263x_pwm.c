/****************************************************************************
 * arch/arm/src/am263x/am263x_pwm.c
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

/* The ePWMs as PWM lower halves, with their ADC SOC outputs.
 *
 * Each ePWM counts up from 0 to TBPRD on a time base divided down from
 * SYSCLK.  Output A (channel 1) is set at zero and cleared at CMPA, output
 * B (channel 2) the same with CMPB; CPOL low inverts that.  0% and 100%
 * duty are a continuous software force, as is the idle level while
 * stopped.  CMPA, CMPB and TBPRD are shadowed and load at counter zero,
 * so a duty or frequency change takes effect at the next period; a
 * change of time-base divider takes effect at once.
 *
 * SOCA and SOCB come from the event-trigger block (ETSEL, ETSOCPS); see
 * am263x_pwm_socconfig().  Trip zones, dead band, high resolution and
 * the sync chain are left at their reset state: off.
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

#include <fixedmath.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/pwm.h>

#include "arm_internal.h"
#include "hardware/am263x_epwm.h"
#include "am263x_clockconfig.h"
#include "am263x_controlss.h"
#include "am263x_pwm.h"

#ifdef CONFIG_AM263X_EPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Two outputs per ePWM.  TBPRD stops one short of 0xffff so that a
 * compare of TBPRD + 1, never reached, still fits.
 */

#define EPWM_NOUTPUTS     2
#define EPWM_PRD_MAX      0xfffe
#define EPWM_PRD_MIN      1

#ifdef CONFIG_PWM_FREQUENCY_FIXED
#  error CONFIG_PWM_FREQUENCY_FIXED is not supported
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_epwm_s
{
  FAR const struct pwm_ops_s *ops;        /* Lower half operations, first */
  uintptr_t base;                         /* Register base */
  uint8_t   pwm;                          /* 0-31 */
  bool      running;                      /* Counter running */
  uint32_t  frequency;                    /* Current frequency, Hz */
  uint16_t  tbdiv;                        /* TBCTL HSPCLKDIV and CLKDIV */
  uint16_t  prd;                          /* TBPRD */
  uint8_t   dcpol[EPWM_NOUTPUTS];         /* Idle level, PWM_DCPOL_* */
  struct am263x_epwm_soc_s soc[2];        /* SOCA, SOCB */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int epwm_setup(FAR struct pwm_lowerhalf_s *dev);
static int epwm_shutdown(FAR struct pwm_lowerhalf_s *dev);
static int epwm_start(FAR struct pwm_lowerhalf_s *dev,
                      FAR const struct pwm_info_s *info);
static int epwm_stop(FAR struct pwm_lowerhalf_s *dev);
static int epwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                      unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pwm_ops_s g_epwmops =
{
  .setup    = epwm_setup,
  .shutdown = epwm_shutdown,
  .start    = epwm_start,
  .stop     = epwm_stop,
  .ioctl    = epwm_ioctl,
};

static FAR struct am263x_epwm_s *g_epwm[AM263X_EPWM_NINSTANCES];
static spinlock_t g_epwm_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t epwm_getreg(FAR struct am263x_epwm_s *priv,
                                   unsigned int offset)
{
  return getreg16(priv->base + offset);
}

static inline void epwm_putreg(FAR struct am263x_epwm_s *priv,
                               unsigned int offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

static inline void epwm_modifyreg(FAR struct am263x_epwm_s *priv,
                                  unsigned int offset, uint16_t clrbits,
                                  uint16_t setbits)
{
  epwm_putreg(priv, offset,
              (epwm_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: epwm_timebase
 *
 * Description:
 *   Pick the smallest time-base divider that fits frequency into a 16-bit
 *   period, for the finest duty resolution.  TBCLK = SYSCLK /
 *   (HSPCLKDIV * CLKDIV), HSPCLKDIV 1 or 2-14 even, CLKDIV 1-128.
 *
 ****************************************************************************/

static int epwm_timebase(uint32_t frequency, FAR uint16_t *tbdiv,
                         FAR uint16_t *prd)
{
  uint32_t bestdiv = UINT32_MAX;
  uint64_t ticks;
  uint32_t div;
  int clkdiv;
  int hsp;

  if (frequency == 0)
    {
      return -EINVAL;
    }

  for (clkdiv = 0; clkdiv < 8; clkdiv++)
    {
      for (hsp = 0; hsp < 8; hsp++)
        {
          div = (1u << clkdiv) * (hsp == 0 ? 1 : 2 * hsp);
          if (div >= bestdiv)
            {
              continue;
            }

          ticks = ((uint64_t)AM263X_SYSCLK_FREQUENCY +
                   (uint64_t)div * frequency / 2) /
                  ((uint64_t)div * frequency);
          if (ticks >= EPWM_PRD_MIN + 1 && ticks <= EPWM_PRD_MAX + 1)
            {
              bestdiv = div;
              *tbdiv  = (hsp << EPWM_TBCTL_HSPCLKDIV_SHIFT) |
                        (clkdiv << EPWM_TBCTL_CLKDIV_SHIFT);
              *prd    = ticks - 1;
            }
        }
    }

  return bestdiv == UINT32_MAX ? -EINVAL : OK;
}

/****************************************************************************
 * Name: epwm_force
 *
 * Description:
 *   Continuously force output <out> (0 A, 1 B) low or high, or release it
 *   to the action qualifier (EPWM_AQCSFRC_OFF).
 *
 ****************************************************************************/

static void epwm_force(FAR struct am263x_epwm_s *priv, int out,
                       uint16_t force)
{
  int shift = out == 0 ? EPWM_AQCSFRC_CSFA_SHIFT : EPWM_AQCSFRC_CSFB_SHIFT;

  epwm_modifyreg(priv, AM263X_EPWM_AQCSFRC, EPWM_AQCSFRC_MASK << shift,
                 force << shift);
}

/****************************************************************************
 * Name: epwm_idle
 *
 * Description:
 *   Hold output <out> at its idle level.
 *
 ****************************************************************************/

static void epwm_idle(FAR struct am263x_epwm_s *priv, int out)
{
  epwm_force(priv, out, priv->dcpol[out] == PWM_DCPOL_HIGH ?
                        EPWM_AQCSFRC_HIGH : EPWM_AQCSFRC_LOW);
}

/****************************************************************************
 * Name: epwm_output
 *
 * Description:
 *   Set output <out>'s polarity and duty against the current period.
 *
 ****************************************************************************/

static void epwm_output(FAR struct am263x_epwm_s *priv, int out,
                        FAR const struct pwm_chan_s *chan)
{
  bool activelow = chan->cpol == PWM_CPOL_LOW;
  uint16_t active = activelow ? EPWM_AQ_CLEAR : EPWM_AQ_SET;
  uint16_t inactive = activelow ? EPWM_AQ_SET : EPWM_AQ_CLEAR;
  uint32_t cmp;

  priv->dcpol[out] = chan->dcpol;

  epwm_putreg(priv, out == 0 ? AM263X_EPWM_AQCTLA : AM263X_EPWM_AQCTLB,
              (active << EPWM_AQCTL_ZRO_SHIFT) |
              (inactive << (out == 0 ? EPWM_AQCTL_CAU_SHIFT :
                                       EPWM_AQCTL_CBU_SHIFT)));

  cmp = ((uint64_t)chan->duty * (priv->prd + 1) + b16HALF) >> 16;
  if (cmp == 0)
    {
      epwm_force(priv, out, activelow ? EPWM_AQCSFRC_HIGH :
                                        EPWM_AQCSFRC_LOW);
    }
  else if (cmp > priv->prd)
    {
      epwm_force(priv, out, activelow ? EPWM_AQCSFRC_LOW :
                                        EPWM_AQCSFRC_HIGH);
    }
  else
    {
      putreg32(cmp << EPWM_CMP_SHIFT, priv->base +
               (out == 0 ? AM263X_EPWM_CMPA : AM263X_EPWM_CMPB));
      epwm_force(priv, out, EPWM_AQCSFRC_OFF);
    }
}

/****************************************************************************
 * Name: epwm_outputs
 *
 * Description:
 *   Set every channel listed in info.
 *
 ****************************************************************************/

static void epwm_outputs(FAR struct am263x_epwm_s *priv,
                         FAR const struct pwm_info_s *info)
{
  int8_t ch;
  int i;

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      ch = info->channels[i].channel;
      if (ch == -1)
        {
          break;
        }

      if (ch > 0)
        {
          epwm_output(priv, ch - 1, &info->channels[i]);
        }
    }
}

/****************************************************************************
 * Name: epwm_applysoc
 *
 * Description:
 *   Program SOCA or SOCB from priv->soc[] against the current period.
 *
 ****************************************************************************/

static void epwm_applysoc(FAR struct am263x_epwm_s *priv, int soc)
{
  FAR const struct am263x_epwm_soc_s *cfg = &priv->soc[soc];
  uint16_t selcmp;
  uint16_t clrbits;
  uint16_t setbits = 0;
  uint16_t sel;
  uint32_t cmp;
  int selshift;
  int psshift;

  if (soc == AM263X_EPWM_SOCA)
    {
      selcmp   = EPWM_ETSEL_SOCASELCMP;
      selshift = EPWM_ETSEL_SOCASEL_SHIFT;
      psshift  = EPWM_ETSOCPS_SOCAPRD2_SHIFT;
      clrbits  = EPWM_ETSEL_SOCAEN | EPWM_ETSEL_SOCASEL_MASK | selcmp;
    }
  else
    {
      selcmp   = EPWM_ETSEL_SOCBSELCMP;
      selshift = EPWM_ETSEL_SOCBSEL_SHIFT;
      psshift  = EPWM_ETSOCPS_SOCBPRD2_SHIFT;
      clrbits  = EPWM_ETSEL_SOCBEN | EPWM_ETSEL_SOCBSEL_MASK | selcmp;
    }

  switch (cfg->event)
    {
      case AM263X_EPWM_SOC_ZERO:
        sel = EPWM_SOCSEL_ZERO;
        break;

      case AM263X_EPWM_SOC_PERIOD:
        sel = EPWM_SOCSEL_PERIOD;
        break;

      case AM263X_EPWM_SOC_ZERO_PERIOD:
        sel = EPWM_SOCSEL_ZERO_PERIOD;
        break;

      case AM263X_EPWM_SOC_DUTY_A:
        sel = EPWM_SOCSEL_CMPA_UP;
        break;

      case AM263X_EPWM_SOC_DUTY_B:
        sel = EPWM_SOCSEL_CMPB_UP;
        break;

      case AM263X_EPWM_SOC_POSITION:

        /* CMPC up for SOCA, CMPD up for SOCB.  CMPC and CMPD load at
         * once (CMPCTL2, set in setup()).
         */

        cmp = ((uint64_t)cfg->position * (priv->prd + 1) + b16HALF) >> 16;
        if (cmp > priv->prd)
          {
            cmp = priv->prd;
          }

        epwm_putreg(priv, soc == AM263X_EPWM_SOCA ? AM263X_EPWM_CMPC :
                                                    AM263X_EPWM_CMPD, cmp);
        sel = soc == AM263X_EPWM_SOCA ? EPWM_SOCSEL_CMPA_UP :
                                        EPWM_SOCSEL_CMPB_UP;
        setbits = selcmp;
        break;

      default:
        sel = 0;
        break;
    }

  epwm_modifyreg(priv, AM263X_EPWM_ETSOCPS, 0xf << psshift,
                 cfg->prescale << psshift);

  if (sel != 0)
    {
      setbits |= (sel << selshift) |
                 (soc == AM263X_EPWM_SOCA ? EPWM_ETSEL_SOCAEN :
                                            EPWM_ETSEL_SOCBEN);
    }

  epwm_modifyreg(priv, AM263X_EPWM_ETSEL, clrbits, setbits);
}

/****************************************************************************
 * Name: epwm_setup
 *
 * Description:
 *   Clock the ePWM and put it in a known state: counter stopped, both
 *   outputs held low, SOCs off.
 *
 ****************************************************************************/

static int epwm_setup(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct am263x_epwm_s *priv = (FAR struct am263x_epwm_s *)dev;
  int out;

  am263x_css_ctrl_modify(AM263X_CSS_CTRL_EPWM_CLK_GATE(priv->pwm),
                         CSS_CTRL_CLK_GATED, CSS_CTRL_CLK_UNGATED);

  epwm_putreg(priv, AM263X_EPWM_TBCTL, EPWM_TBCTL_CTRMODE_STOP |
              EPWM_TBCTL_PRDLD | EPWM_TBCTL_FREE_RUN);
  epwm_putreg(priv, AM263X_EPWM_TBCTR, 0);

  /* Software forces act at once; hold both outputs low */

  epwm_putreg(priv, AM263X_EPWM_AQSFRC, EPWM_AQSFRC_RLDCSF_IMMEDIATE);
  for (out = 0; out < EPWM_NOUTPUTS; out++)
    {
      priv->dcpol[out] = PWM_DCPOL_LOW;
      epwm_idle(priv, out);
    }

  epwm_putreg(priv, AM263X_EPWM_AQCTLA, 0);
  epwm_putreg(priv, AM263X_EPWM_AQCTLB, 0);

  /* CMPA/CMPB shadowed, loaded at zero; CMPC/CMPD (SOC positions) at
   * once.
   */

  epwm_putreg(priv, AM263X_EPWM_CMPCTL, 0);
  epwm_putreg(priv, AM263X_EPWM_CMPCTL2,
              EPWM_CMPCTL_SHDWAMODE | EPWM_CMPCTL_SHDWBMODE);

  /* SOCs off, four-bit prescalers */

  epwm_putreg(priv, AM263X_EPWM_ETSEL, 0);
  epwm_putreg(priv, AM263X_EPWM_ETPS, EPWM_ETPS_SOCPSSEL);
  epwm_putreg(priv, AM263X_EPWM_ETSOCPS, 0);
  epwm_putreg(priv, AM263X_EPWM_ETCLR,
              EPWM_ET_INT | EPWM_ET_SOCA | EPWM_ET_SOCB);

  /* Let the time-base clock through */

  am263x_css_ctrl_modify(AM263X_CSS_CTRL_EPWM_CLKSYNC, 0, 1 << priv->pwm);

  priv->running   = false;
  priv->frequency = 0;
  return OK;
}

/****************************************************************************
 * Name: epwm_shutdown
 ****************************************************************************/

static int epwm_shutdown(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct am263x_epwm_s *priv = (FAR struct am263x_epwm_s *)dev;

  epwm_stop(dev);
  epwm_putreg(priv, AM263X_EPWM_ETSEL, 0);

  am263x_css_ctrl_modify(AM263X_CSS_CTRL_EPWM_CLKSYNC, 1 << priv->pwm, 0);
  am263x_css_ctrl_modify(AM263X_CSS_CTRL_EPWM_CLK_GATE(priv->pwm),
                         0, CSS_CTRL_CLK_GATED);
  return OK;
}

/****************************************************************************
 * Name: epwm_start
 ****************************************************************************/

static int epwm_start(FAR struct pwm_lowerhalf_s *dev,
                      FAR const struct pwm_info_s *info)
{
  FAR struct am263x_epwm_s *priv = (FAR struct am263x_epwm_s *)dev;
  irqstate_t flags;
  uint16_t tbdiv;
  uint16_t prd;
  int8_t ch;
  int ret;
  int i;

  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      ch = info->channels[i].channel;
      if (ch == -1)
        {
          break;
        }

      if (ch < 0 || ch > EPWM_NOUTPUTS || info->channels[i].duty > b16ONE)
        {
          pwmerr("ERROR: EPWM%d: bad channel %d or duty\n", priv->pwm, ch);
          return -EINVAL;
        }
    }

  flags = spin_lock_irqsave(&g_epwm_lock);

  if (!priv->running || info->frequency != priv->frequency)
    {
      ret = epwm_timebase(info->frequency, &tbdiv, &prd);
      if (ret < 0)
        {
          spin_unlock_irqrestore(&g_epwm_lock, flags);
          pwmerr("ERROR: EPWM%d: %" PRIu32 " Hz out of range\n",
                 priv->pwm, (uint32_t)info->frequency);
          return ret;
        }

      /* Stopped, TBPRD loads at once; running, at the next zero */

      epwm_modifyreg(priv, AM263X_EPWM_TBCTL,
                     EPWM_TBCTL_HSPCLKDIV_MASK | EPWM_TBCTL_CLKDIV_MASK,
                     tbdiv);
      epwm_putreg(priv, AM263X_EPWM_TBPRD, prd);

      priv->tbdiv     = tbdiv;
      priv->prd       = prd;
      priv->frequency = info->frequency;
    }

  if (!priv->running)
    {
      epwm_putreg(priv, AM263X_EPWM_TBCTR, 0);
      epwm_putreg(priv, AM263X_EPWM_CMPCTL,
                  EPWM_CMPCTL_SHDWAMODE | EPWM_CMPCTL_SHDWBMODE);
    }

  epwm_outputs(priv, info);
  epwm_applysoc(priv, AM263X_EPWM_SOCA);
  epwm_applysoc(priv, AM263X_EPWM_SOCB);

  if (!priv->running)
    {
      /* From here on TBPRD, CMPA and CMPB load at counter zero.  The
       * immediate writes above did not reach the shadow registers, which
       * would load their old contents at the first zero: write them too.
       */

      epwm_putreg(priv, AM263X_EPWM_CMPCTL, 0);
      epwm_modifyreg(priv, AM263X_EPWM_TBCTL, EPWM_TBCTL_PRDLD, 0);
      epwm_putreg(priv, AM263X_EPWM_TBPRD, priv->prd);
      epwm_outputs(priv, info);

      epwm_modifyreg(priv, AM263X_EPWM_TBCTL, EPWM_TBCTL_CTRMODE_MASK,
                     EPWM_TBCTL_CTRMODE_UP);
      priv->running = true;
    }

  spin_unlock_irqrestore(&g_epwm_lock, flags);

  pwminfo("EPWM%d: %" PRIu32 " Hz, TBPRD %u, TBCTL %04x\n", priv->pwm,
          priv->frequency, priv->prd,
          epwm_getreg(priv, AM263X_EPWM_TBCTL));
  return OK;
}

/****************************************************************************
 * Name: epwm_stop
 *
 * Description:
 *   Hold both outputs at their idle level and freeze the counter.  The
 *   SOCs stop with it.
 *
 ****************************************************************************/

static int epwm_stop(FAR struct pwm_lowerhalf_s *dev)
{
  FAR struct am263x_epwm_s *priv = (FAR struct am263x_epwm_s *)dev;
  irqstate_t flags;
  int out;

  flags = spin_lock_irqsave(&g_epwm_lock);

  for (out = 0; out < EPWM_NOUTPUTS; out++)
    {
      epwm_idle(priv, out);
    }

  epwm_modifyreg(priv, AM263X_EPWM_TBCTL, EPWM_TBCTL_CTRMODE_MASK,
                 EPWM_TBCTL_CTRMODE_STOP | EPWM_TBCTL_PRDLD);
  priv->running = false;

  spin_unlock_irqrestore(&g_epwm_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: epwm_ioctl
 ****************************************************************************/

static int epwm_ioctl(FAR struct pwm_lowerhalf_s *dev, int cmd,
                      unsigned long arg)
{
  UNUSED(dev);
  UNUSED(arg);
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_pwminitialize
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *am263x_pwminitialize(int pwm)
{
  FAR struct am263x_epwm_s *priv;

  if (pwm < 0 || pwm >= AM263X_EPWM_NINSTANCES)
    {
      return NULL;
    }

  priv = g_epwm[pwm];
  if (priv == NULL)
    {
      priv = kmm_zalloc(sizeof(struct am263x_epwm_s));
      if (priv == NULL)
        {
          return NULL;
        }

      priv->ops  = &g_epwmops;
      priv->base = AM263X_EPWM_BASE(pwm);
      priv->pwm  = pwm;
      priv->soc[AM263X_EPWM_SOCA].prescale = 1;
      priv->soc[AM263X_EPWM_SOCB].prescale = 1;
      g_epwm[pwm] = priv;
    }

  return (FAR struct pwm_lowerhalf_s *)priv;
}

/****************************************************************************
 * Name: am263x_pwm_socconfig
 ****************************************************************************/

int am263x_pwm_socconfig(FAR struct pwm_lowerhalf_s *dev, int soc,
                         FAR const struct am263x_epwm_soc_s *config)
{
  FAR struct am263x_epwm_s *priv = (FAR struct am263x_epwm_s *)dev;
  irqstate_t flags;

  if (priv == NULL || config == NULL ||
      (soc != AM263X_EPWM_SOCA && soc != AM263X_EPWM_SOCB) ||
      config->event > AM263X_EPWM_SOC_POSITION ||
      config->prescale < 1 || config->prescale > 15 ||
      config->position > b16ONE)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&g_epwm_lock);

  priv->soc[soc] = *config;
  if (priv->running)
    {
      epwm_applysoc(priv, soc);
    }

  spin_unlock_irqrestore(&g_epwm_lock, flags);
  return OK;
}

#endif /* CONFIG_AM263X_EPWM */
