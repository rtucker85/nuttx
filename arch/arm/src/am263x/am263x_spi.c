/****************************************************************************
 * arch/arm/src/am263x/am263x_spi.c
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

/* MCSPI controller driver, polled, one word at a time.  Each MCSPI has
 * four channels, one per CS pin; the driver keeps one channel configured
 * and enabled and moves to another when the board selects a different
 * hardware CS.  Words are 4 to 32 bits.
 *
 * In single-channel mode a channel only shifts while CHxCONF.FORCE is set.
 * With a hardware CS (am263x_spi_hwcs()) FORCE is the CS.  Otherwise, for
 * devices on GPIO chip selects, the module runs in 3-pin mode, where the
 * CS pin is not driven, and FORCE is set around each word.
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
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "am263x_clockconfig.h"
#include "am263x_spi.h"
#include "hardware/am263x_mcspi.h"

#ifdef CONFIG_AM263X_MCSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* With CHCONF.CLKG set the clock divides by any of 1 to 4096 */

#define MCSPI_MAX_DIVIDER     4096

/* Polls of CHxSTAT before a transfer counts as stuck: far longer than a
 * 32-bit word at the slowest clock (11.7 kHz, about 3 ms).
 */

#define MCSPI_POLL_TIMEOUT    1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_spidev_s
{
  struct spi_dev_s spidev;    /* Externally visible part; must be first */
  uintptr_t        base;      /* MCSPI register base */
  mutex_t          lock;      /* Held by the bus user (SPI_LOCK) */
  uint32_t         frequency; /* Requested SCLK */
  uint32_t         actual;    /* SCLK the divider gives */
  uint32_t         chconf;    /* CHxCONF for the current settings */
  uint32_t         chctrl;    /* CHxCTRL, without EN */
  uint8_t          mode;      /* SPIDEV_MODEn */
  uint8_t          nbits;     /* Word length */
  uint8_t          channel;   /* Channel (and CS) in use */
  bool             enabled;   /* Channel enabled */
  bool             hwcs;      /* Board holds a hardware CS */
  bool             initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev,
                            enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static int      spi_exchange(FAR struct spi_dev_s *dev,
                             FAR const void *txbuffer, FAR void *rxbuffer,
                             size_t nwords);
#else
static void     spi_sndblock(FAR struct spi_dev_s *dev,
                             FAR const void *txbuffer, size_t nwords);
static void     spi_recvblock(FAR struct spi_dev_s *dev,
                              FAR void *rxbuffer, size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SPI_EXCHANGE
#  define SPI_XFER_OPS .exchange = spi_exchange,
#else
#  define SPI_XFER_OPS .sndblock = spi_sndblock, .recvblock = spi_recvblock,
#endif

#ifdef CONFIG_SPI_CMDDATA
#  define SPI_CMDDATA_OPS(n) .cmddata = am263x_spi##n##cmddata,
#else
#  define SPI_CMDDATA_OPS(n)
#endif

#define SPI_OPS(n) \
  { \
    .lock         = spi_lock, \
    .select       = am263x_spi##n##select, \
    .setfrequency = spi_setfrequency, \
    .setmode      = spi_setmode, \
    .setbits      = spi_setbits, \
    .status       = am263x_spi##n##status, \
    SPI_CMDDATA_OPS(n) \
    .send         = spi_send, \
    SPI_XFER_OPS \
  }

#define SPI_DEV(n) \
  { \
    .spidev = \
      { \
        &g_spi##n##ops \
      }, \
    .base   = AM263X_MCSPI_BASE(n), \
    .lock   = NXMUTEX_INITIALIZER, \
  }

#ifdef CONFIG_AM263X_MCSPI0
static const struct spi_ops_s g_spi0ops = SPI_OPS(0);
static struct am263x_spidev_s g_spi0dev = SPI_DEV(0);
#endif

#ifdef CONFIG_AM263X_MCSPI1
static const struct spi_ops_s g_spi1ops = SPI_OPS(1);
static struct am263x_spidev_s g_spi1dev = SPI_DEV(1);
#endif

#ifdef CONFIG_AM263X_MCSPI2
static const struct spi_ops_s g_spi2ops = SPI_OPS(2);
static struct am263x_spidev_s g_spi2dev = SPI_DEV(2);
#endif

#ifdef CONFIG_AM263X_MCSPI3
static const struct spi_ops_s g_spi3ops = SPI_OPS(3);
static struct am263x_spidev_s g_spi3dev = SPI_DEV(3);
#endif

#ifdef CONFIG_AM263X_MCSPI4
static const struct spi_ops_s g_spi4ops = SPI_OPS(4);
static struct am263x_spidev_s g_spi4dev = SPI_DEV(4);
#endif

static FAR struct am263x_spidev_s * const g_spidevs[] =
{
#ifdef CONFIG_AM263X_MCSPI0
  &g_spi0dev,
#else
  NULL,
#endif
#ifdef CONFIG_AM263X_MCSPI1
  &g_spi1dev,
#else
  NULL,
#endif
#ifdef CONFIG_AM263X_MCSPI2
  &g_spi2dev,
#else
  NULL,
#endif
#ifdef CONFIG_AM263X_MCSPI3
  &g_spi3dev,
#else
  NULL,
#endif
#ifdef CONFIG_AM263X_MCSPI4
  &g_spi4dev,
#else
  NULL,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t spi_getreg(FAR struct am263x_spidev_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void spi_putreg(FAR struct am263x_spidev_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: spi_waitstat
 *
 * Description:
 *   Wait for a CHxSTAT bit.  Returns false on a timeout.
 *
 ****************************************************************************/

static bool spi_waitstat(FAR struct am263x_spidev_s *priv, uint32_t bit)
{
  uint32_t count;

  for (count = 0; count < MCSPI_POLL_TIMEOUT; count++)
    {
      if ((spi_getreg(priv, AM263X_MCSPI_CHSTAT(priv->channel)) & bit) != 0)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: spi_enable
 ****************************************************************************/

static void spi_enable(FAR struct am263x_spidev_s *priv, bool enable)
{
  spi_putreg(priv, AM263X_MCSPI_CHCTRL(priv->channel),
             priv->chctrl | (enable ? MCSPI_CHCTRL_EN : 0));
  priv->enabled = enable;
}

/****************************************************************************
 * Name: spi_writeconf
 *
 * Description:
 *   Write the current settings to the channel, with the channel disabled
 *   while CHxCONF changes.  FORCE (the CS level) is kept as it is.
 *
 ****************************************************************************/

static void spi_writeconf(FAR struct am263x_spidev_s *priv)
{
  uint32_t offset = AM263X_MCSPI_CHCONF(priv->channel);
  uint32_t force = spi_getreg(priv, offset) & MCSPI_CHCONF_FORCE;
  bool enabled = priv->enabled;

  if (enabled)
    {
      spi_enable(priv, false);
    }

  spi_putreg(priv, offset, priv->chconf | force);

  if (enabled)
    {
      spi_enable(priv, true);
    }
  else
    {
      spi_putreg(priv, AM263X_MCSPI_CHCTRL(priv->channel), priv->chctrl);
    }
}

/****************************************************************************
 * Name: spi_buildconf
 *
 * Description:
 *   Work out CHxCONF and CHxCTRL from the mode, word length and frequency.
 *
 ****************************************************************************/

static void spi_buildconf(FAR struct am263x_spidev_s *priv)
{
  uint32_t div;
  uint32_t chconf;

  /* Divide by 1..4096: low 4 bits of (div - 1) in CLKD, the rest in
   * EXTCLK.
   */

  div = (AM263X_MCSPI_FREQUENCY + priv->frequency - 1) / priv->frequency;
  if (div < 1)
    {
      div = 1;
    }
  else if (div > MCSPI_MAX_DIVIDER)
    {
      div = MCSPI_MAX_DIVIDER;
    }

  priv->actual = AM263X_MCSPI_FREQUENCY / div;
  priv->chctrl = ((div - 1) >> 4) << MCSPI_CHCTRL_EXTCLK_SHIFT;

  chconf = MCSPI_CHCONF_CLKG | MCSPI_CHCONF_EPOL |
           (((div - 1) & 15) << MCSPI_CHCONF_CLKD_SHIFT) |
           ((uint32_t)(priv->nbits - 1) << MCSPI_CHCONF_WL_SHIFT);

  /* Transmit on D0.  Receive on D1, or on D0 for the loopback test. */

  chconf |= MCSPI_CHCONF_DPE1;
#ifndef CONFIG_AM263X_MCSPI_LOOPBACK
  chconf |= MCSPI_CHCONF_IS;
#endif

  if (priv->mode == SPIDEV_MODE1 || priv->mode == SPIDEV_MODE3)
    {
      chconf |= MCSPI_CHCONF_PHA;
    }

  if (priv->mode == SPIDEV_MODE2 || priv->mode == SPIDEV_MODE3)
    {
      chconf |= MCSPI_CHCONF_POL;
    }

  priv->chconf = chconf;
}

/****************************************************************************
 * Name: spi_transfer
 *
 * Description:
 *   Send one word and return the word received with it.
 *
 ****************************************************************************/

static int spi_transfer(FAR struct am263x_spidev_s *priv, uint32_t txwd,
                        FAR uint32_t *rxwd)
{
  uint32_t chconf = AM263X_MCSPI_CHCONF(priv->channel);
  bool force = !priv->hwcs;
  int ret = OK;

  if (!priv->enabled)
    {
      spi_enable(priv, true);
    }

  /* Without a hardware CS, FORCE only starts the shifter (3-pin mode) */

  if (force)
    {
      spi_putreg(priv, chconf,
                 spi_getreg(priv, chconf) | MCSPI_CHCONF_FORCE);
    }

  if (!spi_waitstat(priv, MCSPI_CHSTAT_TXS))
    {
      spierr("ERROR: MCSPI TX timeout, CHSTAT %08" PRIx32 "\n",
             spi_getreg(priv, AM263X_MCSPI_CHSTAT(priv->channel)));
      ret = -ETIMEDOUT;
      goto out;
    }

  spi_putreg(priv, AM263X_MCSPI_TX(priv->channel), txwd);

  if (!spi_waitstat(priv, MCSPI_CHSTAT_RXS))
    {
      spierr("ERROR: MCSPI RX timeout, CHSTAT %08" PRIx32 "\n",
             spi_getreg(priv, AM263X_MCSPI_CHSTAT(priv->channel)));
      ret = -ETIMEDOUT;
      goto out;
    }

  *rxwd = spi_getreg(priv, AM263X_MCSPI_RX(priv->channel));

out:
  if (force)
    {
      spi_putreg(priv, chconf,
                 spi_getreg(priv, chconf) & ~MCSPI_CHCONF_FORCE);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_lock
 ****************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct am263x_spidev_s *priv = (FAR struct am263x_spidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: spi_setfrequency
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency)
{
  FAR struct am263x_spidev_s *priv = (FAR struct am263x_spidev_s *)dev;

  if (frequency == 0)
    {
      frequency = 1;
    }

  if (frequency != priv->frequency)
    {
      priv->frequency = frequency;
      spi_buildconf(priv);
      spi_writeconf(priv);
      spiinfo("MCSPI frequency %" PRIu32 " actual %" PRIu32 "\n",
              frequency, priv->actual);
    }

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setmode
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct am263x_spidev_s *priv = (FAR struct am263x_spidev_s *)dev;

  if (mode != priv->mode)
    {
      priv->mode = mode;
      spi_buildconf(priv);
      spi_writeconf(priv);
    }
}

/****************************************************************************
 * Name: spi_setbits
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct am263x_spidev_s *priv = (FAR struct am263x_spidev_s *)dev;

  if (nbits < 4 || nbits > 32)
    {
      spierr("ERROR: %d-bit words not supported\n", nbits);
      return;
    }

  if (nbits != priv->nbits)
    {
      priv->nbits = nbits;
      spi_buildconf(priv);
      spi_writeconf(priv);
    }
}

/****************************************************************************
 * Name: spi_send
 ****************************************************************************/

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct am263x_spidev_s *priv = (FAR struct am263x_spidev_s *)dev;
  uint32_t rxwd = 0;

  spi_transfer(priv, wd, &rxwd);
  return rxwd;
}

/****************************************************************************
 * Name: spi_exchange
 *
 * Description:
 *   Exchange nwords words.  Buffers hold uint8_t words up to 8 bits,
 *   uint16_t up to 16 and uint32_t above.  A NULL txbuffer sends all ones;
 *   a NULL rxbuffer discards what comes in.
 *
 ****************************************************************************/

static int spi_xfer(FAR struct am263x_spidev_s *priv,
                    FAR const void *txbuffer, FAR void *rxbuffer,
                    size_t nwords)
{
  uint32_t txwd;
  uint32_t rxwd;
  size_t i;
  int ret;

  for (i = 0; i < nwords; i++)
    {
      txwd = 0xffffffff;
      if (txbuffer != NULL)
        {
          if (priv->nbits <= 8)
            {
              txwd = ((FAR const uint8_t *)txbuffer)[i];
            }
          else if (priv->nbits <= 16)
            {
              txwd = ((FAR const uint16_t *)txbuffer)[i];
            }
          else
            {
              txwd = ((FAR const uint32_t *)txbuffer)[i];
            }
        }

      ret = spi_transfer(priv, txwd, &rxwd);
      if (ret < 0)
        {
          return ret;
        }

      if (rxbuffer != NULL)
        {
          if (priv->nbits <= 8)
            {
              ((FAR uint8_t *)rxbuffer)[i] = (uint8_t)rxwd;
            }
          else if (priv->nbits <= 16)
            {
              ((FAR uint16_t *)rxbuffer)[i] = (uint16_t)rxwd;
            }
          else
            {
              ((FAR uint32_t *)rxbuffer)[i] = rxwd;
            }
        }
    }

  return OK;
}

#ifdef CONFIG_SPI_EXCHANGE
static int spi_exchange(FAR struct spi_dev_s *dev,
                        FAR const void *txbuffer, FAR void *rxbuffer,
                        size_t nwords)
{
  return spi_xfer((FAR struct am263x_spidev_s *)dev, txbuffer, rxbuffer,
                  nwords);
}
#else
static void spi_sndblock(FAR struct spi_dev_s *dev,
                         FAR const void *txbuffer, size_t nwords)
{
  spi_xfer((FAR struct am263x_spidev_s *)dev, txbuffer, NULL, nwords);
}

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *rxbuffer,
                          size_t nwords)
{
  spi_xfer((FAR struct am263x_spidev_s *)dev, NULL, rxbuffer, nwords);
}
#endif

/****************************************************************************
 * Name: spi_hwinit
 *
 * Description:
 *   Reset the MCSPI and set it up as a controller, one channel at a time,
 *   1 MHz, mode 0, 8-bit words, on channel 0.
 *
 ****************************************************************************/

static int spi_hwinit(FAR struct am263x_spidev_s *priv)
{
  uint32_t count;

  /* Keep the module out of idle while the driver polls it */

  spi_putreg(priv, AM263X_MCSPI_HL_SYSCONFIG,
             MCSPI_HL_SYSCONFIG_IDLEMODE_NO);

  spi_putreg(priv, AM263X_MCSPI_SYSCONFIG, MCSPI_SYSCONFIG_SOFTRESET);
  for (count = 0;
       (spi_getreg(priv, AM263X_MCSPI_SYSSTATUS) &
        MCSPI_SYSSTATUS_RESETDONE) == 0;
       count++)
    {
      if (count >= MCSPI_POLL_TIMEOUT)
        {
          spierr("ERROR: MCSPI at %08" PRIxPTR " stuck in reset\n",
                 priv->base);
          return -EIO;
        }
    }

  spi_putreg(priv, AM263X_MCSPI_SYSCONFIG,
             MCSPI_SYSCONFIG_CLKACT_BOTH | MCSPI_SYSCONFIG_SIDLEMODE_NO);

  /* Controller mode (MS = 0), one channel at a time, 3-pin until the
   * board selects a hardware CS.
   */

  spi_putreg(priv, AM263X_MCSPI_MODULCTRL,
             MCSPI_MODULCTRL_SINGLE | MCSPI_MODULCTRL_PIN34);

  priv->frequency = 1000000;
  priv->mode      = SPIDEV_MODE0;
  priv->nbits     = 8;
  priv->channel   = 0;
  priv->enabled   = false;

  spi_buildconf(priv);
  spi_writeconf(priv);

  spiinfo("MCSPI at %08" PRIxPTR " revision %08" PRIx32 "\n",
          priv->base, spi_getreg(priv, AM263X_MCSPI_REVISION));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_spibus_initialize
 ****************************************************************************/

FAR struct spi_dev_s *am263x_spibus_initialize(int bus)
{
  FAR struct am263x_spidev_s *priv;
  irqstate_t flags;

  if (bus < 0 || bus >= AM263X_MCSPI_NINSTANCES || g_spidevs[bus] == NULL)
    {
      spierr("ERROR: MCSPI%d not enabled\n", bus);
      return NULL;
    }

  priv = g_spidevs[bus];

  flags = enter_critical_section();
  if (!priv->initialized)
    {
      if (spi_hwinit(priv) < 0)
        {
          leave_critical_section(flags);
          return NULL;
        }

      priv->initialized = true;
    }

  leave_critical_section(flags);
  return &priv->spidev;
}

/****************************************************************************
 * Name: am263x_spi_hwcs
 ****************************************************************************/

void am263x_spi_hwcs(FAR struct spi_dev_s *dev, int cs, bool selected)
{
  FAR struct am263x_spidev_s *priv = (FAR struct am263x_spidev_s *)dev;
  uint32_t offset;
  uint32_t regval;

  DEBUGASSERT(cs >= 0 && cs < AM263X_MCSPI_NCHANNELS);

  if (cs != priv->channel)
    {
      /* Move the settings to the new channel */

      if (priv->enabled)
        {
          spi_enable(priv, false);
        }

      priv->channel = cs;
      spi_writeconf(priv);
    }

  offset = AM263X_MCSPI_CHCONF(priv->channel);
  regval = spi_getreg(priv, offset);

  if (selected)
    {
      /* 4-pin mode: FORCE drives CS<cs> */

      spi_putreg(priv, AM263X_MCSPI_MODULCTRL, MCSPI_MODULCTRL_SINGLE);
      spi_putreg(priv, offset, regval | MCSPI_CHCONF_FORCE);
      priv->hwcs = true;
      if (!priv->enabled)
        {
          spi_enable(priv, true);
        }
    }
  else
    {
      spi_putreg(priv, offset, regval & ~MCSPI_CHCONF_FORCE);
      spi_putreg(priv, AM263X_MCSPI_MODULCTRL,
                 MCSPI_MODULCTRL_SINGLE | MCSPI_MODULCTRL_PIN34);
      priv->hwcs = false;
    }
}

#endif /* CONFIG_AM263X_MCSPI */
