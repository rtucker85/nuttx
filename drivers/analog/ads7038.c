/****************************************************************************
 * drivers/analog/ads7038.c
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

/* TI ADS7038 / ADS7038-Q1: 8-channel, 12-bit, 1 MSPS SAR ADC on SPI
 * (datasheet SBAS981A).
 *
 * The device runs in on-the-fly mode: the first five bits of each frame,
 * 1 and a 4-bit channel ID, select the channel sampled at the end of that
 * frame, and its result comes out in the next frame.  Converting n
 * channels takes n + 1 frames.  Each result carries its channel ID
 * (DATA_CFG.APPEND_STATUS), which is checked.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ads7038.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_ADC_ADS7038

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Opcodes */

#define ADS7038_OP_RDREG        0x10
#define ADS7038_OP_WRREG        0x08

/* Registers */

#define ADS7038_SYSTEM_STATUS   0x00
#define ADS7038_GENERAL_CFG     0x01
#define ADS7038_DATA_CFG        0x02
#define ADS7038_OSR_CFG         0x03
#define ADS7038_OPMODE_CFG      0x04
#define ADS7038_PIN_CFG         0x05
#define ADS7038_SEQUENCE_CFG    0x10

/* GENERAL_CFG */

#define GENERAL_CFG_RST         (1 << 0)
#define GENERAL_CFG_CAL         (1 << 1)

/* DATA_CFG: APPEND_STATUS = 01b, the channel ID after the result.
 * CPOL_CPHA stays 00b: SPI mode 0.
 */

#define DATA_CFG_APPEND_CHID    (1 << 4)

/* SEQUENCE_CFG.SEQ_MODE = 10b: on-the-fly */

#define SEQUENCE_CFG_ONTHEFLY   0x02

/* On-the-fly channel select: 1 then the channel ID, MSB first */

#define ADS7038_OTF_SELECT(ch)  (0x80 | ((ch) << 3))

#define ADS7038_NCHANNELS       8
#define ADS7038_SPI_MODE        SPIDEV_MODE0
#define ADS7038_FRAME_SIZE      3

/* Reset and calibration finish within 5 ms (tRST) */

#define ADS7038_RESET_MS        5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ads7038_dev_s
{
  FAR const struct adc_callback_s *cb;
  FAR struct spi_dev_s            *spi;
  uint32_t                         devid;
  uint8_t                          chanmask;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  ads7038_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void ads7038_reset(FAR struct adc_dev_s *dev);
static int  ads7038_setup(FAR struct adc_dev_s *dev);
static void ads7038_shutdown(FAR struct adc_dev_s *dev);
static void ads7038_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  ads7038_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_ads7038_ops =
{
  .ao_bind     = ads7038_bind,
  .ao_reset    = ads7038_reset,
  .ao_setup    = ads7038_setup,
  .ao_shutdown = ads7038_shutdown,
  .ao_rxint    = ads7038_rxint,
  .ao_ioctl    = ads7038_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7038_frame
 *
 * Description:
 *   One 24-bit frame: send tx, return what came back.  The bus must be
 *   locked and configured.
 *
 ****************************************************************************/

static void ads7038_frame(FAR struct ads7038_dev_s *priv,
                          FAR const uint8_t *tx, FAR uint8_t *rx)
{
  SPI_SELECT(priv->spi, priv->devid, true);
  SPI_EXCHANGE(priv->spi, tx, rx, ADS7038_FRAME_SIZE);
  SPI_SELECT(priv->spi, priv->devid, false);

  /* The conversion runs while CS is high; keep the cycle above 1 us */

  up_udelay(1);
}

/****************************************************************************
 * Name: ads7038_lock
 ****************************************************************************/

static void ads7038_lock(FAR struct ads7038_dev_s *priv, bool lock)
{
  SPI_LOCK(priv->spi, lock);
  if (lock)
    {
      SPI_SETMODE(priv->spi, ADS7038_SPI_MODE);
      SPI_SETBITS(priv->spi, 8);
      SPI_SETFREQUENCY(priv->spi, CONFIG_ADS7038_FREQUENCY);
    }
}

/****************************************************************************
 * Name: ads7038_putreg / ads7038_getreg
 ****************************************************************************/

static void ads7038_putreg(FAR struct ads7038_dev_s *priv, uint8_t reg,
                           uint8_t val)
{
  uint8_t tx[ADS7038_FRAME_SIZE];
  uint8_t rx[ADS7038_FRAME_SIZE];

  tx[0] = ADS7038_OP_WRREG;
  tx[1] = reg;
  tx[2] = val;

  ads7038_lock(priv, true);
  ads7038_frame(priv, tx, rx);
  ads7038_lock(priv, false);
}

static uint8_t ads7038_getreg(FAR struct ads7038_dev_s *priv, uint8_t reg)
{
  uint8_t tx[ADS7038_FRAME_SIZE];
  uint8_t rx[ADS7038_FRAME_SIZE];

  /* The register comes out in the first byte of the next frame */

  tx[0] = ADS7038_OP_RDREG;
  tx[1] = reg;
  tx[2] = 0;

  ads7038_lock(priv, true);
  ads7038_frame(priv, tx, rx);
  memset(tx, 0, sizeof(tx));
  ads7038_frame(priv, tx, rx);
  ads7038_lock(priv, false);

  return rx[0];
}

/****************************************************************************
 * Name: ads7038_configure
 *
 * Description:
 *   Reset, check that the device answers, calibrate the offset, and select
 *   on-the-fly mode with all inputs analog.
 *
 ****************************************************************************/

static int ads7038_configure(FAR struct ads7038_dev_s *priv)
{
  uint8_t val;
  int i;

  ads7038_putreg(priv, ADS7038_GENERAL_CFG, GENERAL_CFG_RST);
  up_mdelay(ADS7038_RESET_MS);

  /* There is no ID register: a written register must read back */

  ads7038_putreg(priv, ADS7038_DATA_CFG, DATA_CFG_APPEND_CHID);
  val = ads7038_getreg(priv, ADS7038_DATA_CFG);
  if (val != DATA_CFG_APPEND_CHID)
    {
      aerr("ERROR: ADS7038 not responding (DATA_CFG %02x)\n", val);
      return -ENODEV;
    }

  ads7038_putreg(priv, ADS7038_PIN_CFG, 0);
  ads7038_putreg(priv, ADS7038_OSR_CFG, 0);
  ads7038_putreg(priv, ADS7038_OPMODE_CFG, 0);

  ads7038_putreg(priv, ADS7038_GENERAL_CFG, GENERAL_CFG_CAL);
  for (i = 0; (ads7038_getreg(priv, ADS7038_GENERAL_CFG) &
               GENERAL_CFG_CAL) != 0; i++)
    {
      if (i >= ADS7038_RESET_MS)
        {
          aerr("ERROR: ADS7038 offset calibration did not finish\n");
          return -EIO;
        }

      up_mdelay(1);
    }

  ads7038_putreg(priv, ADS7038_SEQUENCE_CFG, SEQUENCE_CFG_ONTHEFLY);
  return OK;
}

/****************************************************************************
 * Name: ads7038_convert
 *
 * Description:
 *   Convert each channel in chanmask once and pass the results up.
 *
 ****************************************************************************/

static int ads7038_convert(FAR struct adc_dev_s *dev)
{
  FAR struct ads7038_dev_s *priv = dev->ad_priv;
  uint8_t tx[ADS7038_FRAME_SIZE];
  uint8_t rx[ADS7038_FRAME_SIZE];
  uint8_t chans[ADS7038_NCHANNELS];
  int32_t data[ADS7038_NCHANNELS];
  int nchans = 0;
  int ret = OK;
  int prev = -1;
  int ch;
  int i;

  ads7038_lock(priv, true);

  /* Frame ch selects ch and returns the previous channel's result; one
   * more frame (a no-op, channel unchanged) returns the last.
   */

  for (ch = 0; ch <= ADS7038_NCHANNELS; ch++)
    {
      if (ch < ADS7038_NCHANNELS && (priv->chanmask & (1 << ch)) == 0)
        {
          continue;
        }

      memset(tx, 0, sizeof(tx));
      if (ch < ADS7038_NCHANNELS)
        {
          tx[0] = ADS7038_OTF_SELECT(ch);
        }

      ads7038_frame(priv, tx, rx);

      if (prev >= 0)
        {
          /* 12-bit result, then the channel ID */

          if ((rx[1] & 0x0f) != prev)
            {
              aerr("ERROR: AIN%d result tagged AIN%d\n", prev, rx[1] & 0x0f);
              ret = -EIO;
              break;
            }

          chans[nchans] = prev;
          data[nchans]  = (rx[0] << 4) | (rx[1] >> 4);
          nchans++;
        }

      prev = ch;
    }

  ads7038_lock(priv, false);

  if (ret == OK && priv->cb != NULL)
    {
      for (i = 0; i < nchans; i++)
        {
          priv->cb->au_receive(dev, chans[i], data[i]);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ads7038_bind
 ****************************************************************************/

static int ads7038_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ads7038_dev_s *priv = dev->ad_priv;

  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: ads7038_reset
 ****************************************************************************/

static void ads7038_reset(FAR struct adc_dev_s *dev)
{
  ads7038_configure(dev->ad_priv);
}

/****************************************************************************
 * Name: ads7038_setup
 ****************************************************************************/

static int ads7038_setup(FAR struct adc_dev_s *dev)
{
  return ads7038_configure(dev->ad_priv);
}

/****************************************************************************
 * Name: ads7038_shutdown
 ****************************************************************************/

static void ads7038_shutdown(FAR struct adc_dev_s *dev)
{
}

/****************************************************************************
 * Name: ads7038_rxint
 ****************************************************************************/

static void ads7038_rxint(FAR struct adc_dev_s *dev, bool enable)
{
}

/****************************************************************************
 * Name: ads7038_ioctl
 ****************************************************************************/

static int ads7038_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ads7038_dev_s *priv = dev->ad_priv;
  int nchans = 0;
  int ch;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        return ads7038_convert(dev);

      case ANIOC_GET_NCHANNELS:
        for (ch = 0; ch < ADS7038_NCHANNELS; ch++)
          {
            nchans += (priv->chanmask >> ch) & 1;
          }

        return nchans;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ads7038_initialize
 ****************************************************************************/

FAR struct adc_dev_s *ads7038_initialize(FAR struct spi_dev_s *spi,
                                         uint32_t devid, uint8_t chanmask)
{
  FAR struct ads7038_dev_s *priv;
  FAR struct adc_dev_s *dev;

  DEBUGASSERT(spi != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  dev  = kmm_zalloc(sizeof(*dev));
  if (priv == NULL || dev == NULL)
    {
      kmm_free(priv);
      kmm_free(dev);
      return NULL;
    }

  priv->spi      = spi;
  priv->devid    = devid;
  priv->chanmask = chanmask;
  dev->ad_ops    = &g_ads7038_ops;
  dev->ad_priv   = priv;

  if (ads7038_configure(priv) < 0)
    {
      kmm_free(priv);
      kmm_free(dev);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_ADC_ADS7038 */
