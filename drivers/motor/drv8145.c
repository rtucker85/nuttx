/****************************************************************************
 * drivers/motor/drv8145.c
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

/* TI DRV8145-Q1 half-bridge drivers in an SPI daisy chain (SLVSG22B).
 *
 * Every transfer is one daisy-chain frame of 2 + 2n bytes for n devices:
 *
 *   MOSI  HDR1 HDR2 A[n-1] .. A[0] D[n-1] .. D[0]
 *   MISO  S[n-1] .. S[0] HDR1 HDR2 R[n-1] .. R[0]
 *
 * Device 0 is the first in the chain (its SDI is MOSI).  A is a command
 * byte (bit 6 set for a read, address in 5:0), D its data, S a status
 * byte (bits 7:6 always 11) and R the register's contents: the value read,
 * or for a write the value before it.  HDR1 carries the device count and
 * HDR2 a global CLR_FLT.  The headers come back unchanged, so each frame
 * is checked by its header echo (with a sequence number in the spare bits
 * of HDR2) and the fixed bits of the status bytes.  A device with nothing
 * to do in a frame reads DEVICE_ID.
 *
 * SPI mode 1: SCLK idles low, SDO changes on the rising edge and SDI is
 * sampled on the falling edge.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/motor/drv8145.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_MOTOR_DRV8145

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DRV8145_SPI_MODE        SPIDEV_MODE1

/* Frame bytes */

#define DRV8145_HDR1(n)         (0x80 | ((n) & 0x3f))
#define DRV8145_HDR2            0x80
#define DRV8145_HDR2_CLR_FLT    0x20
#define DRV8145_HDR2_SEQ_MASK   0x1f
#define DRV8145_CMD_READ        0x40
#define DRV8145_CMD_ADDR_MASK   0x3f
#define DRV8145_NOP             (DRV8145_CMD_READ | DRV8145_DEVICE_ID)

#define DRV8145_FRAME_SIZE(n)   (2 + 2 * (n))

/* COMMAND as the driver keeps it: SPI_IN unlocked, CONFIG locked or
 * unlocked.
 */

#define DRV8145_CMD_RUN         (DRV8145_CMD_SPI_IN_UNLOCK | \
                                 DRV8145_CMD_REG_LOCK)
#define DRV8145_CMD_CONFIG      (DRV8145_CMD_SPI_IN_UNLOCK | \
                                 DRV8145_CMD_REG_UNLOCK)
#define DRV8145_CMD_LOCK_MASK   (DRV8145_CMD_SPI_IN_MASK | \
                                 DRV8145_CMD_REG_MASK)

/* Bits of CONFIG1-4 that are checked after a write.  N/A bits read 0;
 * the SPARE bits are left out too, as CONFIG3's read back 10b whatever
 * was written (seen on a DRV8145S, despite SLVSG22B calling them R/W).
 */

#define DRV8145_CFG1_RW         0xff
#define DRV8145_CFG2_RW         0x67
#define DRV8145_CFG3_RW         0xdc
#define DRV8145_CFG4_RW         0xdd

/* Timing (SLVSG22B 6.5): tREADY after wake-up or power-up, before the
 * driver can be used; tSLEEP_SPI, the nSLEEP filter; tHI_nSCS between
 * frames.
 */

#define DRV8145_READY_US        3500
#define DRV8145_SLEEP_US        20
#define DRV8145_CSHIGH_US       1

/* Settling time after the DRVOFF pin changes, allowing for whatever the
 * board puts between the pin and the device
 */

#define DRV8145_DRVOFF_US       1000

/* Set-up attempts, for a VM still settling after power-up */

#define DRV8145_SETUP_TRIES     3

/* The fastest PWM on IN the device is specified for */

#define DRV8145_PWM_MAX_HZ      125000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct drv8145_chain_s;

/* One half-bridge, one device node */

struct drv8145_dev_s
{
  FAR struct drv8145_chain_s *chain;
  uint8_t                     index;    /* Position in the chain */
  uint8_t                     devid;    /* DEVICE_ID */
  uint8_t                     status;   /* Status byte of the last frame */
  uint8_t                     spi_in;   /* SPI_IN as last written */
  struct drv8145_config_s     config;   /* CONFIG1-4 as last written */
  struct drv8145_output_s     output;
};

struct drv8145_chain_s
{
  FAR struct spi_dev_s               *spi;
  FAR const struct drv8145_board_s   *board;
  uint32_t                            spidev;   /* Chip select */
  mutex_t                             lock;
  bool                                ready;    /* Set up */
  bool                                asleep;   /* nSLEEP low */
  uint8_t                             seq;      /* HDR2 sequence */
  uint32_t                            por_count;
  uint32_t                            frame_errors;
  FAR uint8_t                        *tx;
  FAR uint8_t                        *rx;
  FAR uint8_t                        *cmd;      /* Per device, scratch */
  FAR uint8_t                        *data;
  FAR uint8_t                        *report;
  struct drv8145_dev_s                dev[1];   /* board->ndev of them */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int drv8145_open(FAR struct file *filep);
static int drv8145_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_drv8145_fops =
{
  .open  = drv8145_open,
  .ioctl = drv8145_ioctl,
};

/* CONFIG1-4 when the board gives none, from Kconfig (whose defaults are
 * the reset values with DRVOFF_SEL OR, so that S_DRVOFF turns the bridge
 * off whatever the DRVOFF pin does).
 */

static const struct drv8145_params_s g_drv8145_default =
{
  .slew       = CONFIG_MOTOR_DRV8145_SR,
  .itrip      = CONFIG_MOTOR_DRV8145_ITRIP,
  .toff       = CONFIG_MOTOR_DRV8145_TOFF,
  .ocp        = CONFIG_MOTOR_DRV8145_OCP,
  .tocp       = CONFIG_MOTOR_DRV8145_TOCP,
  .vmov       = CONFIG_MOTOR_DRV8145_VMOV,
#ifdef CONFIG_MOTOR_DRV8145_LOAD_VM
  .load       = DRV8145_LOAD_VM,
#endif
#ifdef CONFIG_MOTOR_DRV8145_OLP
  .olp        = 1,
#endif
#ifdef CONFIG_MOTOR_DRV8145_OLA
  .ola        = 1,
#endif
#ifdef CONFIG_MOTOR_DRV8145_SSC
  .ssc        = 1,
#endif
#ifdef CONFIG_MOTOR_DRV8145_OCP_RETRY
  .ocp_retry  = 1,
#endif
#ifdef CONFIG_MOTOR_DRV8145_TSD_RETRY
  .tsd_retry  = 1,
#endif
#ifdef CONFIG_MOTOR_DRV8145_VMOV_RETRY
  .vmov_retry = 1,
#endif
#ifdef CONFIG_MOTOR_DRV8145_OLA_RETRY
  .ola_retry  = 1,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv8145_tocfg
 *
 * Description:
 *   Named settings to CONFIG1-4.  The SPARE bits are kept from cfg.
 *
 * Returned Value:
 *   Zero (OK), or -EINVAL if a setting is out of range.
 *
 ****************************************************************************/

static int drv8145_tocfg(FAR const struct drv8145_params_s *p,
                         FAR struct drv8145_config_s *cfg)
{
  if (p->slew >= DRV8145_SR_COUNT || p->itrip >= DRV8145_ITRIP_COUNT ||
      p->toff > DRV8145_TOFF_50US || p->ocp > DRV8145_OCP_75 ||
      p->tocp > DRV8145_TOCP_MIN || p->vmov > DRV8145_VMOV_OFF ||
      p->load > DRV8145_LOAD_VM || p->olp > 1 || p->ola > 1 ||
      p->ssc > 1 || p->ocp_retry > 1 || p->tsd_retry > 1 ||
      p->vmov_retry > 1 || p->ola_retry > 1 || p->drvoff_and > 1 ||
      p->in_and > 1)
    {
      return -EINVAL;
    }

  cfg->config1 = (p->ola ? DRV8145_CFG1_EN_OLA : 0) |
                 (p->vmov << DRV8145_CFG1_VMOV_SHIFT) |
                 (p->ssc ? 0 : DRV8145_CFG1_SSC_DIS) |
                 (p->ocp_retry ? DRV8145_CFG1_OCP_RETRY : 0) |
                 (p->tsd_retry ? DRV8145_CFG1_TSD_RETRY : 0) |
                 (p->vmov_retry ? DRV8145_CFG1_VMOV_RETRY : 0) |
                 (p->ola_retry ? DRV8145_CFG1_OLA_RETRY : 0);
  cfg->config2 = (cfg->config2 & DRV8145_CFG2_SPARE) |
                 (((p->load << 1) | p->olp) << DRV8145_CFG2_DIAG_SHIFT) |
                 (p->itrip << DRV8145_CFG2_ITRIP_SHIFT);
  cfg->config3 = (cfg->config3 & DRV8145_CFG3_SPARE_MASK) |
                 (p->toff << DRV8145_CFG3_TOFF_SHIFT) |
                 (p->slew << DRV8145_CFG3_SR_SHIFT);
  cfg->config4 = (cfg->config4 & DRV8145_CFG4_SPARE) |
                 (p->tocp << DRV8145_CFG4_TOCP_SHIFT) |
                 (p->ocp << DRV8145_CFG4_OCP_SHIFT) |
                 (p->drvoff_and ? DRV8145_CFG4_DRVOFF_AND : 0) |
                 (p->in_and ? DRV8145_CFG4_IN_AND : 0);
  return OK;
}

/****************************************************************************
 * Name: drv8145_fromcfg
 ****************************************************************************/

static void drv8145_fromcfg(FAR const struct drv8145_config_s *cfg,
                            FAR struct drv8145_params_s *p)
{
  uint8_t diag = (cfg->config2 & DRV8145_CFG2_DIAG_MASK) >>
                 DRV8145_CFG2_DIAG_SHIFT;

  p->ola        = (cfg->config1 & DRV8145_CFG1_EN_OLA) != 0;
  p->vmov       = (cfg->config1 & DRV8145_CFG1_VMOV_MASK) >>
                  DRV8145_CFG1_VMOV_SHIFT;
  p->ssc        = (cfg->config1 & DRV8145_CFG1_SSC_DIS) == 0;
  p->ocp_retry  = (cfg->config1 & DRV8145_CFG1_OCP_RETRY) != 0;
  p->tsd_retry  = (cfg->config1 & DRV8145_CFG1_TSD_RETRY) != 0;
  p->vmov_retry = (cfg->config1 & DRV8145_CFG1_VMOV_RETRY) != 0;
  p->ola_retry  = (cfg->config1 & DRV8145_CFG1_OLA_RETRY) != 0;
  p->load       = diag >> 1;
  p->olp        = diag & 1;
  p->itrip      = (cfg->config2 & DRV8145_CFG2_ITRIP_MASK) >>
                  DRV8145_CFG2_ITRIP_SHIFT;
  p->toff       = (cfg->config3 & DRV8145_CFG3_TOFF_MASK) >>
                  DRV8145_CFG3_TOFF_SHIFT;
  p->slew       = (cfg->config3 & DRV8145_CFG3_SR_MASK) >>
                  DRV8145_CFG3_SR_SHIFT;
  p->tocp       = (cfg->config4 & DRV8145_CFG4_TOCP_MASK) >>
                  DRV8145_CFG4_TOCP_SHIFT;
  p->ocp        = (cfg->config4 & DRV8145_CFG4_OCP_MASK) >>
                  DRV8145_CFG4_OCP_SHIFT;
  p->drvoff_and = (cfg->config4 & DRV8145_CFG4_DRVOFF_AND) != 0;
  p->in_and     = (cfg->config4 & DRV8145_CFG4_IN_AND) != 0;
}

/****************************************************************************
 * Name: drv8145_frame
 *
 * Description:
 *   Send one daisy-chain frame: chain->cmd[i] and chain->data[i] to device
 *   i, with a global CLR_FLT if clrflt.  The report bytes land in
 *   chain->report[i] and the status bytes in each device's status.
 *
 * Returned Value:
 *   Zero (OK), or -EIO if the frame came back corrupted.
 *
 ****************************************************************************/

static int drv8145_frame(FAR struct drv8145_chain_s *chain, bool clrflt)
{
  FAR struct spi_dev_s *spi = chain->spi;
  int n = chain->board->ndev;
  int size = DRV8145_FRAME_SIZE(n);
  int i;

  chain->seq = (chain->seq + 1) & DRV8145_HDR2_SEQ_MASK;
  chain->tx[0] = DRV8145_HDR1(n);
  chain->tx[1] = DRV8145_HDR2 | chain->seq |
                 (clrflt ? DRV8145_HDR2_CLR_FLT : 0);

  for (i = 0; i < n; i++)
    {
      chain->tx[2 + (n - 1 - i)]     = chain->cmd[i];
      chain->tx[2 + n + (n - 1 - i)] = chain->data[i];
    }

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, DRV8145_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_MOTOR_DRV8145_FREQUENCY);
  SPI_SELECT(spi, chain->spidev, true);
  SPI_EXCHANGE(spi, chain->tx, chain->rx, size);
  SPI_SELECT(spi, chain->spidev, false);
  SPI_LOCK(spi, false);

  /* nSCS has to stay high for tHI_nSCS before the next frame */

  up_udelay(DRV8145_CSHIGH_US);

  /* The CLR_FLT bit is not compared: SLVSG22B does not say whether the
   * devices pass it on unchanged.
   */

  if (chain->rx[n] != chain->tx[0] ||
      ((chain->rx[n + 1] ^ chain->tx[1]) & ~DRV8145_HDR2_CLR_FLT) != 0)
    {
      chain->frame_errors++;
      mtrinfo("bad header echo %02x %02x, sent %02x %02x\n",
              chain->rx[n], chain->rx[n + 1], chain->tx[0], chain->tx[1]);
      return -EIO;
    }

  for (i = 0; i < n; i++)
    {
      uint8_t status = chain->rx[n - 1 - i];

      if ((status & DRV8145_STAT_ONES) != DRV8145_STAT_ONES)
        {
          chain->frame_errors++;
          mtrinfo("device %d: bad status byte %02x\n", i, status);
          return -EIO;
        }

      chain->dev[i].status = status;
      chain->report[i]     = chain->rx[2 + n + (n - 1 - i)];
    }

  return OK;
}

/****************************************************************************
 * Name: drv8145_nops
 *
 * Description:
 *   Fill the frame with NOPs (DEVICE_ID reads).
 *
 ****************************************************************************/

static void drv8145_nops(FAR struct drv8145_chain_s *chain)
{
  memset(chain->cmd, DRV8145_NOP, chain->board->ndev);
  memset(chain->data, 0, chain->board->ndev);
}

/****************************************************************************
 * Name: drv8145_readall
 *
 * Description:
 *   Read register reg of every device into chain->report.
 *
 ****************************************************************************/

static int drv8145_readall(FAR struct drv8145_chain_s *chain, uint8_t reg)
{
  memset(chain->cmd, DRV8145_CMD_READ | reg, chain->board->ndev);
  memset(chain->data, 0, chain->board->ndev);
  return drv8145_frame(chain, false);
}

/****************************************************************************
 * Name: drv8145_writeall
 *
 * Description:
 *   Write value to register reg of every device.
 *
 ****************************************************************************/

static int drv8145_writeall(FAR struct drv8145_chain_s *chain, uint8_t reg,
                            uint8_t value)
{
  memset(chain->cmd, reg, chain->board->ndev);
  memset(chain->data, value, chain->board->ndev);
  return drv8145_frame(chain, false);
}

/****************************************************************************
 * Name: drv8145_read, drv8145_write
 *
 * Description:
 *   Read or write a register of one device; the others do nothing.
 *
 ****************************************************************************/

static int drv8145_read(FAR struct drv8145_dev_s *dev, uint8_t reg,
                        FAR uint8_t *value)
{
  FAR struct drv8145_chain_s *chain = dev->chain;
  int ret;

  drv8145_nops(chain);
  chain->cmd[dev->index] = DRV8145_CMD_READ | reg;
  ret = drv8145_frame(chain, false);
  if (ret >= 0)
    {
      *value = chain->report[dev->index];
    }

  return ret;
}

static int drv8145_write(FAR struct drv8145_dev_s *dev, uint8_t reg,
                         uint8_t value)
{
  FAR struct drv8145_chain_s *chain = dev->chain;

  drv8145_nops(chain);
  chain->cmd[dev->index]  = reg;
  chain->data[dev->index] = value;
  return drv8145_frame(chain, false);
}

/****************************************************************************
 * Name: drv8145_validreg
 ****************************************************************************/

static bool drv8145_validreg(uint8_t reg)
{
  return reg <= DRV8145_STATUS2 ||
         (reg >= DRV8145_COMMAND && reg <= DRV8145_CONFIG4);
}

/****************************************************************************
 * Name: drv8145_knownid
 ****************************************************************************/

static bool drv8145_knownid(uint8_t id)
{
  switch (id & DRV8145_ID_DEV_MASK)
    {
      case DRV8145_ID_DRV8143S:
      case DRV8145_ID_DRV8144S:
      case DRV8145_ID_DRV8145S:
      case DRV8145_ID_DRV8143P:
      case DRV8145_ID_DRV8145P:
        return true;

      default:
        return false;
    }
}

/****************************************************************************
 * Name: drv8145_pins
 *
 * Description:
 *   Set the board's DRVOFF and IN pins of a device, where it has them.
 *
 ****************************************************************************/

static int drv8145_pins(FAR struct drv8145_dev_s *dev, bool drvoff,
                        uint32_t frequency, ub16_t duty)
{
  FAR const struct drv8145_board_s *board = dev->chain->board;
  int ret = OK;

  if (drvoff && board->drvoff != NULL)
    {
      ret = board->drvoff(board, dev->index, true);
    }

  if (ret >= 0 && board->in != NULL)
    {
      ret = board->in(board, dev->index, frequency, duty);
    }

  if (ret >= 0 && !drvoff && board->drvoff != NULL)
    {
      ret = board->drvoff(board, dev->index, false);
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_configure
 *
 * Description:
 *   Write CONFIG1-4 of the devices whose bit is set in mask, read them
 *   back, and lock the CONFIG registers again.  dev->config is the value
 *   to write.
 *
 ****************************************************************************/

static int drv8145_configure(FAR struct drv8145_chain_s *chain,
                             uint64_t mask)
{
  static const uint8_t rw[4] =
  {
    DRV8145_CFG1_RW, DRV8145_CFG2_RW, DRV8145_CFG3_RW, DRV8145_CFG4_RW
  };

  int n = chain->board->ndev;
  int ret;
  int ret2;
  int reg;
  int i;

  ret = drv8145_writeall(chain, DRV8145_COMMAND, DRV8145_CMD_CONFIG);
  if (ret < 0)
    {
      return ret;
    }

  for (reg = 0; reg < 4 && ret >= 0; reg++)
    {
      drv8145_nops(chain);
      for (i = 0; i < n; i++)
        {
          if (mask & ((uint64_t)1 << i))
            {
              chain->cmd[i]  = DRV8145_CONFIG1 + reg;
              chain->data[i] = ((FAR uint8_t *)&chain->dev[i].config)[reg];
            }
        }

      ret = drv8145_frame(chain, false);
    }

  for (reg = 0; reg < 4 && ret >= 0; reg++)
    {
      ret = drv8145_readall(chain, DRV8145_CONFIG1 + reg);
      for (i = 0; i < n && ret >= 0; i++)
        {
          uint8_t want = ((FAR uint8_t *)&chain->dev[i].config)[reg];

          if ((mask & ((uint64_t)1 << i)) &&
              ((chain->report[i] ^ want) & rw[reg]) != 0)
            {
              mtrerr("ERROR: device %d: CONFIG%d reads %02x, wrote %02x\n",
                     i, reg + 1, chain->report[i], want);
              ret = -EIO;
            }
        }
    }

  /* Lock again whatever happened */

  ret2 = drv8145_writeall(chain, DRV8145_COMMAND, DRV8145_CMD_RUN);
  return ret < 0 ? ret : ret2;
}

/****************************************************************************
 * Name: drv8145_restore
 *
 * Description:
 *   After a CLR_FLT: make sure the lock states are the driver's and every
 *   SPI_IN is as last written.
 *
 ****************************************************************************/

static int drv8145_restore(FAR struct drv8145_chain_s *chain)
{
  int n = chain->board->ndev;
  int ret;
  int i;

  ret = drv8145_writeall(chain, DRV8145_COMMAND, DRV8145_CMD_RUN);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < n; i++)
    {
      chain->cmd[i]  = DRV8145_SPI_IN;
      chain->data[i] = chain->dev[i].spi_in;
    }

  return drv8145_frame(chain, false);
}

/****************************************************************************
 * Name: drv8145_clearfaults
 ****************************************************************************/

static int drv8145_clearfaults(FAR struct drv8145_chain_s *chain)
{
  int ret;

  drv8145_nops(chain);
  ret = drv8145_frame(chain, true);
  if (ret >= 0)
    {
      ret = drv8145_restore(chain);
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_trysetup
 *
 * Description:
 *   One attempt at setting the chain up.  Every output is left Hi-Z.
 *
 ****************************************************************************/

static int drv8145_trysetup(FAR struct drv8145_chain_s *chain)
{
  FAR const struct drv8145_board_s *board = chain->board;
  int n = board->ndev;
  int ret;
  int i;

  /* Acknowledge the power-up or wake-up with a CLR_FLT.  A chain without
   * VM does not drive SDO (an S variant's SDO is powered from nSLEEP and
   * its logic from VM), so this is the first frame that can fail.
   */

  drv8145_nops(chain);
  ret = drv8145_frame(chain, true);
  if (ret < 0)
    {
      return -ENODEV;
    }

  ret = drv8145_readall(chain, DRV8145_DEVICE_ID);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < n; i++)
    {
      chain->dev[i].devid = chain->report[i];
      if (!drv8145_knownid(chain->report[i]))
        {
          mtrerr("ERROR: device %d: unknown DEVICE_ID %02x\n",
                 i, chain->report[i]);
          return -ENODEV;
        }

      if ((chain->report[i] & DRV8145_ID_DEV_MASK) !=
          DRV8145_ID_DRV8145S &&
          (chain->report[i] & DRV8145_ID_DEV_MASK) != DRV8145_ID_DRV8145P)
        {
          mtrwarn("WARNING: device %d: DEVICE_ID %02x is not a DRV8145\n",
                  i, chain->report[i]);
        }
    }

  /* Bridges off before SPI_IN is unlocked: from then on S_DRVOFF counts
   * as well as the pin.
   */

  for (i = 0; i < n; i++)
    {
      FAR struct drv8145_dev_s *dev = &chain->dev[i];

      dev->spi_in = DRV8145_SPI_IN_S_DRVOFF;
      memset(&dev->output, 0, sizeof(dev->output));
      dev->output.mode = DRV8145_OUT_HIZ;
      drv8145_pins(dev, true, 0, 0);
    }

  ret = drv8145_configure(chain, ((uint64_t)1 << n) - 1);
  if (ret < 0)
    {
      return ret;
    }

  ret = drv8145_writeall(chain, DRV8145_SPI_IN, DRV8145_SPI_IN_S_DRVOFF);
  if (ret < 0)
    {
      return ret;
    }

  /* Check the locks took, and clear whatever the set-up raised */

  ret = drv8145_readall(chain, DRV8145_COMMAND);
  for (i = 0; i < n && ret >= 0; i++)
    {
      if ((chain->report[i] & DRV8145_CMD_LOCK_MASK) != DRV8145_CMD_RUN)
        {
          mtrerr("ERROR: device %d: COMMAND reads %02x\n",
                 i, chain->report[i]);
          ret = -EIO;
        }
    }

  if (ret >= 0)
    {
      ret = drv8145_clearfaults(chain);
    }

  if (ret >= 0)
    {
      ret = drv8145_readall(chain, DRV8145_FAULT_SUMMARY);
    }

  for (i = 0; i < n && ret >= 0; i++)
    {
      if (chain->report[i] & DRV8145_FSUM_FAULT)
        {
          mtrwarn("WARNING: device %d: FAULT_SUMMARY %02x after set-up\n",
                  i, chain->report[i]);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_setup
 *
 * Description:
 *   Set the chain up: wake it, acknowledge the power-up, check the IDs,
 *   write and lock the configuration, unlock SPI_IN with every bridge
 *   off.  Retried, as a VM still ringing after power-up can make the
 *   devices reject writes.
 *
 ****************************************************************************/

static int drv8145_setup(FAR struct drv8145_chain_s *chain)
{
  FAR const struct drv8145_board_s *board = chain->board;
  int ret = -EIO;
  int try;

  chain->ready = false;

  if (board->sleep != NULL)
    {
      ret = board->sleep(board, false);
      if (ret < 0)
        {
          return ret;
        }

      chain->asleep = false;
    }

  nxsig_usleep(DRV8145_READY_US);

  for (try = 0; try < DRV8145_SETUP_TRIES; try++)
    {
      ret = drv8145_trysetup(chain);
      if (ret >= 0 || ret == -ENODEV)
        {
          break;
        }

      nxsig_usleep(DRV8145_READY_US);
    }

  if (ret >= 0)
    {
      chain->ready = true;
    }
  else if (ret != -ENODEV)
    {
      /* -ENODEV is a chain that does not answer, most likely without VM:
       * not an error until someone wants to use it.
       */

      mtrerr("ERROR: chain set-up failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_ready
 *
 * Description:
 *   Make sure the chain is set up; set it up again after a power-on reset
 *   of any device (VM dropped below VMPOR, which loses the configuration).
 *
 ****************************************************************************/

static int drv8145_ready(FAR struct drv8145_chain_s *chain)
{
  int n = chain->board->ndev;
  int ret;
  int i;

  if (chain->asleep)
    {
      return -EAGAIN;
    }

  if (!chain->ready)
    {
      return drv8145_setup(chain);
    }

  ret = drv8145_readall(chain, DRV8145_FAULT_SUMMARY);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < n; i++)
    {
      if (chain->report[i] & DRV8145_FSUM_POR)
        {
          mtrwarn("WARNING: device %d was reset, setting up again\n", i);
          chain->por_count++;
          return drv8145_setup(chain);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: drv8145_setoutput
 ****************************************************************************/

static int drv8145_setoutput(FAR struct drv8145_dev_s *dev,
                             FAR const struct drv8145_output_s *output)
{
  FAR const struct drv8145_board_s *board = dev->chain->board;
  bool in_and = (dev->config.config4 & DRV8145_CFG4_IN_AND) != 0;
  uint32_t frequency = 0;
  ub16_t duty = 0;
  uint8_t spi_in;
  uint8_t readback;
  int ret;

  switch (output->mode)
    {
      case DRV8145_OUT_HIZ:
        spi_in = DRV8145_SPI_IN_S_DRVOFF;
        break;

      case DRV8145_OUT_LOW:
        spi_in = 0;
        break;

      case DRV8145_OUT_HIGH:
        spi_in = DRV8145_SPI_IN_S_IN;
        duty   = b16ONE;
        break;

      case DRV8145_OUT_PWM:
        if (board->in == NULL)
          {
            return -ENOTSUP;
          }

        if (output->frequency == 0 ||
            output->frequency > DRV8145_PWM_MAX_HZ ||
            output->duty > b16ONE)
          {
            return -EINVAL;
          }

        /* IN_SEL AND: the pin passes only with S_IN set */

        spi_in    = in_and ? DRV8145_SPI_IN_S_IN : 0;
        frequency = output->frequency;
        duty      = output->duty;
        break;

      default:
        return -EINVAL;
    }

  /* Turning off: S_DRVOFF first, then the pins.  Turning on: IN first,
   * then DRVOFF released.
   */

  if (output->mode == DRV8145_OUT_HIZ)
    {
      ret = drv8145_write(dev, DRV8145_SPI_IN, spi_in);
      dev->spi_in = spi_in;
      if (ret >= 0)
        {
          ret = drv8145_pins(dev, true, 0, 0);
        }
    }
  else
    {
      ret = drv8145_pins(dev, false, frequency, duty);
      if (ret >= 0)
        {
          ret = drv8145_write(dev, DRV8145_SPI_IN, spi_in);
          dev->spi_in = spi_in;
        }
    }

  if (ret < 0)
    {
      return ret;
    }

  ret = drv8145_read(dev, DRV8145_SPI_IN, &readback);
  if (ret < 0)
    {
      return ret;
    }

  if (((readback ^ spi_in) &
       (DRV8145_SPI_IN_S_DRVOFF | DRV8145_SPI_IN_S_IN)) != 0)
    {
      mtrerr("ERROR: device %d: SPI_IN reads %02x, wrote %02x\n",
             dev->index, readback, spi_in);
      return -EIO;
    }

  dev->output = *output;
  if (output->mode != DRV8145_OUT_PWM)
    {
      dev->output.frequency = 0;
      dev->output.duty      = duty;
    }

  return OK;
}

/****************************************************************************
 * Name: drv8145_getstatus
 ****************************************************************************/

static int drv8145_getstatus(FAR struct drv8145_dev_s *dev,
                             FAR struct drv8145_status_s *status)
{
  FAR struct drv8145_chain_s *chain = dev->chain;
  FAR const struct drv8145_board_s *board = chain->board;
  int ret;

  memset(status, 0, sizeof(*status));

  ret = drv8145_read(dev, DRV8145_FAULT_SUMMARY, &status->fault_summary);
  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_STATUS1, &status->status1);
    }

  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_STATUS2, &status->status2);
    }

  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_COMMAND, &status->command);
    }

  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_SPI_IN, &status->spi_in);
    }

  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_DEVICE_ID, &status->devid);
    }

  if (ret < 0)
    {
      return ret;
    }

  status->status       = dev->status;
  status->nfault       = board->nfault != NULL ? board->nfault(board) : -1;
  status->por_count    = chain->por_count;
  status->frame_errors = chain->frame_errors;
  return OK;
}

/****************************************************************************
 * Name: drv8145_olp
 *
 * Description:
 *   Off-state diagnostics: with DRVOFF and IN high, read OLP_CMP with the
 *   output pulled up (S_DIAG 01b) and pulled down (S_DIAG 11b), and judge
 *   them against Table 7-8 (load to GND) or 7-9 (load to VM).
 *
 ****************************************************************************/

static int drv8145_olp(FAR struct drv8145_dev_s *dev,
                       FAR struct drv8145_olp_s *olp)
{
  FAR struct drv8145_chain_s *chain = dev->chain;
  uint8_t config2 = dev->config.config2;
  bool vmload = (config2 & DRV8145_CFG2_DIAG_VM) != 0;
  uint8_t status2;
  int ret;
  int ret2;

  if (dev->output.mode != DRV8145_OUT_HIZ)
    {
      return -EBUSY;
    }

  memset(olp, 0, sizeof(*olp));

  /* STANDBY with DRVOFF = 1 and IN = 1 */

  ret = drv8145_write(dev, DRV8145_SPI_IN,
                      DRV8145_SPI_IN_S_DRVOFF | DRV8145_SPI_IN_S_IN);
  if (ret >= 0)
    {
      ret = drv8145_pins(dev, true, 0, b16ONE);
    }

  if (ret >= 0)
    {
      ret = drv8145_writeall(chain, DRV8145_COMMAND, DRV8145_CMD_CONFIG);
    }

  /* Pull-up, comparator at VOLP_REFH */

  if (ret >= 0)
    {
      ret = drv8145_write(dev, DRV8145_CONFIG2,
                          (config2 & ~DRV8145_CFG2_DIAG_MASK) |
                          DRV8145_CFG2_DIAG_GND_OLP);
    }

  if (ret >= 0)
    {
      nxsig_usleep(CONFIG_MOTOR_DRV8145_OLP_SETTLE_US);
      ret = drv8145_read(dev, DRV8145_STATUS2, &status2);
      olp->cmp_pu = (status2 & DRV8145_STATUS2_OLP_CMP) != 0;
    }

  /* Pull-down, comparator at VOLP_REFL */

  if (ret >= 0)
    {
      ret = drv8145_write(dev, DRV8145_CONFIG2,
                          (config2 & ~DRV8145_CFG2_DIAG_MASK) |
                          DRV8145_CFG2_DIAG_VM_OLP);
    }

  if (ret >= 0)
    {
      nxsig_usleep(CONFIG_MOTOR_DRV8145_OLP_SETTLE_US);
      ret = drv8145_read(dev, DRV8145_STATUS2, &status2);
      olp->cmp_pd = (status2 & DRV8145_STATUS2_OLP_CMP) != 0;
    }

  /* Put everything back, whatever happened */

  ret2 = drv8145_write(dev, DRV8145_CONFIG2, config2);
  if (ret2 >= 0)
    {
      ret2 = drv8145_writeall(chain, DRV8145_COMMAND, DRV8145_CMD_RUN);
    }

  if (ret2 >= 0)
    {
      ret2 = drv8145_write(dev, DRV8145_SPI_IN, dev->spi_in);
    }

  if (ret2 >= 0)
    {
      ret2 = drv8145_pins(dev, true, 0, 0);
    }

  if (ret < 0 || ret2 < 0)
    {
      return ret < 0 ? ret : ret2;
    }

  if (!vmload)
    {
      /* Table 7-8, load to GND: normal L L, open H L, short H H */

      olp->result = !olp->cmp_pu && !olp->cmp_pd ? DRV8145_OLP_NORMAL :
                    olp->cmp_pu && !olp->cmp_pd  ? DRV8145_OLP_OPEN :
                    olp->cmp_pu && olp->cmp_pd   ? DRV8145_OLP_SHORT :
                                                   DRV8145_OLP_UNKNOWN;
    }
  else
    {
      /* Table 7-9, load to VM: normal H H, open H L, short L L */

      olp->result = olp->cmp_pu && olp->cmp_pd   ? DRV8145_OLP_NORMAL :
                    olp->cmp_pu && !olp->cmp_pd  ? DRV8145_OLP_OPEN :
                    !olp->cmp_pu && !olp->cmp_pd ? DRV8145_OLP_SHORT :
                                                   DRV8145_OLP_UNKNOWN;
    }

  return OK;
}

/****************************************************************************
 * Name: drv8145_drvofftest
 *
 * Description:
 *   Drive the DRVOFF pin low, then high, and read DRVOFF_STAT each time.
 *   S_DRVOFF with DRVOFF_SEL OR keeps the bridge off throughout.
 *
 ****************************************************************************/

static int drv8145_drvofftest(FAR struct drv8145_dev_s *dev,
                              FAR struct drv8145_drvofftest_s *test)
{
  FAR const struct drv8145_board_s *board = dev->chain->board;
  uint8_t spi_in;
  uint8_t status2;
  int ret;
  int ret2;

  if (board->drvoff == NULL)
    {
      return -ENOTSUP;
    }

  if (dev->output.mode != DRV8145_OUT_HIZ ||
      (dev->config.config4 & DRV8145_CFG4_DRVOFF_AND) != 0)
    {
      return -EBUSY;
    }

  /* Make sure S_DRVOFF really is set before the pin is released */

  ret = drv8145_read(dev, DRV8145_SPI_IN, &spi_in);
  if (ret < 0)
    {
      return ret;
    }

  if ((spi_in & DRV8145_SPI_IN_S_DRVOFF) == 0)
    {
      return -EIO;
    }

  ret = board->drvoff(board, dev->index, false);
  if (ret >= 0)
    {
      nxsig_usleep(DRV8145_DRVOFF_US);
      ret = drv8145_read(dev, DRV8145_STATUS2, &status2);
      test->stat_low = (status2 & DRV8145_STATUS2_DRVOFF_STAT) != 0;
    }

  ret2 = board->drvoff(board, dev->index, true);
  if (ret >= 0 && ret2 >= 0)
    {
      nxsig_usleep(DRV8145_DRVOFF_US);
      ret = drv8145_read(dev, DRV8145_STATUS2, &status2);
      test->stat_high = (status2 & DRV8145_STATUS2_DRVOFF_STAT) != 0;
    }

  return ret < 0 ? ret : ret2;
}

/****************************************************************************
 * Name: drv8145_sleep
 ****************************************************************************/

static int drv8145_sleep(FAR struct drv8145_chain_s *chain, bool sleep)
{
  FAR const struct drv8145_board_s *board = chain->board;
  int ret;
  int i;

  if (board->sleep == NULL)
    {
      return -ENOTSUP;
    }

  if (!sleep)
    {
      return drv8145_setup(chain);
    }

  for (i = 0; i < board->ndev; i++)
    {
      chain->dev[i].output.mode = DRV8145_OUT_HIZ;
      drv8145_pins(&chain->dev[i], true, 0, 0);
    }

  ret = board->sleep(board, true);
  if (ret >= 0)
    {
      nxsig_usleep(DRV8145_SLEEP_US);
      chain->asleep = true;
      chain->ready  = false;
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_getconfig
 ****************************************************************************/

static int drv8145_getconfig(FAR struct drv8145_dev_s *dev,
                             FAR struct drv8145_config_s *config)
{
  int ret;

  ret = drv8145_read(dev, DRV8145_CONFIG1, &config->config1);
  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_CONFIG2, &config->config2);
    }

  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_CONFIG3, &config->config3);
    }

  if (ret >= 0)
    {
      ret = drv8145_read(dev, DRV8145_CONFIG4, &config->config4);
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_setconfig
 *
 * Description:
 *   Write, check and lock CONFIG1-4 of one device, and keep them for any
 *   later set-up.  DRVOFF_SEL and IN_SEL change what SPI_IN has to be, so
 *   the output is applied again.
 *
 ****************************************************************************/

static int drv8145_setconfig(FAR struct drv8145_dev_s *dev,
                             FAR const struct drv8145_config_s *config)
{
  struct drv8145_output_s output;
  int ret;

  dev->config = *config;
  ret = drv8145_configure(dev->chain, (uint64_t)1 << dev->index);
  if (ret >= 0)
    {
      output = dev->output;
      ret = drv8145_setoutput(dev, &output);
    }

  return ret;
}

/****************************************************************************
 * Name: drv8145_open
 ****************************************************************************/

static int drv8145_open(FAR struct file *filep)
{
  FAR struct drv8145_dev_s *dev = filep->f_inode->i_private;
  FAR struct drv8145_chain_s *chain = dev->chain;
  int ret;

  ret = nxmutex_lock(&chain->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Try a set-up that failed at registration again, but let the open
   * succeed: the status and REINIT are still useful.
   */

  if (!chain->ready && !chain->asleep)
    {
      drv8145_setup(chain);
    }

  nxmutex_unlock(&chain->lock);
  return OK;
}

/****************************************************************************
 * Name: drv8145_ioctl
 ****************************************************************************/

static int drv8145_ioctl(FAR struct file *filep, int cmd,
                         unsigned long arg)
{
  FAR struct drv8145_dev_s *dev = filep->f_inode->i_private;
  FAR struct drv8145_chain_s *chain = dev->chain;
  int ret;

  ret = nxmutex_lock(&chain->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case DRV8145IOC_SLEEP:
        ret = drv8145_sleep(chain, arg != 0);
        goto out;

      case DRV8145IOC_REINIT:
        if (chain->asleep)
          {
            ret = -EAGAIN;
          }
        else
          {
            ret = drv8145_setup(chain);
          }

        goto out;

      case DRV8145IOC_GETOUTPUT:
        {
          FAR struct drv8145_output_s *output =
            (FAR struct drv8145_output_s *)((uintptr_t)arg);

          if (output == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              *output = dev->output;
            }
        }

        goto out;

      default:
        break;
    }

  ret = drv8145_ready(chain);
  if (ret < 0)
    {
      goto out;
    }

  switch (cmd)
    {
      case DRV8145IOC_GETSTATUS:
        {
          FAR struct drv8145_status_s *status =
            (FAR struct drv8145_status_s *)((uintptr_t)arg);

          ret = status != NULL ? drv8145_getstatus(dev, status) : -EINVAL;
        }
        break;

      case DRV8145IOC_CLEARFAULTS:
        ret = drv8145_clearfaults(chain);
        break;

      case DRV8145IOC_GETCONFIG:
        {
          FAR struct drv8145_config_s *config =
            (FAR struct drv8145_config_s *)((uintptr_t)arg);

          ret = config != NULL ? drv8145_getconfig(dev, config) : -EINVAL;
        }
        break;

      case DRV8145IOC_SETCONFIG:
        {
          FAR const struct drv8145_config_s *config =
            (FAR const struct drv8145_config_s *)((uintptr_t)arg);

          ret = config != NULL ? drv8145_setconfig(dev, config) : -EINVAL;
        }
        break;

      case DRV8145IOC_GETPARAMS:
        {
          FAR struct drv8145_params_s *params =
            (FAR struct drv8145_params_s *)((uintptr_t)arg);
          struct drv8145_config_s config;

          if (params == NULL)
            {
              ret = -EINVAL;
              break;
            }

          ret = drv8145_getconfig(dev, &config);
          if (ret >= 0)
            {
              drv8145_fromcfg(&config, params);
            }
        }
        break;

      case DRV8145IOC_SETPARAMS:
        {
          FAR const struct drv8145_params_s *params =
            (FAR const struct drv8145_params_s *)((uintptr_t)arg);
          struct drv8145_config_s config = dev->config;

          ret = params != NULL ? drv8145_tocfg(params, &config) : -EINVAL;
          if (ret >= 0)
            {
              ret = drv8145_setconfig(dev, &config);
            }
        }
        break;

      case DRV8145IOC_SETOUTPUT:
        {
          FAR const struct drv8145_output_s *output =
            (FAR const struct drv8145_output_s *)((uintptr_t)arg);

          ret = output != NULL ? drv8145_setoutput(dev, output) : -EINVAL;
        }
        break;

      case DRV8145IOC_OLP:
        {
          FAR struct drv8145_olp_s *olp =
            (FAR struct drv8145_olp_s *)((uintptr_t)arg);

          ret = olp != NULL ? drv8145_olp(dev, olp) : -EINVAL;
        }
        break;

      case DRV8145IOC_DRVOFFTEST:
        {
          FAR struct drv8145_drvofftest_s *test =
            (FAR struct drv8145_drvofftest_s *)((uintptr_t)arg);

          ret = test != NULL ? drv8145_drvofftest(dev, test) : -EINVAL;
        }
        break;

      case DRV8145IOC_READREG:
        {
          FAR struct drv8145_reg_s *reg =
            (FAR struct drv8145_reg_s *)((uintptr_t)arg);

          if (reg == NULL || !drv8145_validreg(reg->addr))
            {
              ret = -EINVAL;
              break;
            }

          ret = drv8145_read(dev, reg->addr, &reg->value);
        }
        break;

      case DRV8145IOC_WRITEREG:
        {
          FAR const struct drv8145_reg_s *reg =
            (FAR const struct drv8145_reg_s *)((uintptr_t)arg);

          if (reg == NULL || !drv8145_validreg(reg->addr) ||
              reg->addr <= DRV8145_STATUS2)
            {
              ret = -EINVAL;
              break;
            }

          ret = drv8145_write(dev, reg->addr, reg->value);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

out:
  nxmutex_unlock(&chain->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv8145_register
 *
 * Description:
 *   Register a DRV8145 daisy chain.  See include/nuttx/motor/drv8145.h.
 *
 ****************************************************************************/

int drv8145_register(FAR struct spi_dev_s *spi, uint32_t devid,
                     FAR const struct drv8145_board_s *board)
{
  FAR struct drv8145_chain_s *chain;
  FAR uint8_t *buf;
  char path[16];
  size_t size;
  int n;
  int ret;
  int i;

  if (spi == NULL || board == NULL || board->ndev < 1 ||
      board->ndev > DRV8145_MAX_CHAIN)
    {
      return -EINVAL;
    }

  n = board->ndev;

  /* The chain, its devices, then the frame buffers and per-device
   * scratch in one allocation.
   */

  size = sizeof(struct drv8145_chain_s) +
         (n - 1) * sizeof(struct drv8145_dev_s) +
         2 * DRV8145_FRAME_SIZE(n) + 3 * n;

  chain = kmm_zalloc(size);
  if (chain == NULL)
    {
      return -ENOMEM;
    }

  buf           = (FAR uint8_t *)&chain->dev[n];
  chain->tx     = buf;
  chain->rx     = buf + DRV8145_FRAME_SIZE(n);
  chain->cmd    = buf + 2 * DRV8145_FRAME_SIZE(n);
  chain->data   = chain->cmd + n;
  chain->report = chain->data + n;
  chain->spi    = spi;
  chain->spidev = devid;
  chain->board  = board;
  nxmutex_init(&chain->lock);

  for (i = 0; i < n; i++)
    {
      chain->dev[i].chain  = chain;
      chain->dev[i].index  = i;
      if (board->config == NULL)
        {
          drv8145_tocfg(&g_drv8145_default, &chain->dev[i].config);
        }
      else
        {
          chain->dev[i].config = board->config[i];
        }
    }

  ret = drv8145_setup(chain);
  if (ret == -ENODEV)
    {
      mtrwarn("WARNING: DRV8145 chain does not answer (VM off?), "
              "will set it up on open\n");
    }
  else if (ret < 0)
    {
      mtrwarn("WARNING: DRV8145 chain not set up (%d), "
              "will retry on open\n", ret);
    }
  else
    {
      mtrinfo("%d DRV8145 devices set up\n", n);
    }

  for (i = 0; i < n; i++)
    {
      snprintf(path, sizeof(path), "/dev/hb%d", board->minor + i);
      ret = register_driver(path, &g_drv8145_fops, 0666, &chain->dev[i]);
      if (ret < 0)
        {
          mtrerr("ERROR: %s not registered: %d\n", path, ret);
          while (--i >= 0)
            {
              snprintf(path, sizeof(path), "/dev/hb%d", board->minor + i);
              unregister_driver(path);
            }

          nxmutex_destroy(&chain->lock);
          kmm_free(chain);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_MOTOR_DRV8145 */
