/****************************************************************************
 * drivers/misc/slg46826.c
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

/* Renesas GreenPAK SLG46826 (SLG46826V) on I2C.
 *
 * The device answers at four 7-bit addresses: the control code in bits
 * 6:3 and a block select in bits 2:0 - registers (base + 0), NVM
 * (base + 2) and emulated EEPROM (base + 3).  NVM is 16 pages of 16 bytes;
 * page 15 (0xf0-0xff) is the service page and is never erased or written.
 *
 * A RAM load writes registers 0x00-0xc7, the logic configuration.  Above
 * that are the soft reset, the I2C control code, the silicon ID and
 * service bits, and the protection, erase and lock registers, none of
 * which a design load should touch.
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
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/misc/slg46826.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>

#ifdef CONFIG_SLG46826

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Blocks, added to the register block address */

#define SLG46826_BLOCK_REG      0
#define SLG46826_BLOCK_NVM      2

/* Registers */

#define SLG46826_REG_SWRST      0xc8  /* Bit 1: reset, reload from NVM */
#define SLG46826_REG_ERSR       0xe3  /* NVM erase */

#define SLG46826_SWRST_RESET    0x02
#define SLG46826_ERSR_ERASE(n)  (0x80 | ((n) & 0x0f))  /* NVM page n */

/* Geometry */

#define SLG46826_PAGE_SIZE      16
#define SLG46826_NVM_PAGES      15    /* Pages 0-14; 15 is the service page */
#define SLG46826_NVM_SIZE       (SLG46826_NVM_PAGES * SLG46826_PAGE_SIZE)
#define SLG46826_RAM_SIZE       0xc8  /* Registers 0x00-0xc7 */

/* Timing: erase and write take up to 20 ms a page.  The erase write is
 * always NACKed (known erratum), so it is followed by a fixed delay.
 */

#define SLG46826_ERASE_MS       25
#define SLG46826_WRITE_MS       25
#define SLG46826_RESET_MS       10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct slg46826_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t                  addr;
  mutex_t                  lock;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t slg46826_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t slg46826_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen);
static off_t   slg46826_seek(FAR struct file *filep, off_t offset,
                             int whence);
static int     slg46826_ioctl(FAR struct file *filep, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_slg46826_fops =
{
  .read  = slg46826_read,
  .write = slg46826_write,
  .seek  = slg46826_seek,
  .ioctl = slg46826_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slg46826_readblk
 *
 * Description:
 *   Read len bytes of a block from offset: the offset, then a repeated
 *   START and the read.
 *
 ****************************************************************************/

static int slg46826_readblk(FAR struct slg46826_dev_s *priv, int block,
                            uint8_t offset, FAR uint8_t *buf, size_t len)
{
  struct i2c_msg_s msg[2];

  msg[0].frequency = CONFIG_SLG46826_I2C_FREQUENCY;
  msg[0].addr      = priv->addr + block;
  msg[0].flags     = 0;
  msg[0].buffer    = &offset;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_SLG46826_I2C_FREQUENCY;
  msg[1].addr      = priv->addr + block;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buf;
  msg[1].length    = len;

  return I2C_TRANSFER(priv->i2c, msg, 2);
}

/****************************************************************************
 * Name: slg46826_writeblk
 *
 * Description:
 *   Write up to one page to a block at offset.
 *
 ****************************************************************************/

static int slg46826_writeblk(FAR struct slg46826_dev_s *priv, int block,
                             uint8_t offset, FAR const uint8_t *buf,
                             size_t len)
{
  uint8_t frame[1 + SLG46826_PAGE_SIZE];
  struct i2c_msg_s msg;

  DEBUGASSERT(len <= SLG46826_PAGE_SIZE);

  frame[0] = offset;
  memcpy(&frame[1], buf, len);

  msg.frequency = CONFIG_SLG46826_I2C_FREQUENCY;
  msg.addr      = priv->addr + block;
  msg.flags     = 0;
  msg.buffer    = frame;
  msg.length    = 1 + len;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: slg46826_writeregs
 *
 * Description:
 *   Write a range of the register block, a page at a time.
 *
 ****************************************************************************/

static int slg46826_writeregs(FAR struct slg46826_dev_s *priv,
                              unsigned int offset, FAR const uint8_t *buf,
                              size_t len)
{
  size_t chunk;
  int ret;

  while (len > 0)
    {
      chunk = len < SLG46826_PAGE_SIZE ? len : SLG46826_PAGE_SIZE;
      ret = slg46826_writeblk(priv, SLG46826_BLOCK_REG, offset, buf, chunk);
      if (ret < 0)
        {
          return ret;
        }

      offset += chunk;
      buf    += chunk;
      len    -= chunk;
    }

  return OK;
}

/****************************************************************************
 * Name: slg46826_probe
 ****************************************************************************/

static int slg46826_probe(FAR struct slg46826_dev_s *priv)
{
  struct i2c_msg_s msg;

  msg.frequency = CONFIG_SLG46826_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = NULL;
  msg.length    = 0;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: slg46826_reset
 ****************************************************************************/

static int slg46826_reset(FAR struct slg46826_dev_s *priv)
{
  uint8_t val = SLG46826_SWRST_RESET;

  /* The device may reset before it acknowledges the data byte */

  slg46826_writeblk(priv, SLG46826_BLOCK_REG, SLG46826_REG_SWRST, &val, 1);
  nxsched_usleep(SLG46826_RESET_MS * 1000);
  return slg46826_probe(priv);
}

/****************************************************************************
 * Name: slg46826_ramload
 ****************************************************************************/

static int slg46826_ramload(FAR struct slg46826_dev_s *priv,
                            FAR const uint8_t *image)
{
  return slg46826_writeregs(priv, 0, image, SLG46826_RAM_SIZE);
}

/****************************************************************************
 * Name: slg46826_nvmread
 ****************************************************************************/

static int slg46826_nvmread(FAR struct slg46826_dev_s *priv,
                            FAR uint8_t *buf)
{
  return slg46826_readblk(priv, SLG46826_BLOCK_NVM, 0, buf,
                          SLG46826_IMAGE_SIZE);
}

/****************************************************************************
 * Name: slg46826_nvmprogram
 *
 * Description:
 *   Erase and write pages 0-14, reset so the registers reload from the new
 *   NVM, and compare.
 *
 ****************************************************************************/

static int slg46826_nvmprogram(FAR struct slg46826_dev_s *priv,
                               FAR const uint8_t *image)
{
  FAR uint8_t *readback;
  uint8_t val;
  int page;
  int ret;

  for (page = 0; page < SLG46826_NVM_PAGES; page++)
    {
      val = SLG46826_ERSR_ERASE(page);
      slg46826_writeblk(priv, SLG46826_BLOCK_REG, SLG46826_REG_ERSR,
                        &val, 1);
      nxsched_usleep(SLG46826_ERASE_MS * 1000);
    }

  for (page = 0; page < SLG46826_NVM_PAGES; page++)
    {
      ret = slg46826_writeblk(priv, SLG46826_BLOCK_NVM,
                              page * SLG46826_PAGE_SIZE,
                              &image[page * SLG46826_PAGE_SIZE],
                              SLG46826_PAGE_SIZE);
      if (ret < 0)
        {
          _err("ERROR: SLG46826 NVM page %d write failed: %d\n", page, ret);
          return ret;
        }

      nxsched_usleep(SLG46826_WRITE_MS * 1000);
    }

  ret = slg46826_reset(priv);
  if (ret < 0)
    {
      return ret;
    }

  readback = kmm_malloc(SLG46826_IMAGE_SIZE);
  if (readback == NULL)
    {
      return -ENOMEM;
    }

  ret = slg46826_nvmread(priv, readback);
  if (ret >= 0 && memcmp(readback, image, SLG46826_NVM_SIZE) != 0)
    {
      _err("ERROR: SLG46826 NVM verify failed\n");
      ret = -EIO;
    }

  kmm_free(readback);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: slg46826_bootstrap
 *
 * Description:
 *   Do what the board asked with its design.  NVM is programmed only when
 *   the configuration registers read back all zero, as on a blank part;
 *   a part carrying a different design is reported and left alone.
 *
 ****************************************************************************/

static int slg46826_bootstrap(FAR struct slg46826_dev_s *priv,
                              FAR const struct slg46826_config_s *config)
{
  FAR uint8_t *nvm;
  bool blank = true;
  int ret;
  int i;

  if (config->boot == SLG46826_BOOT_NONE || config->image == NULL)
    {
      return OK;
    }

  if (config->boot == SLG46826_BOOT_RAM)
    {
      ret = slg46826_ramload(priv, config->image);
      _info("SLG46826 RAM load: %d\n", ret);
      return ret;
    }

  nvm = kmm_malloc(SLG46826_IMAGE_SIZE);
  if (nvm == NULL)
    {
      return -ENOMEM;
    }

  ret = slg46826_nvmread(priv, nvm);
  if (ret < 0)
    {
      goto out;
    }

  if (memcmp(nvm, config->image, SLG46826_NVM_SIZE) == 0)
    {
      _info("SLG46826 NVM already holds the design\n");
      goto out;
    }

  for (i = 0; i < SLG46826_RAM_SIZE; i++)
    {
      if (nvm[i] != 0)
        {
          blank = false;
          break;
        }
    }

  if (!blank)
    {
      _warn("WARNING: SLG46826 holds another design; NVM not touched\n");
      goto out;
    }

  _warn("WARNING: SLG46826 is blank; programming its NVM\n");
  ret = slg46826_nvmprogram(priv, config->image);

out:
  kmm_free(nvm);
  return ret;
}

/****************************************************************************
 * Name: slg46826_read
 ****************************************************************************/

static ssize_t slg46826_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct slg46826_dev_s *priv = filep->f_inode->i_private;
  size_t len;
  int ret;

  if (filep->f_pos >= SLG46826_IMAGE_SIZE)
    {
      return 0;
    }

  len = SLG46826_IMAGE_SIZE - filep->f_pos;
  if (len > buflen)
    {
      len = buflen;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = slg46826_readblk(priv, SLG46826_BLOCK_REG, filep->f_pos,
                         (FAR uint8_t *)buffer, len);
  nxmutex_unlock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  filep->f_pos += len;
  return len;
}

/****************************************************************************
 * Name: slg46826_write
 ****************************************************************************/

static ssize_t slg46826_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen)
{
  FAR struct slg46826_dev_s *priv = filep->f_inode->i_private;
  size_t len;
  int ret;

  if (filep->f_pos >= SLG46826_IMAGE_SIZE)
    {
      return -EFBIG;
    }

  len = SLG46826_IMAGE_SIZE - filep->f_pos;
  if (len > buflen)
    {
      len = buflen;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = slg46826_writeregs(priv, filep->f_pos,
                           (FAR const uint8_t *)buffer, len);
  nxmutex_unlock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  filep->f_pos += len;
  return len;
}

/****************************************************************************
 * Name: slg46826_seek
 ****************************************************************************/

static off_t slg46826_seek(FAR struct file *filep, off_t offset, int whence)
{
  off_t pos;

  switch (whence)
    {
      case SEEK_SET:
        pos = offset;
        break;

      case SEEK_CUR:
        pos = filep->f_pos + offset;
        break;

      case SEEK_END:
        pos = SLG46826_IMAGE_SIZE + offset;
        break;

      default:
        return -EINVAL;
    }

  if (pos < 0 || pos > SLG46826_IMAGE_SIZE)
    {
      return -EINVAL;
    }

  filep->f_pos = pos;
  return pos;
}

/****************************************************************************
 * Name: slg46826_ioctl
 ****************************************************************************/

static int slg46826_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct slg46826_dev_s *priv = filep->f_inode->i_private;
  FAR uint8_t *buf = (FAR uint8_t *)(uintptr_t)arg;
  int ret;

  if (cmd != SLG46826IOC_RESET && buf == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case SLG46826IOC_RAMLOAD:
        ret = slg46826_ramload(priv, buf);
        break;

      case SLG46826IOC_NVMREAD:
        ret = slg46826_nvmread(priv, buf);
        break;

      case SLG46826IOC_NVMPROGRAM:
        ret = slg46826_nvmprogram(priv, buf);
        break;

      case SLG46826IOC_RESET:
        ret = slg46826_reset(priv);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: slg46826_register
 ****************************************************************************/

int slg46826_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      FAR const struct slg46826_config_s *config)
{
  FAR struct slg46826_dev_s *priv;
  int ret;

  DEBUGASSERT(i2c != NULL && config != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = config->addr;
  nxmutex_init(&priv->lock);

  ret = slg46826_probe(priv);
  if (ret < 0)
    {
      _err("ERROR: no SLG46826 at %02x: %d\n", priv->addr, ret);
      ret = -ENODEV;
      goto errout;
    }

  ret = slg46826_bootstrap(priv, config);
  if (ret < 0)
    {
      _err("ERROR: SLG46826 bootstrap failed: %d\n", ret);
    }

  ret = register_driver(devpath, &g_slg46826_fops, 0666, priv);
  if (ret < 0)
    {
      goto errout;
    }

  return OK;

errout:
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_SLG46826 */
