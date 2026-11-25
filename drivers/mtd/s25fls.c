/****************************************************************************
 * drivers/mtd/s25fls.c
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

/* MTD driver for the Cypress (Spansion, now Infineon) S25FL128S QSPI NOR
 * flash, on a NuttX QSPI device.
 *
 * The part comes in two sector layouts, told apart by the fifth RDID
 * byte: uniform 256 KB sectors with 512-byte pages, or 64 KB sectors with
 * 256-byte pages and thirty-two 4 KB parameter sectors in place of the
 * first (or, with CR1.TBPARM set, the last) two 64 KB sectors.  The MTD
 * erase block is the large sector; a block in the parameter region is
 * erased as sixteen 4 KB sectors.
 *
 * Reads use the quad output read (6Bh) when CR1.QUAD is set, else FAST
 * READ (0Bh), both with 8 dummy cycles, so CR1's latency code must be 00
 * (the default).  Pages are programmed with PP (02h) on one line.  The
 * driver never writes the status or configuration registers: QUAD,
 * TBPARM and TBPROT are non-volatile (the last two one-time), and a boot
 * ROM may rely on them.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/debug.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/sched.h>
#include <nuttx/spi/qspi.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_S25FLS_QSPIMODE
#  define CONFIG_S25FLS_QSPIMODE QSPIDEV_MODE0
#endif

#ifndef CONFIG_S25FLS_QSPI_FREQUENCY
#  define CONFIG_S25FLS_QSPI_FREQUENCY 80000000
#endif

/* Commands (3-byte addresses) */

#define S25FLS_RDID          0x9f  /* Read ID and CFI */
#define S25FLS_RDSR1         0x05  /* Read status register 1 */
#define S25FLS_RDCR          0x35  /* Read configuration register 1 */
#define S25FLS_WREN          0x06  /* Write enable */
#define S25FLS_CLSR          0x30  /* Clear status register errors */
#define S25FLS_PP            0x02  /* Page program */
#define S25FLS_SE            0xd8  /* Erase a 64 KB or 256 KB sector */
#define S25FLS_P4E           0x20  /* Erase a 4 KB parameter sector */
#define S25FLS_BE            0x60  /* Erase the whole chip */
#define S25FLS_FAST_READ     0x0b  /* 8 dummy cycles with LC 00 */
#define S25FLS_QOR           0x6b  /* Quad output read, 8 dummies */

#define S25FLS_ADDRLEN       3
#define S25FLS_DUMMIES       8

/* Status register 1 */

#define S25FLS_SR1_WIP       (1 << 0)  /* Write (program/erase) busy */
#define S25FLS_SR1_WEL       (1 << 1)  /* Write enabled */
#define S25FLS_SR1_BP_MASK   (7 << 2)  /* Block protection */
#define S25FLS_SR1_E_ERR     (1 << 5)  /* Erase failed */
#define S25FLS_SR1_P_ERR     (1 << 6)  /* Program failed */

/* Configuration register 1 */

#define S25FLS_CR1_QUAD      (1 << 1)  /* Quad data lines enabled */
#define S25FLS_CR1_TBPARM    (1 << 2)  /* Parameter sectors at the top */
#define S25FLS_CR1_LC_MASK   (3 << 6)  /* Latency code */

/* RDID: manufacturer, device ID, ID-CFI length, sector architecture,
 * family, ...  CFI byte 0x2a holds log2 of the page buffer size.
 */

#define S25FLS_RDID_LEN      0x2b
#define S25FLS_MANUFACTURER  0x01
#define S25FLS_DEVID_128MB   0x2018    /* S25FL128S (also S25FL127S) */
#define S25FLS_FAMILY_FLS    0x80
#define S25FLS_ARCH_256K     0x00      /* uniform 256 KB sectors */
#define S25FLS_ARCH_64K      0x01      /* 64 KB and 4 KB sectors */
#define S25FLS_CFI_PAGESHIFT 0x2a

#define S25FLS_SIZE          (16 * 1024 * 1024)
#define S25FLS_P4E_SIZE      4096
#define S25FLS_PARAM_SIZE    (32 * S25FLS_P4E_SIZE)

/* Status polling: how long to wait (us) and how often (us).  The data
 * sheet maximums are 750 us for a page, 650 ms for a 64 KB sector,
 * 2.6 s for a 256 KB sector and 165 s for the chip.
 */

#define S25FLS_PP_TIMEOUT    5000
#define S25FLS_PP_POLL       50
#define S25FLS_SE_TIMEOUT    5000000
#define S25FLS_SE_POLL       1000
#define S25FLS_BE_TIMEOUT    330000000
#define S25FLS_BE_POLL       100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct s25fls_dev_s
{
  struct mtd_dev_s       mtd;          /* MTD interface; must be first */
  FAR struct qspi_dev_s *qspi;         /* QSPI device */
  FAR uint8_t           *cmdbuf;       /* QSPI_ALLOC'd, for registers */
  uint32_t               frequency;    /* SCLK */
  uint32_t               paramstart;   /* Parameter region, if any */
  uint32_t               paramend;
  uint16_t               nsectors;     /* Erase blocks */
  uint8_t                sectorshift;  /* log2 erase block size */
  uint8_t                pageshift;    /* log2 page size */
  uint8_t                readcmd;      /* QOR or FAST_READ */
  uint16_t               readflags;    /* QSPIMEM_* for readcmd */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     s25fls_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks);
static ssize_t s25fls_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buf);
static ssize_t s25fls_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf);
static ssize_t s25fls_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t s25fls_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buffer);
#endif
static int     s25fls_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s25fls_lock and s25fls_unlock
 *
 * Description:
 *   Take the QSPI bus and set it up for the flash: another device may
 *   have left it in another mode.
 *
 ****************************************************************************/

static void s25fls_lock(FAR struct s25fls_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, true);
  QSPI_SETMODE(priv->qspi, CONFIG_S25FLS_QSPIMODE);
  QSPI_SETBITS(priv->qspi, 8);
  QSPI_SETFREQUENCY(priv->qspi, priv->frequency);
}

static void s25fls_unlock(FAR struct s25fls_dev_s *priv)
{
  QSPI_LOCK(priv->qspi, false);
}

/****************************************************************************
 * Name: s25fls_command
 *
 * Description:
 *   A command with an optional address and optional data read into
 *   priv->cmdbuf.
 *
 ****************************************************************************/

static int s25fls_command(FAR struct s25fls_dev_s *priv, uint8_t cmd,
                          bool address, uint32_t addr, size_t nread)
{
  struct qspi_cmdinfo_s cmdinfo;

  memset(&cmdinfo, 0, sizeof(cmdinfo));
  cmdinfo.cmd = cmd;

  if (address)
    {
      cmdinfo.flags  |= QSPICMD_ADDRESS;
      cmdinfo.addrlen = S25FLS_ADDRLEN;
      cmdinfo.addr    = addr;
    }

  if (nread > 0)
    {
      cmdinfo.flags  |= QSPICMD_READDATA;
      cmdinfo.buflen  = nread;
      cmdinfo.buffer  = priv->cmdbuf;
    }

  return QSPI_COMMAND(priv->qspi, &cmdinfo);
}

/****************************************************************************
 * Name: s25fls_readreg
 ****************************************************************************/

static int s25fls_readreg(FAR struct s25fls_dev_s *priv, uint8_t cmd)
{
  int ret = s25fls_command(priv, cmd, false, 0, 1);

  return ret < 0 ? ret : priv->cmdbuf[0];
}

/****************************************************************************
 * Name: s25fls_waitready
 *
 * Description:
 *   Wait for a program or erase to finish, polling SR1 every poll us for
 *   up to timeout us.  A failed program or erase is cleared with CLSR
 *   (it leaves WIP set) and reported as -EIO.
 *
 ****************************************************************************/

static int s25fls_waitready(FAR struct s25fls_dev_s *priv,
                            uint32_t timeout, uint32_t poll)
{
  uint32_t waited = 0;
  int sr1;

  for (; ; )
    {
      sr1 = s25fls_readreg(priv, S25FLS_RDSR1);
      if (sr1 < 0)
        {
          return sr1;
        }

      if ((sr1 & (S25FLS_SR1_E_ERR | S25FLS_SR1_P_ERR)) != 0)
        {
          ferr("ERROR: %s failed, SR1 %02x\n",
               (sr1 & S25FLS_SR1_E_ERR) != 0 ? "erase" : "program", sr1);
          s25fls_command(priv, S25FLS_CLSR, false, 0, 0);
          return -EIO;
        }

      if ((sr1 & S25FLS_SR1_WIP) == 0)
        {
          return OK;
        }

      if (waited >= timeout)
        {
          ferr("ERROR: still busy after %" PRIu32 " us, SR1 %02x\n",
               waited, sr1);
          return -ETIMEDOUT;
        }

      nxsched_usleep(poll);
      waited += poll;
    }
}

/****************************************************************************
 * Name: s25fls_writeenable
 ****************************************************************************/

static int s25fls_writeenable(FAR struct s25fls_dev_s *priv)
{
  return s25fls_command(priv, S25FLS_WREN, false, 0, 0);
}

/****************************************************************************
 * Name: s25fls_erasecmd
 *
 * Description:
 *   Write-enable, send an erase command, wait for it to finish.
 *
 ****************************************************************************/

static int s25fls_erasecmd(FAR struct s25fls_dev_s *priv, uint8_t cmd,
                           bool address, uint32_t addr, uint32_t timeout,
                           uint32_t poll)
{
  int ret;

  ret = s25fls_writeenable(priv);
  if (ret == OK)
    {
      ret = s25fls_command(priv, cmd, address, addr, 0);
    }

  if (ret == OK)
    {
      ret = s25fls_waitready(priv, timeout, poll);
    }

  return ret;
}

/****************************************************************************
 * Name: s25fls_erasesector
 *
 * Description:
 *   Erase one MTD erase block: one SE, or sixteen P4E in the parameter
 *   region.
 *
 ****************************************************************************/

static int s25fls_erasesector(FAR struct s25fls_dev_s *priv,
                              off_t sector)
{
  uint32_t addr = (uint32_t)sector << priv->sectorshift;
  uint32_t end  = addr + (1 << priv->sectorshift);
  int ret = OK;

  finfo("sector %jd at %08" PRIx32 "\n", (intmax_t)sector, addr);

  if (addr >= priv->paramstart && addr < priv->paramend)
    {
      for (; addr < end && ret == OK; addr += S25FLS_P4E_SIZE)
        {
          ret = s25fls_erasecmd(priv, S25FLS_P4E, true, addr,
                                S25FLS_SE_TIMEOUT, S25FLS_SE_POLL);
        }

      return ret;
    }

  return s25fls_erasecmd(priv, S25FLS_SE, true, addr, S25FLS_SE_TIMEOUT,
                         S25FLS_SE_POLL);
}

/****************************************************************************
 * Name: s25fls_readbytes
 ****************************************************************************/

static int s25fls_readbytes(FAR struct s25fls_dev_s *priv,
                            FAR uint8_t *buffer, uint32_t addr,
                            size_t nbytes)
{
  struct qspi_meminfo_s meminfo;

  memset(&meminfo, 0, sizeof(meminfo));
  meminfo.flags   = priv->readflags;
  meminfo.addrlen = S25FLS_ADDRLEN;
  meminfo.dummies = S25FLS_DUMMIES;
  meminfo.cmd     = priv->readcmd;
  meminfo.addr    = addr;
  meminfo.buflen  = nbytes;
  meminfo.buffer  = buffer;

  return QSPI_MEMORY(priv->qspi, &meminfo);
}

/****************************************************************************
 * Name: s25fls_program
 *
 * Description:
 *   Program nbytes at addr, which must not cross a page boundary.
 *
 ****************************************************************************/

static int s25fls_program(FAR struct s25fls_dev_s *priv,
                          FAR const uint8_t *buffer, uint32_t addr,
                          size_t nbytes)
{
  struct qspi_meminfo_s meminfo;
  int ret;

  ret = s25fls_writeenable(priv);
  if (ret < 0)
    {
      return ret;
    }

  memset(&meminfo, 0, sizeof(meminfo));
  meminfo.flags   = QSPIMEM_WRITE;
  meminfo.addrlen = S25FLS_ADDRLEN;
  meminfo.cmd     = S25FLS_PP;
  meminfo.addr    = addr;
  meminfo.buflen  = nbytes;
  meminfo.buffer  = (FAR void *)buffer;

  ret = QSPI_MEMORY(priv->qspi, &meminfo);
  if (ret < 0)
    {
      return ret;
    }

  return s25fls_waitready(priv, S25FLS_PP_TIMEOUT, S25FLS_PP_POLL);
}

/****************************************************************************
 * Name: s25fls_writebytes
 *
 * Description:
 *   Program nbytes at addr, a page at a time.
 *
 ****************************************************************************/

static int s25fls_writebytes(FAR struct s25fls_dev_s *priv,
                             FAR const uint8_t *buffer, uint32_t addr,
                             size_t nbytes)
{
  uint32_t pagesize = 1 << priv->pageshift;
  size_t count;
  int ret = OK;

  while (nbytes > 0 && ret == OK)
    {
      count = pagesize - (addr & (pagesize - 1));
      if (count > nbytes)
        {
          count = nbytes;
        }

      ret = s25fls_program(priv, buffer, addr, count);
      buffer += count;
      addr   += count;
      nbytes -= count;
    }

  return ret;
}

/****************************************************************************
 * Name: s25fls_erase
 ****************************************************************************/

static int s25fls_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks)
{
  FAR struct s25fls_dev_s *priv = (FAR struct s25fls_dev_s *)dev;
  size_t i;
  int ret = OK;

  if (startblock < 0 || startblock + nblocks > priv->nsectors)
    {
      return -EINVAL;
    }

  s25fls_lock(priv);
  for (i = 0; i < nblocks && ret == OK; i++)
    {
      ret = s25fls_erasesector(priv, startblock + i);
    }

  s25fls_unlock(priv);
  return ret < 0 ? ret : (int)nblocks;
}

/****************************************************************************
 * Name: s25fls_bread
 ****************************************************************************/

static ssize_t s25fls_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR uint8_t *buf)
{
  FAR struct s25fls_dev_s *priv = (FAR struct s25fls_dev_s *)dev;
  ssize_t nbytes;

  nbytes = s25fls_read(dev, startblock << priv->pageshift,
                       nblocks << priv->pageshift, buf);
  return nbytes < 0 ? nbytes : nbytes >> priv->pageshift;
}

/****************************************************************************
 * Name: s25fls_bwrite
 ****************************************************************************/

static ssize_t s25fls_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct s25fls_dev_s *priv = (FAR struct s25fls_dev_s *)dev;
  off_t offset = startblock << priv->pageshift;
  size_t nbytes = nblocks << priv->pageshift;
  int ret;

  if (startblock < 0 || offset + nbytes > S25FLS_SIZE)
    {
      return -EINVAL;
    }

  s25fls_lock(priv);
  ret = s25fls_writebytes(priv, buf, offset, nbytes);
  s25fls_unlock(priv);

  return ret < 0 ? ret : (ssize_t)nblocks;
}

/****************************************************************************
 * Name: s25fls_read
 ****************************************************************************/

static ssize_t s25fls_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct s25fls_dev_s *priv = (FAR struct s25fls_dev_s *)dev;
  int ret;

  if (offset < 0 || offset > S25FLS_SIZE)
    {
      return -EINVAL;
    }

  if (nbytes > S25FLS_SIZE - offset)
    {
      nbytes = S25FLS_SIZE - offset;
    }

  s25fls_lock(priv);
  ret = s25fls_readbytes(priv, buffer, offset, nbytes);
  s25fls_unlock(priv);

  return ret < 0 ? ret : (ssize_t)nbytes;
}

/****************************************************************************
 * Name: s25fls_write
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t s25fls_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct s25fls_dev_s *priv = (FAR struct s25fls_dev_s *)dev;
  int ret;

  if (offset < 0 || offset > S25FLS_SIZE || nbytes > S25FLS_SIZE - offset)
    {
      return -EINVAL;
    }

  s25fls_lock(priv);
  ret = s25fls_writebytes(priv, buffer, offset, nbytes);
  s25fls_unlock(priv);

  return ret < 0 ? ret : (ssize_t)nbytes;
}
#endif

/****************************************************************************
 * Name: s25fls_ioctl
 ****************************************************************************/

static int s25fls_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg)
{
  FAR struct s25fls_dev_s *priv = (FAR struct s25fls_dev_s *)dev;
  int ret = -EINVAL;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo != NULL)
            {
              memset(geo, 0, sizeof(*geo));
              geo->blocksize    = 1 << priv->pageshift;
              geo->erasesize    = 1 << priv->sectorshift;
              geo->neraseblocks = priv->nsectors;
              strlcpy(geo->model, "S25FL128S", sizeof(geo->model));
              ret = OK;
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;

          if (info != NULL)
            {
              info->numsectors  = S25FLS_SIZE >> priv->pageshift;
              info->sectorsize  = 1 << priv->pageshift;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret = OK;
            }
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;

          if (result != NULL)
            {
              *result = 0xff;
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          s25fls_lock(priv);
          ret = s25fls_erasecmd(priv, S25FLS_BE, false, 0,
                                S25FLS_BE_TIMEOUT, S25FLS_BE_POLL);
          s25fls_unlock(priv);
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: s25fls_identify
 *
 * Description:
 *   Read the ID, CFI and CR1 and set up the geometry and read command.
 *
 ****************************************************************************/

static int s25fls_identify(FAR struct s25fls_dev_s *priv)
{
  struct qspi_cmdinfo_s cmdinfo;
  FAR uint8_t *id;
  uint16_t devid;
  int sr1;
  int cr1;
  int ret;

  id = QSPI_ALLOC(priv->qspi, S25FLS_RDID_LEN);
  if (id == NULL)
    {
      return -ENOMEM;
    }

  memset(id, 0, S25FLS_RDID_LEN);
  memset(&cmdinfo, 0, sizeof(cmdinfo));
  cmdinfo.flags  = QSPICMD_READDATA;
  cmdinfo.cmd    = S25FLS_RDID;
  cmdinfo.buflen = S25FLS_RDID_LEN;
  cmdinfo.buffer = id;

  ret = QSPI_COMMAND(priv->qspi, &cmdinfo);
  if (ret < 0)
    {
      goto out;
    }

  devid = ((uint16_t)id[1] << 8) | id[2];
  if (id[0] != S25FLS_MANUFACTURER || devid != S25FLS_DEVID_128MB ||
      id[5] != S25FLS_FAMILY_FLS)
    {
      ferr("ERROR: unsupported part: %02x %04x family %02x\n",
           id[0], devid, id[5]);
      ret = -ENODEV;
      goto out;
    }

  priv->pageshift = id[S25FLS_CFI_PAGESHIFT];
  if (priv->pageshift != 8 && priv->pageshift != 9)
    {
      ferr("ERROR: page size 2^%d\n", priv->pageshift);
      ret = -ENODEV;
      goto out;
    }

  cr1 = s25fls_readreg(priv, S25FLS_RDCR);
  sr1 = s25fls_readreg(priv, S25FLS_RDSR1);
  if (cr1 < 0 || sr1 < 0)
    {
      ret = cr1 < 0 ? cr1 : sr1;
      goto out;
    }

  if (id[4] == S25FLS_ARCH_256K)
    {
      priv->sectorshift = 18;
      priv->paramstart  = 0;
      priv->paramend    = 0;
    }
  else if (id[4] == S25FLS_ARCH_64K)
    {
      priv->sectorshift = 16;
      priv->paramstart  = (cr1 & S25FLS_CR1_TBPARM) != 0 ?
                          S25FLS_SIZE - S25FLS_PARAM_SIZE : 0;
      priv->paramend    = priv->paramstart + S25FLS_PARAM_SIZE;
    }
  else
    {
      ferr("ERROR: sector architecture %02x\n", id[4]);
      ret = -ENODEV;
      goto out;
    }

  priv->nsectors = S25FLS_SIZE >> priv->sectorshift;

  if ((cr1 & S25FLS_CR1_LC_MASK) != 0)
    {
      ferr("ERROR: CR1 %02x: latency code is not 00\n", cr1);
      ret = -ENODEV;
      goto out;
    }

  if ((cr1 & S25FLS_CR1_QUAD) != 0)
    {
      priv->readcmd   = S25FLS_QOR;
      priv->readflags = QSPIMEM_READ | QSPIMEM_QUADDATA;
    }
  else
    {
      priv->readcmd   = S25FLS_FAST_READ;
      priv->readflags = QSPIMEM_READ;
    }

  if ((sr1 & S25FLS_SR1_BP_MASK) != 0)
    {
      fwarn("WARNING: SR1 %02x: block protection is set\n", sr1);
    }

  finfo("S25FL128S: %d x %d KB sectors, %d-byte pages, CR1 %02x, "
        "SR1 %02x\n", priv->nsectors, 1 << (priv->sectorshift - 10),
        1 << priv->pageshift, cr1, sr1);
  ret = OK;

out:
  QSPI_FREE(priv->qspi, id);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s25fls_initialize
 *
 * Description:
 *   Create an MTD device for an S25FL128S on a QSPI device, or return
 *   NULL if the part is not one.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *s25fls_initialize(FAR struct qspi_dev_s *qspi)
{
  FAR struct s25fls_dev_s *priv;
  int ret;

  DEBUGASSERT(qspi != NULL);

  priv = kmm_zalloc(sizeof(struct s25fls_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->mtd.erase  = s25fls_erase;
  priv->mtd.bread  = s25fls_bread;
  priv->mtd.bwrite = s25fls_bwrite;
  priv->mtd.read   = s25fls_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = s25fls_write;
#endif
  priv->mtd.ioctl  = s25fls_ioctl;
  priv->mtd.name   = "s25fls";
  priv->qspi       = qspi;
  priv->frequency  = CONFIG_S25FLS_QSPI_FREQUENCY;

  priv->cmdbuf = QSPI_ALLOC(qspi, 4);
  if (priv->cmdbuf == NULL)
    {
      goto errout;
    }

  s25fls_lock(priv);

  ret = s25fls_identify(priv);
  if (ret == OK)
    {
      /* Wait out a program or erase that a reset interrupted.  An error
       * left from before is cleared (CLSR) and not held against us.
       */

      ret = s25fls_waitready(priv, S25FLS_SE_TIMEOUT, S25FLS_SE_POLL);
      if (ret == -EIO)
        {
          ret = OK;
        }
    }

  s25fls_unlock(priv);

  if (ret < 0)
    {
      ferr("ERROR: S25FL-S not usable: %d\n", ret);
      QSPI_FREE(qspi, priv->cmdbuf);
      goto errout;
    }

  return &priv->mtd;

errout:
  kmm_free(priv);
  return NULL;
}
