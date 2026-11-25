/****************************************************************************
 * arch/arm/src/am263x/am263x_qspi.c
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

/* QSPI0 controller driver, as a NuttX qspi_dev_s, polled, chip select 0.
 *
 * The controller has two ports.  On the configuration port software
 * shifts one word (1 to 128 bits) per write of SPI_CMD_REG; CS stays
 * asserted until the frame length given in that register has gone, so a
 * flash command is a frame of a command word, an address word and data
 * words.  The configuration port only moves data on one line.
 *
 * On the memory-mapped port the controller turns CPU reads of the flash
 * window at 0x60000000 into read commands (SPI_SETUP0_REG), on one, two
 * or four data lines.  It drives only 23 address bits, so the window
 * reaches the first 8 MB of the flash and aliases above that.
 * QSPI_MEMORY reads of the first 8 MB use it; reads above it, writes and
 * QSPI_COMMAND use the configuration port, whose reads can also use four
 * data lines.  The window must be Normal memory in the MPU (memcpy()
 * makes unaligned accesses).
 *
 * The window is mapped non-cacheable: the memory-mapped port does not
 * take cache line fills.  So nothing goes stale when the flash changes.
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
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/qspi.h>

#include "arm_internal.h"
#include "am263x_clockconfig.h"
#include "am263x_qspi.h"
#include "hardware/am263x_qspi.h"

#ifdef CONFIG_AM263X_QSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The chip select the driver uses: the boot flash */

#define QSPI_CS               0

/* Polls of SPI_STATUS_REG before a word counts as stuck: far longer than
 * a 32-bit word at the slowest clock (80 MHz / 65536, about 26 ms).
 */

#define QSPI_POLL_TIMEOUT     10000000

/* Bytes per configuration-port read frame: 1024 128-bit words, well
 * inside SPI_CMD_REG.FLEN.
 */

#define QSPI_CFGREAD_CHUNK    16384

/* Initial SCLK, until the client sets one */

#define QSPI_DEFAULT_FREQUENCY 10000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_qspidev_s
{
  struct qspi_dev_s qspi;      /* Externally visible part; must be first */
  uintptr_t         base;      /* QSPI register base */
  mutex_t           lock;      /* Held by the bus user (QSPI_LOCK) */
  uint32_t          frequency; /* Requested SCLK */
  uint32_t          actual;    /* SCLK the divider gives */
  uint32_t          setup;     /* SPI_SETUP0_REG last programmed */
  uint8_t           mode;      /* QSPIDEV_MODEn */
  bool              initialized;
};

/* One configuration-port frame being shifted */

struct qspi_frame_s
{
  uint32_t cmdreg;             /* SPI_CMD_REG without CMD and WLEN */
  uint32_t nwords;             /* Words in the frame */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      qspi_lock(FAR struct qspi_dev_s *dev, bool lock);
static uint32_t qspi_setfrequency(FAR struct qspi_dev_s *dev,
                                  uint32_t frequency);
static void     qspi_setmode(FAR struct qspi_dev_s *dev,
                             enum qspi_mode_e mode);
static void     qspi_setbits(FAR struct qspi_dev_s *dev, int nbits);
static int      qspi_command(FAR struct qspi_dev_s *dev,
                             FAR struct qspi_cmdinfo_s *cmdinfo);
static int      qspi_memory(FAR struct qspi_dev_s *dev,
                            FAR struct qspi_meminfo_s *meminfo);
static FAR void *qspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen);
static void     qspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct qspi_ops_s g_qspi0ops =
{
  .lock         = qspi_lock,
  .setfrequency = qspi_setfrequency,
  .setmode      = qspi_setmode,
  .setbits      = qspi_setbits,
#ifdef CONFIG_QSPI_HWFEATURES
  .hwfeatures   = NULL,
#endif
  .command      = qspi_command,
  .memory       = qspi_memory,
  .alloc        = qspi_alloc,
  .free         = qspi_free,
};

static struct am263x_qspidev_s g_qspi0dev =
{
  .qspi =
    {
      &g_qspi0ops
    },
  .base = AM263X_QSPI0_BASE,
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t qspi_getreg(FAR struct am263x_qspidev_s *priv,
                                   uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void qspi_putreg(FAR struct am263x_qspidev_s *priv,
                               uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: qspi_waitidle
 *
 * Description:
 *   Wait for the controller to finish the word it is shifting.  Returns
 *   false on a timeout.
 *
 ****************************************************************************/

static bool qspi_waitidle(FAR struct am263x_qspidev_s *priv)
{
  uint32_t count;

  for (count = 0; count < QSPI_POLL_TIMEOUT; count++)
    {
      if ((qspi_getreg(priv, AM263X_QSPI_STATUS_OFFSET) &
           QSPI_STATUS_BUSY) == 0)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: qspi_cfgport
 *
 * Description:
 *   Select the configuration port (false) or the memory-mapped port
 *   (true).
 *
 ****************************************************************************/

static void qspi_cfgport(FAR struct am263x_qspidev_s *priv, bool mmap)
{
  qspi_putreg(priv, AM263X_QSPI_SWITCH_OFFSET,
              mmap ? QSPI_SWITCH_MMPT_S : 0);
}

/****************************************************************************
 * Name: qspi_word
 *
 * Description:
 *   Shift one word of the current frame: nbits (1-32) bits, MSB first.
 *   cmd is QSPI_CMD_WRITE_SINGLE (txwd goes out) or QSPI_CMD_READ_SINGLE
 *   (the word read comes back in *rxwd).
 *
 ****************************************************************************/

static int qspi_word(FAR struct am263x_qspidev_s *priv,
                     FAR const struct qspi_frame_s *frame, uint32_t cmd,
                     int nbits, uint32_t txwd, FAR uint32_t *rxwd)
{
  if (cmd == QSPI_CMD_WRITE_SINGLE)
    {
      qspi_putreg(priv, AM263X_QSPI_DATA_OFFSET, txwd);
    }

  qspi_putreg(priv, AM263X_QSPI_CMD_OFFSET,
              frame->cmdreg | cmd | QSPI_CMD_WLEN(nbits));

  if (!qspi_waitidle(priv))
    {
      spierr("ERROR: QSPI word timed out, status %08" PRIx32 "\n",
             qspi_getreg(priv, AM263X_QSPI_STATUS_OFFSET));
      return -ETIMEDOUT;
    }

  if (rxwd != NULL)
    {
      *rxwd = qspi_getreg(priv, AM263X_QSPI_DATA_OFFSET);
    }

  return OK;
}

/****************************************************************************
 * Name: qspi_datawords
 *
 * Description:
 *   The number of words buflen bytes of data take: 32-bit words written or
 *   128-bit words read, then single bytes for the rest.
 *
 ****************************************************************************/

static inline uint32_t qspi_datawords(size_t buflen, bool write)
{
  return write ? (buflen >> 2) + (buflen & 3) :
                 (buflen >> 4) + (buflen & 15);
}

/****************************************************************************
 * Name: qspi_txdata
 *
 * Description:
 *   Shift buflen bytes out in the current frame, four at a time.
 *
 ****************************************************************************/

static int qspi_txdata(FAR struct am263x_qspidev_s *priv,
                       FAR const struct qspi_frame_s *frame,
                       FAR const uint8_t *buf, size_t buflen)
{
  uint32_t wd;
  int ret = OK;

  while (buflen >= 4 && ret == OK)
    {
      wd = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | buf[3];
      ret = qspi_word(priv, frame, QSPI_CMD_WRITE_SINGLE, 32, wd, NULL);
      buf += 4;
      buflen -= 4;
    }

  while (buflen > 0 && ret == OK)
    {
      ret = qspi_word(priv, frame, QSPI_CMD_WRITE_SINGLE, 8, *buf++, NULL);
      buflen--;
    }

  return ret;
}

/****************************************************************************
 * Name: qspi_rxdata
 *
 * Description:
 *   Shift buflen bytes in in the current frame, sixteen at a time, on one
 *   line (QSPI_CMD_READ_SINGLE) or four (QSPI_CMD_READ_QUAD).  A 128-bit
 *   word fills all four data registers, first bits in DATA3's MSBs.
 *
 ****************************************************************************/

static void qspi_rxbytes(FAR uint8_t *buf, uint32_t wd)
{
  buf[0] = wd >> 24;
  buf[1] = wd >> 16;
  buf[2] = wd >> 8;
  buf[3] = wd;
}

static int qspi_rxdata(FAR struct am263x_qspidev_s *priv,
                       FAR const struct qspi_frame_s *frame, uint32_t cmd,
                       FAR uint8_t *buf, size_t buflen)
{
  uint32_t wd;
  int ret = OK;

  while (buflen >= 16 && ret == OK)
    {
      ret = qspi_word(priv, frame, cmd, 128, 0, NULL);
      qspi_rxbytes(&buf[0], qspi_getreg(priv, AM263X_QSPI_DATA3_OFFSET));
      qspi_rxbytes(&buf[4], qspi_getreg(priv, AM263X_QSPI_DATA2_OFFSET));
      qspi_rxbytes(&buf[8], qspi_getreg(priv, AM263X_QSPI_DATA1_OFFSET));
      qspi_rxbytes(&buf[12], qspi_getreg(priv, AM263X_QSPI_DATA_OFFSET));
      buf += 16;
      buflen -= 16;
    }

  while (buflen > 0 && ret == OK)
    {
      ret = qspi_word(priv, frame, cmd, 8, 0, &wd);
      *buf++ = wd;
      buflen--;
    }

  return ret;
}

/****************************************************************************
 * Name: qspi_cfgxfer
 *
 * Description:
 *   One frame on the configuration port: an 8-bit command, an optional
 *   address of addrlen bytes, dummies dummy cycles (a word of its own),
 *   then buflen bytes written or read.  Reads come in on one line, or on
 *   four with quad set; everything else goes out on one.
 *
 ****************************************************************************/

static int qspi_cfgxfer(FAR struct am263x_qspidev_s *priv, uint8_t cmd,
                        bool address, uint8_t addrlen, uint32_t addr,
                        uint8_t dummies, bool write, bool quad,
                        FAR void *buffer, size_t buflen)
{
  struct qspi_frame_s frame;
  int ret;

  if ((address && (addrlen < 1 || addrlen > 4)) || dummies > 32)
    {
      return -EINVAL;
    }

  frame.nwords = 1 + (address ? 1 : 0) + (dummies > 0 ? 1 : 0) +
                 qspi_datawords(buflen, write);
  if (frame.nwords > QSPI_CMD_FLEN_MAX)
    {
      return -EINVAL;
    }

  frame.cmdreg = QSPI_CMD_FLEN(frame.nwords) | QSPI_CMD_CSNUM(QSPI_CS);

  qspi_cfgport(priv, false);

  ret = qspi_word(priv, &frame, QSPI_CMD_WRITE_SINGLE, 8, cmd, NULL);
  if (ret == OK && address)
    {
      ret = qspi_word(priv, &frame, QSPI_CMD_WRITE_SINGLE, 8 * addrlen,
                      addr, NULL);
    }

  if (ret == OK && dummies > 0)
    {
      ret = qspi_word(priv, &frame, QSPI_CMD_WRITE_SINGLE, dummies, 0,
                      NULL);
    }

  if (ret == OK && buflen > 0)
    {
      if (write)
        {
          ret = qspi_txdata(priv, &frame, buffer, buflen);
        }
      else
        {
          ret = qspi_rxdata(priv, &frame,
                            quad ? QSPI_CMD_READ_QUAD :
                                   QSPI_CMD_READ_SINGLE,
                            buffer, buflen);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: qspi_copy
 *
 * Description:
 *   Copy out of the flash window.  Each bus access to the window is a
 *   flash read command of its own, so read it 32 bytes at a time with LDM
 *   (one burst) rather than with memcpy(), which loads 8 bytes at a time.
 *
 ****************************************************************************/

static void qspi_copy(FAR uint8_t *dst, uintptr_t src, size_t len)
{
  uint32_t tmp[8];

  /* Bytes up to a 32-byte boundary of the window */

  while ((src & 31) != 0 && len > 0)
    {
      *dst++ = *(FAR volatile const uint8_t *)src++;
      len--;
    }

  while (len >= 32)
    {
      __asm__ __volatile__
      (
        "ldmia %[s]!, {r2, r3, r4, r5, r6, r8, r10, r12}\n\t"
        "stmia %[t], {r2, r3, r4, r5, r6, r8, r10, r12}\n\t"
        : [s] "+r" (src)
        : [t] "r" (tmp)
        : "r2", "r3", "r4", "r5", "r6", "r8", "r10", "r12", "memory"
      );

      memcpy(dst, tmp, 32);
      dst += 32;
      len -= 32;
    }

  memcpy(dst, (FAR const void *)src, len);
}

/****************************************************************************
 * Name: qspi_mmapread
 *
 * Description:
 *   Read len bytes at addr (all inside the flash window) through the
 *   memory-mapped port: program SPI_SETUP0_REG with the read command and
 *   copy out of the window.
 *
 ****************************************************************************/

static int qspi_mmapread(FAR struct am263x_qspidev_s *priv,
                         FAR const struct qspi_meminfo_s *meminfo,
                         uint32_t addr, FAR uint8_t *buf, size_t len)
{
  uint32_t setup;

  setup = QSPI_SETUP_RCMD(meminfo->cmd) |
          QSPI_SETUP_NUM_A_BYTES(meminfo->addrlen) |
          (QSPIMEM_ISQUADDATA(meminfo->flags) ? QSPI_SETUP_READ_QUAD :
                                                QSPI_SETUP_READ_NORMAL);

  /* Dummy cycles: whole bytes (up to three), or up to seven bits */

  if ((meminfo->dummies & 7) == 0 && meminfo->dummies <= 24)
    {
      setup |= QSPI_SETUP_NUM_D_BYTES(meminfo->dummies >> 3);
    }
  else if (meminfo->dummies < 8)
    {
      setup |= QSPI_SETUP_NUM_D_BITS(meminfo->dummies);
    }
  else
    {
      return -EINVAL;
    }

  if (!qspi_waitidle(priv))
    {
      return -ETIMEDOUT;
    }

  if (setup != priv->setup)
    {
      qspi_putreg(priv, AM263X_QSPI_SETUP_OFFSET(QSPI_CS), setup);
      priv->setup = setup;
    }

  qspi_cfgport(priv, true);
  UP_DSB();

  qspi_copy(buf, AM263X_QSPI_FLASH_BASE + addr, len);

  UP_DSB();
  qspi_cfgport(priv, false);
  return OK;
}

/****************************************************************************
 * Name: qspi_cfgread
 *
 * Description:
 *   Read len bytes at addr through the configuration port, for flash
 *   beyond the window, in frames that fit in SPI_CMD_REG.FLEN.
 *
 ****************************************************************************/

static int qspi_cfgread(FAR struct am263x_qspidev_s *priv,
                        FAR const struct qspi_meminfo_s *meminfo,
                        uint32_t addr, FAR uint8_t *buf, size_t len)
{
  size_t chunk;
  int ret = OK;

  while (len > 0 && ret == OK)
    {
      chunk = len < QSPI_CFGREAD_CHUNK ? len : QSPI_CFGREAD_CHUNK;
      ret = qspi_cfgxfer(priv, meminfo->cmd, true, meminfo->addrlen, addr,
                         meminfo->dummies, false,
                         QSPIMEM_ISQUADDATA(meminfo->flags), buf, chunk);
      addr += chunk;
      buf  += chunk;
      len  -= chunk;
    }

  return ret;
}

/****************************************************************************
 * Name: qspi_lock
 ****************************************************************************/

static int qspi_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  FAR struct am263x_qspidev_s *priv = (FAR struct am263x_qspidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: qspi_setfrequency
 *
 * Description:
 *   SCLK is the 80 MHz functional clock divided by 1 to 65536: the
 *   fastest rate not above the one asked for.
 *
 ****************************************************************************/

static uint32_t qspi_setfrequency(FAR struct qspi_dev_s *dev,
                                  uint32_t frequency)
{
  FAR struct am263x_qspidev_s *priv = (FAR struct am263x_qspidev_s *)dev;
  uint32_t div;

  if (frequency == 0)
    {
      frequency = 1;
    }

  if (priv->initialized && frequency == priv->frequency)
    {
      return priv->actual;
    }

  div = (AM263X_QSPI_FREQUENCY + frequency - 1) / frequency;
  if (div < 1)
    {
      div = 1;
    }
  else if (div > 65536)
    {
      div = 65536;
    }

  qspi_waitidle(priv);

  /* Stop SCLK while the divider changes */

  qspi_putreg(priv, AM263X_QSPI_CLKCTRL_OFFSET, QSPI_CLKCTRL_DIV(div - 1));
  qspi_putreg(priv, AM263X_QSPI_CLKCTRL_OFFSET,
              QSPI_CLKCTRL_DIV(div - 1) | QSPI_CLKCTRL_CLKEN);

  priv->frequency = frequency;
  priv->actual    = AM263X_QSPI_FREQUENCY / div;

  spiinfo("QSPI frequency %" PRIu32 " -> %" PRIu32 "\n",
          frequency, priv->actual);
  return priv->actual;
}

/****************************************************************************
 * Name: qspi_setmode
 ****************************************************************************/

static void qspi_setmode(FAR struct qspi_dev_s *dev, enum qspi_mode_e mode)
{
  FAR struct am263x_qspidev_s *priv = (FAR struct am263x_qspidev_s *)dev;
  uint32_t dc;
  uint32_t csdc = 0;

  if (priv->initialized && mode == priv->mode)
    {
      return;
    }

  if (mode == QSPIDEV_MODE2 || mode == QSPIDEV_MODE3)
    {
      csdc |= QSPI_DC_CKP;
    }

  if (mode == QSPIDEV_MODE1 || mode == QSPIDEV_MODE3)
    {
      csdc |= QSPI_DC_CKPH;
    }

  /* CS active low, no CS-to-data delay */

  qspi_waitidle(priv);
  dc  = qspi_getreg(priv, AM263X_QSPI_DC_OFFSET);
  dc &= ~(QSPI_DC_MASK << QSPI_DC_SHIFT(QSPI_CS));
  dc |= csdc << QSPI_DC_SHIFT(QSPI_CS);
  qspi_putreg(priv, AM263X_QSPI_DC_OFFSET, dc);

  priv->mode = mode;
}

/****************************************************************************
 * Name: qspi_setbits
 *
 * Description:
 *   Only 8-bit words are supported: the driver picks its own word sizes
 *   on the wire.
 *
 ****************************************************************************/

static void qspi_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
  if (nbits != 8)
    {
      spierr("ERROR: %d-bit words not supported\n", nbits);
    }
}

/****************************************************************************
 * Name: qspi_command
 *
 * Description:
 *   A command with an optional address and data, all on one line.
 *
 ****************************************************************************/

static int qspi_command(FAR struct qspi_dev_s *dev,
                        FAR struct qspi_cmdinfo_s *cmdinfo)
{
  FAR struct am263x_qspidev_s *priv = (FAR struct am263x_qspidev_s *)dev;
  size_t buflen = 0;

  DEBUGASSERT(cmdinfo != NULL);

  if ((cmdinfo->flags & (QSPICMD_IDUAL | QSPICMD_IQUAD | QSPICMD_IOCTAL |
                         QSPICMD_OCTALIO | QSPICMD_DTR)) != 0 ||
      cmdinfo->cmd > 0xff ||
      (QSPICMD_ISREAD(cmdinfo->flags) && QSPICMD_ISWRITE(cmdinfo->flags)))
    {
      return -ENOSYS;
    }

  if (QSPICMD_ISDATA(cmdinfo->flags))
    {
      DEBUGASSERT(cmdinfo->buffer != NULL);
      buflen = cmdinfo->buflen;
    }

  return qspi_cfgxfer(priv, cmdinfo->cmd,
                      QSPICMD_ISADDRESS(cmdinfo->flags), cmdinfo->addrlen,
                      cmdinfo->addr, 0, QSPICMD_ISWRITE(cmdinfo->flags),
                      false, cmdinfo->buffer, buflen);
}

/****************************************************************************
 * Name: qspi_memory
 *
 * Description:
 *   Reads of the first 8 MB go through the memory-mapped port, the rest
 *   through the configuration port, on one or four data lines.  Writes
 *   are configuration-port frames on one line.
 *
 ****************************************************************************/

static int qspi_memory(FAR struct qspi_dev_s *dev,
                       FAR struct qspi_meminfo_s *meminfo)
{
  FAR struct am263x_qspidev_s *priv = (FAR struct am263x_qspidev_s *)dev;
  FAR uint8_t *buf;
  uint32_t addr;
  size_t len;
  size_t n;
  int ret = OK;

  DEBUGASSERT(meminfo != NULL && meminfo->buffer != NULL);

  if (meminfo->cmd > 0xff)
    {
      return -ENOSYS;
    }

  if (QSPIMEM_ISWRITE(meminfo->flags))
    {
      if ((meminfo->flags & ~QSPIMEM_WRITE) != 0)
        {
          return -ENOSYS;
        }

      return qspi_cfgxfer(priv, meminfo->cmd, true, meminfo->addrlen,
                          meminfo->addr, 0, true, false, meminfo->buffer,
                          meminfo->buflen);
    }

  /* The address always goes out on one line; data on one or four */

  if ((meminfo->flags & (QSPIMEM_DUALIO | QSPIMEM_QUADIO | QSPIMEM_IDUAL |
                         QSPIMEM_IQUAD | QSPIMEM_IOCTAL | QSPIMEM_OCTALIO |
                         QSPIMEM_DTR | QSPIMEM_SCRAMBLE)) != 0)
    {
      return -ENOSYS;
    }

  if (meminfo->addrlen < 1 || meminfo->addrlen > 4)
    {
      return -EINVAL;
    }

  addr = meminfo->addr;
  buf  = meminfo->buffer;
  len  = meminfo->buflen;

  if (addr < AM263X_QSPI_FLASH_SIZE && len > 0)
    {
      n = AM263X_QSPI_FLASH_SIZE - addr;
      if (n > len)
        {
          n = len;
        }

      ret   = qspi_mmapread(priv, meminfo, addr, buf, n);
      addr += n;
      buf  += n;
      len  -= n;
    }

  if (ret == OK && len > 0)
    {
      ret = qspi_cfgread(priv, meminfo, addr, buf, len);
    }

  return ret;
}

/****************************************************************************
 * Name: qspi_alloc and qspi_free
 *
 * Description:
 *   No DMA: any memory will do.
 *
 ****************************************************************************/

static FAR void *qspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_malloc(buflen);
}

static void qspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  kmm_free(buffer);
}

/****************************************************************************
 * Name: qspi_hwinit
 *
 * Description:
 *   Put the controller in a known state whatever the SBL left: no idle,
 *   interrupts off, configuration port, mode 0, the initial SCLK.  The
 *   functional clock is set up (and the module reset) by
 *   am263x_clockconfig().
 *
 ****************************************************************************/

static void qspi_hwinit(FAR struct am263x_qspidev_s *priv)
{
  uint32_t regval;

  qspi_waitidle(priv);

  regval  = qspi_getreg(priv, AM263X_QSPI_SYSCONFIG_OFFSET);
  regval &= ~QSPI_SYSCONFIG_IDLE_MASK;
  regval |= QSPI_SYSCONFIG_IDLE_NONE;
  qspi_putreg(priv, AM263X_QSPI_SYSCONFIG_OFFSET, regval);

  qspi_putreg(priv, AM263X_QSPI_INTR_ENCLR_OFFSET,
              QSPI_INTR_FIRQ | QSPI_INTR_WIRQ);
  qspi_putreg(priv, AM263X_QSPI_INTR_STAT_OFFSET,
              QSPI_INTR_FIRQ | QSPI_INTR_WIRQ);

  qspi_cfgport(priv, false);

  priv->setup       = 0;
  priv->initialized = false;
  qspi_setmode(&priv->qspi, QSPIDEV_MODE0);
  qspi_setfrequency(&priv->qspi, QSPI_DEFAULT_FREQUENCY);
  priv->initialized = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_qspi_initialize
 ****************************************************************************/

FAR struct qspi_dev_s *am263x_qspi_initialize(int intf)
{
  FAR struct am263x_qspidev_s *priv = &g_qspi0dev;

  if (intf != 0)
    {
      return NULL;
    }

  nxmutex_lock(&priv->lock);
  if (!priv->initialized)
    {
      qspi_hwinit(priv);
      spiinfo("QSPI0 PID %08" PRIx32 "\n",
              qspi_getreg(priv, AM263X_QSPI_PID_OFFSET));
    }

  nxmutex_unlock(&priv->lock);
  return &priv->qspi;
}

#endif /* CONFIG_AM263X_QSPI */
