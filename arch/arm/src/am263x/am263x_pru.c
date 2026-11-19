/****************************************************************************
 * arch/arm/src/am263x/am263x_pru.c
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

/* Character driver for the two PRU cores of ICSSM0: load a firmware image
 * into a core's IRAM and DMEM, start and stop the core, and read and write
 * its DMEM.  See arch/arm/include/am263x/pru.h for the user interface.
 *
 * The ICSSM clock is ungated and out of reset at power-on, so nothing
 * needs enabling first.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <arch/chip/pru.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/am263x_icssm.h"
#include "am263x_pru.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many polls of CONTROL to wait for a core to stop */

#define PRU_HALT_POLLS  1000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_pru_s
{
  uintptr_t ctrl;              /* CONTROL register */
  uintptr_t iram;              /* Instruction RAM */
  uintptr_t dram;              /* Data RAM */
  mutex_t lock;                /* Serialises ioctls */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t am263x_pru_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static ssize_t am263x_pru_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static off_t am263x_pru_seek(FAR struct file *filep, off_t offset,
                             int whence);
static int am263x_pru_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_am263x_pru_fops =
{
  .read  = am263x_pru_read,
  .write = am263x_pru_write,
  .seek  = am263x_pru_seek,
  .ioctl = am263x_pru_ioctl,
};

static struct am263x_pru_s g_am263x_pru[AM263X_ICSSM_NPRU] =
{
  {
    .ctrl = AM263X_ICSSM0_BASE + AM263X_ICSSM_PRU0_CTRL_OFFSET +
            AM263X_PRU_CONTROL_OFFSET,
    .iram = AM263X_ICSSM0_BASE + AM263X_ICSSM_PRU0_IRAM_OFFSET,
    .dram = AM263X_ICSSM0_BASE + AM263X_ICSSM_PRU0_DRAM_OFFSET,
    .lock = NXMUTEX_INITIALIZER,
  },
  {
    .ctrl = AM263X_ICSSM0_BASE + AM263X_ICSSM_PRU1_CTRL_OFFSET +
            AM263X_PRU_CONTROL_OFFSET,
    .iram = AM263X_ICSSM0_BASE + AM263X_ICSSM_PRU1_IRAM_OFFSET,
    .dram = AM263X_ICSSM0_BASE + AM263X_ICSSM_PRU1_DRAM_OFFSET,
    .lock = NXMUTEX_INITIALIZER,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_pru_halt
 *
 * Description:
 *   Clear ENABLE and wait for the core to stop at an instruction boundary.
 *
 ****************************************************************************/

static int am263x_pru_halt(FAR struct am263x_pru_s *priv)
{
  int i;

  modifyreg32(priv->ctrl, PRU_CONTROL_ENABLE, 0);

  for (i = 0; i < PRU_HALT_POLLS; i++)
    {
      if ((getreg32(priv->ctrl) & PRU_CONTROL_RUNSTATE) == 0)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: am263x_pru_fill
 *
 * Description:
 *   Copy nwords words into PRU memory, then clear the rest of it.
 *
 ****************************************************************************/

static void am263x_pru_fill(uintptr_t mem, size_t memsize,
                            FAR const uint32_t *src, size_t nwords)
{
  size_t i;

  for (i = 0; i < nwords; i++)
    {
      putreg32(src[i], mem + 4 * i);
    }

  for (; i < memsize / 4; i++)
    {
      putreg32(0, mem + 4 * i);
    }
}

/****************************************************************************
 * Name: am263x_pru_load
 ****************************************************************************/

static int am263x_pru_load(FAR struct am263x_pru_s *priv,
                           FAR const struct pru_image_s *image)
{
  int ret;

  if (image == NULL ||
      image->iram_size > AM263X_ICSSM_IRAM_SIZE ||
      image->dram_size > AM263X_ICSSM_DRAM_SIZE ||
      (image->iram_size & 3) != 0 || (image->dram_size & 3) != 0 ||
      (image->iram_size != 0 && image->iram == NULL) ||
      (image->dram_size != 0 && image->dram == NULL))
    {
      return -EINVAL;
    }

  ret = am263x_pru_halt(priv);
  if (ret < 0)
    {
      return ret;
    }

  am263x_pru_fill(priv->iram, AM263X_ICSSM_IRAM_SIZE,
                  image->iram, image->iram_size / 4);
  am263x_pru_fill(priv->dram, AM263X_ICSSM_DRAM_SIZE,
                  image->dram, image->dram_size / 4);
  return OK;
}

/****************************************************************************
 * Name: am263x_pru_start
 *
 * Description:
 *   Reset the core, which loads the PC from CONTROL[31:16], then run it.
 *
 ****************************************************************************/

static int am263x_pru_start(FAR struct am263x_pru_s *priv,
                            unsigned long entry)
{
  uint32_t regval;

  if (entry >= AM263X_ICSSM_IRAM_SIZE / 4)
    {
      return -EINVAL;
    }

  regval  = getreg32(priv->ctrl);
  regval &= ~(PRU_CONTROL_PCRESET_MASK | PRU_CONTROL_ENABLE |
              PRU_CONTROL_SOFT_RST_N);
  regval |= (uint32_t)entry << PRU_CONTROL_PCRESET_SHIFT;

  putreg32(regval, priv->ctrl);
  putreg32(regval | PRU_CONTROL_SOFT_RST_N | PRU_CONTROL_ENABLE,
           priv->ctrl);
  return OK;
}

/****************************************************************************
 * Name: am263x_pru_status
 ****************************************************************************/

static int am263x_pru_status(FAR struct am263x_pru_s *priv,
                             FAR struct pru_status_s *status)
{
  uint32_t regval;

  if (status == NULL)
    {
      return -EINVAL;
    }

  regval           = getreg32(priv->ctrl);
  status->control  = regval;
  status->running  = (regval & PRU_CONTROL_RUNSTATE) != 0;
  status->sleeping = (regval & PRU_CONTROL_SLEEPING) != 0;
  status->pc       = getreg32(priv->ctrl - AM263X_PRU_CONTROL_OFFSET +
                              AM263X_PRU_STATUS_OFFSET) &
                     PRU_STATUS_PC_MASK;
  return OK;
}

/****************************************************************************
 * Name: am263x_pru_dmem_check
 *
 * Description:
 *   Check a DMEM access at the file position and trim it to the end of
 *   DMEM.  Returns the length to transfer, or a negated errno.
 *
 ****************************************************************************/

static ssize_t am263x_pru_dmem_check(FAR struct file *filep, size_t buflen)
{
  off_t pos = filep->f_pos;

  if (((pos | buflen) & 3) != 0)
    {
      return -EINVAL;
    }

  if (pos >= AM263X_ICSSM_DRAM_SIZE)
    {
      return 0;
    }

  if (buflen > AM263X_ICSSM_DRAM_SIZE - pos)
    {
      buflen = AM263X_ICSSM_DRAM_SIZE - pos;
    }

  return buflen;
}

/****************************************************************************
 * Name: am263x_pru_read
 ****************************************************************************/

static ssize_t am263x_pru_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  FAR struct am263x_pru_s *priv = filep->f_inode->i_private;
  ssize_t nbytes;
  uint32_t word;
  ssize_t i;

  nbytes = am263x_pru_dmem_check(filep, buflen);
  for (i = 0; i < nbytes; i += 4)
    {
      word = getreg32(priv->dram + filep->f_pos + i);
      memcpy(buffer + i, &word, 4);
    }

  if (nbytes > 0)
    {
      filep->f_pos += nbytes;
    }

  return nbytes;
}

/****************************************************************************
 * Name: am263x_pru_write
 ****************************************************************************/

static ssize_t am263x_pru_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct am263x_pru_s *priv = filep->f_inode->i_private;
  ssize_t nbytes;
  uint32_t word;
  ssize_t i;

  nbytes = am263x_pru_dmem_check(filep, buflen);
  if (nbytes == 0 && buflen > 0)
    {
      return -EFBIG;
    }

  for (i = 0; i < nbytes; i += 4)
    {
      memcpy(&word, buffer + i, 4);
      putreg32(word, priv->dram + filep->f_pos + i);
    }

  if (nbytes > 0)
    {
      filep->f_pos += nbytes;
    }

  return nbytes;
}

/****************************************************************************
 * Name: am263x_pru_seek
 ****************************************************************************/

static off_t am263x_pru_seek(FAR struct file *filep, off_t offset,
                             int whence)
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
        pos = AM263X_ICSSM_DRAM_SIZE + offset;
        break;

      default:
        return -EINVAL;
    }

  if (pos < 0 || pos > AM263X_ICSSM_DRAM_SIZE)
    {
      return -EINVAL;
    }

  filep->f_pos = pos;
  return pos;
}

/****************************************************************************
 * Name: am263x_pru_ioctl
 ****************************************************************************/

static int am263x_pru_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct am263x_pru_s *priv = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case PRUIOC_HALT:
        ret = am263x_pru_halt(priv);
        break;

      case PRUIOC_LOAD:
        ret = am263x_pru_load(priv,
                              (FAR const struct pru_image_s *)
                              (uintptr_t)arg);
        break;

      case PRUIOC_START:
        ret = am263x_pru_start(priv, arg);
        break;

      case PRUIOC_STATUS:
        ret = am263x_pru_status(priv,
                                (FAR struct pru_status_s *)(uintptr_t)arg);
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_pru_register
 ****************************************************************************/

int am263x_pru_register(FAR const char *path, int pru)
{
  if (pru < 0 || pru >= AM263X_ICSSM_NPRU)
    {
      return -EINVAL;
    }

  return register_driver(path, &g_am263x_pru_fops, 0666,
                         &g_am263x_pru[pru]);
}
