/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_qspiflash.c
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

/* The LP-AM263 S25FL128S QSPI NOR boot flash on QSPI0 CS0.  The upper
 * 8 MB (0x800000-0xffffff), away from the SBL (0x0) and the application
 * image (0x80000), is a partition registered as /dev/data and mounted at
 * /data with littlefs.  littlefs formats it only if it cannot mount it,
 * that is on the first boot.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>

#include "am263x_pinmux.h"
#include "am263x_qspi.h"
#include "lp-am263.h"

#if defined(CONFIG_AM263X_QSPI) && defined(CONFIG_MTD_S25FLS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The data partition: the upper half of the 16 MB part */

#define QSPIFLASH_PART_OFFSET  0x800000
#define QSPIFLASH_PART_SIZE    0x800000

#define QSPIFLASH_DEVPATH      "/dev/data"
#define QSPIFLASH_MOUNTPT      "/data"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct am263x_pinmux_s g_qspi_pins[] =
{
  { AM263X_PAD_QSPI_CSN0, PAD_MODE(0) | PAD_PULL_UP },
  { AM263X_PAD_QSPI_CLK,  PAD_MODE(0) | PAD_PULL_DISABLE },
  { AM263X_PAD_QSPI_D0,   PAD_MODE(0) | PAD_PULL_DISABLE },
  { AM263X_PAD_QSPI_D1,   PAD_MODE(0) | PAD_PULL_DISABLE },
  { AM263X_PAD_QSPI_D2,   PAD_MODE(0) | PAD_PULL_UP },  /* WP# */
  { AM263X_PAD_QSPI_D3,   PAD_MODE(0) | PAD_PULL_UP },  /* HOLD# */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_qspiflash_initialize
 ****************************************************************************/

int am263x_qspiflash_initialize(void)
{
  FAR struct qspi_dev_s *qspi;
  FAR struct mtd_dev_s *mtd;
  FAR struct mtd_dev_s *part;
  struct mtd_geometry_s geo;
  int ret;

  am263x_pinmux_config(g_qspi_pins,
                       sizeof(g_qspi_pins) / sizeof(g_qspi_pins[0]));

  qspi = am263x_qspi_initialize(0);
  if (qspi == NULL)
    {
      return -ENODEV;
    }

  mtd = s25fls_initialize(qspi);
  if (mtd == NULL)
    {
      return -ENODEV;
    }

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      return ret;
    }

  part = mtd_partition(mtd, QSPIFLASH_PART_OFFSET / geo.blocksize,
                       QSPIFLASH_PART_SIZE / geo.blocksize);
  if (part == NULL)
    {
      return -ENOMEM;
    }

  ret = register_mtddriver(QSPIFLASH_DEVPATH, part, 0755, NULL);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS
  ret = nx_mount(QSPIFLASH_DEVPATH, QSPIFLASH_MOUNTPT, "littlefs", 0,
                 "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount %s: %d\n", QSPIFLASH_MOUNTPT,
             ret);
      return ret;
    }

  syslog(LOG_INFO, "qspi: %s: littlefs on %s, %lu x %lu KB\n",
         QSPIFLASH_MOUNTPT, QSPIFLASH_DEVPATH,
         (unsigned long)(QSPIFLASH_PART_SIZE / geo.erasesize),
         (unsigned long)geo.erasesize / 1024);
#endif

  return OK;
}

#endif /* CONFIG_AM263X_QSPI && CONFIG_MTD_S25FLS */
