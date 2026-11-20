/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_eeprom.c
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

/* The LP-AM263's board ID EEPROM (U16, a 1 Mbit CAT24M01) on I2C1, as
 * /dev/eeprom0 through the generic 24xx driver.  TI's board data in it
 * includes MAC addresses: a count at 0x3b and the addresses from 0x3d,
 * in the upper 64 KiB (TI's SDK reads them at I2C address 0x53).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/eeprom/eeprom.h>

#include "am263x_i2c.h"
#include "am263x_pinmux.h"
#include "lp-am263.h"

#ifdef CONFIG_AM263X_I2C1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EEPROM_I2C_BUS      1
#define EEPROM_DEVNAME      "/dev/eeprom0"

/* 1010 A2 A1 P: base address with the A16 page bit clear */

#define EEPROM_I2C_ADDR     0x52

/* TI board data, in the upper 64 KiB: the MAC addresses from 0x3d */

#define EEPROM_MAC_DATA     (0x10000 + 0x3d)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct am263x_pinmux_s g_i2c1_pins[] =
{
  { AM263X_PAD_I2C1_SCL, PAD_MODE(0) | PAD_PULL_UP },
  { AM263X_PAD_I2C1_SDA, PAD_MODE(0) | PAD_PULL_UP },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_eeprom_initialize
 *
 * Description:
 *   Bring up I2C1 (as /dev/i2c1 with CONFIG_I2C_DRIVER) and register the
 *   board ID EEPROM as /dev/eeprom0, read-only.
 *
 ****************************************************************************/

int am263x_eeprom_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

  am263x_pinmux_config(g_i2c1_pins,
                       sizeof(g_i2c1_pins) / sizeof(g_i2c1_pins[0]));

  i2c = am263x_i2cbus_initialize(EEPROM_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, EEPROM_I2C_BUS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: i2c_register(%d) failed: %d\n",
             EEPROM_I2C_BUS, ret);
    }
#endif

#ifdef CONFIG_I2C_EE_24XX
  ret = ee24xx_initialize(i2c, EEPROM_I2C_ADDR, EEPROM_DEVNAME,
                          EEPROM_M24M01, true);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ee24xx_initialize failed: %d\n", ret);
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: am263x_eeprom_macaddr
 *
 * Description:
 *   The first MAC address in TI's board data.
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_EE_24XX
int am263x_eeprom_macaddr(uint8_t mac[6])
{
  struct file file;
  ssize_t nread;
  int ret;

  ret = file_open(&file, EEPROM_DEVNAME, O_RDONLY);
  if (ret < 0)
    {
      return ret;
    }

  ret = file_seek(&file, EEPROM_MAC_DATA, SEEK_SET);

  if (ret >= 0)
    {
      nread = file_read(&file, mac, 6);
      ret = nread == 6 ? OK : (nread < 0 ? (int)nread : -EIO);
    }

  file_close(&file);
  return ret;
}
#endif

#endif /* CONFIG_AM263X_I2C1 */
