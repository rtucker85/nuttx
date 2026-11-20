/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_bringup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <syslog.h>

#include <nuttx/fs/fs.h>

#include "lp-am263.h"

#ifdef CONFIG_AM263X_PRU
#  include "am263x_pru.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_bringup
 ****************************************************************************/

int am263x_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_AM263X_PRU
  ret = am263x_pru_register("/dev/pru0", 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pru0: %d\n", ret);
    }

  ret = am263x_pru_register("/dev/pru1", 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pru1: %d\n", ret);
    }
#endif

#ifdef CONFIG_AM263X_MCSPI
  ret = am263x_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI: %d\n", ret);
    }
#endif

#ifdef CONFIG_AM263X_I2C1
  ret = am263x_eeprom_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the EEPROM: %d\n", ret);
    }
#endif

#if defined(CONFIG_AM263X_ADC1) || defined(CONFIG_AM263X_EPWM)
  ret = am263x_analog_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the ADC or PWM: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_AM263X_MDIO
  ret = am263x_ethernet_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Ethernet: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
