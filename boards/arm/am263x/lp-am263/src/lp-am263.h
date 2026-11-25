/****************************************************************************
 * boards/arm/am263x/lp-am263/src/lp-am263.h
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

#ifndef __BOARDS_ARM_AM263X_LP_AM263_SRC_LP_AM263_H
#define __BOARDS_ARM_AM263X_LP_AM263_SRC_LP_AM263_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_bringup
 *
 * Description:
 *   Bring up board features: file systems, drivers.
 *
 ****************************************************************************/

int am263x_bringup(void);

/****************************************************************************
 * Name: am263x_spi_initialize
 *
 * Description:
 *   Set up the SPI buses and, with CONFIG_SPI_DRIVER, /dev/spiN.
 *
 ****************************************************************************/

#ifdef CONFIG_AM263X_MCSPI
int am263x_spi_initialize(void);
#endif

#if defined(CONFIG_AM263X_ADC1) || defined(CONFIG_AM263X_EPWM)
int am263x_analog_initialize(void);
#endif

#ifdef CONFIG_AM263X_MDIO
int am263x_ethernet_initialize(void);
#endif

/****************************************************************************
 * Name: am263x_qspiflash_initialize
 *
 * Description:
 *   Set up the QSPI NOR boot flash as an MTD device, register its upper
 *   8 MB as /dev/data and mount it at /data with littlefs (formatting it
 *   if it does not mount).
 *
 ****************************************************************************/

#if defined(CONFIG_AM263X_QSPI) && defined(CONFIG_MTD_S25FLS)
int am263x_qspiflash_initialize(void);
#endif

#ifdef CONFIG_AM263X_I2C1
int am263x_eeprom_initialize(void);
#  ifdef CONFIG_I2C_EE_24XX
int am263x_eeprom_macaddr(uint8_t mac[6]);
#  endif
#endif

#endif /* __BOARDS_ARM_AM263X_LP_AM263_SRC_LP_AM263_H */
