/****************************************************************************
 * arch/arm/src/am263x/am263x_spi.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_SPI_H
#define __ARCH_ARM_SRC_AM263X_AM263X_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/spi/spi.h>

#ifdef CONFIG_AM263X_MCSPI

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_spibus_initialize
 *
 * Description:
 *   Initialize MCSPI<bus> as an SPI controller and return its interface,
 *   or NULL if that MCSPI is not enabled.  The board sets up the pinmux.
 *
 ****************************************************************************/

FAR struct spi_dev_s *am263x_spibus_initialize(int bus);

/****************************************************************************
 * Name: am263x_spi_hwcs
 *
 * Description:
 *   Assert or release the controller's own chip select CS<cs> (0-3).  For
 *   the board's am263x_spi<n>select() when a device is on an MCSPI CS pin
 *   rather than a GPIO.  Words go out on the channel of the CS last
 *   selected this way; channel 0 if none was.
 *
 ****************************************************************************/

void am263x_spi_hwcs(FAR struct spi_dev_s *dev, int cs, bool selected);

/****************************************************************************
 * Name: am263x_spi<n>select, am263x_spi<n>status, am263x_spi<n>cmddata
 *
 * Description:
 *   Provided by the board for each enabled MCSPI<n>: select or release a
 *   device, report its status, and (with CONFIG_SPI_CMDDATA) drive its
 *   command/data line.
 *
 ****************************************************************************/

#ifdef CONFIG_AM263X_MCSPI0
void am263x_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t am263x_spi0status(FAR struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int am263x_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#  endif
#endif

#ifdef CONFIG_AM263X_MCSPI1
void am263x_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t am263x_spi1status(FAR struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int am263x_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#  endif
#endif

#ifdef CONFIG_AM263X_MCSPI2
void am263x_spi2select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t am263x_spi2status(FAR struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int am263x_spi2cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#  endif
#endif

#ifdef CONFIG_AM263X_MCSPI3
void am263x_spi3select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t am263x_spi3status(FAR struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int am263x_spi3cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#  endif
#endif

#ifdef CONFIG_AM263X_MCSPI4
void am263x_spi4select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected);
uint8_t am263x_spi4status(FAR struct spi_dev_s *dev, uint32_t devid);
#  ifdef CONFIG_SPI_CMDDATA
int am263x_spi4cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd);
#  endif
#endif

#endif /* CONFIG_AM263X_MCSPI */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_SPI_H */
