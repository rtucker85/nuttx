/****************************************************************************
 * include/nuttx/analog/ads7038.h
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

#ifndef __INCLUDE_NUTTX_ANALOG_ADS7038_H
#define __INCLUDE_NUTTX_ANALOG_ADS7038_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/analog/adc.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_ADC_ADS7038

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ads7038_initialize
 *
 * Description:
 *   Probe, reset and calibrate a TI ADS7038 or ADS7038-Q1 8-channel 12-bit
 *   SPI ADC, and return its ADC lower half for adc_register().
 *   ANIOC_TRIGGER converts each channel in chanmask once, in ascending
 *   order; the results are 12-bit codes (full scale = AVDD) with the
 *   AIN number as the channel.
 *
 * Input Parameters:
 *   spi      - The SPI bus the device is on
 *   devid    - The device's SPI ID for SPI_SELECT(), e.g. SPIDEV_ADC(0)
 *   chanmask - The analog inputs to convert, bit n for AINn
 *
 * Returned Value:
 *   The ADC lower half, or NULL if the device does not respond.
 *
 ****************************************************************************/

FAR struct adc_dev_s *ads7038_initialize(FAR struct spi_dev_s *spi,
                                         uint32_t devid, uint8_t chanmask);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_ADC_ADS7038 */
#endif /* __INCLUDE_NUTTX_ANALOG_ADS7038_H */
