/****************************************************************************
 * include/nuttx/sensors/lsm6dsv.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DSV_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DSV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/spi/spi.h>

#ifdef CONFIG_SENSORS_LSM6DSV

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: lsm6dsv_register_uorb
 *
 * Description:
 *   Probe an STMicroelectronics LSM6DSV 6-axis IMU on an SPI bus and
 *   register it as the uORB topics sensor_accel<devno> and
 *   sensor_gyro<devno>.  A kernel thread polls the device while either
 *   topic is active.
 *
 * Input Parameters:
 *   devno - Topic instance number
 *   spi   - The SPI bus the device is on
 *   devid - The device's SPI ID for SPI_SELECT(), e.g. SPIDEV_IMU(0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure: -ENODEV if
 *   WHO_AM_I does not read 0x70.
 *
 ****************************************************************************/

int lsm6dsv_register_uorb(int devno, FAR struct spi_dev_s *spi,
                          uint32_t devid);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_LSM6DSV */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM6DSV_H */
