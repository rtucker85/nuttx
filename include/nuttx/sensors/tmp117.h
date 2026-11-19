/****************************************************************************
 * include/nuttx/sensors/tmp117.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_TMP117_H
#define __INCLUDE_NUTTX_SENSORS_TMP117_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_SENSORS_TMP117

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C addresses, chosen by what ADD0 is tied to */

#define TMP117_ADDR_GND         0x48
#define TMP117_ADDR_VPLUS       0x49
#define TMP117_ADDR_SDA         0x4a
#define TMP117_ADDR_SCL         0x4b

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: tmp117_register_uorb
 *
 * Description:
 *   Probe a TI TMP117 temperature sensor (DEVICE_ID 0x117) and register it
 *   as the uORB topic sensor_temp<devno>.
 *
 * Input Parameters:
 *   devno - Topic instance number
 *   i2c   - The I2C bus the device is on
 *   addr  - Its 7-bit address, TMP117_ADDR_*
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure: -ENODEV if the
 *   device ID does not match.
 *
 ****************************************************************************/

int tmp117_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SENSORS_TMP117 */
#endif /* __INCLUDE_NUTTX_SENSORS_TMP117_H */
