/****************************************************************************
 * arch/arm/include/am263x/tsense.h
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

/* User interface of the AM263x on-die temperature sensor driver: a uORB
 * temperature topic (SENSOR_TYPE_TEMPERATURE) carrying the hottest of
 * TSENSE0-2, plus one chip-specific ioctl on its /dev/uorb/sensor_tempN:
 *
 *   AM263X_TSENSEIOC_INFO   Every sensor's last code and temperature, and
 *                           the calibration in use.
 *                           arg: struct am263x_tsense_info_s *.
 */

#ifndef __ARCH_ARM_INCLUDE_AM263X_TSENSE_H
#define __ARCH_ARM_INCLUDE_AM263X_TSENSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Far above the generic SNIOC commands, so the upper half passes it on */

#define AM263X_TSENSEIOC_INFO       _SNIOC(0x0f00)

#define AM263X_TSENSE_COUNT         4

/* struct am263x_tsense_info_s cal[].  TSENSE3 is always NONE: raw only */

#define AM263X_TSENSE_CAL_TABLE     0  /* TRM Table 6-22 (+-8 degC) */
#define AM263X_TSENSE_CAL_2POINT    2  /* Trims at -40 and 150 degC */
#define AM263X_TSENSE_CAL_3POINT    3  /* Trims at -40, 30 and 150 degC */
#define AM263X_TSENSE_CAL_NONE      0xff

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct am263x_tsense_info_s
{
  float    temp[AM263X_TSENSE_COUNT];     /* degC */
  uint8_t  code[AM263X_TSENSE_COUNT];     /* Last ADC code (RESULT.DTEMP) */
  uint8_t  cal[AM263X_TSENSE_COUNT];      /* AM263X_TSENSE_CAL_* */
  uint16_t trim_m40[AM263X_TSENSE_COUNT]; /* eFuse codes at -40 degC */
  uint16_t trim_30[AM263X_TSENSE_COUNT];  /* at 30 degC */
  uint16_t trim_150[AM263X_TSENSE_COUNT]; /* at 150 degC */
  uint8_t  hottest;                       /* Sensor on the topic */
  uint8_t  alert_hot;                     /* Sensors 0-1 over the alert */
  uint32_t alerts;                        /* Alert interrupts since boot */
  uint32_t cfg;                           /* TSENSE_CFG */
  uint32_t tshut[2];                      /* TSENSE0/1_TSHUT */
  uint32_t alert[2];                      /* TSENSE0/1_ALERT */
  uint32_t cntl[2];                       /* TSENSE0/1_CNTL */
  uint32_t status_raw;                    /* TSENSE_STATUS_RAW */
};

#endif /* __ARCH_ARM_INCLUDE_AM263X_TSENSE_H */
