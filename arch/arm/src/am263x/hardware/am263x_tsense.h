/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_tsense.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_TSENSE_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_TSENSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The thermal manager (TRM 6.2.2.2.2), in TOP_CTRL.  Four sensors, each
 * with a 7-bit ADC.  TSENSE0-2 are CTAT (the code falls as the die warms),
 * TSENSE3 is PTAT and for TI's internal use.  TSENSE0 and TSENSE1 sit at
 * the hot spots and have the alert and shutdown comparators, the FIFOs and
 * the accumulators.  TOP_CTRL is locked (LOCK0 kick registers).
 */

#define AM263X_TSENSE_NSENSORS          4

/* Factory trim eFuses: the ADC code of each sensor at -40, 150 and 30 degC,
 * 10 bits per sensor, TSENSE0 in bits 9:0 of TRIM_xxC0 up to TSENSE3
 * across xxC0 bits 31:30 and xxC1 bits 7:0.  TRIM_TEMP_xx holds
 * the tester's temperature for that row; its encoding is not documented.
 */

#define AM263X_TSENSE_TRIM_TEMP_M40C    (AM263X_TOP_CTRL_BASE + 0x0510)
#define AM263X_TSENSE_TRIM_M40C0        (AM263X_TOP_CTRL_BASE + 0x0514)
#define AM263X_TSENSE_TRIM_M40C1        (AM263X_TOP_CTRL_BASE + 0x0518)
#define AM263X_TSENSE_TRIM_TEMP_150C    (AM263X_TOP_CTRL_BASE + 0x051c)
#define AM263X_TSENSE_TRIM_150C0        (AM263X_TOP_CTRL_BASE + 0x0520)
#define AM263X_TSENSE_TRIM_150C1        (AM263X_TOP_CTRL_BASE + 0x0524)
#define AM263X_TSENSE_TRIM_TEMP_30C     (AM263X_TOP_CTRL_BASE + 0x0528)
#define AM263X_TSENSE_TRIM_30C0         (AM263X_TOP_CTRL_BASE + 0x052c)
#define AM263X_TSENSE_TRIM_30C1         (AM263X_TOP_CTRL_BASE + 0x0530)
#define AM263X_TSENSE_N_FACTOR          (AM263X_TOP_CTRL_BASE + 0x0534)

#define TSENSE_TRIM_BITS                10
#define TSENSE_TRIM_MASK                0x3ff

/* Controller registers */

#define AM263X_TSENSE_CFG               (AM263X_TOP_CTRL_BASE + 0x0d00)
#define AM263X_TSENSE_STATUS            (AM263X_TOP_CTRL_BASE + 0x0d04)
#define AM263X_TSENSE_STATUS_RAW        (AM263X_TOP_CTRL_BASE + 0x0d08)

/* TSENSE0 and TSENSE1 have the full set, 0x30 apart; TSENSE2 and TSENSE3
 * only RESULT, still on the same 0x30 stride.
 */

#define AM263X_TSENSE_BASE(n)           (AM263X_TOP_CTRL_BASE + 0x0d10 + \
                                         0x30 * (n))
#define AM263X_TSENSE_TSHUT(n)          (AM263X_TSENSE_BASE(n) + 0x00)
#define AM263X_TSENSE_ALERT(n)          (AM263X_TSENSE_BASE(n) + 0x04)
#define AM263X_TSENSE_CNTL(n)           (AM263X_TSENSE_BASE(n) + 0x08)
#define AM263X_TSENSE_RESULT(n)         (AM263X_TSENSE_BASE(n) + 0x0c)
#define AM263X_TSENSE_DATA(n, i)        (AM263X_TSENSE_BASE(n) + 0x10 + \
                                         4 * (i))
#define AM263X_TSENSE_ACCU(n)           (AM263X_TSENSE_BASE(n) + 0x20)

/* TSENSE_CFG.  The four OFF bits are set at reset (sensors powered down);
 * clear them, then set ENABLE to start the FSM, which converts the
 * sensors in SENSOR_SEL round robin from the 32 kHz clock: 51-54 cycles
 * each, then DELAY cycles (not 0) before the next.
 */

#define TSENSE_CFG_ENABLE               (1 << 0)
#define TSENSE_CFG_SENSOR_SEL_SHIFT     4
#define TSENSE_CFG_SENSOR_SEL_MASK      (0xf << TSENSE_CFG_SENSOR_SEL_SHIFT)
#define TSENSE_CFG_SENSOR_SEL(m)        ((m) << TSENSE_CFG_SENSOR_SEL_SHIFT)
#define TSENSE_CFG_DELAY_SHIFT          8
#define TSENSE_CFG_DELAY_MASK           (0x3f << TSENSE_CFG_DELAY_SHIFT)
#define TSENSE_CFG_DELAY(c)             ((c) << TSENSE_CFG_DELAY_SHIFT)
#define TSENSE_CFG_SNSR_MX_HIZ          (1 << 16)
#define TSENSE_CFG_AIPOFF               (1 << 20)
#define TSENSE_CFG_BGROFF               (1 << 24)
#define TSENSE_CFG_TMPSOFF              (1 << 28)
#define TSENSE_CFG_OFF                  (TSENSE_CFG_SNSR_MX_HIZ | \
                                         TSENSE_CFG_AIPOFF | \
                                         TSENSE_CFG_BGROFF | \
                                         TSENSE_CFG_TMPSOFF)

/* TSENSE_STATUS (masked) and TSENSE_STATUS_RAW, 4 bits per sensor 0-1 */

#define TSENSE_STATUS_LOW_THRHLD(n)     (1 << (4 * (n) + 0))
#define TSENSE_STATUS_HOT(n)            (1 << (4 * (n) + 1))
#define TSENSE_STATUS_COLD(n)           (1 << (4 * (n) + 2))

/* TSENSEn_TSHUT: the warm-reset comparator thresholds, ADC codes.  The
 * eFuse values apply unless EFUSE_OVERRIDE is 0x7.
 */

#define TSENSE_TSHUT_COLD_SHIFT         0
#define TSENSE_TSHUT_COLD_MASK          (0xff << TSENSE_TSHUT_COLD_SHIFT)
#define TSENSE_TSHUT_HOT_SHIFT          16
#define TSENSE_TSHUT_HOT_MASK           (0xff << TSENSE_TSHUT_HOT_SHIFT)
#define TSENSE_TSHUT_EFUSE_OVERRIDE     (0x7 << 29)

/* TSENSEn_ALERT: the interrupt thresholds, ADC codes.  The HOT
 * comparator is set while code >= HOT, the COLD one while code <= COLD
 * (measured): for these CTAT sensors COLD is the over-temperature one.
 */

#define TSENSE_ALERT_COLD_SHIFT         0
#define TSENSE_ALERT_COLD_MASK          (0xff << TSENSE_ALERT_COLD_SHIFT)
#define TSENSE_ALERT_HOT_SHIFT          16
#define TSENSE_ALERT_HOT_MASK           (0xff << TSENSE_ALERT_HOT_SHIFT)

/* TSENSEn_CNTL.  A set MASK_* bit masks that interrupt. */

#define TSENSE_CNTL_FIFO_CLEAR          (1 << 0)
#define TSENSE_CNTL_FIFO_FREEZE         (1 << 4)
#define TSENSE_CNTL_ACCU_CLEAR          (1 << 8)
#define TSENSE_CNTL_MASK_HOT            (1 << 16)
#define TSENSE_CNTL_MASK_COLD           (1 << 20)
#define TSENSE_CNTL_MASK_LOW_THRHLD     (1 << 24)
#define TSENSE_CNTL_MASK_ALL            (TSENSE_CNTL_MASK_HOT | \
                                         TSENSE_CNTL_MASK_COLD | \
                                         TSENSE_CNTL_MASK_LOW_THRHLD)

/* TSENSEn_RESULT: the last code, and EOCZ, set while a conversion runs */

#define TSENSE_RESULT_DTEMP_MASK        0xff
#define TSENSE_RESULT_EOCZ              (1 << 16)

/* TRM Table 6-22, for TSENSE0-2 without trims: 2 degC per code, 150 degC
 * at code 24 and below, -40 degC at 119 and above (+-8 degC).
 */

#define TSENSE_TABLE_CODE_MIN           24
#define TSENSE_TABLE_CODE_MAX           119
#define TSENSE_TABLE_TEMP(code)         (198 - 2 * (code))

/* R5F interrupts */

#define AM263X_IRQ_TSENSE_H             133  /* Hot threshold */
#define AM263X_IRQ_TSENSE_L             134  /* Cold threshold */

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_TSENSE_H */
