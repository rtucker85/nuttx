/****************************************************************************
 * arch/arm/src/mspm0/hardware/mspm0_gpio.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_GPIO_H
#define __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/mspm0_memorymap.h"
#include "hardware/mspm0_gprcm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Power/reset control and the CPU interrupt registers are at the common
 * offsets in mspm0_gprcm.h.  Bit n of the interrupt registers is DIOn.
 */

#define MSPM0_GPIO_DOUT_OFFSET(n)       (0x1200 + ((n) & ~3)) /* Word holding DIOn, a byte per pin */
#define MSPM0_GPIO_DOUT31_0_OFFSET      0x1280                /* Data output */
#define MSPM0_GPIO_DOUTSET31_0_OFFSET   0x1290                /* Data output set */
#define MSPM0_GPIO_DOUTCLR31_0_OFFSET   0x12a0                /* Data output clear */
#define MSPM0_GPIO_DOUTTGL31_0_OFFSET   0x12b0                /* Data output toggle */
#define MSPM0_GPIO_DOE31_0_OFFSET       0x12c0                /* Data output enable */
#define MSPM0_GPIO_DOESET31_0_OFFSET    0x12d0                /* Data output enable set */
#define MSPM0_GPIO_DOECLR31_0_OFFSET    0x12e0                /* Data output enable clear */
#define MSPM0_GPIO_DIN_OFFSET(n)        (0x1300 + ((n) & ~3)) /* Word holding DIOn, a byte per pin */
#define MSPM0_GPIO_DIN31_0_OFFSET       0x1380                /* Data input */
#define MSPM0_GPIO_POLARITY15_0_OFFSET  0x1390                /* Interrupt edge, DIO0-15 */
#define MSPM0_GPIO_POLARITY31_16_OFFSET 0x13a0                /* Interrupt edge, DIO16-31 */
#define MSPM0_GPIO_CTL_OFFSET           0x1400                /* Fast wake global enable */
#define MSPM0_GPIO_FASTWAKE_OFFSET      0x1404                /* Fast wake enable */
#define MSPM0_GPIO_FILTEREN15_0_OFFSET  0x1508                /* Input filter, DIO0-15 */
#define MSPM0_GPIO_FILTEREN31_16_OFFSET 0x150c                /* Input filter, DIO16-31 */

/* Register bit definitions *************************************************/

/* POLARITY15_0 and POLARITY31_16: two bits per pin */

#define GPIO_POLARITY_SHIFT(n)  (((n) & 15) << 1)
#define GPIO_POLARITY_MASK(n)   (3 << GPIO_POLARITY_SHIFT(n))
#  define GPIO_POLARITY_NONE(n) (0 << GPIO_POLARITY_SHIFT(n))
#  define GPIO_POLARITY_RISE(n) (1 << GPIO_POLARITY_SHIFT(n))
#  define GPIO_POLARITY_FALL(n) (2 << GPIO_POLARITY_SHIFT(n))
#  define GPIO_POLARITY_BOTH(n) (3 << GPIO_POLARITY_SHIFT(n))

/* CTL */

#define GPIO_CTL_FASTWAKEONLY   (1 << 0)

#endif /* __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_GPIO_H */
