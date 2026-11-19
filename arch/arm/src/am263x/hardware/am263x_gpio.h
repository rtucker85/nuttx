/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_gpio.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_GPIO_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO0 belongs to R5FSS0-0 (pads with GPIO_SEL 0); GPIO1-3 to the other
 * cores.  Pins 0-139, 32 to a register set.
 */

#define AM263X_GPIO_NPINS           140

/* Register set n (pins 32n to 32n + 31) */

#define AM263X_GPIO_DIR(n)          (0x10 + 0x28 * (n)) /* 1 = input */
#define AM263X_GPIO_OUT_DATA(n)     (0x14 + 0x28 * (n))
#define AM263X_GPIO_SET_DATA(n)     (0x18 + 0x28 * (n))
#define AM263X_GPIO_CLR_DATA(n)     (0x1c + 0x28 * (n))
#define AM263X_GPIO_IN_DATA(n)      (0x20 + 0x28 * (n))

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_GPIO_H */
