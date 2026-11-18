/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_iomux.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_IOMUX_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_IOMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IOMUX lock registers.  The IOMUX has its own unlock values. */

#define AM263X_IOMUX_KICK0_OFFSET   0x0298
#define AM263X_IOMUX_KICK1_OFFSET   0x029c
#define AM263X_IOMUX_KICK0          (AM263X_IOMUX_BASE + AM263X_IOMUX_KICK0_OFFSET)
#define AM263X_IOMUX_KICK1          (AM263X_IOMUX_BASE + AM263X_IOMUX_KICK1_OFFSET)
#define AM263X_IOMUX_KICK0_UNLOCK   0x83e70b13
#define AM263X_IOMUX_KICK1_UNLOCK   0x95a4f1e0

/* Pad configuration register offsets (one 32-bit register per pad) */

#define AM263X_PAD_UART0_RXD        0x006c
#define AM263X_PAD_UART0_TXD        0x0070

/* Pad configuration fields */

#define PAD_MODE(n)                 ((uint32_t)(n) & 0xf)
#define PAD_FORCE_INPUT_ENABLE      (1 << 4)
#define PAD_FORCE_INPUT_DISABLE     (3 << 4)
#define PAD_FORCE_OUTPUT_ENABLE     (1 << 6)
#define PAD_FORCE_OUTPUT_DISABLE    (3 << 6)
#define PAD_PULL_DISABLE            (1 << 8)
#define PAD_PULL_UP                 (1 << 9)  /* when pull enabled */
#define PAD_PULL_DOWN               (0 << 9)
#define PAD_SLEW_LOW                (1 << 10)
#define PAD_GPIO_OWNER(core)        ((uint32_t)(core) << 16)
#define PAD_QUAL_SYNC               (0 << 18)
#define PAD_QUAL_3SAMPLE            (1 << 18)
#define PAD_QUAL_6SAMPLE            (2 << 18)
#define PAD_QUAL_ASYNC              (3 << 18)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_IOMUX_H */
