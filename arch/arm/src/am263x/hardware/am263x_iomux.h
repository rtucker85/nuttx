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

#define AM263X_PAD_QSPI_CSN1        0x0004  /* GPIO1 in mode 7 */
#define AM263X_PAD_SPI0_CS0         0x002c
#define AM263X_PAD_SPI0_CLK         0x0030
#define AM263X_PAD_SPI0_D0          0x0034
#define AM263X_PAD_SPI0_D1          0x0038
#define AM263X_PAD_RGMII1_RXC       0x0074  /* RGMII1 and MDIO: mode 0 */
#define AM263X_PAD_RGMII1_RX_CTL    0x0078
#define AM263X_PAD_RGMII1_RD0       0x007c
#define AM263X_PAD_RGMII1_RD1       0x0080
#define AM263X_PAD_RGMII1_RD2       0x0084
#define AM263X_PAD_RGMII1_RD3       0x0088
#define AM263X_PAD_RGMII1_TXC       0x008c
#define AM263X_PAD_RGMII1_TX_CTL    0x0090
#define AM263X_PAD_RGMII1_TD0       0x0094
#define AM263X_PAD_RGMII1_TD1       0x0098
#define AM263X_PAD_RGMII1_TD2       0x009c
#define AM263X_PAD_RGMII1_TD3       0x00a0
#define AM263X_PAD_MDIO_MDIO        0x00a4
#define AM263X_PAD_MDIO_MDC         0x00a8
#define AM263X_PAD_EPWM7_B          0x00e8  /* GPIO58 in mode 7 */
#define AM263X_PAD_EPWM8_A          0x00ec  /* I2C3_SDA in mode 2 */
#define AM263X_PAD_EPWM8_B          0x00f0  /* I2C3_SCL in mode 2 */
#define AM263X_PAD_EPWM10_A         0x00fc  /* GPIO63 in mode 7 */
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
