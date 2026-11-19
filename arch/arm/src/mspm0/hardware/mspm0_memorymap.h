/****************************************************************************
 * arch/arm/src/mspm0/hardware/mspm0_memorymap.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_MEMORYMAP_H
#define __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory regions ***********************************************************/

#define MSPM0_FLASH_BASE       0x00000000 /* Main flash */
#define MSPM0_SRAM0_BASE       0x20200000 /* SRAM bank 0 */
#define MSPM0_SRAM1_BASE       0x20210000 /* SRAM bank 1 */
#define MSPM0_NONMAIN_BASE     0x41c00000 /* BCR/BSL configuration */
#define MSPM0_FACTORY_BASE     0x41c40000 /* Factory trim region */
#define MSPM0_DATA_BASE        0x41d00000 /* Data flash */

/* Peripheral base addresses ************************************************/

#define MSPM0_ADC0_BASE        0x40000000
#define MSPM0_ADC1_BASE        0x40002000
#define MSPM0_COMP0_BASE       0x40008000
#define MSPM0_COMP1_BASE       0x4000a000
#define MSPM0_COMP2_BASE       0x4000c000
#define MSPM0_DAC0_BASE        0x40018000
#define MSPM0_VREF_BASE        0x40030000
#define MSPM0_WWDT0_BASE       0x40080000
#define MSPM0_WWDT1_BASE       0x40082000
#define MSPM0_TIMG0_BASE       0x40084000
#define MSPM0_TIMG8_BASE       0x40090000
#define MSPM0_TIMG9_BASE       0x40092000
#define MSPM0_LFSS_BASE        0x40094000
#define MSPM0_TIMG14_BASE      0x40096000
#define MSPM0_GPIOA_BASE       0x400a0000
#define MSPM0_GPIOB_BASE       0x400a2000
#define MSPM0_GPIOC_BASE       0x400a4000
#define MSPM0_KEYSTORECTL_BASE 0x400ac000
#define MSPM0_SYSCTL_BASE      0x400af000
#define MSPM0_DEBUGSS_BASE     0x400c7000
#define MSPM0_FLASHCTL_BASE    0x400cd000
#define MSPM0_I2C0_BASE        0x400f0000
#define MSPM0_I2C1_BASE        0x400f2000
#define MSPM0_I2C2_BASE        0x400f4000
#define MSPM0_UART1_BASE       0x40100000
#define MSPM0_UART0_BASE       0x40108000
#define MSPM0_UART7_BASE       0x4010a000
#define MSPM0_CPUSS_BASE       0x40400000
#define MSPM0_MATHACL_BASE     0x40410000
#define MSPM0_WUC_BASE         0x40424000
#define MSPM0_IOMUX_BASE       0x40428000
#define MSPM0_DMA_BASE         0x4042a000
#define MSPM0_CRCP0_BASE       0x40440000
#define MSPM0_AESADV_BASE      0x40442000
#define MSPM0_TRNG_BASE        0x40444000
#define MSPM0_SPI0_BASE        0x40468000
#define MSPM0_SPI1_BASE        0x4046a000
#define MSPM0_SPI2_BASE        0x4046c000
#define MSPM0_UART3_BASE       0x40500000
#define MSPM0_UART4_BASE       0x40502000
#define MSPM0_UART5_BASE       0x40504000
#define MSPM0_UART6_BASE       0x40506000
#define MSPM0_CANFD0_BASE      0x40508000
#define MSPM0_CANFD1_BASE      0x40510000
#define MSPM0_TIMA0_BASE       0x40860000
#define MSPM0_TIMA1_BASE       0x40862000
#define MSPM0_TIMG6_BASE       0x40868000
#define MSPM0_TIMG7_BASE       0x4086a000
#define MSPM0_TIMG12_BASE      0x40870000

/* LFSS and RTC_B share a register block */

#define MSPM0_RTC_BASE         MSPM0_LFSS_BASE

#endif /* __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_MEMORYMAP_H */
