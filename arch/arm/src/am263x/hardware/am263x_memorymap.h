/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_memorymap.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MEMORYMAP_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memories, as seen from an R5F core */

#define AM263X_TCMA_BASE            0x00000000  /* ATCM: the core resets here */
#define AM263X_TCMA_SIZE            (32 * 1024) /* per core, split mode */
#define AM263X_TCMB_BASE            0x00080000
#define AM263X_TCMB_SIZE            (32 * 1024)
#define AM263X_QSPI_FLASH_BASE      0x60000000  /* memory-mapped QSPI flash */
#define AM263X_OCRAM_BASE           0x70000000
#define AM263X_OCRAM_SIZE           (2 * 1024 * 1024)

/* Peripherals */

#define AM263X_ICSSM0_BASE          0x48000000  /* PRU-ICSS, see am263x_icssm.h */
#define AM263X_EPWM0_BASE           0x50000000  /* EPWM0-31, 4 KB apart */
#define AM263X_ADC0_RESULT_BASE     0x50100000  /* ADC0-4 results, 4 KB apart */
#define AM263X_ADC0_BASE            0x502c0000  /* ADC0-4, 4 KB apart */
#define AM263X_INTXBAR_BASE         0x502d5000  /* CONTROLSS interrupt xbar */
#define AM263X_CSS_CTRL_BASE        0x502f0000  /* CONTROLSS_CTRL */
#define AM263X_MSS_CTRL_BASE        0x50d00000
#define AM263X_TOP_CTRL_BASE        0x50d80000
#define AM263X_VIM_BASE             0x50f00000  /* each core sees its own */
#define AM263X_GPIO0_BASE           0x52000000  /* R5FSS0-0's GPIO */
#define AM263X_RTI0_BASE            0x52180000
#define AM263X_RTI1_BASE            0x52181000
#define AM263X_RTI2_BASE            0x52182000
#define AM263X_RTI3_BASE            0x52183000
#define AM263X_I2C0_BASE            0x52500000  /* I2C0-3, 4 KB apart */
#define AM263X_MCAN0_BASE           0x52600000  /* MCAN0-3, 64 KB apart */
#define AM263X_MCSPI0_BASE          0x52200000  /* MCSPI0-4, 4 KB apart */
#define AM263X_CPSW_BASE            0x52800000  /* CPSW3G, see am263x_cpsw.h */
#define AM263X_UART0_BASE           0x52300000
#define AM263X_UART1_BASE           0x52301000
#define AM263X_UART2_BASE           0x52302000
#define AM263X_UART3_BASE           0x52303000
#define AM263X_UART4_BASE           0x52304000
#define AM263X_UART5_BASE           0x52305000
#define AM263X_IOMUX_BASE           0x53100000
#define AM263X_TOP_RCM_BASE         0x53200000
#define AM263X_MSS_RCM_BASE         0x53208000

/* Lock registers ("kick") of the RCM and CTRL modules */

#define AM263X_KICK0_UNLOCK         0x01234567
#define AM263X_KICK1_UNLOCK         0x0fedcba8
#define AM263X_KICK_LOCK            0x00000000

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MEMORYMAP_H */
