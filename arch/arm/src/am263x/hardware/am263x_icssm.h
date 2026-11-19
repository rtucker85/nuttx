/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_icssm.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_ICSSM_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_ICSSM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ICSSM (PRU-ICSS) memories and PRU control registers, as offsets from
 * AM263X_ICSSM0_BASE.  Each PRU has its own instruction RAM and data RAM;
 * a PRU sees its own data RAM at address 0.
 */

#define AM263X_ICSSM_PRU0_DRAM_OFFSET   0x00000
#define AM263X_ICSSM_PRU1_DRAM_OFFSET   0x02000
#define AM263X_ICSSM_SHARED_RAM_OFFSET  0x10000
#define AM263X_ICSSM_PRU0_CTRL_OFFSET   0x22000
#define AM263X_ICSSM_PRU1_CTRL_OFFSET   0x24000
#define AM263X_ICSSM_PRU0_IRAM_OFFSET   0x34000
#define AM263X_ICSSM_PRU1_IRAM_OFFSET   0x38000

#define AM263X_ICSSM_DRAM_SIZE          (8 * 1024)   /* per PRU */
#define AM263X_ICSSM_SHARED_RAM_SIZE    (32 * 1024)
#define AM263X_ICSSM_IRAM_SIZE          (12 * 1024)  /* per PRU */

#define AM263X_ICSSM_NPRU               2

/* PRU control register offsets */

#define AM263X_PRU_CONTROL_OFFSET       0x0000
#define AM263X_PRU_STATUS_OFFSET        0x0004

/* CONTROL register */

#define PRU_CONTROL_SOFT_RST_N          (1 << 0)   /* Write 0 to reset */
#define PRU_CONTROL_ENABLE              (1 << 1)   /* Run */
#define PRU_CONTROL_SLEEPING            (1 << 2)   /* In SLP */
#define PRU_CONTROL_COUNTER_ENABLE      (1 << 3)   /* Cycle/stall counters */
#define PRU_CONTROL_SINGLE_STEP         (1 << 8)
#define PRU_CONTROL_RUNSTATE            (1 << 15)  /* 1: running */
#define PRU_CONTROL_PCRESET_SHIFT       16         /* PC after reset (words) */
#define PRU_CONTROL_PCRESET_MASK        (0xffff << PRU_CONTROL_PCRESET_SHIFT)

/* STATUS register: current program counter, in words */

#define PRU_STATUS_PC_MASK              0xffff

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_ICSSM_H */
