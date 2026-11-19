/****************************************************************************
 * arch/arm/src/mspm0/hardware/mspm0_gprcm.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_GPRCM_H
#define __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_GPRCM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Most MSPM0 peripherals share a common layout for power, reset and clock
 * control (GPRCM), the CPU interrupt event registers and a few others.
 * These offsets are relative to each peripheral's base address.
 */

/* Register offsets *********************************************************/

#define MSPM0_GPRCM_PWREN_OFFSET    0x0800 /* Power enable */
#define MSPM0_GPRCM_RSTCTL_OFFSET   0x0804 /* Reset control */
#define MSPM0_GPRCM_CLKCFG_OFFSET   0x0808 /* Clock configuration (some peripherals) */
#define MSPM0_GPRCM_STAT_OFFSET     0x0814 /* Status */

#define MSPM0_CLKDIV_OFFSET         0x1000 /* Clock divider (some peripherals) */
#define MSPM0_CLKSEL_OFFSET         0x1008 /* Clock select (some peripherals) */
#define MSPM0_PDBGCTL_OFFSET        0x1018 /* Peripheral debug control */

#define MSPM0_CPU_INT_IIDX_OFFSET   0x1020 /* Interrupt index */
#define MSPM0_CPU_INT_IMASK_OFFSET  0x1028 /* Interrupt mask */
#define MSPM0_CPU_INT_RIS_OFFSET    0x1030 /* Raw interrupt status */
#define MSPM0_CPU_INT_MIS_OFFSET    0x1038 /* Masked interrupt status */
#define MSPM0_CPU_INT_ISET_OFFSET   0x1040 /* Interrupt set */
#define MSPM0_CPU_INT_ICLR_OFFSET   0x1048 /* Interrupt clear */

#define MSPM0_EVT_MODE_OFFSET       0x10e0 /* Event mode */
#define MSPM0_DESC_OFFSET           0x10fc /* Module description */

/* Register bit definitions *************************************************/

/* PWREN: writes take effect only with the key in bits 31:24 */

#define GPRCM_PWREN_ENABLE          (1 << 0)
#define GPRCM_PWREN_KEY             (0x26 << 24)

/* RSTCTL: writes take effect only with the key in bits 31:24 */

#define GPRCM_RSTCTL_RESETASSERT    (1 << 0) /* Assert peripheral reset */
#define GPRCM_RSTCTL_RESETSTKYCLR   (1 << 1) /* Clear STAT RESETSTKY */
#define GPRCM_RSTCTL_KEY            (0xb1 << 24)

/* STAT */

#define GPRCM_STAT_RESETSTKY        (1 << 16) /* Reset since last cleared */

/* PDBGCTL */

#define PDBGCTL_FREE                (1 << 0) /* Keep running when halted */
#define PDBGCTL_SOFT                (1 << 1) /* Stop after current transfer */

#endif /* __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_GPRCM_H */
