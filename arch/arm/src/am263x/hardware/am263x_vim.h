/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_vim.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_VIM_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_VIM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VIM (vectored interrupt manager) register offsets.  Interrupts are in
 * groups of 32; (n) is an interrupt number.
 */

#define VIM_IRQVEC_OFFSET           0x0018  /* Read: vector; write: ack */
#define VIM_FIQVEC_OFFSET           0x001c
#define VIM_ACTIRQ_OFFSET           0x0020  /* Active IRQ number */
#define VIM_ACTFIQ_OFFSET           0x0024
#define VIM_GRP_OFFSET(n)           (((n) >> 5) * 0x20)
#define VIM_RAW_OFFSET(n)           (0x0400 + VIM_GRP_OFFSET(n))
#define VIM_STS_OFFSET(n)           (0x0404 + VIM_GRP_OFFSET(n))
#define VIM_INTEN_OFFSET(n)         (0x0408 + VIM_GRP_OFFSET(n))
#define VIM_INTDIS_OFFSET(n)        (0x040c + VIM_GRP_OFFSET(n))
#define VIM_INTMAP_OFFSET(n)        (0x0418 + VIM_GRP_OFFSET(n))  /* 1 = FIQ */
#define VIM_INTTYPE_OFFSET(n)       (0x041c + VIM_GRP_OFFSET(n))  /* 1 = pulse */
#define VIM_PRI_OFFSET(n)           (0x1000 + ((n) * 4))
#define VIM_VEC_OFFSET(n)           (0x2000 + ((n) * 4))

#define VIM_BIT(n)                  (1u << ((n) & 0x1f))

#define VIM_ACTIRQ_VALID            (1u << 31)
#define VIM_ACTIRQ_NUM_MASK         0x3ff

#define VIM_PRI_LOWEST              15

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_VIM_H */
