/****************************************************************************
 * arch/arm/include/am263x/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_AM263X_IRQ_H
#define __ARCH_ARM_INCLUDE_AM263X_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VIM interrupt lines of R5FSS0-0.  IRQ numbers are VIM input numbers,
 * which are the same on every R5F core of the AM263x.  Only the lines
 * used by NuttX are named here; see the TRM interrupt map for the rest.
 */

#define AM263X_IRQ_CPSW0_TX         12  /* CPDMA TX (pulse) */
#define AM263X_IRQ_CPSW0_RX         13  /* CPDMA RX (pulse) */
#define AM263X_IRQ_CPSW0_RX_THRESH  14
#define AM263X_IRQ_CPSW0_MISC       15  /* CPTS, MDIO, statistics */
#define AM263X_IRQ_UART0            38
#define AM263X_IRQ_UART1            39
#define AM263X_IRQ_UART2            40
#define AM263X_IRQ_UART3            41
#define AM263X_IRQ_UART4            42
#define AM263X_IRQ_UART5            43
#define AM263X_IRQ_RTI0_0           84  /* RTI0 compare 0: system tick */
#define AM263X_IRQ_RTI0_1           85
#define AM263X_IRQ_RTI0_2           86
#define AM263X_IRQ_RTI0_3           87

#define NR_IRQS                     256

#endif /* __ARCH_ARM_INCLUDE_AM263X_IRQ_H */
