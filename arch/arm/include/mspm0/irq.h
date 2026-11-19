/****************************************************************************
 * arch/arm/include/mspm0/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_MSPM0_IRQ_H
#define __ARCH_ARM_INCLUDE_MSPM0_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds to the vector number and hence
 * maps directly to bits in the NVIC.
 */

/* Processor exceptions (vectors 0-15) */

#define MSPM0_IRQ_RESERVED     (0)  /* Reserved vector (only used with CONFIG_DEBUG_FEATURES) */
                                    /* Vector  0: Reset stack pointer value */
                                    /* Vector  1: Reset (not handled as an IRQ) */
#define MSPM0_IRQ_NMI          (2)  /* Vector  2: Non-Maskable Interrupt (NMI) */
#define MSPM0_IRQ_HARDFAULT    (3)  /* Vector  3: Hard fault */
                                    /* Vector  4-10: Reserved */
#define MSPM0_IRQ_SVCALL       (11) /* Vector 11: SVC call */
                                    /* Vector 12-13: Reserved */
#define MSPM0_IRQ_PENDSV       (14) /* Vector 14: Pendable system service request */
#define MSPM0_IRQ_SYSTICK      (15) /* Vector 15: System tick */
#define MSPM0_IRQ_EXTINT       (16) /* Vector 16: First external interrupt */

/* External interrupts (vectors >= 16).
 *
 * Lines 0 and 1 are shared group interrupts.  The source within a group is
 * read from the CPUSS INT_GROUP IIDX register.
 */

#if defined(CONFIG_ARCH_CHIP_MSPM0G351X)
#  define MSPM0_IRQ_GROUP0     (MSPM0_IRQ_EXTINT + 0)  /* WWDT0/1, DEBUGSS, SYSCTL, FLASHCTL */
#  define MSPM0_IRQ_GROUP1     (MSPM0_IRQ_EXTINT + 1)  /* GPIOA/B/C, COMP0/1/2, TRNG */
#  define MSPM0_IRQ_TIMG8      (MSPM0_IRQ_EXTINT + 2)
#  define MSPM0_IRQ_UART3      (MSPM0_IRQ_EXTINT + 3)
#  define MSPM0_IRQ_ADC0       (MSPM0_IRQ_EXTINT + 4)
#  define MSPM0_IRQ_ADC1       (MSPM0_IRQ_EXTINT + 5)
#  define MSPM0_IRQ_CANFD0     (MSPM0_IRQ_EXTINT + 6)
#  define MSPM0_IRQ_DAC0       (MSPM0_IRQ_EXTINT + 7)
#  define MSPM0_IRQ_TIMG9      (MSPM0_IRQ_EXTINT + 8)
#  define MSPM0_IRQ_SPI0       (MSPM0_IRQ_EXTINT + 9)
#  define MSPM0_IRQ_SPI1       (MSPM0_IRQ_EXTINT + 10)
#  define MSPM0_IRQ_SPI2       (MSPM0_IRQ_EXTINT + 11)
#  define MSPM0_IRQ_CANFD1     (MSPM0_IRQ_EXTINT + 12)
#  define MSPM0_IRQ_UART1      (MSPM0_IRQ_EXTINT + 13)
#  define MSPM0_IRQ_UART4      (MSPM0_IRQ_EXTINT + 14)
#  define MSPM0_IRQ_UART0      (MSPM0_IRQ_EXTINT + 15)
#  define MSPM0_IRQ_TIMG0      (MSPM0_IRQ_EXTINT + 16)
#  define MSPM0_IRQ_TIMG6      (MSPM0_IRQ_EXTINT + 17)
#  define MSPM0_IRQ_TIMA0      (MSPM0_IRQ_EXTINT + 18)
#  define MSPM0_IRQ_TIMA1      (MSPM0_IRQ_EXTINT + 19)
#  define MSPM0_IRQ_TIMG7      (MSPM0_IRQ_EXTINT + 20)
#  define MSPM0_IRQ_TIMG12     (MSPM0_IRQ_EXTINT + 21)
#  define MSPM0_IRQ_TIMG14     (MSPM0_IRQ_EXTINT + 22)
#  define MSPM0_IRQ_UART5      (MSPM0_IRQ_EXTINT + 23)
#  define MSPM0_IRQ_I2C0       (MSPM0_IRQ_EXTINT + 24)
#  define MSPM0_IRQ_I2C1       (MSPM0_IRQ_EXTINT + 25)
#  define MSPM0_IRQ_I2C2       (MSPM0_IRQ_EXTINT + 26)
#  define MSPM0_IRQ_UART7      (MSPM0_IRQ_EXTINT + 27)
#  define MSPM0_IRQ_AESADV     (MSPM0_IRQ_EXTINT + 28)
#  define MSPM0_IRQ_UART6      (MSPM0_IRQ_EXTINT + 29)
#  define MSPM0_IRQ_LFSS       (MSPM0_IRQ_EXTINT + 30) /* LFSS and RTC_B */
#  define MSPM0_IRQ_DMA        (MSPM0_IRQ_EXTINT + 31)
#  define MSPM0_IRQ_NEXTINT    (32)
#else
#  error "Unsupported MSPM0 chip"
#endif

#define MSPM0_IRQ_NIRQS        (MSPM0_IRQ_EXTINT + MSPM0_IRQ_NEXTINT)

#define NR_VECTORS             MSPM0_IRQ_NIRQS
#define NR_IRQS                MSPM0_IRQ_NIRQS

#endif /* __ARCH_ARM_INCLUDE_MSPM0_IRQ_H */
