/****************************************************************************
 * arch/arm/include/mspm0/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_MSPM0_CHIP_H
#define __ARCH_ARM_INCLUDE_MSPM0_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip capabilities ********************************************************/

#if defined(CONFIG_ARCH_CHIP_MSPM0G351X)
#  define MSPM0_FLASH_SIZE       (512 * 1024)  /* Main flash, two banks */
#  define MSPM0_SRAM0_SIZE       (64 * 1024)   /* SRAM bank 0 */
#  define MSPM0_SRAM1_SIZE       (64 * 1024)   /* SRAM bank 1, lost below SLEEP */
#  define MSPM0_NUART            7             /* UART0-UART7, no UART2 */
#  define MSPM0_NCANFD           2
#else
#  error "Unsupported MSPM0 chip"
#endif

/* NVIC priority levels *****************************************************/

/* The Cortex-M0+ implements two bits of priority, bits[7:6] of each field.
 * Lower values are higher priority.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits[7:6] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Two bits of interrupt priority used */

#endif /* __ARCH_ARM_INCLUDE_MSPM0_CHIP_H */
