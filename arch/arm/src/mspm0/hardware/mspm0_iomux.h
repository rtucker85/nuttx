/****************************************************************************
 * arch/arm/src/mspm0/hardware/mspm0_iomux.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_IOMUX_H
#define __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_IOMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/mspm0_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* There is one PINCM register per package pin, numbered from 1.  The
 * PINCM number of each GPIO depends on the device and is listed in the
 * device datasheet.
 */

#define MSPM0_IOMUX_PINCM_OFFSET(n) ((n) << 2)  /* n = 1..251 */

/* Register addresses *******************************************************/

#define MSPM0_IOMUX_PINCM(n)   (MSPM0_IOMUX_BASE + MSPM0_IOMUX_PINCM_OFFSET(n))

/* Register bit definitions *************************************************/

/* PINCMn */

#define IOMUX_PINCM_PF_SHIFT   (0)        /* Bits 0-5: Peripheral function */
#define IOMUX_PINCM_PF_MASK    (0x3f << IOMUX_PINCM_PF_SHIFT)
#  define IOMUX_PINCM_PF(n)    ((uint32_t)(n) << IOMUX_PINCM_PF_SHIFT)
#  define IOMUX_PF_UNCONNECTED 0          /* No function */
#  define IOMUX_PF_GPIO        1          /* GPIO on every pin that has one */
#define IOMUX_PINCM_PC         (1 << 7)   /* Peripheral connected */
#define IOMUX_PINCM_WAKESTAT   (1 << 13)  /* Pin caused a wakeup */
#define IOMUX_PINCM_PIPD       (1 << 16)  /* Pull-down enable */
#define IOMUX_PINCM_PIPU       (1 << 17)  /* Pull-up enable */
#define IOMUX_PINCM_INENA      (1 << 18)  /* Input enable */
#define IOMUX_PINCM_HYSTDIS    (1 << 19)  /* Hysteresis disable (0 = on) */
#define IOMUX_PINCM_DRV        (1 << 20)  /* High drive strength */
#define IOMUX_PINCM_HIZ1       (1 << 25)  /* Open drain (high is Hi-Z) */
#define IOMUX_PINCM_INV        (1 << 26)  /* Invert data */
#define IOMUX_PINCM_WUEN       (1 << 27)  /* Wakeup enable */
#define IOMUX_PINCM_WCOMP      (1 << 28)  /* Wakeup compare value */

#endif /* __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_IOMUX_H */
