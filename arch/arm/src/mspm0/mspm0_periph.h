/****************************************************************************
 * arch/arm/src/mspm0/mspm0_periph.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_MSPM0_PERIPH_H
#define __ARCH_ARM_SRC_MSPM0_MSPM0_PERIPH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "arm_internal.h"
#include "hardware/mspm0_gprcm.h"

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: mspm0_periph_enable
 *
 * Description:
 *   Reset a peripheral, then power it up.  The peripheral needs a few
 *   cycles after power-up before its registers can be used.
 *
 * Input Parameters:
 *   base - Base address of the peripheral
 *
 ****************************************************************************/

static inline void mspm0_periph_enable(uintptr_t base)
{
  volatile int i;

  putreg32(GPRCM_RSTCTL_KEY | GPRCM_RSTCTL_RESETSTKYCLR |
           GPRCM_RSTCTL_RESETASSERT, base + MSPM0_GPRCM_RSTCTL_OFFSET);
  putreg32(GPRCM_PWREN_KEY | GPRCM_PWREN_ENABLE,
           base + MSPM0_GPRCM_PWREN_OFFSET);

  for (i = 0; i < 16; i++)
    {
    }
}

/****************************************************************************
 * Name: mspm0_periph_powered
 *
 * Description:
 *   Return true if the peripheral is powered.
 *
 ****************************************************************************/

static inline bool mspm0_periph_powered(uintptr_t base)
{
  return (getreg32(base + MSPM0_GPRCM_PWREN_OFFSET) &
          GPRCM_PWREN_ENABLE) != 0;
}

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MSPM0_MSPM0_PERIPH_H */
