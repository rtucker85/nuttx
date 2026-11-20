/****************************************************************************
 * arch/arm/src/am263x/am263x_controlss.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/am263x_memorymap.h"
#include "am263x_controlss.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_controlss_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_css_ctrl_modify
 ****************************************************************************/

void am263x_css_ctrl_modify(uintptr_t addr, uint32_t clrbits,
                            uint32_t setbits)
{
  irqstate_t flags = spin_lock_irqsave(&g_controlss_lock);

  putreg32(AM263X_KICK0_UNLOCK, AM263X_CSS_CTRL_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_CSS_CTRL_LOCK0_KICK1);
  modifyreg32(addr, clrbits, setbits);
  putreg32(AM263X_KICK_LOCK, AM263X_CSS_CTRL_LOCK0_KICK0);
  putreg32(AM263X_KICK_LOCK, AM263X_CSS_CTRL_LOCK0_KICK1);

  spin_unlock_irqrestore(&g_controlss_lock, flags);
}

/****************************************************************************
 * Name: am263x_top_ctrl_modify16
 ****************************************************************************/

void am263x_top_ctrl_modify16(uintptr_t addr, uint16_t clrbits,
                              uint16_t setbits)
{
  irqstate_t flags = spin_lock_irqsave(&g_controlss_lock);

  putreg32(AM263X_KICK0_UNLOCK, AM263X_TOP_CTRL_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_TOP_CTRL_LOCK0_KICK1);
  putreg16((getreg16(addr) & ~clrbits) | setbits, addr);
  putreg32(AM263X_KICK_LOCK, AM263X_TOP_CTRL_LOCK0_KICK0);
  putreg32(AM263X_KICK_LOCK, AM263X_TOP_CTRL_LOCK0_KICK1);

  spin_unlock_irqrestore(&g_controlss_lock, flags);
}

/****************************************************************************
 * Name: am263x_intxbar_select
 ****************************************************************************/

void am263x_intxbar_select(int out, int group, uint32_t mask, bool enable)
{
  irqstate_t flags = spin_lock_irqsave(&g_controlss_lock);

  modifyreg32(AM263X_INTXBAR_SEL(out, group), enable ? 0 : mask,
              enable ? mask : 0);

  spin_unlock_irqrestore(&g_controlss_lock, flags);
}
