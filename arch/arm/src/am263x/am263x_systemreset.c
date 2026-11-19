/****************************************************************************
 * arch/arm/src/am263x/am263x_systemreset.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/am263x_rcm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Warm-reset the SoC through TOP_RCM.  The ROM then boots again in the
 *   boot mode latched at power-on: from QSPI flash in QSPI boot mode, or
 *   waiting for an image in UART boot mode.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  up_irq_save();

  putreg32(AM263X_KICK0_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK1);
  putreg32(0, AM263X_TOP_RCM_WARM_RESET_REQ);

  for (; ; )
    {
      asm volatile("wfi");
    }
}
