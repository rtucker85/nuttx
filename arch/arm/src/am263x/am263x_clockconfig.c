/****************************************************************************
 * arch/arm/src/am263x/am263x_clockconfig.c
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

#include <stdint.h>

#include "arm_internal.h"
#include "hardware/am263x_rcm.h"
#include "am263x_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_set_clock
 *
 * Description:
 *   Program a peripheral clock: divider first, then the source, waiting for
 *   each to read back (the order used by TI's reference code), then ungate.
 *
 ****************************************************************************/

static void am263x_set_clock(uintptr_t srcsel, uintptr_t divval,
                             uintptr_t gate, uint32_t src, uint32_t div)
{
  putreg32(div, divval);
  while (getreg32(divval) != div)
    {
    }

  putreg32(src, srcsel);
  while (getreg32(srcsel) != src)
    {
    }

  putreg32(RCM_CLK_UNGATED, gate);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_clockconfig
 ****************************************************************************/

void am263x_clockconfig(void)
{
  putreg32(AM263X_KICK0_UNLOCK, AM263X_MSS_RCM_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_MSS_RCM_LOCK0_KICK1);

  /* UART0: PER_HSDIV0_CLKOUT1 (192 MHz) / 4 = 48 MHz */

  am263x_set_clock(AM263X_MSS_RCM_UART0_CLK_SRC_SEL,
                   AM263X_MSS_RCM_UART0_CLK_DIV_VAL,
                   AM263X_MSS_RCM_UART0_CLK_GATE,
                   RCM_UART_CLKSRC_PER_HSDIV0_CLKOUT1, RCM_CLK_DIV(4));

  /* RTI0: the 25 MHz crystal, so the tick does not depend on the PLLs */

  am263x_set_clock(AM263X_MSS_RCM_RTI0_CLK_SRC_SEL,
                   AM263X_MSS_RCM_RTI0_CLK_DIV_VAL,
                   AM263X_MSS_RCM_RTI0_CLK_GATE,
                   RCM_RTI_CLKSRC_XTAL, RCM_CLK_DIV(1));
}
