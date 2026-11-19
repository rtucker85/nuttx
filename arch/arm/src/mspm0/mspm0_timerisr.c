/****************************************************************************
 * arch/arm/src/mspm0/mspm0_timerisr.c
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
#include <time.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SysTick counts the CPU clock.  CLK_TCK is the number of system ticks
 * per second (from CONFIG_USEC_PER_TICK).
 */

#define SYSTICK_RELOAD ((BOARD_SYSTICK_FREQUENCY / CLK_TCK) - 1)

/* The reload value is 24 bits */

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mspm0_timerisr
 *
 * Description:
 *   The SysTick interrupt: advance the system timer.
 *
 ****************************************************************************/

static int mspm0_timerisr(int irq, void *context, void *arg)
{
  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Called during start-up to set up the system timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Set the SysTick interrupt to the default priority */

  modifyreg32(ARMV6M_SYSCON_SHPR3, SYSCON_SHPR3_PRI_15_MASK,
              NVIC_SYSH_PRIORITY_DEFAULT << SYSCON_SHPR3_PRI_15_SHIFT);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, ARMV6M_SYSTICK_RVR);
  putreg32(0, ARMV6M_SYSTICK_CVR);

  irq_attach(MSPM0_IRQ_SYSTICK, mspm0_timerisr, NULL);

  /* Count the CPU clock and enable the counter and its interrupt */

  putreg32(SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT | SYSTICK_CSR_ENABLE,
           ARMV6M_SYSTICK_CSR);
}
