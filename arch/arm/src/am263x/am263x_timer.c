/****************************************************************************
 * arch/arm/src/am263x/am263x_timer.c
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
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/am263x_rti.h"
#include "am263x_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTI0 counter block 0 is prescaled to 1 MHz, so FRC0 counts
 * microseconds; compare 0 then fires every CONFIG_USEC_PER_TICK counts.
 */

#define RTI_BASE            AM263X_RTI0_BASE
#define RTI_COUNT_HZ        1000000
#define RTI_PRESCALE        (AM263X_RTI0_FREQUENCY / RTI_COUNT_HZ)

#if (AM263X_RTI0_FREQUENCY % RTI_COUNT_HZ) != 0
#  error "RTI0 clock must be a whole number of MHz"
#endif

#define RTI_REG(off)        (RTI_BASE + (off))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_timerisr
 ****************************************************************************/

static int am263x_timerisr(int irq, void *context, void *arg)
{
  putreg32(RTI_INT(0), RTI_REG(RTI_INTFLAG_OFFSET));
  nxsched_process_timer();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Start the periodic system tick on RTI0 compare 0 (VIM line 84, a
 *   pulse interrupt).  The SBL may have left RTI0 running for its own
 *   tick, so it is stopped and fully reprogrammed.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  modifyreg32(RTI_REG(RTI_GCTRL_OFFSET), RTI_GCTRL_CNT0EN, 0);
  putreg32(RTI_INT(0) | RTI_INT(1) | RTI_INT(2) | RTI_INT(3),
           RTI_REG(RTI_CLEARINT_OFFSET));
  putreg32(RTI_INT(0) | RTI_INT(1) | RTI_INT(2) | RTI_INT(3),
           RTI_REG(RTI_INTFLAG_OFFSET));

  putreg32(0, RTI_REG(RTI_UC0_OFFSET));
  putreg32(0, RTI_REG(RTI_FRC0_OFFSET));
  putreg32(RTI_PRESCALE - 1, RTI_REG(RTI_CPUC0_OFFSET));
  putreg32(0, RTI_REG(RTI_COMPCTRL_OFFSET));  /* compare 0 uses FRC0 */
  putreg32(CONFIG_USEC_PER_TICK, RTI_REG(RTI_COMP0_OFFSET));
  putreg32(CONFIG_USEC_PER_TICK, RTI_REG(RTI_UDCP0_OFFSET));

  up_set_irq_type(AM263X_IRQ_RTI0_0, IRQ_RISING_EDGE);
  DEBUGVERIFY(irq_attach(AM263X_IRQ_RTI0_0,
                         am263x_timerisr, NULL));

  putreg32(RTI_INT(0), RTI_REG(RTI_SETINT_OFFSET));
  modifyreg32(RTI_REG(RTI_GCTRL_OFFSET), 0, RTI_GCTRL_CNT0EN);

  up_enable_irq(AM263X_IRQ_RTI0_0);
}
