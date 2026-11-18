/****************************************************************************
 * arch/arm/src/am263x/am263x_irq.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/am263x_vim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VIM_REG(off)    (AM263X_VIM_BASE + (off))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool am263x_irq_is_pulse(int irq)
{
  return (getreg32(VIM_REG(VIM_INTTYPE_OFFSET(irq))) & VIM_BIT(irq)) != 0;
}

static inline void am263x_irq_clear(int irq)
{
  putreg32(VIM_BIT(irq), VIM_REG(VIM_STS_OFFSET(irq)));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Put the VIM into a known state: every line disabled, cleared, level
 *   type, routed to IRQ (not FIQ), lowest priority.  The VIM is used in
 *   non-vectored mode: the core takes the IRQ exception through the
 *   NuttX vector table and arm_decodeirq() reads the active line.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;

  for (i = 0; i < NR_IRQS; i += 32)
    {
      putreg32(0xffffffff, VIM_REG(VIM_INTDIS_OFFSET(i)));
      putreg32(0xffffffff, VIM_REG(VIM_STS_OFFSET(i)));
      putreg32(0, VIM_REG(VIM_INTTYPE_OFFSET(i)));
      putreg32(0, VIM_REG(VIM_INTMAP_OFFSET(i)));
    }

  for (i = 0; i < NR_IRQS; i++)
    {
      putreg32(VIM_PRI_LOWEST, VIM_REG(VIM_PRI_OFFSET(i)));
      putreg32((uint32_t)(uintptr_t)arm_vectorirq & ~3u,
               VIM_REG(VIM_VEC_OFFSET(i)));
    }

  /* Drop anything the SBL left active: reading IRQVEC latches the active
   * interrupt, writing it acknowledges.
   */

  getreg32(VIM_REG(VIM_IRQVEC_OFFSET));
  putreg32(0, VIM_REG(VIM_IRQVEC_OFFSET));

  /* Paint the interrupt stack, for stack usage monitoring */

  arm_color_intstack();

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: arm_decodeirq
 *
 * Description:
 *   Called from the IRQ exception handler.  Same sequence as TI's
 *   reference handler: read IRQVEC to make the VIM resolve priorities,
 *   read ACTIRQ for the line, clear a pulse source before running its
 *   handler and a level source after it (so a still-asserted level source
 *   re-latches), then acknowledge by writing IRQVEC.
 *
 ****************************************************************************/

uint32_t *arm_decodeirq(uint32_t *regs)
{
  uint32_t active;
  int irq;
  bool pulse;

  getreg32(VIM_REG(VIM_IRQVEC_OFFSET));
  active = getreg32(VIM_REG(VIM_ACTIRQ_OFFSET));

  if ((active & VIM_ACTIRQ_VALID) == 0)
    {
      /* Spurious: nothing active by the time we looked */

      putreg32(0, VIM_REG(VIM_IRQVEC_OFFSET));
      return regs;
    }

  irq = active & VIM_ACTIRQ_NUM_MASK;
  pulse = am263x_irq_is_pulse(irq);

  if (pulse)
    {
      am263x_irq_clear(irq);
    }

  regs = arm_doirq(irq, regs);

  if (!pulse)
    {
      am263x_irq_clear(irq);
    }

  putreg32(irq, VIM_REG(VIM_IRQVEC_OFFSET));
  return regs;
}

/****************************************************************************
 * Name: up_enable_irq
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq >= 0 && irq < NR_IRQS)
    {
      UP_MB();
      putreg32(VIM_BIT(irq), VIM_REG(VIM_INTEN_OFFSET(irq)));
    }
}

/****************************************************************************
 * Name: up_disable_irq
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if (irq >= 0 && irq < NR_IRQS)
    {
      putreg32(VIM_BIT(irq), VIM_REG(VIM_INTDIS_OFFSET(irq)));
      UP_MB();
    }
}

/****************************************************************************
 * Name: up_set_irq_type
 *
 * Description:
 *   Edge modes make the line a VIM pulse interrupt, level modes a level
 *   interrupt.  Each peripheral's output is one or the other; the TRM
 *   (and TI's SDK configuration) says which.
 *
 ****************************************************************************/

int up_set_irq_type(int irq, int mode)
{
  uintptr_t reg;
  irqstate_t flags;

  if (irq < 0 || irq >= NR_IRQS)
    {
      return -EINVAL;
    }

  reg = VIM_REG(VIM_INTTYPE_OFFSET(irq));
  flags = enter_critical_section();

  switch (mode)
    {
      case IRQ_RISING_EDGE:
      case IRQ_FALLING_EDGE:
      case IRQ_BOTH_EDGE:
        modifyreg32(reg, 0, VIM_BIT(irq));
        break;

      case IRQ_HIGH_LEVEL:
      case IRQ_LOW_LEVEL:
        modifyreg32(reg, VIM_BIT(irq), 0);
        break;

      default:
        leave_critical_section(flags);
        return -EINVAL;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the VIM priority of a line: 0 is highest, 15 lowest.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  if (irq < 0 || irq >= NR_IRQS || priority < 0 ||
      priority > VIM_PRI_LOWEST)
    {
      return -EINVAL;
    }

  putreg32(priority, VIM_REG(VIM_PRI_OFFSET(irq)));
  return OK;
}
#endif

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Not supported by the VIM; present because ARCH_HAVE_IRQTRIGGER is
 *   selected for up_set_irq_type().
 *
 ****************************************************************************/

void up_trigger_irq(int irq, cpu_set_t cpuset)
{
  UNUSED(irq);
  UNUSED(cpuset);
}
