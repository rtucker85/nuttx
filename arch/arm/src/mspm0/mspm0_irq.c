/****************************************************************************
 * arch/arm/src/mspm0/mspm0_irq.c
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
#include <inttypes.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "nvic.h"
#include "ram_vectors.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default priority of all interrupts and exceptions, four per register */

#define DEFPRIORITY32 \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 | NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
   NVIC_SYSH_PRIORITY_DEFAULT << 8  | NVIC_SYSH_PRIORITY_DEFAULT)

/* Bit in the NVIC registers for an external interrupt */

#define NVIC_BIT(irq) (1 << ((irq) - MSPM0_IRQ_EXTINT))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mspm0_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_IRQ_INFO
static void mspm0_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  ISER:     %08" PRIx32 " ICER: %08" PRIx32 "\n",
          getreg32(ARMV6M_NVIC_ISER), getreg32(ARMV6M_NVIC_ICER));
  irqinfo("  ISPR:     %08" PRIx32 " ICPR: %08" PRIx32 "\n",
          getreg32(ARMV6M_NVIC_ISPR), getreg32(ARMV6M_NVIC_ICPR));
  irqinfo("  IRQ PRIO: %08" PRIx32 " %08" PRIx32 " %08" PRIx32
          " %08" PRIx32 "\n",
          getreg32(ARMV6M_NVIC_IPR0), getreg32(ARMV6M_NVIC_IPR1),
          getreg32(ARMV6M_NVIC_IPR2), getreg32(ARMV6M_NVIC_IPR3));
  irqinfo("            %08" PRIx32 " %08" PRIx32 " %08" PRIx32
          " %08" PRIx32 "\n",
          getreg32(ARMV6M_NVIC_IPR4), getreg32(ARMV6M_NVIC_IPR5),
          getreg32(ARMV6M_NVIC_IPR6), getreg32(ARMV6M_NVIC_IPR7));
  irqinfo("  SHPR2:    %08" PRIx32 " SHPR3: %08" PRIx32 "\n",
          getreg32(ARMV6M_SYSCON_SHPR2), getreg32(ARMV6M_SYSCON_SHPR3));

  leave_critical_section(flags);
}
#else
#  define mspm0_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: mspm0_nmi, mspm0_pendsv, mspm0_reserved
 *
 * Description:
 *   Handlers for exceptions that are never expected.  They are all fatal;
 *   these only add a diagnostic message over the default handler.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
static int mspm0_nmi(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! NMI received\n");
  PANIC();
  return 0;
}

static int mspm0_pendsv(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! PendSV received\n");
  PANIC();
  return 0;
}

static int mspm0_reserved(int irq, void *context, void *arg)
{
  up_irq_save();
  _err("PANIC!!! Reserved interrupt\n");
  PANIC();
  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  int i;

  /* Disable all interrupts and stop SysTick */

  putreg32(0xffffffff, ARMV6M_NVIC_ICER);
  putreg32(0, ARMV6M_SYSTICK_CSR);

  /* Set all exceptions and interrupt lines to the default priority */

  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR2);
  putreg32(DEFPRIORITY32, ARMV6M_SYSCON_SHPR3);

  for (i = 0; i < 8; i++)
    {
      putreg32(DEFPRIORITY32, ARMV6M_NVIC_IPR(i));
    }

  /* Point VTOR at our vector table.  It is at 0x0000:0000 unless the image
   * is linked to run behind a bootloader.
   */

  putreg32((uint32_t)_vectors, ARMV6M_SYSCON_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  arm_ramvec_initialize();
#endif

  /* Attach the SVCall and Hard Fault exception handlers.  SVCall is used
   * for context switches.  Hard Fault must also be caught because an
   * SVCall may show up as a Hard Fault under certain conditions.
   */

  irq_attach(MSPM0_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(MSPM0_IRQ_HARDFAULT, arm_hardfault, NULL);

#ifdef CONFIG_DEBUG_FEATURES
  irq_attach(MSPM0_IRQ_NMI, mspm0_nmi, NULL);
  irq_attach(MSPM0_IRQ_PENDSV, mspm0_pendsv, NULL);
  irq_attach(MSPM0_IRQ_RESERVED, mspm0_reserved, NULL);
#endif

  mspm0_dumpnvic("initial", NR_IRQS);

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  arm_color_intstack();
  up_irq_enable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (irq >= MSPM0_IRQ_EXTINT)
    {
      putreg32(NVIC_BIT(irq), ARMV6M_NVIC_ICER);
    }
  else if (irq == MSPM0_IRQ_SYSTICK)
    {
      /* SysTick is the only processor exception that can be disabled */

      modifyreg32(ARMV6M_SYSTICK_CSR, SYSTICK_CSR_TICKINT, 0);
    }

  mspm0_dumpnvic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  DEBUGASSERT((unsigned)irq < NR_IRQS);

  if (irq >= MSPM0_IRQ_EXTINT)
    {
      putreg32(NVIC_BIT(irq), ARMV6M_NVIC_ISER);
    }
  else if (irq == MSPM0_IRQ_SYSTICK)
    {
      modifyreg32(ARMV6M_SYSTICK_CSR, 0, SYSTICK_CSR_TICKINT);
    }

  mspm0_dumpnvic("enable", irq);
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ.  Clear it at the NVIC so a level that has already
 *   been cleared at the peripheral does not fire again.
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
  if (irq >= MSPM0_IRQ_EXTINT)
    {
      putreg32(NVIC_BIT(irq), ARMV6M_NVIC_ICPR);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.  Only the top two bits of the priority are
 *   implemented.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  int shift;

  DEBUGASSERT(priority >= NVIC_SYSH_PRIORITY_MAX &&
              priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq >= MSPM0_IRQ_EXTINT && irq < NR_IRQS)
    {
      /* IPR0-IPR7 hold four 8-bit priorities each */

      regaddr = ARMV6M_NVIC_IPR((irq - MSPM0_IRQ_EXTINT) >> 2);
      shift   = ((irq - MSPM0_IRQ_EXTINT) & 3) << 3;
    }
  else if (irq == MSPM0_IRQ_PENDSV)
    {
      regaddr = ARMV6M_SYSCON_SHPR3;
      shift   = SYSCON_SHPR3_PRI_14_SHIFT;
    }
  else if (irq == MSPM0_IRQ_SYSTICK)
    {
      regaddr = ARMV6M_SYSCON_SHPR3;
      shift   = SYSCON_SHPR3_PRI_15_SHIFT;
    }
  else
    {
      /* SVCall must keep its priority; the rest are fixed */

      return -EINVAL;
    }

  modifyreg32(regaddr, (uint32_t)0xff << shift, (uint32_t)priority << shift);

  mspm0_dumpnvic("prioritize", irq);
  return OK;
}
#endif
