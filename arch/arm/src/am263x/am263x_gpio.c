/****************************************************************************
 * arch/arm/src/am263x/am263x_gpio.c
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

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <strings.h>

#include <nuttx/irq.h>

#include "arm_internal.h"
#include "am263x_gpio.h"
#include "hardware/am263x_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_REG(off)   (AM263X_GPIO0_BASE + (off))
#define GPIO_SET(pin)   ((pin) >> 5)
#define GPIO_BIT(pin)   (1u << ((pin) & 31))
#define GPIO_BANK(pin)  ((pin) >> 4)

/* The INTSTAT bits of bank b, in register set b / 2 */

#define GPIO_BANK_MASK(b) (0xffffu << (((b) & 1) * 16))

#ifdef CONFIG_AM263X_GPIO_IRQ

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_gpio_irq_s
{
  xcpt_t handler;
  void *arg;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct am263x_gpio_irq_s g_gpio_irq[AM263X_GPIO_NPINS];

/* The GPIO_INT_XBAR output each bank has, plus one; 0 for none */

static uint8_t g_gpio_bank_out[AM263X_GPIO_NBANKS];
static uint8_t g_gpio_nout;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_gpio_interrupt
 *
 * Description:
 *   A bank interrupt (a VIM pulse): clear the pins' INTSTAT bits, then run
 *   their handlers.  An edge that comes in after the read sets its bit
 *   again and raises another pulse.
 *
 ****************************************************************************/

static int am263x_gpio_interrupt(int irq, void *context, void *arg)
{
  int bank = (int)(uintptr_t)arg;
  int set = bank >> 1;
  uintptr_t intstat = GPIO_REG(AM263X_GPIO_INTSTAT(set));
  uint32_t pending;
  int pin;

  pending = getreg32(intstat) & GPIO_BANK_MASK(bank);
  putreg32(pending, intstat);

  while (pending != 0)
    {
      pin = ffs(pending) - 1;
      pending &= ~(1u << pin);
      pin += set * 32;

      if (g_gpio_irq[pin].handler != NULL)
        {
          g_gpio_irq[pin].handler(irq, context, g_gpio_irq[pin].arg);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: am263x_gpio_bank_enable
 *
 * Description:
 *   Route a bank's interrupt through the next free GPIO_INT_XBAR output to
 *   the VIM, once.  Called in a critical section.
 *
 ****************************************************************************/

static int am263x_gpio_bank_enable(int bank)
{
  int out;
  int irq;

  if (g_gpio_bank_out[bank] != 0)
    {
      return OK;
    }

  if (g_gpio_nout >= AM263X_GPIO_INTXBAR_NOUT)
    {
      return -EBUSY;
    }

  out = AM263X_GPIO_INTXBAR_FIRST + g_gpio_nout++;
  g_gpio_bank_out[bank] = out + 1;
  irq = AM263X_IRQ_GPIO_INTXBAR(out);

  putreg32(GPIO_INTXBAR_MUXCNTL_ENABLE | GPIO_INTXBAR_GPIO0_BANK(bank),
           AM263X_GPIO_INTXBAR_MUXCNTL(out));
  modifyreg32(GPIO_REG(AM263X_GPIO_BINTEN), 0, 1u << bank);

  irq_attach(irq, am263x_gpio_interrupt, (void *)(uintptr_t)bank);
  up_set_irq_type(irq, IRQ_RISING_EDGE);
  up_enable_irq(irq);
  return OK;
}

#endif /* CONFIG_AM263X_GPIO_IRQ */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_gpio_write
 ****************************************************************************/

void am263x_gpio_write(int pin, bool value)
{
  DEBUGASSERT(pin >= 0 && pin < AM263X_GPIO_NPINS);

  /* SET_DATA and CLR_DATA change only the bits written as one */

  putreg32(GPIO_BIT(pin),
           GPIO_REG(value ? AM263X_GPIO_SET_DATA(GPIO_SET(pin)) :
                            AM263X_GPIO_CLR_DATA(GPIO_SET(pin))));
}

/****************************************************************************
 * Name: am263x_gpio_read
 ****************************************************************************/

bool am263x_gpio_read(int pin)
{
  DEBUGASSERT(pin >= 0 && pin < AM263X_GPIO_NPINS);

  return (getreg32(GPIO_REG(AM263X_GPIO_IN_DATA(GPIO_SET(pin)))) &
          GPIO_BIT(pin)) != 0;
}

/****************************************************************************
 * Name: am263x_gpio_output
 *
 * Description:
 *   Drive the pin, at value from the start.
 *
 ****************************************************************************/

void am263x_gpio_output(int pin, bool value)
{
  irqstate_t flags;

  am263x_gpio_write(pin, value);

  flags = enter_critical_section();
  modifyreg32(GPIO_REG(AM263X_GPIO_DIR(GPIO_SET(pin))), GPIO_BIT(pin), 0);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: am263x_gpio_input
 ****************************************************************************/

void am263x_gpio_input(int pin)
{
  irqstate_t flags;

  DEBUGASSERT(pin >= 0 && pin < AM263X_GPIO_NPINS);

  flags = enter_critical_section();
  modifyreg32(GPIO_REG(AM263X_GPIO_DIR(GPIO_SET(pin))), 0, GPIO_BIT(pin));
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: am263x_gpio_irq
 *
 * Description:
 *   Call handler(irq, context, arg) on the given edges of an input pin, or
 *   with edges 0 or a NULL handler, stop.
 *
 ****************************************************************************/

#ifdef CONFIG_AM263X_GPIO_IRQ
int am263x_gpio_irq(int pin, int edges, xcpt_t handler, void *arg)
{
  irqstate_t flags;
  int set = GPIO_SET(pin);
  uint32_t bit = GPIO_BIT(pin);
  int ret = OK;

  if (pin < 0 || pin >= AM263X_GPIO_NPINS || (edges & ~AM263X_GPIO_BOTH))
    {
      return -EINVAL;
    }

  if (handler == NULL)
    {
      edges = 0;
    }

  flags = enter_critical_section();

  /* Stop the pin first, so that the handler never runs half-updated */

  putreg32(bit, GPIO_REG(AM263X_GPIO_CLR_RIS_TRIG(set)));
  putreg32(bit, GPIO_REG(AM263X_GPIO_CLR_FAL_TRIG(set)));
  putreg32(bit, GPIO_REG(AM263X_GPIO_INTSTAT(set)));

  g_gpio_irq[pin].handler = handler;
  g_gpio_irq[pin].arg     = arg;

  if (edges != 0)
    {
      ret = am263x_gpio_bank_enable(GPIO_BANK(pin));
      if (ret < 0)
        {
          g_gpio_irq[pin].handler = NULL;
        }
      else
        {
          if (edges & AM263X_GPIO_RISING)
            {
              putreg32(bit, GPIO_REG(AM263X_GPIO_SET_RIS_TRIG(set)));
            }

          if (edges & AM263X_GPIO_FALLING)
            {
              putreg32(bit, GPIO_REG(AM263X_GPIO_SET_FAL_TRIG(set)));
            }
        }
    }

  leave_critical_section(flags);
  return ret;
}
#endif
