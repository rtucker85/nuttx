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
#include <stdbool.h>
#include <stdint.h>

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
