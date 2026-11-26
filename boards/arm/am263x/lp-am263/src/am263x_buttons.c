/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_buttons.c
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

/* The LaunchPad's user push button on GPIO123.  The other two buttons are
 * PORz and the warm reset, wired straight to the SoC.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "am263x_gpio.h"
#include "am263x_pinmux.h"
#include "lp-am263.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_BUTTON_USER    123

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The pull-up is on the board */

static const struct am263x_pinmux_s g_button_pins[] =
{
  { AM263X_PAD_SDFM0_D0, PAD_MODE(7) | PAD_PULL_DISABLE },  /* GPIO123 */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  am263x_pinmux_config(g_button_pins,
                       sizeof(g_button_pins) / sizeof(g_button_pins[0]));
  am263x_gpio_input(GPIO_BUTTON_USER);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return am263x_gpio_read(GPIO_BUTTON_USER) ? 0 : BUTTON_USER_BIT;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   Call irqhandler on every press and release of the button, or stop with
 *   a NULL irqhandler.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  if (id != BUTTON_USER)
    {
      return -EINVAL;
    }

  return am263x_gpio_irq(GPIO_BUTTON_USER, AM263X_GPIO_BOTH,
                         irqhandler, arg);
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
