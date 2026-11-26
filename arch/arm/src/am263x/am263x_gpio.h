/****************************************************************************
 * arch/arm/src/am263x/am263x_gpio.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_GPIO_H
#define __ARCH_ARM_SRC_AM263X_AM263X_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Edges for am263x_gpio_irq() */

#define AM263X_GPIO_RISING          (1 << 0)
#define AM263X_GPIO_FALLING         (1 << 1)
#define AM263X_GPIO_BOTH            (AM263X_GPIO_RISING | AM263X_GPIO_FALLING)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Pins of this core's GPIO instance (GPIO0).  The pad must be muxed to
 * GPIO (mode 7) with GPIO_SEL 0, see am263x_pinmux_config().
 */

void am263x_gpio_output(int pin, bool value);
void am263x_gpio_input(int pin);
void am263x_gpio_write(int pin, bool value);
bool am263x_gpio_read(int pin);

/****************************************************************************
 * Name: am263x_gpio_irq
 *
 * Description:
 *   Call handler(irq, context, arg) from the GPIO bank interrupt on the
 *   given edges of an input pin.  edges 0 or a NULL handler turns the
 *   pin's interrupt off.
 *
 * Returned Value:
 *   Zero (OK) on success; -EINVAL for a bad pin or edges, -EBUSY if the
 *   pin's bank needs a GPIO_INT_XBAR output and all four are taken.
 *
 ****************************************************************************/

#ifdef CONFIG_AM263X_GPIO_IRQ
int am263x_gpio_irq(int pin, int edges, xcpt_t handler, void *arg);
#endif

#endif /* __ARCH_ARM_SRC_AM263X_AM263X_GPIO_H */
