/****************************************************************************
 * boards/arm/am263x/lp-am263/include/board.h
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

#ifndef __BOARDS_ARM_AM263X_LP_AM263_INCLUDE_BOARD_H
#define __BOARDS_ARM_AM263X_LP_AM263_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking.  The SBL sets up the PLLs with TI's defaults:
 * R5F 400 MHz, SYS_CLK 200 MHz, 25 MHz crystal.
 */

#define BOARD_XTAL_FREQUENCY        25000000
#define BOARD_CPU_FREQUENCY         400000000
#define BOARD_SYSCLK_FREQUENCY      200000000

/* Console: UART0, wired to the XDS110 USB virtual COM port */

/* LEDs.  The user LED on GPIO26 (UART0_CTSn pad, high is on) is LED 0 of
 * /dev/userleds, or, with CONFIG_ARCH_LEDS, shows NuttX's state.  The
 * eight green LEDs on the TPIC2810 LED driver (I2C1 at 0x61) are
 * /dev/userleds1, with CONFIG_TPIC2810.
 */

#define BOARD_LED_USER              0
#define BOARD_NLEDS                 1
#define BOARD_LED_USER_BIT          (1 << BOARD_LED_USER)

#define BOARD_TPIC2810_I2C_BUS      1
#define BOARD_TPIC2810_I2C_ADDR     0x61

/* CONFIG_ARCH_LEDS:
 *
 *   LED_STARTED .. LED_IRQSENABLED   off
 *   LED_STACKCREATED                 on: the OS is running
 *   LED_INIRQ, LED_SIGNAL, ...       no change
 *   LED_PANIC                        blinking
 */

#define LED_STARTED                 0
#define LED_HEAPALLOCATE            0
#define LED_IRQSENABLED             0
#define LED_STACKCREATED            1
#define LED_INIRQ                   2
#define LED_SIGNAL                  2
#define LED_ASSERTION               2
#define LED_PANIC                   1

/* Buttons.  The user push button on GPIO123 (SDFM0_D0 pad), pulled up on
 * the board, low while pressed.
 */

#define BUTTON_USER                 0
#define NUM_BUTTONS                 1
#define BUTTON_USER_BIT             (1 << BUTTON_USER)

#endif /* __BOARDS_ARM_AM263X_LP_AM263_INCLUDE_BOARD_H */
