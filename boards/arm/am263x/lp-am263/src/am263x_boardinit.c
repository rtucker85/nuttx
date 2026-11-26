/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_boardinit.c
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

#include <nuttx/board.h>

#include "am263x_boot.h"
#include "am263x_pinmux.h"
#include "lp-am263.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART0 (XDS110 virtual COM port): pads UART0_RXD/TXD, mux mode 0 */

static const struct am263x_pinmux_s g_console_pins[] =
{
  { AM263X_PAD_UART0_RXD, PAD_MODE(0) | PAD_PULL_UP | PAD_SLEW_LOW },
  { AM263X_PAD_UART0_TXD, PAD_MODE(0) | PAD_PULL_DISABLE | PAD_SLEW_LOW },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_board_initialize
 ****************************************************************************/

void am263x_board_initialize(void)
{
  am263x_pinmux_config(g_console_pins,
                       sizeof(g_console_pins) / sizeof(g_console_pins[0]));

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  am263x_bringup();
}
#endif
