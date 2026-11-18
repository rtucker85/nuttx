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

#endif /* __BOARDS_ARM_AM263X_LP_AM263_INCLUDE_BOARD_H */
