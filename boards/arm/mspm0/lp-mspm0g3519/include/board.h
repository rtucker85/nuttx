/****************************************************************************
 * boards/arm/mspm0/lp-mspm0g3519/include/board.h
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

#ifndef __BOARDS_ARM_MSPM0_LP_MSPM0G3519_INCLUDE_BOARD_H
#define __BOARDS_ARM_MSPM0_LP_MSPM0G3519_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The chip runs from SYSOSC at its reset frequency.  MCLK feeds the CPU and
 * the PD1 peripherals; ULPCLK (MCLK / 1) feeds the PD0 peripherals.
 */

#define BOARD_SYSOSC_FREQUENCY  32000000
#define BOARD_MCLK_FREQUENCY    BOARD_SYSOSC_FREQUENCY
#define BOARD_ULPCLK_FREQUENCY  BOARD_MCLK_FREQUENCY

/* SysTick runs from the CPU clock */

#define BOARD_SYSTICK_FREQUENCY BOARD_MCLK_FREQUENCY

/* Pin selections ***********************************************************/

/* UART0 is the XDS110 back-channel UART: PA10 (PINCM21) TX and
 * PA11 (PINCM22) RX, peripheral function 2.
 */

#define BOARD_UART0_TX_PINCM    21
#define BOARD_UART0_TX_PF       2
#define BOARD_UART0_RX_PINCM    22
#define BOARD_UART0_RX_PF       2

#endif /* __BOARDS_ARM_MSPM0_LP_MSPM0G3519_INCLUDE_BOARD_H */
