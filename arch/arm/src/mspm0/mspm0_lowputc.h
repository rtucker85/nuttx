/****************************************************************************
 * arch/arm/src/mspm0/mspm0_lowputc.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_MSPM0_LOWPUTC_H
#define __ARCH_ARM_SRC_MSPM0_MSPM0_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which UART, if any, is the console */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_MSPM0_UART0)
#  define HAVE_SERIAL_CONSOLE 1
#  define CONSOLE_BASE        MSPM0_UART0_BASE
#  define CONSOLE_CLOCK       BOARD_ULPCLK_FREQUENCY
#  define CONSOLE_BAUD        CONFIG_UART0_BAUD
#  define CONSOLE_BITS        CONFIG_UART0_BITS
#  define CONSOLE_PARITY      CONFIG_UART0_PARITY
#  define CONSOLE_2STOP       CONFIG_UART0_2STOP
#  define CONSOLE_TX_PINCM    BOARD_UART0_TX_PINCM
#  define CONSOLE_TX_PF       BOARD_UART0_TX_PF
#  define CONSOLE_RX_PINCM    BOARD_UART0_RX_PINCM
#  define CONSOLE_RX_PF       BOARD_UART0_RX_PF
#else
#  undef HAVE_SERIAL_CONSOLE
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct mspm0_uartcfg_s
{
  uint32_t baud;    /* Baud rate */
  uint8_t  bits;    /* Data bits, 5-8 */
  uint8_t  parity;  /* 0 = none, 1 = odd, 2 = even */
  bool     stop2;   /* Two stop bits */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: mspm0_lowsetup
 *
 * Description:
 *   Set up the console UART early in start-up so that arm_lowputc() works
 *   before the serial driver is running.
 *
 ****************************************************************************/

void mspm0_lowsetup(void);

/****************************************************************************
 * Name: mspm0_uart_pins
 *
 * Description:
 *   Connect a UART's TX and RX pins.  RX gets a pull-up so that it idles
 *   high when nothing is connected.
 *
 ****************************************************************************/

void mspm0_uart_pins(unsigned int txpincm, unsigned int txpf,
                     unsigned int rxpincm, unsigned int rxpf);

/****************************************************************************
 * Name: mspm0_uart_configure
 *
 * Description:
 *   Power up the UART if needed and set the format and baud rate.  The
 *   UART is left enabled with its FIFOs on and all interrupts masked.
 *
 * Input Parameters:
 *   base  - UART base address
 *   clock - Bus clock that feeds the UART, in Hz
 *   cfg   - Line settings
 *
 ****************************************************************************/

void mspm0_uart_configure(uintptr_t base, uint32_t clock,
                          const struct mspm0_uartcfg_s *cfg);

/****************************************************************************
 * Name: mspm0_uart_disable
 *
 * Description:
 *   Wait for the transmitter to finish, then disable the UART.
 *
 ****************************************************************************/

void mspm0_uart_disable(uintptr_t base);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MSPM0_MSPM0_LOWPUTC_H */
