/****************************************************************************
 * arch/arm/src/am263x/am263x_serial.c
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

#include <nuttx/serial/uart_16550.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The AM263x UARTs are TI 16550-compatible UARTs with 32-bit registers
 * (CONFIG_16550_REGWIDTH=32), driven by the common uart_16550 driver.  One
 * TI extension matters: MDR1 selects the mode, and the TRM requires the
 * UART to be disabled (MDR1 = 7) while the divisor latches are written.
 * uart_putreg() below does that whenever the driver sets LCR.DLAB, and
 * selects 16x UART mode again when DLAB is cleared.
 *
 * In FIFO mode the TI THR interrupt stays pending while the TX FIFO is
 * below its trigger level, but the driver refills it only when LSR.THRE
 * shows it empty, so the interrupt would fire again and again until the
 * FIFO drains.  SCR.TX_EMPTY_CTL_IT makes it fire only when the FIFO and
 * the shift register are empty, as on a 16550.  uart_putreg() sets it
 * when the driver sets up the FIFOs.
 */

#define TI_UART_MDR1_OFFSET     0x20
#define TI_UART_MDR1_16X        0
#define TI_UART_MDR1_DISABLE    7

#define TI_UART_SCR_OFFSET      0x40
#define TI_UART_SCR_TXEMPTYIT   (1 << 3)

#define TI_UART_LCR_OFFSET      (UART_LCR_OFFSET * sizeof(uart_datawidth_t))
#define TI_UART_FCR_OFFSET      (UART_FCR_OFFSET * sizeof(uart_datawidth_t))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_16550_UART

/****************************************************************************
 * Name: uart_getreg
 ****************************************************************************/

uart_datawidth_t uart_getreg(struct u16550_s *priv, unsigned int offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: uart_putreg
 ****************************************************************************/

void uart_putreg(struct u16550_s *priv, unsigned int offset,
                 uart_datawidth_t value)
{
  uintptr_t mdr1 = priv->uartbase + TI_UART_MDR1_OFFSET;

  if (offset == TI_UART_LCR_OFFSET && (value & UART_LCR_DLAB) != 0)
    {
      putreg32(TI_UART_MDR1_DISABLE, mdr1);
      putreg32(value, priv->uartbase + offset);
      return;
    }

  putreg32(value, priv->uartbase + offset);

  if (offset == TI_UART_FCR_OFFSET)
    {
      modifyreg32(priv->uartbase + TI_UART_SCR_OFFSET, 0,
                  TI_UART_SCR_TXEMPTYIT);
    }

  if (offset == TI_UART_LCR_OFFSET &&
      getreg32(mdr1) == TI_UART_MDR1_DISABLE)
    {
      putreg32(TI_UART_MDR1_16X, mdr1);
    }
}

#ifdef USE_EARLYSERIALINIT

/****************************************************************************
 * Name: arm_earlyserialinit
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  u16550_earlyserialinit();
}
#endif

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Name: arm_serialinit
 ****************************************************************************/

void arm_serialinit(void)
{
  u16550_serialinit();
}
#endif

#endif /* CONFIG_16550_UART */
