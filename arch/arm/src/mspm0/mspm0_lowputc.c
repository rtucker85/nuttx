/****************************************************************************
 * arch/arm/src/mspm0/mspm0_lowputc.c
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

#include "arm_internal.h"
#include "chip.h"
#include "hardware/mspm0_iomux.h"
#include "hardware/mspm0_uart.h"
#include "mspm0_periph.h"
#include "mspm0_lowputc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the console UART, waiting for FIFO space.
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  while ((getreg32(CONSOLE_BASE + MSPM0_UART_STAT_OFFSET) &
          UART_STAT_TXFF) != 0)
    {
    }

  putreg32((uint32_t)ch, CONSOLE_BASE + MSPM0_UART_TXDATA_OFFSET);
#endif
}

/****************************************************************************
 * Name: mspm0_uart_pins
 ****************************************************************************/

void mspm0_uart_pins(unsigned int txpincm, unsigned int txpf,
                     unsigned int rxpincm, unsigned int rxpf)
{
  putreg32(IOMUX_PINCM_PC | IOMUX_PINCM_PF(txpf),
           MSPM0_IOMUX_PINCM(txpincm));
  putreg32(IOMUX_PINCM_PC | IOMUX_PINCM_INENA | IOMUX_PINCM_PIPU |
           IOMUX_PINCM_PF(rxpf), MSPM0_IOMUX_PINCM(rxpincm));
}

/****************************************************************************
 * Name: mspm0_uart_disable
 ****************************************************************************/

void mspm0_uart_disable(uintptr_t base)
{
  if (!mspm0_periph_powered(base))
    {
      return;
    }

  while ((getreg32(base + MSPM0_UART_STAT_OFFSET) & UART_STAT_BUSY) != 0)
    {
    }

  modifyreg32(base + MSPM0_UART_CTL0_OFFSET, UART_CTL0_ENABLE, 0);
}

/****************************************************************************
 * Name: mspm0_uart_configure
 ****************************************************************************/

void mspm0_uart_configure(uintptr_t base, uint32_t clock,
                          const struct mspm0_uartcfg_s *cfg)
{
  uint32_t brd64;
  uint32_t lcrh;

  /* The UART must be disabled while it is configured */

  if (mspm0_periph_powered(base))
    {
      mspm0_uart_disable(base);
    }
  else
    {
      mspm0_periph_enable(base);
    }

  /* Run from the bus clock, undivided */

  putreg32(UART_CLKSEL_BUSCLK, base + MSPM0_CLKSEL_OFFSET);
  putreg32(UART_CLKDIV_RATIO(1), base + MSPM0_CLKDIV_OFFSET);

  /* Mask and clear all interrupts */

  putreg32(0, base + MSPM0_CPU_INT_IMASK_OFFSET);
  putreg32(0xffffffff, base + MSPM0_CPU_INT_ICLR_OFFSET);

  /* Divisor with 16x oversampling: clock / (16 * baud), with six bits of
   * fraction.  That is (clock * 4) / baud in 64ths, rounded.  clock * 4
   * fits in 32 bits for any clock up to 1 GHz.
   */

  brd64 = ((clock << 2) + (cfg->baud >> 1)) / cfg->baud;
  putreg32((brd64 >> 6) & UART_IBRD_MASK, base + MSPM0_UART_IBRD_OFFSET);
  putreg32(brd64 & UART_FBRD_MASK, base + MSPM0_UART_FBRD_OFFSET);

  /* Line control.  Writing LCRH also latches the new divisor. */

  lcrh = ((uint32_t)(cfg->bits - 5) << UART_LCRH_WLEN_SHIFT) &
         UART_LCRH_WLEN_MASK;

  if (cfg->parity == 1)
    {
      lcrh |= UART_LCRH_PEN;
    }
  else if (cfg->parity == 2)
    {
      lcrh |= UART_LCRH_PEN | UART_LCRH_EPS;
    }

  if (cfg->stop2)
    {
      lcrh |= UART_LCRH_STP2;
    }

  putreg32(lcrh, base + MSPM0_UART_LCRH_OFFSET);

  /* Interrupt on each received byte; TX interrupt when half empty */

  putreg32(UART_IFLS_RX_1 | UART_IFLS_TX_1_2, base + MSPM0_UART_IFLS_OFFSET);

  /* Enable */

  putreg32(UART_CTL0_MODE_UART | UART_CTL0_HSE_OVS16 | UART_CTL0_FEN |
           UART_CTL0_TXE | UART_CTL0_RXE | UART_CTL0_ENABLE,
           base + MSPM0_UART_CTL0_OFFSET);
}

/****************************************************************************
 * Name: mspm0_lowsetup
 ****************************************************************************/

void mspm0_lowsetup(void)
{
#if defined(HAVE_SERIAL_CONSOLE) && !defined(CONFIG_SUPPRESS_UART_CONFIG)
  const struct mspm0_uartcfg_s cfg =
  {
    .baud   = CONSOLE_BAUD,
    .bits   = CONSOLE_BITS,
    .parity = CONSOLE_PARITY,
    .stop2  = CONSOLE_2STOP,
  };

  mspm0_uart_pins(CONSOLE_TX_PINCM, CONSOLE_TX_PF,
                  CONSOLE_RX_PINCM, CONSOLE_RX_PF);
  mspm0_uart_configure(CONSOLE_BASE, CONSOLE_CLOCK, &cfg);
#endif
}
