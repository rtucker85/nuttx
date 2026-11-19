/****************************************************************************
 * arch/arm/src/mspm0/hardware/mspm0_uart.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_UART_H
#define __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/mspm0_memorymap.h"
#include "hardware/mspm0_gprcm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

/* Power/reset control, CLKDIV, CLKSEL, PDBGCTL and the CPU interrupt
 * registers are at the common offsets in mspm0_gprcm.h.
 */

#define MSPM0_UART_INTCTL_OFFSET  0x10e4 /* Interrupt control */
#define MSPM0_UART_CTL0_OFFSET    0x1100 /* Control 0 */
#define MSPM0_UART_LCRH_OFFSET    0x1104 /* Line control */
#define MSPM0_UART_STAT_OFFSET    0x1108 /* Status */
#define MSPM0_UART_IFLS_OFFSET    0x110c /* Interrupt FIFO level select */
#define MSPM0_UART_IBRD_OFFSET    0x1110 /* Integer baud-rate divisor */
#define MSPM0_UART_FBRD_OFFSET    0x1114 /* Fractional baud-rate divisor */
#define MSPM0_UART_GFCTL_OFFSET   0x1118 /* Glitch filter control */
#define MSPM0_UART_TXDATA_OFFSET  0x1120 /* Transmit data */
#define MSPM0_UART_RXDATA_OFFSET  0x1124 /* Receive data */
#define MSPM0_UART_CLKDIV2_OFFSET 0x1160 /* Second clock divider */

/* Register bit definitions *************************************************/

/* CLKSEL: exactly one source */

#define UART_CLKSEL_LFCLK         (1 << 1)
#define UART_CLKSEL_MFCLK         (1 << 2)
#define UART_CLKSEL_BUSCLK        (1 << 3)

/* CLKDIV and CLKDIV2: divide by RATIO + 1 */

#define UART_CLKDIV_RATIO_MASK    (7 << 0)
#  define UART_CLKDIV_RATIO(n)    ((uint32_t)((n) - 1))  /* n = 1..8 */

/* CPU_INT IMASK, RIS, MIS, ISET and ICLR */

#define UART_INT_RTOUT            (1 << 0)  /* Receive timeout */
#define UART_INT_FRMERR           (1 << 1)  /* Framing error */
#define UART_INT_PARERR           (1 << 2)  /* Parity error */
#define UART_INT_BRKERR           (1 << 3)  /* Break error */
#define UART_INT_OVRERR           (1 << 4)  /* Receive overrun */
#define UART_INT_RXNE             (1 << 5)  /* Negative edge on RX */
#define UART_INT_RXPE             (1 << 6)  /* Positive edge on RX */
#define UART_INT_RXINT            (1 << 10) /* RX FIFO at IFLS level */
#define UART_INT_TXINT            (1 << 11) /* TX FIFO at IFLS level */
#define UART_INT_EOT              (1 << 12) /* End of transmission */
#define UART_INT_CTS              (1 << 14) /* CTS changed */
#define UART_INT_NERR             (1 << 17) /* Noise error */

/* CTL0 */

#define UART_CTL0_ENABLE          (1 << 0)  /* Module enable */
#define UART_CTL0_LBE             (1 << 2)  /* Loopback */
#define UART_CTL0_RXE             (1 << 3)  /* Receive enable */
#define UART_CTL0_TXE             (1 << 4)  /* Transmit enable */
#define UART_CTL0_TXD_OUT_EN      (1 << 5)  /* Drive TXD from TXD_OUT */
#define UART_CTL0_TXD_OUT         (1 << 6)  /* TXD level when TXD_OUT_EN */
#define UART_CTL0_MENC            (1 << 7)  /* Manchester encoding */
#define UART_CTL0_MODE_SHIFT      (8)       /* Bits 8-10: Mode */
#define UART_CTL0_MODE_MASK       (7 << UART_CTL0_MODE_SHIFT)
#  define UART_CTL0_MODE_UART     (0 << UART_CTL0_MODE_SHIFT)
#  define UART_CTL0_MODE_RS485    (1 << UART_CTL0_MODE_SHIFT)
#  define UART_CTL0_MODE_IDLELINE (2 << UART_CTL0_MODE_SHIFT)
#  define UART_CTL0_MODE_ADDR9BIT (3 << UART_CTL0_MODE_SHIFT)
#  define UART_CTL0_MODE_SMART    (4 << UART_CTL0_MODE_SHIFT)
#  define UART_CTL0_MODE_DALI     (5 << UART_CTL0_MODE_SHIFT)
#define UART_CTL0_RTS             (1 << 12) /* Assert RTS (software) */
#define UART_CTL0_RTSEN           (1 << 13) /* Hardware RTS */
#define UART_CTL0_CTSEN           (1 << 14) /* Hardware CTS */
#define UART_CTL0_HSE_SHIFT       (15)      /* Bits 15-16: Oversampling */
#define UART_CTL0_HSE_MASK        (3 << UART_CTL0_HSE_SHIFT)
#  define UART_CTL0_HSE_OVS16     (0 << UART_CTL0_HSE_SHIFT)
#  define UART_CTL0_HSE_OVS8      (1 << UART_CTL0_HSE_SHIFT)
#  define UART_CTL0_HSE_OVS3      (2 << UART_CTL0_HSE_SHIFT)
#define UART_CTL0_FEN             (1 << 17) /* FIFOs enable */
#define UART_CTL0_MAJVOTE         (1 << 18) /* Majority vote sampling */
#define UART_CTL0_MSBFIRST        (1 << 19) /* MSB first */

/* LCRH */

#define UART_LCRH_BRK             (1 << 0)  /* Send break */
#define UART_LCRH_PEN             (1 << 1)  /* Parity enable */
#define UART_LCRH_EPS             (1 << 2)  /* Even parity */
#define UART_LCRH_STP2            (1 << 3)  /* Two stop bits */
#define UART_LCRH_WLEN_SHIFT      (4)       /* Bits 4-5: Word length */
#define UART_LCRH_WLEN_MASK       (3 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_5BIT     (0 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_6BIT     (1 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_7BIT     (2 << UART_LCRH_WLEN_SHIFT)
#  define UART_LCRH_WLEN_8BIT     (3 << UART_LCRH_WLEN_SHIFT)
#define UART_LCRH_SPS             (1 << 6)  /* Stick parity */
#define UART_LCRH_SENDIDLE        (1 << 7)  /* Send idle pattern */

/* STAT */

#define UART_STAT_BUSY            (1 << 0)  /* Transmitting */
#define UART_STAT_RXFE            (1 << 2)  /* RX FIFO empty */
#define UART_STAT_RXFF            (1 << 3)  /* RX FIFO full */
#define UART_STAT_TXFE            (1 << 6)  /* TX FIFO empty */
#define UART_STAT_TXFF            (1 << 7)  /* TX FIFO full */
#define UART_STAT_CTS             (1 << 8)  /* CTS level */
#define UART_STAT_IDLE            (1 << 9)  /* Idle line detected */

/* IFLS.  TX levels are how empty the TX FIFO is (TX_1: at least one entry
 * free); RX levels are how full the RX FIFO is (RX_1: at least one entry).
 */

#define UART_IFLS_TXIFLSEL_SHIFT  (0)       /* Bits 0-2: TX interrupt level */
#define UART_IFLS_TXIFLSEL_MASK   (7 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TX_3_4        (1 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TX_1_2        (2 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TX_1_4        (3 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TX_EMPTY      (5 << UART_IFLS_TXIFLSEL_SHIFT)
#  define UART_IFLS_TX_1          (7 << UART_IFLS_TXIFLSEL_SHIFT)
#define UART_IFLS_RXIFLSEL_SHIFT  (4)       /* Bits 4-6: RX interrupt level */
#define UART_IFLS_RXIFLSEL_MASK   (7 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RX_1_4        (1 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RX_1_2        (2 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RX_3_4        (3 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RX_FULL       (5 << UART_IFLS_RXIFLSEL_SHIFT)
#  define UART_IFLS_RX_1          (7 << UART_IFLS_RXIFLSEL_SHIFT)
#define UART_IFLS_RXTOSEL_SHIFT   (8)       /* Bits 8-11: RX timeout, bit times */
#define UART_IFLS_RXTOSEL_MASK    (15 << UART_IFLS_RXTOSEL_SHIFT)
#  define UART_IFLS_RXTOSEL(n)    ((uint32_t)(n) << UART_IFLS_RXTOSEL_SHIFT)

/* IBRD and FBRD.  The divisor is UARTclk / (oversampling * baud): IBRD
 * holds the integer part and FBRD the fraction in 64ths.
 */

#define UART_IBRD_MASK            (0xffff)
#define UART_FBRD_MASK            (0x3f)

/* RXDATA */

#define UART_RXDATA_DATA_MASK     (0xff)
#define UART_RXDATA_FRMERR        (1 << 8)  /* Framing error */
#define UART_RXDATA_PARERR        (1 << 9)  /* Parity error */
#define UART_RXDATA_BRKERR        (1 << 10) /* Break error */
#define UART_RXDATA_OVRERR        (1 << 11) /* Overrun error */
#define UART_RXDATA_ERRORS        (UART_RXDATA_FRMERR | UART_RXDATA_PARERR | \
                                   UART_RXDATA_BRKERR | UART_RXDATA_OVRERR)

#endif /* __ARCH_ARM_SRC_MSPM0_HARDWARE_MSPM0_UART_H */
