/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_rcm.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RCM_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RCM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MSS_RCM register offsets (only the ones the port uses so far) */

#define AM263X_MSS_RCM_RTI0_CLK_SRC_SEL_OFFSET   0x0114
#define AM263X_MSS_RCM_UART0_CLK_SRC_SEL_OFFSET  0x0174
#define AM263X_MSS_RCM_RTI0_CLK_DIV_VAL_OFFSET   0x0214
#define AM263X_MSS_RCM_UART0_CLK_DIV_VAL_OFFSET  0x0274
#define AM263X_MSS_RCM_RTI0_CLK_GATE_OFFSET      0x0314
#define AM263X_MSS_RCM_UART0_CLK_GATE_OFFSET     0x038c
#define AM263X_MSS_RCM_MCSPI_CLK_SRC_SEL_OFFSET(n) (0x013c + 4 * (n))
#define AM263X_MSS_RCM_MCSPI_CLK_DIV_VAL_OFFSET(n) (0x023c + 4 * (n))
#define AM263X_MSS_RCM_MCSPI_CLK_GATE_OFFSET(n)    (0x033c + 4 * (n))
#define AM263X_MSS_RCM_I2C_CLK_SRC_SEL_OFFSET    0x0164 /* all four */
#define AM263X_MSS_RCM_I2C_CLK_DIV_VAL_OFFSET    0x0264
#define AM263X_MSS_RCM_I2C_CLK_GATE_OFFSET(n)    (0x0364 + 4 * (n))
#define AM263X_MSS_RCM_MCAN_CLK_SRC_SEL_OFFSET(n)  (0x0100 + 4 * (n))
#define AM263X_MSS_RCM_MCAN_CLK_DIV_VAL_OFFSET(n)  (0x0200 + 4 * (n))
#define AM263X_MSS_RCM_MCAN_CLK_GATE_OFFSET(n)     (0x0300 + 4 * (n))
#define AM263X_MSS_RCM_MCAN_RST_CTRL_OFFSET(n)     (0x0588 + 4 * (n))
#define AM263X_MSS_RCM_LIN_CLK_SRC_SEL_OFFSET(n)  (0x0174 + 4 * (n)) /* = UARTn's */
#define AM263X_MSS_RCM_LIN_CLK_DIV_VAL_OFFSET(n)  (0x0274 + 4 * (n))
#define AM263X_MSS_RCM_LIN_CLK_GATE_OFFSET(n)     (0x0374 + 4 * (n))
#define AM263X_MSS_RCM_LIN_RST_CTRL_OFFSET(n)     (0x05c0 + 4 * (n))
#define AM263X_MSS_RCM_CPSW_CLK_GATE_OFFSET      0x03b8
#define AM263X_MSS_RCM_QSPI0_CLK_SRC_SEL_OFFSET  0x0110
#define AM263X_MSS_RCM_QSPI0_CLK_DIV_VAL_OFFSET  0x0210
#define AM263X_MSS_RCM_QSPI0_CLK_GATE_OFFSET     0x0310
#define AM263X_MSS_RCM_QSPI0_RST_CTRL_OFFSET     0x0584
#define AM263X_MSS_RCM_CPSW_RST_CTRL_OFFSET      0x05e0
#define AM263X_MSS_RCM_WDT_CLK_SRC_SEL_OFFSET(n) (0x0128 + 4 * (n))
#define AM263X_MSS_RCM_WDT_CLK_DIV_VAL_OFFSET(n) (0x0228 + 4 * (n))
#define AM263X_MSS_RCM_WDT_CLK_GATE_OFFSET(n)    (0x0328 + 4 * (n))
#define AM263X_MSS_RCM_WDT_CLK_STATUS_OFFSET(n)  (0x0428 + 4 * (n))
#define AM263X_MSS_RCM_WDT_RST_CTRL_OFFSET(n)    (0x054c + 4 * (n))
#define AM263X_MSS_RCM_LOCK0_KICK0_OFFSET        0x1008
#define AM263X_MSS_RCM_LOCK0_KICK1_OFFSET        0x100c

#define AM263X_MSS_RCM_RTI0_CLK_SRC_SEL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_RTI0_CLK_SRC_SEL_OFFSET)
#define AM263X_MSS_RCM_UART0_CLK_SRC_SEL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_UART0_CLK_SRC_SEL_OFFSET)
#define AM263X_MSS_RCM_RTI0_CLK_DIV_VAL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_RTI0_CLK_DIV_VAL_OFFSET)
#define AM263X_MSS_RCM_UART0_CLK_DIV_VAL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_UART0_CLK_DIV_VAL_OFFSET)
#define AM263X_MSS_RCM_RTI0_CLK_GATE \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_RTI0_CLK_GATE_OFFSET)
#define AM263X_MSS_RCM_UART0_CLK_GATE \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_UART0_CLK_GATE_OFFSET)
#define AM263X_MSS_RCM_MCSPI_CLK_SRC_SEL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCSPI_CLK_SRC_SEL_OFFSET(n))
#define AM263X_MSS_RCM_MCSPI_CLK_DIV_VAL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCSPI_CLK_DIV_VAL_OFFSET(n))
#define AM263X_MSS_RCM_MCSPI_CLK_GATE(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCSPI_CLK_GATE_OFFSET(n))
#define AM263X_MSS_RCM_I2C_CLK_SRC_SEL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_I2C_CLK_SRC_SEL_OFFSET)
#define AM263X_MSS_RCM_I2C_CLK_DIV_VAL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_I2C_CLK_DIV_VAL_OFFSET)
#define AM263X_MSS_RCM_I2C_CLK_GATE(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_I2C_CLK_GATE_OFFSET(n))
#define AM263X_MSS_RCM_CPSW_CLK_GATE \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_CPSW_CLK_GATE_OFFSET)
#define AM263X_MSS_RCM_CPSW_RST_CTRL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_CPSW_RST_CTRL_OFFSET)
#define AM263X_MSS_RCM_MCAN_CLK_SRC_SEL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCAN_CLK_SRC_SEL_OFFSET(n))
#define AM263X_MSS_RCM_MCAN_CLK_DIV_VAL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCAN_CLK_DIV_VAL_OFFSET(n))
#define AM263X_MSS_RCM_MCAN_CLK_GATE(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCAN_CLK_GATE_OFFSET(n))
#define AM263X_MSS_RCM_MCAN_RST_CTRL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_MCAN_RST_CTRL_OFFSET(n))
#define AM263X_MSS_RCM_LIN_CLK_SRC_SEL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LIN_CLK_SRC_SEL_OFFSET(n))
#define AM263X_MSS_RCM_LIN_CLK_DIV_VAL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LIN_CLK_DIV_VAL_OFFSET(n))
#define AM263X_MSS_RCM_LIN_CLK_GATE(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LIN_CLK_GATE_OFFSET(n))
#define AM263X_MSS_RCM_LIN_RST_CTRL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LIN_RST_CTRL_OFFSET(n))
#define AM263X_MSS_RCM_QSPI0_CLK_SRC_SEL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_QSPI0_CLK_SRC_SEL_OFFSET)
#define AM263X_MSS_RCM_QSPI0_CLK_DIV_VAL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_QSPI0_CLK_DIV_VAL_OFFSET)
#define AM263X_MSS_RCM_QSPI0_CLK_GATE \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_QSPI0_CLK_GATE_OFFSET)
#define AM263X_MSS_RCM_QSPI0_RST_CTRL \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_QSPI0_RST_CTRL_OFFSET)
#define AM263X_MSS_RCM_WDT_CLK_SRC_SEL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_WDT_CLK_SRC_SEL_OFFSET(n))
#define AM263X_MSS_RCM_WDT_CLK_DIV_VAL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_WDT_CLK_DIV_VAL_OFFSET(n))
#define AM263X_MSS_RCM_WDT_CLK_GATE(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_WDT_CLK_GATE_OFFSET(n))
#define AM263X_MSS_RCM_WDT_CLK_STATUS(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_WDT_CLK_STATUS_OFFSET(n))
#define AM263X_MSS_RCM_WDT_RST_CTRL(n) \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_WDT_RST_CTRL_OFFSET(n))
#define AM263X_MSS_RCM_LOCK0_KICK0 \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LOCK0_KICK0_OFFSET)
#define AM263X_MSS_RCM_LOCK0_KICK1 \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LOCK0_KICK1_OFFSET)

/* TOP_RCM: SoC warm reset.  Writing 0 to WARM_RESET_REQ.SW_RST (bits 2:0)
 * resets the SoC, which then boots again in the latched boot mode.
 * WARM_RESET_CONFIG has a 3-bit enable per warm-reset source; 7 (the reset
 * value) lets the source reset the SoC.
 */

#define AM263X_TOP_RCM_WARM_RESET_CONFIG_OFFSET 0x0004
#define AM263X_TOP_RCM_WARM_RESET_REQ_OFFSET 0x0008
#define AM263X_TOP_RCM_WARM_RST_CAUSE_OFFSET 0x000c
#define AM263X_TOP_RCM_WARM_RST_CAUSE_CLR_OFFSET 0x0010
#define AM263X_TOP_RCM_LOCK0_KICK0_OFFSET    0x1008
#define AM263X_TOP_RCM_LOCK0_KICK1_OFFSET    0x100c

#define AM263X_TOP_RCM_WARM_RESET_CONFIG \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_WARM_RESET_CONFIG_OFFSET)
#define AM263X_TOP_RCM_WARM_RESET_REQ \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_WARM_RESET_REQ_OFFSET)
#define AM263X_TOP_RCM_WARM_RST_CAUSE \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_WARM_RST_CAUSE_OFFSET)
#define AM263X_TOP_RCM_WARM_RST_CAUSE_CLR \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_WARM_RST_CAUSE_CLR_OFFSET)
#define AM263X_TOP_RCM_LOCK0_KICK0 \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_LOCK0_KICK0_OFFSET)
#define AM263X_TOP_RCM_LOCK0_KICK1 \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_LOCK0_KICK1_OFFSET)

/* TOP_RCM.WARM_RST_CAUSE: one bit per source, kept until cleared (write
 * 0x7 to WARM_RST_CAUSE_CLR) or a power-on reset.  WDT(n) is MSS WDTn,
 * SW is WARM_RESET_REQ.  Bit 6 is set with every cause; alone it is the
 * WARMRSTn pin.
 */

#define TOP_RCM_WARM_RESET_CONFIG_WDOG_SHIFT(n) (16 + 4 * (n))
#define TOP_RCM_WARM_RESET_CONFIG_WDOG_MASK(n) \
  (7 << TOP_RCM_WARM_RESET_CONFIG_WDOG_SHIFT(n))
#define TOP_RCM_WARM_RESET_CONFIG_WDOG_EN(n) \
  (7 << TOP_RCM_WARM_RESET_CONFIG_WDOG_SHIFT(n))

#define TOP_RCM_RST_CAUSE_POR              (1 << 0)
#define TOP_RCM_RST_CAUSE_WDT(n)           (1 << (1 + (n)))
#define TOP_RCM_RST_CAUSE_SW               (1 << 5)
#define TOP_RCM_RST_CAUSE_WARM             (1 << 6)
#define TOP_RCM_RST_CAUSE_HSM_WDT          (1 << 7)
#define TOP_RCM_RST_CAUSE_DEBUG            (1 << 8)
#define TOP_RCM_RST_CAUSE_TSENSE0          (1 << 9)
#define TOP_RCM_RST_CAUSE_TSENSE1          (1 << 10)
#define TOP_RCM_RST_CAUSE_MASK             0xfff
#define TOP_RCM_RST_CAUSE_CLEAR            0x7

/* MSS_RCM: the gate of the thermal manager's 32 kHz clock */

#define AM263X_MSS_RCM_TEMPSENSE_32K_CLK_GATE (AM263X_MSS_RCM_BASE + 0x03b4)

/* Peripheral clock source selectors.  The 3-bit selector is repeated in
 * three nibbles; the value differs per peripheral group.
 */

#define RCM_UART_CLKSRC_XTAL               0x666 /* 25 MHz crystal */
#define RCM_UART_CLKSRC_SYS_CLK            0x222
#define RCM_UART_CLKSRC_PER_HSDIV0_CLKOUT1 0x333 /* 192 MHz */

#define RCM_MCSPI_CLKSRC_XTAL               0x666 /* 25 MHz crystal */
#define RCM_MCSPI_CLKSRC_SYS_CLK            0x222
#define RCM_MCSPI_CLKSRC_PER_HSDIV0_CLKOUT1 0x333 /* 192 MHz */

#define RCM_I2C_CLKSRC_PER_HSDIV0_CLKOUT1   0x333 /* 192 MHz */

#define RCM_MCAN_CLKSRC_CORE_HSDIV0_CLKOUT0 0x444 /* 400 MHz */

#define RCM_LIN_CLKSRC_PER_HSDIV0_CLKOUT1   0x333 /* 192 MHz */

#define RCM_QSPI_CLKSRC_CORE_HSDIV0_CLKOUT0 0x444 /* 400 MHz */

#define RCM_RTI_CLKSRC_XTAL                0x666 /* 25 MHz crystal */
#define RCM_RTI_CLKSRC_SYS_CLK             0x222
#define RCM_RTI_CLKSRC_PER_HSDIV0_CLKOUT1  0x333

#define RCM_WDT_CLKSRC_WUCPUCLK            0x000 /* reset value */
#define RCM_WDT_CLKSRC_RCCLK10M            0x111 /* 10 MHz RC oscillator */
#define RCM_WDT_CLKSRC_SYS_CLK             0x222 /* 200 MHz */
#define RCM_WDT_CLKSRC_PER_HSDIV0_CLKOUT1  0x333 /* 192 MHz */
#define RCM_WDT_CLKSRC_CORE_HSDIV0_CLKOUT1 0x444
#define RCM_WDT_CLKSRC_RCCLK10M_ALT        0x555
#define RCM_WDT_CLKSRC_XTAL                0x666 /* 25 MHz crystal */
#define RCM_WDT_CLKSRC_RCCLK32K            0x777 /* 32 kHz RC oscillator */

/* Clock dividers hold (divide-by - 1), repeated in three nibbles */

#define RCM_CLK_DIV(n)                     ((((n) - 1) & 0xf) * 0x111)

/* Clock status (WDTn_CLK_STATUS, ...): one CLKINUSE bit per source in
 * bits 7:0, the divider in use (divide-by - 1) in bits 15:8
 */

#define RCM_CLK_STATUS(src, div)           ((1 << ((src) & 7)) | \
                                            (((div) - 1) << 8))

/* Clock gates: writing 0 ungates the clock */

#define RCM_CLK_UNGATED                    0
#define RCM_CLK_GATED                      7

/* Module resets (CPSW_RST_CTRL, MCANn_RST_CTRL, LINn_RST_CTRL,
 * QSPI0_RST_CTRL, WDTn_RST_CTRL): 7 holds the module in reset
 */

#define RCM_RST_ASSERT                     7
#define RCM_RST_DEASSERT                   0

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RCM_H */
