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
#define AM263X_MSS_RCM_CPSW_CLK_GATE_OFFSET      0x03b8
#define AM263X_MSS_RCM_CPSW_RST_CTRL_OFFSET      0x05e0
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
#define AM263X_MSS_RCM_LOCK0_KICK0 \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LOCK0_KICK0_OFFSET)
#define AM263X_MSS_RCM_LOCK0_KICK1 \
  (AM263X_MSS_RCM_BASE + AM263X_MSS_RCM_LOCK0_KICK1_OFFSET)

/* TOP_RCM: SoC warm reset.  Writing 0 to WARM_RESET_REQ.SW_RST (bits 2:0)
 * resets the SoC, which then boots again in the latched boot mode.
 */

#define AM263X_TOP_RCM_WARM_RESET_REQ_OFFSET 0x0008
#define AM263X_TOP_RCM_LOCK0_KICK0_OFFSET    0x1008
#define AM263X_TOP_RCM_LOCK0_KICK1_OFFSET    0x100c

#define AM263X_TOP_RCM_WARM_RESET_REQ \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_WARM_RESET_REQ_OFFSET)
#define AM263X_TOP_RCM_LOCK0_KICK0 \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_LOCK0_KICK0_OFFSET)
#define AM263X_TOP_RCM_LOCK0_KICK1 \
  (AM263X_TOP_RCM_BASE + AM263X_TOP_RCM_LOCK0_KICK1_OFFSET)

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

#define RCM_RTI_CLKSRC_XTAL                0x666 /* 25 MHz crystal */
#define RCM_RTI_CLKSRC_SYS_CLK             0x222
#define RCM_RTI_CLKSRC_PER_HSDIV0_CLKOUT1  0x333

/* Clock dividers hold (divide-by - 1), repeated in three nibbles */

#define RCM_CLK_DIV(n)                     ((((n) - 1) & 0xf) * 0x111)

/* Clock gates: writing 0 ungates the clock */

#define RCM_CLK_UNGATED                    0
#define RCM_CLK_GATED                      7

/* Module resets (CPSW_RST_CTRL): 7 holds the module in reset */

#define RCM_RST_ASSERT                     7
#define RCM_RST_DEASSERT                   0

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RCM_H */
