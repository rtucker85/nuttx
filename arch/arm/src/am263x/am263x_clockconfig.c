/****************************************************************************
 * arch/arm/src/am263x/am263x_clockconfig.c
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

#include <nuttx/arch.h>

#include "arm_internal.h"
#include "hardware/am263x_rcm.h"
#include "am263x_clockconfig.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_set_clock
 *
 * Description:
 *   Program a peripheral clock: divider first, then the source, waiting for
 *   each to read back (the order used by TI's reference code), then ungate.
 *
 ****************************************************************************/

static void am263x_set_clock(uintptr_t srcsel, uintptr_t divval,
                             uintptr_t gate, uint32_t src, uint32_t div)
{
  putreg32(div, divval);
  while (getreg32(divval) != div)
    {
    }

  putreg32(src, srcsel);
  while (getreg32(srcsel) != src)
    {
    }

  putreg32(RCM_CLK_UNGATED, gate);
}

/****************************************************************************
 * Name: am263x_mcspi_clock
 ****************************************************************************/

#ifdef CONFIG_AM263X_MCSPI
static void am263x_mcspi_clock(int n)
{
  am263x_set_clock(AM263X_MSS_RCM_MCSPI_CLK_SRC_SEL(n),
                   AM263X_MSS_RCM_MCSPI_CLK_DIV_VAL(n),
                   AM263X_MSS_RCM_MCSPI_CLK_GATE(n),
                   RCM_MCSPI_CLKSRC_PER_HSDIV0_CLKOUT1, RCM_CLK_DIV(4));
}
#endif

/****************************************************************************
 * Name: am263x_mcan_clock
 *
 * Description:
 *   MCANn: DPLL_CORE_HSDIV0_CLKOUT0 (400 MHz) / 5 = 80 MHz, the usual
 *   CAN FD clock, then a module reset so it starts clean after a JTAG
 *   reload.
 *
 ****************************************************************************/

#ifdef CONFIG_AM263X_MCAN
static void am263x_mcan_clock(int n)
{
  am263x_set_clock(AM263X_MSS_RCM_MCAN_CLK_SRC_SEL(n),
                   AM263X_MSS_RCM_MCAN_CLK_DIV_VAL(n),
                   AM263X_MSS_RCM_MCAN_CLK_GATE(n),
                   RCM_MCAN_CLKSRC_CORE_HSDIV0_CLKOUT0, RCM_CLK_DIV(5));

  putreg32(RCM_RST_ASSERT, AM263X_MSS_RCM_MCAN_RST_CTRL(n));
  while (getreg32(AM263X_MSS_RCM_MCAN_RST_CTRL(n)) != RCM_RST_ASSERT)
    {
    }

  putreg32(RCM_RST_DEASSERT, AM263X_MSS_RCM_MCAN_RST_CTRL(n));
  while (getreg32(AM263X_MSS_RCM_MCAN_RST_CTRL(n)) != RCM_RST_DEASSERT)
    {
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_clockconfig
 ****************************************************************************/

void am263x_clockconfig(void)
{
  putreg32(AM263X_KICK0_UNLOCK, AM263X_MSS_RCM_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_MSS_RCM_LOCK0_KICK1);

  /* UART0: PER_HSDIV0_CLKOUT1 (192 MHz) / 4 = 48 MHz */

  am263x_set_clock(AM263X_MSS_RCM_UART0_CLK_SRC_SEL,
                   AM263X_MSS_RCM_UART0_CLK_DIV_VAL,
                   AM263X_MSS_RCM_UART0_CLK_GATE,
                   RCM_UART_CLKSRC_PER_HSDIV0_CLKOUT1, RCM_CLK_DIV(4));

  /* RTI0: the 25 MHz crystal, so the tick does not depend on the PLLs */

  am263x_set_clock(AM263X_MSS_RCM_RTI0_CLK_SRC_SEL,
                   AM263X_MSS_RCM_RTI0_CLK_DIV_VAL,
                   AM263X_MSS_RCM_RTI0_CLK_GATE,
                   RCM_RTI_CLKSRC_XTAL, RCM_CLK_DIV(1));

  /* I2C0-3 share one clock: PER_HSDIV0_CLKOUT1 (192 MHz) / 2 = 96 MHz.
   * Each module has its own gate.
   */

#ifdef CONFIG_AM263X_I2C
  am263x_set_clock(AM263X_MSS_RCM_I2C_CLK_SRC_SEL,
                   AM263X_MSS_RCM_I2C_CLK_DIV_VAL,
                   AM263X_MSS_RCM_I2C_CLK_GATE(0),
                   RCM_I2C_CLKSRC_PER_HSDIV0_CLKOUT1, RCM_CLK_DIV(2));
#endif
#ifdef CONFIG_AM263X_I2C1
  putreg32(RCM_CLK_UNGATED, AM263X_MSS_RCM_I2C_CLK_GATE(1));
#endif
#ifdef CONFIG_AM263X_I2C2
  putreg32(RCM_CLK_UNGATED, AM263X_MSS_RCM_I2C_CLK_GATE(2));
#endif
#ifdef CONFIG_AM263X_I2C3
  putreg32(RCM_CLK_UNGATED, AM263X_MSS_RCM_I2C_CLK_GATE(3));
#endif

  /* MCSPIn: PER_HSDIV0_CLKOUT1 (192 MHz) / 4 = 48 MHz */

#ifdef CONFIG_AM263X_MCSPI0
  am263x_mcspi_clock(0);
#endif
#ifdef CONFIG_AM263X_MCSPI1
  am263x_mcspi_clock(1);
#endif
#ifdef CONFIG_AM263X_MCSPI2
  am263x_mcspi_clock(2);
#endif
#ifdef CONFIG_AM263X_MCSPI3
  am263x_mcspi_clock(3);
#endif
#ifdef CONFIG_AM263X_MCSPI4
  am263x_mcspi_clock(4);
#endif

  /* MCANn: 80 MHz, see am263x_mcan_clock() */

#ifdef CONFIG_AM263X_MCAN0
  am263x_mcan_clock(0);
#endif
#ifdef CONFIG_AM263X_MCAN1
  am263x_mcan_clock(1);
#endif
#ifdef CONFIG_AM263X_MCAN2
  am263x_mcan_clock(2);
#endif
#ifdef CONFIG_AM263X_MCAN3
  am263x_mcan_clock(3);
#endif

  /* CPSW: its clock (SYS_CLK, 200 MHz) and the RGMII clocks are ungated
   * from reset.  Reset the switch so that it starts clean even after a
   * JTAG reload or an SBL that used it.  TI holds the reset for 200 ns
   * and waits another 200 ns after it.
   */

#ifdef CONFIG_AM263X_CPSW
  putreg32(RCM_CLK_UNGATED, AM263X_MSS_RCM_CPSW_CLK_GATE);
  putreg32(RCM_RST_ASSERT, AM263X_MSS_RCM_CPSW_RST_CTRL);
  while (getreg32(AM263X_MSS_RCM_CPSW_RST_CTRL) != RCM_RST_ASSERT)
    {
    }

  up_udelay(1);
  putreg32(RCM_RST_DEASSERT, AM263X_MSS_RCM_CPSW_RST_CTRL);
  while (getreg32(AM263X_MSS_RCM_CPSW_RST_CTRL) != RCM_RST_DEASSERT)
    {
    }

  up_udelay(1);
#endif
}
