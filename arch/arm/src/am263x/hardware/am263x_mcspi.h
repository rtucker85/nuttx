/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_mcspi.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MCSPI_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MCSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Instances, 4 KB apart */

#define AM263X_MCSPI_NINSTANCES     5
#define AM263X_MCSPI_BASE(n)        (AM263X_MCSPI0_BASE + ((n) << 12))

/* Interrupt numbers on R5FSS0-0 */

#define AM263X_IRQ_MCSPI(n)         (78 + (n))

/* Register offsets.  The HL block is at 0x000, the functional registers
 * from 0x100.  Each of the four channels has 0x14 bytes of CHxCONF,
 * CHxSTAT, CHxCTRL, TXx and RXx.
 */

#define AM263X_MCSPI_HL_SYSCONFIG   0x010
#define AM263X_MCSPI_REVISION       0x100
#define AM263X_MCSPI_SYSCONFIG      0x110
#define AM263X_MCSPI_SYSSTATUS      0x114
#define AM263X_MCSPI_IRQSTATUS      0x118
#define AM263X_MCSPI_IRQENABLE      0x11c
#define AM263X_MCSPI_SYST           0x124
#define AM263X_MCSPI_MODULCTRL      0x128
#define AM263X_MCSPI_CHCONF(ch)     (0x12c + (ch) * 0x14)
#define AM263X_MCSPI_CHSTAT(ch)     (0x130 + (ch) * 0x14)
#define AM263X_MCSPI_CHCTRL(ch)     (0x134 + (ch) * 0x14)
#define AM263X_MCSPI_TX(ch)         (0x138 + (ch) * 0x14)
#define AM263X_MCSPI_RX(ch)         (0x13c + (ch) * 0x14)
#define AM263X_MCSPI_XFERLEVEL      0x17c

#define AM263X_MCSPI_NCHANNELS      4

/* HL_SYSCONFIG */

#define MCSPI_HL_SYSCONFIG_IDLEMODE_NO   (1 << 2)

/* SYSCONFIG */

#define MCSPI_SYSCONFIG_SOFTRESET        (1 << 1)
#define MCSPI_SYSCONFIG_SIDLEMODE_NO     (1 << 3)
#define MCSPI_SYSCONFIG_CLKACT_BOTH      (3 << 8)

/* SYSSTATUS */

#define MCSPI_SYSSTATUS_RESETDONE        (1 << 0)

/* MODULCTRL */

#define MCSPI_MODULCTRL_SINGLE           (1 << 0)  /* One channel at a time */
#define MCSPI_MODULCTRL_PIN34            (1 << 1)  /* 3-pin: CS pin not used */
#define MCSPI_MODULCTRL_MS               (1 << 2)  /* 1 = peripheral mode */

/* CHxCONF */

#define MCSPI_CHCONF_PHA                 (1 << 0)
#define MCSPI_CHCONF_POL                 (1 << 1)
#define MCSPI_CHCONF_CLKD_SHIFT          2         /* Divider, low 4 bits */
#define MCSPI_CHCONF_CLKD_MASK           (15 << MCSPI_CHCONF_CLKD_SHIFT)
#define MCSPI_CHCONF_EPOL                (1 << 6)  /* CS active low */
#define MCSPI_CHCONF_WL_SHIFT            7         /* Word length - 1 */
#define MCSPI_CHCONF_WL_MASK             (31 << MCSPI_CHCONF_WL_SHIFT)
#define MCSPI_CHCONF_DPE0                (1 << 16) /* No transmit on D0 */
#define MCSPI_CHCONF_DPE1                (1 << 17) /* No transmit on D1 */
#define MCSPI_CHCONF_IS                  (1 << 18) /* Receive on D1, not D0 */
#define MCSPI_CHCONF_FORCE               (1 << 20) /* Assert CSn */
#define MCSPI_CHCONF_CLKG                (1 << 29) /* Divide by 1..4096 */

/* CHxSTAT */

#define MCSPI_CHSTAT_RXS                 (1 << 0)  /* RX register full */
#define MCSPI_CHSTAT_TXS                 (1 << 1)  /* TX register empty */
#define MCSPI_CHSTAT_EOT                 (1 << 2)  /* End of transfer */

/* CHxCTRL */

#define MCSPI_CHCTRL_EN                  (1 << 0)
#define MCSPI_CHCTRL_EXTCLK_SHIFT        8         /* Divider, high 8 bits */
#define MCSPI_CHCTRL_EXTCLK_MASK         (255 << MCSPI_CHCTRL_EXTCLK_SHIFT)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MCSPI_H */
