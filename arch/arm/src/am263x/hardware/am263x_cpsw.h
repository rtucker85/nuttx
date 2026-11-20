/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_cpsw.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CPSW_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CPSW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPSW3G: a three-port switch (host port 0, MAC ports 1 and 2).  Its
 * functional clock (CPPI_ICLK) is SYS_CLK, 200 MHz.
 */

#define AM263X_CPSW_CLK_HZ            200000000

/* Sub-module offsets from AM263X_CPSW_BASE */

#define AM263X_CPSW_MDIO_OFFSET       0x00000f00

#define AM263X_MDIO_BASE \
  (AM263X_CPSW_BASE + AM263X_CPSW_MDIO_OFFSET)

/* MDIO register offsets */

#define AM263X_MDIO_VERSION_OFFSET    0x0000
#define AM263X_MDIO_CONTROL_OFFSET    0x0004
#define AM263X_MDIO_ALIVE_OFFSET      0x0008
#define AM263X_MDIO_LINK_OFFSET       0x000c
#define AM263X_MDIO_POLL_OFFSET       0x0034
#define AM263X_MDIO_USERACCESS_OFFSET(n) (0x0080 + 8 * (n))
#define AM263X_MDIO_USERPHYSEL_OFFSET(n) (0x0084 + 8 * (n))

#define AM263X_MDIO_VERSION \
  (AM263X_MDIO_BASE + AM263X_MDIO_VERSION_OFFSET)
#define AM263X_MDIO_CONTROL \
  (AM263X_MDIO_BASE + AM263X_MDIO_CONTROL_OFFSET)
#define AM263X_MDIO_ALIVE \
  (AM263X_MDIO_BASE + AM263X_MDIO_ALIVE_OFFSET)
#define AM263X_MDIO_LINK \
  (AM263X_MDIO_BASE + AM263X_MDIO_LINK_OFFSET)
#define AM263X_MDIO_POLL \
  (AM263X_MDIO_BASE + AM263X_MDIO_POLL_OFFSET)
#define AM263X_MDIO_USERACCESS(n) \
  (AM263X_MDIO_BASE + AM263X_MDIO_USERACCESS_OFFSET(n))
#define AM263X_MDIO_USERPHYSEL(n) \
  (AM263X_MDIO_BASE + AM263X_MDIO_USERPHYSEL_OFFSET(n))

/* MDIO CONTROL */

#define MDIO_CONTROL_CLKDIV_SHIFT     0
#define MDIO_CONTROL_CLKDIV_MASK      (0xffff << MDIO_CONTROL_CLKDIV_SHIFT)
#define MDIO_CONTROL_CLKDIV(n)        ((uint32_t)(n) << MDIO_CONTROL_CLKDIV_SHIFT)
#define MDIO_CONTROL_FAULTENB         (1 << 18)
#define MDIO_CONTROL_FAULT            (1 << 19)
#define MDIO_CONTROL_PREAMBLE         (1 << 20) /* 1: no preamble */
#define MDIO_CONTROL_ENABLE           (1 << 30)
#define MDIO_CONTROL_IDLE             (1 << 31)

/* MDIO POLL */

#define MDIO_POLL_STATECHANGEMODE     (1 << 30)
#define MDIO_POLL_MANUALMODE          (1 << 31)

/* MDIO USERACCESSn */

#define MDIO_USERACCESS_DATA_SHIFT    0
#define MDIO_USERACCESS_DATA_MASK     (0xffff << MDIO_USERACCESS_DATA_SHIFT)
#define MDIO_USERACCESS_PHYADR_SHIFT  16
#define MDIO_USERACCESS_PHYADR(n) \
  (((uint32_t)(n) & 0x1f) << MDIO_USERACCESS_PHYADR_SHIFT)
#define MDIO_USERACCESS_REGADR_SHIFT  21
#define MDIO_USERACCESS_REGADR(n) \
  (((uint32_t)(n) & 0x1f) << MDIO_USERACCESS_REGADR_SHIFT)
#define MDIO_USERACCESS_ACK           (1 << 29)
#define MDIO_USERACCESS_WRITE         (1 << 30)
#define MDIO_USERACCESS_GO            (1 << 31)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CPSW_H */
