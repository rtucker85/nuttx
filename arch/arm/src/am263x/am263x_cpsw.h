/****************************************************************************
 * arch/arm/src/am263x/am263x_cpsw.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_CPSW_H
#define __ARCH_ARM_SRC_AM263X_AM263X_CPSW_H

/* Ethernet through the CPSW3G, one MAC port only.  The switch runs with
 * the ALE in bypass: everything a MAC port receives goes to the host, and
 * the host's frames are directed to the one port.  The other MAC port is
 * disabled.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_AM263X_CPSW_ETH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The CPDMA descriptors live in one MPU region of their own, uncached.
 * It must be a power of two and aligned to its size.
 */

#define AM263X_CPSW_DESC_REGION_SIZE  4096

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t g_am263x_cpsw_desc[AM263X_CPSW_DESC_REGION_SIZE];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_cpsw_macaddr
 *
 * Description:
 *   Board hook: the MAC address of the Ethernet port.  The default (weak)
 *   one returns a fixed locally administered address.
 *
 * Input Parameters:
 *   mac - Where to put the six bytes
 *
 * Returned Value:
 *   OK, or a negated errno to use the default.
 *
 ****************************************************************************/

int am263x_cpsw_macaddr(uint8_t mac[6]);

#endif /* CONFIG_AM263X_CPSW_ETH */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_CPSW_H */
