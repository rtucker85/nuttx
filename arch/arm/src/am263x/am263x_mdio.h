/****************************************************************************
 * arch/arm/src/am263x/am263x_mdio.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_MDIO_H
#define __ARCH_ARM_SRC_AM263X_AM263X_MDIO_H

/* The CPSW's MDIO controller as an MDIO bus lower half.  Clause 22 reads
 * and writes through USERACCESS0, polled.  The controller's own PHY
 * polling (link monitoring and its interrupts) is not used.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/net/mdio.h>

#ifdef CONFIG_AM263X_MDIO

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mdio_initialize
 *
 * Description:
 *   Enable the MDIO controller and register it with the MDIO upper half.
 *   The board must have muxed the MDIO and MDC pads.  Later calls return
 *   the same bus.
 *
 * Returned Value:
 *   The MDIO bus, or NULL on failure.
 *
 ****************************************************************************/

FAR struct mdio_bus_s *am263x_mdio_initialize(void);

#endif /* CONFIG_AM263X_MDIO */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_MDIO_H */
