/****************************************************************************
 * arch/arm/src/am263x/am263x_mcan.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_MCAN_H
#define __ARCH_ARM_SRC_AM263X_AM263X_MCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_AM263X_MCAN

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mcan_initialize
 *
 * Description:
 *   Register MCAN<port> as the SocketCAN interface can<port>.  The board
 *   sets up the pins (and the transceiver) first; the module's clock and
 *   reset are handled in am263x_clockconfig().  The controller itself is
 *   configured when the interface is brought up.
 *
 * Input Parameters:
 *   port - 0-3, enabled with CONFIG_AM263X_MCAN<port>
 *
 * Returned Value:
 *   OK, or a negated errno value.
 *
 ****************************************************************************/

int am263x_mcan_initialize(int port);

#endif /* CONFIG_AM263X_MCAN */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_MCAN_H */
