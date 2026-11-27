/****************************************************************************
 * arch/arm/src/am263x/am263x_wdt.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_WDT_H
#define __ARCH_ARM_SRC_AM263X_AM263X_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_AM263X_WDT

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_wdt_initialize
 *
 * Description:
 *   Register MSS watchdog WDTn (0-3) as a watchdog timer device at devpath
 *   (e.g. "/dev/watchdog0").  Its expiry is a SoC warm reset, it stops
 *   while the CPU is halted in the debugger, and the window is open 100 %
 *   of the period.  The initial timeout is CONFIG_AM263X_WDT_TIMEOUT.  A
 *   watchdog found already running (started by a boot loader) is kept
 *   running, with its timeout, and reported as active.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int am263x_wdt_initialize(FAR const char *devpath, int wdt);

#endif /* CONFIG_AM263X_WDT */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_WDT_H */
