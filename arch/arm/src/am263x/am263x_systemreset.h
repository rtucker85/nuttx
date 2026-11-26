/****************************************************************************
 * arch/arm/src/am263x/am263x_systemreset.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_SYSTEMRESET_H
#define __ARCH_ARM_SRC_AM263X_AM263X_SYSTEMRESET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/am263x_rcm.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_reset_cause
 *
 * Description:
 *   Read TOP_RCM.WARM_RST_CAUSE: the TOP_RCM_RST_CAUSE_* bits of the
 *   resets since it was last cleared or the last power-on.
 *
 * Input Parameters:
 *   clear - Clear the register after reading it, so that the next boot
 *           sees only the next reset
 *
 ****************************************************************************/

uint32_t am263x_reset_cause(bool clear);

/****************************************************************************
 * Name: am263x_reset_report
 *
 * Description:
 *   Log the reset cause (a TSENSE thermal shutdown as a warning, so it is
 *   not taken for a crash), then clear it.  Call once, early in bringup.
 *
 * Returned Value:
 *   The cause, as am263x_reset_cause() returns it.
 *
 ****************************************************************************/

uint32_t am263x_reset_report(void);

#endif /* __ARCH_ARM_SRC_AM263X_AM263X_SYSTEMRESET_H */
