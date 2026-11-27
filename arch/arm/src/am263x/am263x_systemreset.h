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

/****************************************************************************
 * Name: am263x_reset_cause_boot
 *
 * Description:
 *   The reset cause of this boot: what am263x_reset_report() read before
 *   it cleared the register, or the register itself if it has not run.
 *   Zero after a debugger load (no reset since the last boot cleared it).
 *
 ****************************************************************************/

uint32_t am263x_reset_cause_boot(void);

/****************************************************************************
 * Name: am263x_reset_cause_get
 *
 * Description:
 *   The reset cause of this boot for BOARDIOC_RESET_CAUSE, for the boards'
 *   board_reset_cause():
 *
 *     MSS WDTn            BOARDIOC_RESETCAUSE_CORE_MWDT, flag n (0-3)
 *     HSM WDT             BOARDIOC_RESETCAUSE_CORE_MWDT, flag 4
 *     TSENSE0/1 shutdown  BOARDIOC_RESETCAUSE_CORE_SOFT,
 *                         flag BOARDIOC_SOFTRESETCAUSE_THERMAL
 *     WARM_RESET_REQ      BOARDIOC_RESETCAUSE_CORE_SOFT,
 *                         flag BOARDIOC_SOFTRESETCAUSE_USER_REBOOT
 *     debugger            BOARDIOC_RESETCAUSE_UNKOWN
 *     power-on            BOARDIOC_RESETCAUSE_SYS_CHIPPOR
 *     WARMRSTn pin        BOARDIOC_RESETCAUSE_PIN
 *     none (JTAG load)    BOARDIOC_RESETCAUSE_NONE
 *
 *   For the causes without a flag of their own, flag is the raw
 *   WARM_RST_CAUSE (am263x_reset_cause_boot()).
 *
 ****************************************************************************/

#ifdef CONFIG_BOARDCTL
struct boardioc_reset_cause_s;
int am263x_reset_cause_get(FAR struct boardioc_reset_cause_s *rc);
#endif

#endif /* __ARCH_ARM_SRC_AM263X_AM263X_SYSTEMRESET_H */
