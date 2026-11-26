/****************************************************************************
 * arch/arm/src/am263x/am263x_systemreset.c
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

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/am263x_rcm.h"
#include "am263x_systemreset.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The WARM_RST_CAUSE bits, in bit order, for am263x_reset_report() */

static const char *const g_rst_cause[] =
{
  "power-on",
  "MSS WDT0",
  "MSS WDT1",
  "MSS WDT2",
  "MSS WDT3",
  "software",
  NULL,                             /* Bit 6: set with every cause */
  "HSM WDT",
  "debugger",
  "TEMP_SENSOR0_RESET (TSENSE0 thermal shutdown)",
  "TEMP_SENSOR1_RESET (TSENSE1 thermal shutdown)",
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_systemreset
 *
 * Description:
 *   Warm-reset the SoC through TOP_RCM.  The ROM then boots again in the
 *   boot mode latched at power-on: from QSPI flash in QSPI boot mode, or
 *   waiting for an image in UART boot mode.
 *
 ****************************************************************************/

void up_systemreset(void)
{
  up_irq_save();

  putreg32(AM263X_KICK0_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK1);
  putreg32(0, AM263X_TOP_RCM_WARM_RESET_REQ);

  for (; ; )
    {
      asm volatile("wfi");
    }
}

/****************************************************************************
 * Name: am263x_reset_cause
 ****************************************************************************/

uint32_t am263x_reset_cause(bool clear)
{
  uint32_t cause = getreg32(AM263X_TOP_RCM_WARM_RST_CAUSE) &
                   TOP_RCM_RST_CAUSE_MASK;

  if (clear)
    {
      putreg32(AM263X_KICK0_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK0);
      putreg32(AM263X_KICK1_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK1);
      putreg32(TOP_RCM_RST_CAUSE_CLEAR, AM263X_TOP_RCM_WARM_RST_CAUSE_CLR);
      putreg32(AM263X_KICK_LOCK, AM263X_TOP_RCM_LOCK0_KICK0);
      putreg32(AM263X_KICK_LOCK, AM263X_TOP_RCM_LOCK0_KICK1);
    }

  return cause;
}

/****************************************************************************
 * Name: am263x_reset_report
 ****************************************************************************/

uint32_t am263x_reset_report(void)
{
  uint32_t cause = am263x_reset_cause(true);
  uint32_t bits = cause & ~TOP_RCM_RST_CAUSE_WARM;
  int i;

  if (cause == 0)
    {
      /* Cleared at the last boot and no reset since: a debugger load */

      syslog(LOG_INFO, "Reset cause: none since the last boot\n");
      return cause;
    }

  if (bits == 0)
    {
      syslog(LOG_INFO, "Reset cause %03" PRIx32 ": WARMRSTn pin\n", cause);
      return cause;
    }

  for (i = 0; i < sizeof(g_rst_cause) / sizeof(g_rst_cause[0]); i++)
    {
      if ((bits & (1 << i)) != 0 && g_rst_cause[i] != NULL)
        {
          syslog((bits & (TOP_RCM_RST_CAUSE_TSENSE0 |
                          TOP_RCM_RST_CAUSE_TSENSE1)) != 0 ?
                 LOG_WARNING : LOG_INFO,
                 "Reset cause %03" PRIx32 ": %s\n", cause, g_rst_cause[i]);
        }
    }

  return cause;
}
