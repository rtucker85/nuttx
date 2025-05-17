/****************************************************************************
 * arch/arm/src/s32k3xx/s32k3xx_clockconfig.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm_internal.h"

#include "Clock_Ip.h"
#include "Clock_Ip_Private.h"
#include "s32k3xx_clockconfig.h"
#include "hardware/s32k3xx_pinmux.h"

#include <arch/board/board.h>  /* Include last.  May have dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXP_S32_CLOCK_CONFIG_IDX 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s32k3xx_clockconfig
 *
 * Description:
 *   Called to initialize the S32K3XX.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.  This function also performs
 *   other low-level chip as necessary.
 *
 * Input Parameters:(
 *   clkcfg - Describes the new clock configuration
 *
 * Returned Value:
 *   Zero (OK) is returned a success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int s32k3xx_clockconfig(void)
{
  s32k3xx_pinconfig(PIN_EMAC_MII_RMII_TX_CLK_2);

  Clock_Ip_StatusType ret = Clock_Ip_Init(&Clock_Ip_aClockConfig[NXP_S32_CLOCK_CONFIG_IDX]);
  if (ret == CLOCK_IP_SUCCESS)
    return 0;
  return -1;
}

/****************************************************************************
 * Name: s32k3xx_get_freq
 *
 * Description:
 *    clock frequency from a clock source.
 *
 * Input Parameters:
 *   clksrc - The requested clock source.
 *
 * Returned Value:
 *   The frequency of the requested clock source.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_freq(Clock_Ip_NameType clksrc)
{
    return Clock_Ip_GetFreq(clksrc);
}

/****************************************************************************
 * Name: s32k3xx_get_coreclk
 *
 * Description:
 *   Return the current value of the CORE clock frequency.
 *
 * Input Parameters:
 *   None
 *
 * Returned Values:
 *   The current value of the CORE clock frequency.  Zero is returned on any
 *   failure.
 *
 ****************************************************************************/

uint32_t s32k3xx_get_coreclk(void)
{
  return s32k3xx_get_freq(CORE_CLK);
}
