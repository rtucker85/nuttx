/****************************************************************************
 * arch/arm/src/am263x/am263x_mpuinit.c
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

#include <nuttx/arch.h>

#include "cp15_cacheops.h"
#include "mpu.h"
#include "sctlr.h"
#include "chip.h"
#include "am263x_mpuinit.h"
#include "am263x_cpsw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Region attributes.  The Cortex-R5 MPU gives the highest-numbered
 * matching region priority, so the table runs from the background region
 * up to the most specific ones.
 *
 * Normal memory is Non-shareable on purpose: on the R5F, LDREX/STREX to
 * Shareable normal memory goes to the external monitor and aborts (seen
 * on the AM67 R5F port).  A single core does not need Shareable.
 */

/* Strongly-ordered, execute-never: peripherals and anything unmapped */

#define AM263X_MPU_DEVICE        (MPU_RACR_AP_RWRW | MPU_RACR_XN)

/* Normal, non-cacheable (TEX=1, C=0, B=0), execute-never: DMA descriptors */

#define AM263X_MPU_NORMAL_NC     (MPU_RACR_TEX(1) | MPU_RACR_AP_RWRW | \
                                  MPU_RACR_XN)

/* Normal, write-back write-allocate (TEX=1, C=1, B=1), executable */

#define AM263X_MPU_NORMAL_WB     (MPU_RACR_TEX(1) | MPU_RACR_C | \
                                  MPU_RACR_B | MPU_RACR_AP_RWRW)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mpu_region_s g_am263x_mpu_regions[] =
{
  { 0x00000000,             0x80000000,        AM263X_MPU_DEVICE    },
  { AM263X_TCMA_BASE,       AM263X_TCMA_SIZE,  AM263X_MPU_NORMAL_WB },
  { AM263X_TCMB_BASE,       AM263X_TCMB_SIZE,  AM263X_MPU_NORMAL_WB },
  { AM263X_OCRAM_BASE,      AM263X_OCRAM_SIZE, AM263X_MPU_NORMAL_WB },
#ifdef CONFIG_AM263X_CPSW_ETH
  { (uintptr_t)g_am263x_cpsw_desc, AM263X_CPSW_DESC_REGION_SIZE,
    AM263X_MPU_NORMAL_NC },
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mpu_clear
 *
 * Description:
 *   Disable every region the MPU implements.  (The armv7-r
 *   CONFIG_ARM_MPU_EARLY_RESET path does not build: it uses Cortex-M
 *   register names.)
 *
 ****************************************************************************/

static void am263x_mpu_clear(void)
{
  unsigned int nregions;
  unsigned int region;

  nregions = (mpu_get_mpuir() & MPUIR_DREGION_MASK) >> MPUIR_DREGION_SHIFT;

  for (region = 0; region < nregions; region++)
    {
      mpu_set_rgnr(region);
      mpu_set_drsr(0);
      mpu_set_dracr(0);
      mpu_set_drbar(0);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mpu_init
 ****************************************************************************/

void am263x_mpu_init(void)
{
  uint32_t sctlr;

  /* arm_head.S has already turned the MPU and the D-cache on, with
   * whatever regions the SBL left behind still enabled, and has zeroed
   * .bss and pushed stack frames through the (write-back) D-cache.  With
   * the MPU off, data accesses use the default map, which is not
   * cacheable, and the architecture does not promise they see dirty
   * lines.  Write the D-cache back and turn it off first, with no stores
   * in between.
   */

  up_flush_dcache_all();
  cp15_disable_dcache();

  /* Turn the MPU off, clear every region, and stop PL1 falling through
   * to the background map so that anything not listed above faults
   * instead of silently using default attributes.
   */

  mpu_control(false);
  am263x_mpu_clear();

  sctlr = cp15_rdsctlr();
  sctlr &= ~SCTLR_BR;
  cp15_wrsctlr(sctlr);

  mpu_initialize(g_am263x_mpu_regions,
                 sizeof(g_am263x_mpu_regions) /
                 sizeof(g_am263x_mpu_regions[0]));

  /* Back on with the real attributes.  Lines left in the cache are stale
   * after the uncached stores above; up_enable_dcache() invalidates first.
   */

#ifndef CONFIG_ARMV7R_DCACHE_DISABLE
  up_enable_dcache();
#endif
}
