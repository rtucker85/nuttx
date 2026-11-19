/****************************************************************************
 * arch/arm/src/am263x/am263x_boot.c
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "am263x_boot.h"
#include "am263x_clockconfig.h"
#include "am263x_mpuinit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Continue the boot after arm_head.S has set up the idle stack, the
 *   caches, and zeroed .bss.
 *
 *   How the core got here: the TI SBL copied every loadable segment of the
 *   image (vectors into ATCM at 0x0, everything else into OCRAM) and reset
 *   R5SS0.  The ELF entry point is ignored; the core starts at the reset
 *   vector at 0x0, which jumps to __start.  The SBL has started the PLLs.
 *
 ****************************************************************************/

void arm_boot(void)
{
  /* C code is built hard-float, so the FPU goes on first */

#ifdef CONFIG_ARCH_FPU
  arm_fpuconfig();
#endif

  /* Real MPU regions before touching any peripheral: until now the
   * background map has been in force.
   */

  am263x_mpu_init();

  am263x_clockconfig();

#ifdef CONFIG_ARCH_PERF_EVENTS
  /* Start the PMU cycle counter behind up_perf_gettime() and so
   * perf_gettime().
   */

  up_perf_init((void *)BOARD_CPU_FREQUENCY);
#endif

  /* Board pinmux, at least the console pins */

  am263x_board_initialize();

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  nx_start();
}
