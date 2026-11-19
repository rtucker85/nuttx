/****************************************************************************
 * arch/arm/src/mspm0/mspm0_start.c
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

#include <stdint.h>

#include <nuttx/init.h>

#include "arm_internal.h"
#include "mspm0_lowputc.h"
#include "mspm0_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory map (bank 0 of SRAM only):
 *
 * 0x0000:0000 - Start of flash: vector table, .text, .rodata, .data image
 * 0x2020:0000 - Start of SRAM bank 0 and of .data (_sdata)
 *             - End of .data (_edata) and start of .bss (_sbss)
 *             - End of .bss (_ebss) and bottom of the idle stack
 *             - _ebss + CONFIG_IDLETHREAD_STACKSIZE: top of the idle
 *               stack and start of the heap
 * 0x2020:ffff - End of SRAM bank 0 and of the heap
 */

#define IDLE_STACK ((uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack = IDLE_STACK;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.  The chip comes out of reset running
 *   from SYSOSC at 32 MHz, which is what NuttX uses for now.
 *
 ****************************************************************************/

void __start(void)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Set up the console UART so that arm_lowputc() works from here on.
   * This must not use .data or .bss.
   */

  mspm0_lowsetup();

  /* Clear .bss.  This is done inline (rather than with memset) so that
   * nothing depends on global variables before they are set up.
   */

  dest = (uint32_t *)_sbss;

  while (dest < (uint32_t *)_ebss)
    {
      *dest++ = 0;
    }

  /* Copy the initialized data from flash, where it starts at _eronly, to
   * its place in SRAM between _sdata and _edata.
   */

  src  = (const uint32_t *)_eronly;
  dest = (uint32_t *)_sdata;

  while (dest < (uint32_t *)_edata)
    {
      *dest++ = *src++;
    }

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  /* Initialize on-board resources */

  mspm0_boardinitialize();

  /* Then start NuttX */

  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
