/****************************************************************************
 * arch/arm/include/am263x/pru.h
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

/* User interface of the ICSSM PRU driver (/dev/pruN).
 *
 * read(), write() and lseek() access the PRU's data RAM (DMEM).  Offsets
 * and lengths must be multiples of 4: the RAM is accessed by word.
 *
 * ioctl() controls the core:
 *
 *   PRUIOC_HALT    Stop the core.  arg: unused.
 *   PRUIOC_LOAD    Stop the core, clear its IRAM and DMEM, and copy in an
 *                  image.  arg: const struct pru_image_s *.
 *   PRUIOC_START   Reset the core with its PC at a word address in IRAM
 *                  and run it.  arg: the start address, in words.
 *   PRUIOC_STATUS  arg: struct pru_status_s *.
 */

#ifndef __ARCH_ARM_INCLUDE_AM263X_PRU_H
#define __ARCH_ARM_INCLUDE_AM263X_PRU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PRUIOC_HALT    _PRUIOC(0x0001)
#define PRUIOC_LOAD    _PRUIOC(0x0002)
#define PRUIOC_START   _PRUIOC(0x0003)
#define PRUIOC_STATUS  _PRUIOC(0x0004)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A firmware image.  Sizes are in bytes and must be multiples of 4.
 * Either part may be empty.  Anything beyond the image is cleared.
 */

struct pru_image_s
{
  FAR const uint32_t *iram;    /* Instructions, loaded at IRAM offset 0 */
  size_t iram_size;
  FAR const uint32_t *dram;    /* Data, loaded at DMEM offset 0 */
  size_t dram_size;
};

struct pru_status_s
{
  bool running;                /* Executing instructions */
  bool sleeping;               /* Stopped in a SLP instruction */
  uint16_t pc;                 /* Program counter, in words */
  uint32_t control;            /* Raw CONTROL register */
};

#endif /* __ARCH_ARM_INCLUDE_AM263X_PRU_H */
