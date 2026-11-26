/****************************************************************************
 * arch/arm/src/am263x/am263x_controlss.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_CONTROLSS_H
#define __ARCH_ARM_SRC_AM263X_AM263X_CONTROLSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "hardware/am263x_controlss.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_css_ctrl_modify
 *
 * Description:
 *   Read-modify-write a 32-bit CONTROLSS_CTRL register: unlock the
 *   partition, clear then set bits, lock it again.  Safe against other
 *   callers and interrupt handlers.
 *
 ****************************************************************************/

void am263x_css_ctrl_modify(uintptr_t addr, uint32_t clrbits,
                            uint32_t setbits);

/****************************************************************************
 * Name: am263x_top_ctrl_modify
 *
 * Description:
 *   As am263x_css_ctrl_modify(), for a 32-bit TOP_CTRL register.
 *
 ****************************************************************************/

void am263x_top_ctrl_modify(uintptr_t addr, uint32_t clrbits,
                            uint32_t setbits);

/****************************************************************************
 * Name: am263x_top_ctrl_modify16
 *
 * Description:
 *   As am263x_css_ctrl_modify(), for a 16-bit TOP_CTRL register.
 *
 ****************************************************************************/

void am263x_top_ctrl_modify16(uintptr_t addr, uint16_t clrbits,
                              uint16_t setbits);

/****************************************************************************
 * Name: am263x_intxbar_select
 *
 * Description:
 *   Add (enable true) or remove the inputs in mask from group <group> of
 *   INTXBAR output <out>.  The output drives interrupt
 *   AM263X_IRQ_INTXBAR(out).
 *
 ****************************************************************************/

void am263x_intxbar_select(int out, int group, uint32_t mask, bool enable);

#endif /* __ARCH_ARM_SRC_AM263X_AM263X_CONTROLSS_H */
