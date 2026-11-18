/****************************************************************************
 * arch/arm/src/am263x/am263x_pinmux.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_PINMUX_H
#define __ARCH_ARM_SRC_AM263X_AM263X_PINMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "hardware/am263x_iomux.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One pad setting: pad register offset and its value */

struct am263x_pinmux_s
{
  uint16_t offset;
  uint32_t value;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_pinmux_config
 *
 * Description:
 *   Apply a table of pad settings, unlocking the IOMUX around the writes.
 *
 ****************************************************************************/

void am263x_pinmux_config(const struct am263x_pinmux_s *pins, int count);

#endif /* __ARCH_ARM_SRC_AM263X_AM263X_PINMUX_H */
