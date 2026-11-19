/****************************************************************************
 * arch/arm/src/mspm0/mspm0_start.h
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

#ifndef __ARCH_ARM_SRC_MSPM0_MSPM0_START_H
#define __ARCH_ARM_SRC_MSPM0_MSPM0_START_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: mspm0_boardinitialize
 *
 * Description:
 *   All MSPM0 boards must provide this function.  It is called from
 *   __start() after .data and .bss are set up and before the OS starts,
 *   to configure pins and on-board devices.
 *
 ****************************************************************************/

void mspm0_boardinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MSPM0_MSPM0_START_H */
