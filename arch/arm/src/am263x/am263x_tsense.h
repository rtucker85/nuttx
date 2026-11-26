/****************************************************************************
 * arch/arm/src/am263x/am263x_tsense.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_TSENSE_H
#define __ARCH_ARM_SRC_AM263X_AM263X_TSENSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/chip/tsense.h>

#ifdef CONFIG_AM263X_TSENSE

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_tsense_initialize
 *
 * Description:
 *   Put the thermal manager in a known state whatever the boot loader
 *   left: power the four sensors up, start the FSM converting all of them
 *   round robin, and read the factory trims.  Alerts are masked unless
 *   CONFIG_AM263X_TSENSE_ALERT.  The thermal shutdown comparators and the
 *   TOP_RCM warm-reset enables are not touched.  Called by
 *   am263x_tsense_register(); safe to call again.
 *
 * Returned Value:
 *   Zero on success; a negated errno value if no conversion completed.
 *
 ****************************************************************************/

int am263x_tsense_initialize(void);

/****************************************************************************
 * Name: am263x_tsense_read
 *
 * Description:
 *   The last temperature of sensor n (0-3), in degC.  With factory trims
 *   it is interpolated between them; without, it follows TRM Table
 *   6-22.  TSENSE3 has no documented conversion (-ENODATA).
 *
 ****************************************************************************/

int am263x_tsense_read(int n, FAR float *celsius);

/****************************************************************************
 * Name: am263x_tsense_register
 *
 * Description:
 *   Initialize the sensors and register the uORB temperature topic
 *   /dev/uorb/sensor_temp<devno>, the hottest of TSENSE0-2, read at the
 *   interval subscribers ask for.
 *
 ****************************************************************************/

int am263x_tsense_register(int devno);

#endif /* CONFIG_AM263X_TSENSE */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_TSENSE_H */
