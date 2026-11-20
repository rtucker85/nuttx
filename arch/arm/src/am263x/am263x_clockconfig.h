/****************************************************************************
 * arch/arm/src/am263x/am263x_clockconfig.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_AM263X_AM263X_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocks the port relies on.  The SBL starts the PLLs; these are the
 * resulting rates with TI's default PLL setup.
 */

#define AM263X_XTAL_FREQUENCY       25000000  /* LP-AM263 crystal */
#define AM263X_SYSCLK_FREQUENCY     200000000 /* ADC and ePWM clock */
#define AM263X_UART0_FREQUENCY      48000000  /* PER_HSDIV0_CLKOUT1 / 4 */
#define AM263X_RTI0_FREQUENCY       AM263X_XTAL_FREQUENCY
#define AM263X_I2C_FREQUENCY        96000000  /* PER_HSDIV0_CLKOUT1 / 2 */
#define AM263X_MCSPI_FREQUENCY      48000000  /* PER_HSDIV0_CLKOUT1 / 4 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_clockconfig
 *
 * Description:
 *   Select and ungate the functional clocks of the peripherals used by the
 *   core port (UART0 for the console, RTI0 for the system tick).  Called
 *   early from arm_boot().  PLLs are left as the SBL configured them.
 *
 ****************************************************************************/

void am263x_clockconfig(void);

#endif /* __ARCH_ARM_SRC_AM263X_AM263X_CLOCKCONFIG_H */
