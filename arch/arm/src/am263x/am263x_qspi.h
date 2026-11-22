/****************************************************************************
 * arch/arm/src/am263x/am263x_qspi.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_QSPI_H
#define __ARCH_ARM_SRC_AM263X_AM263X_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/spi/qspi.h>

#ifdef CONFIG_AM263X_QSPI

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_qspi_initialize
 *
 * Description:
 *   Initialize QSPI<intf> (only 0 exists) and return its interface, or
 *   NULL for any other number.  It drives chip select 0, the boot flash.
 *   The board sets up the pinmux.
 *
 ****************************************************************************/

FAR struct qspi_dev_s *am263x_qspi_initialize(int intf);

#endif /* CONFIG_AM263X_QSPI */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_QSPI_H */
