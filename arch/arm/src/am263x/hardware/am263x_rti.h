/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_rti.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RTI_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RTI_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTI (real-time interrupt) timer register offsets */

#define RTI_GCTRL_OFFSET            0x00  /* Global control */
#define RTI_COMPCTRL_OFFSET         0x0c  /* Compare control */
#define RTI_FRC0_OFFSET             0x10  /* Free-running counter 0 */
#define RTI_UC0_OFFSET              0x14  /* Up-counter 0 (prescaler) */
#define RTI_CPUC0_OFFSET            0x18  /* Compare up-counter 0 */
#define RTI_COMP0_OFFSET            0x50  /* Compare 0 */
#define RTI_UDCP0_OFFSET            0x54  /* Update compare 0 */
#define RTI_COMP1_OFFSET            0x58  /* Compare 1 */
#define RTI_UDCP1_OFFSET            0x5c  /* Update compare 1 */
#define RTI_SETINT_OFFSET           0x80  /* Set interrupt enable */
#define RTI_CLEARINT_OFFSET         0x84  /* Clear interrupt enable */
#define RTI_INTFLAG_OFFSET          0x88  /* Interrupt flags, write 1 clears */

/* Register bits */

#define RTI_GCTRL_CNT0EN            (1 << 0)   /* Counter block 0 enable */
#define RTI_GCTRL_COS               (1 << 15)  /* Continue on debug suspend */
#define RTI_INT(n)                  (1 << (n)) /* Compare n interrupt, n=0..3 */

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_RTI_H */
