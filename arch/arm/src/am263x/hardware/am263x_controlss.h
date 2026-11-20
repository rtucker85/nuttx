/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_controlss.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CONTROLSS_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CONTROLSS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONTROLSS_CTRL: clock gates and resets of the control peripherals, the
 * ePWM time-base clock sync.  Locked; unlock with the usual kick values.
 */

#define AM263X_CSS_CTRL_EPWM_CLKSYNC    (AM263X_CSS_CTRL_BASE + 0x0010)
#define AM263X_CSS_CTRL_EPWM_CLK_GATE(n) \
  (AM263X_CSS_CTRL_BASE + 0x0100 + ((n) << 2))
#define AM263X_CSS_CTRL_ADC_CLK_GATE(n) \
  (AM263X_CSS_CTRL_BASE + 0x025c + ((n) << 2))
#define AM263X_CSS_CTRL_LOCK0_KICK0     (AM263X_CSS_CTRL_BASE + 0x1008)
#define AM263X_CSS_CTRL_LOCK0_KICK1     (AM263X_CSS_CTRL_BASE + 0x100c)

#define CSS_CTRL_CLK_GATED              0x7  /* ADCn/EPWMn_CLK_GATE */
#define CSS_CTRL_CLK_UNGATED            0x0

/* INTXBAR: 32 outputs, each the OR of the inputs selected in its seven
 * group registers.  Output n is R5FSS0-0 interrupt 146 + n.
 */

#define AM263X_INTXBAR_NOUTPUTS         32
#define AM263X_INTXBAR_SEL(out, g) \
  (AM263X_INTXBAR_BASE + 0x100 + ((out) << 6) + ((g) << 2))
#define AM263X_IRQ_INTXBAR(out)         (146 + (out))

/* Group 2 holds the ADC interrupts: five bits per ADC, INT1-4 then EVT */

#define INTXBAR_GROUP_ADC               2
#define INTXBAR_ADC_INT(adc, i)         (1 << ((adc) * 5 + (i) - 1))

/* TOP_CTRL: the ADC reference buffers and monitors.  ADC0 has its own
 * reference; ADC1/2 share one and ADC3/4 another.  REFBUF1 drives the
 * ADC3/4 reference, REFBUF0 both of the others.
 */

#define AM263X_TOP_CTRL_ADC_REFBUF_CTRL(n) \
  (AM263X_TOP_CTRL_BASE + 0x0c00 + ((n) << 2))
#define AM263X_TOP_CTRL_ADC_REF_COMP_CTRL (AM263X_TOP_CTRL_BASE + 0x0c08)
#define AM263X_TOP_CTRL_ADC_REF_GOOD    (AM263X_TOP_CTRL_BASE + 0x0c0c)
#define AM263X_TOP_CTRL_MASK_ANA_ISO    (AM263X_TOP_CTRL_BASE + 0x0c30)
#define AM263X_TOP_CTRL_LOCK0_KICK0     (AM263X_TOP_CTRL_BASE + 0x1008)
#define AM263X_TOP_CTRL_LOCK0_KICK1     (AM263X_TOP_CTRL_BASE + 0x100c)

#define TOP_CTRL_ADC_REFBUF_ENABLE      0x7
#define TOP_CTRL_ADC_REFOK_EN_ADC0      (0x7 << 0)
#define TOP_CTRL_ADC_REFOK_EN_ADC12     (0x7 << 4)
#define TOP_CTRL_ADC_REFOK_EN_ADC34     (0x7 << 8)
#define TOP_CTRL_ADC_REF_GOOD_ADC34     (0x3 << 0)  /* OV and UV good */
#define TOP_CTRL_ADC_REF_GOOD_ADC0      (0x3 << 2)
#define TOP_CTRL_ADC_REF_GOOD_ADC12     (0x3 << 4)
#define TOP_CTRL_MASK_ANA_ISO_HHV       0x7  /* mask the HHV reset */

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CONTROLSS_H */
