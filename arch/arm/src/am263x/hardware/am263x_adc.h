/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_adc.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_ADC_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Five 12-bit SAR ADCs.  Each has a control block and a separate result
 * block, both 4 KB apart per instance.  Most registers are 16 bits wide
 * and must be accessed as such; ADCSOCxCTL is 32 bits.
 */

#define AM263X_ADC_NINSTANCES       5
#define AM263X_ADC_NSOC             16
#define AM263X_ADC_BASE(n)          (AM263X_ADC0_BASE + ((n) << 12))
#define AM263X_ADC_RESULT_BASE(n)   (AM263X_ADC0_RESULT_BASE + ((n) << 12))

/* Control register offsets */

#define AM263X_ADC_CTL1             0x0000
#define AM263X_ADC_CTL2             0x0002
#define AM263X_ADC_BURSTCTL         0x0004
#define AM263X_ADC_INTFLG           0x0006
#define AM263X_ADC_INTFLGCLR        0x0008
#define AM263X_ADC_INTOVF           0x000a
#define AM263X_ADC_INTOVFCLR        0x000c
#define AM263X_ADC_INTSEL1N2        0x000e
#define AM263X_ADC_INTSEL3N4        0x0010
#define AM263X_ADC_SOCPRICTL        0x0012
#define AM263X_ADC_INTSOCSEL1       0x0014
#define AM263X_ADC_INTSOCSEL2       0x0016
#define AM263X_ADC_SOCFLG1          0x0018
#define AM263X_ADC_SOCFRC1          0x001a
#define AM263X_ADC_SOCOVF1          0x001c
#define AM263X_ADC_SOCOVFCLR1       0x001e
#define AM263X_ADC_SOCCTL(n)        (0x0020 + ((n) << 2))
#define AM263X_ADC_REV              0x0074
#define AM263X_ADC_OFFTRIM          0x0076
#define AM263X_ADC_INTCYCLE         0x00de

/* Result register offsets, in the result block */

#define AM263X_ADC_RESULT(n)        ((n) << 1)

/* ADCCTL1 */

#define ADC_CTL1_INTPULSEPOS        (1 << 2)  /* ADCINT at end of conversion */
#define ADC_CTL1_ADCPWDNZ           (1 << 7)  /* analog circuits powered */
#define ADC_CTL1_ADCBSY             (1 << 13)

/* ADCCTL2 */

#define ADC_CTL2_PRESCALE_SHIFT     0
#define ADC_CTL2_PRESCALE_MASK      (0xf << ADC_CTL2_PRESCALE_SHIFT)
#define ADC_CTL2_RESOLUTION         (1 << 6)
#define ADC_CTL2_SIGNALMODE         (1 << 7)  /* differential */

/* ADCINTFLG, ADCINTFLGCLR, ADCINTOVF, ADCINTOVFCLR */

#define ADC_INT(i)                  (1 << ((i) - 1))  /* ADCINT1-4 */

/* ADCINTSEL1N2: ADCINT1 in the low byte, ADCINT2 in the high */

#define ADC_INTSEL_SEL_SHIFT        0     /* EOC that sets the flag */
#define ADC_INTSEL_SEL_MASK         (0xf << ADC_INTSEL_SEL_SHIFT)
#define ADC_INTSEL_E                (1 << 5)  /* interrupt enable */
#define ADC_INTSEL_CONT             (1 << 6)  /* pulse even if flag set */

/* ADCSOCxCTL */

#define ADC_SOCCTL_ACQPS_SHIFT      0     /* sample window - 1, SYSCLKs */
#define ADC_SOCCTL_ACQPS_MASK       (0x1ff << ADC_SOCCTL_ACQPS_SHIFT)
#define ADC_SOCCTL_CHSEL_SHIFT      15
#define ADC_SOCCTL_CHSEL_MASK       (0x1f << ADC_SOCCTL_CHSEL_SHIFT)
#define ADC_SOCCTL_TRIGSEL_SHIFT    20
#define ADC_SOCCTL_TRIGSEL_MASK     (0x7f << ADC_SOCCTL_TRIGSEL_SHIFT)

/* TRIGSEL values */

#define ADC_TRIGSEL_SW              0x00
#define ADC_TRIGSEL_RTI(n)          (0x01 + (n))         /* RTI0-3 */
#define ADC_TRIGSEL_INPUTXBAR5      0x05
#define ADC_TRIGSEL_EPWM_SOCA(n)    (0x08 + ((n) << 1))  /* EPWM0-31 */
#define ADC_TRIGSEL_EPWM_SOCB(n)    (0x09 + ((n) << 1))
#define ADC_TRIGSEL_ECAP(n)         (0x48 + (n))         /* ECAP0-9 */
#define ADC_TRIGSEL_MAX             0x51

/* CHSEL values in single-ended mode */

#define ADC_CHSEL_AIN(n)            (n)   /* ADCINx, 0-5 */
#define ADC_CHSEL_CAL0              6
#define ADC_CHSEL_CAL1              7
#define ADC_CHSEL_MAX               7

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_ADC_H */
