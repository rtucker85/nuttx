/****************************************************************************
 * arch/arm/src/am263x/am263x_adc.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_ADC_H
#define __ARCH_ARM_SRC_AM263X_AM263X_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/analog/adc.h>

#include "hardware/am263x_adc.h"

#ifdef CONFIG_AM263X_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* What starts a scan.  ANIOC_TRIGGER starts one whatever is chosen here. */

#define AM263X_ADC_TRIG_SW            ADC_TRIGSEL_SW
#define AM263X_ADC_TRIG_RTI(n)        ADC_TRIGSEL_RTI(n)
#define AM263X_ADC_TRIG_EPWM_SOCA(n)  ADC_TRIGSEL_EPWM_SOCA(n)
#define AM263X_ADC_TRIG_EPWM_SOCB(n)  ADC_TRIGSEL_EPWM_SOCB(n)

/* Inputs.  CAL0 and CAL1 are internal calibration inputs. */

#define AM263X_ADC_AIN(n)             ADC_CHSEL_AIN(n)
#define AM263X_ADC_CAL0               ADC_CHSEL_CAL0
#define AM263X_ADC_CAL1               ADC_CHSEL_CAL1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* An ADC's scan: one SOC per entry in channels[], converted in order on
 * every trigger.  Each result goes to the upper half tagged with its
 * channel number.
 */

struct am263x_adc_config_s
{
  uint8_t trigger;                        /* AM263X_ADC_TRIG_* */
  uint8_t nchannels;                      /* 1 to AM263X_ADC_NSOC */
  uint8_t channels[AM263X_ADC_NSOC];      /* AM263X_ADC_AIN(n), CAL0/1 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_adc_initialize
 *
 * Description:
 *   Return the lower half for ADC<adc> (0-4), set up to scan the channels
 *   in config, for adc_register() or another upper half.  Nothing is
 *   powered until the upper half calls setup().  The analog inputs are
 *   dedicated pins and need no pinmux.
 *
 * Returned Value:
 *   The lower half, or NULL if that ADC is not enabled or config is not
 *   valid.
 *
 ****************************************************************************/

FAR struct adc_dev_s *
am263x_adc_initialize(int adc, FAR const struct am263x_adc_config_s *config);

#endif /* CONFIG_AM263X_ADC */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_ADC_H */
