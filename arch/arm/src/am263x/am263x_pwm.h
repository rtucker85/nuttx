/****************************************************************************
 * arch/arm/src/am263x/am263x_pwm.h
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

#ifndef __ARCH_ARM_SRC_AM263X_AM263X_PWM_H
#define __ARCH_ARM_SRC_AM263X_AM263X_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <fixedmath.h>
#include <nuttx/timers/pwm.h>

#ifdef CONFIG_AM263X_EPWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* An ePWM's two ADC start-of-conversion outputs.  The ADC takes them as
 * AM263X_ADC_TRIG_EPWM_SOCA(n) and AM263X_ADC_TRIG_EPWM_SOCB(n).
 */

#define AM263X_EPWM_SOCA        0
#define AM263X_EPWM_SOCB        1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* When in each period an SOC fires.  The counter counts up from 0 to the
 * period; output A (channel 1) is active from 0 to its duty, output B
 * (channel 2) likewise.
 */

enum am263x_epwm_socevent_e
{
  AM263X_EPWM_SOC_DISABLED = 0,
  AM263X_EPWM_SOC_ZERO,         /* start of the period */
  AM263X_EPWM_SOC_PERIOD,       /* end of the period */
  AM263X_EPWM_SOC_ZERO_PERIOD,  /* both */
  AM263X_EPWM_SOC_DUTY_A,       /* end of output A's active part */
  AM263X_EPWM_SOC_DUTY_B,       /* end of output B's active part */
  AM263X_EPWM_SOC_POSITION,     /* at a fraction of the period */
};

struct am263x_epwm_soc_s
{
  uint8_t event;                /* enum am263x_epwm_socevent_e */
  uint8_t prescale;             /* every nth event, 1-15 */
  ub16_t  position;             /* AM263X_EPWM_SOC_POSITION: 0 to b16ONE */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_pwminitialize
 *
 * Description:
 *   Return the PWM lower half for EPWM<pwm> (0-31), for pwm_register().
 *   Channel 1 is output A, channel 2 output B.  The board sets up the
 *   pinmux of the outputs it uses; an ePWM used only to trigger the ADC
 *   needs none.
 *
 * Returned Value:
 *   The lower half, or NULL if pwm is out of range or memory ran out.
 *
 ****************************************************************************/

FAR struct pwm_lowerhalf_s *am263x_pwminitialize(int pwm);

/****************************************************************************
 * Name: am263x_pwm_socconfig
 *
 * Description:
 *   Set when the ePWM's SOCA or SOCB fires.  The setting is kept and
 *   applied whenever the PWM starts, and at once if it is running.  An SOC
 *   fires only while the PWM runs.
 *
 *   AM263X_EPWM_SOC_POSITION uses CMPC for SOCA and CMPD for SOCB, so the
 *   sample point stays at the same fraction of the period when the
 *   frequency changes.
 *
 * Returned Value:
 *   Zero on success, -EINVAL for a bad soc or config.
 *
 ****************************************************************************/

int am263x_pwm_socconfig(FAR struct pwm_lowerhalf_s *dev, int soc,
                         FAR const struct am263x_epwm_soc_s *config);

#endif /* CONFIG_AM263X_EPWM */
#endif /* __ARCH_ARM_SRC_AM263X_AM263X_PWM_H */
