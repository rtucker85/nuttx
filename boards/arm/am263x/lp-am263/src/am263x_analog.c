/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_analog.c
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

/* The on-chip ADC and ePWM on the LP-AM263.
 *
 *   /dev/adc0  ADC1 AIN0-5, BoosterPack headers (ADC1_AIN0 is J3.24)
 *   /dev/pwm0  EPWM0, outputs not pinned out
 *
 * With CONFIG_LP_AM263_ADC1_EPWM0, ADC1 scans at the end of every EPWM0
 * period while /dev/pwm0 runs; otherwise on ANIOC_TRIGGER.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>

#include <nuttx/analog/adc.h>
#include <nuttx/timers/pwm.h>

#include "am263x_adc.h"
#include "am263x_pwm.h"
#include "lp-am263.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_AM263X_ADC1
static const struct am263x_adc_config_s g_adc1_config =
{
#ifdef CONFIG_LP_AM263_ADC1_EPWM0
  .trigger   = AM263X_ADC_TRIG_EPWM_SOCA(0),
#else
  .trigger   = AM263X_ADC_TRIG_SW,
#endif
  .nchannels = 6,
  .channels  =
  {
    AM263X_ADC_AIN(0), AM263X_ADC_AIN(1), AM263X_ADC_AIN(2),
    AM263X_ADC_AIN(3), AM263X_ADC_AIN(4), AM263X_ADC_AIN(5),
  },
};
#endif

#ifdef CONFIG_LP_AM263_ADC1_EPWM0
static const struct am263x_epwm_soc_s g_epwm0_soca =
{
  .event    = AM263X_EPWM_SOC_PERIOD,
  .prescale = 1,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_analog_initialize
 *
 * Description:
 *   Register /dev/pwm0 and /dev/adc0.
 *
 ****************************************************************************/

int am263x_analog_initialize(void)
{
#ifdef CONFIG_AM263X_EPWM
  FAR struct pwm_lowerhalf_s *pwm;
#endif
#ifdef CONFIG_AM263X_ADC1
  FAR struct adc_dev_s *adc;
#endif
  int ret = OK;

#ifdef CONFIG_AM263X_EPWM
  pwm = am263x_pwminitialize(0);
  if (pwm == NULL)
    {
      return -ENODEV;
    }

#  ifdef CONFIG_LP_AM263_ADC1_EPWM0
  ret = am263x_pwm_socconfig(pwm, AM263X_EPWM_SOCA, &g_epwm0_soca);
  if (ret < 0)
    {
      return ret;
    }
#  endif

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_AM263X_ADC1
  adc = am263x_adc_initialize(1, &g_adc1_config);
  if (adc == NULL)
    {
      return -ENODEV;
    }

  ret = adc_register("/dev/adc0", adc);
#endif

  return ret;
}
