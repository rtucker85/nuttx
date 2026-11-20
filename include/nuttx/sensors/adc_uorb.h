/****************************************************************************
 * include/nuttx/sensors/adc_uorb.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ADC_UORB_H
#define __INCLUDE_NUTTX_SENSORS_ADC_UORB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC channel numbers the bridge can handle: 0 to ADC_UORB_MAXCHAN - 1 */

#define ADC_UORB_MAXCHAN        32

/* Path of the raw topic, %d is the devno passed in the configuration */

#define ADC_UORB_RAW_FMT        "/dev/uorb/adc_raw%d"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* One published channel.  The value published is
 *
 *   gain * f(volts) + offset
 *
 * where volts is the voltage at the ADC input and f is convert, or the
 * identity when convert is NULL.  A linear input (a divider, a current
 * sense resistor, a linear temperature sensor) needs only gain and
 * offset; a non-linear one (a thermistor) supplies convert and leaves
 * gain at 1 and offset at 0, so that SNIOC_SET_CALIBVALUE can still trim
 * it.  Values are in the SI units of the topic: V, A, degC, ...
 */

struct adc_uorb_chan_s
{
  uint8_t channel;   /* ADC channel number */
  uint8_t type;      /* SENSOR_TYPE_VOLTAGE, _CURRENT, _TEMPERATURE,
                      * _AMBIENT_TEMPERATURE, _RELATIVE_HUMIDITY, _POWER or
                      * _RESISTANCE */
  uint8_t devno;     /* Topic instance: /dev/uorb/sensor_<type><devno> */
  float   gain;
  float   offset;
  CODE float (*convert)(FAR const struct adc_uorb_chan_s *chan,
                        float volts);
  FAR void *arg;     /* For convert */
};

struct adc_dev_s;

struct adc_uorb_config_s
{
  FAR struct adc_dev_s *adc;      /* Lower half; must handle ANIOC_TRIGGER */
  float                 vref;     /* Reference voltage, V */
  uint8_t               bits;     /* Resolution: full scale is 2^bits */
  uint8_t               nchans;
  FAR const struct adc_uorb_chan_s *chans;
  int                   rawdevno; /* Raw topic instance, or -1 for none */
};

/* SNIOC_SET_CALIBVALUE sets, and SNIOC_CALIBRATE reads back, a channel's
 * gain and offset.  arg: FAR struct adc_uorb_calib_s *
 */

struct adc_uorb_calib_s
{
  float gain;
  float offset;
};

/* The raw topic: the codes of every configured channel from one scan */

struct adc_uorb_raw_s
{
  uint64_t timestamp;                  /* Microseconds */
  uint32_t nchans;
  uint8_t  channel[ADC_UORB_MAXCHAN];
  int32_t  code[ADC_UORB_MAXCHAN];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: adc_uorb_register
 *
 * Description:
 *   Bind an ADC lower half and publish its channels, converted, as uORB
 *   topics.  The bridge owns the lower half: do not also pass it to
 *   adc_register().  All channels are converted together in one scan, at
 *   the shortest interval any active topic asks for.
 *
 *   config and config->chans must persist as long as the bridge does.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int adc_uorb_register(FAR const struct adc_uorb_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_ADC_UORB_H */
