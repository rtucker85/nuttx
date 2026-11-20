/****************************************************************************
 * drivers/sensors/adc_uorb.c
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

/* ADC to uORB bridge.  A scan worker on the low-priority work queue
 * triggers a conversion of every channel; the ADC hands the codes back
 * through au_receive(), possibly from an interrupt, and once the scan is
 * complete a second work item converts and publishes them.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/sensors/adc_uorb.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_SENSORS_ADC_UORB

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_UORB_DEFAULT_INTERVAL  100000   /* 10 Hz */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct adc_uorb_dev_s;

/* One topic: a converted channel, or the raw topic (chan == NULL) */

struct adc_uorb_sensor_s
{
  struct sensor_lowerhalf_s         lower;    /* Must be first */
  FAR struct adc_uorb_dev_s        *dev;
  FAR const struct adc_uorb_chan_s *chan;
  float                             gain;
  float                             offset;
  uint32_t                          interval; /* Microseconds */
  bool                              enabled;
};

/* The single-value topics share this layout */

struct adc_uorb_value_s
{
  uint64_t      timestamp;
  sensor_data_t value;
};

struct adc_uorb_dev_s
{
  FAR struct adc_uorb_dev_s        *flink;     /* g_adc_uorb list */
  FAR const struct adc_uorb_config_s *config;
  float                             lsb;       /* Volts per code */
  uint32_t                          want;      /* Channels in a scan */
  uint32_t                          interval;  /* Scan interval, us */
  bool                              running;
  bool                              setup;     /* ADC set up for a run */
  mutex_t                           lock;      /* running, interval */
  struct work_s                     scanwork;
  struct work_s                     pubwork;

  /* Filled by au_receive(), under spinlock */

  spinlock_t                        spinlock;
  uint32_t                          got;
  int32_t                           codes[ADC_UORB_MAXCHAN];

  /* The last complete scan, for the publish worker */

  uint64_t                          stamp;
  int32_t                           scan[ADC_UORB_MAXCHAN];

  int                               nsensors;
  struct adc_uorb_sensor_s          sensors[1]; /* Raw topic last */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int adc_uorb_receive(FAR struct adc_dev_s *adc, uint8_t ch,
                            int32_t data);
static int adc_uorb_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable);
static int adc_uorb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us);
static int adc_uorb_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   unsigned long arg);
static int adc_uorb_calibrate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, unsigned long arg);
static int adc_uorb_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_callback_s g_adc_uorb_callback =
{
  .au_receive = adc_uorb_receive,
};

static const struct sensor_ops_s g_adc_uorb_ops =
{
  .activate       = adc_uorb_activate,
  .set_interval   = adc_uorb_set_interval,
  .set_calibvalue = adc_uorb_set_calibvalue,
  .calibrate      = adc_uorb_calibrate,
  .get_info       = adc_uorb_get_info,
};

/* Bridges, so au_receive() can find its own from the ADC */

static FAR struct adc_uorb_dev_s *g_adc_uorb;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_uorb_singlevalue
 *
 * Description:
 *   Whether a topic type is a timestamp and one sensor_data_t.
 *
 ****************************************************************************/

static bool adc_uorb_singlevalue(int type)
{
  switch (type)
    {
      case SENSOR_TYPE_TEMPERATURE:
      case SENSOR_TYPE_AMBIENT_TEMPERATURE:
      case SENSOR_TYPE_RELATIVE_HUMIDITY:
      case SENSOR_TYPE_VOLTAGE:
      case SENSOR_TYPE_CURRENT:
      case SENSOR_TYPE_POWER:
      case SENSOR_TYPE_RESISTANCE:
        return true;

      default:
        return false;
    }
}

/****************************************************************************
 * Name: adc_uorb_delay
 ****************************************************************************/

static clock_t adc_uorb_delay(FAR struct adc_uorb_dev_s *dev)
{
  clock_t ticks = USEC2TICK(dev->interval);

  return ticks > 0 ? ticks : 1;
}

/****************************************************************************
 * Name: adc_uorb_publish
 *
 * Description:
 *   Work item: convert the last complete scan and push it to every active
 *   topic.
 *
 ****************************************************************************/

static void adc_uorb_publish(FAR void *arg)
{
  FAR struct adc_uorb_dev_s *dev = arg;
  FAR const struct adc_uorb_chan_s *chan;
  FAR struct adc_uorb_sensor_s *sensor;
  struct adc_uorb_value_s value;
  struct adc_uorb_raw_s raw;
  irqstate_t flags;
  int32_t scan[ADC_UORB_MAXCHAN];
  uint64_t stamp;
  float volts;
  float out;
  int i;

  flags = spin_lock_irqsave(&dev->spinlock);
  stamp = dev->stamp;
  memcpy(scan, dev->scan, sizeof(scan));
  spin_unlock_irqrestore(&dev->spinlock, flags);

  for (i = 0; i < dev->nsensors; i++)
    {
      sensor = &dev->sensors[i];
      chan   = sensor->chan;
      if (!sensor->enabled)
        {
          continue;
        }

      if (chan == NULL)
        {
          memset(&raw, 0, sizeof(raw));
          raw.timestamp = stamp;
          for (raw.nchans = 0; raw.nchans < dev->config->nchans;
               raw.nchans++)
            {
              chan = &dev->config->chans[raw.nchans];
              raw.channel[raw.nchans] = chan->channel;
              raw.code[raw.nchans]    = scan[chan->channel];
            }

          sensor->lower.push_event(sensor->lower.priv, &raw, sizeof(raw));
          continue;
        }

      volts = (float)scan[chan->channel] * dev->lsb;
      out   = chan->convert != NULL ? chan->convert(chan, volts) : volts;

      value.timestamp = stamp;
      value.value     = sensor_data_ftof(sensor->gain * out +
                                         sensor->offset);
      sensor->lower.push_event(sensor->lower.priv, &value, sizeof(value));
    }
}

/****************************************************************************
 * Name: adc_uorb_scan
 *
 * Description:
 *   Work item: start a scan, and queue the next one.
 *
 ****************************************************************************/

static void adc_uorb_scan(FAR void *arg)
{
  FAR struct adc_uorb_dev_s *dev = arg;
  FAR struct adc_dev_s *adc = dev->config->adc;
  irqstate_t flags;
  int ret;

  nxmutex_lock(&dev->lock);
  if (!dev->running)
    {
      /* Stopped: shut the ADC down, as the ADC upper half's close() */

      if (dev->setup)
        {
          adc->ad_ops->ao_rxint(adc, false);
          adc->ad_ops->ao_shutdown(adc);
          dev->setup = false;
        }

      nxmutex_unlock(&dev->lock);
      return;
    }

  /* First scan of a run: set the ADC up, as the upper half's open() */

  if (!dev->setup)
    {
      ret = adc->ad_ops->ao_setup(adc);
      if (ret < 0)
        {
          snerr("ERROR: ADC setup failed: %d\n", ret);
        }
      else
        {
          adc->ad_ops->ao_rxint(adc, true);
          dev->setup = true;
        }
    }

  work_queue(LPWORK, &dev->scanwork, adc_uorb_scan, dev,
             adc_uorb_delay(dev));
  nxmutex_unlock(&dev->lock);

  if (!dev->setup)
    {
      return;
    }

  flags = spin_lock_irqsave(&dev->spinlock);
  dev->got = 0;
  spin_unlock_irqrestore(&dev->spinlock, flags);

  ret = adc->ad_ops->ao_ioctl(adc, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      snerr("ERROR: ADC trigger failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: adc_uorb_receive
 *
 * Description:
 *   ADC callback, possibly in interrupt context: collect one code, and
 *   hand a complete scan to the publish worker.
 *
 ****************************************************************************/

static int adc_uorb_receive(FAR struct adc_dev_s *adc, uint8_t ch,
                            int32_t data)
{
  FAR struct adc_uorb_dev_s *dev;
  irqstate_t flags;
  bool complete = false;

  for (dev = g_adc_uorb; dev != NULL && dev->config->adc != adc;
       dev = dev->flink)
    {
    }

  if (dev == NULL || ch >= ADC_UORB_MAXCHAN)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&dev->spinlock);
  dev->codes[ch] = data;
  dev->got |= 1u << ch;
  if ((dev->got & dev->want) == dev->want)
    {
      dev->got   = 0;
      dev->stamp = sensor_get_timestamp();
      memcpy(dev->scan, dev->codes, sizeof(dev->scan));
      complete   = true;
    }

  spin_unlock_irqrestore(&dev->spinlock, flags);

  if (complete)
    {
      work_queue(LPWORK, &dev->pubwork, adc_uorb_publish, dev, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: adc_uorb_update
 *
 * Description:
 *   Recompute the scan interval from the active topics, and start or stop
 *   scanning.  Called with dev->lock held.
 *
 ****************************************************************************/

static void adc_uorb_update(FAR struct adc_uorb_dev_s *dev)
{
  uint32_t interval = UINT32_MAX;
  bool running = false;
  int i;

  for (i = 0; i < dev->nsensors; i++)
    {
      if (dev->sensors[i].enabled)
        {
          interval = MIN(interval, dev->sensors[i].interval);
          running  = true;
        }
    }

  if (running)
    {
      dev->interval = interval;
    }

  if (running && !dev->running)
    {
      dev->running = true;
      work_queue(LPWORK, &dev->scanwork, adc_uorb_scan, dev, 0);
    }
  else if (!running && dev->running)
    {
      /* One last pass of the worker shuts the ADC down */

      dev->running = false;
      work_queue(LPWORK, &dev->scanwork, adc_uorb_scan, dev, 0);
    }
}

/****************************************************************************
 * Name: adc_uorb_activate
 ****************************************************************************/

static int adc_uorb_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct adc_uorb_sensor_s *sensor =
    (FAR struct adc_uorb_sensor_s *)lower;
  FAR struct adc_uorb_dev_s *dev = sensor->dev;

  nxmutex_lock(&dev->lock);
  sensor->enabled = enable;
  adc_uorb_update(dev);
  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Name: adc_uorb_set_interval
 ****************************************************************************/

static int adc_uorb_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct adc_uorb_sensor_s *sensor =
    (FAR struct adc_uorb_sensor_s *)lower;
  FAR struct adc_uorb_dev_s *dev = sensor->dev;

  nxmutex_lock(&dev->lock);
  sensor->interval = MAX(*period_us, CONFIG_SENSORS_ADC_UORB_MIN_INTERVAL);
  *period_us       = sensor->interval;
  adc_uorb_update(dev);
  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Name: adc_uorb_set_calibvalue
 ****************************************************************************/

static int adc_uorb_set_calibvalue(FAR struct sensor_lowerhalf_s *lower,
                                   FAR struct file *filep,
                                   unsigned long arg)
{
  FAR struct adc_uorb_sensor_s *sensor =
    (FAR struct adc_uorb_sensor_s *)lower;
  FAR const struct adc_uorb_calib_s *calib =
    (FAR const struct adc_uorb_calib_s *)(uintptr_t)arg;

  if (sensor->chan == NULL || calib == NULL)
    {
      return -EINVAL;
    }

  sensor->gain   = calib->gain;
  sensor->offset = calib->offset;
  return OK;
}

/****************************************************************************
 * Name: adc_uorb_calibrate
 *
 * Description:
 *   Report the gain and offset in use.
 *
 ****************************************************************************/

static int adc_uorb_calibrate(FAR struct sensor_lowerhalf_s *lower,
                              FAR struct file *filep, unsigned long arg)
{
  FAR struct adc_uorb_sensor_s *sensor =
    (FAR struct adc_uorb_sensor_s *)lower;
  FAR struct adc_uorb_calib_s *calib =
    (FAR struct adc_uorb_calib_s *)(uintptr_t)arg;

  if (sensor->chan == NULL || calib == NULL)
    {
      return -EINVAL;
    }

  calib->gain   = sensor->gain;
  calib->offset = sensor->offset;
  return OK;
}

/****************************************************************************
 * Name: adc_uorb_get_info
 ****************************************************************************/

static int adc_uorb_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info)
{
  FAR struct adc_uorb_sensor_s *sensor =
    (FAR struct adc_uorb_sensor_s *)lower;
  FAR struct adc_uorb_dev_s *dev = sensor->dev;

  memset(info, 0, sizeof(*info));
  info->version   = 1;
  info->min_delay = CONFIG_SENSORS_ADC_UORB_MIN_INTERVAL;
  info->max_delay = UINT32_MAX;
  if (sensor->chan != NULL)
    {
      info->resolution = sensor_data_ftof(sensor->gain * dev->lsb);
      snprintf(info->name, sizeof(info->name), "ADC channel %d",
               sensor->chan->channel);
    }
  else
    {
      strlcpy(info->name, "ADC raw codes", sizeof(info->name));
    }

  strlcpy(info->vendor, "adc_uorb", sizeof(info->vendor));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_uorb_register
 ****************************************************************************/

int adc_uorb_register(FAR const struct adc_uorb_config_s *config)
{
  FAR const struct adc_uorb_chan_s *chan;
  FAR struct adc_uorb_sensor_s *sensor;
  FAR struct adc_uorb_dev_s *dev;
  FAR struct adc_dev_s *adc;
  char path[32];
  int nsensors;
  int ret;
  int i;

  DEBUGASSERT(config != NULL && config->adc != NULL &&
              config->chans != NULL && config->bits > 0 &&
              config->bits < 32);

  adc = config->adc;
  for (i = 0; i < config->nchans; i++)
    {
      chan = &config->chans[i];
      if (chan->channel >= ADC_UORB_MAXCHAN ||
          !adc_uorb_singlevalue(chan->type))
        {
          snerr("ERROR: channel %d: bad channel or type %d\n",
                chan->channel, chan->type);
          return -EINVAL;
        }
    }

  nsensors = config->nchans + (config->rawdevno >= 0 ? 1 : 0);
  if (nsensors == 0)
    {
      return -EINVAL;
    }

  dev = kmm_zalloc(sizeof(*dev) +
                   (nsensors - 1) * sizeof(struct adc_uorb_sensor_s));
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->config   = config;
  dev->lsb      = config->vref / (float)(1u << config->bits);
  dev->nsensors = nsensors;
  dev->interval = ADC_UORB_DEFAULT_INTERVAL;
  nxmutex_init(&dev->lock);
  spin_lock_init(&dev->spinlock);

  for (i = 0; i < nsensors; i++)
    {
      sensor              = &dev->sensors[i];
      sensor->dev         = dev;
      sensor->interval    = ADC_UORB_DEFAULT_INTERVAL;
      sensor->lower.ops   = &g_adc_uorb_ops;
      if (i < config->nchans)
        {
          sensor->chan       = &config->chans[i];
          sensor->gain       = sensor->chan->gain;
          sensor->offset     = sensor->chan->offset;
          sensor->lower.type = sensor->chan->type;
          dev->want         |= 1u << sensor->chan->channel;
        }
      else
        {
          sensor->lower.type = SENSOR_TYPE_CUSTOM;
        }
    }

  /* Take over the lower half.  The scan worker sets it up when a topic is
   * activated and shuts it down when none is.
   */

  ret = adc->ad_ops->ao_bind(adc, &g_adc_uorb_callback);
  if (ret < 0)
    {
      snerr("ERROR: ADC bind failed: %d\n", ret);
      goto errout;
    }

  dev->flink = g_adc_uorb;
  g_adc_uorb = dev;

  for (i = 0; i < nsensors; i++)
    {
      sensor = &dev->sensors[i];
      if (sensor->chan != NULL)
        {
          ret = sensor_register(&sensor->lower, sensor->chan->devno);
        }
      else
        {
          snprintf(path, sizeof(path), ADC_UORB_RAW_FMT, config->rawdevno);
          ret = sensor_custom_register(&sensor->lower, path,
                                       sizeof(struct adc_uorb_raw_s));
        }

      if (ret < 0)
        {
          /* Topics already registered stay; this one is skipped */

          snerr("ERROR: topic %d not registered: %d\n", i, ret);
        }
    }

  return OK;

errout:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return ret;
}

#endif /* CONFIG_SENSORS_ADC_UORB */
