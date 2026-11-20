/****************************************************************************
 * drivers/sensors/tmp117_uorb.c
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

/* TI TMP117 digital temperature sensor as the uORB sensor_temp topic.
 *
 * The part converts continuously with 8-sample averaging; the conversion
 * cycle follows the requested interval (125 ms to 16 s).  A low-priority
 * work item reads the last completed result at that interval.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/clock.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/tmp117.h>
#include <nuttx/signal.h>
#include <nuttx/wqueue.h>

#ifdef CONFIG_SENSORS_TMP117

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers, 16 bits, MSB first */

#define TMP117_REG_TEMP         0x00
#define TMP117_REG_CONFIG       0x01
#define TMP117_REG_DEVICE_ID    0x0f

/* Configuration */

#define CONFIG_SOFT_RESET       (1 << 1)
#define CONFIG_AVG_8            (1 << 5)
#define CONFIG_CONV_SHIFT       7
#define CONFIG_MOD_CONTINUOUS   (0 << 10)

#define TMP117_DEVICE_ID        0x0117
#define TMP117_DEVICE_ID_MASK   0x0fff

/* 7.8125 m degC per LSB: raw / 128 */

#define TMP117_LSB_DEN          128

/* Soft reset takes 2 ms */

#define TMP117_RESET_MS         2

#define TMP117_DEFAULT_INTERVAL 1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tmp117_dev_s
{
  struct sensor_lowerhalf_s lower;    /* Must be first */
  FAR struct i2c_master_s  *i2c;
  uint8_t                   addr;
  uint32_t                  interval; /* Microseconds between readings */
  struct work_s             work;
  bool                      enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tmp117_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int tmp117_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);
static int tmp117_get_info(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR struct sensor_device_info_s *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_tmp117_ops =
{
  .activate     = tmp117_activate,
  .set_interval = tmp117_set_interval,
  .get_info     = tmp117_get_info,
};

/* Conversion cycle per CONFIG.CONV code with 8-sample averaging, in
 * microseconds (codes 0-1 are both limited to 125 ms by the averaging).
 */

static const uint32_t g_tmp117_cycle[] =
{
  125000, 125000, 250000, 500000, 1000000, 4000000, 8000000, 16000000
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp117_getreg / tmp117_putreg
 ****************************************************************************/

static int tmp117_getreg(FAR struct tmp117_dev_s *priv, uint8_t reg,
                         FAR uint16_t *value)
{
  struct i2c_msg_s msg[2];
  uint8_t buffer[2];
  int ret;

  msg[0].frequency = CONFIG_TMP117_I2C_FREQUENCY;
  msg[0].addr      = priv->addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  msg[1].frequency = CONFIG_TMP117_I2C_FREQUENCY;
  msg[1].addr      = priv->addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = buffer;
  msg[1].length    = 2;

  ret = I2C_TRANSFER(priv->i2c, msg, 2);
  if (ret < 0)
    {
      return ret;
    }

  *value = ((uint16_t)buffer[0] << 8) | buffer[1];
  return OK;
}

static int tmp117_putreg(FAR struct tmp117_dev_s *priv, uint8_t reg,
                         uint16_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[3];

  buffer[0] = reg;
  buffer[1] = value >> 8;
  buffer[2] = value & 0xff;

  msg.frequency = CONFIG_TMP117_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 3;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

/****************************************************************************
 * Name: tmp117_configure
 *
 * Description:
 *   Continuous conversion, 8-sample averaging, and the longest conversion
 *   cycle no longer than the interval.
 *
 ****************************************************************************/

static int tmp117_configure(FAR struct tmp117_dev_s *priv)
{
  int conv;

  for (conv = nitems(g_tmp117_cycle) - 1; conv > 0; conv--)
    {
      if (g_tmp117_cycle[conv] <= priv->interval)
        {
          break;
        }
    }

  return tmp117_putreg(priv, TMP117_REG_CONFIG,
                       CONFIG_MOD_CONTINUOUS | CONFIG_AVG_8 |
                       (conv << CONFIG_CONV_SHIFT));
}

/****************************************************************************
 * Name: tmp117_delay
 ****************************************************************************/

static clock_t tmp117_delay(FAR struct tmp117_dev_s *priv)
{
  clock_t ticks = USEC2TICK(priv->interval);

  return ticks > 0 ? ticks : 1;
}

/****************************************************************************
 * Name: tmp117_worker
 ****************************************************************************/

static void tmp117_worker(FAR void *arg)
{
  FAR struct tmp117_dev_s *priv = arg;
  struct sensor_temp temp;
  uint16_t raw;
  int ret;

  /* Queue the next reading first, so a failed transfer costs one sample */

  work_queue(LPWORK, &priv->work, tmp117_worker, priv, tmp117_delay(priv));

  ret = tmp117_getreg(priv, TMP117_REG_TEMP, &raw);
  if (ret < 0)
    {
      snerr("ERROR: TMP117 read failed: %d\n", ret);
      return;
    }

  temp.timestamp   = sensor_get_timestamp();
  temp.temperature = sensor_data_divi(sensor_data_itof((int16_t)raw),
                                      TMP117_LSB_DEN);
  priv->lower.push_event(priv->lower.priv, &temp, sizeof(temp));
}

/****************************************************************************
 * Name: tmp117_activate
 ****************************************************************************/

static int tmp117_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct tmp117_dev_s *priv = (FAR struct tmp117_dev_s *)lower;

  if (enable == priv->enabled)
    {
      return OK;
    }

  if (enable)
    {
      work_queue(LPWORK, &priv->work, tmp117_worker, priv,
                 tmp117_delay(priv));
    }
  else
    {
      work_cancel(LPWORK, &priv->work);
    }

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: tmp117_set_interval
 *
 * Description:
 *   The interval is held to the shortest conversion cycle, 125 ms.
 *
 ****************************************************************************/

static int tmp117_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct tmp117_dev_s *priv = (FAR struct tmp117_dev_s *)lower;
  int ret;

  priv->interval = MAX(*period_us, g_tmp117_cycle[0]);
  *period_us     = priv->interval;
  ret            = tmp117_configure(priv);

  /* Activation comes before the interval when a topic is opened, so a
   * reading already queued at the old interval is queued again at the new
   * one: otherwise a change from 16 s to 125 ms waits out the 16 s.
   */

  if (priv->enabled)
    {
      work_queue(LPWORK, &priv->work, tmp117_worker, priv,
                 tmp117_delay(priv));
    }

  return ret;
}

/****************************************************************************
 * Name: tmp117_get_info
 ****************************************************************************/

static int tmp117_get_info(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR struct sensor_device_info_s *info)
{
  memset(info, 0, sizeof(*info));
  info->version    = 1;
  info->max_range  = sensor_data_ftof(150.0f);       /* -55 to +150 degC */
  info->resolution = sensor_data_ftof(0.0078125f);
  info->min_delay  = g_tmp117_cycle[0];
  info->max_delay  = g_tmp117_cycle[nitems(g_tmp117_cycle) - 1];
  strlcpy(info->name, "TMP117", sizeof(info->name));
  strlcpy(info->vendor, "Texas Instruments", sizeof(info->vendor));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tmp117_register_uorb
 ****************************************************************************/

int tmp117_register_uorb(int devno, FAR struct i2c_master_s *i2c,
                         uint8_t addr)
{
  FAR struct tmp117_dev_s *priv;
  uint16_t id;
  int ret;

  DEBUGASSERT(i2c != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c        = i2c;
  priv->addr       = addr;
  priv->interval   = TMP117_DEFAULT_INTERVAL;
  priv->lower.ops  = &g_tmp117_ops;
  priv->lower.type = SENSOR_TYPE_TEMPERATURE;

  ret = tmp117_getreg(priv, TMP117_REG_DEVICE_ID, &id);
  if (ret < 0 || (id & TMP117_DEVICE_ID_MASK) != TMP117_DEVICE_ID)
    {
      snerr("ERROR: no TMP117 at %02x (%d, ID %04x)\n", addr, ret,
            ret < 0 ? 0 : id);
      ret = -ENODEV;
      goto errout;
    }

  ret = tmp117_putreg(priv, TMP117_REG_CONFIG, CONFIG_SOFT_RESET);
  if (ret >= 0)
    {
      nxsched_usleep(TMP117_RESET_MS * 1000);
      ret = tmp117_configure(priv);
    }

  if (ret < 0)
    {
      snerr("ERROR: TMP117 configuration failed: %d\n", ret);
      goto errout;
    }

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: sensor_register failed: %d\n", ret);
      goto errout;
    }

  return OK;

errout:
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_SENSORS_TMP117 */
