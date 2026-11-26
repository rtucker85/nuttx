/****************************************************************************
 * drivers/leds/tpic2810.c
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

/* TI TPIC2810: eight open-drain outputs behind I2C, as a user LED device.
 * Every change is one TPIC2810_CMD_IMMEDIATE write of all eight outputs.
 * TPIC2810_CMD_READ and a repeated-START read return the output latch,
 * which is what ULEDIOC_GETALL reports with
 * CONFIG_USERLED_LOWER_READSTATE.  A read without the command byte first
 * returns 0xff.
 *
 * The upper half serialises its calls, so the driver needs no lock of its
 * own.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/userled.h>
#include <nuttx/leds/tpic2810.h>

#ifdef CONFIG_TPIC2810

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tpic2810_dev_s
{
  struct userled_lowerhalf_s lower;  /* Must be first */
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  uint8_t outputs;                   /* Last written */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static userled_set_t
tpic2810_supported(FAR const struct userled_lowerhalf_s *lower);
static void tpic2810_setled(FAR const struct userled_lowerhalf_s *lower,
                            int led, bool ledon);
static void tpic2810_setall(FAR const struct userled_lowerhalf_s *lower,
                            userled_set_t ledset);
#ifdef CONFIG_USERLED_LOWER_READSTATE
static void tpic2810_getall(FAR const struct userled_lowerhalf_s *lower,
                            FAR userled_set_t *ledset);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tpic2810_config
 ****************************************************************************/

static void tpic2810_config(FAR struct tpic2810_dev_s *priv,
                            FAR struct i2c_config_s *config)
{
  config->frequency = CONFIG_TPIC2810_I2C_FREQUENCY;
  config->address   = priv->addr;
  config->addrlen   = 7;
}

/****************************************************************************
 * Name: tpic2810_write
 *
 * Description:
 *   Set all eight outputs at once.
 *
 ****************************************************************************/

static int tpic2810_write(FAR struct tpic2810_dev_s *priv, uint8_t outputs)
{
  struct i2c_config_s config;
  uint8_t buffer[2];
  int ret;

  tpic2810_config(priv, &config);
  buffer[0] = TPIC2810_CMD_IMMEDIATE;
  buffer[1] = outputs;

  ret = i2c_write(priv->i2c, &config, buffer, sizeof(buffer));
  if (ret < 0)
    {
      lederr("ERROR: TPIC2810 at 0x%02x: write failed: %d\n",
             priv->addr, ret);
      return ret;
    }

  priv->outputs = outputs;
  return OK;
}

/****************************************************************************
 * Name: tpic2810_supported
 ****************************************************************************/

static userled_set_t
tpic2810_supported(FAR const struct userled_lowerhalf_s *lower)
{
  return (1 << TPIC2810_NOUTPUTS) - 1;
}

/****************************************************************************
 * Name: tpic2810_setled
 ****************************************************************************/

static void tpic2810_setled(FAR const struct userled_lowerhalf_s *lower,
                            int led, bool ledon)
{
  FAR struct tpic2810_dev_s *priv = (FAR struct tpic2810_dev_s *)lower;
  uint8_t outputs;

  if (led < 0 || led >= TPIC2810_NOUTPUTS)
    {
      return;
    }

  outputs = priv->outputs;
  if (ledon)
    {
      outputs |= 1 << led;
    }
  else
    {
      outputs &= ~(1 << led);
    }

  tpic2810_write(priv, outputs);
}

/****************************************************************************
 * Name: tpic2810_setall
 ****************************************************************************/

static void tpic2810_setall(FAR const struct userled_lowerhalf_s *lower,
                            userled_set_t ledset)
{
  tpic2810_write((FAR struct tpic2810_dev_s *)lower, (uint8_t)ledset);
}

/****************************************************************************
 * Name: tpic2810_getall
 *
 * Description:
 *   Read the output latch back.  If the read fails, report what was last
 *   written.
 *
 ****************************************************************************/

#ifdef CONFIG_USERLED_LOWER_READSTATE
static void tpic2810_getall(FAR const struct userled_lowerhalf_s *lower,
                            FAR userled_set_t *ledset)
{
  FAR struct tpic2810_dev_s *priv = (FAR struct tpic2810_dev_s *)lower;
  struct i2c_config_s config;
  uint8_t cmd = TPIC2810_CMD_READ;
  uint8_t outputs;
  int ret;

  tpic2810_config(priv, &config);
  ret = i2c_writeread(priv->i2c, &config, &cmd, 1, &outputs, 1);
  if (ret < 0)
    {
      lederr("ERROR: TPIC2810 at 0x%02x: read failed: %d\n",
             priv->addr, ret);
      outputs = priv->outputs;
    }

  *ledset = outputs;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tpic2810_register
 *
 * Description:
 *   Turn all eight outputs of a TPIC2810 off and register it as a user LED
 *   device.
 *
 ****************************************************************************/

int tpic2810_register(FAR const char *devpath,
                      FAR struct i2c_master_s *i2c, uint8_t addr)
{
  FAR struct tpic2810_dev_s *priv;
  int ret;

  DEBUGASSERT(devpath != NULL && i2c != NULL);

  priv = kmm_zalloc(sizeof(struct tpic2810_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->lower.ll_supported = tpic2810_supported;
  priv->lower.ll_setled    = tpic2810_setled;
  priv->lower.ll_setall    = tpic2810_setall;
#ifdef CONFIG_USERLED_LOWER_READSTATE
  priv->lower.ll_getall    = tpic2810_getall;
#endif
  priv->i2c                = i2c;
  priv->addr               = addr;

  /* All off.  This is also the probe: no ACK, no device. */

  ret = tpic2810_write(priv, 0);
  if (ret < 0)
    {
      kmm_free(priv);
      return ret == -ENXIO ? -ENODEV : ret;
    }

  ret = userled_register(devpath, &priv->lower);
  if (ret < 0)
    {
      lederr("ERROR: userled_register(%s) failed: %d\n", devpath, ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_TPIC2810 */
