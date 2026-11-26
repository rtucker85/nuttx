/****************************************************************************
 * drivers/sensors/lsm6dsv_uorb.c
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

/* STMicroelectronics LSM6DSV 6-axis IMU on SPI, as uORB sensor_accel and
 * sensor_gyro topics.  On the LSM6DSV320X the high-g accelerometer is a
 * second sensor_accel topic, and the SFLP sensor fusion's game rotation
 * vector a sensor_rotation topic.
 *
 * All sensors run at their requested rates and are batched into the
 * device FIFO.  A kernel thread drains the FIFO every
 * CONFIG_SENSORS_LSM6DSV_POLL_INTERVAL microseconds and publishes every
 * sample, so the publish rate is not limited by the system tick.  Samples
 * carry the time the FIFO was read, less one sample period for each newer
 * sample of the same sensor.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/sensors/lsm6dsv.h>
#include <nuttx/sensors/sensor.h>

#ifdef CONFIG_SENSORS_LSM6DSV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Registers */

#define LSM6DSV_FUNC_CFG_ACCESS 0x01
#define LSM6DSV_IF_CFG          0x03
#define LSM6DSV_FIFO_CTRL3      0x09
#define LSM6DSV_FIFO_CTRL4      0x0a
#define LSM6DSV_COUNTER_BDR1    0x0b
#define LSM6DSV_WHO_AM_I        0x0f
#define LSM6DSV_CTRL1           0x10  /* Accelerometer mode and ODR */
#define LSM6DSV_CTRL2           0x11  /* Gyroscope mode and ODR */
#define LSM6DSV_CTRL3           0x12
#define LSM6DSV_CTRL6           0x15  /* Gyroscope full scale */
#define LSM6DSV_CTRL8           0x17  /* Accelerometer full scale */
#define LSM6DSV_FIFO_STATUS1    0x1b
#define LSM6DSV_OUT_TEMP_L      0x20
#define LSM6DSV_CTRL1_XL_HG     0x4e  /* LSM6DSV320X high-g FS and ODR */

/* Embedded functions bank (FUNC_CFG_ACCESS.EMB_FUNC_REG_ACCESS) */

#define LSM6DSV_EMB_FUNC_EN_A   0x04
#define LSM6DSV_EMB_FIFO_EN_A   0x44
#define LSM6DSV_SFLP_ODR        0x5e
#define LSM6DSV_EMB_FUNC_INIT_A 0x66
#define LSM6DSV_FIFO_DATA_TAG   0x78

#define LSM6DSV_ID              0x70
#define LSM6DSV320X_ID          0x73  /* Same core, plus a high-g accel */

/* FUNC_CFG_ACCESS */

#define FUNC_CFG_EMB_FUNC_REG_ACCESS (1 << 7)

/* EMB_FUNC_EN_A, EMB_FUNC_FIFO_EN_A and EMB_FUNC_INIT_A: the SFLP game
 * rotation vector's enable, FIFO batching and reset
 */

#define EMB_SFLP_GAME           (1 << 1)

/* SFLP_ODR.SFLP_GAME_ODR, bits 5:3: 15 Hz doubling up to 480 Hz */

#define SFLP_ODR_SHIFT          3
#define SFLP_ODR_MASK           (7 << SFLP_ODR_SHIFT)

/* IF_CFG */

#define IF_CFG_I2C_I3C_DISABLE  (1 << 0)

/* CTRL3 */

#define CTRL3_SW_RESET          (1 << 0)
#define CTRL3_IF_INC            (1 << 2)
#define CTRL3_BDU               (1 << 6)

/* COUNTER_BDR_REG1 */

#define COUNTER_BDR1_XL_HG_BATCH_EN (1 << 3)

/* CTRL1_XL_HG: FS_XL_HG in bits 2:0, ODR_XL_HG in bits 5:3 */

#define CTRL1_XL_HG_ODR_SHIFT   3

/* FIFO_CTRL4.FIFO_MODE */

#define FIFO_MODE_BYPASS        0x00
#define FIFO_MODE_STREAM        0x06

/* FIFO_STATUS2, the byte after FIFO_STATUS1 */

#define FIFO_STATUS2_DIFF8      (1 << 0)  /* Bit 8 of the unread count */
#define FIFO_STATUS2_OVR_IA     (1 << 6)

/* FIFO_DATA_OUT_TAG.TAG_SENSOR (bits 7:3) */

#define FIFO_TAG_SHIFT          3
#define FIFO_TAG_GY             0x01
#define FIFO_TAG_XL             0x02
#define FIFO_TAG_XL_HG          0x1d
#define FIFO_TAG_SFLP_GAME      0x13  /* Two words: X, Y then Z, W */

/* Output data rates, CTRL1/CTRL2.ODR and FIFO_CTRL3.BDR codes 0x2..0xc:
 * 7.5 Hz doubling up to 7.68 kHz.
 */

#define LSM6DSV_ODR_FIRST       0x02
#define LSM6DSV_ODR_LAST        0x0c
#define LSM6DSV_ODR_PERIOD(odr) (8000000 / (60 << ((odr) - LSM6DSV_ODR_FIRST)))

/* The high-g accelerometer has 480 Hz to 7.68 kHz, ODR_XL_HG codes 3..7.
 * Its odr is kept as the matching CTRL1 code, 0x8..0xc.
 */

#define LSM6DSV_HG_ODR_FIRST    0x08
#define LSM6DSV_HG_ODR_OFFSET   5      /* CTRL1 code - ODR_XL_HG code */

/* SFLP runs at 15 Hz to 480 Hz, SFLP_GAME_ODR codes 0..5, kept as the
 * matching CTRL1 code, 0x3..0x8.
 */

#define LSM6DSV_SFLP_ODR_FIRST  0x03
#define LSM6DSV_SFLP_ODR_LAST   0x08

/* SPI: read is address | 0x80; mode 3 */

#define LSM6DSV_READ            0x80
#define LSM6DSV_SPI_MODE        SPIDEV_MODE3

/* FIFO words: tag byte and three 16-bit values.  The unread count is 9
 * bits.
 */

#define LSM6DSV_WORD_SIZE       7
#define LSM6DSV_FIFO_WORDS      512

#define LSM6DSV_ONE_G           9.80665f

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum lsm6dsv_idx_e
{
  LSM6DSV_ACCEL_IDX = 0,
  LSM6DSV_GYRO_IDX,
  LSM6DSV_ACCEL_HG_IDX,            /* LSM6DSV320X only */
  LSM6DSV_ROT_IDX,                 /* LSM6DSV320X only: SFLP */
  LSM6DSV_MAX_IDX
};

struct lsm6dsv_dev_s;

struct lsm6dsv_sensor_s
{
  struct sensor_lowerhalf_s lower;     /* Must be first */
  FAR struct lsm6dsv_dev_s *dev;
  sensor_data_t             scale;     /* SI units per LSB */
  sensor_data_t             range;     /* Full scale in SI units */
  uint8_t                   odr;       /* ODR code while enabled */
  uint8_t                   fs;        /* Full scale code */
  bool                      enabled;
  uint64_t                  last;      /* Last published timestamp */
};

struct lsm6dsv_dev_s
{
  struct lsm6dsv_sensor_s   sensor[LSM6DSV_MAX_IDX];
  FAR struct spi_dev_s     *spi;
  uint32_t                  devid;
  mutex_t                   lock;      /* Settings and enable state */
  sem_t                     run;       /* Wakes the thread */
  bool                      overrun;   /* FIFO overrun already reported */
  uint8_t                   id;        /* WHO_AM_I */
  int                       hgdevno;   /* High-g topic, or -1 if none */
  int                       rotdevno;  /* SFLP topic, or -1 if none */
  bool                      sflp;      /* SFLP running */
  bool                      half;      /* quat[0..1] hold a first word */
  float                     quat[2];   /* SFLP X and Y awaiting Z and W */
  uint8_t                   words[LSM6DSV_FIFO_WORDS][LSM6DSV_WORD_SIZE];
};

/* One full-scale setting */

struct lsm6dsv_fs_s
{
  uint16_t range;  /* g or dps */
  uint8_t  code;
  float    lsb;    /* mg or mdps per LSB */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6dsv_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable);
static int lsm6dsv_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us);
static int lsm6dsv_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info);
static int lsm6dsv_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_lsm6dsv_ops =
{
  .activate     = lsm6dsv_activate,
  .set_interval = lsm6dsv_set_interval,
  .get_info     = lsm6dsv_get_info,
  .control      = lsm6dsv_control,
};

/* Full scales and sensitivities (datasheet DS13510, and ST's
 * lsm6dsv_from_fs*() conversions).
 */

static const struct lsm6dsv_fs_s g_accel_fs[] =
{
  {    2, 0x00,   0.061f },
  {    4, 0x01,   0.122f },
  {    8, 0x02,   0.244f },
  {   16, 0x03,   0.488f },
};

static const struct lsm6dsv_fs_s g_gyro_fs[] =
{
  {  125, 0x00,   4.375f },
  {  250, 0x01,   8.750f },
  {  500, 0x02,  17.500f },
  { 1000, 0x03,  35.000f },
  { 2000, 0x04,  70.000f },
  { 4000, 0x0c, 140.000f },
};

/* LSM6DSV320X high-g accelerometer (ST's lsm6dsv320x_from_fs*_to_mg()) */

static const struct lsm6dsv_fs_s g_accel_hg_fs[] =
{
  {   32, 0x00,   0.976f },
  {   64, 0x01,   1.952f },
  {  128, 0x02,   3.904f },
  {  256, 0x03,   7.808f },
  {  320, 0x04,  10.417f },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsv_configspi
 ****************************************************************************/

static void lsm6dsv_configspi(FAR struct lsm6dsv_dev_s *priv)
{
  SPI_SETMODE(priv->spi, LSM6DSV_SPI_MODE);
  SPI_SETBITS(priv->spi, 8);
  SPI_SETFREQUENCY(priv->spi, CONFIG_SENSORS_LSM6DSV_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: lsm6dsv_readregs
 ****************************************************************************/

static void lsm6dsv_readregs(FAR struct lsm6dsv_dev_s *priv, uint8_t reg,
                             FAR uint8_t *buf, size_t len)
{
  SPI_LOCK(priv->spi, true);
  lsm6dsv_configspi(priv);
  SPI_SELECT(priv->spi, priv->devid, true);
  SPI_SEND(priv->spi, reg | LSM6DSV_READ);
  SPI_RECVBLOCK(priv->spi, buf, len);
  SPI_SELECT(priv->spi, priv->devid, false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: lsm6dsv_getreg
 ****************************************************************************/

static uint8_t lsm6dsv_getreg(FAR struct lsm6dsv_dev_s *priv, uint8_t reg)
{
  uint8_t val;

  lsm6dsv_readregs(priv, reg, &val, 1);
  return val;
}

/****************************************************************************
 * Name: lsm6dsv_putreg
 ****************************************************************************/

static void lsm6dsv_putreg(FAR struct lsm6dsv_dev_s *priv, uint8_t reg,
                           uint8_t val)
{
  SPI_LOCK(priv->spi, true);
  lsm6dsv_configspi(priv);
  SPI_SELECT(priv->spi, priv->devid, true);
  SPI_SEND(priv->spi, reg);
  SPI_SEND(priv->spi, val);
  SPI_SELECT(priv->spi, priv->devid, false);
  SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: lsm6dsv_odr_range
 *
 * Description:
 *   The sensor's slowest and fastest ODR codes.
 *
 ****************************************************************************/

static void lsm6dsv_odr_range(FAR struct lsm6dsv_sensor_s *sensor,
                              FAR uint8_t *first, FAR uint8_t *last)
{
  FAR struct lsm6dsv_dev_s *priv = sensor->dev;

  *first = LSM6DSV_ODR_FIRST;
  *last  = LSM6DSV_ODR_LAST;

  if (sensor == &priv->sensor[LSM6DSV_ACCEL_HG_IDX])
    {
      *first = LSM6DSV_HG_ODR_FIRST;
    }
  else if (sensor == &priv->sensor[LSM6DSV_ROT_IDX])
    {
      *first = LSM6DSV_SFLP_ODR_FIRST;
      *last  = LSM6DSV_SFLP_ODR_LAST;
    }
}

/****************************************************************************
 * Name: lsm6dsv_odr
 *
 * Description:
 *   The slowest ODR code the sensor has whose period is no longer than
 *   period_us.
 *
 ****************************************************************************/

static uint8_t lsm6dsv_odr(FAR struct lsm6dsv_sensor_s *sensor,
                           uint32_t period_us)
{
  uint8_t first;
  uint8_t last;
  uint8_t odr;

  lsm6dsv_odr_range(sensor, &first, &last);
  for (odr = first; odr < last; odr++)
    {
      if (LSM6DSV_ODR_PERIOD(odr) <= period_us)
        {
          break;
        }
    }

  return odr;
}

/****************************************************************************
 * Name: lsm6dsv_running
 *
 * Description:
 *   True if any sensor is enabled.
 *
 ****************************************************************************/

static bool lsm6dsv_running(FAR struct lsm6dsv_dev_s *priv)
{
  int i;

  for (i = 0; i < LSM6DSV_MAX_IDX; i++)
    {
      if (priv->sensor[i].enabled)
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: lsm6dsv_put_hg
 *
 * Description:
 *   Write the high-g accelerometer's full scale and ODR, or off.  It is
 *   batched into the FIFO at its ODR, and its output registers are left
 *   off.
 *
 ****************************************************************************/

static void lsm6dsv_put_hg(FAR struct lsm6dsv_dev_s *priv)
{
  FAR struct lsm6dsv_sensor_s *hg = &priv->sensor[LSM6DSV_ACCEL_HG_IDX];
  uint8_t odr = hg->enabled ? hg->odr - LSM6DSV_HG_ODR_OFFSET : 0;

  lsm6dsv_putreg(priv, LSM6DSV_CTRL1_XL_HG,
                 (odr << CTRL1_XL_HG_ODR_SHIFT) | hg->fs);
}

/****************************************************************************
 * Name: lsm6dsv_put_sflp
 *
 * Description:
 *   Start, retune or stop the SFLP game rotation vector and its FIFO
 *   batching.  It is reset when it starts.  The embedded functions bank is
 *   switched in for this, so the caller holds priv->lock, which the FIFO
 *   reader also takes.
 *
 ****************************************************************************/

static void lsm6dsv_put_sflp(FAR struct lsm6dsv_dev_s *priv)
{
  FAR struct lsm6dsv_sensor_s *rot = &priv->sensor[LSM6DSV_ROT_IDX];
  uint8_t mask = rot->enabled ? EMB_SFLP_GAME : 0;
  uint8_t val;

  lsm6dsv_putreg(priv, LSM6DSV_FUNC_CFG_ACCESS,
                 FUNC_CFG_EMB_FUNC_REG_ACCESS);

  if (rot->enabled)
    {
      val  = lsm6dsv_getreg(priv, LSM6DSV_SFLP_ODR) & ~SFLP_ODR_MASK;
      val |= (rot->odr - LSM6DSV_SFLP_ODR_FIRST) << SFLP_ODR_SHIFT;
      lsm6dsv_putreg(priv, LSM6DSV_SFLP_ODR, val);
    }

  val = lsm6dsv_getreg(priv, LSM6DSV_EMB_FUNC_EN_A) & ~EMB_SFLP_GAME;
  lsm6dsv_putreg(priv, LSM6DSV_EMB_FUNC_EN_A, val | mask);
  val = lsm6dsv_getreg(priv, LSM6DSV_EMB_FIFO_EN_A) & ~EMB_SFLP_GAME;
  lsm6dsv_putreg(priv, LSM6DSV_EMB_FIFO_EN_A, val | mask);

  if (rot->enabled && !priv->sflp)
    {
      lsm6dsv_putreg(priv, LSM6DSV_EMB_FUNC_INIT_A, EMB_SFLP_GAME);
      priv->half = false;
    }

  lsm6dsv_putreg(priv, LSM6DSV_FUNC_CFG_ACCESS, 0);
  priv->sflp = rot->enabled;
}

/****************************************************************************
 * Name: lsm6dsv_apply
 *
 * Description:
 *   Write the ODR and FIFO batching of every sensor, and switch the FIFO
 *   to stream mode while any runs, or bypass (empty) when none does.
 *   SFLP needs the accelerometer and gyroscope running at least at its
 *   rate, whether or not their topics are.  Called with priv->lock held.
 *
 ****************************************************************************/

static void lsm6dsv_apply(FAR struct lsm6dsv_dev_s *priv)
{
  FAR struct lsm6dsv_sensor_s *accel = &priv->sensor[LSM6DSV_ACCEL_IDX];
  FAR struct lsm6dsv_sensor_s *gyro = &priv->sensor[LSM6DSV_GYRO_IDX];
  FAR struct lsm6dsv_sensor_s *hg = &priv->sensor[LSM6DSV_ACCEL_HG_IDX];
  FAR struct lsm6dsv_sensor_s *rot = &priv->sensor[LSM6DSV_ROT_IDX];
  uint8_t xl = accel->enabled ? accel->odr : 0;
  uint8_t gy = gyro->enabled ? gyro->odr : 0;
  uint8_t sflp = rot->enabled ? rot->odr : 0;

  /* High-performance mode (OP_MODE 0) at the ODR, or off.  The FIFO
   * batches each at its topic's rate.
   */

  lsm6dsv_putreg(priv, LSM6DSV_CTRL1, MAX(xl, sflp));
  lsm6dsv_putreg(priv, LSM6DSV_CTRL2, MAX(gy, sflp));
  lsm6dsv_putreg(priv, LSM6DSV_FIFO_CTRL3, (gy << 4) | xl);

  if (priv->rotdevno >= 0 && (rot->enabled || priv->sflp))
    {
      lsm6dsv_put_sflp(priv);
    }

  if (priv->hgdevno >= 0)
    {
      lsm6dsv_put_hg(priv);
      lsm6dsv_putreg(priv, LSM6DSV_COUNTER_BDR1,
                     hg->enabled ? COUNTER_BDR1_XL_HG_BATCH_EN : 0);
    }

  lsm6dsv_putreg(priv, LSM6DSV_FIFO_CTRL4, lsm6dsv_running(priv) ?
                 FIFO_MODE_STREAM : FIFO_MODE_BYPASS);
}

/****************************************************************************
 * Name: lsm6dsv_setfs
 *
 * Description:
 *   Select the smallest full scale that covers range (g or dps).
 *
 ****************************************************************************/

static int lsm6dsv_setfs(FAR struct lsm6dsv_sensor_s *sensor,
                         unsigned long range)
{
  FAR struct lsm6dsv_dev_s *priv = sensor->dev;
  FAR const struct lsm6dsv_fs_s *table;
  size_t count;
  size_t i;
  float unit;

  if (sensor == &priv->sensor[LSM6DSV_ACCEL_HG_IDX])
    {
      table = g_accel_hg_fs;
      count = nitems(g_accel_hg_fs);
      unit  = LSM6DSV_ONE_G;
    }
  else if (sensor->lower.type == SENSOR_TYPE_ACCELEROMETER)
    {
      table = g_accel_fs;
      count = nitems(g_accel_fs);
      unit  = LSM6DSV_ONE_G;            /* g to m/s^2 */
    }
  else
    {
      table = g_gyro_fs;
      count = nitems(g_gyro_fs);

      /* The LSM6DSV320X's FS_G is three bits: no 4000 dps setting */

      if (priv->id == LSM6DSV320X_ID)
        {
          count--;
        }

      unit  = (float)M_PI / 180.0f;     /* dps to rad/s */
    }

  for (i = 0; i < count - 1; i++)
    {
      if (table[i].range >= range)
        {
          break;
        }
    }

  sensor->fs    = table[i].code;
  sensor->scale = sensor_data_ftof(table[i].lsb * unit / 1000.0f);
  sensor->range = sensor_data_ftof(table[i].range * unit);

  if (sensor == &priv->sensor[LSM6DSV_ACCEL_HG_IDX])
    {
      lsm6dsv_put_hg(priv);
    }
  else if (sensor->lower.type == SENSOR_TYPE_ACCELEROMETER)
    {
      lsm6dsv_putreg(priv, LSM6DSV_CTRL8, sensor->fs);
    }
  else
    {
      lsm6dsv_putreg(priv, LSM6DSV_CTRL6, sensor->fs);
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dsv_activate
 ****************************************************************************/

static int lsm6dsv_activate(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, bool enable)
{
  FAR struct lsm6dsv_sensor_s *sensor = (FAR struct lsm6dsv_sensor_s *)lower;
  FAR struct lsm6dsv_dev_s *priv = sensor->dev;
  bool wake;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  wake = enable && !lsm6dsv_running(priv);

  sensor->enabled = enable;
  lsm6dsv_apply(priv);
  nxmutex_unlock(&priv->lock);

  if (wake)
    {
      nxsem_post(&priv->run);
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dsv_set_interval
 ****************************************************************************/

static int lsm6dsv_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *period_us)
{
  FAR struct lsm6dsv_sensor_s *sensor = (FAR struct lsm6dsv_sensor_s *)lower;
  FAR struct lsm6dsv_dev_s *priv = sensor->dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  sensor->odr = lsm6dsv_odr(sensor, *period_us);
  *period_us  = LSM6DSV_ODR_PERIOD(sensor->odr);

  if (sensor->enabled)
    {
      lsm6dsv_apply(priv);
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: lsm6dsv_get_info
 ****************************************************************************/

static int lsm6dsv_get_info(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep,
                            FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6dsv_sensor_s *sensor = (FAR struct lsm6dsv_sensor_s *)lower;
  uint8_t first;
  uint8_t last;

  memset(info, 0, sizeof(*info));
  info->version    = 1;
  info->power      = sensor_data_ftof(0.65f); /* mA, both in HP mode */
  info->max_range  = sensor->range;
  info->resolution = sensor->scale;
  lsm6dsv_odr_range(sensor, &first, &last);
  info->min_delay  = LSM6DSV_ODR_PERIOD(last);
  info->max_delay  = LSM6DSV_ODR_PERIOD(first);
  strlcpy(info->name, sensor->dev->id == LSM6DSV320X_ID ?
          "LSM6DSV320X" : "LSM6DSV", sizeof(info->name));
  strlcpy(info->vendor, "STMicroelectronics", sizeof(info->vendor));
  return OK;
}

/****************************************************************************
 * Name: lsm6dsv_control
 *
 * Description:
 *   SNIOC_SET_SCALE_XL: full scale, in g for the accelerometers and in dps
 *   for the gyroscope.  The smallest scale that covers it is used.
 *
 ****************************************************************************/

static int lsm6dsv_control(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct lsm6dsv_sensor_s *sensor = (FAR struct lsm6dsv_sensor_s *)lower;
  FAR struct lsm6dsv_dev_s *priv = sensor->dev;
  int ret;

  switch (cmd)
    {
      case SNIOC_SET_SCALE_XL:
        if (sensor == &priv->sensor[LSM6DSV_ROT_IDX])
          {
            return -ENOTTY;
          }

        ret = nxmutex_lock(&priv->lock);
        if (ret >= 0)
          {
            ret = lsm6dsv_setfs(sensor, arg);
            nxmutex_unlock(&priv->lock);
          }

        return ret;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: lsm6dsv_stamp
 *
 * Description:
 *   The timestamp of a sample with newer samples of the same sensor after
 *   it in the FIFO, read at now.
 *
 ****************************************************************************/

static uint64_t lsm6dsv_stamp(FAR struct lsm6dsv_sensor_s *sensor,
                              uint64_t now, unsigned int newer)
{
  uint64_t timestamp;

  /* Never earlier than the last one published, if the ODR runs fast */

  timestamp = now - (uint64_t)newer * LSM6DSV_ODR_PERIOD(sensor->odr);
  if (timestamp <= sensor->last)
    {
      timestamp = sensor->last + 1;
    }

  sensor->last = timestamp;
  return timestamp;
}

/****************************************************************************
 * Name: lsm6dsv_half
 *
 * Description:
 *   An IEEE 754 half-precision value, little-endian at p, as a float.
 *
 ****************************************************************************/

static float lsm6dsv_half(FAR const uint8_t *p)
{
  uint16_t h = p[0] | (p[1] << 8);
  int exp = (h >> 10) & 0x1f;
  float mant = (float)(h & 0x3ff);
  float val;

  if (exp == 0)
    {
      val = ldexpf(mant, -24);                   /* Subnormal */
    }
  else if (exp == 0x1f)
    {
      val = mant == 0.0f ? INFINITY : NAN;
    }
  else
    {
      val = ldexpf(mant + 1024.0f, exp - 25);
    }

  return (h & 0x8000) != 0 ? -val : val;
}

/****************************************************************************
 * Name: lsm6dsv_push_rot
 *
 * Description:
 *   Take one SFLP game rotation word: the first holds X and Y, the second
 *   Z and W, which completes a quaternion and publishes it.  A second word
 *   without a first, after an overrun, is dropped.
 *
 ****************************************************************************/

static void lsm6dsv_push_rot(FAR struct lsm6dsv_dev_s *priv,
                             FAR const uint8_t *word, uint64_t now,
                             unsigned int newer)
{
  FAR struct lsm6dsv_sensor_s *sensor = &priv->sensor[LSM6DSV_ROT_IDX];
  FAR struct sensor_lowerhalf_s *lower = &sensor->lower;
  struct sensor_rotation rot;

  if (word[1] == 0)
    {
      priv->quat[0] = lsm6dsv_half(&word[3]);
      priv->quat[1] = lsm6dsv_half(&word[5]);
      priv->half    = true;
      return;
    }

  if (!priv->half)
    {
      return;
    }

  priv->half    = false;
  rot.timestamp = lsm6dsv_stamp(sensor, now, newer);
  rot.x         = sensor_data_ftof(priv->quat[0]);
  rot.y         = sensor_data_ftof(priv->quat[1]);
  rot.z         = sensor_data_ftof(lsm6dsv_half(&word[3]));
  rot.w         = sensor_data_ftof(lsm6dsv_half(&word[5]));
  rot.status    = sensor_data_itof(-1);         /* No heading accuracy */
  lower->push_event(lower->priv, &rot, sizeof(rot));
}

/****************************************************************************
 * Name: lsm6dsv_push
 *
 * Description:
 *   Publish one sample.  newer is the number of samples of the same sensor
 *   that came after it in the FIFO.
 *
 ****************************************************************************/

static void lsm6dsv_push(FAR struct lsm6dsv_sensor_s *sensor,
                         FAR const uint8_t *word, uint64_t now,
                         unsigned int newer, sensor_data_t temp)
{
  FAR struct sensor_lowerhalf_s *lower = &sensor->lower;
  uint64_t timestamp;
  int16_t x;
  int16_t y;
  int16_t z;

  x = (int16_t)(word[1] | (word[2] << 8));
  y = (int16_t)(word[3] | (word[4] << 8));
  z = (int16_t)(word[5] | (word[6] << 8));

  timestamp = lsm6dsv_stamp(sensor, now, newer);

  if (lower->type == SENSOR_TYPE_ACCELEROMETER)
    {
      struct sensor_accel accel;

      accel.timestamp   = timestamp;
      accel.x           = sensor_data_muli(sensor->scale, x);
      accel.y           = sensor_data_muli(sensor->scale, y);
      accel.z           = sensor_data_muli(sensor->scale, z);
      accel.temperature = temp;
      lower->push_event(lower->priv, &accel, sizeof(accel));
    }
  else
    {
      struct sensor_gyro gyro;

      gyro.timestamp   = timestamp;
      gyro.x           = sensor_data_muli(sensor->scale, x);
      gyro.y           = sensor_data_muli(sensor->scale, y);
      gyro.z           = sensor_data_muli(sensor->scale, z);
      gyro.temperature = temp;
      lower->push_event(lower->priv, &gyro, sizeof(gyro));
    }
}

/****************************************************************************
 * Name: lsm6dsv_drain
 *
 * Description:
 *   Read everything in the FIFO and publish it.
 *
 ****************************************************************************/

static void lsm6dsv_drain(FAR struct lsm6dsv_dev_s *priv)
{
  FAR struct lsm6dsv_sensor_s *accel = &priv->sensor[LSM6DSV_ACCEL_IDX];
  FAR struct lsm6dsv_sensor_s *gyro = &priv->sensor[LSM6DSV_GYRO_IDX];
  FAR struct lsm6dsv_sensor_s *hg = &priv->sensor[LSM6DSV_ACCEL_HG_IDX];
  FAR struct lsm6dsv_sensor_s *rot = &priv->sensor[LSM6DSV_ROT_IDX];
  uint8_t status[2];
  uint8_t raw[2];
  sensor_data_t temp;
  unsigned int count;
  unsigned int nxl;
  unsigned int ngy;
  unsigned int nhg;
  unsigned int nrot;
  unsigned int i;
  uint64_t now;
  uint8_t tag;

  /* The reads hold priv->lock, which keeps the embedded functions bank
   * switched out.  Publishing does not: push_event() takes the upper
   * half's lock, which is held while it calls activate().
   */

  nxmutex_lock(&priv->lock);
  lsm6dsv_readregs(priv, LSM6DSV_FIFO_STATUS1, status, 2);
  now = sensor_get_timestamp();

  count = status[0] | ((status[1] & FIFO_STATUS2_DIFF8) << 8);
  if ((status[1] & FIFO_STATUS2_OVR_IA) != 0 && !priv->overrun)
    {
      snwarn("WARNING: FIFO overrun, samples lost\n");
      priv->overrun = true;
    }

  if (count == 0)
    {
      nxmutex_unlock(&priv->lock);
      return;
    }

  /* One temperature reading for the batch: 256 LSB/degC, 0 at 25 degC */

  lsm6dsv_readregs(priv, LSM6DSV_OUT_TEMP_L, raw, 2);
  temp = sensor_data_add(sensor_data_divi(sensor_data_itof(
                           (int16_t)(raw[0] | (raw[1] << 8))), 256),
                         sensor_data_itof(25));

  /* Read the whole FIFO, then count each sensor's samples so that each
   * can be stamped by how many newer ones follow it.
   */

  nxl = 0;
  ngy = 0;
  nhg = 0;
  nrot = 0;
  for (i = 0; i < count; i++)
    {
      lsm6dsv_readregs(priv, LSM6DSV_FIFO_DATA_TAG, priv->words[i],
                       LSM6DSV_WORD_SIZE);
      tag = priv->words[i][0] >> FIFO_TAG_SHIFT;
      nxl += (tag == FIFO_TAG_XL);
      ngy += (tag == FIFO_TAG_GY);
      nhg += (tag == FIFO_TAG_XL_HG);
      nrot += (tag == FIFO_TAG_SFLP_GAME && priv->words[i][1] != 0);
    }

  nxmutex_unlock(&priv->lock);

  for (i = 0; i < count; i++)
    {
      tag = priv->words[i][0] >> FIFO_TAG_SHIFT;
      if (tag == FIFO_TAG_XL)
        {
          nxl--;
          if (accel->enabled)
            {
              lsm6dsv_push(accel, priv->words[i], now, nxl, temp);
            }
        }
      else if (tag == FIFO_TAG_GY)
        {
          ngy--;
          if (gyro->enabled)
            {
              lsm6dsv_push(gyro, priv->words[i], now, ngy, temp);
            }
        }
      else if (tag == FIFO_TAG_XL_HG)
        {
          nhg--;
          if (hg->enabled)
            {
              lsm6dsv_push(hg, priv->words[i], now, nhg, temp);
            }
        }
      else if (tag == FIFO_TAG_SFLP_GAME)
        {
          nrot -= (priv->words[i][1] != 0);
          if (rot->enabled)
            {
              lsm6dsv_push_rot(priv, priv->words[i], now, nrot);
            }
        }
    }
}

/****************************************************************************
 * Name: lsm6dsv_thread
 ****************************************************************************/

static int lsm6dsv_thread(int argc, FAR char **argv)
{
  FAR struct lsm6dsv_dev_s *priv =
    (FAR struct lsm6dsv_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (true)
    {
      if (!lsm6dsv_running(priv))
        {
          nxsem_wait_uninterruptible(&priv->run);
          continue;
        }

      lsm6dsv_drain(priv);
      nxsched_usleep(CONFIG_SENSORS_LSM6DSV_POLL_INTERVAL);
    }

  return OK;
}

/****************************************************************************
 * Name: lsm6dsv_init
 *
 * Description:
 *   Check WHO_AM_I, reset, and set up SPI-only, block data update,
 *   address auto-increment, all sensors off.
 *
 ****************************************************************************/

static int lsm6dsv_init(FAR struct lsm6dsv_dev_s *priv)
{
  uint8_t id;
  int i;

  id = lsm6dsv_getreg(priv, LSM6DSV_WHO_AM_I);
  if (id != LSM6DSV_ID && id != LSM6DSV320X_ID)
    {
      snerr("ERROR: WHO_AM_I %02x, expected %02x or %02x\n", id,
            LSM6DSV_ID, LSM6DSV320X_ID);
      return -ENODEV;
    }

  priv->id = id;

  lsm6dsv_putreg(priv, LSM6DSV_CTRL3, CTRL3_SW_RESET);
  for (i = 0; (lsm6dsv_getreg(priv, LSM6DSV_CTRL3) & CTRL3_SW_RESET) != 0;
       i++)
    {
      if (i >= 100)
        {
          snerr("ERROR: software reset did not finish\n");
          return -EIO;
        }

      up_udelay(100);
    }

  lsm6dsv_putreg(priv, LSM6DSV_IF_CFG, IF_CFG_I2C_I3C_DISABLE);
  lsm6dsv_putreg(priv, LSM6DSV_CTRL3, CTRL3_BDU | CTRL3_IF_INC);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsv_register_uorb
 ****************************************************************************/

int lsm6dsv_register_uorb(int devno, int hgdevno, int rotdevno,
                          FAR struct spi_dev_s *spi, uint32_t devid)
{
  FAR struct lsm6dsv_dev_s *priv;
  FAR struct lsm6dsv_sensor_s *sensor;
  FAR char *argv[2];
  char arg1[32];
  int ret;
  int i;

  DEBUGASSERT(spi != NULL);

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->spi   = spi;
  priv->devid = devid;
  nxmutex_init(&priv->lock);
  nxsem_init(&priv->run, 0, 0);

  ret = lsm6dsv_init(priv);
  if (ret < 0)
    {
      goto errout;
    }

  priv->hgdevno  = priv->id == LSM6DSV320X_ID ? hgdevno : -1;
  priv->rotdevno = priv->id == LSM6DSV320X_ID ? rotdevno : -1;

  for (i = 0; i < LSM6DSV_MAX_IDX; i++)
    {
      sensor              = &priv->sensor[i];
      sensor->dev         = priv;
      sensor->lower.ops   = &g_lsm6dsv_ops;
      sensor->lower.type  = i == LSM6DSV_GYRO_IDX ?
                            SENSOR_TYPE_GYROSCOPE :
                            i == LSM6DSV_ROT_IDX ?
                            SENSOR_TYPE_ROTATION_VECTOR :
                            SENSOR_TYPE_ACCELEROMETER;
      sensor->lower.nbuffer = CONFIG_SENSORS_LSM6DSV_BUFSIZE;
      sensor->odr         = lsm6dsv_odr(sensor,
                                        CONFIG_SENSORS_LSM6DSV_INTERVAL);
    }

  lsm6dsv_setfs(&priv->sensor[LSM6DSV_ACCEL_IDX],
                CONFIG_SENSORS_LSM6DSV_ACCEL_FS);
  lsm6dsv_setfs(&priv->sensor[LSM6DSV_GYRO_IDX],
                CONFIG_SENSORS_LSM6DSV_GYRO_FS);
  if (priv->hgdevno >= 0)
    {
      lsm6dsv_setfs(&priv->sensor[LSM6DSV_ACCEL_HG_IDX],
                    CONFIG_SENSORS_LSM6DSV_HG_FS);
    }

  /* A unit quaternion, to about 1e-3 (half precision) */

  priv->sensor[LSM6DSV_ROT_IDX].range = sensor_data_itof(1);
  priv->sensor[LSM6DSV_ROT_IDX].scale = sensor_data_ftof(1.0f / 1024.0f);

  ret = sensor_register(&priv->sensor[LSM6DSV_ACCEL_IDX].lower, devno);
  if (ret < 0)
    {
      goto errout;
    }

  ret = sensor_register(&priv->sensor[LSM6DSV_GYRO_IDX].lower, devno);
  if (ret < 0)
    {
      goto errout_accel;
    }

  if (priv->hgdevno >= 0)
    {
      ret = sensor_register(&priv->sensor[LSM6DSV_ACCEL_HG_IDX].lower,
                            priv->hgdevno);
      if (ret < 0)
        {
          goto errout_gyro;
        }
    }

  if (priv->rotdevno >= 0)
    {
      ret = sensor_register(&priv->sensor[LSM6DSV_ROT_IDX].lower,
                            priv->rotdevno);
      if (ret < 0)
        {
          goto errout_hg;
        }
    }

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;
  ret = kthread_create("lsm6dsv", CONFIG_SENSORS_LSM6DSV_THREAD_PRIORITY,
                       CONFIG_SENSORS_LSM6DSV_THREAD_STACKSIZE,
                       lsm6dsv_thread, argv);
  if (ret < 0)
    {
      goto errout_rot;
    }

  return OK;

errout_rot:
  if (priv->rotdevno >= 0)
    {
      sensor_unregister(&priv->sensor[LSM6DSV_ROT_IDX].lower,
                        priv->rotdevno);
    }

errout_hg:
  if (priv->hgdevno >= 0)
    {
      sensor_unregister(&priv->sensor[LSM6DSV_ACCEL_HG_IDX].lower,
                        priv->hgdevno);
    }

errout_gyro:
  sensor_unregister(&priv->sensor[LSM6DSV_GYRO_IDX].lower, devno);
errout_accel:
  sensor_unregister(&priv->sensor[LSM6DSV_ACCEL_IDX].lower, devno);
errout:
  nxsem_destroy(&priv->run);
  nxmutex_destroy(&priv->lock);
  kmm_free(priv);
  return ret;
}

#endif /* CONFIG_SENSORS_LSM6DSV */
