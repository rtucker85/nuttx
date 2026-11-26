/****************************************************************************
 * arch/arm/src/am263x/am263x_tsense.c
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

/* On-die temperature sensors (TRM 6.2.2.2.2 Thermal Manager).
 *
 * The FSM converts all four sensors round robin from the 32 kHz clock
 * (about 11 ms a round) and keeps each one's last 7-bit code in its
 * RESULT register, so a read is a register read.  The codes are
 * calibrated with the per-chip trims in eFuse: each sensor's code at -40,
 * 30 and 150 degC.  A trim is used only if it is near the TRM's typical
 * curve, and the 30 degC one only if it lies between the other two (on
 * the parts seen so far it repeats the 150 degC row, and TSENSE2's field
 * holds no CTAT code).  Without usable trims a sensor follows the TRM's
 * typical table.
 *
 * The uORB topic carries the hottest of TSENSE0-2.  TSENSE3, a PTAT
 * sensor for TI's internal use with no documented conversion, is only
 * reported raw through AM263X_TSENSEIOC_INFO.
 *
 * Alerts (CONFIG_AM263X_TSENSE_ALERT): TSENSE0 and TSENSE1 interrupt when
 * the die goes over the hot threshold, then again when it has cooled
 * below the cold one; each is logged.  The comparators work on the codes
 * as if they rose with temperature (measured on an LP-AM263: the HOT one
 * is set while code >= ALERT.HOT, the COLD one while code <= ALERT.COLD).
 * These sensors are CTAT, so it is the COLD comparator, given the code
 * of the hot threshold, that trips when the die overheats, and the HOT
 * comparator, given the code of the cold threshold, that says it has
 * cooled: the reverse of the sequence in TRM 6.2.2.2.2.4.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>

#include <arch/chip/tsense.h>

#include "arm_internal.h"
#include "hardware/am263x_rcm.h"
#include "hardware/am263x_tsense.h"
#include "am263x_controlss.h"
#include "am263x_tsense.h"

#ifdef CONFIG_AM263X_TSENSE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32 kHz cycles between conversions (1 ms): a round of four then takes
 * 4 * (54 + 32) cycles, about 11 ms.
 */

#define TSENSE_DELAY_CYCLES     32

/* The first round, at most, after the FSM starts */

#define TSENSE_START_TIMEOUT_US 50000

/* The sensors the topic takes the hottest of: TSENSE0-2 (CTAT) */

#define TSENSE_TOPIC_MASK       0x7

/* A trim is used only if it is within this many codes (2 degC each) of
 * the TRM's typical curve: a field that is not a trim (TSENSE2's, on the
 * parts seen so far) would otherwise give a plausible-looking but wrong
 * temperature.
 */

#define TSENSE_TRIM_WINDOW      10

#define TSENSE_DEFAULT_INTERVAL 1000000
#define TSENSE_MIN_INTERVAL     20000
#define TSENSE_MAX_INTERVAL     10000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tsense_cal_s
{
  uint16_t m40;                       /* Code at -40 degC */
  uint16_t t30;                       /* at 30 degC */
  uint16_t t150;                      /* at 150 degC */
  uint8_t  kind;                      /* AM263X_TSENSE_CAL_* */
};

struct tsense_dev_s
{
  struct sensor_lowerhalf_s lower;    /* Must be first */
  uint32_t                  interval; /* Microseconds between readings */
  struct work_s             work;
  bool                      enabled;
};

struct tsense_state_s
{
  struct tsense_cal_s cal[AM263X_TSENSE_NSENSORS];
  bool                initialized;
  uint8_t             hottest;
#ifdef CONFIG_AM263X_TSENSE_ALERT
  uint8_t             alert_hot;      /* Bit n: TSENSEn over the alert */
  uint32_t            alerts;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int tsense_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable);
static int tsense_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us);
static int tsense_get_info(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR struct sensor_device_info_s *info);
static int tsense_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sensor_ops_s g_tsense_ops =
{
  .activate     = tsense_activate,
  .set_interval = tsense_set_interval,
  .get_info     = tsense_get_info,
  .control      = tsense_control,
};

static struct tsense_state_s g_tsense;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tsense_trim
 *
 * Description:
 *   Sensor n's 10-bit field of a trim row (xxC1:xxC0, 41 bits).
 *
 ****************************************************************************/

static uint16_t tsense_trim(uintptr_t row0, int n)
{
  uint64_t row = getreg32(row0) | ((uint64_t)getreg32(row0 + 4) << 32);

  return (row >> (TSENSE_TRIM_BITS * n)) & TSENSE_TRIM_MASK;
}

/****************************************************************************
 * Name: tsense_between
 *
 * Description:
 *   Whether b lies strictly between a and c, at least min from each.
 *
 ****************************************************************************/

static bool tsense_between(int a, int b, int c, int min)
{
  return (a > c) ? (b >= c + min && b <= a - min) :
                   (b >= a + min && b <= c - min);
}

/****************************************************************************
 * Name: tsense_read_trims
 ****************************************************************************/

static void tsense_read_trims(void)
{
  FAR struct tsense_cal_s *cal;
  int n;

  for (n = 0; n < AM263X_TSENSE_NSENSORS; n++)
    {
      cal       = &g_tsense.cal[n];
      cal->m40  = tsense_trim(AM263X_TSENSE_TRIM_M40C0, n);
      cal->t30  = tsense_trim(AM263X_TSENSE_TRIM_30C0, n);
      cal->t150 = tsense_trim(AM263X_TSENSE_TRIM_150C0, n);

      if (n == 3)
        {
          cal->kind = AM263X_TSENSE_CAL_NONE;
        }
      else if (abs((int)cal->m40 - TSENSE_TABLE_CODE_MAX) >
               TSENSE_TRIM_WINDOW ||
               abs((int)cal->t150 - TSENSE_TABLE_CODE_MIN) >
               TSENSE_TRIM_WINDOW)
        {
          cal->kind = AM263X_TSENSE_CAL_TABLE;
        }
      else if (tsense_between(cal->m40, cal->t30, cal->t150, 2))
        {
          cal->kind = AM263X_TSENSE_CAL_3POINT;
        }
      else
        {
          cal->kind = AM263X_TSENSE_CAL_2POINT;
        }

      sninfo("TSENSE%d: trims %u %u %u (-40, 30, 150 degC), cal %u\n",
             n, cal->m40, cal->t30, cal->t150, cal->kind);
    }

  if (g_tsense.cal[0].kind == AM263X_TSENSE_CAL_TABLE ||
      g_tsense.cal[1].kind == AM263X_TSENSE_CAL_TABLE)
    {
      syslog(LOG_WARNING, "TSENSE: no usable factory trims for TSENSE0/1, "
             "using the TRM's typical table (+-8 degC)\n");
    }
}

/****************************************************************************
 * Name: tsense_lerp
 *
 * Description:
 *   The temperature at code, on the line through (c1, t1) and (c2, t2).
 *
 ****************************************************************************/

static float tsense_lerp(int code, int c1, float t1, int c2, float t2)
{
  return t1 + (code - c1) * (t2 - t1) / (c2 - c1);
}

/****************************************************************************
 * Name: tsense_to_celsius
 ****************************************************************************/

static float tsense_to_celsius(int n, int code)
{
  FAR const struct tsense_cal_s *cal = &g_tsense.cal[n];

  switch (cal->kind)
    {
      case AM263X_TSENSE_CAL_3POINT:

        /* The segment on the code's side of the 30 degC point */

        if ((code - (int)cal->t30) * ((int)cal->m40 - (int)cal->t30) > 0)
          {
            return tsense_lerp(code, cal->m40, -40.0f, cal->t30, 30.0f);
          }

        return tsense_lerp(code, cal->t30, 30.0f, cal->t150, 150.0f);

      case AM263X_TSENSE_CAL_2POINT:
        return tsense_lerp(code, cal->m40, -40.0f, cal->t150, 150.0f);

      case AM263X_TSENSE_CAL_TABLE:
        code = MIN(MAX(code, TSENSE_TABLE_CODE_MIN), TSENSE_TABLE_CODE_MAX);
        return TSENSE_TABLE_TEMP(code);

      default:
        return NAN;
    }
}

/****************************************************************************
 * Name: tsense_code
 *
 * Description:
 *   Sensor n's last code, or a negated errno if it has none yet.
 *
 ****************************************************************************/

static int tsense_code(int n)
{
  uint32_t result = getreg32(AM263X_TSENSE_RESULT(n));
  int code = result & TSENSE_RESULT_DTEMP_MASK;

  /* The register reads 0 until the first conversion completes; no sensor
   * gives 0 in the -40 to 150 degC range.
   */

  return code != 0 ? code : -EAGAIN;
}

#ifdef CONFIG_AM263X_TSENSE_ALERT

/****************************************************************************
 * Name: tsense_to_code
 *
 * Description:
 *   The code sensor n gives at celsius: the threshold to program.
 *
 ****************************************************************************/

static int tsense_to_code(int n, int celsius)
{
  FAR const struct tsense_cal_s *cal = &g_tsense.cal[n];
  float code;

  switch (cal->kind)
    {
      case AM263X_TSENSE_CAL_3POINT:
        if (celsius < 30)
          {
            code = tsense_lerp(celsius, -40, cal->m40, 30, cal->t30);
          }
        else
          {
            code = tsense_lerp(celsius, 30, cal->t30, 150, cal->t150);
          }
        break;

      case AM263X_TSENSE_CAL_2POINT:
        code = tsense_lerp(celsius, -40, cal->m40, 150, cal->t150);
        break;

      default:
        code = (198 - celsius) / 2.0f;
        break;
    }

  return MIN(MAX((int)lroundf(code), 1), TSENSE_RESULT_DTEMP_MASK);
}

/****************************************************************************
 * Name: tsense_interrupt
 *
 * Description:
 *   TSENSE_H and TSENSE_L.  Overheated (the COLD comparator, see above):
 *   mask it and unmask the HOT comparator, which fires once the die has
 *   cooled below the cold threshold; then back.
 *
 ****************************************************************************/

static int tsense_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status = getreg32(AM263X_TSENSE_STATUS);
  int n;

  for (n = 0; n < 2; n++)
    {
      if ((status & TSENSE_STATUS_COLD(n)) != 0)
        {
          am263x_top_ctrl_modify(AM263X_TSENSE_CNTL(n),
                                 TSENSE_CNTL_MASK_HOT,
                                 TSENSE_CNTL_MASK_COLD);
          g_tsense.alert_hot |= 1 << n;
          g_tsense.alerts++;
          syslog(LOG_WARNING, "TSENSE%d: over %d degC (code %d)\n", n,
                 CONFIG_AM263X_TSENSE_ALERT_HOT, tsense_code(n));
        }
      else if ((status & TSENSE_STATUS_HOT(n)) != 0)
        {
          am263x_top_ctrl_modify(AM263X_TSENSE_CNTL(n),
                                 TSENSE_CNTL_MASK_COLD,
                                 TSENSE_CNTL_MASK_HOT);
          g_tsense.alert_hot &= ~(1 << n);
          g_tsense.alerts++;
          syslog(LOG_WARNING, "TSENSE%d: back under %d degC (code %d)\n",
                 n, CONFIG_AM263X_TSENSE_ALERT_COLD, tsense_code(n));
        }
    }

  return OK;
}

/****************************************************************************
 * Name: tsense_alert_setup
 ****************************************************************************/

static void tsense_alert_setup(void)
{
  int hot;
  int cold;
  int n;

  for (n = 0; n < 2; n++)
    {
      /* CTAT: the hot threshold is the smaller code.  Keep at least one
       * code between them, or the two comparators would hand the
       * interrupt back and forth.
       */

      hot  = tsense_to_code(n, CONFIG_AM263X_TSENSE_ALERT_HOT);
      cold = MAX(tsense_to_code(n, CONFIG_AM263X_TSENSE_ALERT_COLD),
                 hot + 2);

      /* Over: code <= ALERT.COLD.  Cooled: code >= ALERT.HOT. */

      am263x_top_ctrl_modify(AM263X_TSENSE_ALERT(n),
                             TSENSE_ALERT_HOT_MASK | TSENSE_ALERT_COLD_MASK,
                             (cold << TSENSE_ALERT_HOT_SHIFT) |
                             (hot << TSENSE_ALERT_COLD_SHIFT));
      sninfo("TSENSE%d: alert at code %d, cleared at %d\n", n, hot, cold);
    }

  irq_attach(AM263X_IRQ_TSENSE_H, tsense_interrupt, NULL);
  irq_attach(AM263X_IRQ_TSENSE_L, tsense_interrupt, NULL);

  /* Watch for overheating: the cooled comparator stays masked until then */

  for (n = 0; n < 2; n++)
    {
      am263x_top_ctrl_modify(AM263X_TSENSE_CNTL(n), TSENSE_CNTL_MASK_COLD,
                             0);
    }

  up_enable_irq(AM263X_IRQ_TSENSE_H);
  up_enable_irq(AM263X_IRQ_TSENSE_L);
}

#endif /* CONFIG_AM263X_TSENSE_ALERT */

/****************************************************************************
 * Name: tsense_hottest
 *
 * Description:
 *   The hottest of the TSENSE_TOPIC_MASK sensors.
 *
 ****************************************************************************/

static int tsense_hottest(FAR float *celsius)
{
  float temp;
  float max = -INFINITY;
  int ret = -EAGAIN;
  int n;

  for (n = 0; n < AM263X_TSENSE_NSENSORS; n++)
    {
      if ((TSENSE_TOPIC_MASK & (1 << n)) != 0 &&
          am263x_tsense_read(n, &temp) >= 0 && temp > max)
        {
          max              = temp;
          g_tsense.hottest = n;
          ret              = OK;
        }
    }

  *celsius = max;
  return ret;
}

/****************************************************************************
 * Name: tsense_delay
 ****************************************************************************/

static clock_t tsense_delay(FAR struct tsense_dev_s *priv)
{
  clock_t ticks = USEC2TICK(priv->interval);

  return ticks > 0 ? ticks : 1;
}

/****************************************************************************
 * Name: tsense_worker
 ****************************************************************************/

static void tsense_worker(FAR void *arg)
{
  FAR struct tsense_dev_s *priv = arg;
  struct sensor_temp temp;
  float celsius;

  work_queue(LPWORK, &priv->work, tsense_worker, priv, tsense_delay(priv));

  if (tsense_hottest(&celsius) < 0)
    {
      return;
    }

  temp.timestamp   = sensor_get_timestamp();
  temp.temperature = sensor_data_ftof(celsius);
  priv->lower.push_event(priv->lower.priv, &temp, sizeof(temp));
}

/****************************************************************************
 * Name: tsense_activate
 ****************************************************************************/

static int tsense_activate(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep, bool enable)
{
  FAR struct tsense_dev_s *priv = (FAR struct tsense_dev_s *)lower;

  if (enable == priv->enabled)
    {
      return OK;
    }

  if (enable)
    {
      work_queue(LPWORK, &priv->work, tsense_worker, priv, 0);
    }
  else
    {
      work_cancel(LPWORK, &priv->work);
    }

  priv->enabled = enable;
  return OK;
}

/****************************************************************************
 * Name: tsense_set_interval
 ****************************************************************************/

static int tsense_set_interval(FAR struct sensor_lowerhalf_s *lower,
                               FAR struct file *filep,
                               FAR uint32_t *period_us)
{
  FAR struct tsense_dev_s *priv = (FAR struct tsense_dev_s *)lower;

  priv->interval = MIN(MAX(*period_us, TSENSE_MIN_INTERVAL),
                       TSENSE_MAX_INTERVAL);
  *period_us     = priv->interval;

  if (priv->enabled)
    {
      work_queue(LPWORK, &priv->work, tsense_worker, priv,
                 tsense_delay(priv));
    }

  return OK;
}

/****************************************************************************
 * Name: tsense_get_info
 ****************************************************************************/

static int tsense_get_info(FAR struct sensor_lowerhalf_s *lower,
                           FAR struct file *filep,
                           FAR struct sensor_device_info_s *info)
{
  memset(info, 0, sizeof(*info));
  info->version    = 1;
  info->max_range  = sensor_data_ftof(150.0f);   /* -40 to 150 degC */
  info->resolution = sensor_data_ftof(2.0f);
  info->min_delay  = TSENSE_MIN_INTERVAL;
  info->max_delay  = TSENSE_MAX_INTERVAL;
  strlcpy(info->name, "AM263x TSENSE", sizeof(info->name));
  strlcpy(info->vendor, "Texas Instruments", sizeof(info->vendor));
  return OK;
}

/****************************************************************************
 * Name: tsense_control
 ****************************************************************************/

static int tsense_control(FAR struct sensor_lowerhalf_s *lower,
                          FAR struct file *filep, int cmd,
                          unsigned long arg)
{
  FAR struct am263x_tsense_info_s *info =
    (FAR struct am263x_tsense_info_s *)(uintptr_t)arg;
  float temp;
  int code;
  int n;

  if (cmd != AM263X_TSENSEIOC_INFO)
    {
      return -ENOTTY;
    }

  if (info == NULL)
    {
      return -EINVAL;
    }

  memset(info, 0, sizeof(*info));
  for (n = 0; n < AM263X_TSENSE_NSENSORS; n++)
    {
      code = tsense_code(n);
      info->code[n]     = code > 0 ? code : 0;
      info->temp[n]     = am263x_tsense_read(n, &temp) >= 0 ? temp : NAN;
      info->cal[n]      = g_tsense.cal[n].kind;
      info->trim_m40[n] = g_tsense.cal[n].m40;
      info->trim_30[n]  = g_tsense.cal[n].t30;
      info->trim_150[n] = g_tsense.cal[n].t150;
    }

  tsense_hottest(&temp);
  info->hottest  = g_tsense.hottest;
#ifdef CONFIG_AM263X_TSENSE_ALERT
  info->alert_hot = g_tsense.alert_hot;
  info->alerts    = g_tsense.alerts;
#endif
  info->cfg      = getreg32(AM263X_TSENSE_CFG);
  for (n = 0; n < 2; n++)
    {
      info->tshut[n] = getreg32(AM263X_TSENSE_TSHUT(n));
      info->alert[n] = getreg32(AM263X_TSENSE_ALERT(n));
      info->cntl[n]  = getreg32(AM263X_TSENSE_CNTL(n));
    }

  info->status_raw = getreg32(AM263X_TSENSE_STATUS_RAW);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_tsense_initialize
 ****************************************************************************/

int am263x_tsense_initialize(void)
{
  uint32_t sel = TSENSE_CFG_SENSOR_SEL((1 << AM263X_TSENSE_NSENSORS) - 1);
  int waited;
  int n;

  if (g_tsense.initialized)
    {
      return OK;
    }

  /* The 32 kHz clock is ungated at reset; make sure */

  if ((getreg32(AM263X_MSS_RCM_TEMPSENSE_32K_CLK_GATE) & RCM_CLK_GATED) != 0)
    {
      putreg32(AM263X_KICK0_UNLOCK, AM263X_MSS_RCM_LOCK0_KICK0);
      putreg32(AM263X_KICK1_UNLOCK, AM263X_MSS_RCM_LOCK0_KICK1);
      putreg32(0, AM263X_MSS_RCM_TEMPSENSE_32K_CLK_GATE);
      putreg32(AM263X_KICK_LOCK, AM263X_MSS_RCM_LOCK0_KICK0);
      putreg32(AM263X_KICK_LOCK, AM263X_MSS_RCM_LOCK0_KICK1);
    }

  tsense_read_trims();

  /* Stop the FSM and power the sensors down, mask the alerts and zero
   * their thresholds, and clear the FIFOs and accumulators: a known state
   * whatever the boot loader (or the last image) did.
   */

  am263x_top_ctrl_modify(AM263X_TSENSE_CFG, TSENSE_CFG_ENABLE,
                         TSENSE_CFG_OFF);
  for (n = 0; n < 2; n++)
    {
      am263x_top_ctrl_modify(AM263X_TSENSE_CNTL(n), 0,
                             TSENSE_CNTL_MASK_ALL | TSENSE_CNTL_FIFO_CLEAR |
                             TSENSE_CNTL_ACCU_CLEAR);
      am263x_top_ctrl_modify(AM263X_TSENSE_ALERT(n),
                             TSENSE_ALERT_HOT_MASK | TSENSE_ALERT_COLD_MASK,
                             0);
    }

  /* Power up, select all four, then start the FSM */

  am263x_top_ctrl_modify(AM263X_TSENSE_CFG,
                         TSENSE_CFG_OFF | TSENSE_CFG_SENSOR_SEL_MASK |
                         TSENSE_CFG_DELAY_MASK,
                         sel | TSENSE_CFG_DELAY(TSENSE_DELAY_CYCLES));
  up_udelay(100);
  am263x_top_ctrl_modify(AM263X_TSENSE_CFG, 0, TSENSE_CFG_ENABLE);

  /* Wait for a first code from every sensor */

  for (waited = 0; waited < TSENSE_START_TIMEOUT_US; waited += 1000)
    {
      for (n = 0; n < AM263X_TSENSE_NSENSORS; n++)
        {
          if (tsense_code(n) < 0)
            {
              break;
            }
        }

      if (n == AM263X_TSENSE_NSENSORS)
        {
          break;
        }

      up_udelay(1000);
    }

  if (waited >= TSENSE_START_TIMEOUT_US)
    {
      snerr("ERROR: TSENSE: no conversion (CFG %08" PRIx32 ")\n",
            getreg32(AM263X_TSENSE_CFG));
      return -ETIMEDOUT;
    }

  sninfo("TSENSE: running after %d us\n", waited);

#ifdef CONFIG_AM263X_TSENSE_ALERT
  tsense_alert_setup();
#endif

  g_tsense.initialized = true;
  return OK;
}

/****************************************************************************
 * Name: am263x_tsense_read
 ****************************************************************************/

int am263x_tsense_read(int n, FAR float *celsius)
{
  int code;

  if (n < 0 || n >= AM263X_TSENSE_NSENSORS || celsius == NULL)
    {
      return -EINVAL;
    }

  if (g_tsense.cal[n].kind == AM263X_TSENSE_CAL_NONE)
    {
      return -ENODATA;
    }

  code = tsense_code(n);
  if (code < 0)
    {
      return code;
    }

  *celsius = tsense_to_celsius(n, code);
  return OK;
}

/****************************************************************************
 * Name: am263x_tsense_register
 ****************************************************************************/

int am263x_tsense_register(int devno)
{
  FAR struct tsense_dev_s *priv;
  int ret;

  ret = am263x_tsense_initialize();
  if (ret < 0)
    {
      return ret;
    }

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->interval   = TSENSE_DEFAULT_INTERVAL;
  priv->lower.ops  = &g_tsense_ops;
  priv->lower.type = SENSOR_TYPE_TEMPERATURE;

  ret = sensor_register(&priv->lower, devno);
  if (ret < 0)
    {
      snerr("ERROR: sensor_register failed: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}

#endif /* CONFIG_AM263X_TSENSE */
