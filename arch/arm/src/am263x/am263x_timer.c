/****************************************************************************
 * arch/arm/src/am263x/am263x_timer.c
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

/* The system timer: RTI0 as a oneshot lower half, under the generic
 * arch_alarm layer (CONFIG_ALARM_ARCH), which drives either the periodic
 * tick or, with CONFIG_SCHED_TICKLESS, the tickless alarm.
 *
 * RTI0 counter block 0 counts at 1 MHz from the 25 MHz crystal, so the
 * timer does not depend on the PLLs.  FRC0 is 32 bits and wraps every
 * 71.6 minutes; the driver extends it to 64 bits in software.  Compare 0
 * is the alarm.  Compare 1 fires every half wrap, only to keep the
 * extension current when nothing else reads the counter.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/oneshot.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/am263x_rti.h"
#include "am263x_clockconfig.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RTI_BASE            AM263X_RTI0_BASE
#define RTI_COUNT_HZ        1000000
#define RTI_PRESCALE        (AM263X_RTI0_FREQUENCY / RTI_COUNT_HZ)

#if (AM263X_RTI0_FREQUENCY % RTI_COUNT_HZ) != 0
#  error "RTI0 clock must be a whole number of MHz"
#endif

#define RTI_REG(off)        (RTI_BASE + (off))

/* Compare 0 matches when FRC0 equals it, so an alarm must be set far
 * enough ahead that the counter cannot pass it before the write lands.
 */

#define RTI_MIN_DELAY       2           /* counts (us) */

/* Longest alarm: half the counter, so that a target is never ambiguous */

#define RTI_MAX_DELAY       0x7fffffff  /* counts (us) */

/* Compare 1 period: half the counter */

#define RTI_WRAP_PERIOD     0x80000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_rti_s
{
  struct oneshot_lowerhalf_s lower;  /* Must be first */
  spinlock_t lock;
  uint32_t last;                     /* FRC0 at the last read */
  uint64_t high;                     /* Upper 32 bits of the count */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static clkcnt_t am263x_rti_current(FAR struct oneshot_lowerhalf_s *lower);
static void am263x_rti_start(FAR struct oneshot_lowerhalf_s *lower,
                             clkcnt_t delay);
static void am263x_rti_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                                      clkcnt_t cnt);
static void am263x_rti_cancel(FAR struct oneshot_lowerhalf_s *lower);
static clkcnt_t am263x_rti_max_delay(FAR struct oneshot_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct oneshot_operations_s g_am263x_rti_ops =
{
  .current        = am263x_rti_current,
  .start          = am263x_rti_start,
  .start_absolute = am263x_rti_start_absolute,
  .cancel         = am263x_rti_cancel,
  .max_delay      = am263x_rti_max_delay,
};

static struct am263x_rti_s g_am263x_rti =
{
  .lower =
  {
    .ops = &g_am263x_rti_ops,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_rti_count
 *
 * Description:
 *   The 64-bit count.  The caller holds priv->lock.  It must run at least
 *   once per wrap (compare 1 sees to that).
 *
 ****************************************************************************/

static uint64_t am263x_rti_count(FAR struct am263x_rti_s *priv)
{
  uint32_t now = getreg32(RTI_REG(RTI_FRC0_OFFSET));

  if (now < priv->last)
    {
      priv->high += (uint64_t)1 << 32;
    }

  priv->last = now;
  return priv->high | now;
}

static clkcnt_t am263x_rti_current(FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct am263x_rti_s *priv = (FAR struct am263x_rti_s *)lower;
  irqstate_t flags;
  uint64_t count;

  flags = spin_lock_irqsave(&priv->lock);
  count = am263x_rti_count(priv);
  spin_unlock_irqrestore(&priv->lock, flags);

  return count;
}

static void am263x_rti_start_absolute(FAR struct oneshot_lowerhalf_s *lower,
                                      clkcnt_t cnt)
{
  FAR struct am263x_rti_s *priv = (FAR struct am263x_rti_s *)lower;
  irqstate_t flags;
  uint64_t target;
  uint64_t now;

  flags = spin_lock_irqsave(&priv->lock);

  now = am263x_rti_count(priv);
  target = cnt;
  if ((int64_t)(target - now) < RTI_MIN_DELAY)
    {
      target = now + RTI_MIN_DELAY;
    }

  putreg32(RTI_INT(0), RTI_REG(RTI_INTFLAG_OFFSET));

  /* Compare 0 only matches on equality: if the counter got past the
   * target before the write took effect, set it again further on.
   */

  for (; ; )
    {
      putreg32((uint32_t)target, RTI_REG(RTI_COMP0_OFFSET));

      now = am263x_rti_count(priv);
      if ((int64_t)(target - now) > 0 ||
          (getreg32(RTI_REG(RTI_INTFLAG_OFFSET)) & RTI_INT(0)) != 0)
        {
          break;
        }

      target = now + RTI_MIN_DELAY;
    }

  spin_unlock_irqrestore(&priv->lock, flags);
}

static void am263x_rti_start(FAR struct oneshot_lowerhalf_s *lower,
                             clkcnt_t delay)
{
  am263x_rti_start_absolute(lower, am263x_rti_current(lower) + delay);
}

static void am263x_rti_cancel(FAR struct oneshot_lowerhalf_s *lower)
{
  FAR struct am263x_rti_s *priv = (FAR struct am263x_rti_s *)lower;
  irqstate_t flags;

  /* Compare 0 stays enabled (see up_timer_initialize()): move it a whole
   * wrap away instead.  If nothing sets an alarm before then, the callback
   * runs once early, which is harmless.
   */

  flags = spin_lock_irqsave(&priv->lock);
  putreg32((uint32_t)am263x_rti_count(priv) - 1, RTI_REG(RTI_COMP0_OFFSET));
  putreg32(RTI_INT(0), RTI_REG(RTI_INTFLAG_OFFSET));
  spin_unlock_irqrestore(&priv->lock, flags);
}

static clkcnt_t am263x_rti_max_delay(FAR struct oneshot_lowerhalf_s *lower)
{
  return RTI_MAX_DELAY;
}

/****************************************************************************
 * Name: am263x_rti_alarm
 *
 * Description:
 *   Compare 0: the alarm.
 *
 ****************************************************************************/

static int am263x_rti_alarm(int irq, FAR void *context, FAR void *arg)
{
  FAR struct am263x_rti_s *priv = arg;

  putreg32(RTI_INT(0), RTI_REG(RTI_INTFLAG_OFFSET));
  oneshot_process_callback(&priv->lower);
  return OK;
}

/****************************************************************************
 * Name: am263x_rti_wrap
 *
 * Description:
 *   Compare 1: every half wrap, read the counter so that the 64-bit
 *   extension never misses a wrap.
 *
 ****************************************************************************/

static int am263x_rti_wrap(int irq, FAR void *context, FAR void *arg)
{
  FAR struct am263x_rti_s *priv = arg;

  putreg32(RTI_INT(1), RTI_REG(RTI_INTFLAG_OFFSET));
  am263x_rti_current(&priv->lower);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 ****************************************************************************/

void up_timer_initialize(void)
{
  FAR struct am263x_rti_s *priv = &g_am263x_rti;

  modifyreg32(RTI_REG(RTI_GCTRL_OFFSET), RTI_GCTRL_CNT0EN, 0);
  putreg32(RTI_INT(0) | RTI_INT(1) | RTI_INT(2) | RTI_INT(3),
           RTI_REG(RTI_CLEARINT_OFFSET));
  putreg32(RTI_INT(0) | RTI_INT(1) | RTI_INT(2) | RTI_INT(3),
           RTI_REG(RTI_INTFLAG_OFFSET));

  /* FRC0 at 1 MHz; compares 0 and 1 both on FRC0.  Compare 0 does not
   * reload (it is set for every alarm); compare 1 reloads every half wrap.
   * Both interrupts stay enabled: toggling the enable around each alarm
   * lost alarms (the VIM stopped presenting the RTI's interrupt).
   */

  putreg32(0, RTI_REG(RTI_UC0_OFFSET));
  putreg32(0, RTI_REG(RTI_FRC0_OFFSET));
  putreg32(RTI_PRESCALE - 1, RTI_REG(RTI_CPUC0_OFFSET));
  putreg32(0, RTI_REG(RTI_COMPCTRL_OFFSET));
  putreg32(0, RTI_REG(RTI_UDCP0_OFFSET));
  putreg32(RTI_WRAP_PERIOD, RTI_REG(RTI_COMP1_OFFSET));
  putreg32(RTI_WRAP_PERIOD, RTI_REG(RTI_UDCP1_OFFSET));

  spin_lock_init(&priv->lock);
  priv->last = 0;
  priv->high = 0;

  up_set_irq_type(AM263X_IRQ_RTI0_0, IRQ_RISING_EDGE);
  up_set_irq_type(AM263X_IRQ_RTI0_1, IRQ_RISING_EDGE);
  DEBUGVERIFY(irq_attach(AM263X_IRQ_RTI0_0, am263x_rti_alarm, priv));
  DEBUGVERIFY(irq_attach(AM263X_IRQ_RTI0_1, am263x_rti_wrap, priv));

  putreg32(0xffffffff, RTI_REG(RTI_COMP0_OFFSET));
  putreg32(RTI_INT(0) | RTI_INT(1), RTI_REG(RTI_SETINT_OFFSET));
  modifyreg32(RTI_REG(RTI_GCTRL_OFFSET), 0, RTI_GCTRL_CNT0EN);

  up_enable_irq(AM263X_IRQ_RTI0_0);
  up_enable_irq(AM263X_IRQ_RTI0_1);

  oneshot_count_init(&priv->lower, RTI_COUNT_HZ);
  up_alarm_set_lowerhalf(&priv->lower);
}
