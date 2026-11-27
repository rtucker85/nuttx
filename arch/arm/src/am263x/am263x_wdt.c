/****************************************************************************
 * arch/arm/src/am263x/am263x_wdt.c
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

/* The MSS watchdogs WDT0-3 are RTI modules used only for their digital
 * windowed watchdog (DWWD).  The 25-bit down counter runs from the WDT
 * functional clock (here the 25 MHz crystal / 4) and is reloaded with
 * (DWDPRLD << 13) | 0x1fff by the key sequence 0xe51a, 0xa35c.  At zero,
 * or on a wrong key or a feed outside the window, the reaction is either
 * a reset or an NMI; this driver always selects the reset.  The WDTn reset
 * goes to TOP_RCM, which turns it into a SoC warm reset if its enable in
 * WARM_RESET_CONFIG is set (the reset value).  The ROM then boots again,
 * through the SBL, and WARM_RST_CAUSE shows the WDT.
 *
 * Once enabled, RTIDWDCTRL cannot be written back to disabled: the only
 * way to stop the counter is to reset the module, through MSS_RCM
 * WDTn_RST_CTRL.  stop() does that.
 *
 * With RTIGCTRL.COS clear, the counter stops while the CPU is halted in
 * the debugger, so a JTAG session does not reset the SoC.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <inttypes.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/watchdog.h>

#include "arm_internal.h"
#include "hardware/am263x_memorymap.h"
#include "hardware/am263x_rcm.h"
#include "hardware/am263x_rti.h"
#include "am263x_wdt.h"

#ifdef CONFIG_AM263X_WDT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AM263X_NWDT          4
#define AM263X_WDT_BASE(n)   (AM263X_WDT0_BASE + 0x1000 * (n))

/* WDT clock: the 25 MHz crystal / 4 = 6.25 MHz.  One preload step is
 * 2^13 clocks = 1.31 ms, so the timeout ranges from 1.31 ms to 5.37 s.  A
 * slower clock would give longer timeouts, but the module's registers
 * are not accessible below 6.25 MHz (/5 and slower data-abort on the
 * LP-AM263).
 */

#define WDT_XTAL_HZ          25000000
#define WDT_CLKDIV           4
#define WDT_CLK_HZ           (WDT_XTAL_HZ / WDT_CLKDIV)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_wdt_s
{
  FAR const struct watchdog_ops_s *ops; /* Lower half operations */
  uintptr_t base;                       /* RTI (WDTn) base address */
  uint32_t clkhz;                       /* WDT functional clock */
  uint32_t timeout;                     /* Timeout, ms */
  uint16_t preload;                     /* RTIDWDPRLD for timeout */
  uint8_t  wdt;                         /* WDT instance, 0-3 */
  bool     started;                     /* The counter is running */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int am263x_wdt_start(FAR struct watchdog_lowerhalf_s *lower);
static int am263x_wdt_stop(FAR struct watchdog_lowerhalf_s *lower);
static int am263x_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int am263x_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                FAR struct watchdog_status_s *status);
static int am263x_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_wdt_ops =
{
  .start      = am263x_wdt_start,
  .stop       = am263x_wdt_stop,
  .keepalive  = am263x_wdt_keepalive,
  .getstatus  = am263x_wdt_getstatus,
  .settimeout = am263x_wdt_settimeout,
};

static struct am263x_wdt_s g_wdt[AM263X_NWDT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_wdt_ms2preload / am263x_wdt_preload2ms
 *
 * Description:
 *   Convert between a timeout in ms and RTIDWDPRLD.  The counter expires
 *   (DWDPRLD + 1) * 2^13 clocks after a reload.  A timeout that is not a
 *   whole number of steps is rounded up.
 *
 ****************************************************************************/

static uint32_t am263x_wdt_ms2preload(uint32_t clkhz, uint32_t ms)
{
  uint64_t steps = ((uint64_t)ms * clkhz + 1000ull * (1 << 13) - 1) /
                   (1000ull * (1 << 13));

  return steps > 0 ? steps - 1 : 0;
}

static uint32_t am263x_wdt_preload2ms(uint32_t clkhz, uint32_t preload)
{
  return ((uint64_t)(preload + 1) << RTI_DWDPRLD_SHIFT) * 1000 / clkhz;
}

/****************************************************************************
 * Name: am263x_wdt_clkhz
 *
 * Description:
 *   The WDTn functional clock as MSS_RCM has it set up, for a watchdog
 *   found running (started by a boot loader or an image loaded over it).
 *
 ****************************************************************************/

static uint32_t am263x_wdt_clkhz(int wdt)
{
  uint32_t src = getreg32(AM263X_MSS_RCM_WDT_CLK_SRC_SEL(wdt)) & 0x7;
  uint32_t div = (getreg32(AM263X_MSS_RCM_WDT_CLK_DIV_VAL(wdt)) & 0xf) + 1;
  uint32_t hz;

  switch (src)
    {
      case 1:
      case 5:
        hz = 10000000;
        break;

      case 2:
        hz = 200000000;
        break;

      case 3:
        hz = 192000000;
        break;

      case 7:
        hz = 32768;
        break;

      default:                  /* XTAL; WUCPUCLK is the crystal too */
        hz = WDT_XTAL_HZ;
        break;
    }

  return hz / div;
}

/****************************************************************************
 * Name: am263x_wdt_feed
 ****************************************************************************/

static void am263x_wdt_feed(FAR struct am263x_wdt_s *priv)
{
  irqstate_t flags = up_irq_save();

  putreg32(RTI_WDKEY_FIRST, priv->base + RTI_WDKEY_OFFSET);
  putreg32(RTI_WDKEY_SECOND, priv->base + RTI_WDKEY_OFFSET);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: am263x_wdt_modreset
 *
 * Description:
 *   Pulse the WDTn module reset in MSS_RCM: the DWWD is disabled and every
 *   register back at its reset value.  The MSS_RCM partition is unlocked
 *   by am263x_clockconfig().
 *
 ****************************************************************************/

static void am263x_wdt_modreset(FAR struct am263x_wdt_s *priv)
{
  putreg32(RCM_RST_ASSERT, AM263X_MSS_RCM_WDT_RST_CTRL(priv->wdt));
  getreg32(AM263X_MSS_RCM_WDT_RST_CTRL(priv->wdt));
  putreg32(RCM_RST_DEASSERT, AM263X_MSS_RCM_WDT_RST_CTRL(priv->wdt));
  getreg32(AM263X_MSS_RCM_WDT_RST_CTRL(priv->wdt));
}

/****************************************************************************
 * Name: am263x_wdt_start
 *
 * Description:
 *   Start the watchdog with the current timeout: reset the module, set
 *   its clock, make its expiry a SoC warm reset, and enable the DWWD with
 *   the window open 100 % of the period.
 *
 ****************************************************************************/

static int am263x_wdt_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct am263x_wdt_s *priv = (FAR struct am263x_wdt_s *)lower;
  irqstate_t flags;
  uint32_t regval;
  int i;

  if (priv->started)
    {
      am263x_wdt_feed(priv);
      return OK;
    }

  am263x_wdt_modreset(priv);

  /* Switch the clock and wait for the switch: the module's registers do
   * not answer while it is slow or switching
   */

  putreg32(RCM_WDT_CLKSRC_XTAL, AM263X_MSS_RCM_WDT_CLK_SRC_SEL(priv->wdt));
  putreg32(RCM_CLK_DIV(WDT_CLKDIV),
           AM263X_MSS_RCM_WDT_CLK_DIV_VAL(priv->wdt));
  putreg32(RCM_CLK_UNGATED, AM263X_MSS_RCM_WDT_CLK_GATE(priv->wdt));

  for (i = 0; i < 1000; i++)
    {
      if ((getreg32(AM263X_MSS_RCM_WDT_CLK_STATUS(priv->wdt)) & 0xffff) ==
          RCM_CLK_STATUS(RCM_WDT_CLKSRC_XTAL, WDT_CLKDIV))
        {
          break;
        }

      up_udelay(1);
    }

  if (priv->clkhz != WDT_CLK_HZ)
    {
      priv->clkhz   = WDT_CLK_HZ;
      priv->preload = am263x_wdt_ms2preload(priv->clkhz, priv->timeout);
      priv->timeout = am263x_wdt_preload2ms(priv->clkhz, priv->preload);
    }

  /* WDTn expiry -> TOP_RCM -> SoC warm reset (the reset value, but a boot
   * loader may have changed it)
   */

  flags = up_irq_save();
  putreg32(AM263X_KICK0_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_TOP_RCM_LOCK0_KICK1);
  regval  = getreg32(AM263X_TOP_RCM_WARM_RESET_CONFIG);
  regval &= ~TOP_RCM_WARM_RESET_CONFIG_WDOG_MASK(priv->wdt);
  regval |= TOP_RCM_WARM_RESET_CONFIG_WDOG_EN(priv->wdt);
  putreg32(regval, AM263X_TOP_RCM_WARM_RESET_CONFIG);
  putreg32(AM263X_KICK_LOCK, AM263X_TOP_RCM_LOCK0_KICK0);
  putreg32(AM263X_KICK_LOCK, AM263X_TOP_RCM_LOCK0_KICK1);
  up_irq_restore(flags);

  /* Counter stops while the CPU is halted (COS = 0) */

  putreg32(0, priv->base + RTI_GCTRL_OFFSET);
  putreg32(RTI_WDSTATUS_ALL, priv->base + RTI_WDSTATUS_OFFSET);
  putreg32(RTI_WWDSIZECTRL_100, priv->base + RTI_WWDSIZECTRL_OFFSET);
  putreg32(RTI_WWDRXNCTRL_RESET, priv->base + RTI_WWDRXNCTRL_OFFSET);
  putreg32(priv->preload, priv->base + RTI_DWDPRLD_OFFSET);
  putreg32(RTI_DWDCTRL_ENABLE, priv->base + RTI_DWDCTRL_OFFSET);

  if (getreg32(priv->base + RTI_DWDCTRL_OFFSET) != RTI_DWDCTRL_ENABLE)
    {
      return -EIO;
    }

  priv->started = true;
  return OK;
}

/****************************************************************************
 * Name: am263x_wdt_stop
 ****************************************************************************/

static int am263x_wdt_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct am263x_wdt_s *priv = (FAR struct am263x_wdt_s *)lower;

  am263x_wdt_modreset(priv);
  if (getreg32(priv->base + RTI_DWDCTRL_OFFSET) == RTI_DWDCTRL_ENABLE)
    {
      return -EBUSY;
    }

  priv->started = false;
  return OK;
}

/****************************************************************************
 * Name: am263x_wdt_keepalive
 ****************************************************************************/

static int am263x_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  am263x_wdt_feed((FAR struct am263x_wdt_s *)lower);
  return OK;
}

/****************************************************************************
 * Name: am263x_wdt_getstatus
 ****************************************************************************/

static int am263x_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                                FAR struct watchdog_status_s *status)
{
  FAR struct am263x_wdt_s *priv = (FAR struct am263x_wdt_s *)lower;
  uint32_t cntr;

  status->flags   = WDFLAGS_RESET;
  status->timeout = priv->timeout;
  status->timeleft = 0;

  if (priv->started)
    {
      cntr = getreg32(priv->base + RTI_DWDCNTR_OFFSET);
      status->flags   |= WDFLAGS_ACTIVE;
      status->timeleft = (uint64_t)cntr * 1000 / priv->clkhz;
    }

  return OK;
}

/****************************************************************************
 * Name: am263x_wdt_settimeout
 *
 * Description:
 *   Set the timeout, in ms.  If the watchdog is running, the new preload
 *   takes effect at once: the counter is reloaded with it.
 *
 ****************************************************************************/

static int am263x_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                                 uint32_t timeout)
{
  FAR struct am263x_wdt_s *priv = (FAR struct am263x_wdt_s *)lower;
  uint32_t preload;

  if (timeout < 1 ||
      timeout > am263x_wdt_preload2ms(priv->clkhz, RTI_DWDPRLD_MAX))
    {
      return -ERANGE;
    }

  preload = am263x_wdt_ms2preload(priv->clkhz, timeout);
  priv->preload = preload;
  priv->timeout = am263x_wdt_preload2ms(priv->clkhz, preload);

  if (priv->started)
    {
      putreg32(preload, priv->base + RTI_DWDPRLD_OFFSET);
      am263x_wdt_feed(priv);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_wdt_initialize
 ****************************************************************************/

int am263x_wdt_initialize(FAR const char *devpath, int wdt)
{
  FAR struct am263x_wdt_s *priv;
  FAR void *handle;

  if (wdt < 0 || wdt >= AM263X_NWDT)
    {
      return -EINVAL;
    }

  priv        = &g_wdt[wdt];
  priv->ops   = &g_wdt_ops;
  priv->base  = AM263X_WDT_BASE(wdt);
  priv->wdt   = wdt;

  if (getreg32(priv->base + RTI_DWDCTRL_OFFSET) == RTI_DWDCTRL_ENABLE)
    {
      /* Already running: started by a boot loader, or by the image this
       * one was loaded over.  Keep it running and feed it.
       */

      priv->clkhz   = am263x_wdt_clkhz(wdt);
      priv->preload = getreg32(priv->base + RTI_DWDPRLD_OFFSET) &
                      RTI_DWDPRLD_MAX;
      priv->timeout = am263x_wdt_preload2ms(priv->clkhz, priv->preload);
      priv->started = true;
      am263x_wdt_feed(priv);

      syslog(LOG_INFO, "WDT%d already running, timeout %" PRIu32 " ms\n",
             wdt, priv->timeout);
    }
  else
    {
      priv->clkhz   = WDT_CLK_HZ;
      priv->preload = am263x_wdt_ms2preload(priv->clkhz,
                                            CONFIG_AM263X_WDT_TIMEOUT);
      priv->timeout = am263x_wdt_preload2ms(priv->clkhz, priv->preload);
      priv->started = false;
    }

  handle = watchdog_register(devpath,
                             (FAR struct watchdog_lowerhalf_s *)priv);
  return handle != NULL ? OK : -ENODEV;
}

#endif /* CONFIG_AM263X_WDT */
