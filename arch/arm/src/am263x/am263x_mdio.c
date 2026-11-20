/****************************************************************************
 * arch/arm/src/am263x/am263x_mdio.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/net/mdio.h>

#include "arm_internal.h"
#include "hardware/am263x_cpsw.h"
#include "am263x_mdio.h"

#ifdef CONFIG_AM263X_MDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A Clause 22 frame is 64 bits, 29 us at 2.2 MHz.  Allow for slow clocks
 * and for the controller finishing a frame of its own first.
 */

#define MDIO_TIMEOUT_US     2000

#define MDIO_CLKDIV \
  ((AM263X_CPSW_CLK_HZ + CONFIG_AM263X_MDIO_FREQUENCY - 1) / \
   CONFIG_AM263X_MDIO_FREQUENCY - 1)

#if MDIO_CLKDIV > 0xffff
#  error "CONFIG_AM263X_MDIO_FREQUENCY is too low"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int am263x_mdio_read(FAR struct mdio_lowerhalf_s *lower,
                            uint8_t phyaddr, uint8_t regaddr,
                            FAR uint16_t *value);
static int am263x_mdio_write(FAR struct mdio_lowerhalf_s *lower,
                             uint8_t phyaddr, uint8_t regaddr,
                             uint16_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct mdio_ops_s g_am263x_mdio_ops =
{
  .read  = am263x_mdio_read,
  .write = am263x_mdio_write,
};

static struct mdio_lowerhalf_s g_am263x_mdio_lower =
{
  .ops = &g_am263x_mdio_ops,
};

static FAR struct mdio_bus_s *g_am263x_mdio_bus;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mdio_wait
 *
 * Description:
 *   Wait for USERACCESS0.GO to clear.  Returns the register's last value,
 *   or -ETIMEDOUT.
 *
 ****************************************************************************/

static int64_t am263x_mdio_wait(void)
{
  uint32_t regval;
  int us;

  for (us = 0; us < MDIO_TIMEOUT_US; us++)
    {
      regval = getreg32(AM263X_MDIO_USERACCESS(0));
      if ((regval & MDIO_USERACCESS_GO) == 0)
        {
          return regval;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: am263x_mdio_read
 ****************************************************************************/

static int am263x_mdio_read(FAR struct mdio_lowerhalf_s *lower,
                            uint8_t phyaddr, uint8_t regaddr,
                            FAR uint16_t *value)
{
  int64_t ret;

  ret = am263x_mdio_wait();
  if (ret < 0)
    {
      nerr("ERROR: MDIO busy\n");
      return (int)ret;
    }

  putreg32(MDIO_USERACCESS_GO | MDIO_USERACCESS_PHYADR(phyaddr) |
           MDIO_USERACCESS_REGADR(regaddr), AM263X_MDIO_USERACCESS(0));

  ret = am263x_mdio_wait();
  if (ret < 0)
    {
      nerr("ERROR: MDIO read of PHY %u reg %u timed out\n",
           phyaddr, regaddr);
      return (int)ret;
    }

  /* No acknowledge: no PHY at this address */

  if ((ret & MDIO_USERACCESS_ACK) == 0)
    {
      return -EIO;
    }

  *value = (uint16_t)(ret & MDIO_USERACCESS_DATA_MASK);
  return OK;
}

/****************************************************************************
 * Name: am263x_mdio_write
 ****************************************************************************/

static int am263x_mdio_write(FAR struct mdio_lowerhalf_s *lower,
                             uint8_t phyaddr, uint8_t regaddr,
                             uint16_t value)
{
  int64_t ret;

  ret = am263x_mdio_wait();
  if (ret < 0)
    {
      nerr("ERROR: MDIO busy\n");
      return (int)ret;
    }

  putreg32(MDIO_USERACCESS_GO | MDIO_USERACCESS_WRITE |
           MDIO_USERACCESS_PHYADR(phyaddr) |
           MDIO_USERACCESS_REGADR(regaddr) | value,
           AM263X_MDIO_USERACCESS(0));

  ret = am263x_mdio_wait();
  if (ret < 0)
    {
      nerr("ERROR: MDIO write of PHY %u reg %u timed out\n",
           phyaddr, regaddr);
      return (int)ret;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mdio_initialize
 ****************************************************************************/

FAR struct mdio_bus_s *am263x_mdio_initialize(void)
{
  if (g_am263x_mdio_bus != NULL)
    {
      return g_am263x_mdio_bus;
    }

  /* Normal mode: no manual (bit-bang) mode and no state-change
   * monitoring; software drives every frame through USERACCESS0.
   */

  modifyreg32(AM263X_MDIO_POLL,
              MDIO_POLL_MANUALMODE | MDIO_POLL_STATECHANGEMODE, 0);

  putreg32(MDIO_CONTROL_ENABLE | MDIO_CONTROL_FAULTENB |
           MDIO_CONTROL_CLKDIV(MDIO_CLKDIV), AM263X_MDIO_CONTROL);

  ninfo("MDIO version %08" PRIx32 ", MDC %d Hz\n",
        getreg32(AM263X_MDIO_VERSION),
        AM263X_CPSW_CLK_HZ / (MDIO_CLKDIV + 1));

  g_am263x_mdio_bus = mdio_register(&g_am263x_mdio_lower);
  if (g_am263x_mdio_bus == NULL)
    {
      nerr("ERROR: mdio_register failed\n");
    }

  return g_am263x_mdio_bus;
}

#endif /* CONFIG_AM263X_MDIO */
