/****************************************************************************
 * drivers/net/phy/ti/dp83869.c
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

/* TI DP83869HM gigabit PHY, in RGMII-to-copper mode.  The only setup
 * beyond the generic Clause 22 driver is the RGMII clock delays.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

#include <nuttx/debug.h>
#include <nuttx/net/phy.h>
#include <nuttx/net/ti_phy.h>

#include "phy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DP83869_PHY_ID          0x2000a0f0
#define DP83869_PHY_ID_MASK     0xfffffff0

/* Extended registers, reached through MMD 0x1f */

#define DP83869_DEVADDR         0x1f

#define DP83869_RGMIICTL        0x0032
#define DP83869_RGMIIDCTL       0x0086
#define DP83869_OP_MODE_DECODE  0x01df

/* RGMIICTL: the delay bits are "delay disabled" when set */

#define RGMIICTL_RX_DELAY_DIS   (1 << 0)
#define RGMIICTL_TX_DELAY_DIS   (1 << 1)

/* RGMIIDCTL: delays in 0.25 ns steps, (n + 1) * 0.25 ns */

#define RGMIIDCTL_RX_SHIFT      0
#define RGMIIDCTL_TX_SHIFT      4
#define RGMIIDCTL_ENFORCE       (3 << 8)
#define RGMIIDCTL_STEP_PS       250
#define RGMIIDCTL_MAX_PS        4000

#define DP83869_DELAY(ps) \
  ((((ps) + RGMIIDCTL_STEP_PS - 1) / RGMIIDCTL_STEP_PS) - 1)

#if CONFIG_NET_PHY_TI_DP83869_TX_DELAY_PS < RGMIIDCTL_STEP_PS || \
    CONFIG_NET_PHY_TI_DP83869_TX_DELAY_PS > RGMIIDCTL_MAX_PS || \
    CONFIG_NET_PHY_TI_DP83869_RX_DELAY_PS < RGMIIDCTL_STEP_PS || \
    CONFIG_NET_PHY_TI_DP83869_RX_DELAY_PS > RGMIIDCTL_MAX_PS
#  error "DP83869 RGMII delays must be 250-4000 ps"
#endif

/* OP_MODE_DECODE: bits 2:0 are the operating mode, 0 for RGMII to
 * copper; bit 5 selects MII rather than RGMII.
 */

#define OP_MODE_MASK            7
#define OP_MODE_RGMII_COPPER    0
#define OP_MODE_MII_SEL         (1 << 5)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dp83869_config_init(FAR struct phy_device_s *phydev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct phy_driver_s g_dp83869_driver =
{
  .name         = "TI DP83869",
  .phy_id       = DP83869_PHY_ID,
  .phy_id_mask  = DP83869_PHY_ID_MASK,
  .config_init  = dp83869_config_init,
  .config_aneg  = genphy_config_aneg,
  .read_status  = genphy_read_status,
  .config_power = genphy_config_power,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dp83869_config_init(FAR struct phy_device_s *phydev)
{
  uint16_t ctl;
  int ret;

  ret = phy_read_mmd(phydev, DP83869_DEVADDR, DP83869_OP_MODE_DECODE);
  if (ret < 0)
    {
      return ret;
    }

  if ((ret & OP_MODE_MASK) != OP_MODE_RGMII_COPPER ||
      (ret & OP_MODE_MII_SEL) != 0)
    {
      nerr("ERROR: DP83869 strapped for mode %#x, not RGMII to copper\n",
           ret);
      return -EOPNOTSUPP;
    }

  switch (phydev->interface)
    {
      case PHY_INTERFACE_RGMII:
        ctl = RGMIICTL_TX_DELAY_DIS | RGMIICTL_RX_DELAY_DIS;
        break;

      case PHY_INTERFACE_RGMII_ID:
        ctl = 0;
        break;

      case PHY_INTERFACE_RGMII_TXID:
        ctl = RGMIICTL_RX_DELAY_DIS;
        break;

      case PHY_INTERFACE_RGMII_RXID:
        ctl = RGMIICTL_TX_DELAY_DIS;
        break;

      default:
        return -EOPNOTSUPP;
    }

  ret = phy_write_mmd(phydev, DP83869_DEVADDR, DP83869_RGMIIDCTL,
           RGMIIDCTL_ENFORCE |
           DP83869_DELAY(CONFIG_NET_PHY_TI_DP83869_TX_DELAY_PS) <<
             RGMIIDCTL_TX_SHIFT |
           DP83869_DELAY(CONFIG_NET_PHY_TI_DP83869_RX_DELAY_PS) <<
             RGMIIDCTL_RX_SHIFT);
  if (ret < 0)
    {
      return ret;
    }

  ret = phy_modify_mmd(phydev, DP83869_DEVADDR, DP83869_RGMIICTL,
                       RGMIICTL_TX_DELAY_DIS | RGMIICTL_RX_DELAY_DIS, ctl);
  if (ret < 0)
    {
      return ret;
    }

  return genphy_config_init(phydev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dp83869_register
 ****************************************************************************/

int dp83869_register(void)
{
  return phy_driver_register(&g_dp83869_driver);
}
