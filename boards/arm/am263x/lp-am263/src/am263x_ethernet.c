/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_ethernet.c
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

/* Ethernet on the LP-AM263: CPSW MAC port 1, RGMII1, to the DP83869 (U22)
 * at MDIO address 3 behind RJ-45 J7.  RGMII1 reaches the PHY through
 * muxes shared with ICSSM MII1: GPIO1 (RGMII1_ICSSM_MUX_EN) and GPIO58
 * (AM263_MUX_EN) both high select the CPSW.  The PHYs' reset is the
 * SoC warm reset, so there is no reset GPIO.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/net/mdio.h>
#include <nuttx/net/mii.h>
#include <nuttx/net/ti_phy.h>

#include "am263x_cpsw.h"
#include "am263x_gpio.h"
#include "am263x_mdio.h"
#include "am263x_pinmux.h"
#include "lp-am263.h"

#ifdef CONFIG_AM263X_MDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_RGMII1_MUX_EN  1
#define GPIO_MUX_EN         58

#define PHY_ADDR_MAX        31

/* RGMII pads: no pulls, fast slew.  MDIO has a pull-up. */

#define PAD_RGMII           (PAD_MODE(0) | PAD_PULL_DISABLE)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct am263x_pinmux_s g_eth_pins[] =
{
  { AM263X_PAD_RGMII1_RXC,    PAD_RGMII },
  { AM263X_PAD_RGMII1_RX_CTL, PAD_RGMII },
  { AM263X_PAD_RGMII1_RD0,    PAD_RGMII },
  { AM263X_PAD_RGMII1_RD1,    PAD_RGMII },
  { AM263X_PAD_RGMII1_RD2,    PAD_RGMII },
  { AM263X_PAD_RGMII1_RD3,    PAD_RGMII },
  { AM263X_PAD_RGMII1_TXC,    PAD_RGMII },
  { AM263X_PAD_RGMII1_TX_CTL, PAD_RGMII },
  { AM263X_PAD_RGMII1_TD0,    PAD_RGMII },
  { AM263X_PAD_RGMII1_TD1,    PAD_RGMII },
  { AM263X_PAD_RGMII1_TD2,    PAD_RGMII },
  { AM263X_PAD_RGMII1_TD3,    PAD_RGMII },
  { AM263X_PAD_MDIO_MDIO,     PAD_MODE(0) | PAD_PULL_UP },
  { AM263X_PAD_MDIO_MDC,      PAD_MODE(0) | PAD_PULL_DISABLE },
  { AM263X_PAD_QSPI_CSN1,     PAD_MODE(7) | PAD_PULL_DISABLE },  /* GPIO1 */
  { AM263X_PAD_EPWM7_B,       PAD_MODE(7) | PAD_PULL_DISABLE },  /* GPIO58 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_phy_probe
 *
 * Description:
 *   Log every PHY that answers on the bus, with its ID and link state.
 *
 ****************************************************************************/

static int am263x_phy_probe(FAR struct mdio_bus_s *bus)
{
  uint16_t id1;
  uint16_t id2;
  uint16_t msr;
  int found = 0;
  int addr;

  for (addr = 0; addr <= PHY_ADDR_MAX; addr++)
    {
      if (mdio_read(bus, addr, MII_PHYID1, &id1) < 0 ||
          mdio_read(bus, addr, MII_PHYID2, &id2) < 0 ||
          id1 == 0xffff || (id1 == 0 && id2 == 0))
        {
          continue;
        }

      /* The link bit latches low: read twice for the current state */

      mdio_read(bus, addr, MII_MSR, &msr);
      if (mdio_read(bus, addr, MII_MSR, &msr) < 0)
        {
          msr = 0;
        }

      syslog(LOG_INFO, "PHY %d: ID %04x:%04x, link %s%s\n", addr, id1, id2,
             (msr & MII_MSR_LINKSTATUS) ? "up" : "down",
             (msr & MII_MSR_ANEGCOMPLETE) ? ", autoneg complete" : "");
      found++;
    }

  if (found == 0)
    {
      syslog(LOG_ERR, "ERROR: no PHY answers on MDIO\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_ethernet_initialize
 *
 * Description:
 *   Mux the RGMII1 and MDIO pads, route RGMII1 to the CPSW, bring up the
 *   MDIO bus, register the PHY driver and report the PHYs on the bus.
 *   The CPSW driver (eth0) uses the PHY when the interface comes up.
 *
 ****************************************************************************/

int am263x_ethernet_initialize(void)
{
  FAR struct mdio_bus_s *bus;
  int ret;

  am263x_pinmux_config(g_eth_pins,
                       sizeof(g_eth_pins) / sizeof(g_eth_pins[0]));
  am263x_gpio_output(GPIO_RGMII1_MUX_EN, true);
  am263x_gpio_output(GPIO_MUX_EN, true);

  bus = am263x_mdio_initialize();
  if (bus == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_NET_PHY_TI_DP83869
  ret = dp83869_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: dp83869_register failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_AM263X_CPSW_ETH) && defined(CONFIG_AM263X_I2C1) && \
    defined(CONFIG_I2C_EE_24XX)
  /* No MAC address in the eFuse: use the one in the board ID EEPROM */

  {
    uint8_t mac[6];

    if (am263x_efuse_macaddr(mac) < 0 && am263x_eeprom_macaddr(mac) >= 0)
      {
        ret = am263x_cpsw_setmacaddr(mac);
        syslog(LOG_INFO, "eth0: MAC address from the EEPROM: %d\n", ret);
      }
  }
#endif

  UNUSED(ret);
  return am263x_phy_probe(bus);
}

#endif /* CONFIG_AM263X_MDIO */
