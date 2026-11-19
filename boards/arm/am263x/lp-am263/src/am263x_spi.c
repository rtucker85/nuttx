/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_spi.c
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

#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "am263x_gpio.h"
#include "am263x_pinmux.h"
#include "am263x_spi.h"
#include "lp-am263.h"

#ifdef CONFIG_AM263X_MCSPI0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* MCSPI0 on its own pads, mux mode 0: SPI0_CLK, SPI0_D0 (controller out),
 * SPI0_D1 (controller in) and SPI0_CS0.  D0 and D1 go straight to
 * BoosterPack site 1 (J2.15, J2.14); CLK and CS0 go through the U51 mux
 * (J1.7, J2.18), see CONFIG_LP_AM263_BOOSTERPACK1_SPI.
 */

static const struct am263x_pinmux_s g_spi0_pins[] =
{
  { AM263X_PAD_SPI0_CS0, PAD_MODE(0) | PAD_PULL_DISABLE | PAD_SLEW_LOW },
  { AM263X_PAD_SPI0_CLK, PAD_MODE(0) | PAD_PULL_DISABLE | PAD_SLEW_LOW },
  { AM263X_PAD_SPI0_D0,  PAD_MODE(0) | PAD_PULL_DISABLE | PAD_SLEW_LOW },
  { AM263X_PAD_SPI0_D1,  PAD_MODE(0) | PAD_PULL_UP | PAD_SLEW_LOW },
#ifdef CONFIG_LP_AM263_BOOSTERPACK1_SPI
  { AM263X_PAD_EPWM7_B,  PAD_MODE(7) | PAD_PULL_DISABLE },  /* GPIO58 */
  { AM263X_PAD_EPWM10_A, PAD_MODE(7) | PAD_PULL_DISABLE },  /* GPIO63 */
#endif
};

/* GPIO58 high: the BoosterPack muxes (U51, U52) on and the PRU0 PHY mux
 * (U49) off.  GPIO63 low: U51 passes SPI0_CLK and SPI0_CS0, not the PRU0
 * GPIOs.
 */

#define GPIO_MUX_EN         58
#define GPIO_BP_MUX_SEL     63

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_spi0select, am263x_spi0status
 *
 * Description:
 *   Every device on MCSPI0 uses the controller's CS0.
 *
 ****************************************************************************/

void am263x_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool selected)
{
  spiinfo("devid: %08" PRIx32 " CS: %s\n",
          devid, selected ? "assert" : "de-assert");
  am263x_spi_hwcs(dev, 0, selected);
}

uint8_t am263x_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int am263x_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid,
                       bool cmd)
{
  return -ENODEV;
}
#endif

#endif /* CONFIG_AM263X_MCSPI0 */

/****************************************************************************
 * Name: am263x_spi_initialize
 *
 * Description:
 *   Mux the SPI pads, bring up the enabled buses and, with
 *   CONFIG_SPI_DRIVER, register them as /dev/spiN.
 *
 ****************************************************************************/

int am263x_spi_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_AM263X_MCSPI0
  FAR struct spi_dev_s *spi;

  am263x_pinmux_config(g_spi0_pins,
                       sizeof(g_spi0_pins) / sizeof(g_spi0_pins[0]));
#  ifdef CONFIG_LP_AM263_BOOSTERPACK1_SPI
  am263x_gpio_output(GPIO_BP_MUX_SEL, false);
  am263x_gpio_output(GPIO_MUX_EN, true);
#  endif

  spi = am263x_spibus_initialize(0);
  if (spi == NULL)
    {
      return -ENODEV;
    }

#  ifdef CONFIG_SPI_DRIVER
  ret = spi_register(spi, 0);
#  endif
#endif

  return ret;
}
