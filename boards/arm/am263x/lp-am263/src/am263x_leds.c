/****************************************************************************
 * boards/arm/am263x/lp-am263/src/am263x_leds.c
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

/* The LaunchPad's LEDs: the user LED on GPIO26, as NuttX's status LED
 * (CONFIG_ARCH_LEDS) or as /dev/userleds, and the eight TPIC2810 LEDs on
 * I2C1 as /dev/userleds1.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/leds/tpic2810.h>
#include <arch/board/board.h>

#include "am263x_gpio.h"
#include "am263x_i2c.h"
#include "am263x_pinmux.h"
#include "lp-am263.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_LED_USER       26

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct am263x_pinmux_s g_led_pins[] =
{
  { AM263X_PAD_UART0_CTSN, PAD_MODE(7) | PAD_PULL_DISABLE },  /* GPIO26 */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void led_initialize(void)
{
  am263x_pinmux_config(g_led_pins,
                       sizeof(g_led_pins) / sizeof(g_led_pins[0]));
  am263x_gpio_output(GPIO_LED_USER, false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  led_initialize();
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == 1)
    {
      am263x_gpio_write(GPIO_LED_USER, true);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == 1)
    {
      am263x_gpio_write(GPIO_LED_USER, false);
    }
}

#else /* CONFIG_ARCH_LEDS */
/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  led_initialize();
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED_USER)
    {
      am263x_gpio_write(GPIO_LED_USER, ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  am263x_gpio_write(GPIO_LED_USER, (ledset & BOARD_LED_USER_BIT) != 0);
}

/****************************************************************************
 * Name: board_userled_getall
 ****************************************************************************/

#ifdef CONFIG_USERLED_LOWER_READSTATE
void board_userled_getall(uint32_t *ledset)
{
  /* IN_DATA follows the pad, so this is what the pin is doing */

  *ledset = am263x_gpio_read(GPIO_LED_USER) ? BOARD_LED_USER_BIT : 0;
}
#endif
#endif /* CONFIG_ARCH_LEDS */

/****************************************************************************
 * Name: am263x_tpic2810_initialize
 *
 * Description:
 *   Register the TPIC2810 on I2C1 as /dev/userleds1.  The I2C1 pins are
 *   muxed by am263x_eeprom_initialize(), which runs first.
 *
 ****************************************************************************/

#if defined(CONFIG_TPIC2810) && defined(CONFIG_AM263X_I2C1)
int am263x_tpic2810_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = am263x_i2cbus_initialize(BOARD_TPIC2810_I2C_BUS);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  ret = tpic2810_register("/dev/userleds1", i2c, BOARD_TPIC2810_I2C_ADDR);
  if (ret < 0)
    {
      am263x_i2cbus_uninitialize(i2c);
    }

  return ret;
}
#endif
