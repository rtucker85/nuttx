# ##############################################################################
# arch/arm/src/am263x/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS
    am263x_boot.c
    am263x_clockconfig.c
    am263x_gpio.c
    am263x_irq.c
    am263x_mpuinit.c
    am263x_pinmux.c
    am263x_serial.c
    am263x_systemreset.c
    am263x_timer.c)

if(CONFIG_AM263X_PRU)
  list(APPEND SRCS am263x_pru.c)
endif()

if(CONFIG_AM263X_I2C)
  list(APPEND SRCS am263x_i2c.c)
endif()

if(CONFIG_AM263X_MCSPI)
  list(APPEND SRCS am263x_spi.c)
endif()

if(CONFIG_AM263X_QSPI)
  list(APPEND SRCS am263x_qspi.c)
endif()

if(CONFIG_AM263X_CPSW)
  list(APPEND SRCS am263x_cpsw.c)
endif()

if(CONFIG_AM263X_MDIO)
  list(APPEND SRCS am263x_mdio.c)
endif()

if(CONFIG_AM263X_CONTROLSS)
  list(APPEND SRCS am263x_controlss.c)
endif()

if(CONFIG_AM263X_ADC)
  list(APPEND SRCS am263x_adc.c)
endif()

if(CONFIG_AM263X_EPWM)
  list(APPEND SRCS am263x_pwm.c)
endif()

if(CONFIG_AM263X_MCAN)
  list(APPEND SRCS am263x_mcan.c)
endif()

if(CONFIG_AM263X_LIN)
  list(APPEND SRCS am263x_lin.c)
endif()

target_sources(arch PRIVATE ${SRCS})
