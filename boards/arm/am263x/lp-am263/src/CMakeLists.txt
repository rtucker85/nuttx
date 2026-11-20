# ##############################################################################
# boards/arm/am263x/lp-am263/src/CMakeLists.txt
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

set(SRCS am263x_boardinit.c am263x_bringup.c)

if(CONFIG_AM263X_MCSPI)
  list(APPEND SRCS am263x_spi.c)
endif()

if(CONFIG_AM263X_I2C1)
  list(APPEND SRCS am263x_eeprom.c)
endif()

if(CONFIG_AM263X_MDIO)
  list(APPEND SRCS am263x_ethernet.c)
endif()

if(CONFIG_AM263X_ADC1 OR CONFIG_AM263X_EPWM)
  list(APPEND SRCS am263x_analog.c)
endif()

if(CONFIG_BOARDCTL)
  list(APPEND SRCS am263x_appinit.c)
endif()

if(CONFIG_BOARDCTL_RESET)
  list(APPEND SRCS am263x_reset.c)
endif()

target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ocram.ld")
