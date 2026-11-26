/****************************************************************************
 * include/nuttx/leds/tpic2810.h
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

#ifndef __INCLUDE_NUTTX_LEDS_TPIC2810_H
#define __INCLUDE_NUTTX_LEDS_TPIC2810_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_TPIC2810

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The 7-bit I2C address is 1100 A2 A1 A0 */

#define TPIC2810_ADDR(a)        (0x60 | ((a) & 7))

/* Command bytes: the data byte follows */

#define TPIC2810_CMD_SHIFT      0x11  /* Write the shift register only */
#define TPIC2810_CMD_LOAD       0x22  /* Shift register to the outputs */
#define TPIC2810_CMD_IMMEDIATE  0x44  /* Write straight to the outputs */

/* The read-back: TPIC2810_CMD_READ alone, then a repeated START and a
 * one-byte read of the outputs.  The outputs do not change.
 */

#define TPIC2810_CMD_READ       TPIC2810_CMD_SHIFT

#define TPIC2810_NOUTPUTS       8

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s;

/****************************************************************************
 * Name: tpic2810_register
 *
 * Description:
 *   Turn all eight outputs of a TPIC2810 off and register it as a user LED
 *   device (include/nuttx/leds/userled.h), LED n being output Dn.  A bit
 *   set in a userled set turns the output on: it sinks current and the
 *   LED lights.
 *
 * Input Parameters:
 *   devpath - The device to register, e.g. "/dev/userleds1"
 *   i2c     - The I2C bus the TPIC2810 is on
 *   addr    - Its 7-bit address, TPIC2810_ADDR(A2..A0)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure, -ENODEV among
 *   them if the device does not answer.
 *
 ****************************************************************************/

int tpic2810_register(FAR const char *devpath,
                      FAR struct i2c_master_s *i2c, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_TPIC2810 */
#endif /* __INCLUDE_NUTTX_LEDS_TPIC2810_H */
