/****************************************************************************
 * include/nuttx/misc/slg46826.h
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

#ifndef __INCLUDE_NUTTX_MISC_SLG46826_H
#define __INCLUDE_NUTTX_MISC_SLG46826_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_SLG46826

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A design image: the 256 bytes of the NVM (and register) map */

#define SLG46826_IMAGE_SIZE     256

/* Default I2C address of the register block (control code 0001).  The
 * NVM and EEPROM blocks are at base + 2 and base + 3.
 */

#define SLG46826_ADDR_DEFAULT   0x08

/* ioctl commands.  The register block itself is read and written with
 * read(), write() and lseek() on the device, offsets 0-255.
 *
 * SLG46826IOC_RAMLOAD     Write the configuration part of a design
 *                         (registers 0x00-0xc7) into the live registers.
 *                         Lost at the next reset.
 *                         arg: FAR const uint8_t[SLG46826_IMAGE_SIZE]
 * SLG46826IOC_NVMREAD     Read the NVM block.
 *                         arg: FAR uint8_t[SLG46826_IMAGE_SIZE]
 * SLG46826IOC_NVMPROGRAM  Erase and write NVM pages 0-14 from a design,
 *                         reset, and verify.  Wears the NVM.
 *                         arg: FAR const uint8_t[SLG46826_IMAGE_SIZE]
 * SLG46826IOC_RESET       Soft reset: reload the registers from NVM.
 *                         arg: none
 * SLG46826IOC_NVMERASE    Erase NVM pages 0-14.  The registers keep the
 *                         old design until the next reset.  Wears the
 *                         NVM.
 *                         arg: none
 */

#define SLG46826IOC_RAMLOAD     _SLGIOC(1)
#define SLG46826IOC_NVMREAD     _SLGIOC(2)
#define SLG46826IOC_NVMPROGRAM  _SLGIOC(3)
#define SLG46826IOC_RESET       _SLGIOC(4)
#define SLG46826IOC_NVMERASE    _SLGIOC(5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* What slg46826_register() does with the board's design */

enum slg46826_boot_e
{
  SLG46826_BOOT_NONE = 0,     /* Leave the device as it is */
  SLG46826_BOOT_RAM,          /* RAM-load the design */
  SLG46826_BOOT_NVM_IF_BLANK  /* Program the NVM, only if it is blank */
};

struct slg46826_config_s
{
  uint8_t                addr;   /* Register block address */
  enum slg46826_boot_e   boot;
  FAR const uint8_t     *image;  /* SLG46826_IMAGE_SIZE bytes, or NULL */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: slg46826_register
 *
 * Description:
 *   Probe a Renesas GreenPAK SLG46826, bootstrap it as config->boot says,
 *   and register it as a character device.  A failed bootstrap is logged
 *   and the device is still registered.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure: -ENODEV if the
 *   device does not acknowledge its address.
 *
 ****************************************************************************/

int slg46826_register(FAR const char *devpath, FAR struct i2c_master_s *i2c,
                      FAR const struct slg46826_config_s *config);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SLG46826 */
#endif /* __INCLUDE_NUTTX_MISC_SLG46826_H */
