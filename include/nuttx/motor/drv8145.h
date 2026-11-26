/****************************************************************************
 * include/nuttx/motor/drv8145.h
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

#ifndef __INCLUDE_NUTTX_MOTOR_DRV8145_H
#define __INCLUDE_NUTTX_MOTOR_DRV8145_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <fixedmath.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/spi/spi.h>

#ifdef CONFIG_MOTOR_DRV8145

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The most devices one chain can hold (HDR1 has six bits for the count) */

#define DRV8145_MAX_CHAIN           63

/* Registers (SLVSG22B Table 8-1) */

#define DRV8145_DEVICE_ID           0x00  /* R */
#define DRV8145_FAULT_SUMMARY       0x01  /* R, reset 0x40 */
#define DRV8145_STATUS1             0x02  /* R, reset 0x00 */
#define DRV8145_STATUS2             0x03  /* R, reset 0x80 */
#define DRV8145_COMMAND             0x08  /* R/W, reset 0x09 */
#define DRV8145_SPI_IN              0x09  /* R/W, reset 0x0c */
#define DRV8145_CONFIG1             0x0a  /* R/W, reset 0x10 */
#define DRV8145_CONFIG2             0x0b  /* R/W, reset 0x00 */
#define DRV8145_CONFIG3             0x0c  /* R/W, reset 0x40 */
#define DRV8145_CONFIG4             0x0d  /* R/W, reset 0x04 */

/* DEVICE_ID: DEV_ID in bits 7:2, REV_ID in bits 1:0 */

#define DRV8145_ID_DEV_MASK         0xfc
#define DRV8145_ID_REV_MASK         0x03
#define DRV8145_ID_DRV8143S         0xb8
#define DRV8145_ID_DRV8144S         0xc8
#define DRV8145_ID_DRV8145S         0xd8
#define DRV8145_ID_DRV8143P         0xbc
#define DRV8145_ID_DRV8145P         0xdc

/* FAULT_SUMMARY */

#define DRV8145_FSUM_SPI_ERR        (1 << 7)
#define DRV8145_FSUM_POR            (1 << 6)
#define DRV8145_FSUM_FAULT          (1 << 5)  /* OR of all the others */
#define DRV8145_FSUM_VMOV           (1 << 4)
#define DRV8145_FSUM_VMUV           (1 << 3)
#define DRV8145_FSUM_OCP            (1 << 2)
#define DRV8145_FSUM_TSD            (1 << 1)
#define DRV8145_FSUM_VCPUV          (1 << 0)

/* The status byte returned in every frame: bits 7:6 always 11, then
 * FAULT_SUMMARY bits 5:1, and SPI_ERR in place of VCPUV.
 */

#define DRV8145_STAT_ONES           0xc0
#define DRV8145_STAT_FAULT          (1 << 5)
#define DRV8145_STAT_VMOV           (1 << 4)
#define DRV8145_STAT_VMUV           (1 << 3)
#define DRV8145_STAT_OCP            (1 << 2)
#define DRV8145_STAT_TSD            (1 << 1)
#define DRV8145_STAT_SPI_ERR        (1 << 0)

/* STATUS1 */

#define DRV8145_STATUS1_OLA         (3 << 6)  /* Either bit: open load */
#define DRV8145_STATUS1_ITRIP_CMP   (1 << 5)  /* Current reached ITRIP */
#define DRV8145_STATUS1_ACTIVE      (1 << 4)
#define DRV8145_STATUS1_OCP_H       0x0a      /* Either bit: HS FET OCP */
#define DRV8145_STATUS1_OCP_L       0x05      /* Either bit: LS FET OCP */

/* STATUS2 */

#define DRV8145_STATUS2_DRVOFF_STAT (1 << 7)  /* DRVOFF pin level */
#define DRV8145_STATUS2_ACTIVE      (1 << 4)
#define DRV8145_STATUS2_OLP_CMP     (1 << 0)  /* Off-state comparator */

/* COMMAND.  SPI_IN_LOCK unlocks only with 10b; REG_LOCK locks only with
 * 10b.  At reset SPI_IN is locked and the CONFIG registers are unlocked.
 */

#define DRV8145_CMD_CLR_FLT         (1 << 7)
#define DRV8145_CMD_SPI_IN_MASK     (3 << 3)
#define DRV8145_CMD_SPI_IN_UNLOCK   (2 << 3)
#define DRV8145_CMD_SPI_IN_LOCK     (1 << 3)
#define DRV8145_CMD_REG_MASK        (3 << 0)
#define DRV8145_CMD_REG_LOCK        (2 << 0)
#define DRV8145_CMD_REG_UNLOCK      (1 << 0)

/* SPI_IN: register equivalents of the DRVOFF and IN pins, combined with
 * the pins as CONFIG4 DRVOFF_SEL and IN_SEL say, while SPI_IN is
 * unlocked.
 */

#define DRV8145_SPI_IN_S_DRVOFF     (1 << 3)
#define DRV8145_SPI_IN_S_IN         (1 << 0)

/* CONFIG1 */

#define DRV8145_CFG1_EN_OLA         (1 << 7)  /* Open load in ACTIVE */
#define DRV8145_CFG1_VMOV_SHIFT     5
#define DRV8145_CFG1_VMOV_MASK      (3 << 5)
#define DRV8145_CFG1_VMOV_35V       (0 << 5)
#define DRV8145_CFG1_VMOV_28V       (1 << 5)
#define DRV8145_CFG1_VMOV_18V       (2 << 5)
#define DRV8145_CFG1_VMOV_OFF       (3 << 5)
#define DRV8145_CFG1_SSC_DIS        (1 << 4)  /* Spread spectrum off */
#define DRV8145_CFG1_OCP_RETRY      (1 << 3)  /* Retry, else latch */
#define DRV8145_CFG1_TSD_RETRY      (1 << 2)
#define DRV8145_CFG1_VMOV_RETRY     (1 << 1)
#define DRV8145_CFG1_OLA_RETRY      (1 << 0)

/* CONFIG2.  S_DIAG gives the load type and turns off-state diagnostics
 * (OLP) on or off.  A load to GND (driven by the high side) keeps
 * IPROPI and ITRIP; a load to VM (driven by the low side) keeps active
 * open load detection (OLA) instead.
 */

#define DRV8145_CFG2_SPARE          (1 << 7)
#define DRV8145_CFG2_DIAG_SHIFT     5
#define DRV8145_CFG2_DIAG_MASK      (3 << 5)
#define DRV8145_CFG2_DIAG_GND       (0 << 5)  /* Load to GND, no OLP */
#define DRV8145_CFG2_DIAG_GND_OLP   (1 << 5)  /* Load to GND, OLP */
#define DRV8145_CFG2_DIAG_VM        (2 << 5)  /* Load to VM, no OLP */
#define DRV8145_CFG2_DIAG_VM_OLP    (3 << 5)  /* Load to VM, OLP */
#define DRV8145_CFG2_ITRIP_SHIFT    0
#define DRV8145_CFG2_ITRIP_MASK     (7 << 0)  /* 0 off, else Table 7-5 */

/* CONFIG3 */

#define DRV8145_CFG3_TOFF_SHIFT     6
#define DRV8145_CFG3_TOFF_MASK      (3 << 6)  /* 20, 30, 40, 50 us */
#define DRV8145_CFG3_SR_SHIFT       2
#define DRV8145_CFG3_SR_MASK        (7 << 2)  /* Slew rate, 0 slowest */
#define DRV8145_CFG3_SPARE_MASK     (3 << 0)

/* CONFIG4 */

#define DRV8145_CFG4_TOCP_SHIFT     6
#define DRV8145_CFG4_TOCP_MASK      (3 << 6)  /* 6, 3, 1.5, 0.2 us */
#define DRV8145_CFG4_OCP_SHIFT      3
#define DRV8145_CFG4_OCP_MASK       (3 << 3)
#define DRV8145_CFG4_OCP_100        (0 << 3)  /* 128 A typical */
#define DRV8145_CFG4_OCP_50         (1 << 3)  /* 68 A typical */
#define DRV8145_CFG4_OCP_75         (2 << 3)  /* 98 A typical */
#define DRV8145_CFG4_DRVOFF_AND     (1 << 2)  /* Else OR */
#define DRV8145_CFG4_SPARE          (1 << 1)
#define DRV8145_CFG4_IN_AND         (1 << 0)  /* Else OR */

/* The settings of the CONFIG1-4 fields, as struct drv8145_params_s takes
 * them (each the field's register code), and what they mean.
 */

#define DRV8145_SR_COUNT            8   /* Slew rate, CONFIG3 S_SR */
#define DRV8145_ITRIP_OFF           0   /* ITRIP regulation, CONFIG2 */
#define DRV8145_ITRIP_COUNT         8
#define DRV8145_TOFF_20US           0   /* ITRIP off time, CONFIG3 */
#define DRV8145_TOFF_30US           1
#define DRV8145_TOFF_40US           2
#define DRV8145_TOFF_50US           3
#define DRV8145_TOCP_6US            0   /* OCP filter, CONFIG4 */
#define DRV8145_TOCP_3US            1
#define DRV8145_TOCP_1_5US          2
#define DRV8145_TOCP_MIN            3   /* About 0.2 us */
#define DRV8145_OCP_100             0   /* OCP threshold, CONFIG4 */
#define DRV8145_OCP_50              1   /* (3 is 50 % too) */
#define DRV8145_OCP_75              2
#define DRV8145_VMOV_35V            0   /* VM overvoltage, CONFIG1 */
#define DRV8145_VMOV_28V            1
#define DRV8145_VMOV_18V            2
#define DRV8145_VMOV_OFF            3
#define DRV8145_LOAD_GND            0   /* Load type, CONFIG2 S_DIAG */
#define DRV8145_LOAD_VM             1

/* The same in units, indexed by the codes above (SLVSG22B 6.5.6, Table
 * 7-5 and 8.1.7-8.1.10), as initializers for userspace tables.  Slew
 * rate: output rise time with high-side recirculation at VM = 13.5 V,
 * typical, in 0.1 V/us; ITRIP: V on IPROPI in mV (0: regulation off);
 * TOFF and TOCP in ns; OCP in % of the full threshold; VMOV in V (0: off).
 */

#define DRV8145_SR_X10_VUS          { 15, 47, 96, 144, 206, 265, 379, 471 }
#define DRV8145_ITRIP_MV            { 0, 1180, 1410, 1650, 1980, 2310, \
                                      2640, 2970 }
#define DRV8145_TOFF_NS             { 20000, 30000, 40000, 50000 }
#define DRV8145_TOCP_NS             { 6000, 3000, 1500, 200 }
#define DRV8145_OCP_PERCENT         { 100, 50, 75, 50 }
#define DRV8145_VMOV_V              { 35, 28, 18, 0 }

/* Register reset values */

#define DRV8145_CONFIG1_RESET       0x10
#define DRV8145_CONFIG2_RESET       0x00
#define DRV8145_CONFIG3_RESET       0x40
#define DRV8145_CONFIG4_RESET       0x04

/* ioctl commands.  Each device node is one half-bridge of the chain.
 *
 * DRV8145IOC_GETSTATUS   Read the device's status registers.
 *                        arg: FAR struct drv8145_status_s *
 * DRV8145IOC_CLEARFAULTS Clear the faults of every device in the chain
 *                        (global CLR_FLT).  The outputs keep their
 *                        settings, except that a latched fault turned
 *                        the bridge off and it stays off.
 *                        arg: none
 * DRV8145IOC_GETCONFIG   Read CONFIG1-4.
 *                        arg: FAR struct drv8145_config_s *
 * DRV8145IOC_SETCONFIG   Write CONFIG1-4, read them back and lock them.
 *                        arg: FAR const struct drv8145_config_s *
 * DRV8145IOC_SETOUTPUT   Drive the output: Hi-Z, low, high, or PWM on
 *                        the IN pin.
 *                        arg: FAR const struct drv8145_output_s *
 * DRV8145IOC_GETOUTPUT   The output setting last made.
 *                        arg: FAR struct drv8145_output_s *
 * DRV8145IOC_OLP         Run off-state (OLP) diagnostics: the output
 *                        must be Hi-Z.
 *                        arg: FAR struct drv8145_olp_s *
 * DRV8145IOC_READREG     Read a register.
 *                        arg: FAR struct drv8145_reg_s *
 * DRV8145IOC_WRITEREG    Write a register.  The driver does not track
 *                        what a raw write changes.
 *                        arg: FAR const struct drv8145_reg_s *
 * DRV8145IOC_SLEEP       Put the whole chain to sleep (arg 1) or wake
 *                        it and set it up again (arg 0).  -ENOTSUP if
 *                        the board has no nSLEEP control.
 *                        arg: int
 * DRV8145IOC_REINIT      Set up the whole chain again as at
 *                        registration: every output goes Hi-Z.
 *                        arg: none
 * DRV8145IOC_GETPARAMS   CONFIG1-4 as named settings, read from the
 *                        device.
 *                        arg: FAR struct drv8145_params_s *
 * DRV8145IOC_SETPARAMS   Check and write named settings, as SETCONFIG
 *                        does: they last until changed, also through a
 *                        set-up after a power-on reset.  A setting out
 *                        of range is -EINVAL and nothing is written.
 *                        Settings can be changed with the output on.
 *                        arg: FAR const struct drv8145_params_s *
 * DRV8145IOC_DRVOFFTEST  Check the board's DRVOFF pin reaches the
 *                        device: drive it low and high and read
 *                        STATUS2 DRVOFF_STAT each time, while S_DRVOFF
 *                        keeps the bridge off.  The output must be Hi-Z
 *                        and DRVOFF_SEL OR.
 *                        arg: FAR struct drv8145_drvofftest_s *
 */

#define DRV8145IOC_GETSTATUS        _DRV8145IOC(1)
#define DRV8145IOC_CLEARFAULTS      _DRV8145IOC(2)
#define DRV8145IOC_GETCONFIG        _DRV8145IOC(3)
#define DRV8145IOC_SETCONFIG        _DRV8145IOC(4)
#define DRV8145IOC_SETOUTPUT        _DRV8145IOC(5)
#define DRV8145IOC_GETOUTPUT        _DRV8145IOC(6)
#define DRV8145IOC_OLP              _DRV8145IOC(7)
#define DRV8145IOC_READREG          _DRV8145IOC(8)
#define DRV8145IOC_WRITEREG         _DRV8145IOC(9)
#define DRV8145IOC_SLEEP            _DRV8145IOC(10)
#define DRV8145IOC_REINIT           _DRV8145IOC(11)
#define DRV8145IOC_DRVOFFTEST       _DRV8145IOC(12)
#define DRV8145IOC_GETPARAMS        _DRV8145IOC(13)
#define DRV8145IOC_SETPARAMS        _DRV8145IOC(14)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* CONFIG1-4, as the DRV8145_CFGn_* bits */

struct drv8145_config_s
{
  uint8_t config1;
  uint8_t config2;
  uint8_t config3;
  uint8_t config4;
};

/* CONFIG1-4 as named settings (the codes above; booleans 0 or 1) */

struct drv8145_params_s
{
  uint8_t slew;             /* DRV8145_SR code, 0 (1.5 V/us) - 7 */
  uint8_t itrip;            /* DRV8145_ITRIP code, 0 off, 1-7 */
  uint8_t toff;             /* DRV8145_TOFF_* */
  uint8_t ocp;              /* DRV8145_OCP_* */
  uint8_t tocp;             /* DRV8145_TOCP_* */
  uint8_t vmov;             /* DRV8145_VMOV_* */
  uint8_t load;             /* DRV8145_LOAD_* */
  uint8_t olp;              /* Off-state diagnostics allowed */
  uint8_t ola;              /* Open load detection in ACTIVE */
  uint8_t ssc;              /* Spread spectrum clocking */
  uint8_t ocp_retry;        /* Retry after the fault, else latch */
  uint8_t tsd_retry;
  uint8_t vmov_retry;       /* Also VM undervoltage */
  uint8_t ola_retry;
  uint8_t drvoff_and;       /* DRVOFF pin AND S_DRVOFF, else OR */
  uint8_t in_and;           /* IN pin AND S_IN, else OR */
};

/* Output modes */

enum drv8145_mode_e
{
  DRV8145_OUT_HIZ = 0,      /* Both FETs off (STANDBY) */
  DRV8145_OUT_LOW,          /* Low-side FET on */
  DRV8145_OUT_HIGH,         /* High-side FET on */
  DRV8145_OUT_PWM           /* IN pin driven by the board's PWM */
};

struct drv8145_output_s
{
  uint8_t  mode;            /* enum drv8145_mode_e */
  uint32_t frequency;       /* DRV8145_OUT_PWM: Hz, up to 125 kHz */
  ub16_t   duty;            /* DRV8145_OUT_PWM: high-side on time,
                             * 0 to b16ONE */
};

struct drv8145_status_s
{
  uint8_t  devid;           /* DEVICE_ID */
  uint8_t  status;          /* Status byte of the last frame */
  uint8_t  fault_summary;   /* FAULT_SUMMARY */
  uint8_t  status1;         /* STATUS1 */
  uint8_t  status2;         /* STATUS2 */
  uint8_t  command;         /* COMMAND: the lock states */
  uint8_t  spi_in;          /* SPI_IN */
  int8_t   nfault;          /* nFAULT pin: 1 asserted (low), 0 not, -1
                             * no pin; shared by the chain */
  uint32_t por_count;       /* Power-on resets seen since registration */
  uint32_t frame_errors;    /* Frames that came back corrupted */
};

/* Off-state diagnostics: the comparator with the output pulled up to
 * VOLP_REFH and pulled down to VOLP_REFL, and what they mean for the load
 * type in CONFIG2 S_DIAG (SLVSG22B Tables 7-8 and 7-9).
 */

enum drv8145_olp_e
{
  DRV8145_OLP_NORMAL = 0,   /* Load present */
  DRV8145_OLP_OPEN,         /* Open load */
  DRV8145_OLP_SHORT,        /* Output shorted to the other rail */
  DRV8145_OLP_UNKNOWN       /* A combination the tables do not list */
};

struct drv8145_olp_s
{
  uint8_t cmp_pu;           /* OLP_CMP with ROLP_PU and VOLP_REFH */
  uint8_t cmp_pd;           /* OLP_CMP with ROLP_PD and VOLP_REFL */
  uint8_t result;           /* enum drv8145_olp_e */
};

struct drv8145_reg_s
{
  uint8_t addr;
  uint8_t value;
};

/* DRVOFF_STAT read with the DRVOFF pin driven low and high: 0 and 1 if
 * the pin reaches the device.
 */

struct drv8145_drvofftest_s
{
  uint8_t stat_low;
  uint8_t stat_high;
};

/* What the board provides.  Device 0 is the first in the chain: its SDI
 * is the controller's MOSI.  Every callback is optional; without one the
 * driver controls that function through SPI_IN alone, or (nSLEEP,
 * nFAULT) not at all.
 */

struct drv8145_board_s
{
  uint8_t ndev;             /* Devices in the chain, 1-63 */
  uint8_t minor;            /* Device n is /dev/hb<minor + n> */

  /* CONFIG1-4 for each device at setup, or NULL for the driver's
   * defaults (reset values, with DRVOFF_SEL OR so that S_DRVOFF alone
   * turns the bridge off).
   */

  FAR const struct drv8145_config_s *config;

  /* nSLEEP of the whole chain: true for sleep */

  CODE int (*sleep)(FAR const struct drv8145_board_s *board, bool sleep);

  /* DRVOFF pin of device dev: true for off */

  CODE int (*drvoff)(FAR const struct drv8145_board_s *board, int dev,
                     bool off);

  /* IN pin of device dev: a PWM at frequency with duty high; duty 0 or
   * frequency 0 holds the pin low and b16ONE holds it high.
   */

  CODE int (*in)(FAR const struct drv8145_board_s *board, int dev,
                 uint32_t frequency, ub16_t duty);

  /* The chain's nFAULT: 1 asserted (low), 0 not */

  CODE int (*nfault)(FAR const struct drv8145_board_s *board);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: drv8145_register
 *
 * Description:
 *   Register a daisy chain of TI DRV8145-Q1 half-bridge drivers (or
 *   DRV8143/DRV8144, which share the register map) on one chip select
 *   as /dev/hb<minor> to /dev/hb<minor + ndev - 1>, and set the chain up:
 *   acknowledge the power-up, check the device IDs, write and lock the
 *   configuration, unlock SPI_IN and leave every output Hi-Z.
 *
 *   If the chain does not answer (with no VM the devices are unpowered),
 *   the devices are still registered and the set-up is retried when one
 *   is opened.
 *
 * Input Parameters:
 *   spi   - The SPI bus
 *   devid - The chain's chip select
 *   board - The board's description; kept, not copied
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int drv8145_register(FAR struct spi_dev_s *spi, uint32_t devid,
                     FAR const struct drv8145_board_s *board);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MOTOR_DRV8145 */
#endif /* __INCLUDE_NUTTX_MOTOR_DRV8145_H */
