/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_qspi.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_QSPI_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_QSPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* QSPI0 registers and its memory-mapped flash window (32 MB, up to four
 * chip selects; CS0 is the one the ROM and the SBL boot from).
 */

#define AM263X_QSPI0_BASE              0x48200000
#define AM263X_QSPI_FLASH_SIZE         (32 * 1024 * 1024)

/* Register offsets.  SPI_SETUPn_REG is the memory-mapped setup for CSn. */

#define AM263X_QSPI_PID_OFFSET         0x0000
#define AM263X_QSPI_SYSCONFIG_OFFSET   0x0010
#define AM263X_QSPI_INTR_RAW_OFFSET    0x0020  /* INTR_STATUS_RAW_SET */
#define AM263X_QSPI_INTR_STAT_OFFSET   0x0024  /* INTR_STATUS_ENABLED_CLEAR */
#define AM263X_QSPI_INTR_ENSET_OFFSET  0x0028
#define AM263X_QSPI_INTR_ENCLR_OFFSET  0x002c
#define AM263X_QSPI_INTC_EOI_OFFSET    0x0030
#define AM263X_QSPI_CLKCTRL_OFFSET     0x0040  /* SPI_CLOCK_CNTRL_REG */
#define AM263X_QSPI_DC_OFFSET          0x0044  /* SPI_DC_REG */
#define AM263X_QSPI_CMD_OFFSET         0x0048  /* SPI_CMD_REG */
#define AM263X_QSPI_STATUS_OFFSET      0x004c  /* SPI_STATUS_REG */
#define AM263X_QSPI_DATA_OFFSET        0x0050  /* SPI_DATA_REG, bits 31:0 */
#define AM263X_QSPI_SETUP_OFFSET(n)    (0x0054 + 4 * (n))
#define AM263X_QSPI_SWITCH_OFFSET      0x0064  /* SPI_SWITCH_REG */
#define AM263X_QSPI_DATA1_OFFSET       0x0068  /* bits 63:32 */
#define AM263X_QSPI_DATA2_OFFSET       0x006c  /* bits 95:64 */
#define AM263X_QSPI_DATA3_OFFSET       0x0070  /* bits 127:96 */

/* SYSCONFIG */

#define QSPI_SYSCONFIG_IDLE_SHIFT      2
#define QSPI_SYSCONFIG_IDLE_MASK       (3 << QSPI_SYSCONFIG_IDLE_SHIFT)
#define QSPI_SYSCONFIG_IDLE_FORCE      (0 << QSPI_SYSCONFIG_IDLE_SHIFT)
#define QSPI_SYSCONFIG_IDLE_NONE       (1 << QSPI_SYSCONFIG_IDLE_SHIFT)
#define QSPI_SYSCONFIG_IDLE_SMART      (2 << QSPI_SYSCONFIG_IDLE_SHIFT)

/* Interrupt registers: frame and word complete */

#define QSPI_INTR_FIRQ                 (1 << 0)
#define QSPI_INTR_WIRQ                 (1 << 1)

/* SPI_CLOCK_CNTRL_REG: SCLK = functional clock / (DCLK_DIV + 1) */

#define QSPI_CLKCTRL_DIV_SHIFT         0
#define QSPI_CLKCTRL_DIV_MASK          (0xffff << QSPI_CLKCTRL_DIV_SHIFT)
#define QSPI_CLKCTRL_DIV(n) \
  ((uint32_t)(n) << QSPI_CLKCTRL_DIV_SHIFT)
#define QSPI_CLKCTRL_CLKEN             (1u << 31)

/* SPI_DC_REG: one byte per chip select */

#define QSPI_DC_SHIFT(cs)              (8 * (cs))
#define QSPI_DC_CKP                    (1 << 0) /* clock idles high (CPOL) */
#define QSPI_DC_CSP                    (1 << 1) /* CS active high */
#define QSPI_DC_CKPH                   (1 << 2) /* CPHA */
#define QSPI_DC_DD_SHIFT               3        /* CS to data delay, cycles */
#define QSPI_DC_DD_MASK                (3 << QSPI_DC_DD_SHIFT)
#define QSPI_DC_MASK                   0x1f

/* SPI_CMD_REG.  Writing it shifts one word; CS stays asserted until the
 * FLEN words of the frame have gone.
 */

#define QSPI_CMD_FLEN_SHIFT            0        /* words in frame - 1 */
#define QSPI_CMD_FLEN_MASK             (0xfff << QSPI_CMD_FLEN_SHIFT)
#define QSPI_CMD_FLEN(n) \
  ((uint32_t)((n) - 1) << QSPI_CMD_FLEN_SHIFT)
#define QSPI_CMD_FLEN_MAX              4096
#define QSPI_CMD_WIRQ                  (1 << 14)
#define QSPI_CMD_FIRQ                  (1 << 15)
#define QSPI_CMD_CMD_SHIFT             16
#define QSPI_CMD_CMD_MASK              (7 << QSPI_CMD_CMD_SHIFT)
#define QSPI_CMD_READ_SINGLE           (1 << QSPI_CMD_CMD_SHIFT) /* 4-pin */
#define QSPI_CMD_WRITE_SINGLE          (2 << QSPI_CMD_CMD_SHIFT) /* 4-pin */
#define QSPI_CMD_READ_DUAL             (3 << QSPI_CMD_CMD_SHIFT)
#define QSPI_CMD_READ_QUAD             (7 << QSPI_CMD_CMD_SHIFT) /* 6-pin */
#define QSPI_CMD_WLEN_SHIFT            19                        /* bits - 1 */
#define QSPI_CMD_WLEN_MASK             (0x7f << QSPI_CMD_WLEN_SHIFT)
#define QSPI_CMD_WLEN(n) \
  ((uint32_t)((n) - 1) << QSPI_CMD_WLEN_SHIFT)
#define QSPI_CMD_CSNUM_SHIFT           28
#define QSPI_CMD_CSNUM(cs) \
  ((uint32_t)(cs) << QSPI_CMD_CSNUM_SHIFT)

/* SPI_STATUS_REG */

#define QSPI_STATUS_BUSY               (1 << 0)
#define QSPI_STATUS_WC                 (1 << 1) /* word complete */
#define QSPI_STATUS_FC                 (1 << 2) /* frame complete */
#define QSPI_STATUS_WDCNT_SHIFT        16       /* words done in frame */
#define QSPI_STATUS_WDCNT_MASK         (0xfff << QSPI_STATUS_WDCNT_SHIFT)

/* SPI_SETUPn_REG: the memory-mapped read (and write) command for CSn */

#define QSPI_SETUP_RCMD_SHIFT          0
#define QSPI_SETUP_RCMD_MASK           (0xff << QSPI_SETUP_RCMD_SHIFT)
#define QSPI_SETUP_RCMD(c) \
  ((uint32_t)(c) << QSPI_SETUP_RCMD_SHIFT)
#define QSPI_SETUP_NUM_A_BYTES_SHIFT   8        /* address bytes - 1 */
#define QSPI_SETUP_NUM_A_BYTES(n) \
  ((uint32_t)((n) - 1) << QSPI_SETUP_NUM_A_BYTES_SHIFT)
#define QSPI_SETUP_NUM_D_BYTES_SHIFT   10       /* dummy bytes, 0-3 */
#define QSPI_SETUP_NUM_D_BYTES(n) \
  ((uint32_t)(n) << QSPI_SETUP_NUM_D_BYTES_SHIFT)
#define QSPI_SETUP_READ_TYPE_SHIFT     12
#define QSPI_SETUP_READ_NORMAL         (0 << QSPI_SETUP_READ_TYPE_SHIFT)
#define QSPI_SETUP_READ_DUAL           (1 << QSPI_SETUP_READ_TYPE_SHIFT)
#define QSPI_SETUP_READ_QUAD           (3 << QSPI_SETUP_READ_TYPE_SHIFT)
#define QSPI_SETUP_WCMD_SHIFT          16
#define QSPI_SETUP_WCMD(c) \
  ((uint32_t)(c) << QSPI_SETUP_WCMD_SHIFT)
#define QSPI_SETUP_NUM_D_BITS_SHIFT    24       /* dummy bits if no bytes */
#define QSPI_SETUP_NUM_D_BITS(n) \
  ((uint32_t)(n) << QSPI_SETUP_NUM_D_BITS_SHIFT)

/* SPI_SWITCH_REG */

#define QSPI_SWITCH_MMPT_S             (1 << 0) /* 1: memory-mapped port */
#define QSPI_SWITCH_MM_INT_EN          (1 << 1)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_QSPI_H */
