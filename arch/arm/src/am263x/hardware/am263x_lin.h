/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_lin.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_LIN_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_LIN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Five SCI/LIN modules, 4 KB apart */

#define AM263X_LIN_NINSTANCES        5
#define AM263X_LIN_BASE(n)           (AM263X_LIN0_BASE + ((n) << 12))

/* Interrupt line 0 of each module on R5FSS0-0 (line 1 is IRQ + 1) */

#define AM263X_IRQ_LIN(n)            (16 + 2 * (n))

/* Register offsets *********************************************************/

#define AM263X_LIN_SCIGCR0_OFFSET    0x0000 /* Global control 0 */
#define AM263X_LIN_SCIGCR1_OFFSET    0x0004 /* Global control 1 */
#define AM263X_LIN_SCIGCR2_OFFSET    0x0008 /* Global control 2 */
#define AM263X_LIN_SCISETINT_OFFSET  0x000c /* Interrupt enable set */
#define AM263X_LIN_SCICLRINT_OFFSET  0x0010 /* Interrupt enable clear */
#define AM263X_LIN_SCISETLVL_OFFSET  0x0014 /* Interrupt line 1 select */
#define AM263X_LIN_SCICLRLVL_OFFSET  0x0018 /* Interrupt line 0 select */
#define AM263X_LIN_SCIFLR_OFFSET     0x001c /* Flags, write 1 to clear */
#define AM263X_LIN_SCIFORMAT_OFFSET  0x0028 /* Character and frame length */
#define AM263X_LIN_BRSR_OFFSET       0x002c /* Baud rate */
#define AM263X_LIN_SCIPIO0_OFFSET    0x003c /* Pin function */
#define AM263X_LIN_LINCOMP_OFFSET    0x0060 /* Sync break and delimiter */
#define AM263X_LIN_LINRD0_OFFSET     0x0064 /* RX bytes 0-3 */
#define AM263X_LIN_LINRD1_OFFSET     0x0068 /* RX bytes 4-7 */
#define AM263X_LIN_LINMASK_OFFSET    0x006c /* ID match masks */
#define AM263X_LIN_LINID_OFFSET      0x0070 /* ID bytes */
#define AM263X_LIN_LINTD0_OFFSET     0x0074 /* TX bytes 0-3 */
#define AM263X_LIN_LINTD1_OFFSET     0x0078 /* TX bytes 4-7 */
#define AM263X_LIN_MBRSR_OFFSET      0x007c /* Maximum baud rate */
#define AM263X_LIN_IODFTCTRL_OFFSET  0x0090 /* Loopback and test */
#define AM263X_LIN_GLBINTEN_OFFSET   0x00e0 /* Interrupt line enables */
#define AM263X_LIN_GLBINTFLG_OFFSET  0x00e4
#define AM263X_LIN_GLBINTCLR_OFFSET  0x00e8

/* Register bits ************************************************************/

/* SCIGCR0 */

#define LIN_SCIGCR0_RESET            (1 << 0)  /* 1: module out of reset */

/* SCIGCR1 */

#define LIN_SCIGCR1_COMMMODE         (1 << 0)  /* 1: length from ID4/ID5 */
#define LIN_SCIGCR1_PARITYENA        (1 << 2)  /* ID parity */
#define LIN_SCIGCR1_CLK_MASTER       (1 << 5)  /* Commander */
#define LIN_SCIGCR1_LINMODE          (1 << 6)
#define LIN_SCIGCR1_SWNRST           (1 << 7)  /* 0: configuration state */
#define LIN_SCIGCR1_ADAPT            (1 << 9)  /* Automatic baud rate */
#define LIN_SCIGCR1_MBUFMODE         (1 << 10) /* Use the 8-byte buffers */
#define LIN_SCIGCR1_CTYPE            (1 << 11) /* 1: enhanced checksum */
#define LIN_SCIGCR1_HGENCTRL         (1 << 12) /* 1: match IDSLAVETASKBYTE */
#define LIN_SCIGCR1_LOOPBACK         (1 << 16) /* Internal loopback */
#define LIN_SCIGCR1_CONT             (1 << 17) /* Run on in debug halt */
#define LIN_SCIGCR1_RXENA            (1 << 24)
#define LIN_SCIGCR1_TXENA            (1 << 25)

/* SCISETINT, SCICLRINT, SCISETLVL, SCICLRLVL and SCIFLR */

#define LIN_INT_BRKDT                (1 << 0)
#define LIN_INT_WAKEUP               (1 << 1)
#define LIN_INT_TIMEOUT              (1 << 4)
#define LIN_INT_TOAWUS               (1 << 6)
#define LIN_INT_TOA3WUS              (1 << 7)
#define LIN_INT_TX                   (1 << 8)  /* TXRDY */
#define LIN_INT_RX                   (1 << 9)  /* RXRDY */
#define LIN_INT_ID                   (1 << 13)
#define LIN_INT_PE                   (1 << 24) /* ID parity error */
#define LIN_INT_OE                   (1 << 25) /* Overrun */
#define LIN_INT_FE                   (1 << 26) /* Framing error */
#define LIN_INT_NRE                  (1 << 27) /* No response */
#define LIN_INT_ISFE                 (1 << 28) /* Inconsistent sync field */
#define LIN_INT_CE                   (1 << 29) /* Checksum error */
#define LIN_INT_PBE                  (1 << 30) /* Physical bus error */

/* LIN_INT_BE: bit error */

#define LIN_INT_BE                   (1u << 31)
#define LIN_INT_ALL                  0xff0023d3

#define LIN_FLR_IDLE                 (1 << 2)
#define LIN_FLR_BUSY                 (1 << 3)
#define LIN_FLR_TXEMPTY              (1 << 11)
#define LIN_FLR_IDTXFLAG             (1 << 13)
#define LIN_FLR_IDRXFLAG             (1 << 14)

/* SCIFORMAT */

#define LIN_SCIFORMAT_CHAR_SHIFT     0         /* Bits per character - 1 */
#define LIN_SCIFORMAT_LENGTH_SHIFT   16        /* Response bytes - 1 */
#define LIN_SCIFORMAT_LENGTH_MASK    (7 << LIN_SCIFORMAT_LENGTH_SHIFT)

/* BRSR: baud = clock / (16 * (P + 1 + M / 16)) */

#define LIN_BRSR_P_SHIFT             0
#define LIN_BRSR_P_MASK              (0xffffff << LIN_BRSR_P_SHIFT)
#define LIN_BRSR_M_SHIFT             24
#define LIN_BRSR_M_MASK              (15 << LIN_BRSR_M_SHIFT)

/* SCIPIO0 */

#define LIN_SCIPIO0_RXFUNC           (1 << 1)
#define LIN_SCIPIO0_TXFUNC           (1 << 2)

/* LINCOMP */

#define LIN_LINCOMP_SBREAK_SHIFT     0         /* Break: 13 + SBREAK bits */
#define LIN_LINCOMP_SDEL_SHIFT       8         /* Delimiter: SDEL + 1 bits */

/* LINMASK: a 1 bit is left out of the ID compare */

#define LIN_LINMASK_TXIDMASK_SHIFT   0
#define LIN_LINMASK_RXIDMASK_SHIFT   16

/* LINID */

#define LIN_LINID_IDBYTE_SHIFT       0         /* Commander: sends a header */
#define LIN_LINID_IDBYTE_MASK        (0xff << LIN_LINID_IDBYTE_SHIFT)
#define LIN_LINID_IDSLAVE_SHIFT      8         /* ID to match */
#define LIN_LINID_IDSLAVE_MASK       (0xff << LIN_LINID_IDSLAVE_SHIFT)
#define LIN_LINID_RECEIVEDID_SHIFT   16
#define LIN_LINID_RECEIVEDID_MASK    (0xff << LIN_LINID_RECEIVEDID_SHIFT)

/* GLBINTEN, GLBINTFLG, GLBINTCLR */

#define LIN_GLBINT_LINE0             (1 << 0)
#define LIN_GLBINT_LINE1             (1 << 1)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_LIN_H */
