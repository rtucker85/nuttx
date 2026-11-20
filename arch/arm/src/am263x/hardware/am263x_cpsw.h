/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_cpsw.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CPSW_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CPSW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CPSW3G: a three-port switch (host port 0, MAC ports 1 and 2).  Its
 * functional clock (CPPI_ICLK) is SYS_CLK, 200 MHz.
 */

#define AM263X_CPSW_CLK_HZ            200000000

/* Sub-module offsets from AM263X_CPSW_BASE */

#define AM263X_CPSW_SS_OFFSET         0x00000000 /* Subsystem wrapper */
#define AM263X_CPSW_MDIO_OFFSET       0x00000f00
#define AM263X_CPSW_NU_OFFSET         0x00020000 /* Switch core */
#define AM263X_CPSW_CPDMA_OFFSET      0x00034000
#define AM263X_CPSW_ALE_OFFSET        0x0003e000

#define AM263X_CPSW_SS_BASE \
  (AM263X_CPSW_BASE + AM263X_CPSW_SS_OFFSET)
#define AM263X_MDIO_BASE \
  (AM263X_CPSW_BASE + AM263X_CPSW_MDIO_OFFSET)
#define AM263X_CPSW_NU_BASE \
  (AM263X_CPSW_BASE + AM263X_CPSW_NU_OFFSET)
#define AM263X_CPDMA_BASE \
  (AM263X_CPSW_BASE + AM263X_CPSW_CPDMA_OFFSET)
#define AM263X_ALE_BASE \
  (AM263X_CPSW_BASE + AM263X_CPSW_ALE_OFFSET)

/* Subsystem wrapper: interrupt enables and status per core interrupt
 * group.  R5FSS0-0 uses group 0.
 */

#define AM263X_CPSW_SS_IDVER          (AM263X_CPSW_SS_BASE + 0x0000)
#define AM263X_CPSW_SS_RGMII_STATUS(p) \
  (AM263X_CPSW_SS_BASE + 0x0030 + 4 * ((p) - 1))
#define AM263X_CPSW_SS_C_RX_EN(c) \
  (AM263X_CPSW_SS_BASE + 0x1804 + 0x40 * (c))
#define AM263X_CPSW_SS_C_TX_EN(c) \
  (AM263X_CPSW_SS_BASE + 0x1808 + 0x40 * (c))
#define AM263X_CPSW_SS_C_MISC_EN(c) \
  (AM263X_CPSW_SS_BASE + 0x180c + 0x40 * (c))
#define AM263X_CPSW_SS_C_RX_STAT(c) \
  (AM263X_CPSW_SS_BASE + 0x1814 + 0x40 * (c))
#define AM263X_CPSW_SS_C_TX_STAT(c) \
  (AM263X_CPSW_SS_BASE + 0x1818 + 0x40 * (c))

/* RGMIIn_STATUS: the in-band status the PHY sends */

#define CPSW_RGMII_STATUS_LINK        (1 << 0)
#define CPSW_RGMII_STATUS_SPEED_SHIFT 1         /* 0: 10, 1: 100, 2: 1000 */
#define CPSW_RGMII_STATUS_SPEED_MASK  (3 << CPSW_RGMII_STATUS_SPEED_SHIFT)
#define CPSW_RGMII_STATUS_FULLDUPLEX  (1 << 3)

/* Switch core.  Port 0 is the host port; MAC ports 1 and 2 are the
 * Ethernet ports.
 */

#define AM263X_CPSW_IDVER             (AM263X_CPSW_NU_BASE + 0x0000)
#define AM263X_CPSW_CONTROL           (AM263X_CPSW_NU_BASE + 0x0004)
#define AM263X_CPSW_STAT_PORT_EN      (AM263X_CPSW_NU_BASE + 0x0014)
#define AM263X_CPSW_P0_CONTROL        (AM263X_CPSW_NU_BASE + 0x1004)
#define AM263X_CPSW_P0_RX_MAXLEN      (AM263X_CPSW_NU_BASE + 0x1024)

#define AM263X_CPSW_PN_BASE(p) \
  (AM263X_CPSW_NU_BASE + 0x2000 + 0x1000 * ((p) - 1))
#define AM263X_CPSW_PN_CONTROL(p)     (AM263X_CPSW_PN_BASE(p) + 0x0004)
#define AM263X_CPSW_PN_RX_MAXLEN(p)   (AM263X_CPSW_PN_BASE(p) + 0x0024)
#define AM263X_CPSW_PN_SA_L(p)        (AM263X_CPSW_PN_BASE(p) + 0x0308)
#define AM263X_CPSW_PN_SA_H(p)        (AM263X_CPSW_PN_BASE(p) + 0x030c)
#define AM263X_CPSW_PN_MAC_CONTROL(p) (AM263X_CPSW_PN_BASE(p) + 0x0330)
#define AM263X_CPSW_PN_MAC_STATUS(p)  (AM263X_CPSW_PN_BASE(p) + 0x0334)
#define AM263X_CPSW_PN_MAC_SOFT_RESET(p) \
  (AM263X_CPSW_PN_BASE(p) + 0x0338)

/* CPSW CONTROL */

#define CPSW_CONTROL_VLAN_AWARE       (1 << 1)
#define CPSW_CONTROL_P0_ENABLE        (1 << 2)
#define CPSW_CONTROL_P0_TX_CRC_REMOVE (1 << 13) /* Strip FCS towards host */
#define CPSW_CONTROL_P0_RX_PAD        (1 << 14) /* Pad short host frames */

/* PN_MAC_CONTROL */

#define CPSW_MAC_CONTROL_FULLDUPLEX   (1 << 0)
#define CPSW_MAC_CONTROL_LOOPBACK     (1 << 1)
#define CPSW_MAC_CONTROL_RX_FLOW_EN   (1 << 3)
#define CPSW_MAC_CONTROL_TX_FLOW_EN   (1 << 4)
#define CPSW_MAC_CONTROL_GMII_EN      (1 << 5)  /* MAC transmit/receive on */
#define CPSW_MAC_CONTROL_GIG          (1 << 7)  /* 1000 Mbps */
#define CPSW_MAC_CONTROL_CMD_IDLE     (1 << 11)
#define CPSW_MAC_CONTROL_IFCTL_A      (1 << 15) /* RMII: 100 Mbps */
#define CPSW_MAC_CONTROL_EXT_EN       (1 << 18) /* RGMII in-band speed */

/* PN_MAC_STATUS */

#define CPSW_MAC_STATUS_IDLE          (1 << 31)

/* CPDMA */

#define AM263X_CPDMA_TX_CONTROL       (AM263X_CPDMA_BASE + 0x0004)
#define AM263X_CPDMA_TX_TEARDOWN      (AM263X_CPDMA_BASE + 0x0008)
#define AM263X_CPDMA_RX_CONTROL       (AM263X_CPDMA_BASE + 0x0014)
#define AM263X_CPDMA_RX_TEARDOWN      (AM263X_CPDMA_BASE + 0x0018)
#define AM263X_CPDMA_SOFT_RESET       (AM263X_CPDMA_BASE + 0x001c)
#define AM263X_CPDMA_DMACONTROL       (AM263X_CPDMA_BASE + 0x0020)
#define AM263X_CPDMA_DMASTATUS        (AM263X_CPDMA_BASE + 0x0024)
#define AM263X_CPDMA_RX_BUFFER_OFFSET (AM263X_CPDMA_BASE + 0x0028)
#define AM263X_CPDMA_TX_INTSTAT_RAW   (AM263X_CPDMA_BASE + 0x0080)
#define AM263X_CPDMA_TX_INTMASK_SET   (AM263X_CPDMA_BASE + 0x0088)
#define AM263X_CPDMA_TX_INTMASK_CLEAR (AM263X_CPDMA_BASE + 0x008c)
#define AM263X_CPDMA_EOI_VECTOR       (AM263X_CPDMA_BASE + 0x0094)
#define AM263X_CPDMA_RX_INTSTAT_RAW   (AM263X_CPDMA_BASE + 0x00a0)
#define AM263X_CPDMA_RX_INTMASK_SET   (AM263X_CPDMA_BASE + 0x00a8)
#define AM263X_CPDMA_RX_INTMASK_CLEAR (AM263X_CPDMA_BASE + 0x00ac)
#define AM263X_CPDMA_DMA_INTSTAT_RAW  (AM263X_CPDMA_BASE + 0x00b0)
#define AM263X_CPDMA_DMA_INTMASK_SET  (AM263X_CPDMA_BASE + 0x00b8)
#define AM263X_CPDMA_DMA_INTMASK_CLEAR (AM263X_CPDMA_BASE + 0x00bc)
#define AM263X_CPDMA_RX_FREEBUFFER(n) (AM263X_CPDMA_BASE + 0x00e0 + 4 * (n))
#define AM263X_CPDMA_TX_HDP(n)        (AM263X_CPDMA_BASE + 0x0200 + 4 * (n))
#define AM263X_CPDMA_RX_HDP(n)        (AM263X_CPDMA_BASE + 0x0220 + 4 * (n))
#define AM263X_CPDMA_TX_CP(n)         (AM263X_CPDMA_BASE + 0x0240 + 4 * (n))
#define AM263X_CPDMA_RX_CP(n)         (AM263X_CPDMA_BASE + 0x0260 + 4 * (n))

#define CPDMA_NCHANNELS               8

/* DMA_EOI_VECTOR values for core interrupt group 0 */

#define CPDMA_EOI_RX_THRESH           0
#define CPDMA_EOI_RX                  1
#define CPDMA_EOI_TX                  2
#define CPDMA_EOI_MISC                3

/* DMASTATUS: host error codes */

#define CPDMA_DMASTATUS_IDLE          (1 << 31)
#define CPDMA_DMASTATUS_TX_ERR_SHIFT  20
#define CPDMA_DMASTATUS_TX_ERR_MASK   (15 << CPDMA_DMASTATUS_TX_ERR_SHIFT)
#define CPDMA_DMASTATUS_RX_ERR_SHIFT  12
#define CPDMA_DMASTATUS_RX_ERR_MASK   (15 << CPDMA_DMASTATUS_RX_ERR_SHIFT)

/* Teardown complete marker, written to CP */

#define CPDMA_TEARDOWN_COMPLETE       0xfffffffc

/* CPPI 3.0 buffer descriptor, word 3: flags and packet length */

#define CPDMA_DESC_SOP                (1u << 31)
#define CPDMA_DESC_EOP                (1u << 30)
#define CPDMA_DESC_OWNER              (1u << 29)
#define CPDMA_DESC_EOQ                (1u << 28)
#define CPDMA_DESC_TDOWN_CMPLT        (1u << 27)
#define CPDMA_DESC_PASS_CRC           (1u << 26)
#define CPDMA_DESC_RX_LONG            (1u << 25)
#define CPDMA_DESC_RX_SHORT           (1u << 24)
#define CPDMA_DESC_RX_MAC_CTL         (1u << 23)
#define CPDMA_DESC_RX_OVERRUN         (1u << 22)
#define CPDMA_DESC_RX_PKT_ERR_MASK    (3u << 20)
#define CPDMA_DESC_TX_TO_PORT_EN      (1u << 20)
#define CPDMA_DESC_TX_TO_PORT(p)      ((uint32_t)(p) << 16)
#define CPDMA_DESC_RX_FROM_PORT_SHIFT 16
#define CPDMA_DESC_RX_FROM_PORT_MASK  (7u << CPDMA_DESC_RX_FROM_PORT_SHIFT)
#define CPDMA_DESC_PKTLEN_MASK        0x7ff

#define CPDMA_DESC_RX_ERRORS \
  (CPDMA_DESC_RX_LONG | CPDMA_DESC_RX_SHORT | CPDMA_DESC_RX_MAC_CTL | \
   CPDMA_DESC_RX_OVERRUN | CPDMA_DESC_RX_PKT_ERR_MASK)

/* ALE (address lookup engine) */

#define AM263X_ALE_CONTROL            (AM263X_ALE_BASE + 0x0008)
#define AM263X_ALE_PORTCTL(p)         (AM263X_ALE_BASE + 0x0040 + 4 * (p))

#define ALE_CONTROL_ENABLE_BYPASS     (1 << 4)  /* Everything to the host */
#define ALE_CONTROL_CLEAR_TABLE       (1 << 30)
#define ALE_CONTROL_ENABLE_ALE        (1u << 31)

#define ALE_PORTCTL_STATE_DISABLED    0
#define ALE_PORTCTL_STATE_BLOCKED     1
#define ALE_PORTCTL_STATE_LEARN       2
#define ALE_PORTCTL_STATE_FORWARD     3
#define ALE_PORTCTL_STATE_MASK        3

/* MSS_CTRL CPSW_CONTROL: the MAC ports' interface modes */

#define AM263X_MSS_CTRL_CPSW_CONTROL  (AM263X_MSS_CTRL_BASE + 0x0810)
#define AM263X_MSS_CTRL_LOCK0_KICK0   (AM263X_MSS_CTRL_BASE + 0x1008)
#define AM263X_MSS_CTRL_LOCK0_KICK1   (AM263X_MSS_CTRL_BASE + 0x100c)

#define MSS_CPSW_CONTROL_PORT_SHIFT(p) (16 * ((p) - 1))
#define MSS_CPSW_CONTROL_MODE_MASK(p) (7 << MSS_CPSW_CONTROL_PORT_SHIFT(p))
#define MSS_CPSW_CONTROL_MODE_MII(p)  (0 << MSS_CPSW_CONTROL_PORT_SHIFT(p))
#define MSS_CPSW_CONTROL_MODE_RMII(p) (1 << MSS_CPSW_CONTROL_PORT_SHIFT(p))
#define MSS_CPSW_CONTROL_MODE_RGMII(p) (2 << MSS_CPSW_CONTROL_PORT_SHIFT(p))
#define MSS_CPSW_CONTROL_RMII_REF_CLK_OE_N(p) \
  (1 << (4 + MSS_CPSW_CONTROL_PORT_SHIFT(p)))
#define MSS_CPSW_CONTROL_RGMII_ID_MODE(p) \
  (1 << (8 + MSS_CPSW_CONTROL_PORT_SHIFT(p)))  /* 1: no internal delay */

/* MDIO register offsets */

#define AM263X_MDIO_VERSION_OFFSET    0x0000
#define AM263X_MDIO_CONTROL_OFFSET    0x0004
#define AM263X_MDIO_ALIVE_OFFSET      0x0008
#define AM263X_MDIO_LINK_OFFSET       0x000c
#define AM263X_MDIO_POLL_OFFSET       0x0034
#define AM263X_MDIO_USERACCESS_OFFSET(n) (0x0080 + 8 * (n))
#define AM263X_MDIO_USERPHYSEL_OFFSET(n) (0x0084 + 8 * (n))

#define AM263X_MDIO_VERSION \
  (AM263X_MDIO_BASE + AM263X_MDIO_VERSION_OFFSET)
#define AM263X_MDIO_CONTROL \
  (AM263X_MDIO_BASE + AM263X_MDIO_CONTROL_OFFSET)
#define AM263X_MDIO_ALIVE \
  (AM263X_MDIO_BASE + AM263X_MDIO_ALIVE_OFFSET)
#define AM263X_MDIO_LINK \
  (AM263X_MDIO_BASE + AM263X_MDIO_LINK_OFFSET)
#define AM263X_MDIO_POLL \
  (AM263X_MDIO_BASE + AM263X_MDIO_POLL_OFFSET)
#define AM263X_MDIO_USERACCESS(n) \
  (AM263X_MDIO_BASE + AM263X_MDIO_USERACCESS_OFFSET(n))
#define AM263X_MDIO_USERPHYSEL(n) \
  (AM263X_MDIO_BASE + AM263X_MDIO_USERPHYSEL_OFFSET(n))

/* MDIO CONTROL */

#define MDIO_CONTROL_CLKDIV_SHIFT     0
#define MDIO_CONTROL_CLKDIV_MASK      (0xffff << MDIO_CONTROL_CLKDIV_SHIFT)
#define MDIO_CONTROL_CLKDIV(n) \
  ((uint32_t)(n) << MDIO_CONTROL_CLKDIV_SHIFT)
#define MDIO_CONTROL_FAULTENB         (1 << 18)
#define MDIO_CONTROL_FAULT            (1 << 19)
#define MDIO_CONTROL_PREAMBLE         (1 << 20) /* 1: no preamble */
#define MDIO_CONTROL_ENABLE           (1 << 30)
#define MDIO_CONTROL_IDLE             (1 << 31)

/* MDIO POLL */

#define MDIO_POLL_STATECHANGEMODE     (1 << 30)
#define MDIO_POLL_MANUALMODE          (1 << 31)

/* MDIO USERACCESSn */

#define MDIO_USERACCESS_DATA_SHIFT    0
#define MDIO_USERACCESS_DATA_MASK     (0xffff << MDIO_USERACCESS_DATA_SHIFT)
#define MDIO_USERACCESS_PHYADR_SHIFT  16
#define MDIO_USERACCESS_PHYADR(n) \
  (((uint32_t)(n) & 0x1f) << MDIO_USERACCESS_PHYADR_SHIFT)
#define MDIO_USERACCESS_REGADR_SHIFT  21
#define MDIO_USERACCESS_REGADR(n) \
  (((uint32_t)(n) & 0x1f) << MDIO_USERACCESS_REGADR_SHIFT)
#define MDIO_USERACCESS_ACK           (1 << 29)
#define MDIO_USERACCESS_WRITE         (1 << 30)
#define MDIO_USERACCESS_GO            (1 << 31)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_CPSW_H */
