/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_mcan.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MCAN_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MCAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Four MCAN modules (Bosch M_CAN with TI's MCANSS wrapper), 64 KB apart.
 * Each has its message RAM at its base, the MCANSS wrapper at base +
 * 0x8000 and the M_CAN core registers at base + 0x8200.  The message RAM
 * holds 4352 words (17 KB).
 */

#define AM263X_MCAN_NINSTANCES       4
#define AM263X_MCAN_BASE(n)          (AM263X_MCAN0_BASE + ((n) << 16))
#define AM263X_MCAN_MSGRAM(n)        AM263X_MCAN_BASE(n)
#define AM263X_MCAN_SS(n)            (AM263X_MCAN_BASE(n) + 0x8000)
#define AM263X_MCAN_CORE(n)          (AM263X_MCAN_BASE(n) + 0x8200)
#define AM263X_MCAN_MSGRAM_WORDS     4352

/* Interrupt line 0 of each module on R5FSS0-0 (line 1 is IRQ + 1, the
 * external timestamp rollover IRQ - 1).
 */

#define AM263X_IRQ_MCAN(n)           (27 + 3 * (n))

/* MCANSS wrapper register offsets ******************************************/

#define AM263X_MCANSS_PID_OFFSET     0x0000
#define AM263X_MCANSS_CTRL_OFFSET    0x0004
#define AM263X_MCANSS_STAT_OFFSET    0x0008

#define MCANSS_STAT_RESET            (1 << 0)  /* Reset in progress */
#define MCANSS_STAT_MEM_INIT_DONE    (1 << 1)

/* M_CAN core register offsets **********************************************/

#define AM263X_MCAN_CREL_OFFSET      0x0000 /* Core release */
#define AM263X_MCAN_ENDN_OFFSET      0x0004 /* Endian (0x87654321) */
#define AM263X_MCAN_DBTP_OFFSET      0x000c /* Data bit timing */
#define AM263X_MCAN_TEST_OFFSET      0x0010
#define AM263X_MCAN_RWD_OFFSET       0x0014 /* RAM watchdog */
#define AM263X_MCAN_CCCR_OFFSET      0x0018 /* CC control */
#define AM263X_MCAN_NBTP_OFFSET      0x001c /* Nominal bit timing */
#define AM263X_MCAN_TSCC_OFFSET      0x0020 /* Timestamp counter config */
#define AM263X_MCAN_TSCV_OFFSET      0x0024
#define AM263X_MCAN_TOCC_OFFSET      0x0028 /* Timeout counter config */
#define AM263X_MCAN_ECR_OFFSET       0x0040 /* Error counters */
#define AM263X_MCAN_PSR_OFFSET       0x0044 /* Protocol status */
#define AM263X_MCAN_TDCR_OFFSET      0x0048 /* Transmitter delay comp. */
#define AM263X_MCAN_IR_OFFSET        0x0050 /* Interrupt flags */
#define AM263X_MCAN_IE_OFFSET        0x0054 /* Interrupt enables */
#define AM263X_MCAN_ILS_OFFSET       0x0058 /* Interrupt line select */
#define AM263X_MCAN_ILE_OFFSET       0x005c /* Interrupt line enable */
#define AM263X_MCAN_GFC_OFFSET       0x0080 /* Global filter config */
#define AM263X_MCAN_SIDFC_OFFSET     0x0084 /* Standard ID filter config */
#define AM263X_MCAN_XIDFC_OFFSET     0x0088 /* Extended ID filter config */
#define AM263X_MCAN_XIDAM_OFFSET     0x0090 /* Extended ID AND mask */
#define AM263X_MCAN_NDAT1_OFFSET     0x0098
#define AM263X_MCAN_NDAT2_OFFSET     0x009c
#define AM263X_MCAN_RXF0C_OFFSET     0x00a0 /* RX FIFO 0 config */
#define AM263X_MCAN_RXF0S_OFFSET     0x00a4 /* RX FIFO 0 status */
#define AM263X_MCAN_RXF0A_OFFSET     0x00a8 /* RX FIFO 0 acknowledge */
#define AM263X_MCAN_RXBC_OFFSET      0x00ac
#define AM263X_MCAN_RXF1C_OFFSET     0x00b0
#define AM263X_MCAN_RXESC_OFFSET     0x00bc /* RX element size config */
#define AM263X_MCAN_TXBC_OFFSET      0x00c0 /* TX buffer config */
#define AM263X_MCAN_TXFQS_OFFSET     0x00c4 /* TX FIFO/queue status */
#define AM263X_MCAN_TXESC_OFFSET     0x00c8 /* TX element size config */
#define AM263X_MCAN_TXBRP_OFFSET     0x00cc /* TX request pending */
#define AM263X_MCAN_TXBAR_OFFSET     0x00d0 /* TX add request */
#define AM263X_MCAN_TXBCR_OFFSET     0x00d4 /* TX cancellation request */
#define AM263X_MCAN_TXBTO_OFFSET     0x00d8 /* TX occurred */
#define AM263X_MCAN_TXBCF_OFFSET     0x00dc /* TX cancellation finished */
#define AM263X_MCAN_TXBTIE_OFFSET    0x00e0 /* TX interrupt enable */
#define AM263X_MCAN_TXBCIE_OFFSET    0x00e4
#define AM263X_MCAN_TXEFC_OFFSET     0x00f0 /* TX event FIFO config */

/* M_CAN register bit definitions *******************************************/

/* Data bit timing (fields hold value - 1) */

#define MCAN_DBTP_DSJW_SHIFT         0
#define MCAN_DBTP_DSJW_MASK          (0xf << MCAN_DBTP_DSJW_SHIFT)
#define MCAN_DBTP_DTSEG2_SHIFT       4
#define MCAN_DBTP_DTSEG2_MASK        (0xf << MCAN_DBTP_DTSEG2_SHIFT)
#define MCAN_DBTP_DTSEG1_SHIFT       8
#define MCAN_DBTP_DTSEG1_MASK        (0x1f << MCAN_DBTP_DTSEG1_SHIFT)
#define MCAN_DBTP_DBRP_SHIFT         16
#define MCAN_DBTP_DBRP_MASK          (0x1f << MCAN_DBTP_DBRP_SHIFT)
#define MCAN_DBTP_TDC                (1 << 23)

#define MCAN_DBTP_DSJW_MAX           16
#define MCAN_DBTP_DTSEG2_MAX         16
#define MCAN_DBTP_DTSEG1_MAX         32
#define MCAN_DBTP_DBRP_MAX           32

/* Test */

#define MCAN_TEST_LBCK               (1 << 4)  /* Loopback mode */

/* CC control */

#define MCAN_CCCR_INIT               (1 << 0)  /* Initialisation */
#define MCAN_CCCR_CCE                (1 << 1)  /* Configuration change */
#define MCAN_CCCR_ASM                (1 << 2)  /* Restricted operation */
#define MCAN_CCCR_CSA                (1 << 3)  /* Clock stop ack */
#define MCAN_CCCR_CSR                (1 << 4)  /* Clock stop request */
#define MCAN_CCCR_MON                (1 << 5)  /* Bus monitoring */
#define MCAN_CCCR_DAR                (1 << 6)  /* Disable auto retransmit */
#define MCAN_CCCR_TEST               (1 << 7)  /* Test mode enable */
#define MCAN_CCCR_FDOE               (1 << 8)  /* FD operation enable */
#define MCAN_CCCR_BRSE               (1 << 9)  /* Bit rate switch enable */
#define MCAN_CCCR_PXHD               (1 << 12) /* Protocol exc. disable */
#define MCAN_CCCR_EFBI               (1 << 13) /* Edge filtering */
#define MCAN_CCCR_TXP                (1 << 14) /* Transmit pause */

/* Nominal bit timing (fields hold value - 1) */

#define MCAN_NBTP_NTSEG2_SHIFT       0
#define MCAN_NBTP_NTSEG2_MASK        (0x7f << MCAN_NBTP_NTSEG2_SHIFT)
#define MCAN_NBTP_NTSEG1_SHIFT       8
#define MCAN_NBTP_NTSEG1_MASK        (0xff << MCAN_NBTP_NTSEG1_SHIFT)
#define MCAN_NBTP_NBRP_SHIFT         16
#define MCAN_NBTP_NBRP_MASK          (0x1ff << MCAN_NBTP_NBRP_SHIFT)
#define MCAN_NBTP_NSJW_SHIFT         25
#define MCAN_NBTP_NSJW_MASK          (0x7f << MCAN_NBTP_NSJW_SHIFT)

#define MCAN_NBTP_NSJW_MAX           128
#define MCAN_NBTP_NTSEG2_MAX         128
#define MCAN_NBTP_NTSEG1_MAX         256
#define MCAN_NBTP_NBRP_MAX           512

/* Error counters */

#define MCAN_ECR_TEC_SHIFT           0
#define MCAN_ECR_TEC_MASK            (0xff << MCAN_ECR_TEC_SHIFT)
#define MCAN_ECR_REC_SHIFT           8
#define MCAN_ECR_REC_MASK            (0x7f << MCAN_ECR_REC_SHIFT)
#define MCAN_ECR_RP                  (1 << 15)

/* Protocol status */

#define MCAN_PSR_LEC_SHIFT           0         /* Last error code */
#define MCAN_PSR_LEC_MASK            (7 << MCAN_PSR_LEC_SHIFT)
#  define MCAN_PSR_LEC_NONE          0
#  define MCAN_PSR_LEC_STUFF         1
#  define MCAN_PSR_LEC_FORM          2
#  define MCAN_PSR_LEC_ACK           3
#  define MCAN_PSR_LEC_BIT1          4
#  define MCAN_PSR_LEC_BIT0          5
#  define MCAN_PSR_LEC_CRC           6
#  define MCAN_PSR_LEC_NOCHANGE      7
#define MCAN_PSR_EP                  (1 << 5)  /* Error passive */
#define MCAN_PSR_EW                  (1 << 6)  /* Error warning */
#define MCAN_PSR_BO                  (1 << 7)  /* Bus off */

/* Transmitter delay compensation */

#define MCAN_TDCR_TDCF_SHIFT         0
#define MCAN_TDCR_TDCO_SHIFT         8
#define MCAN_TDCR_MAX                127

/* Interrupt flags, enables and line selects */

#define MCAN_INT_RF0N                (1 << 0)  /* RX FIFO 0 new message */
#define MCAN_INT_RF0F                (1 << 2)  /* RX FIFO 0 full */
#define MCAN_INT_RF0L                (1 << 3)  /* RX FIFO 0 message lost */
#define MCAN_INT_TC                  (1 << 9)  /* Transmission completed */
#define MCAN_INT_TCF                 (1 << 10) /* TX cancellation done */
#define MCAN_INT_MRAF                (1 << 17) /* Message RAM access fail */
#define MCAN_INT_TOO                 (1 << 18) /* Timeout occurred */
#define MCAN_INT_BEC                 (1 << 20) /* Bit error corrected */
#define MCAN_INT_BEU                 (1 << 21) /* Bit error uncorrected */
#define MCAN_INT_ELO                 (1 << 22) /* Error logging overflow */
#define MCAN_INT_EP                  (1 << 23) /* Error passive */
#define MCAN_INT_EW                  (1 << 24) /* Warning status */
#define MCAN_INT_BO                  (1 << 25) /* Bus off status */
#define MCAN_INT_WDI                 (1 << 26) /* RAM watchdog */
#define MCAN_INT_PEA                 (1 << 27) /* Protocol error, arb. */
#define MCAN_INT_PED                 (1 << 28) /* Protocol error, data */
#define MCAN_INT_ARA                 (1 << 29) /* Access to reserved addr */
#define MCAN_INT_ALL                 0x3fffffff

#define MCAN_ILE_EINT0               (1 << 0)
#define MCAN_ILE_EINT1               (1 << 1)

/* Global filter configuration */

#define MCAN_GFC_RRFE                (1 << 0)  /* Reject remote, extended */
#define MCAN_GFC_RRFS                (1 << 1)  /* Reject remote, standard */
#define MCAN_GFC_ANFE_SHIFT          2         /* Non-matching extended */
#define MCAN_GFC_ANFS_SHIFT          4         /* Non-matching standard */
#  define MCAN_GFC_ANF_FIFO0         0
#  define MCAN_GFC_ANF_FIFO1         1
#  define MCAN_GFC_ANF_REJECT        2

/* Standard and extended ID filter configuration */

#define MCAN_SIDFC_FLSSA_SHIFT       0         /* Byte offset, 4-aligned */
#define MCAN_SIDFC_LSS_SHIFT         16        /* Number of elements */
#define MCAN_XIDFC_FLESA_SHIFT       0
#define MCAN_XIDFC_LSE_SHIFT         16

/* RX FIFO 0 */

#define MCAN_RXF0C_F0SA_SHIFT        0         /* Byte offset, 4-aligned */
#define MCAN_RXF0C_F0S_SHIFT         16        /* Number of elements */
#define MCAN_RXF0C_F0WM_SHIFT        24        /* Watermark */
#define MCAN_RXF0C_F0OM              (1 << 31) /* Overwrite mode */

#define MCAN_RXF0S_F0FL_SHIFT        0         /* Fill level */
#define MCAN_RXF0S_F0FL_MASK         (0x7f << MCAN_RXF0S_F0FL_SHIFT)
#define MCAN_RXF0S_F0GI_SHIFT        8         /* Get index */
#define MCAN_RXF0S_F0GI_MASK         (0x3f << MCAN_RXF0S_F0GI_SHIFT)
#define MCAN_RXF0S_F0F               (1 << 24) /* Full */
#define MCAN_RXF0S_RF0L              (1 << 25) /* Message lost */

/* RX and TX element size: 0-7 = 8, 12, 16, 20, 24, 32, 48, 64 bytes */

#define MCAN_ESC_64BYTES             7
#define MCAN_RXESC_F0DS_SHIFT        0
#define MCAN_RXESC_F1DS_SHIFT        4
#define MCAN_RXESC_RBDS_SHIFT        8
#define MCAN_TXESC_TBDS_SHIFT        0

/* TX buffer configuration and FIFO status */

#define MCAN_TXBC_TBSA_SHIFT         0         /* Byte offset, 4-aligned */
#define MCAN_TXBC_NDTB_SHIFT         16        /* Dedicated buffers */
#define MCAN_TXBC_TFQS_SHIFT         24        /* FIFO/queue elements */
#define MCAN_TXBC_TFQM               (1 << 30) /* Queue (not FIFO) mode */

#define MCAN_TXFQS_TFFL_SHIFT        0         /* Free level */
#define MCAN_TXFQS_TFFL_MASK         (0x3f << MCAN_TXFQS_TFFL_SHIFT)
#define MCAN_TXFQS_TFQPI_SHIFT       16        /* Put index */
#define MCAN_TXFQS_TFQPI_MASK        (0x1f << MCAN_TXFQS_TFQPI_SHIFT)
#define MCAN_TXFQS_TFQF              (1 << 21) /* FIFO/queue full */

/* Message RAM elements *****************************************************/

/* RX and TX buffer element, word 0 (T0/R0) */

#define MCAN_ELEM_ID_SHIFT           0
#define MCAN_ELEM_ID_MASK            0x1fffffff
#define MCAN_ELEM_STDID_SHIFT        18        /* Standard ID: bits 28-18 */
#define MCAN_ELEM_RTR                (1 << 29)
#define MCAN_ELEM_XTD                (1 << 30)
#define MCAN_ELEM_ESI                (1 << 31)

/* Word 1 (T1/R1) */

#define MCAN_ELEM_RXTS_MASK          0xffff    /* RX: timestamp */
#define MCAN_ELEM_DLC_SHIFT          16
#define MCAN_ELEM_DLC_MASK           (0xf << MCAN_ELEM_DLC_SHIFT)
#define MCAN_ELEM_BRS                (1 << 20)
#define MCAN_ELEM_FDF                (1 << 21)
#define MCAN_ELEM_EFC                (1 << 23) /* TX: store TX event */
#define MCAN_ELEM_FIDX_SHIFT         24        /* RX: filter index */
#define MCAN_ELEM_ANMF               (1 << 31) /* RX: no filter matched */

/* Element size with 64 data bytes: 2 header + 16 data words */

#define MCAN_ELEM_WORDS              18

/* Standard ID filter element (one word) */

#define MCAN_SFE_SFID2_SHIFT         0
#define MCAN_SFE_SFID1_SHIFT         16
#define MCAN_SFE_SFEC_SHIFT          27
#define MCAN_SFE_SFT_SHIFT           30

/* Extended ID filter element (two words): EFEC in F0, EFT in F1 */

#define MCAN_XFE_EFID_MASK           0x1fffffff
#define MCAN_XFE_EFEC_SHIFT          29
#define MCAN_XFE_EFT_SHIFT           30

/* Filter types (SFT, EFT) and element configurations (SFEC, EFEC) */

#define MCAN_FT_RANGE                0
#define MCAN_FT_DUAL                 1
#define MCAN_FT_CLASSIC              2         /* ID and mask */
#define MCAN_FT_DISABLED             3

#define MCAN_FEC_DISABLE             0
#define MCAN_FEC_FIFO0               1

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_MCAN_H */
