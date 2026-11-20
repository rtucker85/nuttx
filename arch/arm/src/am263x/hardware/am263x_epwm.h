/****************************************************************************
 * arch/arm/src/am263x/hardware/am263x_epwm.h
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

#ifndef __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_EPWM_H
#define __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_EPWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/am263x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32 ePWMs, 4 KB apart.  This is the group 0 view of each module, the one
 * EPWM_STATICXBAR_SEL selects after reset.  The time base runs from
 * SYSCLK.  Registers are 16 bits wide unless noted.
 */

#define AM263X_EPWM_NINSTANCES      32
#define AM263X_EPWM_BASE(n)         (AM263X_EPWM0_BASE + ((n) << 12))

/* Register offsets */

#define AM263X_EPWM_TBCTL           0x0000
#define AM263X_EPWM_TBCTL2          0x0002
#define AM263X_EPWM_SYNCINSEL       0x0006
#define AM263X_EPWM_TBCTR           0x0008
#define AM263X_EPWM_TBSTS           0x000a
#define AM263X_EPWM_SYNCOUTEN       0x000c
#define AM263X_EPWM_CMPCTL          0x0010
#define AM263X_EPWM_CMPCTL2         0x0012
#define AM263X_EPWM_AQCTLA          0x0080
#define AM263X_EPWM_AQCTLB          0x0084
#define AM263X_EPWM_AQSFRC          0x008e
#define AM263X_EPWM_AQCSFRC         0x0092
#define AM263X_EPWM_TBPHS           0x00c0  /* 32 bits */
#define AM263X_EPWM_TBPRD           0x00c6
#define AM263X_EPWM_CMPA            0x00d4  /* 32 bits, CMPA in 31:16 */
#define AM263X_EPWM_CMPB            0x00d8  /* 32 bits, CMPB in 31:16 */
#define AM263X_EPWM_CMPC            0x00de
#define AM263X_EPWM_CMPD            0x00e2
#define AM263X_EPWM_TZCTL           0x0108
#define AM263X_EPWM_TZFLG           0x0126
#define AM263X_EPWM_ETSEL           0x0148
#define AM263X_EPWM_ETPS            0x014c
#define AM263X_EPWM_ETFLG           0x0150
#define AM263X_EPWM_ETCLR           0x0154
#define AM263X_EPWM_ETSOCPS         0x0160
#define AM263X_EPWM_REV             0x007c

/* TBCTL */

#define EPWM_TBCTL_CTRMODE_SHIFT    0
#define EPWM_TBCTL_CTRMODE_MASK     (3 << EPWM_TBCTL_CTRMODE_SHIFT)
#  define EPWM_TBCTL_CTRMODE_UP     (0 << EPWM_TBCTL_CTRMODE_SHIFT)
#  define EPWM_TBCTL_CTRMODE_DOWN   (1 << EPWM_TBCTL_CTRMODE_SHIFT)
#  define EPWM_TBCTL_CTRMODE_UPDOWN (2 << EPWM_TBCTL_CTRMODE_SHIFT)
#  define EPWM_TBCTL_CTRMODE_STOP   (3 << EPWM_TBCTL_CTRMODE_SHIFT)
#define EPWM_TBCTL_PHSEN            (1 << 2)
#define EPWM_TBCTL_PRDLD            (1 << 3)  /* 1: TBPRD writes immediate */
#define EPWM_TBCTL_SWFSYNC          (1 << 6)
#define EPWM_TBCTL_HSPCLKDIV_SHIFT  7         /* /1, then /2n for n = 1-7 */
#define EPWM_TBCTL_HSPCLKDIV_MASK   (7 << EPWM_TBCTL_HSPCLKDIV_SHIFT)
#define EPWM_TBCTL_CLKDIV_SHIFT     10        /* /2^n */
#define EPWM_TBCTL_CLKDIV_MASK      (7 << EPWM_TBCTL_CLKDIV_SHIFT)
#define EPWM_TBCTL_FREESOFT_SHIFT   14
#define EPWM_TBCTL_FREESOFT_MASK    (3 << EPWM_TBCTL_FREESOFT_SHIFT)
#  define EPWM_TBCTL_FREE_RUN       (2 << EPWM_TBCTL_FREESOFT_SHIFT)

/* CMPCTL (CMPA, CMPB) and CMPCTL2 (CMPC, CMPD) share a layout */

#define EPWM_CMPCTL_LOADAMODE_SHIFT 0         /* 0: load at CTR = 0 */
#define EPWM_CMPCTL_LOADAMODE_MASK  (3 << EPWM_CMPCTL_LOADAMODE_SHIFT)
#define EPWM_CMPCTL_LOADBMODE_SHIFT 2
#define EPWM_CMPCTL_LOADBMODE_MASK  (3 << EPWM_CMPCTL_LOADBMODE_SHIFT)
#define EPWM_CMPCTL_SHDWAMODE       (1 << 4)  /* 1: immediate */
#define EPWM_CMPCTL_SHDWBMODE       (1 << 6)

/* AQCTLA and AQCTLB: a two-bit action per event */

#define EPWM_AQ_NOTHING             0
#define EPWM_AQ_CLEAR               1
#define EPWM_AQ_SET                 2
#define EPWM_AQ_TOGGLE              3
#define EPWM_AQCTL_ZRO_SHIFT        0
#define EPWM_AQCTL_PRD_SHIFT        2
#define EPWM_AQCTL_CAU_SHIFT        4
#define EPWM_AQCTL_CAD_SHIFT        6
#define EPWM_AQCTL_CBU_SHIFT        8
#define EPWM_AQCTL_CBD_SHIFT        10

/* AQSFRC and AQCSFRC: continuous software force */

#define EPWM_AQSFRC_RLDCSF_SHIFT    6
#define EPWM_AQSFRC_RLDCSF_MASK     (3 << EPWM_AQSFRC_RLDCSF_SHIFT)
#  define EPWM_AQSFRC_RLDCSF_IMMEDIATE (3 << EPWM_AQSFRC_RLDCSF_SHIFT)
#define EPWM_AQCSFRC_OFF            0
#define EPWM_AQCSFRC_LOW            1
#define EPWM_AQCSFRC_HIGH           2
#define EPWM_AQCSFRC_CSFA_SHIFT     0
#define EPWM_AQCSFRC_CSFB_SHIFT     2
#define EPWM_AQCSFRC_MASK           3

/* CMPA, CMPB: the 16-bit compare value over the high-resolution bits */

#define EPWM_CMP_SHIFT              16

/* ETSEL */

#define EPWM_ETSEL_SOCASELCMP       (1 << 4)  /* CMPC/CMPD, not CMPA/CMPB */
#define EPWM_ETSEL_SOCBSELCMP       (1 << 5)
#define EPWM_ETSEL_SOCASEL_SHIFT    8
#define EPWM_ETSEL_SOCASEL_MASK     (7 << EPWM_ETSEL_SOCASEL_SHIFT)
#define EPWM_ETSEL_SOCAEN           (1 << 11)
#define EPWM_ETSEL_SOCBSEL_SHIFT    12
#define EPWM_ETSEL_SOCBSEL_MASK     (7 << EPWM_ETSEL_SOCBSEL_SHIFT)
#define EPWM_ETSEL_SOCBEN           (1 << 15)

/* SOCxSEL values.  With SOCxSELCMP set, 4-7 mean CMPC up, CMPC down,
 * CMPD up and CMPD down.
 */

#define EPWM_SOCSEL_ZERO            1
#define EPWM_SOCSEL_PERIOD          2
#define EPWM_SOCSEL_ZERO_PERIOD     3
#define EPWM_SOCSEL_CMPA_UP         4
#define EPWM_SOCSEL_CMPA_DOWN       5
#define EPWM_SOCSEL_CMPB_UP         6
#define EPWM_SOCSEL_CMPB_DOWN       7

/* ETPS: SOCPSSEL selects the four-bit prescalers in ETSOCPS */

#define EPWM_ETPS_SOCPSSEL          (1 << 5)

/* ETSOCPS */

#define EPWM_ETSOCPS_SOCAPRD2_SHIFT 0         /* SOC every n events, 1-15 */
#define EPWM_ETSOCPS_SOCAPRD2_MASK  (0xf << EPWM_ETSOCPS_SOCAPRD2_SHIFT)
#define EPWM_ETSOCPS_SOCBPRD2_SHIFT 8
#define EPWM_ETSOCPS_SOCBPRD2_MASK  (0xf << EPWM_ETSOCPS_SOCBPRD2_SHIFT)

/* ETFLG, ETCLR */

#define EPWM_ET_INT                 (1 << 0)
#define EPWM_ET_SOCA                (1 << 2)
#define EPWM_ET_SOCB                (1 << 3)

#endif /* __ARCH_ARM_SRC_AM263X_HARDWARE_AM263X_EPWM_H */
