/****************************************************************************
 * arch/arm/src/am263x/am263x_mcan.c
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

/* SocketCAN driver for the AM263x's four MCAN modules (Bosch M_CAN in TI's
 * MCANSS wrapper), CAN FD capable, on the netdev lower-half interface.
 *
 * Message RAM layout, the same for every module:
 *
 *   0x0000  standard ID filters, MCAN_NSTDFILTER x 1 word
 *   0x0080  extended ID filters, MCAN_NEXTFILTER x 2 words
 *   0x0100  RX FIFO 0, MCAN_NRX elements of 64 data bytes
 *   0x1300  TX FIFO, MCAN_NTX elements of 64 data bytes
 *
 * RX: line 0's interrupt masks RF0N and wakes the upper half, whose
 * receive() empties FIFO 0 and unmasks it again.  TX: transmit() writes
 * the element at the FIFO's put index and keeps the packet until the
 * buffer's request is no longer pending, when reclaim() frees it; TC wakes
 * the upper half to do that.  Bus-off starts recovery at once (the
 * controller then waits for 129 x 11 recessive bits).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <net/if.h>
#include <nuttx/arch.h>
#include <nuttx/can.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "arm_internal.h"
#include "hardware/am263x_mcan.h"
#include "hardware/am263x_rcm.h"
#include "am263x_mcan.h"

#ifdef CONFIG_AM263X_MCAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Functional clock: DPLL_CORE_HSDIV0_CLKOUT0 (400 MHz) / 5, set up in
 * am263x_clockconfig().
 */

#define MCAN_CLK_HZ          80000000

/* Message RAM layout (byte offsets) */

#define MCAN_NSTDFILTER      32
#define MCAN_NEXTFILTER      16
#define MCAN_NRX             64
#define MCAN_NTX             32
#define MCAN_ELEM_BYTES      (MCAN_ELEM_WORDS * 4)

#define MCAN_RAM_SFE         0x0000
#define MCAN_RAM_XFE         (MCAN_RAM_SFE + MCAN_NSTDFILTER * 4)
#define MCAN_RAM_RXF0        (MCAN_RAM_XFE + MCAN_NEXTFILTER * 8)
#define MCAN_RAM_TXB         (MCAN_RAM_RXF0 + MCAN_NRX * MCAN_ELEM_BYTES)
#define MCAN_RAM_END         (MCAN_RAM_TXB + MCAN_NTX * MCAN_ELEM_BYTES)

#if MCAN_RAM_END > AM263X_MCAN_MSGRAM_WORDS * 4
#  error "MCAN message RAM layout too large"
#endif

/* Packets the driver may hold: one per TX element, a few for RX */

#define MCAN_TX_QUOTA        MCAN_NTX
#define MCAN_RX_QUOTA        8

/* Polls for CCCR.INIT and the message RAM initialisation */

#define MCAN_TIMEOUT         100000

/* Interrupts.  Error interrupts are masked when they fire and unmasked
 * once receive() has reported them, so a bus fault cannot flood the CPU.
 */

#define MCAN_INT_RX          MCAN_INT_RF0N
#ifdef CONFIG_AM263X_MCAN_BUSERR
#  define MCAN_INT_BUSERR    (MCAN_INT_PEA | MCAN_INT_PED)
#else
#  define MCAN_INT_BUSERR    0
#endif
#ifdef CONFIG_NET_CAN_ERRORS
#  define MCAN_INT_ERRORS    (MCAN_INT_RF0L | MCAN_INT_EW | MCAN_INT_EP | \
                              MCAN_INT_BO | MCAN_INT_BUSERR)
#else
#  define MCAN_INT_ERRORS    MCAN_INT_BO
#endif
#define MCAN_INT_DEFAULT     (MCAN_INT_RX | MCAN_INT_TC | MCAN_INT_ERRORS)

/* Error counter levels for SocketCAN's warning and passive states */

#define MCAN_ERR_WARNING     96
#define MCAN_ERR_PASSIVE     128

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mcan_timing_s
{
  uint16_t brp;                 /* Prescaler */
  uint16_t tseg1;               /* Prop + phase 1, in time quanta */
  uint16_t tseg2;               /* Phase 2, in time quanta */
  uint16_t sjw;
};

struct am263x_mcan_s
{
  struct netdev_lowerhalf_s dev; /* Must be first */

  uintptr_t core;               /* M_CAN registers */
  uintptr_t ss;                 /* MCANSS wrapper registers */
  uintptr_t ram;                /* Message RAM */
  uint8_t   port;
  uint8_t   irq;
  spinlock_t lock;              /* IE and the error state */
  uint32_t  ie;                 /* What IE holds while up */

  /* Bit rates (bit/s) and sample points (percent) */

  uint32_t  nbitrate;
  uint32_t  dbitrate;
  uint16_t  nsamplep;
  uint16_t  dsamplep;

  /* TX: the packet held for each FIFO element */

  FAR netpkt_t *txpkt[MCAN_NTX];
  uint32_t  txbusy;             /* Elements with a packet in them */

  /* Errors noticed by the interrupt, reported by receive() */

  uint32_t  errir;              /* IR bits */
  uint32_t  errpsr;             /* PSR when they fired */
  uint32_t  nerrors;            /* Bus errors since ifup */
  uint32_t  noverruns;          /* Lost RX frames since ifup */

  /* Hardware filters, written to the message RAM on ifup.  With none,
   * every frame is accepted.
   */

  uint32_t  sfe[MCAN_NSTDFILTER];
  uint32_t  xfe[MCAN_NEXTFILTER][2];
  uint8_t   nsfe;
  uint8_t   nxfe;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  mcan_ifup(FAR struct netdev_lowerhalf_s *dev);
static int  mcan_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int  mcan_transmit(FAR struct netdev_lowerhalf_s *dev,
                          FAR netpkt_t *pkt);
static FAR netpkt_t *mcan_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int  mcan_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                       unsigned long arg);
#endif
static void mcan_reclaim(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_mcan_ops =
{
  .ifup     = mcan_ifup,
  .ifdown   = mcan_ifdown,
  .transmit = mcan_transmit,
  .receive  = mcan_receive,
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = mcan_ioctl,
#endif
  .reclaim  = mcan_reclaim,
};

static struct am263x_mcan_s g_mcan[AM263X_MCAN_NINSTANCES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcan_getreg / mcan_putreg
 ****************************************************************************/

static inline uint32_t mcan_getreg(FAR struct am263x_mcan_s *priv,
                                   unsigned int offset)
{
  return getreg32(priv->core + offset);
}

static inline void mcan_putreg(FAR struct am263x_mcan_s *priv,
                               unsigned int offset, uint32_t value)
{
  putreg32(value, priv->core + offset);
}

/****************************************************************************
 * Name: mcan_ram
 *
 * Description:
 *   Address of a word in the message RAM.  It only takes 32-bit accesses.
 *
 ****************************************************************************/

static inline uintptr_t mcan_ram(FAR struct am263x_mcan_s *priv,
                                 unsigned int offset)
{
  return priv->ram + offset;
}

/****************************************************************************
 * Name: mcan_setie
 *
 * Description:
 *   Set and clear bits of the interrupt enables, from any context.
 *
 ****************************************************************************/

static void mcan_setie(FAR struct am263x_mcan_s *priv, uint32_t set,
                       uint32_t clear)
{
  irqstate_t flags = spin_lock_irqsave(&priv->lock);

  priv->ie = (priv->ie & ~clear) | set;
  mcan_putreg(priv, AM263X_MCAN_IE_OFFSET, priv->ie);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: mcan_bittiming
 *
 * Description:
 *   Find a prescaler and segments for a bit rate and sample point: the
 *   smallest prescaler (most time quanta) that divides the clock exactly
 *   and fits the segment limits.
 *
 ****************************************************************************/

static int mcan_bittiming(uint32_t bitrate, uint16_t samplep,
                          uint32_t brpmax, uint32_t seg1max,
                          uint32_t seg2max, uint32_t sjwmax,
                          FAR struct mcan_timing_s *t)
{
  uint32_t brp;
  uint32_t ntq;
  uint32_t seg1;
  uint32_t seg2;

  if (bitrate == 0 || samplep == 0 || samplep >= 100)
    {
      return -EINVAL;
    }

  for (brp = 1; brp <= brpmax; brp++)
    {
      if (MCAN_CLK_HZ % (brp * bitrate) != 0)
        {
          continue;
        }

      ntq = MCAN_CLK_HZ / (brp * bitrate);
      if (ntq > 1 + seg1max + seg2max)
        {
          continue;
        }

      if (ntq < 5)
        {
          break;
        }

      /* The sample point is at the end of the sync quantum plus TSEG1 */

      seg1 = (ntq * samplep + 50) / 100 - 1;
      seg2 = ntq - 1 - seg1;
      if (seg2 > seg2max)
        {
          seg2 = seg2max;
          seg1 = ntq - 1 - seg2;
        }
      else if (seg2 < 2)
        {
          seg2 = 2;
          seg1 = ntq - 1 - seg2;
        }

      if (seg1 < 2 || seg1 > seg1max)
        {
          continue;
        }

      t->brp   = brp;
      t->tseg1 = seg1;
      t->tseg2 = seg2;
      t->sjw   = MIN(seg2, sjwmax);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: mcan_waitinit
 *
 * Description:
 *   Wait for CCCR.INIT to read back as requested.
 *
 ****************************************************************************/

static int mcan_waitinit(FAR struct am263x_mcan_s *priv, bool init)
{
  int i;

  for (i = 0; i < MCAN_TIMEOUT; i++)
    {
      if (((mcan_getreg(priv, AM263X_MCAN_CCCR_OFFSET) &
            MCAN_CCCR_INIT) != 0) == init)
        {
          return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: mcan_txfree
 *
 * Description:
 *   Drop every packet held for TX.
 *
 ****************************************************************************/

static void mcan_txfree(FAR struct am263x_mcan_s *priv)
{
  int i;

  for (i = 0; i < MCAN_NTX; i++)
    {
      if (priv->txpkt[i] != NULL)
        {
          netpkt_free(&priv->dev, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
        }
    }

  priv->txbusy = 0;
}

/****************************************************************************
 * Name: mcan_setup
 *
 * Description:
 *   Configure the controller and start it on the bus.
 *
 ****************************************************************************/

static int mcan_setup(FAR struct am263x_mcan_s *priv)
{
  struct mcan_timing_s nt;
#ifdef CONFIG_NET_CAN_CANFD
  struct mcan_timing_s dt;
  uint32_t tdco;
#endif
  uint32_t cccr;
  uint32_t regval;
  int ret;
  int i;

  ret = mcan_bittiming(priv->nbitrate, priv->nsamplep,
                       MCAN_NBTP_NBRP_MAX, MCAN_NBTP_NTSEG1_MAX,
                       MCAN_NBTP_NTSEG2_MAX, MCAN_NBTP_NSJW_MAX, &nt);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d: no timing for %" PRIu32 " bit/s at %u%%\n",
             priv->port, priv->nbitrate, priv->nsamplep);
      return ret;
    }

#ifdef CONFIG_NET_CAN_CANFD
  ret = mcan_bittiming(priv->dbitrate, priv->dsamplep,
                       MCAN_DBTP_DBRP_MAX, MCAN_DBTP_DTSEG1_MAX,
                       MCAN_DBTP_DTSEG2_MAX, MCAN_DBTP_DSJW_MAX, &dt);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d: no data timing for %" PRIu32
             " bit/s at %u%%\n",
             priv->port, priv->dbitrate, priv->dsamplep);
      return ret;
    }
#endif

  /* Reset the module, so that every ifup starts with the error counters
   * at zero (the MCANSS soft reset leaves them), then wait for the
   * hardware to clear the message RAM.  am263x_clockconfig() left the
   * RCM unlocked.
   */

  putreg32(RCM_RST_ASSERT, AM263X_MSS_RCM_MCAN_RST_CTRL(priv->port));
  while (getreg32(AM263X_MSS_RCM_MCAN_RST_CTRL(priv->port)) !=
         RCM_RST_ASSERT)
    {
    }

  putreg32(RCM_RST_DEASSERT, AM263X_MSS_RCM_MCAN_RST_CTRL(priv->port));
  while (getreg32(AM263X_MSS_RCM_MCAN_RST_CTRL(priv->port)) !=
         RCM_RST_DEASSERT)
    {
    }

  for (i = 0; i < MCAN_TIMEOUT; i++)
    {
      if ((getreg32(priv->ss + AM263X_MCANSS_STAT_OFFSET) &
           (MCANSS_STAT_RESET | MCANSS_STAT_MEM_INIT_DONE)) ==
          MCANSS_STAT_MEM_INIT_DONE)
        {
          break;
        }
    }

  if (i == MCAN_TIMEOUT)
    {
      canerr("ERROR: MCAN%d: reset timed out\n", priv->port);
      return -ETIMEDOUT;
    }

  /* Initialisation mode, then allow configuration changes */

  mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET, MCAN_CCCR_INIT);
  ret = mcan_waitinit(priv, true);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d: no INIT (clock off?)\n", priv->port);
      return ret;
    }

  cccr = MCAN_CCCR_INIT | MCAN_CCCR_CCE;
  mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET, cccr);

#ifdef CONFIG_NET_CAN_CANFD
  cccr |= MCAN_CCCR_FDOE | MCAN_CCCR_BRSE;
#endif
#ifdef CONFIG_AM263X_MCAN_LOOPBACK
  /* Internal loopback: TX goes straight to RX and the pins stay
   * recessive.
   */

  cccr |= MCAN_CCCR_TEST | MCAN_CCCR_MON;
#endif
  mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET, cccr);
#ifdef CONFIG_AM263X_MCAN_LOOPBACK
  mcan_putreg(priv, AM263X_MCAN_TEST_OFFSET, MCAN_TEST_LBCK);
#endif

  /* Bit timing (the registers hold value - 1) */

  mcan_putreg(priv, AM263X_MCAN_NBTP_OFFSET,
              ((uint32_t)(nt.sjw - 1) << MCAN_NBTP_NSJW_SHIFT) |
              ((uint32_t)(nt.brp - 1) << MCAN_NBTP_NBRP_SHIFT) |
              ((uint32_t)(nt.tseg1 - 1) << MCAN_NBTP_NTSEG1_SHIFT) |
              ((uint32_t)(nt.tseg2 - 1) << MCAN_NBTP_NTSEG2_SHIFT));

#ifdef CONFIG_NET_CAN_CANFD
  /* Transmitter delay compensation above 1 Mbit/s: the secondary sample
   * point is the measured delay plus the sample point, in clock periods.
   */

  regval = ((uint32_t)(dt.brp - 1) << MCAN_DBTP_DBRP_SHIFT) |
           ((uint32_t)(dt.tseg1 - 1) << MCAN_DBTP_DTSEG1_SHIFT) |
           ((uint32_t)(dt.tseg2 - 1) << MCAN_DBTP_DTSEG2_SHIFT) |
           ((uint32_t)(dt.sjw - 1) << MCAN_DBTP_DSJW_SHIFT);

  tdco = dt.brp * (1 + dt.tseg1);
  if (priv->dbitrate > 1000000 && tdco <= MCAN_TDCR_MAX)
    {
      regval |= MCAN_DBTP_TDC;
      mcan_putreg(priv, AM263X_MCAN_TDCR_OFFSET,
                  tdco << MCAN_TDCR_TDCO_SHIFT);
    }
  else
    {
      mcan_putreg(priv, AM263X_MCAN_TDCR_OFFSET, 0);
    }

  mcan_putreg(priv, AM263X_MCAN_DBTP_OFFSET, regval);
#endif

  /* Filters.  Without any, frames that match none go to FIFO 0. */

  for (i = 0; i < priv->nsfe; i++)
    {
      putreg32(priv->sfe[i], mcan_ram(priv, MCAN_RAM_SFE + 4 * i));
    }

  for (i = 0; i < priv->nxfe; i++)
    {
      putreg32(priv->xfe[i][0], mcan_ram(priv, MCAN_RAM_XFE + 8 * i));
      putreg32(priv->xfe[i][1], mcan_ram(priv, MCAN_RAM_XFE + 8 * i + 4));
    }

  regval = 0;
  if (priv->nsfe > 0)
    {
      regval |= MCAN_GFC_ANF_REJECT << MCAN_GFC_ANFS_SHIFT;
    }

  if (priv->nxfe > 0)
    {
      regval |= MCAN_GFC_ANF_REJECT << MCAN_GFC_ANFE_SHIFT;
    }

#ifndef CONFIG_NET_CAN_EXTID
  regval |= MCAN_GFC_ANF_REJECT << MCAN_GFC_ANFE_SHIFT;
#endif
  mcan_putreg(priv, AM263X_MCAN_GFC_OFFSET, regval);

  mcan_putreg(priv, AM263X_MCAN_SIDFC_OFFSET,
              ((uint32_t)priv->nsfe << MCAN_SIDFC_LSS_SHIFT) |
              (MCAN_RAM_SFE << MCAN_SIDFC_FLSSA_SHIFT));
  mcan_putreg(priv, AM263X_MCAN_XIDFC_OFFSET,
              ((uint32_t)priv->nxfe << MCAN_XIDFC_LSE_SHIFT) |
              (MCAN_RAM_XFE << MCAN_XIDFC_FLESA_SHIFT));
  mcan_putreg(priv, AM263X_MCAN_XIDAM_OFFSET, MCAN_XFE_EFID_MASK);

  /* RX FIFO 0 (blocking: a full FIFO drops new frames) and the TX FIFO,
   * all elements with 64 data bytes.  No RX FIFO 1, dedicated buffers or
   * TX event FIFO.
   */

  mcan_putreg(priv, AM263X_MCAN_RXF0C_OFFSET,
              ((uint32_t)MCAN_NRX << MCAN_RXF0C_F0S_SHIFT) |
              (MCAN_RAM_RXF0 << MCAN_RXF0C_F0SA_SHIFT));
  mcan_putreg(priv, AM263X_MCAN_RXF1C_OFFSET, 0);
  mcan_putreg(priv, AM263X_MCAN_RXBC_OFFSET, 0);
  mcan_putreg(priv, AM263X_MCAN_RXESC_OFFSET,
              (MCAN_ESC_64BYTES << MCAN_RXESC_F0DS_SHIFT) |
              (MCAN_ESC_64BYTES << MCAN_RXESC_F1DS_SHIFT) |
              (MCAN_ESC_64BYTES << MCAN_RXESC_RBDS_SHIFT));

  mcan_putreg(priv, AM263X_MCAN_TXBC_OFFSET,
              ((uint32_t)MCAN_NTX << MCAN_TXBC_TFQS_SHIFT) |
              (MCAN_RAM_TXB << MCAN_TXBC_TBSA_SHIFT));
  mcan_putreg(priv, AM263X_MCAN_TXESC_OFFSET,
              MCAN_ESC_64BYTES << MCAN_TXESC_TBDS_SHIFT);
  mcan_putreg(priv, AM263X_MCAN_TXEFC_OFFSET, 0);
  mcan_putreg(priv, AM263X_MCAN_TXBTIE_OFFSET, 0xffffffff);

  /* Every interrupt on line 0 */

  mcan_putreg(priv, AM263X_MCAN_IR_OFFSET, MCAN_INT_ALL);
  mcan_putreg(priv, AM263X_MCAN_ILS_OFFSET, 0);
  mcan_putreg(priv, AM263X_MCAN_ILE_OFFSET, MCAN_ILE_EINT0);

  priv->txbusy    = 0;
  priv->errir     = 0;
  priv->errpsr    = 0;
  priv->nerrors   = 0;
  priv->noverruns = 0;
  mcan_setie(priv, MCAN_INT_DEFAULT, MCAN_INT_ALL);

  /* Leave initialisation: the controller joins the bus after 11
   * recessive bits.
   */

  cccr &= ~(MCAN_CCCR_INIT | MCAN_CCCR_CCE);
  mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET, cccr);
  ret = mcan_waitinit(priv, false);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d: stuck in INIT\n", priv->port);
      return ret;
    }

  caninfo("MCAN%d: %" PRIu32 " bit/s: brp %u tseg1 %u tseg2 %u sjw %u\n",
          priv->port, priv->nbitrate, nt.brp, nt.tseg1, nt.tseg2, nt.sjw);
#ifdef CONFIG_NET_CAN_CANFD
  caninfo("MCAN%d: data %" PRIu32 " bit/s: brp %u tseg1 %u tseg2 %u\n",
          priv->port, priv->dbitrate, dt.brp, dt.tseg1, dt.tseg2);
#endif
  return OK;
}

/****************************************************************************
 * Name: mcan_stop
 *
 * Description:
 *   Take the controller off the bus and silence it.
 *
 ****************************************************************************/

static void mcan_stop(FAR struct am263x_mcan_s *priv)
{
  mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET, MCAN_CCCR_INIT);
  mcan_waitinit(priv, true);

  mcan_setie(priv, 0, MCAN_INT_ALL);
  mcan_putreg(priv, AM263X_MCAN_ILE_OFFSET, 0);
  mcan_putreg(priv, AM263X_MCAN_IR_OFFSET, MCAN_INT_ALL);
}

/****************************************************************************
 * Name: mcan_interrupt
 ****************************************************************************/

static int mcan_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct am263x_mcan_s *priv = arg;
  irqstate_t flags;
  uint32_t ir;
  bool rx = false;

  flags = spin_lock_irqsave(&priv->lock);

  ir = mcan_getreg(priv, AM263X_MCAN_IR_OFFSET) & priv->ie;
  mcan_putreg(priv, AM263X_MCAN_IR_OFFSET, ir);

  if ((ir & MCAN_INT_RX) != 0)
    {
      priv->ie &= ~MCAN_INT_RX;
      rx = true;
    }

  if ((ir & MCAN_INT_BO) != 0)
    {
      /* Bus-off set INIT.  Clearing it starts the recovery sequence. */

      mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET,
                  mcan_getreg(priv, AM263X_MCAN_CCCR_OFFSET) &
                  ~MCAN_CCCR_INIT);
    }

  if ((ir & MCAN_INT_RF0L) != 0)
    {
      priv->noverruns++;
    }

  if ((ir & (MCAN_INT_PEA | MCAN_INT_PED)) != 0)
    {
      priv->nerrors++;
    }

#ifdef CONFIG_NET_CAN_ERRORS
  if ((ir & MCAN_INT_ERRORS) != 0)
    {
      /* Reading PSR clears its last error codes: keep them */

      priv->errir  |= ir & MCAN_INT_ERRORS;
      priv->errpsr |= mcan_getreg(priv, AM263X_MCAN_PSR_OFFSET);
      priv->ie     &= ~MCAN_INT_ERRORS;
      rx = true;
    }
#endif

  mcan_putreg(priv, AM263X_MCAN_IE_OFFSET, priv->ie);
  spin_unlock_irqrestore(&priv->lock, flags);

  if (rx)
    {
      netdev_lower_rxready(&priv->dev);
    }

  if ((ir & MCAN_INT_TC) != 0)
    {
      netdev_lower_txdone(&priv->dev);
    }

  return OK;
}

#ifdef CONFIG_NET_CAN_ERRORS
/****************************************************************************
 * Name: mcan_error
 *
 * Description:
 *   Turn the errors the interrupt noticed into an error frame and unmask
 *   the error interrupts again.
 *
 ****************************************************************************/

static FAR netpkt_t *mcan_error(FAR struct am263x_mcan_s *priv)
{
  FAR struct can_frame *frame;
  FAR netpkt_t *pkt;
  irqstate_t flags;
  uint32_t ir;
  uint32_t psr;
  uint32_t ecr;
  uint32_t id = 0;
  uint8_t tec;
  uint8_t rec;

  flags = spin_lock_irqsave(&priv->lock);
  ir  = priv->errir;
  psr = priv->errpsr;
  priv->errir  = 0;
  priv->errpsr = 0;
  spin_unlock_irqrestore(&priv->lock, flags);

  pkt = netpkt_alloc(&priv->dev, NETPKT_RX);
  if (pkt == NULL)
    {
      mcan_setie(priv, MCAN_INT_ERRORS, 0);
      return NULL;
    }

  netpkt_setdatalen(&priv->dev, pkt, sizeof(struct can_frame));
  frame = (FAR struct can_frame *)netpkt_getdata(&priv->dev, pkt);
  memset(frame, 0, sizeof(*frame));

  ecr = mcan_getreg(priv, AM263X_MCAN_ECR_OFFSET);
  tec = (ecr & MCAN_ECR_TEC_MASK) >> MCAN_ECR_TEC_SHIFT;
  rec = (ecr & MCAN_ECR_REC_MASK) >> MCAN_ECR_REC_SHIFT;
  if ((ecr & MCAN_ECR_RP) != 0)
    {
      rec = MCAN_ERR_PASSIVE;
    }

  if ((ir & MCAN_INT_BO) != 0)
    {
      id |= CAN_ERR_BUSOFF;
    }

  if ((ir & (MCAN_INT_EW | MCAN_INT_EP)) != 0)
    {
      id |= CAN_ERR_CRTL;
      if (tec >= MCAN_ERR_PASSIVE)
        {
          frame->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
        }
      else if (tec >= MCAN_ERR_WARNING)
        {
          frame->data[1] |= CAN_ERR_CRTL_TX_WARNING;
        }

      if (rec >= MCAN_ERR_PASSIVE)
        {
          frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
        }
      else if (rec >= MCAN_ERR_WARNING)
        {
          frame->data[1] |= CAN_ERR_CRTL_RX_WARNING;
        }
    }

  if ((ir & MCAN_INT_RF0L) != 0)
    {
      id |= CAN_ERR_CRTL;
      frame->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
    }

  if ((ir & (MCAN_INT_PEA | MCAN_INT_PED)) != 0)
    {
      id |= CAN_ERR_BUSERROR;
      switch ((psr & MCAN_PSR_LEC_MASK) >> MCAN_PSR_LEC_SHIFT)
        {
          case MCAN_PSR_LEC_STUFF:
            id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_STUFF;
            break;

          case MCAN_PSR_LEC_FORM:
            id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_FORM;
            break;

          case MCAN_PSR_LEC_ACK:
            id |= CAN_ERR_ACK;
            break;

          case MCAN_PSR_LEC_BIT1:
            id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_BIT1;
            break;

          case MCAN_PSR_LEC_BIT0:
            id |= CAN_ERR_PROT;
            frame->data[2] |= CAN_ERR_PROT_BIT0;
            break;

          case MCAN_PSR_LEC_CRC:
            id |= CAN_ERR_PROT;
            frame->data[3] = CAN_ERR_PROT_LOC_CRC_SEQ;
            break;

          default:
            break;
        }
    }

  id |= CAN_ERR_CNT;
  frame->data[6] = tec;
  frame->data[7] = rec;

  frame->can_id  = CAN_ERR_FLAG | id;
  frame->can_dlc = CAN_ERR_DLC;

  mcan_setie(priv, MCAN_INT_ERRORS, 0);
  return pkt;
}
#endif

/****************************************************************************
 * Name: mcan_ifup
 ****************************************************************************/

static int mcan_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_mcan_s *priv = (FAR struct am263x_mcan_s *)dev;
  int ret;

  ret = mcan_setup(priv);
  if (ret < 0)
    {
      mcan_stop(priv);
      return ret;
    }

  up_enable_irq(priv->irq);
  netdev_lower_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: mcan_ifdown
 ****************************************************************************/

static int mcan_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_mcan_s *priv = (FAR struct am263x_mcan_s *)dev;

  up_disable_irq(priv->irq);

  /* Cancel what is still queued, then stop */

  mcan_putreg(priv, AM263X_MCAN_TXBCR_OFFSET, 0xffffffff);
  mcan_stop(priv);
  mcan_txfree(priv);

  netdev_lower_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: mcan_reclaim
 *
 * Description:
 *   Free the packets of TX elements whose request is no longer pending:
 *   sent, or cancelled.
 *
 ****************************************************************************/

static void mcan_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_mcan_s *priv = (FAR struct am263x_mcan_s *)dev;
  uint32_t done;
  int i;

  done = priv->txbusy & ~mcan_getreg(priv, AM263X_MCAN_TXBRP_OFFSET);
  for (i = 0; done != 0; i++, done >>= 1)
    {
      if ((done & 1) != 0)
        {
          netpkt_free(dev, priv->txpkt[i], NETPKT_TX);
          priv->txpkt[i] = NULL;
          priv->txbusy &= ~(1u << i);
        }
    }
}

/****************************************************************************
 * Name: mcan_transmit
 ****************************************************************************/

static int mcan_transmit(FAR struct netdev_lowerhalf_s *dev,
                         FAR netpkt_t *pkt)
{
  FAR struct am263x_mcan_s *priv = (FAR struct am263x_mcan_s *)dev;
  FAR const uint8_t *data;
  unsigned int len;             /* Data bytes in the frame */
  unsigned int nbytes;          /* Data bytes on the wire */
  uintptr_t elem;
  uint32_t canid;
  uint32_t txfqs;
  uint32_t t0;
  uint32_t t1;
  uint32_t word;
  unsigned int put;
  unsigned int i;

  mcan_reclaim(dev);

  txfqs = mcan_getreg(priv, AM263X_MCAN_TXFQS_OFFSET);
  put   = (txfqs & MCAN_TXFQS_TFQPI_MASK) >> MCAN_TXFQS_TFQPI_SHIFT;
  if ((txfqs & MCAN_TXFQS_TFQF) != 0 || priv->txpkt[put] != NULL)
    {
      return -EBUSY;
    }

  len = netpkt_getdatalen(dev, pkt);
  if (len == sizeof(struct can_frame))
    {
      FAR struct can_frame *frame =
        (FAR struct can_frame *)netpkt_getdata(dev, pkt);

      if (frame->can_dlc > CAN_MAX_DLEN)
        {
          return -EINVAL;
        }

      canid  = frame->can_id;
      data   = frame->data;
      len    = frame->can_dlc;
      nbytes = frame->can_dlc;
      t0     = (canid & CAN_RTR_FLAG) != 0 ? MCAN_ELEM_RTR : 0;
      t1     = (uint32_t)frame->can_dlc << MCAN_ELEM_DLC_SHIFT;
    }
#ifdef CONFIG_NET_CAN_CANFD
  else if (len == sizeof(struct canfd_frame))
    {
      FAR struct canfd_frame *frame =
        (FAR struct canfd_frame *)netpkt_getdata(dev, pkt);
      uint8_t dlc;

      if (frame->len > CANFD_MAX_DLEN)
        {
          return -EINVAL;
        }

      /* The controller sends the whole DLC's worth: pad to it */

      dlc    = can_bytes2dlc(frame->len);
      canid  = frame->can_id;
      data   = frame->data;
      nbytes = can_dlc2bytes(dlc);
      t0     = (frame->flags & CANFD_ESI) != 0 ? MCAN_ELEM_ESI : 0;
      t1     = ((uint32_t)dlc << MCAN_ELEM_DLC_SHIFT) | MCAN_ELEM_FDF;
      if ((frame->flags & CANFD_BRS) != 0)
        {
          t1 |= MCAN_ELEM_BRS;
        }

      len = frame->len;
    }
#endif
  else
    {
      return -EINVAL;
    }

  if ((canid & CAN_EFF_FLAG) != 0)
    {
#ifdef CONFIG_NET_CAN_EXTID
      t0 |= MCAN_ELEM_XTD | (canid & CAN_EFF_MASK);
#else
      return -EINVAL;
#endif
    }
  else
    {
      t0 |= (canid & CAN_SFF_MASK) << MCAN_ELEM_STDID_SHIFT;
    }

  elem = mcan_ram(priv, MCAN_RAM_TXB + put * MCAN_ELEM_BYTES);
  putreg32(t0, elem);
  putreg32(t1, elem + 4);

  for (i = 0; i < nbytes; i += 4)
    {
      word  = (uint32_t)(i < len ? data[i] : 0);
      word |= (uint32_t)(i + 1 < len ? data[i + 1] : 0) << 8;
      word |= (uint32_t)(i + 2 < len ? data[i + 2] : 0) << 16;
      word |= (uint32_t)(i + 3 < len ? data[i + 3] : 0) << 24;
      putreg32(word, elem + 8 + i);
    }

  priv->txpkt[put] = pkt;
  priv->txbusy |= 1u << put;
  mcan_putreg(priv, AM263X_MCAN_TXBAR_OFFSET, 1u << put);
  return OK;
}

/****************************************************************************
 * Name: mcan_receive
 ****************************************************************************/

static FAR netpkt_t *mcan_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_mcan_s *priv = (FAR struct am263x_mcan_s *)dev;
  FAR netpkt_t *pkt;
  FAR uint8_t *data;
  uintptr_t elem;
  uint32_t rxf0s;
  uint32_t canid;
  uint32_t r0;
  uint32_t r1;
  uint32_t word;
  unsigned int nbytes;
  unsigned int gi;
  unsigned int i;
  uint8_t dlc;

#ifdef CONFIG_NET_CAN_ERRORS
  if (priv->errir != 0)
    {
      pkt = mcan_error(priv);
      if (pkt != NULL)
        {
          return pkt;
        }
    }
#endif

  for (; ; )
    {
      rxf0s = mcan_getreg(priv, AM263X_MCAN_RXF0S_OFFSET);
      if ((rxf0s & MCAN_RXF0S_F0FL_MASK) == 0)
        {
          /* Empty.  RF0N is still flagged if a frame came in since the
           * interrupt, so unmasking it cannot miss one.
           */

          mcan_setie(priv, MCAN_INT_RX, 0);
          return NULL;
        }

      gi   = (rxf0s & MCAN_RXF0S_F0GI_MASK) >> MCAN_RXF0S_F0GI_SHIFT;
      elem = mcan_ram(priv, MCAN_RAM_RXF0 + gi * MCAN_ELEM_BYTES);
      r0   = getreg32(elem);
      r1   = getreg32(elem + 4);
      dlc  = (r1 & MCAN_ELEM_DLC_MASK) >> MCAN_ELEM_DLC_SHIFT;

      if ((r0 & MCAN_ELEM_XTD) != 0)
        {
          canid = (r0 & MCAN_ELEM_ID_MASK) | CAN_EFF_FLAG;
        }
      else
        {
          canid = (r0 >> MCAN_ELEM_STDID_SHIFT) & CAN_SFF_MASK;
        }

#ifndef CONFIG_NET_CAN_EXTID
      if ((canid & CAN_EFF_FLAG) != 0)
        {
          mcan_putreg(priv, AM263X_MCAN_RXF0A_OFFSET, gi);
          continue;
        }
#endif

#ifndef CONFIG_NET_CAN_CANFD
      if ((r1 & MCAN_ELEM_FDF) != 0)
        {
          mcan_putreg(priv, AM263X_MCAN_RXF0A_OFFSET, gi);
          continue;
        }
#endif

      break;
    }

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      /* Leave the frame in the FIFO for the next new-frame interrupt */

      mcan_setie(priv, MCAN_INT_RX, 0);
      return NULL;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if ((r1 & MCAN_ELEM_FDF) != 0)
    {
      FAR struct canfd_frame *frame;

      netpkt_setdatalen(dev, pkt, sizeof(struct canfd_frame));
      frame = (FAR struct canfd_frame *)netpkt_getdata(dev, pkt);
      memset(frame, 0, sizeof(*frame));

      frame->can_id = canid;
      frame->len    = can_dlc2bytes(dlc);
      if ((r1 & MCAN_ELEM_BRS) != 0)
        {
          frame->flags |= CANFD_BRS;
        }

      if ((r0 & MCAN_ELEM_ESI) != 0)
        {
          frame->flags |= CANFD_ESI;
        }

      data   = frame->data;
      nbytes = frame->len;
    }
  else
#endif
    {
      FAR struct can_frame *frame;

      netpkt_setdatalen(dev, pkt, sizeof(struct can_frame));
      frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);
      memset(frame, 0, sizeof(*frame));

      /* A classic frame carries at most 8 bytes, also with DLC 9-15 */

      frame->can_id  = canid;
      frame->can_dlc = MIN(dlc, CAN_MAX_DLEN);
      if ((r0 & MCAN_ELEM_RTR) != 0)
        {
          frame->can_id |= CAN_RTR_FLAG;
        }

      data   = frame->data;
      nbytes = (r0 & MCAN_ELEM_RTR) != 0 ? 0 : frame->can_dlc;
    }

  for (i = 0; i < nbytes; i += 4)
    {
      word = getreg32(elem + 8 + i);
      data[i] = word & 0xff;
      if (i + 1 < nbytes)
        {
          data[i + 1] = (word >> 8) & 0xff;
        }

      if (i + 2 < nbytes)
        {
          data[i + 2] = (word >> 16) & 0xff;
        }

      if (i + 3 < nbytes)
        {
          data[i + 3] = (word >> 24) & 0xff;
        }
    }

  mcan_putreg(priv, AM263X_MCAN_RXF0A_OFFSET, gi);
  return pkt;
}

#ifdef CONFIG_NETDEV_IOCTL
#ifdef CONFIG_NETDEV_CAN_IOCTL
/****************************************************************************
 * Name: mcan_filter
 *
 * Description:
 *   Add or delete a filter element.  They take effect on the next ifup.
 *
 ****************************************************************************/

static int mcan_filter(FAR struct am263x_mcan_s *priv, bool ext, bool add,
                       FAR const struct can_ioctl_filter_s *req)
{
  uint32_t e0;
  uint32_t e1 = 0;
  uint32_t ft;
  int i;

  switch (req->ftype)
    {
      case CAN_FILTER_MASK:
        ft = MCAN_FT_CLASSIC;
        break;

      case CAN_FILTER_DUAL:
        ft = MCAN_FT_DUAL;
        break;

      case CAN_FILTER_RANGE:
        ft = MCAN_FT_RANGE;
        break;

      default:
        return -EINVAL;
    }

  if (!ext)
    {
      if (req->fid1 > CAN_SFF_MASK || req->fid2 > CAN_SFF_MASK)
        {
          return -EINVAL;
        }

      e0 = (ft << MCAN_SFE_SFT_SHIFT) |
           (MCAN_FEC_FIFO0 << MCAN_SFE_SFEC_SHIFT) |
           (req->fid1 << MCAN_SFE_SFID1_SHIFT) |
           (req->fid2 << MCAN_SFE_SFID2_SHIFT);

      for (i = 0; i < priv->nsfe && priv->sfe[i] != e0; i++)
        {
        }

      if (add)
        {
          if (i < priv->nsfe)
            {
              return OK;
            }

          if (priv->nsfe >= MCAN_NSTDFILTER)
            {
              return -ENOSPC;
            }

          priv->sfe[priv->nsfe++] = e0;
          return OK;
        }

      if (i == priv->nsfe)
        {
          return -ENOENT;
        }

      priv->nsfe--;
      memmove(&priv->sfe[i], &priv->sfe[i + 1],
              (priv->nsfe - i) * sizeof(priv->sfe[0]));
      return OK;
    }

#ifdef CONFIG_NET_CAN_EXTID
  if (req->fid1 > CAN_EFF_MASK || req->fid2 > CAN_EFF_MASK)
    {
      return -EINVAL;
    }

  e0 = ((uint32_t)MCAN_FEC_FIFO0 << MCAN_XFE_EFEC_SHIFT) | req->fid1;
  e1 = (ft << MCAN_XFE_EFT_SHIFT) | req->fid2;

  for (i = 0; i < priv->nxfe &&
       (priv->xfe[i][0] != e0 || priv->xfe[i][1] != e1); i++)
    {
    }

  if (add)
    {
      if (i < priv->nxfe)
        {
          return OK;
        }

      if (priv->nxfe >= MCAN_NEXTFILTER)
        {
          return -ENOSPC;
        }

      priv->xfe[priv->nxfe][0] = e0;
      priv->xfe[priv->nxfe][1] = e1;
      priv->nxfe++;
      return OK;
    }

  if (i == priv->nxfe)
    {
      return -ENOENT;
    }

  priv->nxfe--;
  memmove(&priv->xfe[i], &priv->xfe[i + 1],
          (priv->nxfe - i) * sizeof(priv->xfe[0]));
  return OK;
#else
  UNUSED(e1);
  return -EINVAL;
#endif
}
#endif /* CONFIG_NETDEV_CAN_IOCTL */

/****************************************************************************
 * Name: mcan_ioctl
 ****************************************************************************/

static int mcan_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                      unsigned long arg)
{
#ifdef CONFIG_NETDEV_CAN_IOCTL
  FAR struct am263x_mcan_s *priv = (FAR struct am263x_mcan_s *)dev;
  bool up = (dev->netdev.d_flags & IFF_UP) != 0;

  switch (cmd)
    {
      case SIOCGCANBITRATE:
        {
          FAR struct can_ioctl_data_s *req =
            (FAR struct can_ioctl_data_s *)((uintptr_t)arg);

          req->arbi_bitrate = priv->nbitrate;
          req->arbi_samplep = priv->nsamplep;
#ifdef CONFIG_NET_CAN_CANFD
          req->data_bitrate = priv->dbitrate;
          req->data_samplep = priv->dsamplep;
#else
          req->data_bitrate = 0;
          req->data_samplep = 0;
#endif
          return OK;
        }

      case SIOCSCANBITRATE:
        {
          FAR struct can_ioctl_data_s *req =
            (FAR struct can_ioctl_data_s *)((uintptr_t)arg);
          struct mcan_timing_s t;
          uint16_t nsp = req->arbi_samplep ? req->arbi_samplep :
                         priv->nsamplep;
#ifdef CONFIG_NET_CAN_CANFD
          uint16_t dsp = req->data_samplep ? req->data_samplep :
                         priv->dsamplep;
          uint32_t dbr = req->data_bitrate ? req->data_bitrate :
                         req->arbi_bitrate;
#endif
          int ret;

          /* The upper half only gets here with the interface down */

          ret = mcan_bittiming(req->arbi_bitrate, nsp,
                               MCAN_NBTP_NBRP_MAX, MCAN_NBTP_NTSEG1_MAX,
                               MCAN_NBTP_NTSEG2_MAX, MCAN_NBTP_NSJW_MAX,
                               &t);
          if (ret < 0)
            {
              return ret;
            }

#ifdef CONFIG_NET_CAN_CANFD
          ret = mcan_bittiming(dbr, dsp,
                               MCAN_DBTP_DBRP_MAX, MCAN_DBTP_DTSEG1_MAX,
                               MCAN_DBTP_DTSEG2_MAX, MCAN_DBTP_DSJW_MAX,
                               &t);
          if (ret < 0)
            {
              return ret;
            }

          priv->dbitrate = dbr;
          priv->dsamplep = dsp;
#endif
          priv->nbitrate = req->arbi_bitrate;
          priv->nsamplep = nsp;
          return OK;
        }

      case SIOCACANSTDFILTER:
      case SIOCDCANSTDFILTER:
      case SIOCACANEXTFILTER:
      case SIOCDCANEXTFILTER:
        if (up)
          {
            return -EBUSY;
          }

        return mcan_filter(priv,
                           cmd == SIOCACANEXTFILTER ||
                           cmd == SIOCDCANEXTFILTER,
                           cmd == SIOCACANSTDFILTER ||
                           cmd == SIOCACANEXTFILTER,
                           (FAR const struct can_ioctl_filter_s *)
                           ((uintptr_t)arg));

      case SIOCGCANERRORS:
        {
          FAR struct can_ioctl_errors_s *req =
            (FAR struct can_ioctl_errors_s *)((uintptr_t)arg);
          uint32_t ecr = mcan_getreg(priv, AM263X_MCAN_ECR_OFFSET);
          uint32_t psr = mcan_getreg(priv, AM263X_MCAN_PSR_OFFSET);

          req->txerr = (ecr & MCAN_ECR_TEC_MASK) >> MCAN_ECR_TEC_SHIFT;
          req->rxerr = (ecr & MCAN_ECR_REC_MASK) >> MCAN_ECR_REC_SHIFT;
          if ((psr & MCAN_PSR_BO) != 0)
            {
              req->state = CAN_ERRSTATE_BUSOFF;
            }
          else if ((psr & MCAN_PSR_EP) != 0)
            {
              req->state = CAN_ERRSTATE_PASSIVE;
            }
          else if ((psr & MCAN_PSR_EW) != 0)
            {
              req->state = CAN_ERRSTATE_WARNING;
            }
          else
            {
              req->state = CAN_ERRSTATE_ACTIVE;
            }

          req->errors      = priv->nerrors;
          req->rx_overruns = priv->noverruns;
          return OK;
        }

      case SIOCCANRECOVERY:
        if (!up)
          {
            return -ENETDOWN;
          }

        mcan_putreg(priv, AM263X_MCAN_CCCR_OFFSET,
                    mcan_getreg(priv, AM263X_MCAN_CCCR_OFFSET) &
                    ~MCAN_CCCR_INIT);
        return OK;

      case SIOCCANOFLUSH:
        if (up)
          {
            mcan_putreg(priv, AM263X_MCAN_TXBCR_OFFSET, priv->txbusy);
          }

        return OK;

      default:
        break;
    }
#endif /* CONFIG_NETDEV_CAN_IOCTL */

  return -ENOTTY;
}
#endif /* CONFIG_NETDEV_IOCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_mcan_initialize
 ****************************************************************************/

int am263x_mcan_initialize(int port)
{
  FAR struct am263x_mcan_s *priv;
  FAR struct netdev_lowerhalf_s *dev;
  int ret;

  switch (port)
    {
#ifdef CONFIG_AM263X_MCAN0
      case 0:
#endif
#ifdef CONFIG_AM263X_MCAN1
      case 1:
#endif
#ifdef CONFIG_AM263X_MCAN2
      case 2:
#endif
#ifdef CONFIG_AM263X_MCAN3
      case 3:
#endif
        break;

      default:
        return -ENODEV;
    }

  priv = &g_mcan[port];
  dev  = &priv->dev;
  if (dev->ops != NULL)
    {
      return -EBUSY;
    }

  priv->core     = AM263X_MCAN_CORE(port);
  priv->ss       = AM263X_MCAN_SS(port);
  priv->ram      = AM263X_MCAN_MSGRAM(port);
  priv->port     = port;
  priv->irq      = AM263X_IRQ_MCAN(port);
  priv->nbitrate = CONFIG_AM263X_MCAN_BITRATE;
  priv->nsamplep = CONFIG_AM263X_MCAN_SAMPLEP;
#ifdef CONFIG_NET_CAN_CANFD
  priv->dbitrate = CONFIG_AM263X_MCAN_DATA_BITRATE;
  priv->dsamplep = CONFIG_AM263X_MCAN_DATA_SAMPLEP;
#endif
  spin_lock_init(&priv->lock);

  /* Quiet until ifup.  The core is readable once its clock runs. */

  mcan_stop(priv);

  ret = irq_attach(priv->irq, mcan_interrupt, priv);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d: irq_attach(%d) failed: %d\n",
             port, priv->irq, ret);
      return ret;
    }

  up_set_irq_type(priv->irq, IRQ_HIGH_LEVEL);

  /* can<port>, whichever ports are enabled */

  snprintf(dev->netdev.d_ifname, IFNAMSIZ, "can%d", port);

  dev->ops = &g_mcan_ops;
  dev->quota[NETPKT_TX] = MCAN_TX_QUOTA;
  dev->quota[NETPKT_RX] = MCAN_RX_QUOTA;

  ret = netdev_lower_register(dev, NET_LL_CAN);
  if (ret < 0)
    {
      canerr("ERROR: MCAN%d: netdev_lower_register failed: %d\n",
             port, ret);
      irq_detach(priv->irq);
      dev->ops = NULL;
      return ret;
    }

  caninfo("MCAN%d: core release %08" PRIx32 "\n",
          port, mcan_getreg(priv, AM263X_MCAN_CREL_OFFSET));
  return OK;
}

#endif /* CONFIG_AM263X_MCAN */
