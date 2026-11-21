/****************************************************************************
 * arch/arm/src/am263x/am263x_lin.c
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

/* The SCI/LIN modules as SocketCAN LIN commanders, with the frame format
 * of include/nuttx/lin.h:
 *
 *   - Each frame sent is one LIN frame: the low 6 bits of can_id are the
 *     LIN ID.  With CAN_RTR_FLAG the commander sends the header and reads
 *     can_dlc bytes from a responder; without it, it sends the header and
 *     the can_dlc data bytes itself.  A can_dlc of 0 takes the length from
 *     the ID (2, 2, 4 or 8 bytes, as in LIN 1.x).
 *   - LIN_CHECKSUM_EXTENDED selects the enhanced checksum, else classic.
 *   - A read returns the response as a frame with the ID.  A write that
 *     completes returns its bytes as read back from the bus, with
 *     LIN_TCF_FLAG.  A failed frame returns a LIN_ERR_FLAG frame (with
 *     NET_CAN_ERRORS): the error in data[0]-data[4] as in lin.h, the LIN
 *     ID in data[5], and CAN error classes in can_id (CAN_ERR_TX_TIMEOUT,
 *     CAN_ERR_ACK for no response, CAN_ERR_PROT, CAN_ERR_BUSERROR,
 *     CAN_ERR_CRTL) for CAN_RAW_ERR_FILTER.
 *
 * Frames run one at a time.  A frame is done when its response has been
 * received (a commander receives its own response back), on an error, or
 * on a timeout of twice the frame's maximum length.
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
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/lin.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/net/ioctl.h>
#include <nuttx/net/netdev_lowerhalf.h>

#include "arm_internal.h"
#include "hardware/am263x_lin.h"
#include "am263x_lin.h"

#ifdef CONFIG_AM263X_LIN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Functional clock: PER_HSDIV0_CLKOUT1 (192 MHz) / 4, set up in
 * am263x_clockconfig().
 */

#define LIN_CLK_HZ           48000000

#define LIN_MIN_BITRATE      1000
#define LIN_MAX_BITRATE      20000

/* Results waiting for receive(): responses, TX confirmations, errors */

#define LIN_NRESULTS         4

#define LIN_TX_QUOTA         1
#define LIN_RX_QUOTA         LIN_NRESULTS

/* What ends a frame */

#define LIN_INT_ERRORS       (LIN_INT_PE | LIN_INT_OE | LIN_INT_FE | \
                              LIN_INT_NRE | LIN_INT_ISFE | LIN_INT_CE | \
                              LIN_INT_PBE | LIN_INT_BE)
#define LIN_INT_DEFAULT      (LIN_INT_RX | LIN_INT_ERRORS)

/* Header: 13-bit break, 1-bit delimiter */

#define LIN_SBREAK           0
#define LIN_SDEL             0

/* Bit times to wait for the previous frame to end before a header */

#define LIN_MAX_WAITS        20

/* A frame is at most 1.4 x (34 + 10 x (bytes + 1)) bits long */

#define LIN_FRAME_BITS(n)    (34 + 10 * ((n) + 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct am263x_lin_s
{
  struct netdev_lowerhalf_s dev; /* Must be first */

  uintptr_t base;
  uint8_t   port;
  uint8_t   irq;
  spinlock_t lock;              /* Frame state and the result ring */
  uint32_t  bitrate;
  uint32_t  gcr1;               /* SCIGCR1 in use */
  struct wdog_s wdog;           /* Frame timeout */

  /* The frame on the bus */

  FAR netpkt_t *txpkt;          /* Held until the frame is done */
  bool      busy;
  bool      read;               /* Header only, a responder answers */
  uint8_t   id;                 /* 6-bit ID */
  uint8_t   dlc;
  canid_t   flags;              /* LIN_CHECKSUM_EXTENDED */
  uint32_t  linid;              /* LINID for the header */
  uint8_t   waits;              /* For BUSY, LIN_MAX_WAITS once sent */
  uint32_t  td[2];              /* LINTD0-1 for a write */

  /* Results for receive() */

  struct can_frame result[LIN_NRESULTS];
  uint8_t   head;
  uint8_t   count;

  uint32_t  nerrors;            /* Failed frames since ifup */
  uint32_t  noverruns;          /* Results lost, the ring was full */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  lin_ifup(FAR struct netdev_lowerhalf_s *dev);
static int  lin_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int  lin_transmit(FAR struct netdev_lowerhalf_s *dev,
                         FAR netpkt_t *pkt);
static FAR netpkt_t *lin_receive(FAR struct netdev_lowerhalf_s *dev);
#ifdef CONFIG_NETDEV_IOCTL
static int  lin_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                      unsigned long arg);
#endif
static void lin_reclaim(FAR struct netdev_lowerhalf_s *dev);
static void lin_timeout(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_lin_ops =
{
  .ifup     = lin_ifup,
  .ifdown   = lin_ifdown,
  .transmit = lin_transmit,
  .receive  = lin_receive,
#ifdef CONFIG_NETDEV_IOCTL
  .ioctl    = lin_ioctl,
#endif
  .reclaim  = lin_reclaim,
};

static struct am263x_lin_s g_lin[AM263X_LIN_NINSTANCES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lin_getreg / lin_putreg
 ****************************************************************************/

static inline uint32_t lin_getreg(FAR struct am263x_lin_s *priv,
                                  unsigned int offset)
{
  return getreg32(priv->base + offset);
}

static inline void lin_putreg(FAR struct am263x_lin_s *priv,
                              unsigned int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: lin_pid
 *
 * Description:
 *   The protected identifier: the ID with its two parity bits.
 *
 ****************************************************************************/

static uint8_t lin_pid(uint8_t id)
{
  uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 1;
  uint8_t p1 = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 1;

  return (id & LIN_ID_MASK) | (p0 << 6) | (p1 << 7);
}

/****************************************************************************
 * Name: lin_brsr
 *
 * Description:
 *   BRSR for a bit rate: clock / bitrate = 16 x (P + 1) + M.
 *
 ****************************************************************************/

static uint32_t lin_brsr(uint32_t bitrate)
{
  uint32_t n = (LIN_CLK_HZ + bitrate / 2) / bitrate;

  return ((n / 16 - 1) << LIN_BRSR_P_SHIFT) |
         ((n % 16) << LIN_BRSR_M_SHIFT);
}

/****************************************************************************
 * Name: lin_setup
 *
 * Description:
 *   Reset the module and configure it as a commander, the way TI's
 *   LIN_initModule() does.  It leaves the module ready, interrupts on.
 *
 ****************************************************************************/

static void lin_setup(FAR struct am263x_lin_s *priv)
{
  lin_putreg(priv, AM263X_LIN_SCIGCR0_OFFSET, 0);
  lin_putreg(priv, AM263X_LIN_SCIGCR0_OFFSET, LIN_SCIGCR0_RESET);
  lin_putreg(priv, AM263X_LIN_SCIPIO0_OFFSET,
             LIN_SCIPIO0_RXFUNC | LIN_SCIPIO0_TXFUNC);

  /* Configuration state.  Fixed bit rate, frame length from SCIFORMAT,
   * the 8-byte buffers, ID parity.  The responder-task ID byte decides
   * which frames this node sends the response of: with the TX mask at 0
   * only an exact match, with the RX mask at 0xff it receives every
   * response, its own included.
   */

  priv->gcr1 = LIN_SCIGCR1_LINMODE | LIN_SCIGCR1_CLK_MASTER |
               LIN_SCIGCR1_CONT | LIN_SCIGCR1_HGENCTRL |
               LIN_SCIGCR1_MBUFMODE | LIN_SCIGCR1_PARITYENA |
               LIN_SCIGCR1_RXENA | LIN_SCIGCR1_TXENA;
#ifdef CONFIG_AM263X_LIN_LOOPBACK
  priv->gcr1 |= LIN_SCIGCR1_LOOPBACK;
#endif
  lin_putreg(priv, AM263X_LIN_SCIGCR1_OFFSET, priv->gcr1);

  lin_putreg(priv, AM263X_LIN_BRSR_OFFSET, lin_brsr(priv->bitrate));
  lin_putreg(priv, AM263X_LIN_MBRSR_OFFSET, LIN_CLK_HZ / LIN_MAX_BITRATE);
  lin_putreg(priv, AM263X_LIN_SCIFORMAT_OFFSET,
             7 << LIN_SCIFORMAT_CHAR_SHIFT);
  lin_putreg(priv, AM263X_LIN_LINCOMP_OFFSET,
             (LIN_SBREAK << LIN_LINCOMP_SBREAK_SHIFT) |
             (LIN_SDEL << LIN_LINCOMP_SDEL_SHIFT));
  lin_putreg(priv, AM263X_LIN_LINMASK_OFFSET,
             0xff << LIN_LINMASK_RXIDMASK_SHIFT);
  lin_putreg(priv, AM263X_LIN_IODFTCTRL_OFFSET, 0);

  /* Every interrupt on line 0 */

  lin_putreg(priv, AM263X_LIN_SCICLRINT_OFFSET, LIN_INT_ALL);
  lin_putreg(priv, AM263X_LIN_SCICLRLVL_OFFSET, LIN_INT_ALL);
  lin_putreg(priv, AM263X_LIN_SCIFLR_OFFSET, LIN_INT_ALL);
  lin_putreg(priv, AM263X_LIN_SCISETINT_OFFSET, LIN_INT_DEFAULT);
  lin_putreg(priv, AM263X_LIN_GLBINTCLR_OFFSET,
             LIN_GLBINT_LINE0 | LIN_GLBINT_LINE1);
  lin_putreg(priv, AM263X_LIN_GLBINTEN_OFFSET, LIN_GLBINT_LINE0);

  /* Ready */

  priv->gcr1 |= LIN_SCIGCR1_SWNRST;
  lin_putreg(priv, AM263X_LIN_SCIGCR1_OFFSET, priv->gcr1);
}

/****************************************************************************
 * Name: lin_setgcr1
 *
 * Description:
 *   Change SCIGCR1, through the configuration state.  Only between
 *   frames: the state machine restarts.
 *
 ****************************************************************************/

static void lin_setgcr1(FAR struct am263x_lin_s *priv, uint32_t gcr1)
{
  lin_putreg(priv, AM263X_LIN_SCIGCR1_OFFSET, gcr1 & ~LIN_SCIGCR1_SWNRST);
  lin_putreg(priv, AM263X_LIN_SCIGCR1_OFFSET, gcr1 | LIN_SCIGCR1_SWNRST);
  priv->gcr1 = gcr1 | LIN_SCIGCR1_SWNRST;
}

/****************************************************************************
 * Name: lin_result
 *
 * Description:
 *   A slot at the end of the result ring, or NULL if it is full.  Called
 *   with the lock held.
 *
 ****************************************************************************/

static FAR struct can_frame *lin_result(FAR struct am263x_lin_s *priv)
{
  FAR struct can_frame *frame;

  if (priv->count >= LIN_NRESULTS)
    {
      priv->noverruns++;
      return NULL;
    }

  frame = &priv->result[(priv->head + priv->count) % LIN_NRESULTS];
  priv->count++;
  memset(frame, 0, sizeof(*frame));
  return frame;
}

/****************************************************************************
 * Name: lin_readbuf
 ****************************************************************************/

static void lin_readbuf(FAR struct am263x_lin_s *priv, FAR uint8_t *data,
                        unsigned int n)
{
  uint32_t rd[2];
  unsigned int i;

  rd[0] = lin_getreg(priv, AM263X_LIN_LINRD0_OFFSET);
  rd[1] = lin_getreg(priv, AM263X_LIN_LINRD1_OFFSET);

  /* Byte 0 is the most significant byte of LINRD0 */

  for (i = 0; i < n; i++)
    {
      data[i] = rd[i / 4] >> (24 - 8 * (i % 4));
    }
}

/****************************************************************************
 * Name: lin_error
 *
 * Description:
 *   Fill in an error frame for flags that ended the frame (0: timeout).
 *
 ****************************************************************************/

#ifdef CONFIG_NET_CAN_ERRORS
static void lin_error(FAR struct am263x_lin_s *priv,
                      FAR struct can_frame *frame, uint32_t flr)
{
  FAR uint8_t *data = frame->data;
  canid_t id = LIN_ERR_FLAG;

  frame->can_dlc = CAN_ERR_DLC;
  data[5]        = priv->id;

  if (flr == 0)
    {
      if (priv->read)
        {
          data[0] |= LIN_ERR_RX;
          data[2] |= LIN_ERR_RX_NO_RESPONSE;
        }
      else
        {
          data[0] |= LIN_ERR_TX;
          data[1] |= LIN_ERR_TX_DATA_TMO;
        }
    }

  if ((flr & LIN_INT_PBE) != 0)
    {
      /* The break did not come back: no bus, or it is stuck */

      data[0] |= LIN_ERR_TX;
      data[1] |= LIN_ERR_TX_BREAK_TMO;
    }

  if ((flr & LIN_INT_NRE) != 0)
    {
      data[0] |= LIN_ERR_RX;
      data[2] |= LIN_ERR_RX_NO_RESPONSE;
    }

  if ((flr & LIN_INT_CE) != 0)
    {
      if (priv->read)
        {
          data[0] |= LIN_ERR_RX;
          data[2] |= LIN_ERR_RX_CKSUM;
        }
      else
        {
          data[0] |= LIN_ERR_BUS;
          data[3] |= LIN_ERR_BUS_TXCKSUM;
        }
    }

  if ((flr & LIN_INT_BE) != 0)
    {
      /* What was read back differs from what was sent */

      data[0] |= LIN_ERR_BUS;
      data[3] |= priv->read ? LIN_ERR_BUS_PID : LIN_ERR_BUS_DATA;
    }

  if ((flr & LIN_INT_ISFE) != 0)
    {
      data[0] |= LIN_ERR_BUS;
      data[3] |= LIN_ERR_BUS_SYNC;
    }

  if ((flr & LIN_INT_PE) != 0)
    {
      data[0] |= LIN_ERR_BUS;
      data[3] |= LIN_ERR_BUS_PID;
    }

  if ((flr & LIN_INT_FE) != 0)
    {
      data[0] |= LIN_ERR_CRTL;
      data[4] |= LIN_ERR_CTRL_FRAMEERROR;
    }

  if ((flr & LIN_INT_OE) != 0)
    {
      data[0] |= LIN_ERR_CRTL;
      data[4] |= LIN_ERR_CRTL_RXOVERFLOW;
    }

  /* SocketCAN classes in can_id too, for CAN_RAW_ERR_FILTER */

  if ((data[0] & LIN_ERR_TX) != 0)
    {
      id |= CAN_ERR_TX_TIMEOUT;
    }

  if ((data[2] & LIN_ERR_RX_NO_RESPONSE) != 0)
    {
      id |= CAN_ERR_ACK;
    }

  if ((data[2] & ~LIN_ERR_RX_NO_RESPONSE) != 0)
    {
      id |= CAN_ERR_PROT;
    }

  if ((data[0] & LIN_ERR_BUS) != 0)
    {
      id |= CAN_ERR_BUSERROR;
    }

  if ((data[0] & LIN_ERR_CRTL) != 0)
    {
      id |= CAN_ERR_CRTL;
    }

  frame->can_id = id;
}
#endif

/****************************************************************************
 * Name: lin_done
 *
 * Description:
 *   End the frame on the bus: queue its result and hand the TX slot back.
 *   flr holds the flags that ended it; 0 is a timeout.  Called with the
 *   lock held.
 *
 ****************************************************************************/

static void lin_done(FAR struct am263x_lin_s *priv, uint32_t flr)
{
  FAR struct can_frame *frame;

  wd_cancel(&priv->wdog);
  priv->busy = false;

  if ((flr & LIN_INT_ERRORS) != 0 || flr == 0)
    {
      priv->nerrors++;

      /* Restart the state machine, which may be mid-frame */

      lin_setgcr1(priv, priv->gcr1);
      lin_putreg(priv, AM263X_LIN_SCIFLR_OFFSET, LIN_INT_ALL);

#ifdef CONFIG_NET_CAN_ERRORS
      frame = lin_result(priv);
      if (frame != NULL)
        {
          lin_error(priv, frame, flr);
        }
#endif
    }
  else
    {
      frame = lin_result(priv);
      if (frame != NULL)
        {
          frame->can_id  = priv->id | priv->flags;
          frame->can_dlc = priv->dlc;
          if (!priv->read)
            {
              frame->can_id |= LIN_TCF_FLAG;
            }

          lin_readbuf(priv, frame->data, priv->dlc);
        }
    }
}

/****************************************************************************
 * Name: lin_start
 *
 * Description:
 *   Load the frame set up by lin_transmit() and send its header.  The
 *   previous frame's response has been received before its last stop bit
 *   has left the transmitter, and until it has, writes to the TX buffers
 *   and the ID byte are lost.  So wait for TXEMPTY, a bit time at a time.
 *   Called with the lock held.
 *
 ****************************************************************************/

static void lin_start(FAR struct am263x_lin_s *priv)
{
  uint32_t flr = lin_getreg(priv, AM263X_LIN_SCIFLR_OFFSET);
  uint32_t gcr1;

  if (((flr & LIN_FLR_TXEMPTY) == 0 || (flr & LIN_FLR_BUSY) != 0) &&
      priv->waits < LIN_MAX_WAITS)
    {
      priv->waits++;
      wd_start(&priv->wdog, USEC2TICK(1000000 / priv->bitrate) + 1,
               lin_timeout, (wdparm_t)priv);
      return;
    }

  priv->waits = LIN_MAX_WAITS;

  gcr1 = priv->gcr1 & ~LIN_SCIGCR1_CTYPE;
  if (priv->flags != 0)
    {
      gcr1 |= LIN_SCIGCR1_CTYPE;
    }

  if (gcr1 != priv->gcr1)
    {
      lin_setgcr1(priv, gcr1);
    }

  lin_putreg(priv, AM263X_LIN_SCIFORMAT_OFFSET,
             (7 << LIN_SCIFORMAT_CHAR_SHIFT) |
             ((uint32_t)(priv->dlc - 1) << LIN_SCIFORMAT_LENGTH_SHIFT));

  if (!priv->read)
    {
      lin_putreg(priv, AM263X_LIN_LINTD0_OFFSET, priv->td[0]);
      lin_putreg(priv, AM263X_LIN_LINTD1_OFFSET, priv->td[1]);
    }

  lin_putreg(priv, AM263X_LIN_SCIFLR_OFFSET, LIN_INT_ALL);

  /* Writing the ID byte sends the header */

  lin_putreg(priv, AM263X_LIN_LINID_OFFSET, priv->linid);

  wd_start(&priv->wdog,
           USEC2TICK(LIN_FRAME_BITS(priv->dlc) * 2800000ull /
                     priv->bitrate) + 2,
           lin_timeout, (wdparm_t)priv);
}

/****************************************************************************
 * Name: lin_timeout
 *
 * Description:
 *   Waiting to send the header, or the frame has timed out.
 *
 ****************************************************************************/

static void lin_timeout(wdparm_t arg)
{
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)arg;
  irqstate_t flags;
  bool done = false;

  flags = spin_lock_irqsave(&priv->lock);
  if (priv->busy && priv->waits < LIN_MAX_WAITS)
    {
      lin_start(priv);
    }
  else if (priv->busy)
    {
      lin_done(priv, 0);
      done = true;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (done)
    {
      netdev_lower_rxready(&priv->dev);
      netdev_lower_txdone(&priv->dev);
    }
}

/****************************************************************************
 * Name: lin_interrupt
 ****************************************************************************/

static int lin_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct am263x_lin_s *priv = arg;
  irqstate_t flags;
  uint32_t flr;
  bool done = false;

  flags = spin_lock_irqsave(&priv->lock);

  flr = lin_getreg(priv, AM263X_LIN_SCIFLR_OFFSET) & LIN_INT_DEFAULT;
  lin_putreg(priv, AM263X_LIN_SCIFLR_OFFSET, flr);
  lin_putreg(priv, AM263X_LIN_GLBINTCLR_OFFSET, LIN_GLBINT_LINE0);

  if (priv->busy && priv->waits >= LIN_MAX_WAITS && flr != 0)
    {
      lin_done(priv, flr);
      done = true;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  if (done)
    {
      netdev_lower_rxready(&priv->dev);
      netdev_lower_txdone(&priv->dev);
    }

  return OK;
}

/****************************************************************************
 * Name: lin_stop
 *
 * Description:
 *   Hold the module in reset: TX goes back to a recessive input.
 *
 ****************************************************************************/

static void lin_stop(FAR struct am263x_lin_s *priv)
{
  irqstate_t flags;

  wd_cancel(&priv->wdog);
  lin_putreg(priv, AM263X_LIN_GLBINTEN_OFFSET, 0);
  lin_putreg(priv, AM263X_LIN_SCIGCR0_OFFSET, 0);

  flags = spin_lock_irqsave(&priv->lock);
  priv->busy  = false;
  priv->head  = 0;
  priv->count = 0;
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: lin_ifup
 ****************************************************************************/

static int lin_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)dev;

  priv->nerrors   = 0;
  priv->noverruns = 0;
  lin_setup(priv);

  up_enable_irq(priv->irq);
  netdev_lower_carrier_on(dev);
  return OK;
}

/****************************************************************************
 * Name: lin_ifdown
 ****************************************************************************/

static int lin_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)dev;

  up_disable_irq(priv->irq);
  lin_stop(priv);

  if (priv->txpkt != NULL)
    {
      netpkt_free(dev, priv->txpkt, NETPKT_TX);
      priv->txpkt = NULL;
    }

  netdev_lower_carrier_off(dev);
  return OK;
}

/****************************************************************************
 * Name: lin_reclaim
 ****************************************************************************/

static void lin_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)dev;

  if (priv->txpkt != NULL && !priv->busy)
    {
      netpkt_free(dev, priv->txpkt, NETPKT_TX);
      priv->txpkt = NULL;
    }
}

/****************************************************************************
 * Name: lin_transmit
 ****************************************************************************/

static int lin_transmit(FAR struct netdev_lowerhalf_s *dev,
                        FAR netpkt_t *pkt)
{
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)dev;
  FAR struct can_frame *frame;
  irqstate_t flags;
  uint8_t pid;
  uint8_t match;
  unsigned int i;

  lin_reclaim(dev);
  if (priv->txpkt != NULL)
    {
      return -EBUSY;
    }

  frame = (FAR struct can_frame *)netpkt_getdata(dev, pkt);
  if (netpkt_getdatalen(dev, pkt) != sizeof(struct can_frame) ||
      frame->can_dlc > CAN_MAX_DLEN ||
      (frame->can_id & (CAN_ERR_FLAG | LIN_CACHE_RESPONSE)) != 0)
    {
      /* Not a commander frame: drop it */

      netpkt_free(dev, pkt, NETPKT_TX);
      return OK;
    }

  priv->id    = frame->can_id & LIN_ID_MASK;
  priv->read  = (frame->can_id & CAN_RTR_FLAG) != 0;
  priv->flags = frame->can_id & LIN_CHECKSUM_EXTENDED;
  priv->dlc   = frame->can_dlc;
  if (priv->dlc == 0)
    {
      static const uint8_t len[4] =
      {
        2, 2, 4, 8
      };

      priv->dlc = len[priv->id >> 4];
    }

  pid = lin_pid(priv->id);
  if (priv->read)
    {
      /* Never this node's response */

      match = pid ^ 0xff;
    }
  else
    {
      match = pid;
    }

  memset(priv->td, 0, sizeof(priv->td));
  for (i = 0; !priv->read && i < priv->dlc; i++)
    {
      priv->td[i / 4] |= (uint32_t)frame->data[i] << (24 - 8 * (i % 4));
    }

  priv->txpkt = pkt;
  priv->linid = ((uint32_t)match << LIN_LINID_IDSLAVE_SHIFT) |
                ((uint32_t)pid << LIN_LINID_IDBYTE_SHIFT);

  flags = spin_lock_irqsave(&priv->lock);
  priv->busy  = true;
  priv->waits = 0;
  lin_start(priv);
  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: lin_receive
 ****************************************************************************/

static FAR netpkt_t *lin_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)dev;
  FAR netpkt_t *pkt;
  irqstate_t flags;

  if (priv->count == 0)
    {
      return NULL;
    }

  pkt = netpkt_alloc(dev, NETPKT_RX);
  if (pkt == NULL)
    {
      return NULL;
    }

  flags = spin_lock_irqsave(&priv->lock);
  netpkt_copyin(dev, pkt, (FAR const uint8_t *)&priv->result[priv->head],
                sizeof(struct can_frame), 0);
  priv->head = (priv->head + 1) % LIN_NRESULTS;
  priv->count--;
  spin_unlock_irqrestore(&priv->lock, flags);

  return pkt;
}

/****************************************************************************
 * Name: lin_ioctl
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IOCTL
static int lin_ioctl(FAR struct netdev_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_NETDEV_CAN_IOCTL
  FAR struct am263x_lin_s *priv = (FAR struct am263x_lin_s *)dev;

  switch (cmd)
    {
      case SIOCGCANBITRATE:
        {
          FAR struct can_ioctl_data_s *req =
            (FAR struct can_ioctl_data_s *)((uintptr_t)arg);

          memset(req, 0, sizeof(*req));
          req->arbi_bitrate = priv->bitrate;
          return OK;
        }

      case SIOCSCANBITRATE:
        {
          FAR struct can_ioctl_data_s *req =
            (FAR struct can_ioctl_data_s *)((uintptr_t)arg);

          /* The upper half only gets here with the interface down */

          if (req->arbi_bitrate < LIN_MIN_BITRATE ||
              req->arbi_bitrate > LIN_MAX_BITRATE)
            {
              return -EINVAL;
            }

          priv->bitrate = req->arbi_bitrate;
          return OK;
        }

      case SIOCGCANERRORS:
        {
          FAR struct can_ioctl_errors_s *req =
            (FAR struct can_ioctl_errors_s *)((uintptr_t)arg);

          memset(req, 0, sizeof(*req));
          req->state       = CAN_ERRSTATE_ACTIVE;
          req->errors      = priv->nerrors;
          req->rx_overruns = priv->noverruns;
          return OK;
        }

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
 * Name: am263x_lin_initialize
 ****************************************************************************/

int am263x_lin_initialize(int port)
{
  FAR struct am263x_lin_s *priv;
  FAR struct netdev_lowerhalf_s *dev;
  int ret;

  switch (port)
    {
#ifdef CONFIG_AM263X_LIN0
      case 0:
#endif
#ifdef CONFIG_AM263X_LIN1
      case 1:
#endif
#ifdef CONFIG_AM263X_LIN2
      case 2:
#endif
#ifdef CONFIG_AM263X_LIN3
      case 3:
#endif
#ifdef CONFIG_AM263X_LIN4
      case 4:
#endif
        break;

      default:
        return -ENODEV;
    }

  priv = &g_lin[port];
  dev  = &priv->dev;
  if (dev->ops != NULL)
    {
      return -EBUSY;
    }

  priv->base    = AM263X_LIN_BASE(port);
  priv->port    = port;
  priv->irq     = AM263X_IRQ_LIN(port);
  priv->bitrate = CONFIG_AM263X_LIN_BITRATE;
  spin_lock_init(&priv->lock);

  /* Quiet until ifup */

  lin_stop(priv);

  ret = irq_attach(priv->irq, lin_interrupt, priv);
  if (ret < 0)
    {
      nerr("ERROR: LIN%d: irq_attach(%d) failed: %d\n",
           port, priv->irq, ret);
      return ret;
    }

  up_set_irq_type(priv->irq, IRQ_HIGH_LEVEL);

  snprintf(dev->netdev.d_ifname, IFNAMSIZ, "lin%d", port);

  dev->ops = &g_lin_ops;
  dev->quota[NETPKT_TX] = LIN_TX_QUOTA;
  dev->quota[NETPKT_RX] = LIN_RX_QUOTA;

  ret = netdev_lower_register(dev, NET_LL_CAN);
  if (ret < 0)
    {
      nerr("ERROR: LIN%d: netdev_lower_register failed: %d\n", port, ret);
      irq_detach(priv->irq);
      dev->ops = NULL;
      return ret;
    }

  return OK;
}

#endif /* CONFIG_AM263X_LIN */
