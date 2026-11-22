/****************************************************************************
 * arch/arm/src/am263x/am263x_cpsw.c
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

/* CPSW3G Ethernet, one MAC port, as a netdev lower half.
 *
 * The switch: the ALE is in bypass, so every frame the MAC port receives
 * goes to the host port, and the host sends each frame directed to the MAC
 * port (TO_PORT in the descriptor).  The other MAC port stays disabled.
 *
 * The DMA: one CPDMA TX and one RX channel, CPPI 3.0 descriptors in an
 * uncached MPU region (g_am263x_cpsw_desc), and packet buffers of the
 * driver's own in cached OCRAM.  Frames are copied between those buffers
 * and the network stack's IOBs.  Each queue is a NULL-terminated list:
 * the hardware sets EOQ on the descriptor it stopped at, and the driver
 * restarts the channel from the next one when it finds that.
 *
 * Interrupts: the CPDMA's RX and TX pulses.  RX masks its own interrupt
 * and leaves the work to the upper half, which calls receive() until it
 * returns NULL; that re-enables it.  TX completions are handled in the
 * interrupt, and the packets freed later in reclaim().  The upper half
 * only calls reclaim() from transmit() or when the TX quota is used up,
 * so the TX interrupt also queues am263x_cpsw_txwork() to do it: the
 * packets would otherwise keep up to CPSW_TX_QUOTA IOBs after a burst.
 *
 * The PHY: PHYLIB runs it; a work item follows its link state and sets
 * the MAC's speed and duplex to match.
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
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/cache.h>
#include <nuttx/ethtool.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/mdio.h>
#include <nuttx/net/netdev_lowerhalf.h>
#include <nuttx/net/phy.h>

#include "arm_internal.h"
#include <arch/barriers.h>
#include "hardware/am263x_cpsw.h"
#include "am263x_cpsw.h"
#include "am263x_mdio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_AM263X_CPSW_ETH) && !defined(CONFIG_NETDEV_LATEINIT)

#define CPSW_PORT          CONFIG_AM263X_CPSW_ETH_PORT
#define CPSW_OTHER_PORT    (3 - CPSW_PORT)
#define CPSW_NRXDESC       CONFIG_AM263X_CPSW_ETH_NRXDESC
#define CPSW_NTXDESC       CONFIG_AM263X_CPSW_ETH_NTXDESC

/* Channel 0 in both directions, core interrupt group 0 */

#define CPSW_CH            0
#define CPSW_CORE          0

/* Buffers hold a whole frame: 1500 bytes, the header, a VLAN tag and,
 * on receive, room the hardware never uses for the FCS (it is stripped).
 * A multiple of the cache line.
 */

#define CPSW_BUFSIZE       1536
#define CPSW_MAXFRAME      1522  /* With a VLAN tag and the FCS */
#define CPSW_MINFRAME      60    /* Without the FCS */

#define CPSW_CACHELINE     32
#define CPSW_ALIGN_UP(n) \
  (((n) + CPSW_CACHELINE - 1) & ~(CPSW_CACHELINE - 1))

/* Packets the driver may hold.  TX: one fewer than the descriptors, so
 * that the next descriptor's packet has always been freed (with all of
 * them held, the head would wrap onto a packet not yet freed).  RX: the
 * one between netpkt_alloc() and the upper half.
 */

#define CPSW_TX_QUOTA      (CPSW_NTXDESC - 1)
#define CPSW_RX_QUOTA      2

#define CPSW_LINK_POLL     MSEC2TICK(500)

#define CPSW_RESET_TIMEOUT 1000  /* us */

#if (CPSW_NRXDESC + CPSW_NTXDESC) * 16 > AM263X_CPSW_DESC_REGION_SIZE
#  error "Too many CPSW descriptors for AM263X_CPSW_DESC_REGION_SIZE"
#endif

#if CPSW_NRXDESC < 2 || CPSW_NTXDESC < 2
#  error "The CPSW needs at least two descriptors in each direction"
#endif

/* The MAC port's interface and delays */

#if defined(CONFIG_AM263X_CPSW_ETH_PHY_RGMII_ID)
#  define CPSW_PHY_INTERFACE PHY_INTERFACE_RGMII_ID
#elif defined(CONFIG_AM263X_CPSW_ETH_PHY_RGMII_TXID)
#  define CPSW_PHY_INTERFACE PHY_INTERFACE_RGMII_TXID
#elif defined(CONFIG_AM263X_CPSW_ETH_PHY_RGMII_RXID)
#  define CPSW_PHY_INTERFACE PHY_INTERFACE_RGMII_RXID
#else
#  define CPSW_PHY_INTERFACE PHY_INTERFACE_RGMII
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CPPI 3.0 buffer descriptor */

struct cpsw_desc_s
{
  uint32_t next;      /* Next descriptor, 0 at the end of the queue */
  uint32_t buf;       /* Buffer address */
  uint32_t bufofflen; /* Buffer offset (31:16), length (15:0) */
  uint32_t flagslen;  /* CPDMA_DESC_xxx flags, packet length (10:0) */
};

struct am263x_cpsw_s
{
  struct netdev_lowerhalf_s dev;     /* Must be first */

  FAR struct mdio_bus_s *mdio;
  FAR struct phy_device_s *phydev;
  struct work_s linkwork;
  struct work_s txwork;              /* Reclaims sent packets */
  spinlock_t lock;                   /* TX queue against the interrupt */

  bool up;                           /* Between ifup and ifdown */
  bool link;                         /* Carrier on */
  uint32_t speed;
  uint8_t duplex;

  FAR volatile struct cpsw_desc_s *rxdesc;
  FAR volatile struct cpsw_desc_s *txdesc;

  unsigned int rxhead;               /* Next to complete */
  unsigned int rxtail;               /* Last queued to the hardware */

  unsigned int txhead;               /* Next free */
  unsigned int txdone;               /* Next to complete */
  unsigned int txfree;               /* Next packet to free */
  unsigned int txbusy;               /* Queued and not completed */
  FAR netpkt_t *txpkt[CPSW_NTXDESC];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int am263x_cpsw_ifup(FAR struct netdev_lowerhalf_s *dev);
static int am263x_cpsw_ifdown(FAR struct netdev_lowerhalf_s *dev);
static int am263x_cpsw_transmit(FAR struct netdev_lowerhalf_s *dev,
                                FAR netpkt_t *pkt);
static FAR netpkt_t *am263x_cpsw_receive(FAR struct netdev_lowerhalf_s *dev);
static void am263x_cpsw_reclaim(FAR struct netdev_lowerhalf_s *dev);
static void am263x_cpsw_txwork(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct netdev_ops_s g_am263x_cpsw_ops =
{
  .ifup     = am263x_cpsw_ifup,
  .ifdown   = am263x_cpsw_ifdown,
  .transmit = am263x_cpsw_transmit,
  .receive  = am263x_cpsw_receive,
  .reclaim  = am263x_cpsw_reclaim,
};

static struct am263x_cpsw_s g_am263x_cpsw;

/* Locally administered: only when neither the eFuse nor the board has
 * an address.
 */

static const uint8_t g_cpsw_defmac[6] =
{
  0x02, 0x00, 0x26, 0x34, 0x00, 0x01
};

static uint8_t g_cpsw_rxbuf[CPSW_NRXDESC][CPSW_BUFSIZE]
  aligned_data(CPSW_CACHELINE);
static uint8_t g_cpsw_txbuf[CPSW_NTXDESC][CPSW_BUFSIZE]
  aligned_data(CPSW_CACHELINE);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Uncached, see am263x_mpuinit.c */

uint8_t g_am263x_cpsw_desc[AM263X_CPSW_DESC_REGION_SIZE]
  aligned_data(AM263X_CPSW_DESC_REGION_SIZE);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_macaddr_valid
 *
 * Description:
 *   A usable station address: not all zeros, not all ones, not multicast.
 *
 ****************************************************************************/

static bool am263x_macaddr_valid(FAR const uint8_t mac[6])
{
  static const uint8_t zero[6];
  static const uint8_t ones[6] =
  {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff
  };

  return (mac[0] & 1) == 0 && memcmp(mac, zero, 6) != 0 &&
         memcmp(mac, ones, 6) != 0;
}

static inline uint32_t cpsw_addr(FAR volatile void *p)
{
  return (uint32_t)(uintptr_t)p;
}

/****************************************************************************
 * Name: am263x_cpsw_wait
 *
 * Description:
 *   Wait for (reg & mask) == value, for up to CPSW_RESET_TIMEOUT us.
 *
 ****************************************************************************/

static int am263x_cpsw_wait(uintptr_t reg, uint32_t mask, uint32_t value)
{
  int us;

  for (us = 0; us < CPSW_RESET_TIMEOUT; us++)
    {
      if ((getreg32(reg) & mask) == value)
        {
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: am263x_cpsw_rxdesc_init
 *
 * Description:
 *   Hand RX descriptor i to the hardware, empty and at the end of a queue.
 *
 ****************************************************************************/

static void am263x_cpsw_rxdesc_init(FAR struct am263x_cpsw_s *priv,
                                    unsigned int i)
{
  FAR volatile struct cpsw_desc_s *d = &priv->rxdesc[i];

  d->next      = 0;
  d->buf       = cpsw_addr(g_cpsw_rxbuf[i]);
  d->bufofflen = CPSW_BUFSIZE;
  d->flagslen  = CPDMA_DESC_OWNER;
}

/****************************************************************************
 * Name: am263x_cpsw_macmode
 *
 * Description:
 *   Set the MAC port's interface (RGMII) and the SoC's RGMII delays.
 *
 ****************************************************************************/

static void am263x_cpsw_macmode(void)
{
  uint32_t set = MSS_CPSW_CONTROL_MODE_RGMII(CPSW_PORT);

#ifndef CONFIG_AM263X_CPSW_ETH_RGMII_INTERNAL_DELAY
  set |= MSS_CPSW_CONTROL_RGMII_ID_MODE(CPSW_PORT);
#endif

  putreg32(AM263X_KICK0_UNLOCK, AM263X_MSS_CTRL_LOCK0_KICK0);
  putreg32(AM263X_KICK1_UNLOCK, AM263X_MSS_CTRL_LOCK0_KICK1);

  modifyreg32(AM263X_MSS_CTRL_CPSW_CONTROL,
              MSS_CPSW_CONTROL_MODE_MASK(CPSW_PORT) |
              MSS_CPSW_CONTROL_RGMII_ID_MODE(CPSW_PORT), set);

  putreg32(AM263X_KICK_LOCK, AM263X_MSS_CTRL_LOCK0_KICK0);
  putreg32(AM263X_KICK_LOCK, AM263X_MSS_CTRL_LOCK0_KICK1);
}

/****************************************************************************
 * Name: am263x_cpsw_hwinit
 *
 * Description:
 *   Set up the switch, the MAC port and the DMA, and start receiving.
 *   The MAC itself stays off until the link is up.
 *
 ****************************************************************************/

static int am263x_cpsw_hwinit(FAR struct am263x_cpsw_s *priv)
{
  FAR const uint8_t *mac = priv->dev.netdev.d_mac.ether.ether_addr_octet;
  unsigned int i;
  int ret;

  /* The CPDMA soft reset resets the whole switch, the ALE and the MDIO
   * controller included, so it comes first.
   */

  putreg32(1, AM263X_CPDMA_SOFT_RESET);
  ret = am263x_cpsw_wait(AM263X_CPDMA_SOFT_RESET, 1, 0);
  if (ret < 0)
    {
      nerr("ERROR: CPDMA reset timed out\n");
      return ret;
    }

  am263x_mdio_initialize();
  am263x_cpsw_macmode();

  /* MAC port: reset, then off until there is a link */

  putreg32(1, AM263X_CPSW_PN_MAC_SOFT_RESET(CPSW_PORT));
  ret = am263x_cpsw_wait(AM263X_CPSW_PN_MAC_SOFT_RESET(CPSW_PORT), 1, 0);
  if (ret < 0)
    {
      nerr("ERROR: MAC port %d reset timed out\n", CPSW_PORT);
      return ret;
    }

  putreg32(0, AM263X_CPSW_PN_MAC_CONTROL(CPSW_PORT));
  putreg32(CPSW_MAXFRAME, AM263X_CPSW_PN_RX_MAXLEN(CPSW_PORT));
  putreg32(mac[0] | mac[1] << 8 | mac[2] << 16 | (uint32_t)mac[3] << 24,
           AM263X_CPSW_PN_SA_H(CPSW_PORT));
  putreg32(mac[4] | mac[5] << 8, AM263X_CPSW_PN_SA_L(CPSW_PORT));

  /* Host port: on, FCS stripped from what it receives, short frames it
   * sends padded.
   */

  putreg32(CPSW_CONTROL_P0_ENABLE | CPSW_CONTROL_P0_TX_CRC_REMOVE |
           CPSW_CONTROL_P0_RX_PAD, AM263X_CPSW_CONTROL);
  putreg32(CPSW_MAXFRAME, AM263X_CPSW_P0_RX_MAXLEN);
  putreg32((1 << 0) | (1 << CPSW_PORT), AM263X_CPSW_STAT_PORT_EN);

  /* ALE: bypass first, then let the host port and our port forward */

  putreg32(ALE_CONTROL_ENABLE_ALE | ALE_CONTROL_CLEAR_TABLE |
           ALE_CONTROL_ENABLE_BYPASS, AM263X_ALE_CONTROL);
  putreg32(ALE_PORTCTL_STATE_FORWARD, AM263X_ALE_PORTCTL(0));
  putreg32(ALE_PORTCTL_STATE_FORWARD, AM263X_ALE_PORTCTL(CPSW_PORT));
  putreg32(ALE_PORTCTL_STATE_DISABLED, AM263X_ALE_PORTCTL(CPSW_OTHER_PORT));

  /* CPDMA: clear every channel's pointers */

  for (i = 0; i < CPDMA_NCHANNELS; i++)
    {
      putreg32(0, AM263X_CPDMA_TX_HDP(i));
      putreg32(0, AM263X_CPDMA_RX_HDP(i));
      putreg32(0, AM263X_CPDMA_TX_CP(i));
      putreg32(0, AM263X_CPDMA_RX_CP(i));
    }

  putreg32(0, AM263X_CPDMA_RX_BUFFER_OFFSET);

  /* RX queue: every descriptor, chained.  The buffers must not hold
   * dirty lines that could be written back over received data.
   */

  up_flush_dcache((uintptr_t)g_cpsw_rxbuf,
                  (uintptr_t)g_cpsw_rxbuf + sizeof(g_cpsw_rxbuf));

  for (i = 0; i < CPSW_NRXDESC; i++)
    {
      am263x_cpsw_rxdesc_init(priv, i);
      if (i > 0)
        {
          priv->rxdesc[i - 1].next = cpsw_addr(&priv->rxdesc[i]);
        }
    }

  priv->rxhead = 0;
  priv->rxtail = CPSW_NRXDESC - 1;

  /* TX queue: empty */

  priv->txhead = 0;
  priv->txdone = 0;
  priv->txfree = 0;
  priv->txbusy = 0;

  /* Interrupts: RX and TX on channel 0, to core group 0 */

  putreg32(1 << CPSW_CH, AM263X_CPDMA_RX_INTMASK_SET);
  putreg32(1 << CPSW_CH, AM263X_CPDMA_TX_INTMASK_SET);
  putreg32(1 << CPSW_CH, AM263X_CPSW_SS_C_RX_EN(CPSW_CORE));
  putreg32(1 << CPSW_CH, AM263X_CPSW_SS_C_TX_EN(CPSW_CORE));

  putreg32(1, AM263X_CPDMA_TX_CONTROL);
  putreg32(1, AM263X_CPDMA_RX_CONTROL);

  UP_DSB();
  putreg32(cpsw_addr(&priv->rxdesc[0]), AM263X_CPDMA_RX_HDP(CPSW_CH));
  putreg32(CPSW_NRXDESC, AM263X_CPDMA_RX_FREEBUFFER(CPSW_CH));

  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_teardown
 *
 * Description:
 *   Stop the DMA channels and the MAC, and free any packets still held.
 *
 ****************************************************************************/

static void am263x_cpsw_teardown(FAR struct am263x_cpsw_s *priv)
{
  unsigned int i;

  putreg32(0, AM263X_CPSW_PN_MAC_CONTROL(CPSW_PORT));
  putreg32(0, AM263X_CPSW_SS_C_RX_EN(CPSW_CORE));
  putreg32(0, AM263X_CPSW_SS_C_TX_EN(CPSW_CORE));
  putreg32(1 << CPSW_CH, AM263X_CPDMA_RX_INTMASK_CLEAR);
  putreg32(1 << CPSW_CH, AM263X_CPDMA_TX_INTMASK_CLEAR);

  putreg32(CPSW_CH, AM263X_CPDMA_TX_TEARDOWN);
  if (am263x_cpsw_wait(AM263X_CPDMA_TX_CP(CPSW_CH), 0xffffffff,
                       CPDMA_TEARDOWN_COMPLETE) < 0)
    {
      nwarn("WARNING: TX teardown timed out\n");
    }

  putreg32(CPSW_CH, AM263X_CPDMA_RX_TEARDOWN);
  if (am263x_cpsw_wait(AM263X_CPDMA_RX_CP(CPSW_CH), 0xffffffff,
                       CPDMA_TEARDOWN_COMPLETE) < 0)
    {
      nwarn("WARNING: RX teardown timed out\n");
    }

  putreg32(0, AM263X_CPDMA_TX_CONTROL);
  putreg32(0, AM263X_CPDMA_RX_CONTROL);

  /* This resets the MDIO controller too, under PHYLIB, which keeps
   * running: put it back.
   */

  putreg32(1, AM263X_CPDMA_SOFT_RESET);
  am263x_cpsw_wait(AM263X_CPDMA_SOFT_RESET, 1, 0);
  am263x_mdio_initialize();

  for (i = 0; i < CPSW_NTXDESC; i++)
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
 * Name: am263x_cpsw_rx_interrupt
 ****************************************************************************/

static int am263x_cpsw_rx_interrupt(int irq, FAR void *context,
                                    FAR void *arg)
{
  FAR struct am263x_cpsw_s *priv = arg;

  /* Masked until receive() has emptied the queue */

  putreg32(0, AM263X_CPSW_SS_C_RX_EN(CPSW_CORE));
  netdev_lower_rxready(&priv->dev);
  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_tx_interrupt
 ****************************************************************************/

static int am263x_cpsw_tx_interrupt(int irq, FAR void *context,
                                    FAR void *arg)
{
  FAR struct am263x_cpsw_s *priv = arg;
  FAR volatile struct cpsw_desc_s *d;
  irqstate_t flags;
  uint32_t status;

  flags = spin_lock_irqsave(&priv->lock);

  while (priv->txbusy > 0)
    {
      d = &priv->txdesc[priv->txdone];
      status = d->flagslen;
      if ((status & CPDMA_DESC_OWNER) != 0)
        {
          break;
        }

      putreg32(cpsw_addr(d), AM263X_CPDMA_TX_CP(CPSW_CH));

      /* The channel stopped here, but more was queued after it */

      if ((status & CPDMA_DESC_EOQ) != 0 && d->next != 0)
        {
          putreg32(d->next, AM263X_CPDMA_TX_HDP(CPSW_CH));
        }

      priv->txdone = (priv->txdone + 1) % CPSW_NTXDESC;
      priv->txbusy--;
    }

  spin_unlock_irqrestore(&priv->lock, flags);

  putreg32(CPDMA_EOI_TX, AM263X_CPDMA_EOI_VECTOR);

  if (work_available(&priv->txwork))
    {
      work_queue(LPWORK, &priv->txwork, am263x_cpsw_txwork, priv, 0);
    }

  netdev_lower_txdone(&priv->dev);
  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_setlink
 *
 * Description:
 *   Set the MAC port to the PHY's speed and duplex, or turn it off.
 *
 ****************************************************************************/

static void am263x_cpsw_setlink(FAR struct am263x_cpsw_s *priv)
{
  uint32_t ctl = 0;

  if (priv->link)
    {
      ctl = CPSW_MAC_CONTROL_GMII_EN;

      if (priv->duplex == DUPLEX_FULL)
        {
          ctl |= CPSW_MAC_CONTROL_FULLDUPLEX;
        }

      if (priv->speed == 1000)
        {
          ctl |= CPSW_MAC_CONTROL_GIG;
        }
      else if (priv->speed == 10)
        {
          /* RGMII 10 Mbps works only with in-band status */

          ctl |= CPSW_MAC_CONTROL_EXT_EN;
        }
    }

  putreg32(ctl, AM263X_CPSW_PN_MAC_CONTROL(CPSW_PORT));
}

/****************************************************************************
 * Name: am263x_cpsw_linkwork
 *
 * Description:
 *   Follow the PHY's link state (PHYLIB's state machine reads the PHY).
 *
 ****************************************************************************/

static void am263x_cpsw_linkwork(FAR void *arg)
{
  FAR struct am263x_cpsw_s *priv = arg;
  FAR struct phy_device_s *phydev = priv->phydev;
  uint32_t speed;
  uint8_t duplex;
  bool link;

  if (!priv->up)
    {
      return;
    }

  nxmutex_lock(&phydev->lock);
  link   = phydev->link;
  speed  = phydev->speed;
  duplex = phydev->duplex;
  nxmutex_unlock(&phydev->lock);

  if (link != priv->link ||
      (link && (speed != priv->speed || duplex != priv->duplex)))
    {
      priv->link   = link;
      priv->speed  = speed;
      priv->duplex = duplex;
      am263x_cpsw_setlink(priv);

      if (link)
        {
          ninfo("Link up, %" PRIu32 " Mbps %s duplex\n", speed,
                duplex == DUPLEX_FULL ? "full" : "half");
          netdev_lower_carrier_on(&priv->dev);
        }
      else
        {
          ninfo("Link down\n");
          netdev_lower_carrier_off(&priv->dev);
        }
    }

  work_queue(LPWORK, &priv->linkwork, am263x_cpsw_linkwork, priv,
             CPSW_LINK_POLL);
}

/****************************************************************************
 * Name: am263x_cpsw_phyinit
 *
 * Description:
 *   Hand the PHY to PHYLIB the first time the interface comes up.  The
 *   PHY then keeps running across ifdown.
 *
 ****************************************************************************/

static int am263x_cpsw_phyinit(FAR struct am263x_cpsw_s *priv)
{
  FAR struct phy_device_s *phydev;
  int ret;

  if (priv->phydev != NULL)
    {
      return OK;
    }

  phydev = phy_device_create(priv->mdio, CONFIG_AM263X_CPSW_ETH_PHYADDR);
  if (phydev == NULL)
    {
      nerr("ERROR: No PHY at address %d\n", CONFIG_AM263X_CPSW_ETH_PHYADDR);
      return -ENODEV;
    }

  phydev->interface = CPSW_PHY_INTERFACE;

  ret = phy_connect(phydev, NULL);
  if (ret < 0)
    {
      nerr("ERROR: Failed to connect the PHY: %d\n", ret);
      phy_device_free(phydev);
      return ret;
    }

  ret = phy_start(phydev);
  if (ret < 0)
    {
      nerr("ERROR: Failed to start the PHY: %d\n", ret);
      phy_disconnect(phydev);
      return ret;
    }

  priv->phydev = phydev;
  priv->dev.phydev = phydev;
  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_ifup
 ****************************************************************************/

static int am263x_cpsw_ifup(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_cpsw_s *priv = (FAR struct am263x_cpsw_s *)dev;
  int ret;

  /* The switch first: its reset resets the MDIO controller too */

  ret = am263x_cpsw_hwinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = am263x_cpsw_phyinit(priv);
  if (ret < 0)
    {
      return ret;
    }

  priv->up   = true;
  priv->link = false;

  up_enable_irq(AM263X_IRQ_CPSW0_RX);
  up_enable_irq(AM263X_IRQ_CPSW0_TX);

  work_queue(LPWORK, &priv->linkwork, am263x_cpsw_linkwork, priv, 0);
  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_ifdown
 ****************************************************************************/

static int am263x_cpsw_ifdown(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_cpsw_s *priv = (FAR struct am263x_cpsw_s *)dev;

  priv->up = false;
  work_cancel_sync(LPWORK, &priv->linkwork);

  up_disable_irq(AM263X_IRQ_CPSW0_RX);
  up_disable_irq(AM263X_IRQ_CPSW0_TX);

  /* ifdown() runs with the device locked: the work, if it has started,
   * waits for the lock and then finds nothing to free after teardown.
   */

  work_cancel(LPWORK, &priv->txwork);

  am263x_cpsw_teardown(priv);

  if (priv->link)
    {
      priv->link = false;
      netdev_lower_carrier_off(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_transmit
 ****************************************************************************/

static int am263x_cpsw_transmit(FAR struct netdev_lowerhalf_s *dev,
                                FAR netpkt_t *pkt)
{
  FAR struct am263x_cpsw_s *priv = (FAR struct am263x_cpsw_s *)dev;
  FAR volatile struct cpsw_desc_s *d;
  FAR uint8_t *buf;
  unsigned int len;
  unsigned int i;
  irqstate_t flags;

  len = netpkt_getdatalen(dev, pkt);
  if (len > CPSW_BUFSIZE)
    {
      return -EMSGSIZE;
    }

  /* Free what has completed.  With the TX quota one below the number of
   * descriptors, that frees the next descriptor's packet.
   */

  am263x_cpsw_reclaim(dev);

  i = priv->txhead;
  if (priv->txpkt[i] != NULL)
    {
      return -EBUSY;
    }

  buf = g_cpsw_txbuf[i];
  netpkt_copyout(dev, buf, pkt, len, 0);
  if (len < CPSW_MINFRAME)
    {
      memset(buf + len, 0, CPSW_MINFRAME - len);
      len = CPSW_MINFRAME;
    }

  up_clean_dcache((uintptr_t)buf, (uintptr_t)buf + CPSW_ALIGN_UP(len));

  d = &priv->txdesc[i];
  d->next      = 0;
  d->buf       = cpsw_addr(buf);
  d->bufofflen = len;
  d->flagslen  = CPDMA_DESC_SOP | CPDMA_DESC_EOP | CPDMA_DESC_OWNER |
                 CPDMA_DESC_TX_TO_PORT_EN |
                 CPDMA_DESC_TX_TO_PORT(CPSW_PORT) | len;
  priv->txpkt[i] = pkt;
  UP_DSB();

  flags = spin_lock_irqsave(&priv->lock);

  if (priv->txbusy == 0)
    {
      /* The channel is idle: start it here */

      putreg32(cpsw_addr(d), AM263X_CPDMA_TX_HDP(CPSW_CH));
    }
  else
    {
      /* Chain it on.  If the channel has already stopped at the previous
       * descriptor, the TX interrupt finds EOQ there and restarts it.
       */

      priv->txdesc[(i + CPSW_NTXDESC - 1) % CPSW_NTXDESC].next =
        cpsw_addr(d);
    }

  priv->txbusy++;
  priv->txhead = (i + 1) % CPSW_NTXDESC;

  spin_unlock_irqrestore(&priv->lock, flags);
  return OK;
}

/****************************************************************************
 * Name: am263x_cpsw_reclaim
 *
 * Description:
 *   Free the packets of completed TX descriptors.
 *
 ****************************************************************************/

static void am263x_cpsw_reclaim(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_cpsw_s *priv = (FAR struct am263x_cpsw_s *)dev;
  unsigned int done;
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  done = priv->txdone;
  spin_unlock_irqrestore(&priv->lock, flags);

  while (priv->txfree != done)
    {
      if (priv->txpkt[priv->txfree] != NULL)
        {
          netpkt_free(dev, priv->txpkt[priv->txfree], NETPKT_TX);
          priv->txpkt[priv->txfree] = NULL;
        }

      priv->txfree = (priv->txfree + 1) % CPSW_NTXDESC;
    }
}

/****************************************************************************
 * Name: am263x_cpsw_txwork
 *
 * Description:
 *   Free the packets of completed TX descriptors after the TX interrupt.
 *   transmit() and reclaim() run with the device locked, so this takes
 *   the lock too.
 *
 ****************************************************************************/

static void am263x_cpsw_txwork(FAR void *arg)
{
  FAR struct am263x_cpsw_s *priv = arg;

  netdev_lock(&priv->dev.netdev);
  am263x_cpsw_reclaim(&priv->dev);
  netdev_unlock(&priv->dev.netdev);
}

/****************************************************************************
 * Name: am263x_cpsw_receive
 ****************************************************************************/

static FAR netpkt_t *am263x_cpsw_receive(FAR struct netdev_lowerhalf_s *dev)
{
  FAR struct am263x_cpsw_s *priv = (FAR struct am263x_cpsw_s *)dev;
  FAR volatile struct cpsw_desc_s *d;
  FAR netpkt_t *pkt;
  FAR uint8_t *buf;
  unsigned int len;
  unsigned int i;
  uint32_t status;
  uint32_t next;

  for (; ; )
    {
      i = priv->rxhead;
      d = &priv->rxdesc[i];
      status = d->flagslen;

      if ((status & CPDMA_DESC_OWNER) != 0)
        {
          /* Empty: take interrupts again.  The end-of-interrupt raises a
           * new one if a frame came in meanwhile.
           */

          putreg32(1 << CPSW_CH, AM263X_CPSW_SS_C_RX_EN(CPSW_CORE));
          putreg32(CPDMA_EOI_RX, AM263X_CPDMA_EOI_VECTOR);
          return NULL;
        }

      pkt  = NULL;
      next = d->next;
      len  = status & CPDMA_DESC_PKTLEN_MASK;
      buf  = g_cpsw_rxbuf[i];

      if ((status & (CPDMA_DESC_SOP | CPDMA_DESC_EOP)) !=
          (CPDMA_DESC_SOP | CPDMA_DESC_EOP) ||
          (status & CPDMA_DESC_RX_ERRORS) != 0 || len > CPSW_BUFSIZE)
        {
          NETDEV_RXERRORS(&dev->netdev);
        }
      else
        {
          pkt = netpkt_alloc(dev, NETPKT_RX);
          if (pkt == NULL)
            {
              NETDEV_RXDROPPED(&dev->netdev);
            }
          else
            {
              up_invalidate_dcache((uintptr_t)buf,
                                   (uintptr_t)buf + CPSW_ALIGN_UP(len));
              if (netpkt_copyin(dev, pkt, buf, len, 0) < 0)
                {
                  netpkt_free(dev, pkt, NETPKT_RX);
                  pkt = NULL;
                  NETDEV_RXDROPPED(&dev->netdev);
                }
            }
        }

      /* Acknowledge it and give it back, at the end of the queue */

      putreg32(cpsw_addr(d), AM263X_CPDMA_RX_CP(CPSW_CH));

      am263x_cpsw_rxdesc_init(priv, i);
      UP_DSB();

      if (priv->rxtail != i)
        {
          priv->rxdesc[priv->rxtail].next = cpsw_addr(d);
        }

      priv->rxtail = i;
      priv->rxhead = (i + 1) % CPSW_NRXDESC;

      /* The channel stopped at this descriptor: restart it after it, or
       * at it if nothing else was queued.
       */

      if ((status & CPDMA_DESC_EOQ) != 0)
        {
          UP_DSB();
          putreg32(next != 0 ? next : cpsw_addr(d),
                   AM263X_CPDMA_RX_HDP(CPSW_CH));
        }

      putreg32(1, AM263X_CPDMA_RX_FREEBUFFER(CPSW_CH));

      if (pkt != NULL)
        {
          return pkt;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am263x_efuse_macaddr
 ****************************************************************************/

int am263x_efuse_macaddr(uint8_t mac[6])
{
  uint32_t id0 = getreg32(AM263X_TOP_CTRL_MAC_ID0);
  uint32_t id1 = getreg32(AM263X_TOP_CTRL_MAC_ID1);

  mac[0] = (id1 >> 8) & 0xff;
  mac[1] = id1 & 0xff;
  mac[2] = (id0 >> 24) & 0xff;
  mac[3] = (id0 >> 16) & 0xff;
  mac[4] = (id0 >> 8) & 0xff;
  mac[5] = id0 & 0xff;

  return am263x_macaddr_valid(mac) ? OK : -ENOENT;
}

/****************************************************************************
 * Name: am263x_cpsw_setmacaddr
 ****************************************************************************/

int am263x_cpsw_setmacaddr(FAR const uint8_t mac[6])
{
  FAR struct am263x_cpsw_s *priv = &g_am263x_cpsw;

  if (!am263x_macaddr_valid(mac))
    {
      return -EINVAL;
    }

  if (priv->up)
    {
      return -EBUSY;
    }

  memcpy(priv->dev.netdev.d_mac.ether.ether_addr_octet, mac, 6);
  return OK;
}

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   Register the CPSW network device.
 *
 ****************************************************************************/

void arm_netinitialize(void)
{
  FAR struct am263x_cpsw_s *priv = &g_am263x_cpsw;
  FAR struct netdev_lowerhalf_s *dev = &priv->dev;
  uint8_t mac[6];
  int ret;

  priv->mdio = am263x_mdio_initialize();
  if (priv->mdio == NULL)
    {
      nerr("ERROR: No MDIO bus\n");
      return;
    }

  priv->rxdesc = (FAR volatile struct cpsw_desc_s *)g_am263x_cpsw_desc;
  priv->txdesc = priv->rxdesc + CPSW_NRXDESC;
  spin_lock_init(&priv->lock);

  /* The SoC's own address, from the eFuse.  Without one, a fixed
   * placeholder until the board sets one (am263x_cpsw_setmacaddr()).
   */

  if (am263x_efuse_macaddr(mac) < 0)
    {
      nwarn("WARNING: No MAC address in the eFuse\n");
      memcpy(mac, g_cpsw_defmac, sizeof(mac));
    }

  memcpy(dev->netdev.d_mac.ether.ether_addr_octet, mac, sizeof(mac));

  ret = irq_attach(AM263X_IRQ_CPSW0_RX, am263x_cpsw_rx_interrupt, priv);
  if (ret == OK)
    {
      ret = irq_attach(AM263X_IRQ_CPSW0_TX, am263x_cpsw_tx_interrupt, priv);
    }

  if (ret < 0)
    {
      nerr("ERROR: Failed to attach the CPSW interrupts: %d\n", ret);
      return;
    }

  up_set_irq_type(AM263X_IRQ_CPSW0_RX, IRQ_RISING_EDGE);
  up_set_irq_type(AM263X_IRQ_CPSW0_TX, IRQ_RISING_EDGE);

  dev->ops = &g_am263x_cpsw_ops;
  dev->quota[NETPKT_TX] = CPSW_TX_QUOTA;
  dev->quota[NETPKT_RX] = CPSW_RX_QUOTA;

  ret = netdev_lower_register(dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      nerr("ERROR: netdev_lower_register failed: %d\n", ret);
    }
}

#elif defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)

/****************************************************************************
 * Name: arm_netinitialize
 *
 * Description:
 *   No Ethernet (only the MDIO bus, CONFIG_AM263X_MDIO): nothing to
 *   register.
 *
 ****************************************************************************/

void arm_netinitialize(void)
{
}

#endif /* CONFIG_AM263X_CPSW_ETH && !CONFIG_NETDEV_LATEINIT */
