/****************************************************************************
 * net/can/can_isotp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <nuttx/debug.h>
#include <assert.h>

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>
#include <nuttx/nuttx.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/can.h>
#include <nuttx/mutex.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>
#include <nuttx/poll.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/can.h>
#include <nuttx/can/can_common.h>

#include <nuttx/can/isotp.h>

#include "netdev/netdev.h"
#include "devif/devif.h"
#include "socket/socket.h"
#include <nuttx/mm/mempool.h>

#include "utils/utils.h"
#include "can/can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_CAN_CANFD
#  define ISOTP_FRAME_MAX_DLEN CANFD_MAX_DLEN
#  define ISOTP_FRAME_SIZE     CANFD_MTU
#else
#  define ISOTP_FRAME_MAX_DLEN CAN_MAX_DLEN
#  define ISOTP_FRAME_SIZE     CAN_MTU
#endif

#define ISOTP_FC_WAIT_MS       15000
#define ISOTP_CR_WAIT_MS       1000
#define ISOTP_RX_STMIN_MS      1
#define ISOTP_OPTS_FLAGS       (CAN_ISOTP_WAIT_TX_DONE | CAN_ISOTP_TX_PADDING | \
                                CAN_ISOTP_RX_PADDING | CAN_ISOTP_CHK_PAD_LEN | \
                                CAN_ISOTP_CHK_PAD_DATA | CAN_ISOTP_EXTEND_ADDR | \
                                CAN_ISOTP_RX_EXT_ADDR | CAN_ISOTP_LISTEN_MODE | \
                                CAN_ISOTP_HALF_DUPLEX | CAN_ISOTP_FORCE_TXSTMIN | \
                                CAN_ISOTP_FORCE_RXSTMIN | ISOTP_BC_FLAGS | \
                                CAN_ISOTP_DYN_FC_PARMS)
#define ISOTP_BC_FLAGS         (CAN_ISOTP_SF_BROADCAST | CAN_ISOTP_CF_BROADCAST)
#define ISOTP_CANID_MASK       (CAN_EFF_FLAG | CAN_EFF_MASK)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TX state machine (TX state machine fields below) */

enum isotp_tx_state_e
{
  ISOTP_TX_IDLE = 0,    /* No transfer in progress */
  ISOTP_TX_WAIT_FC,     /* Waiting for flow control */
  ISOTP_TX_SENDING,     /* Sending consecutive frames */
};

struct isotp_poll_s
{
  FAR struct can_conn_s *conn;
  FAR struct net_driver_s *dev;
  FAR struct devif_callback_s *cb;
  FAR struct pollfd_s *fds;
};

/* ISO-TP connection state */

struct isotp_conn_s
{
  struct can_conn_s            conn;            /* Underlying CAN connection */
  struct isotp_poll_s           pollinfo[CONFIG_NET_CAN_NPOLLWAITERS];
  struct can_isotp_options     opt;             /* ISO-TP socket options */
  struct can_isotp_ll_options  ll;              /* Link-layer options */
  uint8_t                      tx_bs;           /* BS of the FC in use */
  uint8_t                      tx_bs_remaining; /* CFs left in block (0=inf) */
  bool                         tx_fc_seen;      /* An FC for this message */
  uint32_t                     tx_stmin_us;     /* CF gap: STmin + N_As (us) */
  uint32_t                     tx_frame_ns;     /* N_As, frame_txtime (ns) */
  uint32_t                     tx_force_ns;     /* CAN_ISOTP_TX_STMIN (ns) */
  int                          tx_result;       /* Outcome for WAIT_TX_DONE */
  enum isotp_tx_state_e        tx_state;        /* TX progress */
  clock_t                      tx_last_send;    /* Tick of last CF (STmin) */
  canid_t                      tx_id;           /* CAN ID for TX frames */
  sem_t                        tx_sem;          /* Serialize senders (init 1) */
  sem_t                        tx_done_sem;     /* TX done */
  FAR uint8_t                 *tx_buf;          /* Copy of the message */
  struct isotp_encode_s        tx_enc;          /* Frames sent so far */
  struct wdog_s                tx_wdog;         /* N_Bs flow-control timeout */
  struct work_s                tx_work;         /* STmin pacing work */
  FAR struct devif_callback_s *tx_cb;           /* CAN_POLL: TX engine + FC */
  struct can_isotp_fc_options  rx_fc;           /* RX flow-control options */
  canid_t                      rx_id;           /* CAN ID for RX frames */
  isotp_len_t                  rx_total;        /* Incoming PDU length (FF) */
  isotp_len_t                  rx_done;         /* Incoming payload seen */
  uint16_t                     rx_ready;        /* Complete PDUs not yet read */
  uint8_t                      rx_sn;           /* Next expected CF SN */
  uint8_t                      rx_block;        /* CFs since the last FC */
  bool                         rx_active;       /* Multi-frame RX running */
  bool                         rx_fc_pending;   /* CTS FC awaiting TX slot */
  bool                         rx_cf_seen;      /* A CF since the FF */
  clock_t                      rx_last_cf;      /* Tick of last CF taken */
  uint32_t                     rx_force_ns;     /* CAN_ISOTP_RX_STMIN (ns) */
  mutex_t                      rx_lock;         /* Serialize concurrent receivers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int       isotp_setup(FAR struct socket *psock);
static sockcaps_t isotp_sockcaps(FAR struct socket *psock);
static void      isotp_addref(FAR struct socket *psock);
static int       isotp_bind(FAR struct socket *psock,
                            FAR const struct sockaddr *addr,
                            socklen_t addrlen);
static int       isotp_getsockname(FAR struct socket *psock,
                                   FAR struct sockaddr *addr,
                                   FAR socklen_t *addrlen);
static int       isotp_poll(FAR struct socket *psock,
                            FAR struct pollfd_s *fds,
                            bool setup);
static ssize_t   isotp_sendmsg(FAR struct socket *psock,
                               FAR const struct msghdr *msg, int flags);
static ssize_t   isotp_recvmsg(FAR struct socket *psock,
                               FAR struct msghdr *msg, int flags);
static int       isotp_close(FAR struct socket *psock);

#ifdef CONFIG_NET_CAN_SOCK_OPTS
static int isotp_getsockopt(FAR struct socket *psock, int level, int option,
                            FAR void *value, FAR socklen_t *value_len);
static int isotp_setsockopt(FAR struct socket *psock, int level, int option,
                            FAR const void *value, socklen_t value_len);
#endif

static void isotp_tx_finish(FAR struct isotp_conn_s *isotp, int result);
static void isotp_tx_wait_fc(FAR struct isotp_conn_s *isotp);
static int  isotp_tx_frame(FAR struct isotp_conn_s *isotp,
                           FAR struct iob_s **iob_out);

/****************************************************************************
 * Private Data
 ****************************************************************************/

MEMPOOL_DEFINE(g_isotp_conns, sizeof(struct isotp_conn_s),
               CONFIG_NET_CAN_ISOTP_PREALLOC_CONNS,
               CONFIG_NET_CAN_ISOTP_MAX_CONNS,
               CONFIG_NET_CAN_ISOTP_ALLOC_CONNS);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct sock_intf_s g_can_isotp_sockif =
{
  isotp_setup,       /* si_setup */
  isotp_sockcaps,    /* si_sockcaps */
  isotp_addref,      /* si_addref */
  isotp_bind,        /* si_bind */
  isotp_getsockname, /* si_getsockname */
  NULL,              /* si_getpeername */
  NULL,              /* si_listen */
  NULL,              /* si_connect */
  NULL,              /* si_accept */
  isotp_poll,        /* si_poll */
  isotp_sendmsg,     /* si_sendmsg */
  isotp_recvmsg,     /* si_recvmsg */
  isotp_close,       /* si_close */
  NULL,              /* si_ioctl */
  NULL,              /* si_socketpair */
  NULL               /* si_shutdown */
#ifdef CONFIG_NET_CAN_SOCK_OPTS
  , isotp_getsockopt /* si_getsockopt */
  , isotp_setsockopt /* si_setsockopt */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: isotp_from_conn
 ****************************************************************************/

static inline_function FAR struct isotp_conn_s *
isotp_from_conn(FAR struct can_conn_s *conn)
{
  return container_of(conn, struct isotp_conn_s, conn);
}

/****************************************************************************
 * Name: isotp_conn
 ****************************************************************************/

static inline_function FAR struct isotp_conn_s *
isotp_conn(FAR struct socket *psock)
{
  return isotp_from_conn((FAR struct can_conn_s *)psock->s_conn);
}

/****************************************************************************
 * Name: isotp_ae
 *
 * Description:
 *   Address bytes before the PCI: 1 with extended addressing, else 0.
 *
 ****************************************************************************/

static inline_function uint8_t isotp_ae(FAR struct isotp_conn_s *isotp)
{
  return (isotp->opt.flags & CAN_ISOTP_EXTEND_ADDR) != 0;
}

/****************************************************************************
 * Name: isotp_listen_mode
 *
 * Description:
 *   A CAN_ISOTP_LISTEN_MODE socket only listens: it sends no flow control.
 *
 ****************************************************************************/

static inline_function bool isotp_listen_mode(FAR struct isotp_conn_s *isotp)
{
  return (isotp->opt.flags & CAN_ISOTP_LISTEN_MODE) != 0;
}

/****************************************************************************
 * Name: isotp_bc_flags
 *
 * Description:
 *   CAN_ISOTP_SF_BROADCAST or CAN_ISOTP_CF_BROADCAST (functional
 *   addressing): the socket only sends, and needs no RX ID.
 *
 ****************************************************************************/

static inline_function uint32_t
isotp_bc_flags(FAR struct isotp_conn_s *isotp)
{
  return isotp->opt.flags & ISOTP_BC_FLAGS;
}

/****************************************************************************
 * Name: isotp_rx_ext_address
 *
 * Description:
 *   The address received frames must carry with extended addressing.
 *
 ****************************************************************************/

static inline_function uint8_t
isotp_rx_ext_address(FAR struct isotp_conn_s *isotp)
{
  return (isotp->opt.flags & CAN_ISOTP_RX_EXT_ADDR) != 0 ?
         isotp->opt.rx_ext_address : isotp->opt.ext_address;
}

/****************************************************************************
 * Name: isotp_get_frame_info
 *
 * Description:
 *   Extract the CAN ID, payload pointer, and payload length from a CAN or
 *   CAN FD frame stored in an IOB.
 *
 * Input Parameters:
 *   iob       - IOB containing the CAN or CAN FD frame
 *   canid_out - Location to store the frame CAN ID
 *   data_out  - Location to store the payload data pointer
 *   dlen_out  - Location to store the payload length
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static inline_function int isotp_get_frame_info(FAR struct iob_s *iob,
                                                FAR canid_t *canid_out,
                                                FAR uint8_t **data_out,
                                                FAR uint8_t *dlen_out)
{
  FAR struct can_frame *cf = (FAR struct can_frame *)IOB_DATA(iob);

  if (iob->io_len < CAN_MTU ||
      (cf->can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0)
    {
      return -EINVAL;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (iob->io_pktlen == CANFD_MTU)
    {
      FAR struct canfd_frame *cfd =
        (FAR struct canfd_frame *)IOB_DATA(iob);

      if (iob->io_len < CANFD_MTU || cfd->len > CANFD_MAX_DLEN)
        {
          return -EINVAL;
        }

      *canid_out = cfd->can_id & ISOTP_CANID_MASK;
      *data_out  = cfd->data;
      *dlen_out  = cfd->len;
      return OK;
    }
#endif

  if (iob->io_pktlen != CAN_MTU || cf->can_dlc > CAN_MAX_DLEN)
    {
      return -EINVAL;
    }

  *canid_out = cf->can_id & ISOTP_CANID_MASK;
  *data_out  = cf->data;
  *dlen_out  = cf->can_dlc;
  return OK;
}

/****************************************************************************
 * Name: isotp_send_iob
 *
 * Description:
 *   Hand one prebuilt CAN frame to the device buffer (zero-copy).  The
 *   device buffer must be free (d_sndlen == 0).  Net locked.
 *
 ****************************************************************************/

static inline_function void isotp_send_iob(FAR struct net_driver_s *dev,
                                           FAR struct iob_s *iob)
{
  netdev_iob_replace(dev, iob);
  dev->d_sndlen = iob->io_pktlen;
#ifdef CONFIG_NET_CAN_CANFD
  dev->d_len = iob->io_pktlen >= CANFD_MTU ? CANFD_MTU : CAN_MTU;
#else
  dev->d_len = CAN_MTU;
#endif
}

/****************************************************************************
 * Name: isotp_notify_pollout
 ****************************************************************************/

static inline_function void
isotp_notify_pollout(FAR struct isotp_conn_s *isotp)
{
  int i;

  for (i = 0; i < CONFIG_NET_CAN_NPOLLWAITERS; i++)
    {
      FAR struct pollfd_s *fds = isotp->pollinfo[i].fds;

      if (fds != NULL && (fds->events & POLLOUT) != 0 &&
          can_iob_navail() > 0)
        {
          poll_notify(&fds, 1, POLLOUT);
        }
    }
}

/****************************************************************************
 * Name: isotp_rx_match
 *
 * Description:
 *   Decode a frame and report whether its CAN ID, and with extended
 *   addressing its first byte, match this socket.  The payload pointer and
 *   length, after any address byte, are returned for the caller's own
 *   PCI/length checks.
 *
 ****************************************************************************/

static inline_function bool isotp_rx_match(FAR struct isotp_conn_s *isotp,
                                           FAR struct iob_s *iob,
                                           FAR uint8_t **data_out,
                                           FAR uint8_t *dlen_out)
{
  canid_t canid;
  uint8_t ae = isotp_ae(isotp);

  if (isotp_bc_flags(isotp) != 0 ||
      isotp_get_frame_info(iob, &canid, data_out, dlen_out) < 0 ||
      canid != (isotp->rx_id & ISOTP_CANID_MASK) || *dlen_out < ae)
    {
      return false;
    }

  if (ae != 0)
    {
      if ((*data_out)[0] != isotp_rx_ext_address(isotp))
        {
          return false;
        }

      (*data_out)++;
      (*dlen_out)--;
    }

  return true;
}

/****************************************************************************
 * Name: isotp_rx_recover
 *
 * Description:
 *   Reset the receive path after transport/protocol failures.
 *
 ****************************************************************************/

static inline_function void isotp_rx_recover(FAR struct isotp_conn_s *isotp)
{
  conn_dev_lock(&isotp->conn.sconn, isotp->conn.dev);
  isotp->rx_active     = false;
  isotp->rx_fc_pending = false;
  isotp->rx_ready      = 0;
  iob_free_queue(&isotp->conn.readahead);
  IOB_QINIT(&isotp->conn.readahead);
  conn_dev_unlock(&isotp->conn.sconn, isotp->conn.dev);
}

/****************************************************************************
 * Name: isotp_rx_need_recover
 ****************************************************************************/

static inline_function bool isotp_rx_need_recover(int ret)
{
  /* Don't reset protocol state for transient caller-visible conditions */

  return ret < 0 && ret != -EAGAIN && ret != -EINTR;
}

/****************************************************************************
 * Name: isotp_fd_len
 *
 * Description:
 *   The shortest CAN FD data length that holds len bytes.
 *
 ****************************************************************************/

static inline_function uint8_t isotp_fd_len(uint8_t len)
{
  return can_dlc2bytes(can_bytes2dlc(len));
}

/****************************************************************************
 * Name: isotp_tx_pad
 *
 * Description:
 *   Pad a frame of len bytes: to 8 bytes with CAN_ISOTP_TX_PADDING, and
 *   always to a valid CAN FD length.  Returns the padded length.
 *
 ****************************************************************************/

static inline_function uint8_t isotp_tx_pad(FAR struct isotp_conn_s *isotp,
                                            FAR uint8_t *data, uint8_t len)
{
  bool    pad = (isotp->opt.flags & CAN_ISOTP_TX_PADDING) != 0;
  uint8_t padlen = len;

  if (pad && padlen < CAN_MAX_DLEN)
    {
      padlen = CAN_MAX_DLEN;
    }

  if (isotp->ll.mtu == CANFD_MTU)
    {
      padlen = isotp_fd_len(padlen);
    }

  memset(&data[len], pad ? isotp->opt.txpad_content :
                           CAN_ISOTP_DEFAULT_PAD_CONTENT, padlen - len);
  return padlen;
}

/****************************************************************************
 * Name: isotp_rx_pad_ok
 *
 * Description:
 *   Check the padding of a received frame whose padding starts at
 *   data[start] (CAN_ISOTP_CHK_PAD_LEN, CAN_ISOTP_CHK_PAD_DATA).  data and
 *   len leave out the address byte.  Without CAN_ISOTP_RX_PADDING,
 *   CHK_PAD_LEN asks for the shortest frame.
 *
 ****************************************************************************/

static bool isotp_rx_pad_ok(FAR struct isotp_conn_s *isotp,
                            FAR const uint8_t *data, uint8_t len,
                            uint8_t start)
{
  uint32_t flags = isotp->opt.flags;
  uint8_t  ae    = isotp_ae(isotp);
  uint8_t  i;

  if ((flags & CAN_ISOTP_RX_PADDING) == 0)
    {
      if ((flags & CAN_ISOTP_CHK_PAD_LEN) == 0)
        {
          return true;
        }

      start += ae;
      return len + ae == (start <= CAN_MAX_DLEN ? start :
                          isotp_fd_len(start));
    }

  if ((flags & CAN_ISOTP_CHK_PAD_LEN) != 0 && len + ae < CAN_MAX_DLEN)
    {
      return false;
    }

  if ((flags & CAN_ISOTP_CHK_PAD_DATA) != 0)
    {
      for (i = start; i < len; i++)
        {
          if (data[i] != isotp->opt.rxpad_content)
            {
              return false;
            }
        }
    }

  return true;
}

/****************************************************************************
 * Name: isotp_stmin_us
 *
 * Description:
 *   Convert an STmin byte to microseconds (ISO 15765-2): 0x00-0x7f are
 *   milliseconds, 0xf1-0xf9 are 100-900 us, and a reserved value means the
 *   longest STmin, 127 ms.
 *
 ****************************************************************************/

static inline_function uint32_t isotp_stmin_us(uint8_t stmin)
{
  if (stmin <= 0x7f)
    {
      return stmin * USEC_PER_MSEC;
    }

  if (stmin >= 0xf1 && stmin <= 0xf9)
    {
      return (stmin - 0xf0) * 100;
    }

  return 0x7f * USEC_PER_MSEC;
}

/****************************************************************************
 * Name: isotp_tx_gap_us
 *
 * Description:
 *   The time to leave between consecutive frames: frame_txtime (N_As) plus
 *   the receiver's STmin, or the forced one with CAN_ISOTP_FORCE_TXSTMIN.
 *
 ****************************************************************************/

static inline_function uint32_t
isotp_tx_gap_us(FAR struct isotp_conn_s *isotp, uint8_t stmin)
{
  uint64_t ns = isotp->tx_frame_ns;

  if ((isotp->opt.flags & CAN_ISOTP_FORCE_TXSTMIN) != 0)
    {
      ns += isotp->tx_force_ns;
    }
  else
    {
      ns += (uint64_t)isotp_stmin_us(stmin) * NSEC_PER_USEC;
    }

  return div_const_roundup(ns, NSEC_PER_USEC);
}

/****************************************************************************
 * Name: isotp_rx_input_fc
 *
 * Description:
 *   Consume a flow-control frame: advance, hold or end a waiting transfer,
 *   or drop a stray FC.  Net locked.
 *
 ****************************************************************************/

static inline_function uint32_t
isotp_rx_input_fc(FAR struct isotp_conn_s *isotp,
                  FAR struct net_driver_s *dev,
                  FAR const uint8_t *data, uint8_t dlen)
{
  FAR struct iob_s *iob = dev->d_iob;
  bool              kick = false;

  if (isotp->tx_state == ISOTP_TX_WAIT_FC)
    {
      if (dlen < ISOTP_FC_LEN ||
          !isotp_rx_pad_ok(isotp, data, dlen, ISOTP_FC_LEN))
        {
          isotp_tx_finish(isotp, -EBADMSG);
        }
      else
        {
          switch (data[0] & ISOTP_FC_FS_MASK)
            {
              case ISOTP_FC_CTS:

                /* BS and STmin of the first FC hold for the whole message
                 * (ISO 15765-2:2016), of each FC with DYN_FC_PARMS.
                 */

                if (!isotp->tx_fc_seen ||
                    (isotp->opt.flags & CAN_ISOTP_DYN_FC_PARMS) != 0)
                  {
                    isotp->tx_bs       = data[1];
                    isotp->tx_stmin_us = isotp_tx_gap_us(isotp, data[2]);
                    isotp->tx_fc_seen  = true;
                  }

                isotp->tx_bs_remaining = isotp->tx_bs;
                isotp->tx_last_send    = clock_systime_ticks();
                isotp->tx_state        = ISOTP_TX_SENDING;
                wd_cancel(&isotp->tx_wdog);
                kick = true;
                break;

              case ISOTP_FC_WAIT:

                /* Keep waiting: N_Bs starts again. */

                isotp_tx_wait_fc(isotp);
                break;

              case ISOTP_FC_OVFLW:
                isotp_tx_finish(isotp, -EMSGSIZE);
                break;

              default:
                isotp_tx_finish(isotp, -EBADMSG);
                break;
            }
        }
    }

  netdev_iob_clear(dev);
  iob_free_chain(iob);

  /* Kick the block only after releasing the RX frame, so the (possibly
   * synchronous) CAN_POLL does not re-enter devif_poll with d_iob set.
   */

  if (kick)
    {
      netdev_txnotify_dev(dev, CAN_POLL);
    }

  return 0;
}

/****************************************************************************
 * Name: isotp_rx_input_sf
 *
 * Description:
 *   Single Frame: abandon any half-done session, count one ready PDU.
 *   Net locked.
 *
 ****************************************************************************/

static inline_function void
isotp_rx_input_sf(FAR struct isotp_conn_s *isotp,
                  FAR const uint8_t *data, uint8_t dlen)
{
  uint8_t len = data[0] & ISOTP_SF_DL_MASK;

  isotp->rx_active = false;

  if (len == 0 ? dlen >= ISOTP_ESF_HDR_LEN &&
                 data[1] <= dlen - ISOTP_ESF_HDR_LEN
               : len <= dlen - 1)
    {
      isotp->rx_ready++;
    }
}

/****************************************************************************
 * Name: isotp_rx_input_ff
 *
 * Description:
 *   First Frame: open the RX session and queue the CTS flow control
 *   (none in listen mode).  Net locked.
 *
 ****************************************************************************/

static inline_function void
isotp_rx_input_ff(FAR struct isotp_conn_s *isotp,
                  FAR const uint8_t *data, uint8_t dlen)
{
  isotp_len_t total;
  isotp_len_t done;

  isotp->rx_active = false;

  if (dlen < 2)
    {
      return;
    }

#ifdef CONFIG_LIBC_CAN_ISOTP_LONG_PDU
  if ((data[0] & ISOTP_FF_DL_MASK) == 0 && data[1] == 0)
    {
      /* Escaped FF: 32-bit FF_DL. */

      if (dlen < ISOTP_ESC_FF_HDR_LEN)
        {
          return;
        }

      total = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
              ((uint32_t)data[4] << 8) | data[5];
      done  = dlen - ISOTP_ESC_FF_HDR_LEN;
    }
  else
#endif
    {
      total = ((data[0] & ISOTP_FF_DL_MASK) << 8) | data[1];
      done  = dlen - 2;
    }

  if (total == 0 || done >= total)
    {
      return;
    }

  isotp->rx_total      = total;
  isotp->rx_done       = done;
  isotp->rx_sn         = 1;
  isotp->rx_block      = 0;
  isotp->rx_cf_seen    = false;
  isotp->rx_active     = true;
  isotp->rx_fc_pending = !isotp_listen_mode(isotp);
}

/****************************************************************************
 * Name: isotp_rx_input_cf
 *
 * Description:
 *   Consecutive Frame: advance the RX session, queue flow control at block
 *   boundaries.  Returns false when the frame must be dropped.  Net locked.
 *
 ****************************************************************************/

static inline_function bool
isotp_rx_input_cf(FAR struct isotp_conn_s *isotp,
                  FAR const uint8_t *data, uint8_t dlen)
{
  if (!isotp->rx_active)
    {
      return false;
    }

  /* CAN_ISOTP_FORCE_RXSTMIN: drop a CF that comes too soon after the last
   * one taken.
   */

  if ((isotp->opt.flags & CAN_ISOTP_FORCE_RXSTMIN) != 0)
    {
      clock_t now = clock_systime_ticks();

      if (isotp->rx_cf_seen &&
          (clock_t)(now - isotp->rx_last_cf) <
          USEC2TICK(div_const_roundup(isotp->rx_force_ns, NSEC_PER_USEC)))
        {
          return false;
        }

      isotp->rx_cf_seen = true;
      isotp->rx_last_cf = now;
    }

  if ((data[0] & ISOTP_CF_SN_MASK) != isotp->rx_sn)
    {
      isotp->rx_active = false;

      /* Retain the failed PDU so recvmsg can report the sequence error. */

      isotp->rx_ready++;
      return true;
    }

  isotp->rx_sn    = (isotp->rx_sn + 1) & ISOTP_CF_SN_MASK;
  isotp->rx_done += MIN(dlen - 1, isotp->rx_total - isotp->rx_done);

  if (isotp->rx_done >= isotp->rx_total)
    {
      isotp->rx_active = false;
      isotp->rx_ready++;
    }
  else if (isotp->rx_fc.bs != 0 && ++isotp->rx_block >= isotp->rx_fc.bs)
    {
      isotp->rx_block      = 0;
      isotp->rx_fc_pending = !isotp_listen_mode(isotp);
    }

  return true;
}

/****************************************************************************
 * Name: isotp_build_fc_frame
 *
 * Description:
 *   Build a CTS flow-control frame into a new IOB.
 *
 ****************************************************************************/

static inline_function FAR struct iob_s *
isotp_build_fc_frame(FAR struct isotp_conn_s *conn)
{
  FAR struct iob_s *iob = can_iob_timedalloc(false, 0, NULL, NULL);
  uint8_t           ae  = isotp_ae(conn);

  if (iob == NULL)
    {
      return NULL;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (conn->ll.mtu == CANFD_MTU)
    {
      FAR struct canfd_frame *cfd =
        (FAR struct canfd_frame *)IOB_DATA(iob);

      cfd->can_id  = conn->tx_id;
      cfd->flags   = CANFD_FDF | (conn->ll.tx_flags & CANFD_BRS);
      cfd->data[0] = conn->opt.ext_address;
      isotp_build_fc(&cfd->data[ae], ISOTP_FC_CTS, conn->rx_fc.bs,
                     conn->rx_fc.stmin);
      cfd->len     = isotp_tx_pad(conn, cfd->data, ae + ISOTP_FC_LEN);
      iob->io_len = CANFD_MTU;
      iob->io_pktlen = CANFD_MTU;
    }
  else
#endif
    {
      FAR struct can_frame *cf =
        (FAR struct can_frame *)IOB_DATA(iob);

      cf->can_id  = conn->tx_id;
      cf->data[0] = conn->opt.ext_address;
      isotp_build_fc(&cf->data[ae], ISOTP_FC_CTS, conn->rx_fc.bs,
                     conn->rx_fc.stmin);
      cf->can_dlc = isotp_tx_pad(conn, cf->data, ae + ISOTP_FC_LEN);
      iob->io_len = CAN_MTU;
      iob->io_pktlen = CAN_MTU;
    }

  return iob;
}

/****************************************************************************
 * Name: isotp_wait_tx_done_enabled
 ****************************************************************************/

static inline_function bool
isotp_wait_tx_done_enabled(FAR struct isotp_conn_s *conn)
{
#ifdef CONFIG_NET_CAN_SOCK_OPTS
  return (conn->opt.flags & CAN_ISOTP_WAIT_TX_DONE) != 0;
#else
  return false;
#endif
}

/****************************************************************************
 * Name: isotp_tx_nbs_timeout
 *
 * Description:
 *   N_Bs watchdog: the receiver never sent flow control.  Runs in timer-ISR
 *   context, so it must not lock, transmit or free IOBs; it only releases
 *   the sender with an error.  tx_buf is freed by the next send/close.
 *
 ****************************************************************************/

static void isotp_tx_nbs_timeout(wdparm_t arg)
{
  FAR struct isotp_conn_s *isotp = (FAR struct isotp_conn_s *)arg;

  if (isotp->tx_state != ISOTP_TX_WAIT_FC)
    {
      return;
    }

  isotp->tx_result = -ECOMM;
  isotp->tx_state  = ISOTP_TX_IDLE;

  nxsem_post(&isotp->tx_done_sem);
  nxsem_post(&isotp->tx_sem);
}

/****************************************************************************
 * Name: isotp_tx_wait_fc
 *
 * Description:
 *   Suspend the sender after a (first or block-final) frame and arm the N_Bs
 *   timeout until the receiver's flow control arrives in isotp_input().
 *
 ****************************************************************************/

static void isotp_tx_wait_fc(FAR struct isotp_conn_s *isotp)
{
  isotp->tx_state = ISOTP_TX_WAIT_FC;
  wd_start(&isotp->tx_wdog, MSEC2TICK(ISOTP_FC_WAIT_MS),
           isotp_tx_nbs_timeout, (wdparm_t)isotp);
}

/****************************************************************************
 * Name: isotp_tx_stmin_ticks
 *
 * Description:
 *   STmin as a delay in ticks, at least one tick (a delayed item of N ticks
 *   may expire after as little as N-1 tick periods).
 *
 ****************************************************************************/

static inline_function clock_t
isotp_tx_stmin_ticks(FAR struct isotp_conn_s *isotp)
{
  clock_t ticks = USEC2TICK(isotp->tx_stmin_us);

  return ticks < 1 ? 1 : ticks;
}

/****************************************************************************
 * Name: isotp_tx_free
 ****************************************************************************/

static inline_function void isotp_tx_free(FAR struct isotp_conn_s *isotp)
{
  kmm_free(isotp->tx_buf);
  isotp->tx_buf = NULL;
}

/****************************************************************************
 * Name: isotp_tx_finish
 *
 * Description:
 *   End the transfer, store the result, release the slot.  Network locked.
 *
 ****************************************************************************/

static inline_function void
isotp_tx_finish(FAR struct isotp_conn_s *isotp, int result)
{
  if (result < 0)
    {
      nerr("ISO-TP TX aborted: %d\n", result);
    }

  wd_cancel(&isotp->tx_wdog);
  work_cancel(HPWORK, &isotp->tx_work);

  isotp->tx_result       = result;
  isotp->tx_state        = ISOTP_TX_IDLE;
  isotp->tx_bs_remaining = 0;
  isotp->tx_stmin_us     = 0;

  isotp_tx_free(isotp);

  isotp_notify_pollout(isotp);
  nxsem_post(&isotp->tx_done_sem);
  nxsem_post(&isotp->tx_sem);
}

/****************************************************************************
 * Name: isotp_tx_resume
 *
 * Description:
 *   Delayed HPWORK item for STmin pacing: re-poke the TX poll from task
 *   context after the STmin delay.  Used only while STmin > 0.
 *
 ****************************************************************************/

static void isotp_tx_resume(FAR void *arg)
{
  FAR struct isotp_conn_s *isotp = arg;

  if (isotp->conn.dev != NULL)
    {
      netdev_txnotify_dev(isotp->conn.dev, CAN_POLL);
    }
}

/****************************************************************************
 * Name: isotp_tx_eventhandler
 *
 * Description:
 *   CAN_POLL callback: hand one frame to the device buffer (zero-copy).
 *   STmin == 0: the driver's TX poll loop calls this again per frame (it
 *   drains a batch per poll and re-polls on TX-done) with no extra context
 *   switch; with STmin > 0 the next frame is deferred (below).  Net locked.
 *
 ****************************************************************************/

static uint32_t isotp_tx_eventhandler(FAR struct net_driver_s *dev,
                                      FAR void *pvpriv, uint32_t flags)
{
  FAR struct isotp_conn_s *isotp = pvpriv;
  FAR struct iob_s        *iob;
  uint8_t                  pci;
  int                      ret;

  if (isotp == NULL || (flags & CAN_POLL) == 0)
    {
      return flags;
    }

  /* The device buffer already holds an outgoing frame, or inbound data is
   * being processed: retry on the next poll.
   */

  if (dev->d_sndlen > 0 || (flags & CAN_NEWDATA) != 0)
    {
      return flags;
    }

  /* A queued flow control outranks our own data frames: the peer is
   * blocked waiting for it.
   */

  if (isotp->rx_fc_pending)
    {
      iob = isotp_build_fc_frame(isotp);
      if (iob != NULL)
        {
          isotp->rx_fc_pending = false;
          isotp_send_iob(dev, iob);
        }

      return flags;
    }

  if (isotp->tx_state != ISOTP_TX_SENDING)
    {
      return flags;
    }

  /* Honor the receiver's STmin between consecutive frames: every frame
   * after the first is a CF.  A CF that is still within STmin defers via a
   * one-shot work item (the only way to wait then send from task context).
   * STmin == 0 never defers.
   */

  if (isotp->tx_enc.done > 0 && isotp->tx_stmin_us > 0)
    {
      clock_t need    = isotp_tx_stmin_ticks(isotp);
      clock_t elapsed = (clock_t)(clock_systime_ticks() -
                                    isotp->tx_last_send);

      if (elapsed < need)
        {
          /* Once per window: re-queueing would reset the delay. */

          if (work_available(&isotp->tx_work))
            {
              work_queue(HPWORK, &isotp->tx_work, isotp_tx_resume,
                         isotp, need - elapsed);
            }

          return flags;
        }
    }

  ret = isotp_tx_frame(isotp, &iob);
  if (ret == -ENOMEM)
    {
      /* No IOB free: try again on the next tick. */

      if (work_available(&isotp->tx_work))
        {
          work_queue(HPWORK, &isotp->tx_work, isotp_tx_resume, isotp, 1);
        }

      return flags;
    }
  else if (ret < 0)
    {
      isotp_tx_finish(isotp, ret);
      return flags;
    }

  /* Hand the frame to the device for this poll (zero-copy). */

  pci = ISOTP_PCI_TYPE(((FAR struct can_frame *)IOB_DATA(iob))->data[
                       isotp_ae(isotp)]);
  isotp_send_iob(dev, iob);
  isotp->tx_last_send = clock_systime_ticks();

  if (pci == ISOTP_PCI_TYPE_FF &&
      (isotp->opt.flags & CAN_ISOTP_CF_BROADCAST) != 0)
    {
      /* No flow control in CF broadcast: send all CFs, paced only by
       * frame_txtime or the forced STmin.
       */

      isotp->tx_bs_remaining = 0;
      isotp->tx_stmin_us     = isotp_tx_gap_us(isotp, 0);
    }
  else if (pci == ISOTP_PCI_TYPE_FF)
    {
      /* First frame sent: wait for the receiver's flow control. */

      isotp_tx_wait_fc(isotp);
    }
  else if (ret == 0)
    {
      /* Single frame, or last consecutive frame: transfer complete. */

      isotp_tx_finish(isotp, OK);
    }
  else if (isotp->tx_bs_remaining > 0 && --isotp->tx_bs_remaining == 0)
    {
      /* Bounded block exhausted: wait for the next flow control. */

      isotp_tx_wait_fc(isotp);
    }

  /* Otherwise more consecutive frames remain in this block; the driver's TX
   * poll loop calls this handler again for the next one (no self-notify, to
   * avoid recursing into devif_poll).
   */

  return flags;
}

/****************************************************************************
 * Name: isotp_tx_start
 *
 * Description:
 *   Arm the CAN_POLL handler and poke a poll (device-locked so an FC can't
 *   race the setup).  On failure the caller frees tx_buf and tx_sem.
 *
 ****************************************************************************/

static int isotp_tx_start(FAR struct isotp_conn_s *isotp)
{
  FAR struct can_conn_s   *conn = &isotp->conn;
  FAR struct net_driver_s *dev  = conn->dev;

  if (dev == NULL || isotp->tx_cb == NULL)
    {
      return -ENODEV;
    }

  conn_dev_lock(&conn->sconn, dev);

  /* Drop a stale completion post from a prior fire-and-forget transfer. */

  nxsem_reset(&isotp->tx_done_sem, 0);

  isotp->tx_bs_remaining = 0;
  isotp->tx_fc_seen      = false;
  isotp->tx_state        = ISOTP_TX_SENDING;

  netdev_txnotify_dev(dev, CAN_POLL);

  conn_dev_unlock(&conn->sconn, dev);
  return OK;
}

/****************************************************************************
 * Name: isotp_tx_idle
 *
 * Description:
 *   Return true if no transmission is currently in progress (the tx_sem
 *   send slot is available).
 *
 ****************************************************************************/

static inline_function bool isotp_tx_idle(FAR struct isotp_conn_s *isotp)
{
  int val = 0;

  return nxsem_get_value(&isotp->tx_sem, &val) == OK && val > 0;
}

/****************************************************************************
 * Name: isotp_tx_prepare
 *
 * Description:
 *   Copy the user message: a send returns before the transfer is done.
 *
 ****************************************************************************/

static int isotp_tx_prepare(FAR struct isotp_conn_s *isotp,
                            FAR const uint8_t *buf, size_t len)
{
  int ret;

  ret = isotp_encode_init(&isotp->tx_enc, buf, len);
  if (ret < 0)
    {
      return ret;
    }

  /* SF broadcast: the message must fit one single frame. */

  if ((isotp->opt.flags & CAN_ISOTP_SF_BROADCAST) != 0 &&
      len > (isotp->ll.tx_dl > CAN_MAX_DLEN ?
             isotp->ll.tx_dl - ISOTP_ESF_HDR_LEN : CAN_MAX_DLEN - 1) -
            isotp_ae(isotp))
    {
      return -EINVAL;
    }

  isotp->tx_buf = kmm_malloc(len);
  if (isotp->tx_buf == NULL)
    {
      return -ENOMEM;
    }

  memcpy(isotp->tx_buf, buf, len);
  isotp->tx_enc.buf = isotp->tx_buf;
  isotp->tx_enc.ae  = isotp_ae(isotp);
  return OK;
}

/****************************************************************************
 * Name: isotp_tx_frame
 *
 * Description:
 *   Encode the next frame of the message into a new IOB.
 *
 * Returned Value:
 *   >0 when more frames follow, 0 for the last one, -ENOMEM when no IOB is
 *   free, or another negated errno value.
 *
 ****************************************************************************/

static int isotp_tx_frame(FAR struct isotp_conn_s *isotp,
                          FAR struct iob_s **iob_out)
{
  FAR struct iob_s     *iob;
  FAR struct can_frame *cf;
  unsigned int          frame_size;
  uint8_t               ae   = isotp_ae(isotp);
  uint8_t               dlen = isotp->ll.tx_dl - ae;
  int                   ret;

  iob = can_iob_timedalloc(false, 0, NULL, NULL);
  if (iob == NULL)
    {
      return -ENOMEM;
    }

  /* can_frame and canfd_frame both have data at the same byte offset,
   * so encode via the classic-frame view, then write the header below.
   */

  cf          = (FAR struct can_frame *)IOB_DATA(iob);
  cf->data[0] = isotp->opt.ext_address;
  ret         = isotp_encode_frame(&isotp->tx_enc, &cf->data[ae], &dlen);
  if (ret < 0)
    {
      iob_free_chain(iob);
      return ret;
    }

  dlen = isotp_tx_pad(isotp, cf->data, ae + dlen);

  if (isotp->ll.mtu == CANFD_MTU)
    {
      FAR struct canfd_frame *cfd = (FAR struct canfd_frame *)cf;

      cfd->can_id = isotp->tx_id;
      cfd->len    = dlen;
      cfd->flags  = CANFD_FDF | (isotp->ll.tx_flags & CANFD_BRS);
      frame_size  = CANFD_MTU;
    }
  else
    {
      cf->can_id  = isotp->tx_id;
      cf->can_dlc = dlen;
      frame_size  = CAN_MTU;
    }

  iob->io_len    = frame_size;
  iob->io_pktlen = frame_size;
  *iob_out       = iob;
  return ret;
}

/****************************************************************************
 * Name: isotp_match
 ****************************************************************************/

static bool isotp_match(FAR struct can_conn_s *can,
                        FAR struct net_driver_s *dev)
{
  FAR struct isotp_conn_s *isotp = isotp_from_conn(can);
  FAR uint8_t             *data;
  uint8_t                  dlen;

  return isotp_rx_match(isotp, dev->d_iob, &data, &dlen);
}

/****************************************************************************
 * Name: isotp_input
 *
 * Description:
 *   conn->input hook: dispatch one frame to the per-PCI handlers above.
 *   Data frames are then left on the read-ahead queue; queued flow control
 *   is sent without needing a reader in recvmsg().
 *
 ****************************************************************************/

static uint32_t isotp_input(FAR struct can_conn_s *conn,
                            FAR struct net_driver_s *dev)
{
  FAR struct isotp_conn_s *isotp = isotp_from_conn(conn);
  FAR struct iob_s        *iob = dev->d_iob;
  FAR uint8_t             *data;
  uint32_t                 flags;
  uint8_t                  dlen;

  if (!isotp_rx_match(isotp, iob, &data, &dlen) || dlen == 0)
    {
      return can_callback(dev, conn, CAN_NEWDATA);
    }

  /* Half duplex: while sending take only flow control, and while receiving
   * no flow control.
   */

  if ((isotp->opt.flags & CAN_ISOTP_HALF_DUPLEX) != 0 &&
      ((isotp->tx_state != ISOTP_TX_IDLE &&
        ISOTP_PCI_TYPE(data[0]) != ISOTP_PCI_TYPE_FC) ||
       (isotp->rx_active && ISOTP_PCI_TYPE(data[0]) == ISOTP_PCI_TYPE_FC)))
    {
      netdev_iob_clear(dev);
      iob_free_chain(iob);
      return 0;
    }

  switch (ISOTP_PCI_TYPE(data[0]))
    {
      case ISOTP_PCI_TYPE_FC:
        return isotp_rx_input_fc(isotp, dev, data, dlen);

      case ISOTP_PCI_TYPE_SF:
        isotp_rx_input_sf(isotp, data, dlen);
        break;

      case ISOTP_PCI_TYPE_FF:
        isotp_rx_input_ff(isotp, data, dlen);
        break;

      case ISOTP_PCI_TYPE_CF:
        if (!isotp_rx_input_cf(isotp, data, dlen))
          {
            /* Drop orphaned CFs of a dead transfer. */

            netdev_iob_clear(dev);
            iob_free_chain(iob);
            return 0;
          }

        break;

      default:

        /* Reserved PCI type: drop it, as it belongs to no message. */

        netdev_iob_clear(dev);
        iob_free_chain(iob);
        return 0;
    }

  flags = can_callback(dev, conn, CAN_NEWDATA);

  /* d_iob was consumed above, safe to kick a queued flow control now. */

  if (isotp->rx_fc_pending && isotp->tx_cb != NULL)
    {
      netdev_txnotify_dev(dev, CAN_POLL);
    }

  return flags;
}

/****************************************************************************
 * Name: isotp_rx_recv
 *
 * Description:
 *   Block until one CAN frame matching rx_id arrives, then return its
 *   ISO-TP payload bytes.
 *
 ****************************************************************************/

static int isotp_rx_recv(FAR struct isotp_conn_s *isotp,
                         FAR struct iob_s **iob_out,
                         bool nonblock, unsigned int timeout_ms)
{
  bool finite = timeout_ms != UINT_MAX;
  clock_t deadline = 0;

  if (finite)
    {
      deadline = clock_systime_ticks() + MSEC2TICK(timeout_ms);
    }

  for (; ; )
    {
      FAR struct iob_s *iob = NULL;
      unsigned int      recv_timeout;
      int               ret;

      if (finite)
        {
          clock_t remaining = deadline - clock_systime_ticks();

          if (remaining <= 0)
            {
              return -ETIMEDOUT;
            }

          recv_timeout = TICK2MSEC(remaining);
          if (recv_timeout == 0)
            {
              recv_timeout = 1;
            }
        }
      else
        {
          recv_timeout = UINT_MAX;
        }

      ret = can_recviob(&isotp->conn, &iob,
                        nonblock ? MSG_DONTWAIT : 0,
                        recv_timeout);
      if (ret >= 0)
        {
          *iob_out = iob;
          return OK;
        }
      else if (ret != -EAGAIN || nonblock || finite)
        {
          return ret;
        }
    }
}

/****************************************************************************
 * Name: isotp_setup
 ****************************************************************************/

static int isotp_setup(FAR struct socket *psock)
{
  FAR struct isotp_conn_s *conn;

  if (psock->s_domain != PF_CAN || psock->s_type != SOCK_DGRAM ||
      psock->s_proto != CAN_ISOTP)
    {
      return -EPROTONOSUPPORT;
    }

  conn = mempool_zallocate(&g_isotp_conns, 0);
  if (conn == NULL)
    {
      return -ENOMEM;
    }

  conn->conn.match = isotp_match;
  conn->conn.input = isotp_input;
  conn->ll.mtu     = CAN_MTU;
  conn->ll.tx_dl   = CAN_MAX_DLEN;
  conn->rx_fc.stmin = ISOTP_RX_STMIN_MS;
  nxmutex_init(&conn->rx_lock);
  nxsem_init(&conn->tx_sem, 0, 1);
  nxsem_init(&conn->tx_done_sem, 0, 0);
  can_conn_init(&conn->conn);
  psock->s_conn = &conn->conn;
  return OK;
}

/****************************************************************************
 * Name: isotp_sockcaps
 ****************************************************************************/

static sockcaps_t isotp_sockcaps(FAR struct socket *psock)
{
  return SOCKCAP_NONBLOCKING;
}

/****************************************************************************
 * Name: isotp_addref
 ****************************************************************************/

static void isotp_addref(FAR struct socket *psock)
{
  FAR struct isotp_conn_s *isotp = isotp_conn(psock);

  DEBUGASSERT(isotp->conn.crefs > 0 && isotp->conn.crefs < SHRT_MAX);
  isotp->conn.crefs++;
}

/****************************************************************************
 * Name: isotp_bind
 *
 * Description:
 *   Bind the socket to a CAN interface.
 *
 *   addr->can_addr.tp.tx_id is the X CAN ID and addr->can_addr.tp.rx_id is
 *   the RX CAN ID.
 *
 ****************************************************************************/

static int isotp_bind(FAR struct socket *psock,
                      FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct isotp_conn_s *conn = isotp_conn(psock);
  FAR struct can_conn_s   *can  = &conn->conn;
  FAR struct sockaddr_can *canaddr;
  FAR struct net_driver_s *dev;
  int                      ret;

  if (addr == NULL || addrlen < sizeof(struct sockaddr_can) ||
      addr->sa_family != AF_CAN || _SS_ISBOUND(can->sconn.s_flags))
    {
      return -EINVAL;
    }

  /* A broadcast socket does not receive, so its RX ID is not checked. */

  canaddr = (FAR struct sockaddr_can *)addr;
  if (canaddr->can_ifindex <= 0 ||
      (isotp_bc_flags(conn) == 0 &&
       (canaddr->can_addr.tp.rx_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0) ||
      (canaddr->can_addr.tp.tx_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0)
    {
      return -EINVAL;
    }

  dev = netdev_findbyindex(canaddr->can_ifindex);
  if (dev == NULL || dev->d_lltype != NET_LL_CAN)
    {
      return -ENODEV;
    }

  /* Publish the address and callback together under the device lock. */

  conn_dev_lock(&can->sconn, dev);
  conn->tx_cb = can_callback_alloc(dev, can);
  ret = -ENOMEM;
  if (conn->tx_cb != NULL)
    {
      conn->tx_id = canaddr->can_addr.tp.tx_id;
      conn->rx_id = canaddr->can_addr.tp.rx_id;
      conn->tx_cb->flags = CAN_POLL;
      conn->tx_cb->priv = conn;
      conn->tx_cb->event = isotp_tx_eventhandler;
      can->dev = dev;
      ret = OK;
    }

  conn_dev_unlock(&can->sconn, dev);
  return ret;
}

/****************************************************************************
 * Name: isotp_getsockname
 *
 * Description:
 *   The isotp_getsockname() function retrieves the locally-bound name of
 *   the specified ISO-TP socket, stores this address in the sockaddr
 *   structure pointed to by the 'addr' argument, and stores the length of
 *   this address in the object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Input Parameters:
 *   psock    Socket structure of the socket to be queried
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address.  Otherwise, a negated errno value is returned.  See
 *   getsockname() for the list of appropriate error numbers.
 *
 ****************************************************************************/

static int isotp_getsockname(FAR struct socket *psock,
                             FAR struct sockaddr *addr,
                             FAR socklen_t *addrlen)
{
  FAR struct isotp_conn_s *conn = isotp_conn(psock);
  struct sockaddr_can      canaddr;
  socklen_t                copylen;

  memset(&canaddr, 0, sizeof(canaddr));
  canaddr.can_family        = AF_CAN;
  canaddr.can_ifindex       = conn->conn.dev == NULL ? 0 :
                             conn->conn.dev->d_ifindex;
  canaddr.can_addr.tp.rx_id = conn->rx_id;
  canaddr.can_addr.tp.tx_id = conn->tx_id;

  copylen = sizeof(canaddr);
  if (copylen > *addrlen)
    {
      copylen = *addrlen;
    }

  memcpy(addr, &canaddr, copylen);
  *addrlen = copylen;
  return OK;
}

/****************************************************************************
 * Name: isotp_poll_eventhandler
 ****************************************************************************/

static uint32_t isotp_poll_eventhandler(FAR struct net_driver_s *dev,
                                        FAR void *pvpriv, uint32_t flags)
{
  FAR struct isotp_poll_s *info = pvpriv;
  FAR struct isotp_conn_s *isotp;
  pollevent_t eventset = 0;

  DEBUGASSERT(!info || (info->conn && info->fds));

  if (info == NULL)
    {
      return flags;
    }

  isotp = isotp_from_conn(info->conn);

  if ((flags & CAN_NEWDATA) != 0 && isotp->rx_ready > 0)
    {
      eventset |= POLLIN;
    }

  if ((flags & NETDEV_DOWN) != 0)
    {
      eventset |= (POLLHUP | POLLERR);
    }
  else if ((flags & CAN_POLL) != 0 && isotp_tx_idle(isotp) &&
           can_iob_navail() > 0)
    {
      eventset |= POLLOUT;
    }

  poll_notify(&info->fds, 1, eventset);
  return flags;
}

/****************************************************************************
 * Name: isotp_poll
 ****************************************************************************/

static int isotp_poll(FAR struct socket *psock, FAR struct pollfd_s *fds,
                      bool setup)
{
  FAR struct isotp_conn_s *isotp = isotp_conn(psock);
  FAR struct can_conn_s *conn = &isotp->conn;
  FAR struct isotp_poll_s *info = NULL;
  FAR struct devif_callback_s *cb;
  pollevent_t eventset = 0;
  int ret = OK;
  int i;

  if (setup)
    {
      conn_dev_lock(&conn->sconn, conn->dev);

      for (i = 0; i < CONFIG_NET_CAN_NPOLLWAITERS; i++)
        {
          if (isotp->pollinfo[i].fds == NULL)
            {
              info = &isotp->pollinfo[i];
              break;
            }
        }

      if (info == NULL)
        {
          ret = -EBUSY;
          goto errout_with_lock;
        }

      info->dev = conn->dev;

      cb = can_callback_alloc(info->dev, conn);
      if (cb == NULL)
        {
          ret = -EBUSY;
          goto errout_with_lock;
        }

      info->conn = conn;
      info->fds   = fds;
      info->cb    = cb;

      cb->flags = NETDEV_DOWN;
      cb->priv  = info;
      cb->event = isotp_poll_eventhandler;

      if ((fds->events & POLLOUT) != 0)
        {
          cb->flags |= CAN_POLL;
        }

      if ((fds->events & POLLIN) != 0)
        {
          cb->flags |= CAN_NEWDATA;
        }

      fds->priv = info;

      if ((fds->events & POLLIN) != 0 && isotp->rx_ready > 0)
        {
          eventset |= POLLRDNORM;
        }

      if ((fds->events & POLLOUT) != 0 && isotp_tx_idle(isotp) &&
          can_iob_navail() > 0)
        {
          eventset |= POLLWRNORM;
        }

      poll_notify(&fds, 1, eventset);

errout_with_lock:
      conn_dev_unlock(&conn->sconn, conn->dev);
    }
  else
    {
      info = (FAR struct isotp_poll_s *)fds->priv;

      if (info != NULL)
        {
          conn_dev_lock(&conn->sconn, info->dev);
          can_callback_free(info->dev, conn, info->cb);

          info->fds->priv = NULL;
          info->fds = NULL;

          info->conn = NULL;
          conn_dev_unlock(&conn->sconn, info->dev);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: isotp_sendmsg
 *
 * Description:
 *   Copy the message and start the event-driven engine, which builds each
 *   frame as it goes out.
 *   WAIT_TX_DONE waits for completion and returns the result; otherwise the
 *   call returns immediately (fire-and-forget).
 *
 ****************************************************************************/

static ssize_t isotp_sendmsg(FAR struct socket *psock,
                             FAR const struct msghdr *msg, int flags)
{
  FAR struct isotp_conn_s *isotp = isotp_conn(psock);
  FAR const uint8_t       *buf;
  size_t                   len;
  bool                     wait_tx_done;
  bool                     nonblock;
  int                      ret;

  if (msg == NULL || msg->msg_iovlen != 1 || msg->msg_iov->iov_len == 0)
    {
      return -EINVAL;
    }

  buf = msg->msg_iov->iov_base;
  len = msg->msg_iov->iov_len;

  wait_tx_done = isotp_wait_tx_done_enabled(isotp);
  nonblock = _SS_ISNONBLOCK(isotp->conn.sconn.s_flags) ||
             (flags & MSG_DONTWAIT) != 0;

  if (nonblock && wait_tx_done)
    {
      return -EAGAIN;
    }

  /* Take the send slot: a blocking socket waits for a prior transfer to
   * release it (bounded by SO_SNDTIMEO); a non-blocking socket fails fast.
   */

  if (nonblock)
    {
      ret = nxsem_trywait(&isotp->tx_sem);
    }
  else
    {
      unsigned int timeout = _SO_TIMEOUT(isotp->conn.sconn.s_sndtimeo);

      if (timeout != UINT_MAX)
        {
          ret = nxsem_tickwait(&isotp->tx_sem, MSEC2TICK(timeout));
        }
      else
        {
          ret = nxsem_wait(&isotp->tx_sem);
        }
    }

  if (ret < 0)
    {
      return ret;
    }

  /* Free the copy left by a prior aborted (N_Bs) transfer, copy the new
   * message and start the engine.
   */

  isotp_tx_free(isotp);

  ret = isotp_tx_prepare(isotp, buf, len);
  if (ret >= 0)
    {
      ret = isotp_tx_start(isotp);
    }

  if (ret < 0)
    {
      isotp_tx_free(isotp);
      nxsem_post(&isotp->tx_sem);
      return ret;
    }

  if (!wait_tx_done)
    {
      /* engine releases the slot via isotp_tx_finish() */

      return len;
    }

  /* WAIT_TX_DONE: wait for the engine (bounded by the N_Bs timeout). */

  nxsem_wait_uninterruptible(&isotp->tx_done_sem);
  return isotp->tx_result < 0 ? isotp->tx_result : len;
}

/****************************************************************************
 * Name: isotp_recvmsg
 *
 * Description:
 *   Receive one complete ISO-TP message into msg->msg_iov[0].
 *
 ****************************************************************************/

static ssize_t isotp_recvmsg(FAR struct socket *psock,
                             FAR struct msghdr *msg, int flags)
{
  FAR struct isotp_conn_s *isotp = isotp_conn(psock);
  FAR struct iob_s        *iob = NULL;
  FAR uint8_t             *buf;
  FAR uint8_t             *frame;
  struct isotp_decode_s    dec;
  isotp_len_t              buflen;
  canid_t                  canid;
  uint8_t                  dlen;
  unsigned int             timeout;
  bool                     nonblock;
  bool                     first = true;
  bool                     badpad = false;
  isotp_len_t              prev;
  ssize_t                  ret;

  if (msg == NULL || msg->msg_iovlen != 1)
    {
      return -EINVAL;
    }

  buf    = msg->msg_iov->iov_base;
#ifdef CONFIG_LIBC_CAN_ISOTP_LONG_PDU
  buflen = msg->msg_iov->iov_len;
#else
  buflen = MIN(msg->msg_iov->iov_len, ISOTP_MAX_MSG_LEN);
#endif

  nonblock = _SS_ISNONBLOCK(isotp->conn.sconn.s_flags) ||
             (flags & MSG_DONTWAIT) != 0;
  timeout  = _SO_TIMEOUT(isotp->conn.sconn.s_rcvtimeo);

  if (buflen == 0)
    {
      return 0;
    }

  /* Serialize receivers; SO_RCVTIMEO covers the lock wait too. */

  if (nonblock)
    {
      ret = nxmutex_trylock(&isotp->rx_lock);
      if (ret < 0)
        {
          return -EAGAIN;
        }
    }
  else if (timeout != UINT_MAX)
    {
      clock_t  start = clock_systime_ticks();
      clock_t left;

      ret = nxmutex_ticklock(&isotp->rx_lock, MSEC2TICK(timeout));
      if (ret < 0)
        {
          return ret == -ETIMEDOUT ? -EAGAIN : ret;
        }

      left = (clock_t)(MSEC2TICK(timeout) -
                        (clock_systime_ticks() - start));
      if (left <= 0)
        {
          nxmutex_unlock(&isotp->rx_lock);
          return -EAGAIN;
        }

      timeout = TICK2MSEC(left);
      if (timeout == 0)
        {
          timeout = 1;
        }
    }
  else
    {
      ret = nxmutex_lock(&isotp->rx_lock);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (nonblock)
    {
      bool ready;

      conn_dev_lock(&isotp->conn.sconn, isotp->conn.dev);
      ready = isotp->rx_ready != 0;
      conn_dev_unlock(&isotp->conn.sconn, isotp->conn.dev);
      if (!ready)
        {
          nxmutex_unlock(&isotp->rx_lock);
          return -EAGAIN;
        }
    }

  isotp_decode_init(&dec, buf, buflen);

  for (; ; )
    {
      ret = isotp_rx_recv(isotp, &iob,
                          nonblock && first,
                          first ? timeout : ISOTP_CR_WAIT_MS);
      if (ret < 0)
        {
          if (!first && (ret == -ETIMEDOUT || ret == -EAGAIN))
            {
              /* N_Cr timeout: sender stopped after FF */

              ret = -ECOMM;
            }
          else if (ret == -ETIMEDOUT)
            {
              ret = -EAGAIN;
            }

          goto out;
        }

      ret = isotp_get_frame_info(iob, &canid, &frame, &dlen);
      if (ret < 0 || dlen <= isotp_ae(isotp))
        {
          ret = ret < 0 ? ret : -EINVAL;
          goto out;
        }

      frame += isotp_ae(isotp);
      dlen  -= isotp_ae(isotp);

      first = false;

      prev = dec.done;
      ret  = isotp_decode_frame(&dec, frame, dlen);
      if (ret < 0)
        {
          goto out;
        }

      /* The last frame of a message (SF or CF) may be padded after the
       * header (1 byte, 2 for an escaped SF) and its data.
       */

      if (ret == 0 &&
          !isotp_rx_pad_ok(isotp, frame, dlen,
                           (frame[0] == ISOTP_PCI_TYPE_SF ? 2 : 1) +
                           dec.done - prev))
        {
          badpad = true;
        }

      iob_free_chain(iob);
      iob = NULL;

      if (ret == 0)
        {
          break;
        }
    }

  conn_dev_lock(&isotp->conn.sconn, isotp->conn.dev);
  if (isotp->rx_ready > 0)
    {
      isotp->rx_ready--;
    }

  conn_dev_unlock(&isotp->conn.sconn, isotp->conn.dev);

  /* A badly padded message is complete: report it, nothing to recover. */

  ret = badpad ? -EBADMSG : dec.done;

out:
  iob_free_chain(iob);

  if (!badpad && isotp_rx_need_recover(ret))
    {
      isotp_rx_recover(isotp);
    }

  nxmutex_unlock(&isotp->rx_lock);
  return ret;
}

/****************************************************************************
 * Name: isotp_close
 ****************************************************************************/

static int isotp_close(FAR struct socket *psock)
{
  FAR struct can_conn_s *can = psock->s_conn;
  FAR struct isotp_conn_s *conn = isotp_from_conn(can);

  if (can->crefs <= 1)
    {
      /* Drain any in-flight TX (finish posts tx_sem), then free resources. */

      nxsem_wait_uninterruptible(&conn->tx_sem);

      wd_cancel(&conn->tx_wdog);
      work_cancel_sync(HPWORK, &conn->tx_work);

      if (conn->tx_cb != NULL)
        {
          can_callback_free(can->dev, can, conn->tx_cb);
          conn->tx_cb = NULL;
        }

      isotp_tx_free(conn);

      nxsem_destroy(&conn->tx_sem);
      nxsem_destroy(&conn->tx_done_sem);
      nxmutex_destroy(&conn->rx_lock);
      can->crefs = 0;
      can_conn_uninit(can);
      mempool_release(&g_isotp_conns, conn);
    }
  else
    {
      can->crefs--;
    }

  return OK;
}

#ifdef CONFIG_NET_CAN_SOCK_OPTS
/****************************************************************************
 * Name: isotp_valid_llopts
 ****************************************************************************/

static inline_function bool
isotp_valid_llopts(FAR const struct can_isotp_ll_options *ll)
{
  if (ll->mtu == CAN_MTU)
    {
      return ll->tx_dl == CAN_MAX_DLEN && ll->tx_flags == 0;
    }

#ifdef CONFIG_NET_CAN_CANFD
  if (ll->mtu == CANFD_MTU && (ll->tx_flags & ~CANFD_BRS) == 0)
    {
      switch (ll->tx_dl)
        {
          case 8:
          case 12:
          case 16:
          case 20:
          case 24:
          case 32:
          case 48:
          case 64:
            return true;

          default:
            return false;
        }
    }
#endif

  return false;
}

/****************************************************************************
 * Name: isotp_getsockopt
 ****************************************************************************/

static int isotp_getsockopt(FAR struct socket *psock, int level,
                            int option, FAR void *value,
                            FAR socklen_t *value_len)
{
  FAR struct isotp_conn_s *conn = isotp_conn(psock);

  if (level != SOL_CAN_ISOTP)
    {
      return -ENOPROTOOPT;
    }

  switch (option)
    {
      case CAN_ISOTP_OPTS:
        {
          if (*value_len < sizeof(conn->opt))
            {
              return -EINVAL;
            }

          memcpy(value, &conn->opt, sizeof(conn->opt));
          *value_len = sizeof(conn->opt);
          return OK;
        }

      case CAN_ISOTP_LL_OPTS:
        {
          if (*value_len < sizeof(conn->ll))
            {
              return -EINVAL;
            }

          memcpy(value, &conn->ll, sizeof(conn->ll));
          *value_len = sizeof(conn->ll);
          return OK;
        }

      case CAN_ISOTP_TX_STMIN:
      case CAN_ISOTP_RX_STMIN:
        {
          if (*value_len < sizeof(uint32_t))
            {
              return -EINVAL;
            }

          *(FAR uint32_t *)value = option == CAN_ISOTP_TX_STMIN ?
                                   conn->tx_force_ns : conn->rx_force_ns;
          *value_len = sizeof(uint32_t);
          return OK;
        }

      case CAN_ISOTP_RECV_FC:
        {
          if (*value_len < sizeof(conn->rx_fc))
            {
              return -EINVAL;
            }

          memcpy(value, &conn->rx_fc, sizeof(conn->rx_fc));
          *value_len = sizeof(conn->rx_fc);
          return OK;
        }

      default:
        return -ENOPROTOOPT;
    }
}

/****************************************************************************
 * Name: isotp_setsockopt
 ****************************************************************************/

static int isotp_setsockopt(FAR struct socket *psock, int level,
                            int option, FAR const void *value,
                            socklen_t value_len)
{
  FAR struct isotp_conn_s *conn = isotp_conn(psock);

  if (level != SOL_CAN_ISOTP)
    {
      return -ENOPROTOOPT;
    }

  switch (option)
    {
      case CAN_ISOTP_OPTS:
        {
          FAR const struct can_isotp_options *opt = value;

          if (value_len < sizeof(*opt))
            {
              return -EINVAL;
            }

          if ((opt->flags & ~ISOTP_OPTS_FLAGS) != 0 ||
              (opt->flags & ISOTP_BC_FLAGS) == ISOTP_BC_FLAGS)
            {
              return -EINVAL;
            }

          /* frame_txtime: 0 keeps the current value, FRAME_TXTIME_ZERO
           * sets 0.
           */

          if (opt->frame_txtime == CAN_ISOTP_FRAME_TXTIME_ZERO)
            {
              conn->tx_frame_ns = 0;
            }
          else if (opt->frame_txtime != 0)
            {
              conn->tx_frame_ns = opt->frame_txtime;
            }

          conn->opt = *opt;
          return OK;
        }

      case CAN_ISOTP_TX_STMIN:
      case CAN_ISOTP_RX_STMIN:
        {
          if (value_len < sizeof(uint32_t))
            {
              return -EINVAL;
            }

          if (option == CAN_ISOTP_TX_STMIN)
            {
              conn->tx_force_ns = *(FAR const uint32_t *)value;
            }
          else
            {
              conn->rx_force_ns = *(FAR const uint32_t *)value;
            }

          return OK;
        }

      case CAN_ISOTP_LL_OPTS:
        {
          FAR const struct can_isotp_ll_options *ll = value;

          if (value_len < sizeof(*ll))
            {
              return -EINVAL;
            }

          if (!isotp_valid_llopts(ll))
            {
              return -EINVAL;
            }

          conn->ll = *ll;
          return OK;
        }

      case CAN_ISOTP_RECV_FC:
        {
          FAR const struct can_isotp_fc_options *fc = value;

          if (value_len < sizeof(*fc))
            {
              return -EINVAL;
            }

          conn->rx_fc = *fc;
          return OK;
        }

      default:
        return -ENOPROTOOPT;
    }
}
#endif /* CONFIG_NET_CAN_SOCK_OPTS */
