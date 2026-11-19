/****************************************************************************
 * arch/arm/src/mspm0/mspm0_serial.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/mspm0_uart.h"
#include "mspm0_lowputc.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Which UART is the console, and which is ttyS0.  Only UART0 for now. */

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_MSPM0_UART0)
#  define CONSOLE_DEV  g_uart0port
#  define TTYS0_DEV    g_uart0port
#elif defined(CONFIG_MSPM0_UART0)
#  define TTYS0_DEV    g_uart0port
#endif

/* Interrupts the driver uses */

#define UART_RXINTS    (UART_INT_RXINT | UART_INT_RTOUT)
#define UART_TXINTS    (UART_INT_TXINT)
#define UART_ERRINTS   (UART_INT_OVRERR | UART_INT_BRKERR | \
                        UART_INT_PARERR | UART_INT_FRMERR)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mspm0_uart_s
{
  uintptr_t base;               /* Register base address */
  uint32_t  clock;              /* Bus clock feeding the UART */
  uint32_t  imask;              /* Shadow of the interrupt mask */
  uint8_t   irq;                /* IRQ number */
  uint8_t   txpincm;            /* TX pin (PINCM number) */
  uint8_t   txpf;               /* TX pin function */
  uint8_t   rxpincm;            /* RX pin (PINCM number) */
  uint8_t   rxpf;               /* RX pin function */
  struct mspm0_uartcfg_s cfg;   /* Line settings */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  mspm0_setup(struct uart_dev_s *dev);
static void mspm0_shutdown(struct uart_dev_s *dev);
static int  mspm0_attach(struct uart_dev_s *dev);
static void mspm0_detach(struct uart_dev_s *dev);
static int  mspm0_interrupt(int irq, void *context, void *arg);
static int  mspm0_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  mspm0_receive(struct uart_dev_s *dev, unsigned int *status);
static void mspm0_rxint(struct uart_dev_s *dev, bool enable);
static bool mspm0_rxavailable(struct uart_dev_s *dev);
static void mspm0_send(struct uart_dev_s *dev, int ch);
static void mspm0_txint(struct uart_dev_s *dev, bool enable);
static bool mspm0_txready(struct uart_dev_s *dev);
static bool mspm0_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct uart_ops_s g_uart_ops =
{
  .setup       = mspm0_setup,
  .shutdown    = mspm0_shutdown,
  .attach      = mspm0_attach,
  .detach      = mspm0_detach,
  .ioctl       = mspm0_ioctl,
  .receive     = mspm0_receive,
  .rxint       = mspm0_rxint,
  .rxavailable = mspm0_rxavailable,
  .send        = mspm0_send,
  .txint       = mspm0_txint,
  .txready     = mspm0_txready,
  .txempty     = mspm0_txempty,
};

#ifdef CONFIG_MSPM0_UART0
static char g_uart0rxbuffer[CONFIG_UART0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_UART0_TXBUFSIZE];

static struct mspm0_uart_s g_uart0priv =
{
  .base    = MSPM0_UART0_BASE,
  .clock   = BOARD_ULPCLK_FREQUENCY,   /* UART0 is in power domain 0 */
  .irq     = MSPM0_IRQ_UART0,
  .txpincm = BOARD_UART0_TX_PINCM,
  .txpf    = BOARD_UART0_TX_PF,
  .rxpincm = BOARD_UART0_RX_PINCM,
  .rxpf    = BOARD_UART0_RX_PF,
  .cfg     =
  {
    .baud   = CONFIG_UART0_BAUD,
    .bits   = CONFIG_UART0_BITS,
    .parity = CONFIG_UART0_PARITY,
    .stop2  = CONFIG_UART0_2STOP,
  },
};

static uart_dev_t g_uart0port =
{
  .recv =
  {
    .size   = CONFIG_UART0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit =
  {
    .size   = CONFIG_UART0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops  = &g_uart_ops,
  .priv = &g_uart0priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mspm0_setimask
 *
 * Description:
 *   Set the interrupt mask and keep the shadow copy in step.
 *
 ****************************************************************************/

static inline void mspm0_setimask(struct mspm0_uart_s *priv, uint32_t imask)
{
  priv->imask = imask;
  putreg32(imask, priv->base + MSPM0_CPU_INT_IMASK_OFFSET);
}

/****************************************************************************
 * Name: mspm0_setup
 *
 * Description:
 *   Configure the UART: pins, baud rate, format.  Called the first time
 *   the port is opened, and early for the console.
 *
 ****************************************************************************/

static int mspm0_setup(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;

#ifndef CONFIG_SUPPRESS_UART_CONFIG
  mspm0_uart_pins(priv->txpincm, priv->txpf, priv->rxpincm, priv->rxpf);
  mspm0_uart_configure(priv->base, priv->clock, &priv->cfg);
#endif

  mspm0_setimask(priv, 0);
  return OK;
}

/****************************************************************************
 * Name: mspm0_shutdown
 *
 * Description:
 *   Disable the UART.  Called when the port is closed, unless it is the
 *   console.
 *
 ****************************************************************************/

static void mspm0_shutdown(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;

  mspm0_setimask(priv, 0);
  mspm0_uart_disable(priv->base);
}

/****************************************************************************
 * Name: mspm0_attach
 *
 * Description:
 *   Attach and enable the UART interrupt.  The individual sources stay
 *   masked until rxint() and txint() enable them.
 *
 ****************************************************************************/

static int mspm0_attach(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;
  int ret;

  ret = irq_attach(priv->irq, mspm0_interrupt, dev);
  if (ret == OK)
    {
      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: mspm0_detach
 ****************************************************************************/

static void mspm0_detach(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: mspm0_interrupt
 *
 * Description:
 *   UART interrupt handler.  Interrupt flags are events that stay set
 *   until cleared, so clear them before servicing: anything that arrives
 *   while servicing then sets them again.
 *
 ****************************************************************************/

static int mspm0_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = arg;
  struct mspm0_uart_s *priv = dev->priv;
  uint32_t mis;
  int passes;

  for (passes = 0; passes < 256; passes++)
    {
      mis = getreg32(priv->base + MSPM0_CPU_INT_MIS_OFFSET);
      if (mis == 0)
        {
          break;
        }

      putreg32(mis, priv->base + MSPM0_CPU_INT_ICLR_OFFSET);

      if ((mis & UART_RXINTS) != 0)
        {
          uart_recvchars(dev);
        }

      if ((mis & UART_TXINTS) != 0)
        {
          uart_xmitchars(dev);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mspm0_ioctl
 ****************************************************************************/

static int mspm0_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Name: mspm0_receive
 *
 * Description:
 *   Return one received byte, with any error flags in *status.
 *
 ****************************************************************************/

static int mspm0_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct mspm0_uart_s *priv = dev->priv;
  uint32_t rxdata;

  rxdata  = getreg32(priv->base + MSPM0_UART_RXDATA_OFFSET);
  *status = rxdata & UART_RXDATA_ERRORS;
  return rxdata & UART_RXDATA_DATA_MASK;
}

/****************************************************************************
 * Name: mspm0_rxint
 ****************************************************************************/

static void mspm0_rxint(struct uart_dev_s *dev, bool enable)
{
  struct mspm0_uart_s *priv = dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      mspm0_setimask(priv, priv->imask | UART_RXINTS | UART_ERRINTS);
#endif
    }
  else
    {
      mspm0_setimask(priv, priv->imask & ~(UART_RXINTS | UART_ERRINTS));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mspm0_rxavailable
 ****************************************************************************/

static bool mspm0_rxavailable(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;

  return (getreg32(priv->base + MSPM0_UART_STAT_OFFSET) &
          UART_STAT_RXFE) == 0;
}

/****************************************************************************
 * Name: mspm0_send
 ****************************************************************************/

static void mspm0_send(struct uart_dev_s *dev, int ch)
{
  struct mspm0_uart_s *priv = dev->priv;

  putreg32((uint32_t)ch, priv->base + MSPM0_UART_TXDATA_OFFSET);
}

/****************************************************************************
 * Name: mspm0_txint
 *
 * Description:
 *   Enable or disable the TX interrupt.  The TX interrupt fires when the
 *   FIFO level drops past its threshold, which may never happen if the
 *   FIFO is already empty, so fill the FIFO here when enabling.
 *
 ****************************************************************************/

static void mspm0_txint(struct uart_dev_s *dev, bool enable)
{
  struct mspm0_uart_s *priv = dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();

  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      mspm0_setimask(priv, priv->imask | UART_TXINTS);
      uart_xmitchars(dev);
#endif
    }
  else
    {
      mspm0_setimask(priv, priv->imask & ~UART_TXINTS);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: mspm0_txready
 ****************************************************************************/

static bool mspm0_txready(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;

  return (getreg32(priv->base + MSPM0_UART_STAT_OFFSET) &
          UART_STAT_TXFF) == 0;
}

/****************************************************************************
 * Name: mspm0_txempty
 ****************************************************************************/

static bool mspm0_txempty(struct uart_dev_s *dev)
{
  struct mspm0_uart_s *priv = dev->priv;

  return (getreg32(priv->base + MSPM0_UART_STAT_OFFSET) &
          UART_STAT_TXFE) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Set up the console early so that it is usable before the OS starts.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
#ifdef CONSOLE_DEV
  CONSOLE_DEV.isconsole = true;
  mspm0_setup(&CONSOLE_DEV);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register the serial console and serial ports.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Low-level, polled output for OS debug messages.  Masks the console's
 *   interrupts while writing so the driver does not interleave output.
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct mspm0_uart_s *priv = CONSOLE_DEV.priv;
  irqstate_t flags;
  uint32_t imask;

  flags = enter_critical_section();
  imask = priv->imask;
  mspm0_setimask(priv, 0);

  arm_lowputc(ch);

  mspm0_setimask(priv, imask);
  leave_critical_section(flags);
#endif
}

#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Low-level, polled output for OS debug messages.
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  arm_lowputc(ch);
#endif
}

#endif /* USE_SERIALDRIVER */
