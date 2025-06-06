/****************************************************************************
 * arch/arm/src/mcx-nxxx/nxxx_lpuart.c
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
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_SERIAL_TERMIOS
#  include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>
#include <nuttx/power/pm.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/nxxx_lpuart.h"
#include "hardware/nxxx_port.h"
#include "nxxx_lowputc.h"
#include "nxxx_serial.h"

#if defined(SERIAL_HAVE_TXDMA) || defined(SERIAL_HAVE_RXDMA)
#  include "chip.h"
#  include "nxxx_edma.h"
#  include "hardware/nxxx_dmamux.h"
#endif

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA should be 32-byte aligned. There is no dcache. */

#define DMA_ALIGNMENT       32
#define RXDMA_BUFFER_MASK   (DMA_ALIGNMENT - 1)
#define RXDMA_BUFFER_SIZE   ((CONFIG_NXXX_SERIAL_RXDMA_BUFFER_SIZE \
                                + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)
#define TXDMA_BUFFER_MASK   (DMA_ALIGNMENT - 1)
#define TXDMA_BUFFER_SIZE   ((CONFIG_NXXX_SERIAL_RXDMA_BUFFER_SIZE \
                              + RXDMA_BUFFER_MASK) & ~RXDMA_BUFFER_MASK)

/* Buffers need to be aligned and multiples of DMA_ALIGNMENT */

#if defined(CONFIG_ARM64_DCACHE_DISABLE)
#  define TXDMA_BUF_SIZE(b)  (b)
#  define TXDMA_BUF_ALIGN
#else
#  define TXDMA_BUF_SIZE(b) (((b) + TXDMA_BUFFER_MASK) & ~TXDMA_BUFFER_MASK)
#  define TXDMA_BUF_ALIGN   aligned_data(DMA_ALIGNMENT);
#endif

#if !defined(CONFIG_LPUART0_TXDMA)
#  define LPUART0_TXBUFSIZE_ADJUSTED  CONFIG_LPUART0_TXBUFSIZE
#  define LPUART0_TXBUFSIZE_ALGN
#else
#  define LPUART0_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART0_TXBUFSIZE)
#  define LPUART0_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART1_TXDMA)
#  define LPUART1_TXBUFSIZE_ADJUSTED  CONFIG_LPUART1_TXBUFSIZE
#  define LPUART1_TXBUFSIZE_ALGN
#else
#  define LPUART1_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART1_TXBUFSIZE)
#  define LPUART1_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART2_TXDMA)
#  define LPUART2_TXBUFSIZE_ADJUSTED  CONFIG_LPUART2_TXBUFSIZE
#  define LPUART2_TXBUFSIZE_ALGN
#else
#  define LPUART2_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART2_TXBUFSIZE)
#  define LPUART2_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART3_TXDMA)
#  define LPUART3_TXBUFSIZE_ADJUSTED  CONFIG_LPUART3_TXBUFSIZE
#  define LPUART3_TXBUFSIZE_ALGN
#else
#  define LPUART3_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART3_TXBUFSIZE)
#  define LPUART3_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART4_TXDMA)
#  define LPUART4_TXBUFSIZE_ADJUSTED  CONFIG_LPUART4_TXBUFSIZE
#  define LPUART4_TXBUFSIZE_ALGN
#else
#  define LPUART4_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART4_TXBUFSIZE)
#  define LPUART4_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART5_TXDMA)
#  define LPUART5_TXBUFSIZE_ADJUSTED  CONFIG_LPUART5_TXBUFSIZE
#  define LPUART5_TXBUFSIZE_ALGN
#else
#  define LPUART5_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART5_TXBUFSIZE)
#  define LPUART5_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART6_TXDMA)
#  define LPUART6_TXBUFSIZE_ADJUSTED  CONFIG_LPUART6_TXBUFSIZE
#  define LPUART6_TXBUFSIZE_ALGN
#else
#  define LPUART6_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART6_TXBUFSIZE)
#  define LPUART6_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

#if !defined(CONFIG_LPUART7_TXDMA)
#  define LPUART7_TXBUFSIZE_ADJUSTED  CONFIG_LPUART7_TXBUFSIZE
#  define LPUART7_TXBUFSIZE_ALGN
#else
#  define LPUART7_TXBUFSIZE_ADJUSTED TXDMA_BUF_SIZE(CONFIG_LPUART7_TXBUFSIZE)
#  define LPUART7_TXBUFSIZE_ALGN TXDMA_BUF_ALIGN
#endif

/* Which LPUART with be console? */

#if defined(CONFIG_LPUART0_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart0priv /* LPUART0 is console */
#  if defined(CONFIG_LPUART0_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART0_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART1_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart1priv /* LPUART1 is console */
#  if defined(CONFIG_LPUART1_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART1_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART2_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart2priv /* LPUART2 is console */
#  if defined(CONFIG_LPUART2_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART2_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART3_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart3priv /* LPUART3 is console */
#  if defined(CONFIG_LPUART3_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART3_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART4_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart4priv /* LPUART4 is console */
#  if defined(CONFIG_LPUART4_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART4_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART5_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart5priv /* LPUART5 is console */
#  if defined(CONFIG_LPUART5_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART5_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART6_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart6priv /* LPUART6 is console */
#  if defined(CONFIG_LPUART6_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART6_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#elif defined(CONFIG_LPUART7_SERIAL_CONSOLE)
#  define CONSOLE_DEV         g_lpuart7priv /* LPUART7 is console */
#  if defined(CONFIG_LPUART7_RXDMA)
#    define SERIAL_HAVE_CONSOLE_RXDMA 1
#  endif
#  if defined(CONFIG_LPUART7_TXDMA)
#    define SERIAL_HAVE_CONSOLE_TXDMA 1
#  endif
#endif

#if defined(SERIAL_HAVE_CONSOLE_RXDMA) || defined(SERIAL_HAVE_CONSOLE_TXDMA)
#  define SERIAL_HAVE_CONSOLE_DMA
#endif

#ifdef CONFIG_NXXX_LPUART0
#define TTYS0_DEV           g_lpuart0priv /* LPUART0 is ttyS0 */
#endif

#ifdef CONFIG_NXXX_LPUART1
#define TTYS1_DEV           g_lpuart1priv /* LPUART1 is ttyS1 */
#endif

#ifdef CONFIG_NXXX_LPUART2
#define TTYS2_DEV           g_lpuart2priv /* LPUART2 is ttyS2 */
#endif

#ifdef CONFIG_NXXX_LPUART3
#define TTYS3_DEV           g_lpuart3priv /* LPUART3 is ttyS3 */
#endif

#ifdef CONFIG_NXXX_LPUART4
#define TTYS4_DEV           g_lpuart4priv /* LPUART4 is ttyS4 */
#endif

#ifdef CONFIG_NXXX_LPUART5
#define TTYS5_DEV           g_lpuart5priv /* LPUART5 is ttyS5 */
#endif

#ifdef CONFIG_NXXX_LPUART6
#define TTYS6_DEV           g_lpuart6priv /* LPUART6 is ttyS6 */
#endif

#ifdef CONFIG_NXXX_LPUART7
#define TTYS7_DEV           g_lpuart7priv /* LPUART7 is ttyS7 */
#endif

/* Power management definitions */

#if defined(CONFIG_PM) && !defined(CONFIG_NXXX_PM_SERIAL_ACTIVITY)
#  define CONFIG_NXXX_PM_SERIAL_ACTIVITY 10
#endif

#if defined(CONFIG_PM)
#  define PM_IDLE_DOMAIN      0 /* Revisit */
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nxxx_uart_s
{
  struct uart_dev_s dev;     /* Generic UART device */

  const uint32_t uartbase;   /* Base address of UART registers */
  const int      uartnum;    /* LPUART number 1-8 */
  const bool     usects;     /* output flow control (CTS) available */
  const bool     userts;     /* input flow control (RTS) available */
  const bool     rs485mode;  /* We are in RS485 (RTS on TX) mode */
  const bool     inviflow;   /* Invert RTS sense */
  spinlock_t     lock;       /* Spinlock */
  uint32_t       baud;       /* Configured baud */
  uint32_t       ie;         /* Saved enabled interrupts */
  uint8_t        irq;        /* IRQ associated with this UART */
  uint8_t        parity;     /* 0=none, 1=odd, 2=even */
  uint8_t        bits;       /* Number of bits (7 or 8) */
  bool           stopbits2;  /* true: Configure with 2 stop bits vs 1 */
  bool           oflow;      /* output flow control (CTS) enabled */
  bool           iflow;      /* input flow control (RTS) enabled */

  /* TX DMA state */

#ifdef SERIAL_HAVE_TXDMA
  const unsigned int txch;   /* DMAMUX source of TX DMA request */
  DMACH_HANDLE       txdma;  /* currently-open transmit DMA stream */
#endif

  /* RX DMA state */

#ifdef SERIAL_HAVE_RXDMA
  const unsigned int rxch;          /* DMAMUX source of RX DMA request */
  DMACH_HANDLE       rxdma;         /* currently-open receive DMA stream */
  bool               rxenable;      /* DMA-based reception en/disable */
  uint32_t           rxdmanext;     /* Next byte in the DMA buffer to be read */
#ifndef CONFIG_ARM64_DCACHE_DISABLE
  uint32_t           rxdmaavail;    /* Number of bytes available without need to
                                     * to invalidate the data cache */
#endif
  char *const        rxfifo;        /* Receive DMA buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline uint32_t nxxx_serialin(struct nxxx_uart_s *priv,
                                      uint32_t offset);
static inline void nxxx_serialout(struct nxxx_uart_s *priv,
                                   uint32_t offset, uint32_t value);
static inline void nxxx_disableuartint(struct nxxx_uart_s *priv,
                                        uint32_t *ie);
static inline void nxxx_restoreuartint(struct nxxx_uart_s *priv,
                                        uint32_t ie);

static int  nxxx_setup(struct uart_dev_s *dev);
static void nxxx_shutdown(struct uart_dev_s *dev);
static int  nxxx_attach(struct uart_dev_s *dev);
static void nxxx_detach(struct uart_dev_s *dev);
static int  nxxx_interrupt(int irq, void *context, void *arg);
static int  nxxx_ioctl(struct file *filep, int cmd, unsigned long arg);
#if !defined(SERIAL_HAVE_ONLY_RXDMA)
static int  nxxx_receive(struct uart_dev_s *dev, unsigned int *status);
static void nxxx_rxint(struct uart_dev_s *dev, bool enable);
static bool nxxx_rxavailable(struct uart_dev_s *dev);
#endif
#if !defined(SERIAL_HAVE_ONLY_TXDMA)
static void nxxx_txint(struct uart_dev_s *dev, bool enable);
#endif

static void nxxx_send(struct uart_dev_s *dev, int ch);

static bool nxxx_txready(struct uart_dev_s *dev);

#ifdef SERIAL_HAVE_TXDMA
static void nxxx_dma_send(struct uart_dev_s *dev);
static void nxxx_dma_txint(struct uart_dev_s *dev, bool enable);
static void nxxx_dma_txavailable(struct uart_dev_s *dev);
static void nxxx_dma_txcallback(DMACH_HANDLE handle, void *arg, bool done,
                                   int result);
#endif

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int  nxxx_dma_setup(struct uart_dev_s *dev);
static void nxxx_dma_shutdown(struct uart_dev_s *dev);
#endif

#ifdef SERIAL_HAVE_RXDMA
static int  nxxx_dma_receive(struct uart_dev_s *dev,
                                unsigned int *status);
#ifdef CONFIG_PM
static void nxxx_dma_reenable(struct nxxx_uart_s *priv);
#endif
static void nxxx_dma_rxint(struct uart_dev_s *dev, bool enable);
static bool nxxx_dma_rxavailable(struct uart_dev_s *dev);

static void nxxx_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                   int result);
#endif

static bool nxxx_txempty(struct uart_dev_s *dev);

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int dowmin,
                         enum pm_state_e pmstate);
static int  up_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Serial driver UART operations */

#if !defined(SERIAL_HAVE_ONLY_TXDMA) && !defined(SERIAL_HAVE_ONLY_RXDMA)
static const struct uart_ops_s g_lpuart_ops =
{
  .setup          = nxxx_setup,
  .shutdown       = nxxx_shutdown,
  .attach         = nxxx_attach,
  .detach         = nxxx_detach,
  .ioctl          = nxxx_ioctl,
  .receive        = nxxx_receive,
  .rxint          = nxxx_rxint,
  .rxavailable    = nxxx_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nxxx_send,
  .txint          = nxxx_txint,
  .txready        = nxxx_txready,
  .txempty        = nxxx_txempty,
};
#endif

#if defined(SERIAL_HAVE_RXDMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_lpuart_rxtxdma_ops =
{
  .setup          = nxxx_dma_setup,
  .shutdown       = nxxx_dma_shutdown,
  .attach         = nxxx_attach,
  .detach         = nxxx_detach,
  .ioctl          = nxxx_ioctl,
  .receive        = nxxx_dma_receive,
  .rxint          = nxxx_dma_rxint,
  .rxavailable    = nxxx_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nxxx_send,
  .txint          = nxxx_dma_txint,
  .txready        = nxxx_txready,
  .txempty        = nxxx_txempty,
  .dmatxavail     = nxxx_dma_txavailable,
  .dmasend        = nxxx_dma_send,
};
#endif

#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_RXDMA)
static const struct uart_ops_s g_lpuart_rxdma_ops =
{
  .setup          = nxxx_dma_setup,
  .shutdown       = nxxx_dma_shutdown,
  .attach         = nxxx_attach,
  .detach         = nxxx_detach,
  .ioctl          = nxxx_ioctl,
  .receive        = nxxx_dma_receive,
  .rxint          = nxxx_dma_rxint,
  .rxavailable    = nxxx_dma_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = nxxx_send,
  .txint          = nxxx_txint,
  .txready        = nxxx_txready,
  .txempty        = nxxx_txempty,
};
#endif

#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_TXDMA)
static const struct uart_ops_s g_lpuart_txdma_ops =
{
    .setup          = nxxx_dma_setup,
    .shutdown       = nxxx_dma_shutdown,
    .attach         = nxxx_attach,
    .detach         = nxxx_detach,
    .ioctl          = nxxx_ioctl,
    .receive        = nxxx_receive,
    .rxint          = nxxx_rxint,
    .rxavailable    = nxxx_rxavailable,
  #ifdef CONFIG_SERIAL_IFLOWCONTROL
    .rxflowcontrol  = NULL,
  #endif
    .send           = nxxx_send,
    .txint          = nxxx_dma_txint,
    .txready        = nxxx_txready,
    .txempty        = nxxx_txempty,
    .dmatxavail     = nxxx_dma_txavailable,
    .dmasend        = nxxx_dma_send,
};
#endif

/* Avoid unused warning */
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_RXDMA)
const struct uart_ops_s *g_o0 = &g_lpuart_rxdma_ops;
#endif
#if !defined(SERIAL_HAVE_ONLY_DMA) && defined(SERIAL_HAVE_TXDMA)
const struct uart_ops_s *g_o1 = &g_lpuart_txdma_ops;
#endif

/* I/O buffers */

#ifdef CONFIG_LPUART0_RXDMA
static char g_lpuart0rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_LPUART1_RXDMA
static char g_lpuart1rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

# ifdef CONFIG_LPUART2_RXDMA
static char g_lpuart2rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_LPUART3_RXDMA
static char g_lpuart3rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_LPUART4_RXDMA
static char g_lpuart4rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_LPUART5_RXDMA
static char g_lpuart5rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_LPUART6_RXDMA
static char g_lpuart6rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_LPUART7_RXDMA
static char g_lpuart7rxfifo[RXDMA_BUFFER_SIZE]
  aligned_data(DMA_ALIGNMENT);
#endif

#ifdef CONFIG_NXXX_LPUART0
static char g_lpuart0rxbuffer[CONFIG_LPUART0_RXBUFSIZE];
static char g_lpuart0txbuffer[LPUART0_TXBUFSIZE_ADJUSTED] \
  LPUART0_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART1
static char g_lpuart1rxbuffer[CONFIG_LPUART1_RXBUFSIZE];
static char g_lpuart1txbuffer[LPUART1_TXBUFSIZE_ADJUSTED]
  LPUART1_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART2
static char g_lpuart2rxbuffer[CONFIG_LPUART2_RXBUFSIZE];
static char g_lpuart2txbuffer[LPUART2_TXBUFSIZE_ADJUSTED]
  LPUART2_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART3
static char g_lpuart3rxbuffer[CONFIG_LPUART3_RXBUFSIZE];
static char g_lpuart3txbuffer[LPUART3_TXBUFSIZE_ADJUSTED]
  LPUART3_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART4
static char g_lpuart4rxbuffer[CONFIG_LPUART4_RXBUFSIZE];
static char g_lpuart4txbuffer[LPUART4_TXBUFSIZE_ADJUSTED]
  LPUART4_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART5
static char g_lpuart5rxbuffer[CONFIG_LPUART5_RXBUFSIZE];
static char g_lpuart5txbuffer[LPUART5_TXBUFSIZE_ADJUSTED]
  LPUART5_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART6
static char g_lpuart6rxbuffer[CONFIG_LPUART6_RXBUFSIZE];
static char g_lpuart6txbuffer[LPUART6_TXBUFSIZE_ADJUSTED]
  LPUART6_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART7
static char g_lpuart7rxbuffer[CONFIG_LPUART7_RXBUFSIZE];
static char g_lpuart7txbuffer[LPUART7_TXBUFSIZE_ADJUSTED]
  LPUART7_TXBUFSIZE_ALGN;
#endif

#ifdef CONFIG_NXXX_LPUART0
static struct nxxx_uart_s g_lpuart0priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART0_RXBUFSIZE,
        .buffer = g_lpuart0rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART0_TXBUFSIZE,
        .buffer = g_lpuart0txbuffer,
      },
    #if defined(CONFIG_LPUART0_RXDMA) && defined(CONFIG_LPUART0_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
    #elif defined(CONFIG_LPUART0_RXDMA) && !defined(CONFIG_LPUART0_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
    #elif !defined(CONFIG_LPUART0_RXDMA) && defined(CONFIG_LPUART0_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
    #else
        .ops    = &g_lpuart_ops,
    #endif
    },

  .uartbase     = NXXX_LPUART0_BASE,
  .uartnum      = 0,
  .baud         = CONFIG_LPUART0_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM0,
  .parity       = CONFIG_LPUART0_PARITY,
  .bits         = CONFIG_LPUART0_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART0_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART0_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART0_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART0_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART0_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART0_TXDMA
  .txch = DMA_REQUEST_MUXLPUART0TX,
#  endif
#  ifdef CONFIG_LPUART0_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART0RX,
  .rxfifo       = g_lpuart0rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART1
static struct nxxx_uart_s g_lpuart1priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART1_RXBUFSIZE,
        .buffer = g_lpuart1rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART1_TXBUFSIZE,
        .buffer = g_lpuart1txbuffer,
      },
#  if defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART1_RXDMA) && !defined(CONFIG_LPUART1_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART1_RXDMA) && defined(CONFIG_LPUART1_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART1_BASE,
  .uartnum      = 1,
  .baud         = CONFIG_LPUART1_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM1,
  .parity       = CONFIG_LPUART1_PARITY,
  .bits         = CONFIG_LPUART1_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART1_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART1_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART1_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART1_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART1_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART1_TXDMA
  .txch = DMA_REQUEST_MUXLPUART1TX,
#  endif
#  ifdef CONFIG_LPUART1_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART1RX,
  .rxfifo       = g_lpuart1rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART2
static struct nxxx_uart_s g_lpuart2priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART2_RXBUFSIZE,
        .buffer = g_lpuart2rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART2_TXBUFSIZE,
        .buffer = g_lpuart2txbuffer,
      },
#  if defined(CONFIG_LPUART2_RXDMA) && defined(CONFIG_LPUART2_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART2_RXDMA) && !defined(CONFIG_LPUART2_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART2_RXDMA) && defined(CONFIG_LPUART2_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART2_BASE,
  .uartnum      = 2,
  .baud         = CONFIG_LPUART2_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM2,
  .parity       = CONFIG_LPUART2_PARITY,
  .bits         = CONFIG_LPUART2_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART2_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART2_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART2_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART2_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART2_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART2_TXDMA
  .txch = DMA_REQUEST_MUXLPUART2TX,
#  endif
#  ifdef CONFIG_LPUART2_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART2RX,
  .rxfifo       = g_lpuart2rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART3
static struct nxxx_uart_s g_lpuart3priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART3_RXBUFSIZE,
        .buffer = g_lpuart3rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART3_TXBUFSIZE,
        .buffer = g_lpuart3txbuffer,
      },
#  if defined(CONFIG_LPUART3_RXDMA) && defined(CONFIG_LPUART3_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART3_RXDMA) && !defined(CONFIG_LPUART3_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART3_RXDMA) && defined(CONFIG_LPUART3_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART3_BASE,
  .uartnum      = 3,
  .baud         = CONFIG_LPUART3_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM3,
  .parity       = CONFIG_LPUART3_PARITY,
  .bits         = CONFIG_LPUART3_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART3_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART3_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART3_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART3_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART3_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART3_TXDMA
  .txch = DMA_REQUEST_MUXLPUART3TX,
#  endif
#  ifdef CONFIG_LPUART3_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART3RX,
  .rxfifo       = g_lpuart3rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART4
static struct nxxx_uart_s g_lpuart4priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART4_RXBUFSIZE,
        .buffer = g_lpuart4rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART4_TXBUFSIZE,
        .buffer = g_lpuart4txbuffer,
      },
#  if defined(CONFIG_LPUART4_RXDMA) && defined(CONFIG_LPUART4_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART4_RXDMA) && !defined(CONFIG_LPUART4_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART4_RXDMA) && defined(CONFIG_LPUART4_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART4_BASE,
  .uartnum      = 4,
  .baud         = CONFIG_LPUART4_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM4,
  .parity       = CONFIG_LPUART4_PARITY,
  .bits         = CONFIG_LPUART4_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART4_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART4_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART4_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART4_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART4_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART4_TXDMA
  .txch = DMA_REQUEST_MUXLPUART4TX,
#  endif
#  ifdef CONFIG_LPUART4_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART4RX,
  .rxfifo       = g_lpuart4rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART5
static struct nxxx_uart_s g_lpuart5priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART5_RXBUFSIZE,
        .buffer = g_lpuart5rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART5_TXBUFSIZE,
        .buffer = g_lpuart5txbuffer,
      },
#  if defined(CONFIG_LPUART5_RXDMA) && defined(CONFIG_LPUART5_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART5_RXDMA) && !defined(CONFIG_LPUART5_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART5_RXDMA) && defined(CONFIG_LPUART5_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART5_BASE,
  .uartnum      = 5,
  .baud         = CONFIG_LPUART5_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM5,
  .parity       = CONFIG_LPUART5_PARITY,
  .bits         = CONFIG_LPUART5_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART5_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART5_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART5_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART5_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART5_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART5_TXDMA
  .txch = DMA_REQUEST_MUXLPUART5TX,
#  endif
#  ifdef CONFIG_LPUART5_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART5RX,
  .rxfifo       = g_lpuart5rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART6
static struct nxxx_uart_s g_lpuart6priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART6_RXBUFSIZE,
        .buffer = g_lpuart6rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART6_TXBUFSIZE,
        .buffer = g_lpuart6txbuffer,
      },
#  if defined(CONFIG_LPUART6_RXDMA) && defined(CONFIG_LPUART6_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART6_RXDMA) && !defined(CONFIG_LPUART6_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART6_RXDMA) && defined(CONFIG_LPUART6_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART6_BASE,
  .uartnum      = 6,
  .baud         = CONFIG_LPUART6_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM6,
  .parity       = CONFIG_LPUART6_PARITY,
  .bits         = CONFIG_LPUART6_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART6_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART6_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART6_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART6_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART6_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART6_TXDMA
  .txch = DMA_REQUEST_MUXLPUART6TX,
#  endif
#  ifdef CONFIG_LPUART6_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART6RX,
  .rxfifo       = g_lpuart6rxfifo,
#  endif
};
#endif

#ifdef CONFIG_NXXX_LPUART7
static struct nxxx_uart_s g_lpuart7priv =
{
  .dev =
    {
      .recv     =
      {
        .size   = CONFIG_LPUART7_RXBUFSIZE,
        .buffer = g_lpuart7rxbuffer,
      },
      .xmit     =
      {
        .size   = CONFIG_LPUART7_TXBUFSIZE,
        .buffer = g_lpuart7txbuffer,
      },
#  if defined(CONFIG_LPUART7_RXDMA) && defined(CONFIG_LPUART7_TXDMA)
        .ops    = &g_lpuart_rxtxdma_ops,
#  elif defined(CONFIG_LPUART7_RXDMA) && !defined(CONFIG_LPUART7_TXDMA)
        .ops    = &g_lpuart_rxdma_ops,
#  elif !defined(CONFIG_LPUART7_RXDMA) && defined(CONFIG_LPUART7_TXDMA)
        .ops    = &g_lpuart_txdma_ops,
#  else
        .ops    = &g_lpuart_ops,
#  endif
    },

  .uartbase     = NXXX_LPUART7_BASE,
  .uartnum      = 7,
  .baud         = CONFIG_LPUART7_BAUD,
  .irq          = NXXX_IRQ_LP_FLEXCOMM7,
  .parity       = CONFIG_LPUART7_PARITY,
  .bits         = CONFIG_LPUART7_BITS,
  .lock         = SP_UNLOCKED,
  .stopbits2    = CONFIG_LPUART7_2STOP,
#  if defined(CONFIG_SERIAL_OFLOWCONTROL) && defined(CONFIG_LPUART7_OFLOWCONTROL)
  .usects       = true,
#  endif
#  if defined(CONFIG_SERIAL_IFLOWCONTROL) && defined(CONFIG_LPUART7_IFLOWCONTROL)
  .userts       = true,
#  endif

#  if (defined(CONFIG_SERIAL_RS485CONTROL) || defined(CONFIG_SERIAL_IFLOWCONTROL)) && \
      defined(CONFIG_LPUART7_INVERTIFLOWCONTROL)
  .inviflow     = true,
#  endif

#  if defined(CONFIG_SERIAL_RS485CONTROL) && defined(CONFIG_LPUART7_RS485RTSCONTROL)
  .rs485mode    = true,
#  endif

#  ifdef CONFIG_LPUART7_TXDMA
  .txch = DMA_REQUEST_MUXLPUART7TX,
#  endif
#  ifdef CONFIG_LPUART7_RXDMA
  .rxch = DMA_REQUEST_MUXLPUART7RX,
  .rxfifo       = g_lpuart7rxfifo,
#  endif
};
#endif

#ifdef CONFIG_PM
static  struct pm_callback_s g_serial_pmcb =
{
  .notify       = up_pm_notify,
  .prepare      = up_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_serialin
 ****************************************************************************/

static inline uint32_t nxxx_serialin(struct nxxx_uart_s *priv,
                                      uint32_t offset)
{
  return getreg32(priv->uartbase + offset);
}

/****************************************************************************
 * Name: nxxx_serialout
 ****************************************************************************/

static inline void nxxx_serialout(struct nxxx_uart_s *priv,
                                     uint32_t offset, uint32_t value)
{
  putreg32(value, priv->uartbase + offset);
}

/****************************************************************************
 * Name: nxxx_dma_nextrx
 *
 * Description:
 *   Returns the index into the RX FIFO where the DMA will place the next
 *   byte that it receives.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int nxxx_dma_nextrx(struct nxxx_uart_s *priv)
{
  int dmaresidual = nxxx_dmach_getcount(priv->rxdma);
  DEBUGASSERT(dmaresidual <= RXDMA_BUFFER_SIZE);

  return (RXDMA_BUFFER_SIZE - dmaresidual) % RXDMA_BUFFER_SIZE;
}
#endif

/****************************************************************************
 * Name: nxxx_disableuartint
 ****************************************************************************/

static inline void nxxx_disableuartint_nolock(struct nxxx_uart_s *priv,
                                              uint32_t *ie)
{
  uint32_t regval;

  regval = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET);

  /* Return the current Rx and Tx interrupt state */

  if (ie != NULL)
    {
      *ie = regval & LPUART_ALL_INTS;
    }

  regval &= ~LPUART_ALL_INTS;
  nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, regval);
}

static inline void nxxx_disableuartint(struct nxxx_uart_s *priv,
                                       uint32_t *ie)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&priv->lock);
  nxxx_disableuartint_nolock(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: nxxx_restoreuartint
 ****************************************************************************/

static inline void nxxx_restoreuartint_nolock(struct nxxx_uart_s *priv,
                                              uint32_t ie)
{
  uint32_t regval;

  regval  = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= ie;
  nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, regval);
}

static inline void nxxx_restoreuartint(struct nxxx_uart_s *priv,
                                       uint32_t ie)
{
  irqstate_t flags;

  /* Enable/disable any interrupts that are currently disabled but should be
   * enabled/disabled.
   */

  flags = spin_lock_irqsave(&priv->lock);
  nxxx_restoreuartint_nolock(priv, ie);
  spin_unlock_irqrestore(&priv->lock, flags);
}

/****************************************************************************
 * Name: nxxx_dma_setup
 *
 * Description:
 *   Configure the LPUART baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static int nxxx_dma_setup(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
#if defined(SERIAL_HAVE_RXDMA)
  struct nxxx_edma_xfrconfig_s config;
#endif
  int result;

  /* Do the basic UART setup first, unless we are the console */

  if (!dev->isconsole)
    {
      result = nxxx_setup(dev);
      if (result != OK)
        {
          return result;
        }
    }

#if defined(SERIAL_HAVE_TXDMA)
  /* Acquire the Tx DMA channel.  This should always succeed. */

  if (priv->txch != 0)
    {
      if (priv->txdma == NULL)
        {
          priv->txdma = nxxx_dmach_alloc(priv->txch, 0);
          if (priv->txdma == NULL)
            {
              return -EBUSY;
            }
        }

      /* Enable Tx DMA for the UART */

      modifyreg32(priv->uartbase + NXXX_LPUART_BAUD_OFFSET,
                  0, LPUART_BAUD_TDMAE);
    }
#endif

#if defined(SERIAL_HAVE_RXDMA)
  /* Acquire the Rx DMA channel.  This should always succeed. */

  if (priv->rxch != 0)
    {
      if (priv->rxdma == NULL)
        {
          priv->rxdma = nxxx_dmach_alloc(priv->rxch, 0);

          if (priv->rxdma == NULL)
            {
              return -EBUSY;
            }
        }
      else
        {
          nxxx_dmach_stop(priv->rxdma);
        }

      /* Configure for circular DMA reception into the RX FIFO */

      config.saddr  = priv->uartbase + NXXX_LPUART_DATA_OFFSET;
      config.daddr  = (uintptr_t)priv->rxfifo;
      config.soff   = 0;
      config.doff   = 1;
      config.iter   = RXDMA_BUFFER_SIZE;
      config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE |
                      EDMA_CONFIG_LOOPDEST |
                      EDMA_CONFIG_INTHALF  |
                      EDMA_CONFIG_INTMAJOR;
      config.ssize  = EDMA_8BIT;
      config.dsize  = EDMA_8BIT;
      config.nbytes = 1;
#ifdef CONFIG_NXXX_EDMA_ELINK
      config.linkch = 0;
#endif

      nxxx_dmach_xfrsetup(priv->rxdma , &config);

      /* Reset our DMA shadow pointer and Rx data availability count to
       * match the address just programmed above.
       */

      priv->rxdmanext = 0;

#ifndef CONFIG_ARM64_DCACHE_DISABLE

      /* Make sure the rx buffer area is all invalid or clean */

      up_invalidate_dcache((uintptr_t)priv->rxfifo,
                           (uintptr_t)priv->rxfifo + RXDMA_BUFFER_SIZE);
      priv->rxdmaavail = 0;
#endif

      /* Enable receive Rx DMA for the UART */

      modifyreg32(priv->uartbase + NXXX_LPUART_BAUD_OFFSET,
                  0, LPUART_BAUD_RDMAE);

      /* Enable interrupt on idle and errors */

      modifyreg32(priv->uartbase + NXXX_LPUART_CTRL_OFFSET, 0,
                  LPUART_CTRL_PEIE       |
                  LPUART_CTRL_FEIE       |
                  LPUART_CTRL_NEIE       |
                  LPUART_CTRL_ILIE);

      /* Start the DMA channel, and arrange for callbacks at the half and
       * full points in the FIFO.  This ensures that we have half a FIFO
       * worth of time to claim bytes before they are overwritten.
       */

      nxxx_dmach_start(priv->rxdma, nxxx_dma_rxcallback, (void *)priv);
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: nxxx_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This
 *   method is called the first time that the serial priv is
 *   opened.
 *
 ****************************************************************************/

static int nxxx_setup(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
#ifndef CONFIG_SUPPRESS_LPUART_CONFIG
  struct uart_config_s config;
  int ret;

  /* Configure the UART */

  memset(&config, 0, sizeof(config));
  config.baud       = priv->baud;       /* Configured baud */
  config.parity     = priv->parity;     /* 0=none, 1=odd, 2=even */
  config.bits       = priv->bits;       /* Number of bits (5-9) */
  config.stopbits2  = priv->stopbits2;  /* true: Configure with 2 stop bits instead of 1 */

  config.users485   = priv->rs485mode;  /* Switch into RS485 mode */
  config.userts     = priv->iflow;
  config.invrts     = priv->inviflow;   /* Inversion of outbound flow control */
  config.usects     = priv->oflow;

  ret = nxxx_lpuart_configure(priv->uartbase, priv->uartnum, &config);

  priv->ie = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return ret;

#else
  priv->ie = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET) & \
             LPUART_ALL_INTS;
  return OK;
#endif
}

/****************************************************************************
 * Name: nxxx_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial
 *   priv is closed
 *
 ****************************************************************************/

static void nxxx_shutdown(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

  /* Disable the UART */

  nxxx_serialout(priv, NXXX_LPUART_GLOBAL_OFFSET, LPUART_GLOBAL_RST);
}

/****************************************************************************
 * Name: nxxx_dma_shutdown
 *
 * Description:
 *   Disable the LPUART.  This method is called when the serial
 *   port is closed
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) || defined(SERIAL_HAVE_TXDMA)
static void nxxx_dma_shutdown(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

  /* Perform the normal UART shutdown */

  nxxx_shutdown(dev);

#if defined(SERIAL_HAVE_RXDMA)
  /* Stop the RX DMA channel */

  if (priv->rxch != 0)
    {
      nxxx_dmach_stop(priv->rxdma);

      /* Release the RX DMA channel */

      nxxx_dmach_free(priv->rxdma);
      priv->rxdma = NULL;
    }
#endif

#if defined(SERIAL_HAVE_TXDMA)
  /* Stop the TX DMA channel */

  if (priv->txch != 0)
    {
      nxxx_dmach_stop(priv->txdma);

      /* Release the TX DMA channel */

      nxxx_dmach_free(priv->txdma);
      priv->txdma = NULL;
    }
#endif
}
#endif

/****************************************************************************
 * Name: nxxx_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial priv is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supprivs multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int nxxx_attach(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  int ret;

  /* Attach and enable the IRQ */

  ret = irq_attach(priv->irq, nxxx_interrupt, dev);
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */

      up_enable_irq(priv->irq);
    }

  return ret;
}

/****************************************************************************
 * Name: nxxx_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial priv is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void nxxx_detach(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: nxxx_interrupt (and front-ends)
 *
 * Description:
 *   This is the common UART interrupt handler.  It will be invoked when an
 *   interrupt is received on the 'irq'.  It should call uart_xmitchars or
 *   uart_recvchars to perform the appropriate data transfers.  The
 *   interrupt handling logic must be able to map the 'arg' to the
 *   appropriate uart_dev_s structure in order to call these functions.
 *
 ****************************************************************************/

static int nxxx_interrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct nxxx_uart_s *priv;
  uint32_t usr;
  uint32_t lsr;

  DEBUGASSERT(dev != NULL && dev != NULL);
  priv = (struct nxxx_uart_s *)dev;

#if defined(CONFIG_PM) && CONFIG_NXXX_PM_SERIAL_ACTIVITY > 0
  /* Repriv serial activity to the power management logic */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_NXXX_PM_SERIAL_ACTIVITY);
#endif

  /* Get the current UART status and check for loop
   * termination conditions
   */

  usr  = nxxx_serialin(priv, NXXX_LPUART_STAT_OFFSET);

  /* Removed all W1C from the last sr */

  lsr  = usr & ~(LPUART_STAT_LBKDIF | LPUART_STAT_RXEDGIF |
                 LPUART_STAT_IDLE   | LPUART_STAT_OR      |
                 LPUART_STAT_NF     | LPUART_STAT_FE      |
                 LPUART_STAT_PF     | LPUART_STAT_MA1F    |
                 LPUART_STAT_MA2F);

  /* Keep what we will service */

  usr &= (LPUART_STAT_RDRF | LPUART_STAT_TDRE | LPUART_STAT_OR |
          LPUART_STAT_FE | LPUART_STAT_NF | LPUART_STAT_PF |
          LPUART_STAT_IDLE);

  /* Clear serial overrun, parity and framing errors */

  if ((usr & LPUART_STAT_OR) != 0)
    {
      nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET,
                     LPUART_STAT_OR | lsr);
    }

  if ((usr & LPUART_STAT_NF) != 0)
    {
      nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET,
                     LPUART_STAT_NF | lsr);
    }

  if ((usr & LPUART_STAT_PF) != 0)
    {
      nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET,
                     LPUART_STAT_PF | lsr);
    }

  if ((usr & LPUART_STAT_FE) != 0)
    {
      nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET,
                     LPUART_STAT_FE | lsr);
    }

  if ((usr & (LPUART_STAT_FE | LPUART_STAT_PF | LPUART_STAT_NF)) != 0)
    {
      /* Discard data */

      nxxx_serialin(priv, NXXX_LPUART_DATA_OFFSET);
    }

#ifdef SERIAL_HAVE_RXDMA
  /* The line going to idle, deliver any fractions of RX data */

  if ((usr & LPUART_STAT_IDLE) != 0)
    {
      nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET,
                     LPUART_STAT_IDLE | lsr);
      nxxx_dma_rxcallback(priv->rxdma, priv, false, LPUART_STAT_IDLE);
    }
#endif

  /* Handle incoming, receive bytes */

  if ((priv->ie & LPUART_CTRL_RIE) != 0 && nxxx_rxavailable(&priv->dev))
    {
      uart_recvchars(dev);
    }

  /* Handle outgoing, transmit bytes */

  if ((priv->ie & LPUART_CTRL_TIE) != 0)
    {
      uart_xmitchars(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: nxxx_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int nxxx_ioctl(struct file *filep, int cmd, unsigned long arg)
{
#if defined(CONFIG_SERIAL_TIOCSERGSTRUCT)   || \
    defined(CONFIG_SERIAL_TERMIOS)          || \
    defined(CONFIG_NXXX_LPUART_SINGLEWIRE ) || \
    defined(CONFIG_NXXX_LPUART_INVERT )
  struct inode *inode = filep->f_inode;
  struct uart_dev_s *dev = inode->i_private;
  irqstate_t flags;
#endif
  int ret   = OK;

  switch (cmd)
    {
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
    case TIOCSERGSTRUCT:
      {
         struct nxxx_uart_s *user = (struct nxxx_uart_s *)arg;
         if (!user)
           {
             ret = -EINVAL;
           }
         else
           {
             memcpy(user, dev, sizeof(struct nxxx_uart_s));
           }
       }
       break;
#endif

#ifdef CONFIG_SERIAL_TERMIOS
    case TCGETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return parity */

        termiosp->c_cflag = ((priv->parity != 0) ? PARENB : 0) |
                            ((priv->parity == 1) ? PARODD : 0);

        /* Return stop bits */

        termiosp->c_cflag |= priv->stopbits2 ? CSTOPB : 0;

        /* Return flow control */

#ifdef CONFIG_SERIAL_OFLOWCONTROL
        termiosp->c_cflag |= (priv->oflow ? CCTS_OFLOW : 0);

#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
        termiosp->c_cflag |= (priv->iflow ? CRTS_IFLOW : 0);
#endif
        /* Return baud */

        cfsetispeed(termiosp, priv->baud);

        /* Return number of bits */

        switch (priv->bits)
          {
          case 5:
            termiosp->c_cflag |= CS5;
            break;

          case 6:
            termiosp->c_cflag |= CS6;
            break;

          case 7:
            termiosp->c_cflag |= CS7;
            break;

          default:
          case 8:
            termiosp->c_cflag |= CS8;
            break;

#if defined(CS9)
          case 9:
            termiosp->c_cflag |= CS9;
            break;
#endif
          }
      }
      break;

    case TCSETS:
      {
        struct termios  *termiosp = (struct termios *)arg;
        struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
        uint32_t baud;
        uint32_t ie;
        uint8_t parity;
        uint8_t nbits;
        bool stop2;

        if ((!termiosp)
#ifdef CONFIG_SERIAL_OFLOWCONTROL
            || ((termiosp->c_cflag & CCTS_OFLOW) && !priv->usects)
#endif
#ifdef CONFIG_SERIAL_IFLOWCONTROL
            || ((termiosp->c_cflag & CRTS_IFLOW) && !priv->userts)
#endif
            )
          {
            ret = -EINVAL;
            break;
          }

        /* Decode baud. */

        ret = OK;
        baud = cfgetispeed(termiosp);

        /* Decode number of bits */

        switch (termiosp->c_cflag & CSIZE)
          {
          case CS5:
            nbits = 5;
            break;

          case CS6:
            nbits = 6;
            break;

          case CS7:
            nbits = 7;
            break;

          case CS8:
            nbits = 8;
            break;

#if defined(CS9)
          case CS9:
            nbits = 9;
            break;
#endif
          default:
            ret = -EINVAL;
            break;
          }

        /* Decode parity */

        if ((termiosp->c_cflag & PARENB) != 0)
          {
            parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            parity = 0;
          }

        /* Decode stop bits */

        stop2 = (termiosp->c_cflag & CSTOPB) != 0;

        /* Verify that all settings are valid before committing */

        if (ret == OK)
          {
            /* Commit */

            priv->baud      = baud;
            priv->parity    = parity;
            priv->bits      = nbits;
            priv->stopbits2 = stop2;
            priv->oflow     = (termiosp->c_cflag & CCTS_OFLOW) != 0;
            priv->iflow     = (termiosp->c_cflag & CRTS_IFLOW) != 0;

            /* effect the changes immediately - note that we do not
             * implement TCSADRAIN / TCSAFLUSH
             */

            flags = spin_lock_irqsave(&priv->lock);
            nxxx_disableuartint_nolock(priv, &ie);
            ret = dev->ops->setup(dev);

            /* Restore the interrupt state */

            nxxx_restoreuartint_nolock(priv, ie);
            priv->ie = ie;
            spin_unlock_irqrestore(&priv->lock, flags);
          }
      }
      break;
#endif /* CONFIG_SERIAL_TERMIOS */

#ifdef CONFIG_NXXX_LPUART_SINGLEWIRE
    case TIOCSSINGLEWIRE:
      {
        uint32_t regval;
        struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

        flags  = spin_lock_irqsave(&priv->lock);
        regval = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET);

        if ((arg & SER_SINGLEWIRE_ENABLED) != 0)
          {
            regval |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
          }
        else
          {
            regval &= ~(LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC);
          }

        nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, regval);

        spin_unlock_irqrestore(&priv->lock, flags);
      }
      break;
#endif

#ifdef CONFIG_NXXX_LPUART_INVERT
    case TIOCSINVERT:
      {
        uint32_t ctrl;
        uint32_t stat;
        uint32_t regval;
        struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

        flags  = spin_lock_irqsave(&priv->lock);
        ctrl   = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET);
        stat   = nxxx_serialin(priv, NXXX_LPUART_STAT_OFFSET);
        regval = ctrl;

        /* {R|T}XINV bit field can only be written when the receiver is
        * disabled (RE=0).
        */

        regval &= ~LPUART_CTRL_RE;

        nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, regval);

        /* Enable/disable signal inversion. */

        if (arg & SER_INVERT_ENABLED_RX)
          {
            stat |= LPUART_STAT_RXINV;
          }
        else
          {
            stat &= ~LPUART_STAT_RXINV;
          }

        /* Do not invert TX when in TIOCSSINGLEWIRE */

        if ((arg & SER_INVERT_ENABLED_TX) &&
            ((ctrl & LPUART_CTRL_LOOPS) != LPUART_CTRL_LOOPS))
          {
            ctrl |= LPUART_CTRL_TXINV;
          }
        else
          {
            ctrl &= ~LPUART_CTRL_TXINV;
          }

        nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET, stat);
        nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, ctrl);

        spin_unlock_irqrestore(&priv->lock, flags);
      }
      break;
#endif

    case TIOCSBRK:  /* BSD compatibility: Turn break on, unconditionally */
    case TIOCCBRK:  /* BSD compatibility: Turn break off, unconditionally */
    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: nxxx_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the UART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static int nxxx_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  uint32_t rxd;

  rxd     = nxxx_serialin(priv, NXXX_LPUART_DATA_OFFSET);
  *status = rxd >> LPUART_DATA_STATUS_SHIFT;
  return (rxd & LPUART_DATA_MASK) >> LPUART_DATA_SHIFT;
}
#endif

/****************************************************************************
 * Name: nxxx_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static void nxxx_rxint(struct uart_dev_s *dev, bool enable)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupts for data available at Rx */

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE;
#endif
    }
  else
    {
      priv->ie &= ~(LPUART_CTRL_RIE | LPUART_CTRL_FEIE | LPUART_CTRL_ORIE);
    }

  regval  = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, regval);
  spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

/****************************************************************************
 * Name: nxxx_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

#ifndef SERIAL_HAVE_ONLY_RXDMA
static bool nxxx_rxavailable(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  uint32_t regval;

  /* Return true is data is ready in the Rx FIFO */

  regval = nxxx_serialin(priv, NXXX_LPUART_WATER_OFFSET);
  return ((regval & LPUART_WATER_RXCOUNT_MASK) != 0);
}
#endif

/****************************************************************************
 * Name: nxxx_dma_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the LPUART.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static int nxxx_dma_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  uint32_t nextrx             = nxxx_dma_nextrx(priv);
  int c                       = 0;

  /* Check if more data is available */

  if (nextrx != priv->rxdmanext)
    {
#ifndef CONFIG_ARM64_DCACHE_DISABLE
      /* If the data cache is enabled, then we will also need to manage
       * cache coherency.  Are any bytes available in the currently coherent
       * region of the data cache?
       */

      if (priv->rxdmaavail == 0)
        {
          uint32_t rxdmaavail;
          uintptr_t addr;

          /* No.. then we will have to invalidate additional space in the Rx
           * DMA buffer.
           */

          if (nextrx > priv->rxdmanext)
            {
              /* Number of available bytes */

              rxdmaavail = nextrx - priv->rxdmanext;
            }
          else
            {
              /* Number of available bytes up to the end of RXDMA buffer */

              rxdmaavail = RXDMA_BUFFER_SIZE - priv->rxdmanext;
            }

          /* Invalidate the DMA buffer range */

          addr = (uintptr_t)&priv->rxfifo[priv->rxdmanext];
          up_invalidate_dcache(addr, addr + rxdmaavail);

          /* We don't need to invalidate the data cache for the next
           * rxdmaavail number of next bytes.
           */

          priv->rxdmaavail = rxdmaavail;
        }

      priv->rxdmaavail--;
#endif

      /* Now read from the DMA buffer */

      c = priv->rxfifo[priv->rxdmanext];

      priv->rxdmanext++;

      if (priv->rxdmanext == RXDMA_BUFFER_SIZE)
        {
          priv->rxdmanext = 0;
        }
    }

  /* NOTE:  If no data is available, then we would return NULL which is,
   * of course, valid binary data.  The protocol is that the upper half
   * driver must call nxxx_dma_rxavailable prior to calling this
   * function to assure that this never happens.
   */

  return c;
}
#endif

/****************************************************************************
 * Name: nxxx_dma_reenable
 *
 * Description:
 *   Call to re-enable RX DMA.
 *
 ****************************************************************************/

#if defined(SERIAL_HAVE_RXDMA) && defined(CONFIG_PM)
static void nxxx_dma_reenable(struct nxxx_uart_s *priv)
{
  struct nxxx_edma_xfrconfig_s config;

  /* Stop an reset the RX DMA */

  nxxx_dmach_stop(priv->rxdma);

  /* Configure for circular DMA reception into the RX FIFO */

  config.saddr  = priv->uartbase + NXXX_LPUART_DATA_OFFSET;
  config.daddr  = (uint32_t) priv->rxfifo;
  config.soff   = 0;
  config.doff   = 1;
  config.iter   = RXDMA_BUFFER_SIZE;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE |
                  EDMA_CONFIG_LOOPDEST |
                  EDMA_CONFIG_INTHALF |
                  EDMA_CONFIG_INTMAJOR;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = 1;
#ifdef CONFIG_NXXX_EDMA_ELINK
  config.linkch = 0;
#endif

  nxxx_dmach_xfrsetup(priv->rxdma, &config);

  /* Reset our DMA shadow pointer and Rx data availability count to match
   * the address just programmed above.
   */

  priv->rxdmanext = 0;
#ifndef CONFIG_ARM64_DCACHE_DISABLE
  priv->rxdmaavail = 0;
#endif

  /* Start the DMA channel, and arrange for callbacks at the half and
   * full points in the FIFO.  This ensures that we have half a FIFO
   * worth of time to claim bytes before they are overwritten.
   */

  nxxx_dmach_start(priv->rxdma, nxxx_dma_rxcallback, (void *)priv);

  /* Clear DMA suspended flag. */

  priv->rxdmasusp  = false;
}
#endif

/****************************************************************************
 * Name: nxxx_dma_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void nxxx_dma_rxint(struct uart_dev_s *dev, bool enable)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

  /* Enable/disable DMA reception.
   *
   * Note that it is not safe to check for available bytes and immediately
   * pass them to uart_recvchars as that could potentially recurse back
   * to us again.  Instead, bytes must wait until the next up_dma_poll or
   * DMA event.
   */

  priv->rxenable = enable;
}
#endif

/****************************************************************************
 * Name: nxxx_dma_rxavailable
 *
 * Description:
 *   Return true if the receive register is not empty
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static bool nxxx_dma_rxavailable(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

  /* Compare our receive pointer to the current DMA pointer, if they
   * do not match, then there are bytes to be received.
   */

  return (nxxx_dma_nextrx(priv) != priv->rxdmanext);
}
#endif

/****************************************************************************
 * Name: nxxx_dma_txcallback
 *
 * Description:
 *   This function clears dma buffer at complete of DMA transfer and wakes up
 *   threads waiting for space in buffer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void nxxx_dma_txcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)arg;

  /* Update 'nbytes' indicating number of bytes actually transferred by DMA.
   * This is important to free TX buffer space by 'uart_xmitchars_done'.
   */

  priv->dev.dmatx.nbytes = priv->dev.dmatx.length;
#if CONFIG_NXXX_EDMA_NTCD > 1
  priv->dev.dmatx.nbytes += priv->dev.dmatx.nlength;
#endif

  /* Adjust the pointers */

  uart_xmitchars_done(&priv->dev);

  /* Send more data if available */

  nxxx_dma_txavailable(&priv->dev);
}
#endif

/****************************************************************************
 * Name: nxxx_dma_txavailable
 *
 * Description:
 *        Informs DMA that Tx data is available and is ready for transfer.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void nxxx_dma_txavailable(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

  /* Only send when the DMA is idle */

  if (nxxx_dmach_idle(priv->txdma) == 0)
    {
      uart_xmitchars_dma(dev);
    }
}
#endif

/****************************************************************************
 * Name: nxxx_dma_send
 *
 * Description:
 *   Called (usually) from the interrupt level to start DMA transfer.
 *   (Re-)Configures DMA Stream updating buffer and buffer length.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void nxxx_dma_send(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  struct nxxx_edma_xfrconfig_s config;

  /* We need to stop DMA before reconfiguration */

  nxxx_dmach_stop(priv->txdma);

  /* Reset the number sent */

  dev->dmatx.nbytes = 0;

  /* Make use of setup function to update buffer and its length for next
   * transfer
   */

  config.iter   = dev->dmatx.length;
  config.flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
  config.ssize  = EDMA_8BIT;
  config.dsize  = EDMA_8BIT;
  config.nbytes = 1;
  config.saddr  = (uintptr_t)dev->dmatx.buffer;
  config.daddr  = priv->uartbase + NXXX_LPUART_DATA_OFFSET;
  config.soff   = 1;
  config.doff   = 0;
#ifdef CONFIG_NXXX_EDMA_ELINK
  config.linkch  = 0;
#endif

  /* Flush the contents of the TX buffer into physical memory */

  up_clean_dcache((uintptr_t)dev->dmatx.buffer,
                  (uintptr_t)dev->dmatx.buffer + dev->dmatx.length);

  /* Setup first half */

  nxxx_dmach_xfrsetup(priv->txdma, &config);

#if CONFIG_NXXX_EDMA_NTCD > 1
  /* Is this a split transfer? */

  if (dev->dmatx.nbuffer)
    {
      config.iter   = priv->dev.dmatx.nlength;
      config.saddr  = (uintptr_t)priv->dev.dmatx.nbuffer;

      /* Flush the contents of the next TX buffer into physical memory */

      up_clean_dcache((uintptr_t)dev->dmatx.nbuffer,
                      (uintptr_t)dev->dmatx.nbuffer + dev->dmatx.nlength);

      nxxx_dmach_xfrsetup(priv->txdma, &config);
    }
#endif

  /* Start transmission with the callback on DMA completion */

  nxxx_dmach_start(priv->txdma, nxxx_dma_txcallback, (void *)priv);
}
#endif

/****************************************************************************
 * Name: nxxx_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void nxxx_send(struct uart_dev_s *dev, int ch)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;

#ifdef CONSOLE_DEV
  if (dev == &CONSOLE_DEV.dev && !dev->isconsole)
    {
      return;
    }
#endif

  nxxx_serialout(priv, NXXX_LPUART_DATA_OFFSET, (uint32_t)ch);
}

/****************************************************************************
 * Name: nxxx_dma_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts from the UART.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_TXDMA
static void nxxx_dma_txint(struct uart_dev_s *dev, bool enable)
{
  /* Nothing to do. */

  /* In case of DMA transfer we do not want to make use of UART interrupts.
   * Instead, we use DMA interrupts that are activated once during boot
   * sequence. Furthermore we can use nxxx_dma_txcallback() to handle
   * stuff at half DMA transfer or after transfer completion (depending
   * on the configuration).
   */
}
#endif

/****************************************************************************
 * Name: nxxx_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

#if !defined(SERIAL_HAVE_ONLY_TXDMA)
static void nxxx_txint(struct uart_dev_s *dev, bool enable)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  irqstate_t flags;
  uint32_t regval;

  /* Enable interrupt for TX complete */

  flags = spin_lock_irqsave(&priv->lock);
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      priv->ie |= LPUART_CTRL_TIE;
#endif
    }
  else
    {
      priv->ie &= ~LPUART_CTRL_TIE;
    }

  regval  = nxxx_serialin(priv, NXXX_LPUART_CTRL_OFFSET);
  regval &= ~LPUART_ALL_INTS;
  regval |= priv->ie;
  nxxx_serialout(priv, NXXX_LPUART_CTRL_OFFSET, regval);

#ifndef CONFIG_SUPPRESS_SERIAL_INTS
  if (enable)
    {
      uart_xmitchars(dev);
    }
#endif

  spin_unlock_irqrestore(&priv->lock, flags);
}
#endif

/****************************************************************************
 * Name: nxxx_txready
 *
 * Description:
 *   Return true if the transmit fifo is available to be written to
 *
 ****************************************************************************/

static bool nxxx_txready(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  uint32_t regval;
  uint32_t fifo_size;
  uint32_t fifo_count;

  /* Read the fifo size and current fill ratio. Return true if fifo is not
   * full
   */

  regval = nxxx_serialin(priv, NXXX_LPUART_FIFO_OFFSET);
  fifo_size = (regval & LPUART_FIFO_TXFIFOSIZE_MASK) >>
    LPUART_FIFO_TXFIFOSIZE_SHIFT;
  fifo_size = fifo_size == 0 ? 1 : (1 << (fifo_size + 1));
  regval = nxxx_serialin(priv, NXXX_LPUART_WATER_OFFSET);
  fifo_count = (regval & LPUART_WATER_TXCOUNT_MASK) >>
    LPUART_WATER_TXCOUNT_SHIFT;

  return fifo_count < fifo_size;
}

/****************************************************************************
 * Name: nxxx_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool nxxx_txempty(struct uart_dev_s *dev)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)dev;
  uint32_t regval;

  regval = nxxx_serialin(priv, NXXX_LPUART_WATER_OFFSET);
  return (regval & LPUART_WATER_TXCOUNT_MASK) == 0;
}

/****************************************************************************
 * Name: nxxx_dma_rxcallback
 *
 * Description:
 *   This function checks the current DMA state and calls the generic
 *   serial stack when bytes appear to be available.
 *
 ****************************************************************************/

#ifdef SERIAL_HAVE_RXDMA
static void nxxx_dma_rxcallback(DMACH_HANDLE handle, void *arg, bool done,
                                  int result)
{
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)arg;
  uint32_t sr;

  if (priv->rxenable && nxxx_dma_rxavailable(&priv->dev))
    {
      uart_recvchars(&priv->dev);
    }

  /* Get the masked LPUART status word to check and clear error flags.
   *
   * When wake-up from low power mode was not fast enough, UART is resumed
   * too late and sometimes exactly when character was coming over UART,
   * resulting to frame error.
   * If error flag is not cleared, Rx DMA will be stuck. Clearing errors
   * will release Rx DMA.
   */

  sr = nxxx_serialin(priv, NXXX_LPUART_STAT_OFFSET);

  if ((sr & (LPUART_STAT_OR | LPUART_STAT_NF | LPUART_STAT_FE)) != 0)
    {
      nxxx_serialout(priv, NXXX_LPUART_STAT_OFFSET,
                      sr & (LPUART_STAT_OR |
                            LPUART_STAT_NF |
                            LPUART_STAT_FE));
    }
}
#endif

/****************************************************************************
 * Name: up_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is  called after
 *   all drivers have had the opprivunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_pm_notify(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case (PM_NORMAL):
        {
          /* Logic for PM_NORMAL goes here */
        }
        break;

      case (PM_IDLE):
        {
          /* Logic for PM_IDLE goes here */
        }
        break;

      case (PM_STANDBY):
        {
          /* Logic for PM_STANDBY goes here */
        }
        break;

      case (PM_SLEEP):
        {
          /* Logic for PM_SLEEP goes here */
        }
        break;

      default:

        /* Should not get here */

        break;
    }
}
#endif

/****************************************************************************
 * Name: up_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         structure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int up_pm_prepare(struct pm_callback_s *cb, int domain,
                         enum pm_state_e pmstate)
{
  /* Logic to prepare for a reduced power state goes here. */

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level UART initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* NOTE: This function assumes that low level hardware configuration
   * -- including all clocking and pin configuration -- was performed by the
   * function nxxx_lowsetup() earlier in the boot sequence.
   */

  /* Enable the console UART.  The other UARTs will be initialized if and
   * when they are first opened.
   */

#ifdef CONSOLE_DEV
  CONSOLE_DEV.dev.isconsole = true;
  nxxx_setup(&CONSOLE_DEV.dev);
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial privs.  This assumes
 *   that nxxx_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
#ifdef CONFIG_PM
  int ret;

  /* Register to receive power management callbacks */

  ret = pm_register(&g_serial_pmcb);
  DEBUGASSERT(ret == OK);
  UNUSED(ret);
#endif

#ifdef CONSOLE_DEV
  uart_register("/dev/console", &CONSOLE_DEV.dev);
#if defined(SERIAL_HAVE_CONSOLE_DMA)
  nxxx_dma_setup(&CONSOLE_DEV.dev);
#endif
#endif

  /* Register all UARTs */

#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV.dev);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV.dev);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV.dev);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV.dev);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV.dev);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV.dev);
#endif
#ifdef TTYS6_DEV
  uart_register("/dev/ttyS6", &TTYS6_DEV.dev);
#endif
#ifdef TTYS7_DEV
  uart_register("/dev/ttyS7", &TTYS7_DEV.dev);
#endif
}

#endif /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to suppriv OS debug  writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef CONSOLE_DEV
  struct nxxx_uart_s *priv = (struct nxxx_uart_s *)&CONSOLE_DEV;
  uint32_t ie;

  if (!CONSOLE_DEV.dev.isconsole)
    {
      return;
    }

  nxxx_disableuartint(priv, &ie);
#endif

  arm_lowputc(ch);
#ifdef CONSOLE_DEV
  nxxx_restoreuartint(priv, ie);
#endif
}
