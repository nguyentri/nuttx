/****************************************************************************
 * arch/arm/src/ra8/ra_serial.c
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
#include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ra_sci.h"
#include "hardware/ra_mstp.h"
#include "hardware/ra_system.h"
#include "hardware/ra_mstp.h"
#include "ra_lowputc.h"
#include "ra_icu.h"
#include "ra_gpio.h"
#include "ra_clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Is there a serial console?  */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI0_UART)
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI1_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI2_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI3_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI4_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI9_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#define HAVE_CONSOLE        1
#else
#if !defined(CONFIG_NO_SERIAL_CONSOLE) && !defined(CONFIG_SERIAL_RTT_CONSOLE)
#warning "No valid CONFIG_SCIn_SERIAL_CONSOLE Setting"
#endif

#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#undef HAVE_CONSOLE
#endif

/* First pick the console and ttys0. */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart0port /* UART0 is console */
#define TTYS0_DEV       g_uart0port /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart1port /* UART1 is console */
#define TTYS0_DEV       g_uart1port /* UART1 is ttyS0 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart2port /* UART2 is console */
#define TTYS0_DEV       g_uart2port /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart3port /* UART3 is console */
#define TTYS0_DEV       g_uart3port /* UART3 is ttyS0 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart4port /* UART4 is console */
#define TTYS0_DEV       g_uart4port /* UART4 is ttyS0 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
#define CONSOLE_DEV     g_uart9port /* UART9 is console */
#define TTYS0_DEV       g_uart9port /* UART9 is ttyS0 */
#define UART9_ASSIGNED  1
#else
#undef CONSOLE_DEV                  /* No console */
#if defined(CONFIG_RA_SCI0_UART)
#define TTYS0_DEV       g_uart0port /* UART0 is ttyS0 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART)
#define TTYS0_DEV       g_uart1port /* UART1 is ttyS0 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART)
#define TTYS0_DEV       g_uart2port /* UART2 is ttyS0 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART)
#define TTYS0_DEV       g_uart3port /* UART3 is ttyS0 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART)
#define TTYS0_DEV       g_uart4port /* UART4 is ttyS0 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART)
#define TTYS0_DEV       g_uart9port /* UART9 is ttyS0 */
#define UART9_ASSIGNED  1
#endif
#endif

/* Pick ttys1. */

#if defined(CONFIG_RA_SCI0_UART) && !defined(UART0_ASSIGNED)
#define TTYS1_DEV       g_uart0port /* UART0 is ttyS1 */
#define UART0_ASSIGNED  1
#elif defined(CONFIG_RA_SCI1_UART) && !defined(UART1_ASSIGNED)
#define TTYS1_DEV       g_uart1port /* UART1 is ttyS1 */
#define UART1_ASSIGNED  1
#elif defined(CONFIG_RA_SCI2_UART) && !defined(UART2_ASSIGNED)
#define TTYS1_DEV       g_uart2port /* UART2 is ttyS1 */
#define UART2_ASSIGNED  1
#elif defined(CONFIG_RA_SCI3_UART) && !defined(UART3_ASSIGNED)
#define TTYS1_DEV       g_uart3port /* UART3 is ttyS1 */
#define UART3_ASSIGNED  1
#elif defined(CONFIG_RA_SCI4_UART) && !defined(UART4_ASSIGNED)
#define TTYS1_DEV       g_uart4port /* UART4 is ttyS1 */
#define UART4_ASSIGNED  1
#elif defined(CONFIG_RA_SCI9_UART) && !defined(UART9_ASSIGNED)
#define TTYS1_DEV       g_uart9port /* UART9 is ttyS1 */
#define UART9_ASSIGNED  1
#endif

/* Check if any UART is enabled */

#ifdef CONFIG_RA_SCI0_UART
#  define HAVE_UART 1
#elif defined(CONFIG_RA_SCI1_UART)
#  define HAVE_UART 1
#elif defined(CONFIG_RA_SCI2_UART)
#  define HAVE_UART 1
#elif defined(CONFIG_RA_SCI3_UART)
#  define HAVE_UART 1
#elif defined(CONFIG_RA_SCI4_UART)
#  define HAVE_UART 1
#elif defined(CONFIG_RA_SCI9_UART)
#  define HAVE_UART 1
#endif

#ifdef HAVE_UART

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int up_rxinterrupt(int irq, void *context, void *arg);
static int up_txinterrupt(int irq, void *context, void *arg);
static int up_txeinterrupt(int irq, void *context, void *arg);
static int up_erinterrupt(int irq, void *context, void *arg);
static int up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct up_dev_s
{
  const uint32_t scibase;   /* Base address of SCI registers */
  uint32_t mstp;            /* Module Stop Control Register */
  uint32_t baud;            /* Configured baud */
  uint32_t sr;              /* Saved status bits */
  int irq_rx;               /* RX IRQ number assigned by ICU */
  int irq_tx;               /* TX IRQ number assigned by ICU */
  int irq_txe;              /* TX End IRQ number assigned by ICU */
  int irq_err;              /* Error IRQ number assigned by ICU */
  int elc_rx;               /* RX event link for ICU configuration */
  int elc_tx;               /* TX event link for ICU configuration */
  int elc_txe;              /* TX End event link for ICU configuration */
  int elc_err;              /* Error event link for ICU configuration */
  int parity;               /* 0=none, 1=odd, 2=even */
  int bits;                 /* Number of bits (5-9) */
  bool stopbits2;           /* true: Configure with 2 stop bits instead of 1 */
};

static const struct uart_ops_s g_uart_ops =
{
  .setup        = up_setup,
  .shutdown     = up_shutdown,
  .attach       = up_attach,
  .detach       = up_detach,
  .ioctl        = up_ioctl,
  .receive      = up_receive,
  .rxint        = up_rxint,
  .rxavailable  = up_rxavailable,
  .send         = up_send,
  .txint        = up_txint,
  .txready      = up_txready,
  .txempty      = up_txempty,
};

/* I/O buffers */
#ifdef CONFIG_RA_SCI0_UART
static char g_uart0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI1_UART
static char g_uart1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI2_UART
static char g_uart2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI3_UART
static char g_uart3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI4_UART
static char g_uart4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI9_UART
static char g_uart9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
static char g_uart9txbuffer[CONFIG_SCI9_TXBUFSIZE];
#endif

#ifdef CONFIG_RA_SCI0_UART
static struct up_dev_s  g_uart0priv =
{
  .scibase      = R_SCI0_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI0,
  .irq_rx       = -1,               /* Will be assigned by ICU */
  .irq_tx       = -1,               /* Will be assigned by ICU */
  .irq_txe      = -1,               /* Will be assigned by ICU */
  .irq_err      = -1,               /* Will be assigned by ICU */
  .elc_rx        = RA_ELC_SCI0_RXI,
  .elc_tx        = RA_ELC_SCI0_TXI,
  .elc_txe       = RA_ELC_SCI0_TEI,
  .elc_err       = RA_ELC_SCI0_ERI,
  .baud         = CONFIG_SCI0_BAUD,
  .parity       = CONFIG_SCI0_PARITY,
  .bits         = CONFIG_SCI0_BITS,
  .stopbits2    = CONFIG_SCI0_2STOP,
};

static uart_dev_t g_uart0port =
{
  .recv     =
  {
    .size   = CONFIG_SCI0_RXBUFSIZE,
    .buffer = g_uart0rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI0_TXBUFSIZE,
    .buffer = g_uart0txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart0priv,
};
#endif

#ifdef CONFIG_RA_SCI1_UART
static struct up_dev_s  g_uart1priv =
{
  .scibase      = R_SCI1_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI1,
  .irq_rx       = -1,               /* Will be assigned by ICU */
  .irq_tx       = -1,               /* Will be assigned by ICU */
  .irq_txe      = -1,               /* Will be assigned by ICU */
  .irq_err      = -1,               /* Will be assigned by ICU */
  .elc_rx       = RA_ELC_SCI1_RXI,
  .elc_tx       = RA_ELC_SCI1_TXI,
  .elc_txe      = RA_ELC_SCI1_TEI,
  .elc_err      = RA_ELC_SCI1_ERI,
  .baud         = CONFIG_SCI1_BAUD,
  .parity       = CONFIG_SCI1_PARITY,
  .bits         = CONFIG_SCI1_BITS,
  .stopbits2    = CONFIG_SCI1_2STOP,
};

static uart_dev_t  g_uart1port =
{
  .recv     =
  {
    .size   = CONFIG_SCI1_RXBUFSIZE,
    .buffer = g_uart1rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI1_TXBUFSIZE,
    .buffer = g_uart1txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart1priv,
};
#endif

#ifdef CONFIG_RA_SCI2_UART
static struct up_dev_s  g_uart2priv =
{
  .scibase      = R_SCI2_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI2,
  .irq_rx       = -1,               /* Will be assigned by ICU */
  .irq_tx       = -1,               /* Will be assigned by ICU */
  .irq_txe      = -1,               /* Will be assigned by ICU */
  .irq_err      = -1,               /* Will be assigned by ICU */
  .elc_rx       = RA_ELC_SCI2_RXI,
  .elc_tx       = RA_ELC_SCI2_TXI,
  .elc_txe      = RA_ELC_SCI2_TEI,
  .elc_err      = RA_ELC_SCI2_ERI,
  .baud         = CONFIG_SCI2_BAUD,
  .parity       = CONFIG_SCI2_PARITY,
  .bits         = CONFIG_SCI2_BITS,
  .stopbits2    = CONFIG_SCI2_2STOP,
};

static uart_dev_t  g_uart2port =
{
  .recv     =
  {
    .size   = CONFIG_SCI2_RXBUFSIZE,
    .buffer = g_uart2rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI2_TXBUFSIZE,
    .buffer = g_uart2txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart2priv,
};
#endif

#ifdef CONFIG_RA_SCI3_UART
static struct up_dev_s  g_uart3priv =
{
  .scibase      = R_SCI3_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI3,
  .irq_rx       = -1,               /* Will be assigned by ICU */
  .irq_tx       = -1,               /* Will be assigned by ICU */
  .irq_txe      = -1,               /* Will be assigned by ICU */
  .irq_err      = -1,               /* Will be assigned by ICU */
  .elc_rx       = RA_ELC_SCI3_RXI,
  .elc_tx       = RA_ELC_SCI3_TXI,
  .elc_txe      = RA_ELC_SCI3_TEI,
  .elc_err      = RA_ELC_SCI3_ERI,
  .baud         = CONFIG_SCI3_BAUD,
  .parity       = CONFIG_SCI3_PARITY,
  .bits         = CONFIG_SCI3_BITS,
  .stopbits2    = CONFIG_SCI3_2STOP,
};

static uart_dev_t  g_uart3port =
{
  .recv     =
  {
    .size   = CONFIG_SCI3_RXBUFSIZE,
    .buffer = g_uart3rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI3_TXBUFSIZE,
    .buffer = g_uart3txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart3priv,
};
#endif

#ifdef CONFIG_RA_SCI4_UART
static struct up_dev_s  g_uart4priv =
{
  .scibase      = R_SCI4_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI4,
  .irq_rx       = -1,               /* Will be assigned by ICU */
  .irq_tx       = -1,               /* Will be assigned by ICU */
  .irq_txe      = -1,               /* Will be assigned by ICU */
  .irq_err      = -1,               /* Will be assigned by ICU */
  .elc_rx       = RA_ELC_SCI4_RXI,   /* RX event link for ICU configuration */
  .elc_tx       = RA_ELC_SCI4_TXI,
  .elc_txe      = RA_ELC_SCI4_TEI,
  .elc_err      = RA_ELC_SCI4_ERI,
  .baud         = CONFIG_SCI4_BAUD,
  .parity       = CONFIG_SCI4_PARITY,
  .bits         = CONFIG_SCI4_BITS,
  .stopbits2    = CONFIG_SCI4_2STOP,
};

static uart_dev_t  g_uart4port =
{
  .recv     =
  {
    .size   = CONFIG_SCI4_RXBUFSIZE,
    .buffer = g_uart4rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI4_TXBUFSIZE,
    .buffer = g_uart4txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart4priv,
};
#endif

#ifdef CONFIG_RA_SCI9_UART
static struct up_dev_s  g_uart9priv =
{
  .scibase      = R_SCI9_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI9,
  .irq_rx       = -1,               /* Will be assigned by ICU */
  .irq_tx       = -1,               /* Will be assigned by ICU */
  .irq_txe      = -1,               /* Will be assigned by ICU */
  .irq_err      = -1,               /* Will be assigned by ICU */
  .elc_rx       = RA_ELC_SCI9_RXI,
  .elc_tx       = RA_ELC_SCI9_TXI,
  .elc_txe      = RA_ELC_SCI9_TEI,
  .elc_err      = RA_ELC_SCI9_ERI,
  .baud         = CONFIG_SCI9_BAUD,
  .parity       = CONFIG_SCI9_PARITY,
  .bits         = CONFIG_SCI9_BITS,
  .stopbits2    = CONFIG_SCI9_2STOP,
};

static uart_dev_t  g_uart9port =
{
  .recv     =
  {
    .size   = CONFIG_SCI9_RXBUFSIZE,
    .buffer = g_uart9rxbuffer,
  },
  .xmit  =
  {
    .size   = CONFIG_SCI9_TXBUFSIZE,
    .buffer = g_uart9txbuffer,
  },
  .ops   = &g_uart_ops,
  .priv = &g_uart9priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_calculate_baud_setting
 *
 * Description:
 *   Calculate baud rate register settings for SCI_B UART.
 *   Based on Renesas FSP R_SCI_B_UART_BaudCalculate algorithm.
 *
 * Input Parameters:
 *   baudrate - Desired baud rate (bps)
 *   p_baud_setting - Output structure for baud rate settings
 *
 * Returned Value:
 *   0 on success, negative value on error
 *
 ****************************************************************************/

/* Baud rate divisor information (UART mode) - from Renesas FSP */
static const struct
{
  uint8_t bgdm : 1;    /* Baud rate generator double-speed mode */
  uint8_t abcs : 1;    /* Asynchronous mode base clock select */
  uint8_t abcse : 1;   /* Asynchronous mode extended base clock select */
  uint8_t cks : 2;     /* Clock select (n value) */
} g_async_baud[13] =
{
  {0, 0, 1, 0},  /* divisor: 6 */
  {1, 1, 0, 0},  /* divisor: 8 */
  {1, 0, 0, 0},  /* divisor: 16 */
  {0, 0, 1, 1},  /* divisor: 24 */
  {0, 0, 0, 0},  /* divisor: 32 */
  {1, 0, 0, 1},  /* divisor: 64 */
  {0, 0, 1, 2},  /* divisor: 96 */
  {0, 0, 0, 1},  /* divisor: 128 */
  {1, 0, 0, 2},  /* divisor: 256 */
  {0, 0, 1, 3},  /* divisor: 384 */
  {0, 0, 0, 2},  /* divisor: 512 */
  {1, 0, 0, 3},  /* divisor: 1024 */
  {0, 0, 0, 3}   /* divisor: 2048 */
};

static const uint16_t g_div_coefficient[13] =
{
  6, 8, 16, 24, 32, 64, 96, 128, 256, 384, 512, 1024, 2048
};

/* Common baud rate lookup table for different SCICLK frequencies */
/* Based on RA8E1 Hardware Manual Table 30.11 and 30.12 examples */
struct common_baudrate_settings_s
{
  uint32_t baud;
  uint8_t bgdm;
  uint8_t abcs;
  uint8_t abcse;
  uint8_t abcse2;
  uint8_t cks;
  uint8_t brr;
  uint16_t mddr;  /* Changed to uint16_t to accommodate 256 */
};

struct clock_baud_table_s
{
  uint32_t clock_freq;
  const struct common_baudrate_settings_s *settings;
  uint32_t num_settings;
};

/* Baud rate settings for 120MHz SCICLK */
static const struct common_baudrate_settings_s g_baud_120mhz[] =
{
  /* baud,   bgdm, abcs, abcse, abcse2, cks, brr, mddr */
  {   9600,    1,    0,    0,     0,     1,  195,  128 },  /* BGDM=1, ABCS=0, CKS=1, BRR=195, MDDR=128 */
  {  19200,    1,    0,    0,     0,     1,   97,  128 },  /* BGDM=1, ABCS=0, CKS=1, BRR=97, MDDR=128 */
  {  38400,    1,    0,    0,     0,     0,   97,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=97, MDDR=128 */
  {  57600,    1,    0,    0,     0,     0,   64,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=64, MDDR=128 */
  { 115200,    1,    0,    0,     0,     0,   32,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=32, MDDR=128 */
  { 230400,    1,    0,    0,     0,     0,   15,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=15, MDDR=128 */
  { 460800,    1,    0,    0,     0,     0,    7,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=7, MDDR=128 */
  { 921600,    1,    0,    0,     0,     0,    3,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=3, MDDR=128 */
  {1843200,    1,    0,    0,     0,     0,    1,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=1, MDDR=128 */
};

/* Baud rate settings for 90MHz SCICLK (360MHz PLL1P / 4) */
static const struct common_baudrate_settings_s g_baud_90mhz[] =
{
  /* baud,   bgdm, abcs, abcse, abcse2, cks, brr, mddr */
  {   9600,    1,    0,    0,     0,     1,  146,  128 },  /* BGDM=1, ABCS=0, CKS=1, BRR=146, MDDR=128 */
  {  19200,    1,    0,    0,     0,     1,   73,  128 },  /* BGDM=1, ABCS=0, CKS=1, BRR=73, MDDR=128 */
  {  38400,    1,    0,    0,     0,     0,   73,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=73, MDDR=128 */
  {  57600,    1,    0,    0,     0,     0,   48,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=48, MDDR=128 */
  { 115200,    1,    0,    0,     0,     0,   47,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=47, MDDR=128 */
  { 230400,    1,    0,    0,     0,     0,   23,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=23, MDDR=128 */
  { 460800,    1,    0,    0,     0,     0,   11,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=11, MDDR=128 */
  { 921600,    1,    0,    0,     0,     0,    5,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=5, MDDR=128 */
  {1843200,    1,    0,    0,     0,     0,    2,  128 },  /* BGDM=1, ABCS=0, CKS=0, BRR=2, MDDR=128 */
};

/* Baud rate settings for 80MHz SCICLK */
static const struct common_baudrate_settings_s g_baud_80mhz[] =
{
  /* baud,   bgdm, abcs, abcse, abcse2, cks, brr, mddr */
  {   9600,    0,    0,    0,     0,     2,  129,  256 },  /* n=2, N=129 */
  {  19200,    0,    0,    0,     0,     2,   64,  256 },  /* n=2, N=64  */
  {  38400,    0,    0,    0,     0,     1,   64,  256 },  /* n=1, N=64  */
  {  57600,    0,    0,    0,     0,     1,   42,  256 },  /* n=1, N=42  */
  { 115200,    0,    0,    0,     0,     1,   21,  256 },  /* n=1, N=21  */
  { 230400,    0,    0,    0,     0,     0,   42,  256 },  /* n=0, N=42  */
  { 460800,    0,    0,    0,     0,     0,   21,  256 },  /* n=0, N=21  */
  { 921600,    0,    0,    0,     0,     0,   10,  256 },  /* n=0, N=10  */
  {1250000,    1,    0,    0,     0,     0,   10,  256 },  /* n=0, N=10, BGDM=1 */
};

/* Baud rate settings for 60MHz SCICLK */
static const struct common_baudrate_settings_s g_baud_60mhz[] =
{
  /* baud,   bgdm, abcs, abcse, abcse2, cks, brr, mddr */
  {   9600,    0,    0,    0,     0,     2,   97,  256 },  /* n=2, N=97  */
  {  19200,    0,    0,    0,     0,     2,   48,  256 },  /* n=2, N=48  */
  {  38400,    0,    0,    0,     0,     1,   48,  256 },  /* n=1, N=48  */
  {  57600,    0,    0,    0,     0,     1,   32,  256 },  /* n=1, N=32  */
  { 115200,    0,    0,    0,     0,     1,   16,  256 },  /* n=1, N=16  */
  { 230400,    0,    0,    0,     0,     0,   32,  256 },  /* n=0, N=32  */
  { 460800,    0,    0,    0,     0,     0,   16,  256 },  /* n=0, N=16  */
  { 921600,    0,    0,    0,     0,     0,    8,  256 },  /* n=0, N=8   */
  {1875000,    1,    0,    0,     0,     0,    8,  256 },  /* n=0, N=8, BGDM=1 */
};

/* Clock-specific baud rate table */
static const struct clock_baud_table_s g_common_baud_settings[] =
{
  { 120000000, g_baud_120mhz, sizeof(g_baud_120mhz) / sizeof(g_baud_120mhz[0]) },
  {  90000000, g_baud_90mhz,  sizeof(g_baud_90mhz) / sizeof(g_baud_90mhz[0]) },
  {  80000000, g_baud_80mhz,  sizeof(g_baud_80mhz) / sizeof(g_baud_80mhz[0]) },
  {  60000000, g_baud_60mhz,  sizeof(g_baud_60mhz) / sizeof(g_baud_60mhz[0]) },
};

#define NUM_CLOCK_BAUD_TABLES (sizeof(g_common_baud_settings) / sizeof(g_common_baud_settings[0]))

struct baud_setting
{
  uint8_t bgdm;
  uint8_t abcs;
  uint8_t abcse;
  uint8_t abcse2;
  uint8_t cks;
  uint8_t brr;
  uint8_t brme;
  uint16_t mddr;  /* Changed to uint16_t to accommodate 256 */
};

static int up_calculate_baud_setting(uint32_t baudrate, struct baud_setting *p_baud_setting)
{
  ra_clock_config_t clock_config;
  uint32_t freq_hz;
  int32_t hit_bit_err = 100000; /* 100% error as starting point */
  uint32_t divisor;

  /* Get SCI clock frequency */
  ra_get_clock_config(&clock_config);
  freq_hz = clock_config.sciclk_freq;

  if (baudrate == 0 || freq_hz == 0)
    {
      return -EINVAL;
    }

  /* First check common baud rate lookup table for exact matches */
  for (uint32_t i = 0; i < NUM_CLOCK_BAUD_TABLES; i++)
    {
      if (g_common_baud_settings[i].clock_freq == freq_hz)
        {
          /* Found matching clock frequency table */
          const struct common_baudrate_settings_s *settings = g_common_baud_settings[i].settings;
          uint32_t num_settings = g_common_baud_settings[i].num_settings;

          for (uint32_t j = 0; j < num_settings; j++)
            {
              if (settings[j].baud == baudrate)
                {
                  p_baud_setting->bgdm = settings[j].bgdm;
                  p_baud_setting->abcs = settings[j].abcs;
                  p_baud_setting->abcse = settings[j].abcse;
                  p_baud_setting->abcse2 = settings[j].abcse2;
                  p_baud_setting->cks = settings[j].cks;
                  p_baud_setting->brr = settings[j].brr;
                  p_baud_setting->brme = 0;
                  p_baud_setting->mddr = settings[j].mddr;
                  return 0;
                }
            }
          break; /* Found clock table but no matching baud rate */
        }
    }

  /* Initialize with worst case values for calculation */
  p_baud_setting->brr = 255;
  p_baud_setting->brme = 0;
  p_baud_setting->mddr = 256; /* Default value for no bit rate modulation */
  p_baud_setting->abcse2 = 0; /* Initialize ABCSE2 */

  /* Find the best BRR (bit rate register) value */
  for (uint32_t select_16_base_clk_cycles = 0;
       select_16_base_clk_cycles <= 1 && (hit_bit_err > 1500); /* 1.5% max error */
       select_16_base_clk_cycles++)
    {
      for (uint32_t i = 0; i < 13; i++)
        {
          /* Skip this calculation for divisors that don't match the clock cycle requirement */
          if (((uint8_t) select_16_base_clk_cycles) ^ (g_async_baud[i].abcs | g_async_baud[i].abcse))
            {
              continue;
            }

          divisor = (uint32_t) g_div_coefficient[i] * baudrate;
          uint32_t temp_brr = freq_hz / divisor;

          if (temp_brr <= 256) /* BRR can be 0-255 */
            {
              while (temp_brr > 0)
                {
                  temp_brr -= 1;

                  /* Calculate the bit rate error. Formula:
                   * bit rate error[%] = {(PCLK / (baud * div_coefficient * (BRR + 1)) - 1} x 100
                   */
                  int32_t err_divisor = (int32_t) (divisor * (temp_brr + 1));
                  int64_t bit_err_calc = (((int64_t) freq_hz) * 100000) / err_divisor - 100000;
                  int32_t bit_err = (int32_t) bit_err_calc;

                  uint16_t mddr = 256; /* No bit rate modulation */

                  /* Take the absolute value of the bit rate error */
                  if (bit_err < 0)
                    {
                      bit_err = -bit_err;
                    }

                  /* If this is the best error so far, save these settings */
                  if (bit_err < hit_bit_err)
                    {
                      p_baud_setting->bgdm = g_async_baud[i].bgdm;
                      p_baud_setting->abcs = g_async_baud[i].abcs;
                      p_baud_setting->abcse = g_async_baud[i].abcse;
                      p_baud_setting->abcse2 = 0; /* Assume ABCSE2=0 for calculation */
                      p_baud_setting->cks = g_async_baud[i].cks;
                      p_baud_setting->brr = (uint8_t) temp_brr;
                      p_baud_setting->mddr = mddr;
                      hit_bit_err = bit_err;
                    }

                  break; /* We don't implement bit rate modulation for simplicity */
                }
            }
        }
    }

  /* Return error if the percent error is too large (>1.5%) */
  if (hit_bit_err > 1500)
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: up_serialin
 ****************************************************************************/

static inline uint32_t up_serialin(struct up_dev_s *priv, int offset)
{
  /* RA8E1 uses SCI_B with 32-bit registers */
  if (offset >= R_SCI_B_CCR0_OFFSET)
    {
      return getreg32(priv->scibase + offset);
    }
  else
    {
      return getreg8(priv->scibase + offset);
    }
}

/****************************************************************************
 * Name: up_serialout
 ****************************************************************************/

static inline void up_serialout(struct up_dev_s *priv, int offset,
                                   uint32_t value)
{
  /* RA8E1 uses SCI_B with 32-bit registers */
  if (offset >= R_SCI_B_CCR0_OFFSET)
    {
      putreg32(value, priv->scibase + offset);
    }
  else
    {
      putreg8((uint8_t)value, priv->scibase + offset);
    }
}

/****************************************************************************
 * Name: up_disableallints
 ****************************************************************************/

static void up_disableallints(struct up_dev_s *priv, uint32_t *ie)
{
  irqstate_t flags;

  /* The following must be atomic */

  flags = enter_critical_section();

  /* RA8E1 uses SCI_B with 32-bit registers */
  if (ie)
    {
      /* Return the current interrupt mask */
      *ie = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
    }

  /* Disable all interrupts */
  uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET) &
    ~(R_SCI_B_CCR0_TIE | R_SCI_B_CCR0_RIE | R_SCI_B_CCR0_TEIE);
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_sci_config
 *
 * Description:
 *   Configure the SCI baud, bits, parity, etc. This method is called the
 *   first time that the serial port is opened.
 *
 ****************************************************************************/

static void up_sci_config(struct up_dev_s *priv)
{
  /* RA8E1 uses SCI_B (version 2) registers */
  uint32_t regval;
  struct baud_setting baud_setting;
  int ret;

  /* Disable SCI_B first */
  regval = 0;
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

  /* Configure CCR1 for basic UART operation
   * From working XML: SPB2DT=1, SPB2IO=1 = 0x00000030
   */
  regval = R_SCI_B_CCR1_SPB2DT |     /* Serial port break data */
           R_SCI_B_CCR1_SPB2IO |     /* Serial port break I/O */
           0;                        /* No CTS flow control for basic UART */
  up_serialout(priv, R_SCI_B_CCR1_OFFSET, regval);

  /* Calculate baud rate settings dynamically */
  ret = up_calculate_baud_setting(priv->baud, &baud_setting);
  if (ret < 0)
    {
      /* Fallback to default 115200 baud if calculation fails */
      sinfo("Baud rate calculation failed for %lu, using 115200\n", (unsigned long)priv->baud);
      ret = up_calculate_baud_setting(115200, &baud_setting);
      if (ret < 0)
        {
          /* Final fallback to hardcoded values for 115200 */
          baud_setting.bgdm = 0;
          baud_setting.abcs = 0;
          baud_setting.abcse = 0;
          baud_setting.abcse2 = 0;
          baud_setting.cks = 1;
          baud_setting.brr = 24;
          baud_setting.brme = 0;
          baud_setting.mddr = 256;
        }
    }

  /* Configure CCR2 for baud rate generation using calculated values */
  regval = (baud_setting.bgdm ? R_SCI_B_CCR2_BGDM : 0) |
           (baud_setting.abcs ? R_SCI_B_CCR2_ABCS : 0) |
           (baud_setting.abcse ? R_SCI_B_CCR2_ABCSE : 0) |
           (baud_setting.abcse2 ? R_SCI_B_CCR2_ABCSE2 : 0) |
           (baud_setting.brme ? R_SCI_B_CCR2_BRME : 0) |
           ((uint32_t)baud_setting.cks << R_SCI_B_CCR2_CKS_SHIFT) |
           ((uint32_t)baud_setting.brr << R_SCI_B_CCR2_BRR_SHIFT) |
           ((uint32_t)baud_setting.mddr << R_SCI_B_CCR2_MDDR_SHIFT);

  up_serialout(priv, R_SCI_B_CCR2_OFFSET, regval);

 // _info("SCI%d: Baud %lu, CCR2=0x%08lx (BGDM=%d, ABCS=%d, ABCSE=%d, ABCSE2=%d, CKS=%d, BRR=%d, MDDR=%d)\n",
  //       priv->scibase == R_SCI0_BASE ? 0 :
  //       priv->scibase == R_SCI1_BASE ? 1 :
  //       priv->scibase == R_SCI2_BASE ? 2 :
  //       priv->scibase == R_SCI3_BASE ? 3 :
  //       priv->scibase == R_SCI4_BASE ? 4 : 9,
  //       (unsigned long)priv->baud, (unsigned long)regval,
  //       baud_setting.bgdm, baud_setting.abcs, baud_setting.abcse, baud_setting.abcse2,
  //       baud_setting.cks, baud_setting.brr, baud_setting.mddr);

  /* Configure CCR3 for character format
   * From working XML: CHR=2 (8-bit), LSBF=1, RXDESEL=1 = 0x00009200
   */
  regval = R_SCI_B_CCR3_LSBF | R_SCI_B_CCR3_RXDESEL; /* Base configuration from XML */

  if (priv->bits == 7)
    {
      regval |= (1 << R_SCI_B_CCR3_CHR_SHIFT); /* CHR=1 for 7-bit */
    }
  else
    {
      regval |= (2 << R_SCI_B_CCR3_CHR_SHIFT); /* CHR=2 for 8-bit (from XML) */
    }

  if (priv->parity == 1)  /* Odd parity */
    {
      regval |= R_SCI_B_CCR3_PE | R_SCI_B_CCR3_PM;
    }
  else if (priv->parity == 2)  /* Even parity */
    {
      regval |= R_SCI_B_CCR3_PE;
    }

  if (priv->stopbits2)
    {
      regval |= R_SCI_B_CCR3_STP;
    }

  up_serialout(priv, R_SCI_B_CCR3_OFFSET, regval);

  /* Configure CCR4 - no special features needed */
  regval = 0;
  up_serialout(priv, R_SCI_B_CCR4_OFFSET, regval);

  /* Clear any pending status flags */
  up_serialout(priv, R_SCI_B_CFCLR_OFFSET, 0xFFFFFFFF);
  up_serialout(priv, R_SCI_B_FFCLR_OFFSET, 0xFFFFFFFF);

  /* Enable transmit and receive
   * Match XML pattern: IDSEL=1, RE=1, TE=1 (interrupts controlled separately)
   */
  regval = R_SCI_B_CCR0_IDSEL | R_SCI_B_CCR0_TE | R_SCI_B_CCR0_RE;
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

  /* Wait for receiver internal state = 1 (RIST bit)
   * This is critical for SCI_B proper initialization
   */
  while ((up_serialin(priv, R_SCI_B_CESR_OFFSET) & R_SCI_B_CESR_RIST) == 0)
    {
      /* Wait for RIST bit to be set */
    }
}

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Hardware setup for non-console devices (reset first) */
  if (!dev->isconsole)
    {
      up_shutdown(dev);
    }

  /* Enable module stop control for all SCI channels */
  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 0);
  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);

  /* Read back to ensure write completed and add delay for module power-up */
  (void)getreg32(R_MSTP_MSTPCRB);

  /* Add a small delay to ensure the module is powered up */
  for (volatile int i = 0; i < 1000; i++)
    {
      /* Wait for module power-up */
    }

  /* Configure the UART hardware registers for both console and non-console devices */
  up_sci_config(priv);

  /* Setup is complete and ready for attach() to configure interrupts */
  return OK;
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the SCI.
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Disable all interrupts */
  up_disableallints(priv, NULL);

  /* Reset SCI_B control */
  up_serialout(priv, R_SCI_B_CCR0_OFFSET, 0);

  /* Stop SCI  */
  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 1);
  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);
}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the SCI to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the setup() method is called, however, the serial console may operate in
 *   a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  struct up_dev_s   *priv = (struct up_dev_s *)dev->priv;
  int               ret;

  /* Attach and enable the IRQ using the ICU API */

  ret = ra_icu_attach(priv->elc_rx, up_rxinterrupt, dev);
  if (ret < 0)
    {
      return ret;
    }
  priv->irq_rx = ret; /* Store the assigned IRQ number */

  ret = ra_icu_attach(priv->elc_tx, up_txinterrupt, dev);
  if (ret < 0)
    {
      ra_icu_detach(priv->irq_rx);
      return ret;
    }
  priv->irq_tx = ret; /* Store the assigned IRQ number */

  ret = ra_icu_attach(priv->elc_txe, up_txeinterrupt, dev);
  if (ret < 0)
    {
      ra_icu_detach(priv->irq_rx);
      ra_icu_detach(priv->irq_tx);
      return ret;
    }
  priv->irq_txe = ret; /* Store the assigned IRQ number */

  ret = ra_icu_attach(priv->elc_err, up_erinterrupt, dev);
  if (ret < 0)
    {
      ra_icu_detach(priv->irq_rx);
      ra_icu_detach(priv->irq_tx);
      ra_icu_detach(priv->irq_txe);
      return ret;
    }
  priv->irq_err = ret; /* Store the assigned IRQ number */

  return OK;
}

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  ra_icu_detach(priv->irq_rx);
  ra_icu_detach(priv->irq_tx);
  ra_icu_detach(priv->irq_txe);
  ra_icu_detach(priv->irq_err);
}

/****************************************************************************
 * Name: up_rxinterrupt
 *
 * Description:
 *   This is the common SCI RX interrupt handler.
 *
 ****************************************************************************/

static int up_rxinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;

  uart_recvchars(dev);

  return OK;
}

/****************************************************************************
 * Name: up_txinterrupt
 *
 * Description:
 *   This is the common SCI TX interrupt handler.
 *
 ****************************************************************************/

static int up_txinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;

  uart_xmitchars(dev);

  return OK;
}

/****************************************************************************
 * Name: up_txeinterrupt
 *
 * Description:
 *   This is the common SCI TEI (Transmit End) interrupt handler.
 *   This interrupt occurs when transmission is completely finished.
 *
 ****************************************************************************/

static int up_txeinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* TEI interrupt indicates transmission is completely finished.
   * This is useful for RS-485 or other half-duplex protocols.
   * For now, we just clear the interrupt by reading the status.
   */

  up_serialin(priv, R_SCI_B_CSR_OFFSET);

  return OK;
}

/****************************************************************************
 * Name: up_erinterrupt
 *
 * Description:
 *   This is the common SCI Error interrupt handler.
 *
 ****************************************************************************/

static int up_erinterrupt(int irq, void *context, void *arg)
{
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  struct up_dev_s   *priv;

  DEBUGASSERT(dev != NULL && dev->priv != NULL);
  priv = (struct up_dev_s *)dev->priv;

  /* Save for error reporting (SCI_B error bits) */
  priv->sr = up_serialin(priv, R_SCI_B_CSR_OFFSET) &
             (R_SCI_B_CSR_PER | R_SCI_B_CSR_FER | R_SCI_B_CSR_ORER);

  /* Clear error flags - this also clears the interrupt */
  up_serialout(priv, R_SCI_B_CFCLR_OFFSET,
               (R_SCI_B_CFCLR_PERC | R_SCI_B_CFCLR_FERC | R_SCI_B_CFCLR_ORERC));

  return OK;
}

/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
  int ret = -ENOTTY;

  return ret;
}

/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one
 *   character from the SCI.  Error bits associated with the
 *   receipt are provided in the return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  int ch;

  /* Return the error information in the saved status */
  *status   = priv->sr;
  priv->sr  = 0;

  /* Read the received byte from RDR_BY register */
  ch = (int)(up_serialin(priv, R_SCI_B_RDR_BY_OFFSET) & 0xff);

  /* Clear RDRF flag by writing to CFCLR register (SCI_B requirement) */
  up_serialout(priv, R_SCI_B_CFCLR_OFFSET, R_SCI_B_CFCLR_RDRFC);

  return ch;
}

/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the RX interrupt */
      uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval |= R_SCI_B_CCR0_RIE;
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
#endif
    }
  else
    {
      /* Disable the RX interrupt */
      uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval &= ~R_SCI_B_CCR0_RIE;
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive holding register is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return (up_serialin(priv, R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_RDRF) != 0;
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the SCI
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Send the character to TDR_BY register (byte access) */
  up_serialout(priv, R_SCI_B_TDR_BY_OFFSET, (uint8_t)ch);

  /* Clear TDRE flag by writing to CFCLR register */
  up_serialout(priv, R_SCI_B_CFCLR_OFFSET, R_SCI_B_CFCLR_TDREC);
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  irqstate_t flags;

  flags = enter_critical_section();
  if (enable)
    {
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
      /* Enable the TX interrupt and TEI interrupt */
      uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval |= (R_SCI_B_CCR0_TIE | R_SCI_B_CCR0_TEIE);
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */
       uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt and TEI interrupt */
      uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval &= ~(R_SCI_B_CCR0_TIE | R_SCI_B_CCR0_TEIE);
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the transmit holding register is empty (SSR.TDRE)
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return ((up_serialin(priv, R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TDRE) == R_SCI_B_CSR_TDRE);
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit holding and shift registers are empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  return (up_serialin(priv, R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TEND) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_earlyserialinit
 *
 * Description:
 *   Performs the low level SCI initialization early in debug so that the
 *   serial console will be available during boot up.  This must be called
 *   before arm_serialinit.
 *
 ****************************************************************************/

void arm_earlyserialinit(void)
{
  /* Disable all UART interrupts on all devices */

#ifdef TTYS0_DEV
  up_disableallints(TTYS0_DEV.priv, NULL);
#endif
#ifdef TTYS1_DEV
  up_disableallints(TTYS1_DEV.priv, NULL);
#endif
#ifdef TTYS2_DEV
  up_disableallints(TTYS2_DEV.priv, NULL);
#endif
#ifdef TTYS3_DEV
  up_disableallints(TTYS3_DEV.priv, NULL);
#endif
#ifdef TTYS4_DEV
  up_disableallints(TTYS4_DEV.priv, NULL);
#endif
#ifdef TTYS5_DEV
  up_disableallints(TTYS5_DEV.priv, NULL);
#endif

#ifdef HAVE_CONSOLE
  /* Configure the console device */
  CONSOLE_DEV.isconsole = true;

  /* Call up_setup() for console device to handle complete initialization */
  up_setup(&CONSOLE_DEV);
#endif

  /* Initialize hardware for all non-console UART devices */
#ifdef TTYS0_DEV
#ifndef HAVE_CONSOLE
  /* If no console, or if TTYS0 is not the console, initialize it */
  up_setup(&TTYS0_DEV);
#elif !defined(CONFIG_SCI0_SERIAL_CONSOLE) && !defined(CONFIG_SCI1_SERIAL_CONSOLE) && !defined(CONFIG_SCI2_SERIAL_CONSOLE) && !defined(CONFIG_SCI3_SERIAL_CONSOLE) && !defined(CONFIG_SCI4_SERIAL_CONSOLE) && !defined(CONFIG_SCI9_SERIAL_CONSOLE)
  /* TTYS0 is not the console, initialize it */
  up_setup(&TTYS0_DEV);
#endif
#endif

#ifdef TTYS1_DEV
#if !defined(HAVE_CONSOLE) || (&TTYS1_DEV != &CONSOLE_DEV)
  up_setup(&TTYS1_DEV);
#endif
#endif

#ifdef TTYS2_DEV
#if !defined(HAVE_CONSOLE) || (&TTYS2_DEV != &CONSOLE_DEV)
  up_setup(&TTYS2_DEV);
#endif
#endif

#ifdef TTYS3_DEV
#if !defined(HAVE_CONSOLE) || (&TTYS3_DEV != &CONSOLE_DEV)
  up_setup(&TTYS3_DEV);
#endif
#endif

#ifdef TTYS4_DEV
#if !defined(HAVE_CONSOLE) || (&TTYS4_DEV != &CONSOLE_DEV)
  up_setup(&TTYS4_DEV);
#endif
#endif

#ifdef TTYS5_DEV
#if !defined(HAVE_CONSOLE) || (&TTYS5_DEV != &CONSOLE_DEV)
  up_setup(&TTYS5_DEV);
#endif
#endif
}

/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{
  /* Register the console */
#ifdef HAVE_CONSOLE
  uart_register("/dev/console", &CONSOLE_DEV);
#endif
  /* Register all SCIs */
#ifdef TTYS0_DEV
  uart_register("/dev/ttyS0", &TTYS0_DEV);
#endif
#ifdef TTYS1_DEV
  uart_register("/dev/ttyS1", &TTYS1_DEV);
#endif
#ifdef TTYS2_DEV
  uart_register("/dev/ttyS2", &TTYS2_DEV);
#endif
#ifdef TTYS3_DEV
  uart_register("/dev/ttyS3", &TTYS3_DEV);
#endif
#ifdef TTYS4_DEV
  uart_register("/dev/ttyS4", &TTYS4_DEV);
#endif
#ifdef TTYS5_DEV
  uart_register("/dev/ttyS5", &TTYS5_DEV);
#endif
}

#endif /* HAVE_UART */
