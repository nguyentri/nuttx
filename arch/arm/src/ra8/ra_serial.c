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

#define SCI_UART_ERR_BITS  (R_SCI_SSR_PER | R_SCI_SSR_FER | R_SCI_SSR_ORER)

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
  uint32_t rxel;            /* RX event link associated with this SCI */
  uint32_t txel;            /* TX event link associated with this SCI */
  uint32_t txeel;           /* TX End event link associated with this SCI */
  uint32_t erel;            /* Error event link associated with this SCI */
  uint32_t parity;          /* 0=none, 1=odd, 2=even */
  uint32_t bits;            /* Number of bits (5-9) */
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
#if defined(CONFIG_RA_SCI0_UART)
static char g_uart0rxbuffer[CONFIG_SCI0_RXBUFSIZE];
static char g_uart0txbuffer[CONFIG_SCI0_TXBUFSIZE];
#elif defined(CONFIG_RA_SCI1_UART)
static char g_uart1rxbuffer[CONFIG_SCI1_RXBUFSIZE];
static char g_uart1txbuffer[CONFIG_SCI1_TXBUFSIZE];
#elif defined(CONFIG_RA_SCI2_UART)
static char g_uart2rxbuffer[CONFIG_SCI2_RXBUFSIZE];
static char g_uart2txbuffer[CONFIG_SCI2_TXBUFSIZE];
#elif defined(CONFIG_RA_SCI3_UART)
static char g_uart3rxbuffer[CONFIG_SCI3_RXBUFSIZE];
static char g_uart3txbuffer[CONFIG_SCI3_TXBUFSIZE];
#elif defined(CONFIG_RA_SCI4_UART)
static char g_uart4rxbuffer[CONFIG_SCI4_RXBUFSIZE];
static char g_uart4txbuffer[CONFIG_SCI4_TXBUFSIZE];
#elif defined(CONFIG_RA_SCI9_UART)
static char g_uart9rxbuffer[CONFIG_SCI9_RXBUFSIZE];
static char g_uart9txbuffer[CONFIG_SCI9_TXBUFSIZE];
#endif

#if defined(CONFIG_RA_SCI0_UART)
static struct up_dev_s  g_uart0priv =
{
  .scibase      = R_SCI0_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI0,
  .rxel         = RA_EL_SCI0_RXI,
  .txel         = RA_EL_SCI0_TXI,
  .txeel        = RA_EL_SCI0_TEI,
  .erel         = RA_EL_SCI0_ERI,
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

#elif defined(CONFIG_RA_SCI1_UART)
static struct up_dev_s  g_uart1priv =
{
  .scibase      = R_SCI1_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI1,
  .rxel         = RA_EL_SCI1_RXI,
  .txel         = RA_EL_SCI1_TXI,
  .txeel        = RA_EL_SCI1_TEI,
  .erel         = RA_EL_SCI1_ERI,
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

#elif defined(CONFIG_RA_SCI2_UART)
static struct up_dev_s  g_uart2priv =
{
  .scibase      = R_SCI2_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI2,
  .rxel         = RA_EL_SCI2_RXI,
  .txel         = RA_EL_SCI2_TXI,
  .txeel        = RA_EL_SCI2_TEI,
  .erel         = RA_EL_SCI2_ERI,
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

#elif defined(CONFIG_RA_SCI3_UART)
static struct up_dev_s  g_uart3priv =
{
  .scibase      = R_SCI3_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI3,
  .rxel         = RA_EL_SCI3_RXI,
  .txel         = RA_EL_SCI3_TXI,
  .txeel        = RA_EL_SCI3_TEI,
  .erel         = RA_EL_SCI3_ERI,
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

#elif defined(CONFIG_RA_SCI4_UART)
static struct up_dev_s  g_uart4priv =
{
  .scibase      = R_SCI4_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI4,
  .rxel         = RA_EL_SCI4_RXI,
  .txel         = RA_EL_SCI4_TXI,
  .txeel        = RA_EL_SCI4_TEI,
  .erel         = RA_EL_SCI4_ERI,
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

#elif defined(CONFIG_RA_SCI9_UART)
static struct up_dev_s  g_uart9priv =
{
  .scibase      = R_SCI9_BASE,
  .mstp         = R_MSTP_MSTPCRB_SCI9,
  .rxel         = RA_EL_SCI9_RXI,
  .txel         = RA_EL_SCI9_TXI,
  .txeel        = RA_EL_SCI9_TEI,
  .erel         = RA_EL_SCI9_ERI,
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
  .ops   = &g_uart_ops, .priv = &g_uart9priv,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  /* Configure CCR2 for baud rate generation
   * From working XML: MDDR=128 (0x80), BRR=47, BGDM=1, CKS=0 = 0x80002F10
   */
  regval = R_SCI_B_CCR2_BGDM |       /* Baud rate generator double-speed mode */
           (0 << R_SCI_B_CCR2_CKS_SHIFT) |  /* CKS = 0 (PCLKA) */
           (47 << R_SCI_B_CCR2_BRR_SHIFT) |  /* BRR = 47 */
           (128UL << R_SCI_B_CCR2_MDDR_SHIFT); /* MDDR = 128 (from XML) */
  up_serialout(priv, R_SCI_B_CCR2_OFFSET, regval);

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
}

static int up_setup(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  /* Configure GPIO pins for non-console UARTs only
   * Console UART pins are already configured in ra_lowsetup()
   */
#if defined(CONFIG_RA_SCI0_UART) && !defined(CONFIG_SCI0_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI0_RX);
  ra_configgpio(GPIO_SCI0_TX);
#elif defined(CONFIG_RA_SCI1_UART) && !defined(CONFIG_SCI1_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI1_RX);
  ra_configgpio(GPIO_SCI1_TX);
#elif defined(CONFIG_RA_SCI2_UART) && !defined(CONFIG_SCI2_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI2_RX);
  ra_configgpio(GPIO_SCI2_TX);
#elif defined(CONFIG_RA_SCI3_UART) && !defined(CONFIG_SCI3_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI3_RX);
  ra_configgpio(GPIO_SCI3_TX);
#elif defined(CONFIG_RA_SCI4_UART) && !defined(CONFIG_SCI4_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI4_RX);
  ra_configgpio(GPIO_SCI4_TX);
#elif defined(CONFIG_RA_SCI9_UART) && !defined(CONFIG_SCI9_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI9_RX);
  ra_configgpio(GPIO_SCI9_TX);
#endif

  /* Full initialization was already done in arm_earlyserialinit()
   * For non-console UARTs, we need to enable the module and configure registers
   */

  /* Skip console initialization as it's done in arm_earlyserialinit() */
  if (dev->isconsole)
    {
      /* Console is already fully configured - just return OK */
      return OK;
    }

  up_shutdown(dev);

  /* Enable module stop control for non-console UARTs */
  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);
  modifyreg32(R_MSTP_MSTPCRB, priv->mstp, 0);
  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);

  /* Configure the UART */
  up_sci_config(priv);

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

  /* Attach and enable the IRQ using the new unified ICU API */

  ret = ra_icu_attach(priv->rxel, up_rxinterrupt, dev);
  if (ret < 0)
    {
      return ret;
    }
  priv->rxel = ret; /* Store the assigned IRQ number */

  ret = ra_icu_attach(priv->txel, up_txinterrupt, dev);
  if (ret < 0)
    {
      ra_icu_detach(priv->rxel);
      return ret;
    }
  priv->txel = ret; /* Store the assigned IRQ number */

  ret = ra_icu_attach(priv->erel, up_erinterrupt, dev);
  if (ret < 0)
    {
      ra_icu_detach(priv->rxel);
      ra_icu_detach(priv->txel);
      return ret;
    }
  priv->erel = ret; /* Store the assigned IRQ number */

  return OK;
}

static void up_detach(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;

  ra_icu_detach(priv->rxel);
  ra_icu_detach(priv->txel);
  ra_icu_detach(priv->erel);
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

  /* SCI RX interrupt is automatically cleared by reading the data */
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

  /* SCI TX interrupt is automatically cleared by writing data */
  uart_xmitchars(dev);

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

  /* Return the error information in the saved status */
  *status   = priv->sr;
  priv->sr  = 0;

  /* Then return the actual received byte */
  return (int)(up_serialin(priv, R_SCI_B_RDR_BY_OFFSET) & 0xff);
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

  up_serialout(priv, R_SCI_B_TDR_BY_OFFSET, (uint8_t)ch);
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
      /* Enable the TX interrupt */
      uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval |= R_SCI_B_CCR0_TIE;
      up_serialout(priv, R_SCI_B_CCR0_OFFSET, regval);

      /* Fake a TX interrupt here by just calling uart_xmitchars() with
       * interrupts disabled (note this may recurse).
       */
       uart_xmitchars(dev);
#endif
    }
  else
    {
      /* Disable the TX interrupt */
      uint32_t regval = up_serialin(priv, R_SCI_B_CCR0_OFFSET);
      regval &= ~R_SCI_B_CCR0_TIE;
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

  return (up_serialin(priv, R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TDRE) != 0;
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
  /* Configuration whichever one is the console
   * lowsetup did GPIO and module power-up,
   * now we do full SCI_B register configuration
   */

  CONSOLE_DEV.isconsole = true;

  /* Perform the full SCI_B initialization for console that was removed from ra_lowsetup()
   * This includes all register configuration that was previously in lowputc
   */
  /* RA8E1 SCI_B full initialization sequence moved from ra_lowsetup()
   * to avoid duplication - ra_lowsetup() only did GPIO and module power-up
   */
  struct up_dev_s *console_priv = (struct up_dev_s *)CONSOLE_DEV.priv;
  uint32_t console_base = console_priv->scibase;
  uint32_t regval;

  /* Initialize SCI_B registers following FSP sequence */

  /* 1. First set CCR0 with IDSEL */
  regval = R_SCI_B_CCR0_IDSEL;
  putreg32(regval, console_base + R_SCI_B_CCR0_OFFSET);

  /* 2. Configure CCR2 for baud rate (115200 @ 120MHz PCLKA)
   * From working XML: MDDR=128 (0x80), BRR=47, BGDM=1, CKS=0 = 0x80002F10
   */
  regval = (128UL << R_SCI_B_CCR2_MDDR_SHIFT) |
           (0 << R_SCI_B_CCR2_CKS_SHIFT) |
           (47 << R_SCI_B_CCR2_BRR_SHIFT) |
           R_SCI_B_CCR2_BGDM;
  putreg32(regval, console_base + R_SCI_B_CCR2_OFFSET);

  /* 3. Configure CCR3 for data format (8-bit, 1 stop, async)
   * From working XML: CHR=2 (8-bit), LSBF=1, RXDESEL=1 = 0x00009200
   */
  regval = R_SCI_B_CCR3_LSBF |
           R_SCI_B_CCR3_RXDESEL |
           (2 << R_SCI_B_CCR3_CHR_SHIFT) |
           (0 & R_SCI_B_CCR3_STP) |
           (0 << R_SCI_B_CCR3_MOD_SHIFT);
  putreg32(regval, console_base + R_SCI_B_CCR3_OFFSET);

  /* 4. Configure CCR1 for proper TXD pin level
   * From working XML: SPB2DT=1, SPB2IO=1 = 0x00000030
   */
  regval = R_SCI_B_CCR1_SPB2DT | R_SCI_B_CCR1_SPB2IO;
  putreg32(regval, console_base + R_SCI_B_CCR1_OFFSET);

  /* 5. Clear CCR4 */
  putreg32(0, console_base + R_SCI_B_CCR4_OFFSET);

  /* 6. Clear all status flags */
  putreg32(R_SCI_B_CFCLR_RDRFC | R_SCI_B_CFCLR_TDREC | R_SCI_B_CFCLR_FERC |
           R_SCI_B_CFCLR_PERC | R_SCI_B_CFCLR_MFFC | R_SCI_B_CFCLR_ORERC |
           R_SCI_B_CFCLR_DFERC | R_SCI_B_CFCLR_DPERC | R_SCI_B_CFCLR_DCMFC |
           R_SCI_B_CFCLR_ERSC, console_base + R_SCI_B_CFCLR_OFFSET);

  /* 7. Clear FIFO flags */
  putreg32(R_SCI_B_FFCLR_DRC, console_base + R_SCI_B_FFCLR_OFFSET);

  /* 8. Enable transmitter and receiver */
  regval = R_SCI_B_CCR0_IDSEL | R_SCI_B_CCR0_RE | R_SCI_B_CCR0_TE;
  putreg32(regval, console_base + R_SCI_B_CCR0_OFFSET);

  /* 9. Wait for receiver internal state = 1 */
  while ((getreg8(console_base + R_SCI_B_CESR_OFFSET) & R_SCI_B_CESR_RIST) == 0)
    {
      /* Wait for RIST bit */
    }
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
