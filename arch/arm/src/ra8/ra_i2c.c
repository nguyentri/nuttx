/****************************************************************************
 * arch/arm/src/ra8/ra_i2c.c
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_slave.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra_icu.h"
#include "hardware/ra_i2c.h"
#include "hardware/ra_dmac.h"
#include "hardware/ra8e1_memorymap.h"
#include "hardware/ra8e1_icu.h"
#include "hardware/ra8e1_mstp.h"
#include "ra_i2c.h"

#ifdef CONFIG_RA8_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2c_dumpgpio(m) ra_dumpgpio(m)
#else
#  define i2c_dumpgpio(m)
#endif

/* DTC timeout */
#define DTC_TIMEOUT_MS          1000

/* I2C timeout */
#define I2C_TIMEOUT_MS          1000

/* I2C State timeout */
#define I2C_STATE_TIMEOUT_US    100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C Device Private Data */
struct ra_i2c_priv_s
{
  /* Standard I2C operations */
  const struct i2c_ops_s *ops;

  /* Port configuration */
  const struct ra_i2c_config_s *config;

  int      refs;          /* Reference count */
  mutex_t  lock;          /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  sem_t    sem_isr;       /* Interrupt wait semaphore */
#endif

  /* I2C work state (see enum ra_i2cstate_e) */
  volatile uint8_t state;

  /* I2C current message */
  struct i2c_msg_s *msgs; /* Remaining transfers - first one is active */
  int      msgc;          /* Number of transfer remaining */

  /* I2C Bus frequency */
  uint32_t frequency;     /* Current I2C frequency */

  /* I2C transfer state */
  uint8_t *ptr;           /* Current message buffer */
  uint32_t dcnt;          /* Current message length */
  uint16_t flags;         /* Current message flags */

  /* I2C address */
  uint8_t  addr;          /* Current message address */

  /* I2C trace support */
#ifdef CONFIG_I2C_TRACE
  int      tndx;          /* Trace array index */
  uint32_t start_time;    /* Time when the trace was started */

  /* The actual trace data */
  struct i2c_trace_s trace[CONFIG_I2C_NTRACE];
#endif

  uint32_t status;        /* End of transfer SR2|SR1 status */
  
  /* DTC support */
  bool     use_dtc;       /* DTC enable flag */
  uint32_t dtc_rx_handle; /* DTC RX handle */
  uint32_t dtc_tx_handle; /* DTC TX handle */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C operations */
static uint32_t ra_i2c_setfrequency(struct i2c_master_s *dev, uint32_t frequency);
static int ra_i2c_setaddress(struct i2c_master_s *dev, int addr, int nbits);
static int ra_i2c_write(struct i2c_master_s *dev, const uint8_t *buffer, int buflen);
static int ra_i2c_read(struct i2c_master_s *dev, uint8_t *buffer, int buflen);
#ifdef CONFIG_I2C_WRITEREAD
static int ra_i2c_writeread(struct i2c_master_s *dev, const uint8_t *wbuffer, 
                           int wbuflen, uint8_t *rbuffer, int rbuflen);
#endif
#ifdef CONFIG_I2C_TRANSFER
static int ra_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count);
#endif
#ifdef CONFIG_I2C_RESET
static int ra_i2c_reset(struct i2c_master_s *dev);
#endif

/* I2C helper functions */
static int ra_i2c_start(struct ra_i2c_priv_s *priv);
static int ra_i2c_stop(struct ra_i2c_priv_s *priv);
static int ra_i2c_sendaddr(struct ra_i2c_priv_s *priv, uint8_t addr, bool readmode);
static int ra_i2c_senddata(struct ra_i2c_priv_s *priv);
static int ra_i2c_readdata(struct ra_i2c_priv_s *priv);
static int ra_i2c_wait_event(struct ra_i2c_priv_s *priv, uint32_t timeout_us);

/* I2C interrupt service routines */
#ifndef CONFIG_I2C_POLLED
static int ra_i2c_isr_rxi(int irq, void *context, void *arg);
static int ra_i2c_isr_txi(int irq, void *context, void *arg);
static int ra_i2c_isr_tei(int irq, void *context, void *arg);
static int ra_i2c_isr_eri(int irq, void *context, void *arg);
static int ra_i2c_isr_start(int irq, void *context, void *arg);
static int ra_i2c_isr_stop(int irq, void *context, void *arg);
static int ra_i2c_isr_nak(int irq, void *context, void *arg);
static int ra_i2c_isr_timeout(int irq, void *context, void *arg);
#endif

/* I2C initialization */
static int ra_i2c_init(struct ra_i2c_priv_s *priv);
static int ra_i2c_deinit(struct ra_i2c_priv_s *priv);

/* DTC functions */
#ifdef CONFIG_RA8_I2C_DTC
static int ra_i2c_dtc_setup(struct ra_i2c_priv_s *priv);
static int ra_i2c_dtc_start_rx(struct ra_i2c_priv_s *priv, uint8_t *buffer, uint32_t len);
static int ra_i2c_dtc_start_tx(struct ra_i2c_priv_s *priv, const uint8_t *buffer, uint32_t len);
static void ra_i2c_dtc_cleanup(struct ra_i2c_priv_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C Interface */
static const struct i2c_ops_s ra_i2c_ops =
{
  .setfrequency = ra_i2c_setfrequency,
  .setaddress   = ra_i2c_setaddress,
  .write        = ra_i2c_write,
  .read         = ra_i2c_read,
#ifdef CONFIG_I2C_WRITEREAD
  .writeread    = ra_i2c_writeread,
#endif
#ifdef CONFIG_I2C_TRANSFER
  .transfer     = ra_i2c_transfer,
#endif
#ifdef CONFIG_I2C_RESET
  .reset        = ra_i2c_reset,
#endif
};

/* I2C device configuration */
#ifdef CONFIG_RA8_I2C0
static const struct ra_i2c_config_s ra_i2c0_config =
{
  .base         = RA_I2C0_BASE,
  .clk_freq     = BOARD_PCLKB_FREQUENCY,
  .bus          = 0,
  .irq_rxi      = 0x35,  /* EVENT_IIC0_RXI */
  .irq_txi      = 0x36,  /* EVENT_IIC0_TXI */
  .irq_tei      = 0x37,  /* EVENT_IIC0_TEI */
  .irq_eri      = 0x38,  /* EVENT_IIC0_ERI */
  .mstpcrb_bit  = 1 << 24,   /* MSTPCRB bit for IIC0 */

  /* Pin configuration - P511/P512 for I2C1 (Arduino compatible) */
  .scl_pin      = (5 << 8) | 12,  /* P512 (SCL1) */
  .sda_pin      = (5 << 8) | 11,  /* P511 (SDA1) */

  /* DTC configuration */
#ifdef CONFIG_RA8_I2C_DTC
  .dtc_rx_ch    = 0,
  .dtc_tx_ch    = 1,
  .dtc_rx_event = 0x35,  /* EVENT_IIC0_RXI */
  .dtc_tx_event = 0x36,  /* EVENT_IIC0_TXI */
#endif
};

static struct ra_i2c_priv_s ra_i2c0_priv =
{
  .ops          = &ra_i2c_ops,
  .config       = &ra_i2c0_config,
  .refs         = 0,
  .lock         = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr      = SEM_INITIALIZER(0),
#endif
  .state        = I2CSTATE_IDLE,
};
#endif

#ifdef CONFIG_RA8_I2C1
static const struct ra_i2c_config_s ra_i2c1_config =
{
  .base         = RA_I2C1_BASE,
  .clk_freq     = BOARD_PCLKB_FREQUENCY,
  .bus          = 1,
  .irq_rxi      = 0x3A,  /* EVENT_IIC1_RXI */
  .irq_txi      = 0x3B,  /* EVENT_IIC1_TXI */
  .irq_tei      = 0x3C,  /* EVENT_IIC1_TEI */
  .irq_eri      = 0x3D,  /* EVENT_IIC1_ERI */
  .mstpcrb_bit  = 1 << 23,   /* MSTPCRB bit for IIC1 */

  /* Pin configuration - default pins for I2C1 */
  .scl_pin      = (5 << 8) | 12,  /* P512 (SCL1) */
  .sda_pin      = (5 << 8) | 11,  /* P511 (SDA1) */

  /* DTC configuration */
#ifdef CONFIG_RA8_I2C_DTC
  .dtc_rx_ch    = 2,
  .dtc_tx_ch    = 3,
  .dtc_rx_event = 0x3A,  /* EVENT_IIC1_RXI */
  .dtc_tx_event = 0x3B,  /* EVENT_IIC1_TXI */
#endif
};

static struct ra_i2c_priv_s ra_i2c1_priv =
{
  .ops          = &ra_i2c_ops,
  .config       = &ra_i2c1_config,
  .refs         = 0,
  .lock         = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr      = SEM_INITIALIZER(0),
#endif
  .state        = I2CSTATE_IDLE,
};
#endif

#ifdef CONFIG_RA8_I2C2
static const struct ra_i2c_config_s ra_i2c2_config =
{
  .base         = RA_I2C2_BASE,
  .clk_freq     = BOARD_PCLKB_FREQUENCY,
  .bus          = 2,
  .irq_rxi      = 0x40,  /* EVENT_IIC2_RXI (estimated) */
  .irq_txi      = 0x41,  /* EVENT_IIC2_TXI (estimated) */
  .irq_tei      = 0x42,  /* EVENT_IIC2_TEI (estimated) */
  .irq_eri      = 0x43,  /* EVENT_IIC2_ERI (estimated) */
  .mstpcrb_bit  = 1 << 22,   /* MSTPCRB bit for IIC2 */

  /* Pin configuration - default pins for I2C2 */
  .scl_pin      = (3 << 8) | 2,   /* P302 */
  .sda_pin      = (3 << 8) | 1,   /* P301 */

  /* DTC configuration */
#ifdef CONFIG_RA8_I2C_DTC
  .dtc_rx_ch    = 4,
  .dtc_tx_ch    = 5,
  .dtc_rx_event = 0x40,  /* EVENT_IIC2_RXI (estimated) */
  .dtc_tx_event = 0x41,  /* EVENT_IIC2_TXI (estimated) */
#endif
};

static struct ra_i2c_priv_s ra_i2c2_priv =
{
  .ops          = &ra_i2c_ops,
  .config       = &ra_i2c2_config,
  .refs         = 0,
  .lock         = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr      = SEM_INITIALIZER(0),
#endif
  .state        = I2CSTATE_IDLE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_i2c_getreg
 *
 * Description:
 *   Get a 8-bit register value by offset
 *
 ****************************************************************************/

static inline uint8_t ra_i2c_getreg(struct ra_i2c_priv_s *priv, uint8_t offset)
{
  return getreg8(priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_i2c_putreg
 *
 * Description:
 *   Put a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra_i2c_putreg(struct ra_i2c_priv_s *priv, uint8_t offset, uint8_t value)
{
  putreg8(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_i2c_modifyreg
 *
 * Description:
 *   Modify a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra_i2c_modifyreg(struct ra_i2c_priv_s *priv, uint8_t offset,
                                   uint8_t clearbits, uint8_t setbits)
{
  modifyreg8(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: ra_i2c_setfrequency
 *
 * Description:
 *   Set the I2C frequency
 *
 ****************************************************************************/

static uint32_t ra_i2c_setfrequency(struct i2c_master_s *dev, uint32_t frequency)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;
  uint32_t pclkb_freq;
  uint32_t cks;
  uint32_t brh, brl;
  uint32_t actual_freq;

  DEBUGASSERT(priv != NULL);

  /* Check if frequency has changed */
  if (frequency == priv->frequency)
    {
      return priv->frequency;
    }

  /* Get PCLKB frequency */
  pclkb_freq = priv->config->clk_freq;

  /* Calculate clock source divider and bit rate registers
   * I2C frequency = PCLKB / (2^(CKS+1) * (BRH + BRL + 2))
   */
  
  /* Start with CKS = 0 (PCLKB/1) */
  for (cks = 0; cks <= 7; cks++)
    {
      uint32_t divisor = 1 << (cks + 1);
      uint32_t scl_freq = pclkb_freq / divisor;
      uint32_t total_count = scl_freq / frequency;
      
      if (total_count >= 4 && total_count <= 514) /* BRH + BRL + 2 can be 4 to 514 */
        {
          /* Split total count between BRH and BRL */
          uint32_t bit_rate = total_count - 2;
          brh = bit_rate / 2;
          brl = bit_rate - brh;
          
          /* Ensure BRH and BRL are in valid range (0-255) */
          if (brh <= 255 && brl <= 255)
            {
              break;
            }
        }
    }

  if (cks > 7)
    {
      /* Cannot achieve requested frequency, use minimum */
      cks = 7;
      brh = 255;
      brl = 255;
    }

  /* Calculate actual frequency */
  actual_freq = pclkb_freq / ((1 << (cks + 1)) * (brh + brl + 2));

  i2cinfo("I2C%d frequency: requested=%lu, actual=%lu, cks=%lu, brh=%lu, brl=%lu\n",
          priv->config->bus, frequency, actual_freq, cks, brh, brl);

  /* Disable I2C while changing settings */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, I2C_ICCR1_ICE, 0);

  /* Set clock source */
  ra_i2c_modifyreg(priv, RA_I2C_ICMR1_OFFSET, I2C_ICMR1_CKS_MASK,
                  (cks << I2C_ICMR1_CKS_SHIFT) & I2C_ICMR1_CKS_MASK);

  /* Set bit rate registers */
  ra_i2c_putreg(priv, RA_I2C_ICBRH_OFFSET, brh & ~I2C_BRR_RESERVED_BITS);
  ra_i2c_putreg(priv, RA_I2C_ICBRL_OFFSET, brl & ~I2C_BRR_RESERVED_BITS);

  /* Re-enable I2C */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, 0, I2C_ICCR1_ICE);

  priv->frequency = actual_freq;
  return actual_freq;
}

/****************************************************************************
 * Name: ra_i2c_setaddress
 *
 * Description:
 *   Set the I2C slave address for a subsequent read/write
 *
 ****************************************************************************/

static int ra_i2c_setaddress(struct i2c_master_s *dev, int addr, int nbits)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(nbits == 7 || nbits == 10);

  priv->addr = addr;

  /* Note: The actual address will be sent when starting the transfer */
  return OK;
}

/****************************************************************************
 * Name: ra_i2c_write
 *
 * Description:
 *   Send a block of data on I2C using the previously selected I2C
 *   frequency and slave address.
 *
 ****************************************************************************/

static int ra_i2c_write(struct i2c_master_s *dev, const uint8_t *buffer, int buflen)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;
  struct i2c_msg_s msg;
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Setup message */
  msg.frequency = priv->frequency;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = (uint8_t *)buffer;
  msg.length    = buflen;

  /* Perform the transfer */
  ret = ra_i2c_transfer(dev, &msg, 1);

  return ret;
}

/****************************************************************************
 * Name: ra_i2c_read
 *
 * Description:
 *   Receive a block of data from I2C using the previously selected I2C
 *   frequency and slave address.
 *
 ****************************************************************************/

static int ra_i2c_read(struct i2c_master_s *dev, uint8_t *buffer, int buflen)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;
  struct i2c_msg_s msg;
  int ret;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Setup message */
  msg.frequency = priv->frequency;
  msg.addr      = priv->addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = buffer;
  msg.length    = buflen;

  /* Perform the transfer */
  ret = ra_i2c_transfer(dev, &msg, 1);

  return ret;
}

#ifdef CONFIG_I2C_WRITEREAD
/****************************************************************************
 * Name: ra_i2c_writeread
 *
 * Description:
 *   Write then read data
 *
 ****************************************************************************/

static int ra_i2c_writeread(struct i2c_master_s *dev, const uint8_t *wbuffer,
                           int wbuflen, uint8_t *rbuffer, int rbuflen)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;
  struct i2c_msg_s msgs[2];
  int ret;

  DEBUGASSERT(priv != NULL && wbuffer != NULL && wbuflen > 0 &&
              rbuffer != NULL && rbuflen > 0);

  /* Setup write message */
  msgs[0].frequency = priv->frequency;
  msgs[0].addr      = priv->addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = (uint8_t *)wbuffer;
  msgs[0].length    = wbuflen;

  /* Setup read message */
  msgs[1].frequency = priv->frequency;
  msgs[1].addr      = priv->addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = rbuffer;
  msgs[1].length    = rbuflen;

  /* Perform the transfer */
  ret = ra_i2c_transfer(dev, msgs, 2);

  return ret;
}
#endif

/****************************************************************************
 * Name: ra_i2c_start
 *
 * Description:
 *   Generate I2C start condition
 *
 ****************************************************************************/

static int ra_i2c_start(struct ra_i2c_priv_s *priv)
{
  uint32_t timeout = I2C_STATE_TIMEOUT_US;

  /* Wait for bus to be free */
  while ((ra_i2c_getreg(priv, RA_I2C_ICCR2_OFFSET) & I2C_ICCR2_BBSY) && timeout--)
    {
      up_udelay(1);
    }

  if (timeout == 0)
    {
      i2cerr("I2C%d: Bus busy timeout\n", priv->config->bus);
      return -EBUSY;
    }

  /* Set master mode and transmit mode */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR2_OFFSET, 0, I2C_ICCR2_MST | I2C_ICCR2_TRS);

  /* Generate start condition */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR2_OFFSET, 0, I2C_ICCR2_ST);

  priv->state = I2CSTATE_START;

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_stop
 *
 * Description:
 *   Generate I2C stop condition
 *
 ****************************************************************************/

static int ra_i2c_stop(struct ra_i2c_priv_s *priv)
{
  uint32_t timeout = I2C_STATE_TIMEOUT_US;

  /* Generate stop condition */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR2_OFFSET, 0, I2C_ICCR2_SP);

  /* Wait for stop condition to complete */
  while ((ra_i2c_getreg(priv, RA_I2C_ICSR2_OFFSET) & I2C_ICSR2_STOP) == 0 && timeout--)
    {
      up_udelay(1);
    }

  /* Clear stop flag */
  ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_STOP, 0);

  /* Clear master mode */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR2_OFFSET, I2C_ICCR2_MST, 0);

  priv->state = I2CSTATE_IDLE;

  return (timeout > 0) ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: ra_i2c_sendaddr
 *
 * Description:
 *   Send I2C address
 *
 ****************************************************************************/

static int ra_i2c_sendaddr(struct ra_i2c_priv_s *priv, uint8_t addr, bool readmode)
{
  uint8_t addr_byte;

  /* Prepare address byte */
  addr_byte = (addr << 1) | (readmode ? 1 : 0);

  /* Send address */
  ra_i2c_putreg(priv, RA_I2C_ICDRT_OFFSET, addr_byte);

  priv->state = readmode ? I2CSTATE_ADDR_READ : I2CSTATE_ADDR_WRITE;

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_senddata
 *
 * Description:
 *   Send data byte
 *
 ****************************************************************************/

static int ra_i2c_senddata(struct ra_i2c_priv_s *priv)
{
  if (priv->dcnt > 0)
    {
      /* Send data byte */
      ra_i2c_putreg(priv, RA_I2C_ICDRT_OFFSET, *priv->ptr++);
      priv->dcnt--;
      priv->state = I2CSTATE_WRITE;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_readdata
 *
 * Description:
 *   Read data byte
 *
 ****************************************************************************/

static int ra_i2c_readdata(struct ra_i2c_priv_s *priv)
{
  if (priv->dcnt > 0)
    {
      /* Read data byte */
      *priv->ptr++ = ra_i2c_getreg(priv, RA_I2C_ICDRR_OFFSET);
      priv->dcnt--;
      priv->state = I2CSTATE_READ;

      /* If this is the last byte, send NACK */
      if (priv->dcnt == 1)
        {
          ra_i2c_modifyreg(priv, RA_I2C_ICMR3_OFFSET, 0, I2C_ICMR3_ACKBT);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_wait_event
 *
 * Description:
 *   Wait for I2C event
 *
 ****************************************************************************/

static int ra_i2c_wait_event(struct ra_i2c_priv_s *priv, uint32_t timeout_us)
{
#ifdef CONFIG_I2C_POLLED
  uint32_t timeout = timeout_us;
  uint8_t sr2;

  /* Poll for events */
  while (timeout--)
    {
      sr2 = ra_i2c_getreg(priv, RA_I2C_ICSR2_OFFSET);

      /* Check for errors */
      if (sr2 & (I2C_ICSR2_AL | I2C_ICSR2_TMOF))
        {
          priv->status = sr2;
          return -EIO;
        }

      /* Check for NACK */
      if (sr2 & I2C_ICSR2_NACKF)
        {
          priv->status = sr2;
          return -ENXIO;
        }

      /* Check for events based on current state */
      switch (priv->state)
        {
          case I2CSTATE_START:
            if (sr2 & I2C_ICSR2_START)
              {
                ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_START, 0);
                return OK;
              }
            break;

          case I2CSTATE_ADDR_WRITE:
          case I2CSTATE_WRITE:
            if (sr2 & I2C_ICSR2_TDRE)
              {
                return OK;
              }
            break;

          case I2CSTATE_ADDR_READ:
            if (sr2 & I2C_ICSR2_TDRE)
              {
                /* Switch to receive mode */
                ra_i2c_modifyreg(priv, RA_I2C_ICCR2_OFFSET, I2C_ICCR2_TRS, 0);
                /* Dummy read to start reception */
                (void)ra_i2c_getreg(priv, RA_I2C_ICDRR_OFFSET);
                return OK;
              }
            break;

          case I2CSTATE_READ:
            if (sr2 & I2C_ICSR2_RDRF)
              {
                return OK;
              }
            break;

          case I2CSTATE_STOP:
            if (sr2 & I2C_ICSR2_STOP)
              {
                ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_STOP, 0);
                return OK;
              }
            break;

          default:
            break;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
#else
  /* Wait for interrupt */
  return nxsem_tickwait_uninterruptible(&priv->sem_isr, USEC2TICK(timeout_us));
#endif
}

#ifdef CONFIG_I2C_TRANSFER
/****************************************************************************
 * Name: ra_i2c_transfer
 *
 * Description:
 *   Generic I2C transfer function
 *
 ****************************************************************************/

static int ra_i2c_transfer(struct i2c_master_s *dev, struct i2c_msg_s *msgs, int count)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;
  int ret = OK;
  int i;

  DEBUGASSERT(priv != NULL && msgs != NULL && count > 0);

  /* Get exclusive access to the I2C bus */
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Set the frequency if it has changed */
  if (msgs[0].frequency != priv->frequency)
    {
      ra_i2c_setfrequency(dev, msgs[0].frequency);
    }

  /* Process each message */
  for (i = 0; i < count && ret == OK; i++)
    {
      priv->msgs = &msgs[i];
      priv->msgc = 1;
      priv->ptr = msgs[i].buffer;
      priv->dcnt = msgs[i].length;
      priv->flags = msgs[i].flags;
      priv->addr = msgs[i].addr;

      /* Generate start condition (or repeated start) */
      ret = ra_i2c_start(priv);
      if (ret != OK)
        {
          break;
        }

      /* Wait for start condition */
      ret = ra_i2c_wait_event(priv, I2C_STATE_TIMEOUT_US);
      if (ret != OK)
        {
          break;
        }

      /* Send address */
      ret = ra_i2c_sendaddr(priv, msgs[i].addr, (msgs[i].flags & I2C_M_READ) != 0);
      if (ret != OK)
        {
          break;
        }

      /* Wait for address ACK */
      ret = ra_i2c_wait_event(priv, I2C_STATE_TIMEOUT_US);
      if (ret != OK)
        {
          break;
        }

      /* Transfer data */
      if (msgs[i].flags & I2C_M_READ)
        {
          /* Reading - switch to receive mode after address */
          ra_i2c_modifyreg(priv, RA_I2C_ICCR2_OFFSET, I2C_ICCR2_TRS, 0);
          
          /* Dummy read to start reception */
          (void)ra_i2c_getreg(priv, RA_I2C_ICDRR_OFFSET);

          /* Read all bytes */
          while (priv->dcnt > 0 && ret == OK)
            {
              ret = ra_i2c_wait_event(priv, I2C_STATE_TIMEOUT_US);
              if (ret == OK)
                {
                  ret = ra_i2c_readdata(priv);
                }
            }
        }
      else
        {
          /* Writing - send all bytes */
          while (priv->dcnt > 0 && ret == OK)
            {
              ret = ra_i2c_wait_event(priv, I2C_STATE_TIMEOUT_US);
              if (ret == OK)
                {
                  ret = ra_i2c_senddata(priv);
                }
            }
        }

      /* Generate stop condition for last message or if I2C_M_NOSTOP is not set */
      if (ret == OK && (i == count - 1 || !(msgs[i].flags & I2C_M_NOSTOP)))
        {
          ret = ra_i2c_stop(priv);
        }
    }

  /* Generate stop condition if transfer failed */
  if (ret != OK && priv->state != I2CSTATE_IDLE)
    {
      ra_i2c_stop(priv);
    }

  nxmutex_unlock(&priv->lock);

  return ret;
}
#endif

#ifdef CONFIG_I2C_RESET
/****************************************************************************
 * Name: ra_i2c_reset
 *
 * Description:
 *   Reset the I2C bus
 *
 ****************************************************************************/

static int ra_i2c_reset(struct i2c_master_s *dev)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Get exclusive access to the I2C bus */
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Disable and re-enable I2C to reset state */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, I2C_ICCR1_ICE, 0);
  up_udelay(10);
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, 0, I2C_ICCR1_ICE);

  priv->state = I2CSTATE_IDLE;

  nxmutex_unlock(&priv->lock);

  return OK;
}
#endif

/****************************************************************************
 * Name: ra_i2c_init
 *
 * Description:
 *   Initialize the I2C hardware
 *
 ****************************************************************************/

static int ra_i2c_init(struct ra_i2c_priv_s *priv)
{
  const struct ra_i2c_config_s *config = priv->config;
  uint32_t regval;

  /* Enable I2C module clock */
  regval = getreg32(0x40036038);  /* MSTPCRB register */
  regval &= ~config->mstpcrb_bit;
  putreg32(regval, 0x40036038);

  /* Configure I2C pins - TODO: Implement proper GPIO configuration */
  /* ra_gpio_config(config->scl_pin); */
  /* ra_gpio_config(config->sda_pin); */

  /* Reset I2C peripheral */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, 0, I2C_ICCR1_IICRST);
  up_udelay(10);
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, I2C_ICCR1_IICRST, 0);

  /* Configure I2C mode registers */
  /* ICMR1: Set internal reference clock select and bit counter */
  ra_i2c_putreg(priv, RA_I2C_ICMR1_OFFSET, 0);

  /* ICMR2: Configure delays and timeout */
  ra_i2c_putreg(priv, RA_I2C_ICMR2_OFFSET, 0);

  /* ICMR3: Configure SMBus/I2C selection and noise filter */
  ra_i2c_putreg(priv, RA_I2C_ICMR3_OFFSET, I2C_ICMR3_NF_MASK); /* Enable noise filter */

  /* ICFER: Configure function enables */
  ra_i2c_putreg(priv, RA_I2C_ICFER_OFFSET, 
                I2C_ICFER_TMOE |    /* Enable timeout */
                I2C_ICFER_MALE |    /* Enable master arbitration-lost detection */
                I2C_ICFER_NALE |    /* Enable NACK arbitration-lost detection */
                I2C_ICFER_SALE |    /* Enable slave arbitration-lost detection */
                I2C_ICFER_NACKE |   /* Enable NACK transmission arbitration-lost detection */
                I2C_ICFER_NFE |     /* Enable digital noise filter */
                I2C_ICFER_SCLE);    /* Enable SCL synchronous circuit */

  /* ICSER: Disable slave address detection */
  ra_i2c_putreg(priv, RA_I2C_ICSER_OFFSET, 0);

#ifndef CONFIG_I2C_POLLED
  /* Configure and enable interrupts */
  ra_i2c_putreg(priv, RA_I2C_ICIER_OFFSET,
                I2C_ICIER_TIE |     /* Transmit data empty interrupt */
                I2C_ICIER_TEIE |    /* Transmit end interrupt */
                I2C_ICIER_RIE |     /* Receive data full interrupt */
                I2C_ICIER_NAKIE |   /* NACK detection interrupt */
                I2C_ICIER_SPIE |    /* Stop condition detection interrupt */
                I2C_ICIER_STIE |    /* Start condition detection interrupt */
                I2C_ICIER_ALIE |    /* Arbitration-lost detection interrupt */
                I2C_ICIER_TMOIE);   /* Timeout detection interrupt */

  /* Attach interrupt handlers */
  irq_attach(config->irq_rxi, ra_i2c_isr_rxi, priv);
  irq_attach(config->irq_txi, ra_i2c_isr_txi, priv);
  irq_attach(config->irq_tei, ra_i2c_isr_tei, priv);
  irq_attach(config->irq_eri, ra_i2c_isr_eri, priv);

  /* Enable interrupts */
  up_enable_irq(config->irq_rxi);
  up_enable_irq(config->irq_txi);
  up_enable_irq(config->irq_tei);
  up_enable_irq(config->irq_eri);
#endif

  /* Enable I2C peripheral */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, 0, I2C_ICCR1_ICE);

  /* Set default frequency */
  priv->frequency = 0;  /* Force frequency setting */
  ra_i2c_setfrequency((struct i2c_master_s *)priv, I2C_FREQ_STANDARD);

#ifdef CONFIG_RA8_I2C_DTC
  /* Setup DTC if enabled */
  priv->use_dtc = true;
  ra_i2c_dtc_setup(priv);
#endif

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_deinit
 *
 * Description:
 *   Deinitialize the I2C hardware
 *
 ****************************************************************************/

static int ra_i2c_deinit(struct ra_i2c_priv_s *priv)
{
  const struct ra_i2c_config_s *config = priv->config;
  uint32_t regval;

  /* Disable I2C peripheral */
  ra_i2c_modifyreg(priv, RA_I2C_ICCR1_OFFSET, I2C_ICCR1_ICE, 0);

#ifndef CONFIG_I2C_POLLED
  /* Disable interrupts */
  up_disable_irq(config->irq_rxi);
  up_disable_irq(config->irq_txi);
  up_disable_irq(config->irq_tei);
  up_disable_irq(config->irq_eri);

  /* Detach interrupt handlers */
  irq_detach(config->irq_rxi);
  irq_detach(config->irq_txi);
  irq_detach(config->irq_tei);
  irq_detach(config->irq_eri);
#endif

#ifdef CONFIG_RA8_I2C_DTC
  /* Cleanup DTC */
  ra_i2c_dtc_cleanup(priv);
#endif

  /* Disable I2C module clock */
  regval = getreg32(0x40036038);  /* MSTPCRB register */
  regval |= config->mstpcrb_bit;
  putreg32(regval, 0x40036038);

  return OK;
}

#ifndef CONFIG_I2C_POLLED
/****************************************************************************
 * Name: ra_i2c_isr_rxi
 *
 * Description:
 *   I2C RX interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_rxi(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_txi
 *
 * Description:
 *   I2C TX interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_txi(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_tei
 *
 * Description:
 *   I2C transfer end interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_tei(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_eri
 *
 * Description:
 *   I2C error interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_eri(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;
  uint8_t sr2;

  DEBUGASSERT(priv != NULL);

  /* Read status to determine error type */
  sr2 = ra_i2c_getreg(priv, RA_I2C_ICSR2_OFFSET);
  priv->status = sr2;

  /* Clear error flags */
  ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, 
                  I2C_ICSR2_AL | I2C_ICSR2_TMOF | I2C_ICSR2_NACKF, 0);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_start
 *
 * Description:
 *   I2C start condition interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_start(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Clear start flag */
  ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_START, 0);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_stop
 *
 * Description:
 *   I2C stop condition interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_stop(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Clear stop flag */
  ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_STOP, 0);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_nak
 *
 * Description:
 *   I2C NAK interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_nak(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Read status */
  priv->status = ra_i2c_getreg(priv, RA_I2C_ICSR2_OFFSET);

  /* Clear NAK flag */
  ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_NACKF, 0);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_isr_timeout
 *
 * Description:
 *   I2C timeout interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_isr_timeout(int irq, void *context, void *arg)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Read status */
  priv->status = ra_i2c_getreg(priv, RA_I2C_ICSR2_OFFSET);

  /* Clear timeout flag */
  ra_i2c_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_TMOF, 0);

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}
#endif /* !CONFIG_I2C_POLLED */

#ifdef CONFIG_RA8_I2C_DTC
/****************************************************************************
 * Name: ra_i2c_dtc_setup
 *
 * Description:
 *   Setup DTC for I2C transfers
 *
 ****************************************************************************/

static int ra_i2c_dtc_setup(struct ra_i2c_priv_s *priv)
{
  /* TODO: Implement DTC setup for I2C */
  i2cinfo("DTC setup for I2C%d\n", priv->config->bus);
  
  /* For now, disable DTC until full implementation */
  priv->use_dtc = false;
  
  return OK;
}

/****************************************************************************
 * Name: ra_i2c_dtc_start_rx
 *
 * Description:
 *   Start DTC for I2C RX transfer
 *
 ****************************************************************************/

static int ra_i2c_dtc_start_rx(struct ra_i2c_priv_s *priv, uint8_t *buffer, uint32_t len)
{
  /* TODO: Implement DTC RX transfer setup */
  UNUSED(priv);
  UNUSED(buffer);
  UNUSED(len);
  
  return -ENOSYS;
}

/****************************************************************************
 * Name: ra_i2c_dtc_start_tx
 *
 * Description:
 *   Start DTC for I2C TX transfer
 *
 ****************************************************************************/

static int ra_i2c_dtc_start_tx(struct ra_i2c_priv_s *priv, const uint8_t *buffer, uint32_t len)
{
  /* TODO: Implement DTC TX transfer setup */
  UNUSED(priv);
  UNUSED(buffer);
  UNUSED(len);
  
  return -ENOSYS;
}

/****************************************************************************
 * Name: ra_i2c_dtc_cleanup
 *
 * Description:
 *   Cleanup DTC resources
 *
 ****************************************************************************/

static void ra_i2c_dtc_cleanup(struct ra_i2c_priv_s *priv)
{
  /* TODO: Implement DTC cleanup */
  i2cinfo("DTC cleanup for I2C%d\n", priv->config->bus);
}
#endif /* CONFIG_RA8_I2C_DTC */

/****************************************************************************
 * Name: ra_i2cbus_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_master_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency and slave address.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_master_s *ra_i2cbus_initialize(int port)
{
  struct ra_i2c_priv_s *priv = NULL;

  i2cinfo("I2C%d: Initialize\n", port);

  /* Get I2C private structure */
  switch (port)
    {
#ifdef CONFIG_RA8_I2C0
      case 0:
        priv = &ra_i2c0_priv;
        break;
#endif

#ifdef CONFIG_RA8_I2C1
      case 1:
        priv = &ra_i2c1_priv;
        break;
#endif

#ifdef CONFIG_RA8_I2C2
      case 2:
        priv = &ra_i2c2_priv;
        break;
#endif

      default:
        i2cerr("I2C%d: Invalid port\n", port);
        return NULL;
    }

  /* Initialize the device structure */
  if (priv->refs++ == 0)
    {
      /* Initialize the I2C hardware */
      ra_i2c_init(priv);
    }

  return (struct i2c_master_s *)priv;
}

/****************************************************************************
 * Name: ra_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by ra_i2cbus_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int ra_i2cbus_uninitialize(struct i2c_master_s *dev)
{
  struct ra_i2c_priv_s *priv = (struct ra_i2c_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* Decrement reference count and check if we should disable the peripheral */
  if (--priv->refs == 0)
    {
      /* Disable the I2C hardware */
      ra_i2c_deinit(priv);
    }

  return OK;
}

#endif /* CONFIG_RA8_I2C */
