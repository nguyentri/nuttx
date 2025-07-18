/****************************************************************************
 * arch/arm/src/ra8/ra_i2c_slave.c
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
#include <nuttx/i2c/i2c_slave.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/ra_i2c.h"
#include "ra_i2c.h"

#ifdef CONFIG_RA8_I2C_SLAVE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_I2C_INFO
#  define i2cs_dumpgpio(m) ra_dumpgpio(m)
#else
#  define i2cs_dumpgpio(m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C Slave Device Private Data */
struct ra_i2c_slave_priv_s
{
  /* Standard I2C slave operations */
  const struct i2c_slave_ops_s *ops;

  /* Port configuration */
  const struct ra_i2c_config_s *config;

  int      refs;          /* Reference count */
  mutex_t  lock;          /* Mutual exclusion mutex */

#ifndef CONFIG_I2C_POLLED
  sem_t    sem_isr;       /* Interrupt wait semaphore */
#endif

  /* I2C slave state */
  volatile uint8_t state;

  /* I2C slave address */
  uint16_t slave_addr;    /* Own slave address */

  /* I2C transfer state */
  uint8_t *buffer;        /* Current transfer buffer */
  uint32_t buflen;        /* Buffer length */
  uint32_t nbytes;        /* Number of bytes transferred */

  /* Callback function */
  i2c_slave_callback_t callback;
  void *callback_arg;

  uint32_t status;        /* End of transfer status */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* I2C slave operations */
static int ra_i2c_slave_setaddress(struct i2c_slave_s *dev, int addr);
static int ra_i2c_slave_write(struct i2c_slave_s *dev, const uint8_t *buffer, int buflen);
static int ra_i2c_slave_read(struct i2c_slave_s *dev, uint8_t *buffer, int buflen);
static int ra_i2c_slave_registercallback(struct i2c_slave_s *dev,
                                        i2c_slave_callback_t callback, void *arg);

/* I2C slave helper functions */
static int ra_i2c_slave_init(struct ra_i2c_slave_priv_s *priv);
static int ra_i2c_slave_deinit(struct ra_i2c_slave_priv_s *priv);

/* I2C slave interrupt service routines */
#ifndef CONFIG_I2C_POLLED
static int ra_i2c_slave_isr_rxi(int irq, void *context, void *arg);
static int ra_i2c_slave_isr_txi(int irq, void *context, void *arg);
static int ra_i2c_slave_isr_tei(int irq, void *context, void *arg);
static int ra_i2c_slave_isr_eri(int irq, void *context, void *arg);
static int ra_i2c_slave_isr_start(int irq, void *context, void *arg);
static int ra_i2c_slave_isr_stop(int irq, void *context, void *arg);
static int ra_i2c_slave_isr_address(int irq, void *context, void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C Slave Interface */
static const struct i2c_slave_ops_s ra_i2c_slave_ops =
{
  .setaddress       = ra_i2c_slave_setaddress,
  .write            = ra_i2c_slave_write,
  .read             = ra_i2c_slave_read,
  .registercallback = ra_i2c_slave_registercallback,
};

/* I2C slave device configuration */
#ifdef CONFIG_RA8_I2C0_SLAVE
static const struct ra_i2c_config_s ra_i2c0_slave_config =
{
  .base         = RA_I2C0_BASE,
  .clk_freq     = BOARD_PCLKB_FREQUENCY,
  .bus          = 0,
  .irq_rxi      = 0x35,  /* EVENT_IIC0_RXI */
  .irq_txi      = 0x36,  /* EVENT_IIC0_TXI */
  .irq_tei      = 0x37,  /* EVENT_IIC0_TEI */
  .irq_eri      = 0x38,  /* EVENT_IIC0_ERI */
  .mstpcrb_bit  = 1 << 24,   /* MSTPCRB bit for IIC0 */

  /* Pin configuration */
  .scl_pin      = (5 << 8) | 12,  /* P512 (SCL1) */
  .sda_pin      = (5 << 8) | 11,  /* P511 (SDA1) */
};

static struct ra_i2c_slave_priv_s ra_i2c0_slave_priv =
{
  .ops          = &ra_i2c_slave_ops,
  .config       = &ra_i2c0_slave_config,
  .refs         = 0,
  .lock         = NXMUTEX_INITIALIZER,
#ifndef CONFIG_I2C_POLLED
  .sem_isr      = SEM_INITIALIZER(0),
#endif
  .state        = I2CSTATE_IDLE,
};
#endif

#ifdef CONFIG_RA8_I2C1_SLAVE
static const struct ra_i2c_config_s ra_i2c1_slave_config =
{
  .base         = RA_I2C1_BASE,
  .clk_freq     = BOARD_PCLKB_FREQUENCY,
  .bus          = 1,
  .irq_rxi      = 0x3A,  /* EVENT_IIC1_RXI */
  .irq_txi      = 0x3B,  /* EVENT_IIC1_TXI */
  .irq_tei      = 0x3C,  /* EVENT_IIC1_TEI */
  .irq_eri      = 0x3D,  /* EVENT_IIC1_ERI */
  .mstpcrb_bit  = 1 << 23,   /* MSTPCRB bit for IIC1 */

  /* Pin configuration */
  .scl_pin      = (5 << 8) | 12,  /* P512 (SCL1) */
  .sda_pin      = (5 << 8) | 11,  /* P511 (SDA1) */
};

static struct ra_i2c_slave_priv_s ra_i2c1_slave_priv =
{
  .ops          = &ra_i2c_slave_ops,
  .config       = &ra_i2c1_slave_config,
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
 * Name: ra_i2c_slave_getreg
 *
 * Description:
 *   Get a 8-bit register value by offset
 *
 ****************************************************************************/

static inline uint8_t ra_i2c_slave_getreg(struct ra_i2c_slave_priv_s *priv, uint8_t offset)
{
  return getreg8(priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_i2c_slave_putreg
 *
 * Description:
 *   Put a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra_i2c_slave_putreg(struct ra_i2c_slave_priv_s *priv, uint8_t offset, uint8_t value)
{
  putreg8(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_i2c_slave_modifyreg
 *
 * Description:
 *   Modify a 8-bit register value by offset
 *
 ****************************************************************************/

static inline void ra_i2c_slave_modifyreg(struct ra_i2c_slave_priv_s *priv, uint8_t offset,
                                         uint8_t clearbits, uint8_t setbits)
{
  modifyreg8(priv->config->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: ra_i2c_slave_setaddress
 *
 * Description:
 *   Set the I2C slave address
 *
 ****************************************************************************/

static int ra_i2c_slave_setaddress(struct i2c_slave_s *dev, int addr)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)dev;
  uint8_t regval;

  DEBUGASSERT(priv != NULL);
  DEBUGASSERT(addr >= 0 && addr <= 0x7F);

  priv->slave_addr = addr;

  /* Set slave address in SARL0/SARU0 registers */
  ra_i2c_slave_putreg(priv, RA_I2C_SARL0_OFFSET, (addr << 1) & 0xFE);
  
  regval = ra_i2c_slave_getreg(priv, RA_I2C_SARU0_OFFSET);
  regval &= ~(I2C_SARU_SVA_MASK | I2C_SARU_FS);
  regval |= ((addr >> 7) & 0x03) << I2C_SARU_SVA_SHIFT;  /* Upper 2 bits */
  /* FS bit = 0 for 7-bit address format */
  ra_i2c_slave_putreg(priv, RA_I2C_SARU0_OFFSET, regval);

  /* Enable slave address 0 detection */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICSER_OFFSET, 0, I2C_ICSER_SAR0E);

  i2cinfo("I2C%d slave address set to 0x%02X\n", priv->config->bus, addr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_write
 *
 * Description:
 *   Send data to I2C master (slave transmit mode)
 *
 ****************************************************************************/

static int ra_i2c_slave_write(struct i2c_slave_s *dev, const uint8_t *buffer, int buflen)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Get exclusive access */
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup transfer */
  priv->buffer = (uint8_t *)buffer;
  priv->buflen = buflen;
  priv->nbytes = 0;

  /* TODO: Implement slave write functionality */
  /* This would typically involve:
   * 1. Waiting for master to request data
   * 2. Sending data bytes when requested
   * 3. Handling NACK from master
   */

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: ra_i2c_slave_read
 *
 * Description:
 *   Receive data from I2C master (slave receive mode)
 *
 ****************************************************************************/

static int ra_i2c_slave_read(struct i2c_slave_s *dev, uint8_t *buffer, int buflen)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)dev;
  int ret = OK;

  DEBUGASSERT(priv != NULL && buffer != NULL && buflen > 0);

  /* Get exclusive access */
  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup transfer */
  priv->buffer = buffer;
  priv->buflen = buflen;
  priv->nbytes = 0;

  /* TODO: Implement slave read functionality */
  /* This would typically involve:
   * 1. Waiting for master to send data
   * 2. Receiving data bytes
   * 3. Sending ACK/NACK as appropriate
   */

  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: ra_i2c_slave_registercallback
 *
 * Description:
 *   Register a callback function for I2C slave events
 *
 ****************************************************************************/

static int ra_i2c_slave_registercallback(struct i2c_slave_s *dev,
                                        i2c_slave_callback_t callback, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  priv->callback = callback;
  priv->callback_arg = arg;

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_init
 *
 * Description:
 *   Initialize the I2C slave hardware
 *
 ****************************************************************************/

static int ra_i2c_slave_init(struct ra_i2c_slave_priv_s *priv)
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
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICCR1_OFFSET, 0, I2C_ICCR1_IICRST);
  up_udelay(10);
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICCR1_OFFSET, I2C_ICCR1_IICRST, 0);

  /* Configure I2C mode registers for slave mode */
  /* ICMR1: Set internal reference clock select and bit counter */
  ra_i2c_slave_putreg(priv, RA_I2C_ICMR1_OFFSET, 0);

  /* ICMR2: Configure delays and timeout */
  ra_i2c_slave_putreg(priv, RA_I2C_ICMR2_OFFSET, 0);

  /* ICMR3: Configure SMBus/I2C selection and noise filter */
  ra_i2c_slave_putreg(priv, RA_I2C_ICMR3_OFFSET, I2C_ICMR3_NF_MASK); /* Enable noise filter */

  /* ICFER: Configure function enables */
  ra_i2c_slave_putreg(priv, RA_I2C_ICFER_OFFSET, 
                      I2C_ICFER_TMOE |    /* Enable timeout */
                      I2C_ICFER_SALE |    /* Enable slave arbitration-lost detection */
                      I2C_ICFER_NFE |     /* Enable digital noise filter */
                      I2C_ICFER_SCLE);    /* Enable SCL synchronous circuit */

  /* ICSER: Configure slave address detection - will be set by setaddress */
  ra_i2c_slave_putreg(priv, RA_I2C_ICSER_OFFSET, 0);

#ifndef CONFIG_I2C_POLLED
  /* Configure and enable interrupts for slave mode */
  ra_i2c_slave_putreg(priv, RA_I2C_ICIER_OFFSET,
                      I2C_ICIER_TIE |     /* Transmit data empty interrupt */
                      I2C_ICIER_TEIE |    /* Transmit end interrupt */
                      I2C_ICIER_RIE |     /* Receive data full interrupt */
                      I2C_ICIER_NAKIE |   /* NACK detection interrupt */
                      I2C_ICIER_SPIE |    /* Stop condition detection interrupt */
                      I2C_ICIER_STIE |    /* Start condition detection interrupt */
                      I2C_ICIER_ALIE |    /* Arbitration-lost detection interrupt */
                      I2C_ICIER_TMOIE);   /* Timeout detection interrupt */

  /* Attach interrupt handlers */
  irq_attach(config->irq_rxi, ra_i2c_slave_isr_rxi, priv);
  irq_attach(config->irq_txi, ra_i2c_slave_isr_txi, priv);
  irq_attach(config->irq_tei, ra_i2c_slave_isr_tei, priv);
  irq_attach(config->irq_eri, ra_i2c_slave_isr_eri, priv);

  /* Enable interrupts */
  up_enable_irq(config->irq_rxi);
  up_enable_irq(config->irq_txi);
  up_enable_irq(config->irq_tei);
  up_enable_irq(config->irq_eri);
#endif

  /* Enable I2C peripheral in slave mode */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICCR1_OFFSET, 0, I2C_ICCR1_ICE);

  /* Clear master mode bit to ensure slave mode */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICCR2_OFFSET, I2C_ICCR2_MST, 0);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_deinit
 *
 * Description:
 *   Deinitialize the I2C slave hardware
 *
 ****************************************************************************/

static int ra_i2c_slave_deinit(struct ra_i2c_slave_priv_s *priv)
{
  const struct ra_i2c_config_s *config = priv->config;
  uint32_t regval;

  /* Disable I2C peripheral */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICCR1_OFFSET, I2C_ICCR1_ICE, 0);

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

  /* Disable I2C module clock */
  regval = getreg32(0x40036038);  /* MSTPCRB register */
  regval |= config->mstpcrb_bit;
  putreg32(regval, 0x40036038);

  return OK;
}

#ifndef CONFIG_I2C_POLLED
/****************************************************************************
 * Name: ra_i2c_slave_isr_rxi
 *
 * Description:
 *   I2C slave RX interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_rxi(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Handle received data */
  if (priv->buffer && priv->nbytes < priv->buflen)
    {
      priv->buffer[priv->nbytes++] = ra_i2c_slave_getreg(priv, RA_I2C_ICDRR_OFFSET);
    }

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_READ);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_isr_txi
 *
 * Description:
 *   I2C slave TX interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_txi(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Send data if available */
  if (priv->buffer && priv->nbytes < priv->buflen)
    {
      ra_i2c_slave_putreg(priv, RA_I2C_ICDRT_OFFSET, priv->buffer[priv->nbytes++]);
    }

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_WRITE);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_isr_tei
 *
 * Description:
 *   I2C slave transfer end interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_tei(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_STOP);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_isr_eri
 *
 * Description:
 *   I2C slave error interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_eri(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;
  uint8_t sr2;

  DEBUGASSERT(priv != NULL);

  /* Read status to determine error type */
  sr2 = ra_i2c_slave_getreg(priv, RA_I2C_ICSR2_OFFSET);
  priv->status = sr2;

  /* Clear error flags */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICSR2_OFFSET, 
                        I2C_ICSR2_AL | I2C_ICSR2_TMOF | I2C_ICSR2_NACKF, 0);

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_ERROR);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_isr_start
 *
 * Description:
 *   I2C slave start condition interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_start(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Clear start flag */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_START, 0);

  /* Reset transfer state */
  priv->nbytes = 0;

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_START);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_isr_stop
 *
 * Description:
 *   I2C slave stop condition interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_stop(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  /* Clear stop flag */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICSR2_OFFSET, I2C_ICSR2_STOP, 0);

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_STOP);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}

/****************************************************************************
 * Name: ra_i2c_slave_isr_address
 *
 * Description:
 *   I2C slave address match interrupt service routine
 *
 ****************************************************************************/

static int ra_i2c_slave_isr_address(int irq, void *context, void *arg)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)arg;
  uint8_t sr1;

  DEBUGASSERT(priv != NULL);

  /* Read status to determine which address matched */
  sr1 = ra_i2c_slave_getreg(priv, RA_I2C_ICSR1_OFFSET);

  /* Clear address match flags */
  ra_i2c_slave_modifyreg(priv, RA_I2C_ICSR1_OFFSET, 
                        I2C_ICSR1_AAS0 | I2C_ICSR1_AAS1 | I2C_ICSR1_AAS2, 0);

  /* Call callback if registered */
  if (priv->callback)
    {
      priv->callback(priv->callback_arg, I2C_SLAVE_ADDRESS);
    }

  /* Signal semaphore to wake up waiting thread */
  nxsem_post(&priv->sem_isr);

  return OK;
}
#endif /* !CONFIG_I2C_POLLED */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_i2c_slave_initialize
 *
 * Description:
 *   Initialize the selected I2C port in slave mode. And return a unique 
 *   instance of struct i2c_slave_s.
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple I2C interfaces)
 *
 * Returned Value:
 *   Valid I2C slave device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_slave_s *ra_i2c_slave_initialize(int port)
{
  struct ra_i2c_slave_priv_s *priv = NULL;

  i2cinfo("I2C%d slave: Initialize\n", port);

  /* Get I2C slave private structure */
  switch (port)
    {
#ifdef CONFIG_RA8_I2C0_SLAVE
      case 0:
        priv = &ra_i2c0_slave_priv;
        break;
#endif

#ifdef CONFIG_RA8_I2C1_SLAVE
      case 1:
        priv = &ra_i2c1_slave_priv;
        break;
#endif

      default:
        i2cerr("I2C%d slave: Invalid port\n", port);
        return NULL;
    }

  /* Initialize the device structure */
  if (priv->refs++ == 0)
    {
      /* Initialize the I2C slave hardware */
      ra_i2c_slave_init(priv);
    }

  return (struct i2c_slave_s *)priv;
}

/****************************************************************************
 * Name: ra_i2c_slave_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port in slave mode, and power down the device.
 *
 * Input Parameters:
 *   Device structure as returned by ra_i2c_slave_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int ra_i2c_slave_uninitialize(struct i2c_slave_s *dev)
{
  struct ra_i2c_slave_priv_s *priv = (struct ra_i2c_slave_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  /* Decrement reference count and check if we should disable the peripheral */
  if (--priv->refs == 0)
    {
      /* Disable the I2C slave hardware */
      ra_i2c_slave_deinit(priv);
    }

  return OK;
}

#endif /* CONFIG_RA8_I2C_SLAVE */
