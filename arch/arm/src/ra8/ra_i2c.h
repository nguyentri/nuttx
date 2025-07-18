/****************************************************************************
 * arch/arm/src/ra8/ra_i2c.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_I2C_H
#define __ARCH_ARM_SRC_RA8_RA_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_slave.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* I2C interrupts */
#define RA_I2C_NEVENTS           8        /* Number of I2C events */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* I2C Device hardware configuration */
struct ra_i2c_config_s
{
  uint32_t base;          /* I2C base address */
  uint32_t clk_freq;      /* Clock frequency */
  uint8_t  bus;           /* I2C bus number */
  uint8_t  irq_rxi;       /* RX interrupt */
  uint8_t  irq_txi;       /* TX interrupt */
  uint8_t  irq_tei;       /* Transfer end interrupt */
  uint8_t  irq_eri;       /* Error interrupt */
  uint8_t  irq_start;     /* Start condition interrupt */
  uint8_t  irq_stop;      /* Stop condition interrupt */
  uint8_t  irq_nak;       /* NAK interrupt */
  uint8_t  irq_timeout;   /* Timeout interrupt */
  uint32_t mstpcrb_bit;   /* Module stop control bit */

  /* Pin configuration */
  uint32_t scl_pin;       /* SCL pin configuration */
  uint32_t sda_pin;       /* SDA pin configuration */

  /* DTC configuration */
  uint8_t  dtc_rx_ch;     /* DTC RX channel */
  uint8_t  dtc_tx_ch;     /* DTC TX channel */
  uint32_t dtc_rx_event;  /* DTC RX event */
  uint32_t dtc_tx_event;  /* DTC TX event */
};

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

/* I2C State Machine States */
enum ra_i2cstate_e
{
  I2CSTATE_IDLE = 0,      /* No I2C activity */
  I2CSTATE_START,         /* START condition sent */
  I2CSTATE_ADDR_WRITE,    /* Address sent, wait for ACK in write mode */
  I2CSTATE_ADDR_READ,     /* Address sent, wait for ACK in read mode */
  I2CSTATE_WRITE,         /* Transmitting data */
  I2CSTATE_READ,          /* Receiving data */
  I2CSTATE_STOP,          /* STOP condition sent */
  I2CSTATE_ERROR,         /* Error occurred */
  I2CSTATE_FINISH         /* Transfer finished */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

struct i2c_master_s *ra_i2cbus_initialize(int port);

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

int ra_i2cbus_uninitialize(struct i2c_master_s *dev);

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

struct i2c_slave_s *ra_i2c_slave_initialize(int port);

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

int ra_i2c_slave_uninitialize(struct i2c_slave_s *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8_RA_I2C_H */
