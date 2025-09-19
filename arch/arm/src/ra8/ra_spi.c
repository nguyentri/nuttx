/****************************************************************************
 * arch/arm/src/ra8/ra_spi.c
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
#include <nuttx/spi/spi.h>
#include <nuttx/power/pm.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra_icu.h"
#include "hardware/ra_spi.h"
#include "hardware/ra_dmac.h"
#include "hardware/ra_memorymap.h"
#include "hardware/ra_icu.h"
#include "hardware/ra_mstp.h"
#include "ra_spi.h"

#ifdef CONFIG_RA_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_SPI_INFO
#  define spi_dumpgpio(m) ra_dumpgpio(m)
#else
#  define spi_dumpgpio(m)
#endif

/* DMA timeout */
#define DMA_TIMEOUT_MS          1000

/* SPI timeout */
#define SPI_TIMEOUT_MS          1000

/* Maximum SPI frequency */
#define RA_SPI_MAX_FREQUENCY    10000000

/* Sensor-specific frequency limits */
#define ICM20948_MAX_FREQ       7000000   /* ICM-20948: =7MHz */
#define BMP388_MAX_FREQ         10000000  /* BMP388: =10MHz */

/* CS timing delays (in RSPCK cycles) */
#define CS_SETUP_DELAY          1
#define CS_HOLD_DELAY           1
#define CS_NEGATION_DELAY       1

/* DTC transfer modes */
#define DTC_MODE_NORMAL         0x00
#define DTC_MODE_REPEAT         0x01
#define DTC_MODE_BLOCK          0x02

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Chip Select Configuration */
struct ra_spi_cs_config_s
{
  uint32_t gpio_pin;        /* GPIO pin for CS control */
  bool     use_hardware;    /* Use hardware SS0 or GPIO */
  uint8_t  ssl_select;      /* SSL select value (0-3) */
  uint32_t max_frequency;   /* Maximum frequency for this device */
  uint8_t  setup_delay;     /* CS setup delay */
  uint8_t  hold_delay;      /* CS hold delay */
  uint8_t  negation_delay;  /* CS negation delay */
};

/* SPI Device hardware configuration */
struct ra_spi_config_s
{
  uint32_t base;          /* SPI peripheral base address */
  uint32_t clk_src;       /* Clock source */
  uint32_t clk_freq;      /* Clock frequency */
  uint8_t  bus;           /* SPI bus number */
  int  irq_rxi;           /* RX interrupt */
  int  irq_txi;           /* TX interrupt */
  int  irq_tei;           /* Transfer end interrupt */
  int  irq_eri;           /* Error interrupt */
  int  el_rxi;            /* Even Link for RX interrupt */
  int  el_txi;            /* Even Link for TX interrupt */
  int  el_tei;            /* Even Link for Transfer end interrupt */
  int  el_eri;            /* Even Link for Error interrupt */
  uint32_t mstpcrb_bit;   /* Module stop control bit */

  /* Pin configuration */
  uint32_t sck_pin;       /* SCK pin configuration */
  uint32_t miso_pin;      /* MISO pin configuration */
  uint32_t mosi_pin;      /* MOSI pin configuration */
  uint32_t cs0_pin;       /* CS0 pin configuration */
  uint32_t cs1_pin;       /* CS1 pin configuration */

  /* CS device configurations */
  struct ra_spi_cs_config_s cs_configs[4];  /* Up to 4 CS devices */
};

/* DTC Transfer Information */
struct ra_spi_dtc_info_s
{
  uint32_t mra;           /* Mode Register A */
  uint32_t mrb;           /* Mode Register B */
  uint32_t sar;           /* Source Address Register */
  uint32_t dar;           /* Destination Address Register */
  uint32_t cra;           /* Transfer Count Register A */
  uint32_t crb;           /* Transfer Count Register B */
};

/* SPI Device Private Data */
struct ra_spi_priv_s
{
  /* Externally visible part of the SPI interface */
  struct spi_dev_s         spidev;

  /* Port configuration */
  const struct ra_spi_config_s *config;

  int                      refs;       /* Reference count */
  mutex_t                  lock;       /* Held while chip is selected for mutual exclusion */
  uint32_t                 frequency;  /* Requested clock frequency */
  uint32_t                 actual;     /* Actual clock frequency */
  uint8_t                  mode;       /* Mode 0,1,2,3 */
  uint8_t                  nbits;      /* Width of word in bits (8 or 16) */
  uint32_t                 current_devid; /* Currently selected device ID */

  /* DTC channels and configuration */
  bool                     use_dtc;
  int                      dtc_tx;     /* TX DTC channel */
  int                      dtc_rx;     /* RX DTC channel */
  struct ra_spi_dtc_info_s dtc_tx_info; /* TX DTC transfer info */
  struct ra_spi_dtc_info_s dtc_rx_info; /* RX DTC transfer info */

  /* Transfer state */
  sem_t                    waitsem;    /* Wait for transfer completion */
  const void              *txbuffer;   /* Source data */
  void                    *rxbuffer;   /* Destination data */
  size_t                   ntxwords;   /* Number of words to transfer */
  size_t                   nrxwords;   /* Number of words to receive */
  bool                     error;      /* Transfer error flag */
  volatile bool            dtc_active; /* DTC transfer in progress */

#ifdef CONFIG_PM
  struct pm_callback_s     pmcb;       /* PM callbacks */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Low-level helpers */
static void ra_spi_putreg8(struct ra_spi_priv_s *priv, uint8_t offset, uint8_t value);
static void ra_spi_putreg16(struct ra_spi_priv_s *priv, uint8_t offset, uint16_t value);
static void ra_spi_putreg32(struct ra_spi_priv_s *priv, uint8_t offset, uint32_t value);
static uint8_t ra_spi_getreg8(struct ra_spi_priv_s *priv, uint8_t offset);
static uint16_t ra_spi_getreg16(struct ra_spi_priv_s *priv, uint8_t offset);
static uint32_t ra_spi_getreg32(struct ra_spi_priv_s *priv, uint8_t offset);

/* CS control functions */
static void ra_spi_cs_assert(struct ra_spi_priv_s *priv, uint32_t devid);
static void ra_spi_cs_deassert(struct ra_spi_priv_s *priv, uint32_t devid);
static void ra_spi_cs_configure(struct ra_spi_priv_s *priv, uint32_t devid);
static uint32_t ra_spi_get_device_max_freq(uint32_t devid);

/* DTC support */
static int ra_spi_dtc_setup(struct ra_spi_priv_s *priv);
static void ra_spi_dtc_start(struct ra_spi_priv_s *priv);
static void ra_spi_dtc_stop(struct ra_spi_priv_s *priv);
static int ra_spi_dtc_configure_transfer(struct ra_spi_priv_s *priv,
                                         const void *txbuffer, void *rxbuffer,
                                         size_t nwords);

/* Transfer helpers */
static void ra_spi_writeword(struct ra_spi_priv_s *priv, uint16_t word);
static uint16_t ra_spi_readword(struct ra_spi_priv_s *priv);
static bool ra_spi_9to16bitmode(struct ra_spi_priv_s *priv);

/* Interrupt handling */
static int ra_spi_rxi_interrupt(int irq, void *context, void *arg);
static int ra_spi_txi_interrupt(int irq, void *context, void *arg);
static int ra_spi_tei_interrupt(int irq, void *context, void *arg);
static int ra_spi_eri_interrupt(int irq, void *context, void *arg);

/* SPI methods */
static int ra_spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t ra_spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void ra_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void ra_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int ra_spi_hwfeatures(struct spi_dev_s *dev,
                             spi_hwfeatures_t features);
#endif
static uint32_t ra_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void ra_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                           void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void ra_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                           size_t nwords);
static void ra_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                            size_t nwords);
#endif
#ifdef CONFIG_SPI_TRIGGER
static int ra_spi_trigger(struct spi_dev_s *dev);
#endif

/* Initialization */
static void ra_spi_bus_initialize(struct ra_spi_priv_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RA_SPI0
static const struct ra_spi_config_s ra_spi0_config =
{
  .base        = R_SPI0_BASE,
  .clk_src     = RA_PCKB_FREQUENCY,
  .clk_freq    = RA_PCKB_FREQUENCY,
  .bus         = 0,

  .irq_rxi     = -1, // Will be assigned by ICU
  .irq_txi     = -1, // Will be assigned by ICU
  .irq_tei     = -1, // Will be assigned by ICU
  .irq_eri     = -1, // Will be assigned by ICU

  .el_rxi       = ELC_EVENT_SPI0_RXI,
  .el_txi       = ELC_EVENT_SPI0_TXI,
  .el_tei       = ELC_EVENT_SPI0_TEI,
  .el_eri       = ELC_EVENT_SPI0_ERI,

  .mstpcrb_bit = R_MSTP_MSTPCRB_SPI0,

  /* Pin configuration for FPB-RA8E1 */
  .sck_pin     = PIN_SPI0_SCK,     /* P611 */
  .miso_pin    = PIN_SPI0_MISO,    /* P610 */
  .mosi_pin    = PIN_SPI0_MOSI,    /* P609 */
  .cs0_pin     = PIN_SPI0_CS0,     /* P612 - Hardware SS0 */
  .cs1_pin     = PIN_SPI0_CS1,     /* P605 - GPIO CS */

  /* CS device configurations */
  .cs_configs = {
    /* CS0: ICM-20948 IMU - Hardware SS0 */
    {
      .gpio_pin = PIN_SPI0_CS0,
      .use_hardware = true,
      .ssl_select = 0,
      .max_frequency = ICM20948_MAX_FREQ,
      .setup_delay = CS_SETUP_DELAY,
      .hold_delay = CS_HOLD_DELAY,
      .negation_delay = CS_NEGATION_DELAY,
    },
    /* CS1: BMP388 Barometer - GPIO CS */
    {
      .gpio_pin = PIN_SPI0_CS1,
      .use_hardware = false,
      .ssl_select = 1,
      .max_frequency = BMP388_MAX_FREQ,
      .setup_delay = CS_SETUP_DELAY,
      .hold_delay = CS_HOLD_DELAY,
      .negation_delay = CS_NEGATION_DELAY,
    },
    /* CS2: Reserved */
    {
      .gpio_pin = 0,
      .use_hardware = false,
      .ssl_select = 2,
      .max_frequency = RA_SPI_MAX_FREQUENCY,
      .setup_delay = CS_SETUP_DELAY,
      .hold_delay = CS_HOLD_DELAY,
      .negation_delay = CS_NEGATION_DELAY,
    },
    /* CS3: Reserved */
    {
      .gpio_pin = 0,
      .use_hardware = false,
      .ssl_select = 3,
      .max_frequency = RA_SPI_MAX_FREQUENCY,
      .setup_delay = CS_SETUP_DELAY,
      .hold_delay = CS_HOLD_DELAY,
      .negation_delay = CS_NEGATION_DELAY,
    },
  },
};

static struct ra_spi_priv_s ra_spi0_priv =
{
  .spidev   =
    {
      .ops    = &ra_spi_ops,
    },
  .config   = &ra_spi0_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .waitsem  = SEM_INITIALIZER(0),
  .current_devid = 0xffffffff,
};
#endif

#ifdef CONFIG_RA_SPI1
static const struct ra_spi_config_s ra_spi1_config =
{
  .base        = R_SPI1_BASE,
  .clk_src     = RA_PCKB_FREQUENCY,
  .clk_freq    = RA_PCKB_FREQUENCY,
  .bus         = 1,
  .irq_rxi     = R_ICU_VECT(EVENT_SPI1_RXI),
  .irq_txi     = R_ICU_VECT(EVENT_SPI1_TXI),
  .irq_tei     = R_ICU_VECT(EVENT_SPI1_TEI),
  .irq_eri     = R_ICU_VECT(EVENT_SPI1_ERI),
  .mstpcrb_bit = R_MSTP_MSTPCRB_SPI1,

  /* Pin configuration */
  .sck_pin     = PIN_SPI1_SCK,
  .miso_pin    = PIN_SPI1_MISO,
  .mosi_pin    = PIN_SPI1_MOSI,
  .cs0_pin     = PIN_SPI1_CS0,
  .cs1_pin     = PIN_SPI1_CS1,

  /* CS device configurations */
  .cs_configs = {
    {
      .gpio_pin = PIN_SPI1_CS0,
      .use_hardware = true,
      .ssl_select = 0,
      .max_frequency = RA_SPI_MAX_FREQUENCY,
      .setup_delay = CS_SETUP_DELAY,
      .hold_delay = CS_HOLD_DELAY,
      .negation_delay = CS_NEGATION_DELAY,
    },
    {
      .gpio_pin = PIN_SPI1_CS1,
      .use_hardware = false,
      .ssl_select = 1,
      .max_frequency = RA_SPI_MAX_FREQUENCY,
      .setup_delay = CS_SETUP_DELAY,
      .hold_delay = CS_HOLD_DELAY,
      .negation_delay = CS_NEGATION_DELAY,
    },
    {0}, {0}, /* Reserved CS2, CS3 */
  },
};

static struct ra_spi_priv_s ra_spi1_priv =
{
  .spidev   =
    {
      .ops    = &ra_spi_ops,
    },
  .config   = &ra_spi1_config,
  .refs     = 0,
  .lock     = NXMUTEX_INITIALIZER,
  .waitsem  = SEM_INITIALIZER(0),
  .current_devid = 0xffffffff,
};
#endif

/* SPI operations */
static const struct spi_ops_s ra_spi_ops =
{
  .lock              = ra_spi_lock,
  .select            = ra_spi_select,           /* Provided externally */
  .setfrequency      = ra_spi_setfrequency,
  .setmode           = ra_spi_setmode,
  .setbits           = ra_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = ra_spi_hwfeatures,
#endif
  .status            = ra_spi_status,           /* Provided externally */
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = ra_spi_cmddata,          /* Provided externally */
#endif
  .send              = ra_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = ra_spi_exchange,
#else
  .sndblock          = ra_spi_sndblock,
  .recvblock         = ra_spi_recvblock,
#endif
#ifdef CONFIG_SPI_TRIGGER
  .trigger           = ra_spi_trigger,
#endif
#ifdef CONFIG_SPI_CALLBACK
  .registercallback  = ra_spi_register_callback, /* Provided externally */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spi_putreg8/16/32
 *
 * Description:
 *   Put a 8/16/32-bit register value by offset
 *
 ****************************************************************************/

static void ra_spi_putreg8(struct ra_spi_priv_s *priv, uint8_t offset,
                           uint8_t value)
{
  putreg8(value, priv->config->base + offset);
}

static void ra_spi_putreg16(struct ra_spi_priv_s *priv, uint8_t offset,
                           uint16_t value)
{
  putreg16(value, priv->config->base + offset);
}

static void ra_spi_putreg32(struct ra_spi_priv_s *priv, uint8_t offset,
                           uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_spi_getreg8/16/32
 *
 * Description:
 *   Get a 8/16/32-bit register value by offset
 *
 ****************************************************************************/

static uint8_t ra_spi_getreg8(struct ra_spi_priv_s *priv, uint8_t offset)
{
  return getreg8(priv->config->base + offset);
}

static uint16_t ra_spi_getreg16(struct ra_spi_priv_s *priv, uint8_t offset)
{
  return getreg16(priv->config->base + offset);
}

static uint32_t ra_spi_getreg32(struct ra_spi_priv_s *priv, uint8_t offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_spi_get_device_max_freq
 *
 * Description:
 *   Get maximum frequency for a specific device
 *
 ****************************************************************************/

static uint32_t ra_spi_get_device_max_freq(uint32_t devid)
{
  switch (devid)
    {
      case SPIDEV_IMU(0):
        return ICM20948_MAX_FREQ;
      case SPIDEV_BAROMETER(0):
        return BMP388_MAX_FREQ;
      default:
        return RA_SPI_MAX_FREQUENCY;
    }
}

/****************************************************************************
 * Name: ra_spi_cs_configure
 *
 * Description:
 *   Configure CS timing and polarity for a specific device
 *
 ****************************************************************************/

static void ra_spi_cs_configure(struct ra_spi_priv_s *priv, uint32_t devid)
{
  const struct ra_spi_cs_config_s *cs_config = NULL;
  uint16_t spcmd;
  uint8_t cs_index;

  /* Determine CS index from device ID */
  switch (devid)
    {
      case SPIDEV_IMU(0):
        cs_index = 0;
        break;
      case SPIDEV_BAROMETER(0):
        cs_index = 1;
        break;
      default:
        cs_index = 0;
        break;
    }

  cs_config = &priv->config->cs_configs[cs_index];

  /* Configure SPCMD register for this CS */
  spcmd = ra_spi_getreg16(priv, RA_SPI_SPCMD0_OFFSET);
  
  /* Clear SSL selection bits */
  spcmd &= ~RA_SPI_SPCMD_SSLA_MASK;
  
  /* Set SSL selection */
  spcmd |= (cs_config->ssl_select << RA_SPI_SPCMD_SSLA_SHIFT) & RA_SPI_SPCMD_SSLA_MASK;
  
  /* Enable timing delays */
  spcmd |= RA_SPI_SPCMD_SCKDEN | RA_SPI_SPCMD_SLNDEN | RA_SPI_SPCMD_SPNDEN;
  
  ra_spi_putreg16(priv, RA_SPI_SPCMD0_OFFSET, spcmd);

  /* Configure timing delays */
  ra_spi_putreg8(priv, RA_SPI_SPCKD_OFFSET, cs_config->setup_delay);
  ra_spi_putreg8(priv, RA_SPI_SSLND_OFFSET, cs_config->negation_delay);
  ra_spi_putreg8(priv, RA_SPI_SPND_OFFSET, cs_config->hold_delay);

  spiinfo("SPI%d CS%d configured: SSL=%d, delays=%d/%d/%d\n",
          priv->config->bus, cs_index, cs_config->ssl_select,
          cs_config->setup_delay, cs_config->hold_delay, cs_config->negation_delay);
}

/****************************************************************************
 * Name: ra_spi_cs_assert
 *
 * Description:
 *   Assert chip select for a specific device
 *
 ****************************************************************************/

static void ra_spi_cs_assert(struct ra_spi_priv_s *priv, uint32_t devid)
{
  const struct ra_spi_cs_config_s *cs_config = NULL;
  uint8_t cs_index;

  /* Determine CS index from device ID */
  switch (devid)
    {
      case SPIDEV_IMU(0):
        cs_index = 0;
        break;
      case SPIDEV_BAROMETER(0):
        cs_index = 1;
        break;
      default:
        cs_index = 0;
        break;
    }

  cs_config = &priv->config->cs_configs[cs_index];

  if (cs_config->use_hardware)
    {
      /* Hardware SS0 - controlled by SPI peripheral */
      spiinfo("SPI%d Hardware CS%d asserted\n", priv->config->bus, cs_index);
    }
  else
    {
      /* GPIO CS - manually control */
      ra_gpio_write(cs_config->gpio_pin, false); /* Active low */
      spiinfo("SPI%d GPIO CS%d asserted\n", priv->config->bus, cs_index);
    }
}

/****************************************************************************
 * Name: ra_spi_cs_deassert
 *
 * Description:
 *   Deassert chip select for a specific device
 *
 ****************************************************************************/

static void ra_spi_cs_deassert(struct ra_spi_priv_s *priv, uint32_t devid)
{
  const struct ra_spi_cs_config_s *cs_config = NULL;
  uint8_t cs_index;

  /* Determine CS index from device ID */
  switch (devid)
    {
      case SPIDEV_IMU(0):
        cs_index = 0;
        break;
      case SPIDEV_BAROMETER(0):
        cs_index = 1;
        break;
      default:
        cs_index = 0;
        break;
    }

  cs_config = &priv->config->cs_configs[cs_index];

  if (cs_config->use_hardware)
    {
      /* Hardware SS0 - controlled by SPI peripheral */
      spiinfo("SPI%d Hardware CS%d deasserted\n", priv->config->bus, cs_index);
    }
  else
    {
      /* GPIO CS - manually control */
      ra_gpio_write(cs_config->gpio_pin, true); /* Active low */
      spiinfo("SPI%d GPIO CS%d deasserted\n", priv->config->bus, cs_index);
    }
}

/****************************************************************************
 * Name: ra_spi_writeword
 *
 * Description:
 *   Write one word to SPI
 *
 ****************************************************************************/

static void ra_spi_writeword(struct ra_spi_priv_s *priv, uint16_t word)
{
  /* Wait until the transmit buffer is empty */
  while ((ra_spi_getreg8(priv, RA_SPI_SPSR_OFFSET) & RA_SPI_SPSR_SPTEF) == 0);

  /* Write the data */
  if (ra_spi_9to16bitmode(priv))
    {
      ra_spi_putreg16(priv, RA_SPI_SPDR_OFFSET, word);
    }
  else
    {
      ra_spi_putreg8(priv, RA_SPI_SPDR_OFFSET, (uint8_t)word);
    }
}

/****************************************************************************
 * Name: ra_spi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 ****************************************************************************/

static uint16_t ra_spi_readword(struct ra_spi_priv_s *priv)
{
  /* Wait until receive buffer is full */
  while ((ra_spi_getreg8(priv, RA_SPI_SPSR_OFFSET) & RA_SPI_SPSR_SPRF) == 0);

  /* Read the data */
  if (ra_spi_9to16bitmode(priv))
    {
      return ra_spi_getreg16(priv, RA_SPI_SPDR_OFFSET);
    }
  else
    {
      return (uint16_t)ra_spi_getreg8(priv, RA_SPI_SPDR_OFFSET);
    }
}

/****************************************************************************
 * Name: ra_spi_9to16bitmode
 *
 * Description:
 *   Check if the current transfer is more than 8 bits
 *
 ****************************************************************************/

static bool ra_spi_9to16bitmode(struct ra_spi_priv_s *priv)
{
  return (priv->nbits > 8);
}

/****************************************************************************
 * Name: ra_spi_dtc_setup
 *
 * Description:
 *   Setup DTC for the SPI
 *
 ****************************************************************************/

static int ra_spi_dtc_setup(struct ra_spi_priv_s *priv)
{
  spiinfo("DTC setup for SPI%d\n", priv->config->bus);

  /* Enable DTC module clock */
  uint32_t regval = getreg32(R_MSTP_MSTPCRB);
  regval &= ~R_MSTP_MSTPCRB_DTC;
  putreg32(regval, R_MSTP_MSTPCRB);

  /* Initialize DTC control register */
  putreg32(0x00000001, R_DTC_BASE + R_DTC_DTCST_OFFSET);

  /* Configure DTC vector base register */
  putreg32((uint32_t)&priv->dtc_tx_info, R_DTC_BASE + R_DTC_DTCVBR_OFFSET);

  priv->use_dtc = true;
  
  spiinfo("DTC setup completed for SPI%d\n", priv->config->bus);
  return OK;
}

/****************************************************************************
 * Name: ra_spi_dtc_configure_transfer
 *
 * Description:
 *   Configure DTC transfer information
 *
 ****************************************************************************/

static int ra_spi_dtc_configure_transfer(struct ra_spi_priv_s *priv,
                                         const void *txbuffer, void *rxbuffer,
                                         size_t nwords)
{
  /* Configure TX DTC */
  if (txbuffer)
    {
      priv->dtc_tx_info.mra = DTC_MODE_NORMAL | 
                              (ra_spi_9to16bitmode(priv) ? 
                               RA_DTC_MRA_SZ_WORD : RA_DTC_MRA_SZ_BYTE);
      priv->dtc_tx_info.mrb = 0;
      priv->dtc_tx_info.sar = (uint32_t)txbuffer;
      priv->dtc_tx_info.dar = priv->config->base + RA_SPI_SPDR_OFFSET;
      priv->dtc_tx_info.cra = nwords;
      priv->dtc_tx_info.crb = 0;
    }

  /* Configure RX DTC */
  if (rxbuffer)
    {
      priv->dtc_rx_info.mra = DTC_MODE_NORMAL | 
                              (ra_spi_9to16bitmode(priv) ? 
                               RA_DTC_MRA_SZ_WORD : RA_DTC_MRA_SZ_BYTE);
      priv->dtc_rx_info.mrb = 0;
      priv->dtc_rx_info.sar = priv->config->base + RA_SPI_SPDR_OFFSET;
      priv->dtc_rx_info.dar = (uint32_t)rxbuffer;
      priv->dtc_rx_info.cra = nwords;
      priv->dtc_rx_info.crb = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_spi_dtc_start
 *
 * Description:
 *   Start DTC transfer
 *
 ****************************************************************************/

static void ra_spi_dtc_start(struct ra_spi_priv_s *priv)
{
  uint8_t spcr;

  spiinfo("DTC start for SPI%d\n", priv->config->bus);

  priv->dtc_active = true;

  /* Enable DTC triggers for SPI TXI and RXI */
  spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
  spcr |= (RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE);
  ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);
}

/****************************************************************************
 * Name: ra_spi_dtc_stop
 *
 * Description:
 *   Stop DTC transfer
 *
 ****************************************************************************/

static void ra_spi_dtc_stop(struct ra_spi_priv_s *priv)
{
  uint8_t spcr;

  spiinfo("DTC stop for SPI%d\n", priv->config->bus);

  /* Disable DTC triggers */
  spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
  spcr &= ~(RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE);
  ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);

  priv->dtc_active = false;
}

/****************************************************************************
 * Name: ra_spi_rxi_interrupt
 *
 * Description:
 *   RX interrupt handler
 *
 ****************************************************************************/

static int ra_spi_rxi_interrupt(int irq, void *context, void *arg)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)arg;
  uint16_t data;

  DEBUGASSERT(priv != NULL);

  if (priv->dtc_active)
    {
      /* DTC is handling the transfer, just check for completion */
      if (priv->dtc_rx_info.cra == 0)
        {
          /* DTC transfer complete */
          ra_spi_dtc_stop(priv);
          nxsem_post(&priv->waitsem);
        }
      return OK;
    }

  /* Read received data */
  data = ra_spi_readword(priv);

  if (priv->rxbuffer)
    {
      if (ra_spi_9to16bitmode(priv))
        {
          *((uint16_t *)priv->rxbuffer)++ = data;
        }
      else
        {
          *((uint8_t *)priv->rxbuffer)++ = (uint8_t)data;
        }
    }

  if (priv->nrxwords > 0)
    {
      priv->nrxwords--;
    }

  /* Check if transfer is complete */
  if (priv->nrxwords == 0)
    {
      /* Disable RX interrupt */
      uint8_t spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~RA_SPI_SPCR_SPRIE;
      ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);

      /* Wake up waiting thread */
      nxsem_post(&priv->waitsem);
    }

  return OK;
}

/****************************************************************************
 * Name: ra_spi_txi_interrupt
 *
 * Description:
 *   TX interrupt handler
 *
 ****************************************************************************/

static int ra_spi_txi_interrupt(int irq, void *context, void *arg)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)arg;
  uint16_t data = 0xffff;

  DEBUGASSERT(priv != NULL);

  if (priv->dtc_active)
    {
      /* DTC is handling the transfer, just check for completion */
      if (priv->dtc_tx_info.cra == 0)
        {
          /* DTC transfer complete */
          uint8_t spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
          spcr &= ~RA_SPI_SPCR_SPTIE;
          ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);
        }
      return OK;
    }

  /* Get next word to transmit */
  if (priv->txbuffer && priv->ntxwords > 0)
    {
      if (ra_spi_9to16bitmode(priv))
        {
          data = *((uint16_t *)priv->txbuffer)++;
        }
      else
        {
          data = (uint16_t)*((uint8_t *)priv->txbuffer)++;
        }
      priv->ntxwords--;
    }
  else
    {
      priv->ntxwords = 0;
    }

  /* Write data */
  ra_spi_writeword(priv, data);

  /* Check if transmission is complete */
  if (priv->ntxwords == 0)
    {
      /* Disable TX interrupt */
      uint8_t spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~RA_SPI_SPCR_SPTIE;
      ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);
    }

  return OK;
}

/****************************************************************************
 * Name: ra_spi_tei_interrupt
 *
 * Description:
 *   Transfer end interrupt handler
 *
 ****************************************************************************/

static int ra_spi_tei_interrupt(int irq, void *context, void *arg)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)arg;

  DEBUGASSERT(priv != NULL);

  spiinfo("SPI%d transfer end interrupt\n", priv->config->bus);

  return OK;
}

/****************************************************************************
 * Name: ra_spi_eri_interrupt
 *
 * Description:
 *   Error interrupt handler
 *
 ****************************************************************************/

static int ra_spi_eri_interrupt(int irq, void *context, void *arg)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)arg;
  uint8_t spsr;

  DEBUGASSERT(priv != NULL);

  spsr = ra_spi_getreg8(priv, RA_SPI_SPSR_OFFSET);

  spierr("SPI%d error interrupt: SPSR=%02x\n", priv->config->bus, spsr);

  if (spsr & RA_SPI_SPSR_OVRF)
    {
      spierr("SPI%d overrun error\n", priv->config->bus);
    }

  if (spsr & RA_SPI_SPSR_MODF)
    {
      spierr("SPI%d mode fault error\n", priv->config->bus);
    }

  if (spsr & RA_SPI_SPSR_PERF)
    {
      spierr("SPI%d parity error\n", priv->config->bus);
    }

  if (spsr & RA_SPI_SPSR_UDRF)
    {
      spierr("SPI%d underrun error\n", priv->config->bus);
    }

  /* Set error flag and wake up waiting thread */
  priv->error = true;
  if (priv->dtc_active)
    {
      ra_spi_dtc_stop(priv);
    }
  nxsem_post(&priv->waitsem);

  return OK;
}

/****************************************************************************
 * Name: ra_spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI buss is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int ra_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxmutex_lock(&priv->lock);
    }
  else
    {
      ret = nxmutex_unlock(&priv->lock);
    }

  return ret;
}

/****************************************************************************
 * Name: ra_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency with device-specific limits
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t ra_spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  uint32_t src_clk = priv->config->clk_freq;
  uint32_t divisor;
  uint8_t spbr;
  uint8_t brdv = 0;
  uint32_t max_freq;

  spiinfo("SPI%d frequency %d\n", priv->config->bus, frequency);

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */
      return priv->actual;
    }

  /* Apply device-specific frequency limits */
  max_freq = ra_spi_get_device_max_freq(priv->current_devid);
  if (frequency > max_freq)
    {
      frequency = max_freq;
      spiinfo("SPI%d frequency limited to %d for device 0x%08x\n",
              priv->config->bus, frequency, priv->current_devid);
    }

  /* Limit to maximum frequency */
  if (frequency > RA_SPI_MAX_FREQUENCY)
    {
      frequency = RA_SPI_MAX_FREQUENCY;
    }

  /* Calculate the best divisor */
  /* The SPI bit rate is calculated as:
   * Bit rate = PCKB / (2 * (SPBR + 1) * 2^BRDV)
   */

  divisor = (src_clk + frequency - 1) / frequency;

  /* Find best BRDV and SPBR combination */
  for (brdv = 0; brdv < 4; brdv++)
    {
      uint32_t div_factor = 2 << brdv;  /* 2^(brdv+1) */
      uint32_t spbr_calc = (divisor + div_factor - 1) / (2 * div_factor) - 1;

      if (spbr_calc <= 255)
        {
          spbr = (uint8_t)spbr_calc;
          break;
        }
    }

  if (brdv >= 4)
    {
      /* Use maximum divisor */
      brdv = 3;
      spbr = 255;
    }

  /* Calculate actual frequency */
  priv->actual = src_clk / (2 * (spbr + 1) * (2 << brdv));
  priv->frequency = frequency;

  spiinfo("SPI%d SPBR=%d BRDV=%d actual=%d\n",
          priv->config->bus, spbr, brdv, priv->actual);

  /* Set the bit rate register */
  ra_spi_putreg8(priv, RA_SPI_SPBR_OFFSET, spbr);

  /* Update SPCMD0 with new BRDV */
  uint16_t spcmd0 = ra_spi_getreg16(priv, RA_SPI_SPCMD0_OFFSET);
  spcmd0 &= ~RA_SPI_SPCMD_BRDV_MASK;
  spcmd0 |= (brdv << RA_SPI_SPCMD_BRDV_SHIFT);
  ra_spi_putreg16(priv, RA_SPI_SPCMD0_OFFSET, spcmd0);

  return priv->actual;
}

/****************************************************************************
 * Name: ra_spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void ra_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  uint16_t spcmd0;

  spiinfo("SPI%d mode %d\n", priv->config->bus, mode);

  if (priv->mode == mode)
    {
      /* We are already in this mode. */
      return;
    }

  spcmd0 = ra_spi_getreg16(priv, RA_SPI_SPCMD0_OFFSET);
  spcmd0 &= ~(RA_SPI_SPCMD_CPOL | RA_SPI_SPCMD_CPHA);

  switch (mode)
    {
      case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
        break;

      case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
        spcmd0 |= RA_SPI_SPCMD_CPHA;
        break;

      case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
        spcmd0 |= RA_SPI_SPCMD_CPOL;
        break;

      case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
        spcmd0 |= (RA_SPI_SPCMD_CPOL | RA_SPI_SPCMD_CPHA);
        break;

      default:
        spierr("SPI%d bad mode %d\n", priv->config->bus, mode);
        DEBUGASSERT(false);
        return;
    }

  ra_spi_putreg16(priv, RA_SPI_SPCMD0_OFFSET, spcmd0);
  priv->mode = mode;
}

/****************************************************************************
 * Name: ra_spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits per word
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  uint16_t spcmd0;
  uint16_t spb_bits;

  spiinfo("SPI%d nbits %d\n", priv->config->bus, nbits);

  if (priv->nbits == nbits)
    {
      /* We are already at this bit width */
      return;
    }

  /* Convert nbits to SPB field value */
  switch (nbits)
    {
      case 4:
        spb_bits = RA_SPI_SPCMD_SPB_4;
        break;
      case 8:
        spb_bits = RA_SPI_SPCMD_SPB_8;
        break;
      case 16:
        spb_bits = RA_SPI_SPCMD_SPB_16;
        break;
      case 20:
        spb_bits = RA_SPI_SPCMD_SPB_20;
        break;
      case 24:
        spb_bits = RA_SPI_SPCMD_SPB_24;
        break;
      case 32:
        spb_bits = RA_SPI_SPCMD_SPB_32;
        break;
      default:
        spierr("SPI%d bad nbits %d\n", priv->config->bus, nbits);
        return;
    }

  spcmd0 = ra_spi_getreg16(priv, RA_SPI_SPCMD0_OFFSET);
  spcmd0 &= ~RA_SPI_SPCMD_SPB_MASK;
  spcmd0 |= spb_bits;
  ra_spi_putreg16(priv, RA_SPI_SPCMD0_OFFSET, spcmd0);

  priv->nbits = nbits;
}

#ifdef CONFIG_SPI_HWFEATURES
/****************************************************************************
 * Name: ra_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

static int ra_spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;

  spiinfo("SPI%d features %08x\n", priv->config->bus, features);

  /* Other H/W features are not supported */
  return ((features & ~HWFEAT_FORCE_CS) == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: ra_spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t ra_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  uint16_t ret;

  DEBUGASSERT(priv != NULL);

  /* Write the word to transmit */
  ra_spi_writeword(priv, (uint16_t)wd);

  /* Read the received word */
  ret = ra_spi_readword(priv);

  spiinfo("SPI%d sent %08x received %04x\n",
          priv->config->bus, wd, ret);

  return (uint32_t)ret;
}

/****************************************************************************
 * Name: ra_spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI with enhanced DTC support
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                           void *rxbuffer, size_t nwords)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  irqstate_t flags;
  uint8_t spcr;

  DEBUGASSERT(priv != NULL);

  spiinfo("SPI%d txbuffer=%p rxbuffer=%p nwords=%zu\n",
          priv->config->bus, txbuffer, rxbuffer, nwords);

  if (nwords == 0)
    {
      return;
    }

  /* Setup the transfer */
  priv->txbuffer = txbuffer;
  priv->rxbuffer = rxbuffer;
  priv->ntxwords = nwords;
  priv->nrxwords = nwords;
  priv->error = false;

  /* Use DTC if enabled and transfer size is large enough */
  if (priv->use_dtc && nwords >= 4)
    {
      /* Configure DTC transfer */
      ra_spi_dtc_configure_transfer(priv, txbuffer, rxbuffer, nwords);
      
      /* Start DTC transfer */
      ra_spi_dtc_start(priv);

      /* Wait for DTC completion */
      nxsem_wait_uninterruptible(&priv->waitsem);

      /* Stop DTC */
      ra_spi_dtc_stop(priv);
    }
  else
    {
      /* Use interrupt-driven transfer */
      flags = enter_critical_section();

      /* Enable interrupts */
      spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
      spcr |= (RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE | RA_SPI_SPCR_SPEIE);
      ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);

      leave_critical_section(flags);

      /* Wait for transfer completion */
      nxsem_wait_uninterruptible(&priv->waitsem);

      /* Disable interrupts */
      flags = enter_critical_section();

      spcr = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~(RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE | RA_SPI_SPCR_SPEIE);
      ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, spcr);

      leave_critical_section(flags);
    }

  if (priv->error)
    {
      spierr("SPI%d transfer error\n", priv->config->bus);
    }
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: ra_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                           size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%zu\n", txbuffer, nwords);
  return ra_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: ra_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                            size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%zu\n", rxbuffer, nwords);
  return ra_spi_exchange(dev, NULL, rxbuffer, nwords);
}

#endif /* !CONFIG_SPI_EXCHANGE */

#ifdef CONFIG_SPI_TRIGGER
/****************************************************************************
 * Name: ra_spi_trigger
 *
 * Description:
 *   Trigger a previously configured DMA transfer.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *
 * Returned Value:
 *   OK       - Trigger was fired
 *   -ENOSYS  - Trigger not fired due to lack of DMA or low level support
 *   -EIO     - Trigger not fired because not previously primed
 *
 ****************************************************************************/

static int ra_spi_trigger(struct spi_dev_s *dev)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;

  if (!priv->use_dtc)
    {
      return -ENOSYS;
    }

  /* TODO: Implement trigger */
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: ra_spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_spi_bus_initialize(struct ra_spi_priv_s *priv)
{
  uint8_t regval;
  uint16_t regval16;
  int i;

  /* Configure GPIO pins for SPI */
  ra_gpio_config(priv->config->sck_pin);
  ra_gpio_config(priv->config->miso_pin);
  ra_gpio_config(priv->config->mosi_pin);
  ra_gpio_config(priv->config->cs0_pin);
  ra_gpio_config(priv->config->cs1_pin);

  /* Initialize CS pins */
  for (i = 0; i < 4; i++)
    {
      if (priv->config->cs_configs[i].gpio_pin != 0)
        {
          if (!priv->config->cs_configs[i].use_hardware)
            {
              /* Configure as GPIO output, initially deasserted (high) */
              ra_gpio_config(priv->config->cs_configs[i].gpio_pin);
              ra_gpio_write(priv->config->cs_configs[i].gpio_pin, true);
            }
        }
    }

  /* Enable SPI module */
  regval = getreg32(R_MSTP_MSTPCRB);
  regval &= ~priv->config->mstpcrb_bit;
  putreg32(regval, R_MSTP_MSTPCRB);

  /* Disable SPI function */
  ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, 0);

  /* Configure as master mode */
  regval = RA_SPI_SPCR_MSTR | RA_SPI_SPCR_MODFEN;
  ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, regval);

  /* Set SSL polarity (active low) */
  ra_spi_putreg8(priv, RA_SPI_SSLP_OFFSET, 0x00);

  /* Configure pin control register */
  ra_spi_putreg8(priv, RA_SPI_SPPCR_OFFSET, 0);

  /* Set sequence length to 1 (using SPCMD0 only) */
  ra_spi_putreg8(priv, RA_SPI_SPSCR_OFFSET, 0);

  /* Configure data control register for 32-bit access */
  ra_spi_putreg8(priv, RA_SPI_SPDCR_OFFSET, 0);

  /* Set clock delays to 1 RSPCK */
  ra_spi_putreg8(priv, RA_SPI_SPCKD_OFFSET, CS_SETUP_DELAY);
  ra_spi_putreg8(priv, RA_SPI_SSLND_OFFSET, CS_NEGATION_DELAY);
  ra_spi_putreg8(priv, RA_SPI_SPND_OFFSET, CS_HOLD_DELAY);

  /* Configure SPCR2 */
  ra_spi_putreg8(priv, RA_SPI_SPCR2_OFFSET, 0);

  /* Configure SPCMD0 for default settings */
  regval16 = RA_SPI_SPCMD_SPB_8        |  /* 8-bit data */
             RA_SPI_SPCMD_BRDV_1       |  /* No division */
             RA_SPI_SPCMD_SSLA_0       |  /* Use SSL0 */
             RA_SPI_SPCMD_SCKDEN       |  /* Enable setup delay */
             RA_SPI_SPCMD_SLNDEN       |  /* Enable negation delay */
             RA_SPI_SPCMD_SPNDEN;         /* Enable hold delay */
  ra_spi_putreg16(priv, RA_SPI_SPCMD0_OFFSET, regval16);

  /* Set default bit rate to 1MHz */
  ra_spi_setfrequency(&priv->spidev, 1000000);

  /* Set default configuration */
  priv->mode = SPIDEV_MODE0;
  priv->nbits = 8;
  priv->frequency = 1000000;

  /* Setup DTC if configured */
  ra_spi_dtc_setup(priv);

  /* Enable SPI function */
  regval = ra_spi_getreg8(priv, RA_SPI_SPCR_OFFSET);
  regval |= RA_SPI_SPCR_SPE;
  ra_spi_putreg8(priv, RA_SPI_SPCR_OFFSET, regval);

  spiinfo("SPI%d initialized with dual CS and DTC support\n", priv->config->bus);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   bus number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct spi_dev_s *ra_spibus_initialize(int bus)
{
  struct ra_spi_priv_s *priv = NULL;

  spiinfo("Initializing SPI%d\n", bus);

#ifdef CONFIG_RA_SPI0
  if (bus == 0)
    {
      priv = &ra_spi0_priv;
    }
#endif
#ifdef CONFIG_RA_SPI1
  if (bus == 1)
    {
      priv = &ra_spi1_priv;
    }
#endif

  if (priv == NULL)
    {
      spierr("SPI%d not configured\n", bus);
      return NULL;
    }

  /* Has the SPI hardware been initialized? */
  if (priv->refs == 0)
    {
      /* Initialize hardware */
      ra_spi_bus_initialize(priv);

      /* Attach and enable interrupts */
      irq_attach(priv->config->irq_rxi, ra_spi_rxi_interrupt, priv);
      irq_attach(priv->config->irq_txi, ra_spi_txi_interrupt, priv);
      irq_attach(priv->config->irq_tei, ra_spi_tei_interrupt, priv);
      irq_attach(priv->config->irq_eri, ra_spi_eri_interrupt, priv);

      up_enable_irq(priv->config->irq_rxi);
      up_enable_irq(priv->config->irq_txi);
      up_enable_irq(priv->config->irq_tei);
      up_enable_irq(priv->config->irq_eri);
    }

  /* Increment reference count */
  priv->refs++;

  return (struct spi_dev_s *)priv;
}

/****************************************************************************
 * Name: ra_spi_select_device
 *
 * Description:
 *   Select a specific device and configure CS and frequency accordingly
 *
 * Input Parameters:
 *   dev   - SPI device structure
 *   devid - Device ID
 *   selected - true to select, false to deselect
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra_spi_select_device(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  spiinfo("SPI%d devid=0x%08x selected=%d\n", 
          priv->config->bus, devid, selected);

  if (selected)
    {
      /* Configure CS and timing for this device */
      ra_spi_cs_configure(priv, devid);
      
      /* Store current device ID for frequency limiting */
      priv->current_devid = devid;
      
      /* Assert CS */
      ra_spi_cs_assert(priv, devid);
    }
  else
    {
      /* Deassert CS */
      ra_spi_cs_deassert(priv, devid);
      
      /* Clear current device ID */
      priv->current_devid = 0xffffffff;
    }
}

#endif /* CONFIG_RA_SPI */
