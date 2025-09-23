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
#include <arch/ra8/ra8e1_irq.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra_icu.h"
#include "ra_clock.h"
#include "hardware/ra_spi.h"
#include "hardware/ra_dtc.h"
#include "ra_dtc.h"
#include "hardware/ra_dmac.h"
#include "hardware/ra_memorymap.h"
#include "hardware/ra_icu.h"
#include "hardware/ra_mstp.h"
#include "ra_mstp.h"
#include "ra_spi.h"

#ifdef CONFIG_RA_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#define spierr       _err

#define spiwarn     _warn

#define spiinfo     _info


/* DMA timeout */
#define DMA_TIMEOUT_MS          1000

/* SPI timeout */
#define SPI_TIMEOUT_MS          1000

/* Default CS timing values     */
#define CS_SETUP_DELAY          0         /* Default CS setup delay cycles */
#define CS_HOLD_DELAY           0         /* Default CS hold delay cycles */
#define CS_NEGATION_DELAY       0         /* Default CS negation delay cycles */

/* DTC transfer modes */
#define DTC_MODE_NORMAL         0x00
#define DTC_MODE_REPEAT         0x01
#define DTC_MODE_BLOCK          0x02

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Note: CS management is now handled by application-specific */
/* ra_spi_select() function implementations that override the weak */
/* functions in this driver. Each application defines its own */
/* device configurations and CS handling logic. */

/* SPI Device hardware configuration */
struct ra_spi_config_s
{
  uint32_t base;          /* SPI peripheral base address */

  uint8_t  bus;           /* SPI bus number */
  int  elc_rxi;            /* Even Link for RX interrupt */
  int  elc_txi;            /* Even Link for TX interrupt */
  int  elc_tei;            /* Even Link for Transfer end interrupt */
  int  elc_eri;            /* Even Link for Error interrupt */
  uint32_t mstpcrb_bit;   /* Module stop control bit */

  /* Pin configuration - hardware specific */
  gpio_pinset_t sck_pin;       /* SCK pin configuration */
  gpio_pinset_t miso_pin;      /* MISO pin configuration */
  gpio_pinset_t mosi_pin;      /* MOSI pin configuration */

  struct ra_spi_cs_config_s *cs_config;  /* Array of CS configurations */
  int num_cs;                     /* Number of CS configurations */
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
  ra_dtc_info_t            dtc_tx_info; /* TX DTC transfer info */
  ra_dtc_info_t            dtc_rx_info; /* RX DTC transfer info */

  /* Runtime IRQ numbers assigned by ICU */
  int                      irq_rxi;    /* RX interrupt number */
  int                      irq_txi;    /* TX interrupt number */
  int                      irq_tei;    /* Transfer end interrupt number */
  int                      irq_eri;    /* Error interrupt number */

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
static void ra_spi_putreg32(struct ra_spi_priv_s *priv, uint8_t offset, uint32_t value);
static uint32_t ra_spi_getreg32(struct ra_spi_priv_s *priv, uint8_t offset);

/* CS control functions */
static void ra_spi_cs_assert(struct ra_spi_priv_s *priv, uint32_t devid);
static void ra_spi_cs_deassert(struct ra_spi_priv_s *priv, uint32_t devid);
static void ra_spi_cs_configure(struct ra_spi_priv_s *priv, uint32_t devid);

/* DTC support */
static int ra_spi_dtc_setup(struct ra_spi_priv_s *priv);
static void ra_spi_dtc_start(struct ra_spi_priv_s *priv);
static void ra_spi_dtc_stop(struct ra_spi_priv_s *priv);
static int ra_spi_dtc_configure_transfer(struct ra_spi_priv_s *priv,
                                         const void *txbuffer, void *rxbuffer,
                                         size_t nwords);

/* Transfer helpers */
static void ra_spi_writeword(struct ra_spi_priv_s *priv, uint32_t word);
static uint32_t ra_spi_readword(struct ra_spi_priv_s *priv);
static int ra_spi_get_transfer_size(struct ra_spi_priv_s *priv);

/* Interrupt handling */
static int ra_spi_rxi_interrupt(int irq, void *context, void *arg);
static int ra_spi_txi_interrupt(int irq, void *context, void *arg);
static int ra_spi_tei_interrupt(int irq, void *context, void *arg);
static int ra_spi_eri_interrupt(int irq, void *context, void *arg);

/* CS configuration */
const struct ra_spi_cs_config_s * weak_function ra_spi_get_cs_config(struct spi_dev_s *dev, uint32_t devid);

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

/* DTC initialization flag */
static bool g_dtc_initialized = false;
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

#ifdef CONFIG_RA_SPI0
static const struct ra_spi_config_s ra_spi0_config =
{
  .base        = R_SPI0_BASE,
  .bus         = 0,

  .elc_rxi     = RA_ELC_SPI0_RXI,
  .elc_txi     = RA_ELC_SPI0_TXI,
  .elc_tei     = RA_ELC_SPI0_TEI,
  .elc_eri     = RA_ELC_SPI0_ERI,

  .mstpcrb_bit = R_MSTP_MSTPCRB_SPI0,

 /* Pin configuration - uses board-specific definitions from board.h */
  .sck_pin     = GPIO_SPI0_SCK,     /* SPI Clock pin */
  .miso_pin    = GPIO_SPI0_MISO,    /* SPI MISO pin */
  .mosi_pin    = GPIO_SPI0_MOSI,    /* SPI MOSI pin */

  .cs_config = NULL,  /* Application-specific CS configurations will be initialized by the runtime */
  .num_cs = 0,        /* Number of CS configurations will be set at runtime */
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
  .bus         = 1,

  .elc_rxi     = RA_ELC_SPI1_RXI,
  .elc_txi     = RA_ELC_SPI1_TXI,
  .elc_tei     = RA_ELC_SPI1_TEI,
  .elc_eri     = RA_ELC_SPI1_ERI,

  .mstpcrb_bit = R_MSTP_MSTPCRB_SPI1,

  /* Pin configuration - uses board-specific definitions from board.h */
  .sck_pin     = GPIO_SPI1_SCK,     /* SPI Clock pin */
  .miso_pin    = GPIO_SPI1_MISO,    /* SPI MISO pin */
  .mosi_pin    = GPIO_SPI1_MOSI,    /* SPI MOSI pin */

  .cs_config = NULL,  /* Application-specific CS configurations will be initialized by the runtime */
  .num_cs = 0,        /* Number of CS configurations will be set at runtime */
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spi_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void ra_spi_putreg32(struct ra_spi_priv_s *priv, uint8_t offset,
                           uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: ra_spi_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t ra_spi_getreg32(struct ra_spi_priv_s *priv, uint8_t offset)
{
  return getreg32(priv->config->base + offset);
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
  uint32_t spcmd;
  uint8_t setup_delay = CS_SETUP_DELAY;
  uint8_t hold_delay = CS_HOLD_DELAY;
  uint8_t negation_delay = CS_NEGATION_DELAY;
  uint8_t ssl_select = 0;  /* Default to SSL0 */
  bool cs_type = false;

  /* Get device-specific CS configuration if available */
  const struct ra_spi_cs_config_s *cs_config = ra_spi_get_cs_config((struct spi_dev_s *)priv, devid);

  if (cs_config != NULL)
    {
      /* Use device-specific configuration */
      setup_delay = cs_config->setup_delay;
      hold_delay = cs_config->hold_delay;
      negation_delay = cs_config->negation_delay;
      ssl_select = cs_config->ssl_select & 0x07;  /* Limit to 0-7 */
      cs_type = cs_config->cs_type;

      spiinfo("SPI%d: Using device-specific CS config for devid=0x%08lx: SSL=%d, delays=%d/%d/%d, hw=%d\n",
              priv->config->bus, (unsigned long)devid, ssl_select, setup_delay, hold_delay, negation_delay, cs_type);
    }
  else
    {
      spiinfo("SPI%d: Using default CS config for devid=0x%08lx\n", priv->config->bus, (unsigned long)devid);
    }

  /* Configure SPCMD register */
  spcmd = ra_spi_getreg32(priv, RA_SPI_SPCMD0_OFFSET);

  /* Clear SSL selection bits */
  spcmd &= ~RA_SPI_SPCMD_SSLA_MASK;

  /* Set SSL selection */
  spcmd |= (ssl_select << RA_SPI_SPCMD_SSLA_SHIFT) & RA_SPI_SPCMD_SSLA_MASK;

  /* Enable timing delays if configured */
  if (setup_delay > 0)
    {
      spcmd |= RA_SPI_SPCMD_SCKDEN;
    }
  else
    {
      spcmd &= ~RA_SPI_SPCMD_SCKDEN;
    }

  if (negation_delay > 0)
    {
      spcmd |= RA_SPI_SPCMD_SLNDEN;
    }
  else
    {
      spcmd &= ~RA_SPI_SPCMD_SLNDEN;
    }

  if (hold_delay > 0)
    {
      spcmd |= RA_SPI_SPCMD_SPNDEN;
    }
  else
    {
      spcmd &= ~RA_SPI_SPCMD_SPNDEN;
    }

  ra_spi_putreg32(priv, RA_SPI_SPCMD0_OFFSET, spcmd);

  /* Configure timing delay register (SPDECR) - all delays in one 32-bit write */
  uint32_t spdecr = ((setup_delay & 0x07) << RA_SPI_SPDECR_SCKDL_SHIFT) |      /* Clock delay */
                    ((negation_delay & 0x07) << RA_SPI_SPDECR_SLNDL_SHIFT) |   /* SSL negation delay */
                    ((hold_delay & 0x07) << RA_SPI_SPDECR_SPNDL_SHIFT);        /* Next access delay */
  ra_spi_putreg32(priv, RA_SPI_SPDECR_OFFSET, spdecr);

  spiinfo("SPI%d CS configured: SSL=%d, delays=%d/%d/%d\n",
          priv->config->bus, ssl_select, setup_delay, hold_delay, negation_delay);
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
  const struct ra_spi_cs_config_s *cs_config = ra_spi_get_cs_config((struct spi_dev_s *)priv, devid);

  if (cs_config != NULL && !cs_config->cs_type)
    {
      /* Use GPIO-based CS control */
      bool assert_level = !cs_config->active_low;  /* Invert if active low */
      ra_gpiowrite(cs_config->cs_gpio, assert_level);

      spiinfo("SPI%d CS GPIO assert for device 0x%08lx: pin=0x%08lx, level=%d\n",
        priv->config->bus, (unsigned long)devid, (unsigned long)cs_config->cs_gpio.cfg, assert_level);
    }
  else
    {
      /* Hardware CS or no configuration - delegated to application ra_spi_select() */
      spiinfo("SPI%d CS assert delegated to application for device 0x%08lx\n",
        priv->config->bus, (unsigned long)devid);
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
  const struct ra_spi_cs_config_s *cs_config = ra_spi_get_cs_config((struct spi_dev_s *)priv, devid);

  if (cs_config != NULL && !cs_config->cs_type)
    {
      /* Use GPIO-based CS control */
      bool deassert_level = cs_config->active_low;  /* Normal level when inactive */
      ra_gpiowrite(cs_config->cs_gpio, deassert_level);

      spiinfo("SPI%d CS GPIO deassert for device 0x%08lx: pin=0x%08lx, level=%d\n",
        priv->config->bus, (unsigned long)devid, (unsigned long)cs_config->cs_gpio.cfg, deassert_level);
    }
  else
    {
      /* Hardware CS or no configuration - delegated to application ra_spi_select() */
      spiinfo("SPI%d CS deassert delegated to application for device 0x%08lx\n",
        priv->config->bus, (unsigned long)devid);
    }
}

/****************************************************************************
 * Name: ra_spi_apply_cs_config
 *
 * Description:
 *   Apply CS-specific configuration (mode, bits, frequency) for a device
 *
 ****************************************************************************/

static void ra_spi_apply_cs_config(struct spi_dev_s *dev, uint32_t devid)
{
  struct ra_spi_priv_s *priv = (struct ra_spi_priv_s *)dev;
  const struct ra_spi_cs_config_s *cs_config = ra_spi_get_cs_config(dev, devid);

  if (cs_config != NULL)
    {
      /* Apply device-specific SPI settings */
      if (cs_config->max_frequency > 0)
        {
          ra_spi_setfrequency(dev, cs_config->max_frequency);
        }

      if (cs_config->mode >= 0 && cs_config->mode <= 3)
        {
          ra_spi_setmode(dev, (enum spi_mode_e)cs_config->mode);
        }

      if (cs_config->bits > 0)
        {
          ra_spi_setbits(dev, cs_config->bits);
        }

      spiinfo("SPI%d applied CS config for device 0x%08lx: freq=%lu, mode=%u, bits=%u\n",
        priv->config->bus, (unsigned long)devid, (unsigned long)cs_config->max_frequency,
        cs_config->mode, cs_config->bits);
    }
}

/****************************************************************************
 * Name: ra_spi_writeword
 *
 * Description:
 *   Write one word to SPI
 *
 ****************************************************************************/

static void ra_spi_writeword(struct ra_spi_priv_s *priv, uint32_t word)
{
  /* Wait until the transmit buffer is empty */
  while ((ra_spi_getreg32(priv, RA_SPI_SPSR_OFFSET) & RA_SPI_SPSR_SPTEF) == 0);

  /* Write the data using 32-bit register access regardless of data width */
  /* The hardware will use only the relevant bits based on the configured nbits */
  ra_spi_putreg32(priv, RA_SPI_SPDR_OFFSET, word);
}

/****************************************************************************
 * Name: ra_spi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 ****************************************************************************/

static uint32_t ra_spi_readword(struct ra_spi_priv_s *priv)
{
  /* Wait until receive buffer is full */
  while ((ra_spi_getreg32(priv, RA_SPI_SPSR_OFFSET) & RA_SPI_SPSR_SPRF) == 0);

  /* Read the data using 32-bit register access regardless of data width */
  /* The hardware will provide only the relevant bits based on the configured nbits */
  return ra_spi_getreg32(priv, RA_SPI_SPDR_OFFSET);
}

/****************************************************************************
 * Name: ra_spi_get_transfer_size
 *
 * Description:
 *   Get the appropriate transfer size in bytes based on nbits
 *
 ****************************************************************************/

static int ra_spi_get_transfer_size(struct ra_spi_priv_s *priv)
{
  if (priv->nbits <= 8)
    {
      return 1;  /* 8-bit transfers */
    }
  else if (priv->nbits <= 16)
    {
      return 2;  /* 16-bit transfers */
    }
  else
    {
      return 4;  /* 32-bit transfers */
    }
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

  /* Initialize DTC module once (global initialization) */
  if (!g_dtc_initialized)
    {
      irqstate_t flags = enter_critical_section();

      if (!g_dtc_initialized)
        {
          /* Enable DTC module clock */
          ra_mstp_start(RA_MSTP_DMAC_DTC);

          /* Initialize DTC module - this will set up the vector table */
          ra_dtc_initialize();

          g_dtc_initialized = true;

          spiinfo("DTC module initialized\n");
        }

      leave_critical_section(flags);
    }

  /* The DTC transfer info structures are already part of priv structure */
  /* They will be configured per-transfer in ra_spi_dtc_configure_transfer */

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
  uint8_t transfer_size;

  spiinfo("Configuring DTC transfer: tx=%p rx=%p nwords=%zu\n",
          txbuffer, rxbuffer, nwords);

  /* Determine transfer size based on SPI bit width */
  if (priv->nbits > 16)
    {
      transfer_size = RA_DTC_MRA_SZ_LONG; /* 32-bit transfers for 17-32 bits */
    }
  else if (priv->nbits > 8)
    {
      transfer_size = RA_DTC_MRA_SZ_WORD; /* 16-bit transfers for 9-16 bits */
    }
  else
    {
      transfer_size = RA_DTC_MRA_SZ_BYTE; /* 8-bit transfers for 4-8 bits */
    }

  /* Clear DTC transfer info structures */
  memset(&priv->dtc_tx_info, 0, sizeof(priv->dtc_tx_info));
  memset(&priv->dtc_rx_info, 0, sizeof(priv->dtc_rx_info));

  /* Configure TX DTC if transmit buffer provided */
  if (txbuffer)
    {
      /* Configure TX DTC: source increment, dest fixed, normal mode */
      priv->dtc_tx_info.mra = RA_DTC_MRA_MD_NORMAL | transfer_size | RA_DTC_MRA_SM_INCREMENT;
      priv->dtc_tx_info.mrb = RA_DTC_MRB_DM_FIXED | RA_DTC_MRB_DISEL; /* IRQ at end */
      priv->dtc_tx_info.sar = (uint32_t)txbuffer;
      priv->dtc_tx_info.dar = priv->config->base + RA_SPI_SPDR_OFFSET;
      priv->dtc_tx_info.cra = (uint16_t)nwords;
      priv->dtc_tx_info.crb = 0;

      spiinfo("TX DTC: MRA=0x%02x MRB=0x%02x SAR=0x%08lx DAR=0x%08lx CRA=%d\n",
              priv->dtc_tx_info.mra, priv->dtc_tx_info.mrb,
              (unsigned long)priv->dtc_tx_info.sar, (unsigned long)priv->dtc_tx_info.dar, priv->dtc_tx_info.cra);
    }

  /* Configure RX DTC if receive buffer provided */
  if (rxbuffer)
    {
      /* Configure RX DTC: source fixed, dest increment, normal mode */
      priv->dtc_rx_info.mra = RA_DTC_MRA_MD_NORMAL | transfer_size | RA_DTC_MRA_SM_FIXED;
      priv->dtc_rx_info.mrb = RA_DTC_MRB_DM_INCREMENT | RA_DTC_MRB_DISEL; /* IRQ at end */
      priv->dtc_rx_info.sar = priv->config->base + RA_SPI_SPDR_OFFSET;
      priv->dtc_rx_info.dar = (uint32_t)rxbuffer;
      priv->dtc_rx_info.cra = (uint16_t)nwords;
      priv->dtc_rx_info.crb = 0;

      spiinfo("RX DTC: MRA=0x%02x MRB=0x%02x SAR=0x%08lx DAR=0x%08lx CRA=%d\n",
              priv->dtc_rx_info.mra, priv->dtc_rx_info.mrb,
              (unsigned long)priv->dtc_rx_info.sar, (unsigned long)priv->dtc_rx_info.dar, priv->dtc_rx_info.cra);
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
  volatile uint32_t spcr;

  spiinfo("DTC start for SPI%d - TX IRQ=%d, RX IRQ=%d, TEI IRQ=%d, ERI IRQ=%d\n",
          priv->config->bus, priv->irq_txi, priv->irq_rxi, priv->irq_tei, priv->irq_eri);

  priv->dtc_active = true;

  /* Configure DTC vector table entries using ICU-assigned slot numbers */
  if (priv->txbuffer && priv->irq_txi >= 0)
    {
      int slot = priv->irq_txi - RA_IRQ_FIRST;
      ra_dtc_set_vector(slot, &priv->dtc_tx_info);
      spiinfo("Set TX DTC vector for slot %d (IRQ %d)\n", slot, priv->irq_txi);
    }

  if (priv->rxbuffer && priv->irq_rxi >= 0)
    {
      int slot = priv->irq_rxi - RA_IRQ_FIRST;
      ra_dtc_set_vector(slot, &priv->dtc_rx_info);
      spiinfo("Set RX DTC vector for slot %d (IRQ %d)\n", slot, priv->irq_rxi);
    }

  /* Enable DTC triggers in ICU using ICU API instead of direct register access */
  if (priv->irq_txi >= 0)
    {
      ra_icu_enable_dtc(priv->irq_txi);
      spiinfo("Enabled DTC trigger for TXI IRQ %d\n", priv->irq_txi);
    }

  if (priv->irq_rxi >= 0)
    {
      ra_icu_enable_dtc(priv->irq_rxi);
      spiinfo("Enabled DTC trigger for RXI IRQ %d\n", priv->irq_rxi);
    }

  /* Enable TEI interrupt for completion detection */
  if (priv->irq_tei >= 0)
    {
      up_enable_irq(priv->irq_tei);
      spiinfo("Enabled TEI interrupt %d for completion\n", priv->irq_tei);
    }

  if (priv->irq_eri >= 0)
    {
      up_enable_irq(priv->irq_eri);
      spiinfo("Enabled ERI interrupt %d for error detection\n", priv->irq_eri);
    }

  /* Note: TXI and RXI interrupts are handled by DTC, but we may need to enable */
  /* them if DTC doesn't handle all cases. For now, DTC should handle them. */

  /* Clear any existing interrupt flags before enabling interrupts */
  /* This is critical - flags that are already set won't trigger interrupts when enabled */
  ra_spi_putreg32(priv, RA_SPI_SPSRC_OFFSET, RA_SPI_SPSRC_ALL_CLEAR);

  /* Clear FIFOs to ensure a clean start */
  ra_spi_putreg32(priv, RA_SPI_SPFCR_OFFSET, RA_SPI_SPFCR_SPFRST);

  /* Get SPCR */
  spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);

  /* SPTIE must be enabled for DTC even if transmitting from RXI */
  if (priv->txbuffer)
    {
      spcr |= RA_SPI_SPCR_SPTIE;
    }

  /* SPRIE only for full-duplex (when both TX and RX are active) */
  if (priv->txbuffer && priv->rxbuffer)
    {
      spcr |= RA_SPI_SPCR_SPRIE;
    }

  /* Always enable error interrupt */
  spcr |= RA_SPI_SPCR_SPEIE;

  /* Always enable communication end interrupt */
  spcr |= RA_SPI_SPCR_CENDIE;

  /* SPI Mode Select d*/
  if (priv->config->cs_config == NULL || priv->config->cs_config->cs_type == RA_SPI_CS_CLK_SYS)
    {
      spcr |= RA_SPI_SPCR_SPMS; /* Clock synchronous operation (3-wire) */
    }
  spcr |= RA_SPI_SPCR_BPEN; /* Bit-rate switch */

  /* Write SPCR without SPE first to ensure interrupt/DTC bits are committed */
  ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr & ~RA_SPI_SPCR_SPE);

  /* Small delay to allow settings to take effect before enabling SPE */
  up_udelay(1);

  /* Now set SPE to start the transfer */
  ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr | RA_SPI_SPCR_SPE);

  /* Mark DTC as active */
  priv->dtc_active = true;

  spiinfo("SPI transfer started: SPCR=0x%08lx\n", spcr);
}/****************************************************************************
 * Name: ra_spi_dtc_stop
 *
 * Description:
 *   Stop DTC transfer
 *
 ****************************************************************************/

static void ra_spi_dtc_stop(struct ra_spi_priv_s *priv)
{
  uint32_t spcr;

  spiinfo("DTC stop for SPI%d\n", priv->config->bus);

  /* Mark DTC as no longer active */
  priv->dtc_active = false;

  /* Disable SPI interrupts */
  spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
  spcr &= ~(RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE | RA_SPI_SPCR_SPEIE | RA_SPI_SPCR_CENDIE);
  ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);

  /* Disable DTC triggers in ICU using assigned slot numbers */
  if (priv->irq_txi >= 0)
    {
      ra_icu_disable_dtc(priv->irq_txi);
      spiinfo("Disabled DTC trigger for TXI IRQ %d\n", priv->irq_txi);
    }

  if (priv->irq_rxi >= 0)
    {
      ra_icu_disable_dtc(priv->irq_rxi);
      spiinfo("Disabled DTC trigger for RXI IRQ %d\n", priv->irq_rxi);
    }

  /* Disable interrupts that were enabled during transfer */
  if (priv->irq_tei >= 0)
    {
      up_disable_irq(priv->irq_tei);
      spiinfo("Disabled TEI interrupt %d\n", priv->irq_tei);
    }

  if (priv->irq_eri >= 0)
    {
      up_disable_irq(priv->irq_eri);
      spiinfo("Disabled ERI interrupt %d\n", priv->irq_eri);
    }

  /* Clear DTC vector table entries */
  if (priv->irq_txi >= 0)
    {
      int slot = priv->irq_txi - RA_IRQ_FIRST;
      ra_dtc_clear_vector(slot);
    }

  if (priv->irq_rxi >= 0)
    {
      int slot = priv->irq_rxi - RA_IRQ_FIRST;
      ra_dtc_clear_vector(slot);
    }

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
  uint32_t data;

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
      int transfer_size = ra_spi_get_transfer_size(priv);

      if (transfer_size == 4)
        {
          *((uint32_t *)priv->rxbuffer) = data;
          priv->rxbuffer = (uint8_t *)priv->rxbuffer + 4;
        }
      else if (transfer_size == 2)
        {
          *((uint16_t *)priv->rxbuffer) = (uint16_t)data;
          priv->rxbuffer = (uint8_t *)priv->rxbuffer + 2;
        }
      else
        {
          *((uint8_t *)priv->rxbuffer) = (uint8_t)data;
          priv->rxbuffer = (uint8_t *)priv->rxbuffer + 1;
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
      uint32_t spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~RA_SPI_SPCR_SPRIE;
      ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);

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
  uint32_t data = 0xffffffff;

  DEBUGASSERT(priv != NULL);

  if (priv->dtc_active)
    {
      /* DTC is handling the transfer, just check for completion */
      if (priv->dtc_tx_info.cra == 0)
        {
          /* DTC transfer complete */
          uint32_t spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
          spcr &= ~RA_SPI_SPCR_SPTIE;
          ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);
        }
      return OK;
    }

  /* Get next word to transmit */
  if (priv->txbuffer && priv->ntxwords > 0)
    {
      int transfer_size = ra_spi_get_transfer_size(priv);

      if (transfer_size == 4)
        {
          data = *((uint32_t *)priv->txbuffer);
          priv->txbuffer = (uint8_t *)priv->txbuffer + 4;
        }
      else if (transfer_size == 2)
        {
          data = (uint32_t)*((uint16_t *)priv->txbuffer);
          priv->txbuffer = (uint8_t *)priv->txbuffer + 2;
        }
      else
        {
          data = (uint32_t)*((uint8_t *)priv->txbuffer);
          priv->txbuffer = (uint8_t *)priv->txbuffer + 1;
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
      uint32_t spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~RA_SPI_SPCR_SPTIE;
      ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);
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

  /* If DTC is active, this signals completion of DTC transfer */
  if (priv->dtc_active)
    {
      spiinfo("DTC transfer completed for SPI%d\n", priv->config->bus);

      /* Disable SPI Transfer  */
      uint32_t spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~RA_SPI_SPCR_SPE;
      ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);

      /* Stop DTC transfers */
      ra_spi_dtc_stop(priv);

      /* Signal completion to waiting thread */
      nxsem_post(&priv->waitsem);
    }

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
  uint32_t spsr;

  DEBUGASSERT(priv != NULL);

  spsr = ra_spi_getreg32(priv, RA_SPI_SPSR_OFFSET);

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
  //uint32_t src_clk = priv->config->clk_freq;
  uint32_t src_clk = RA_PCLKA_FREQUENCY;
  uint32_t divisor;
  uint8_t spbr;
  uint8_t brdv = 0;

  spiinfo("SPI%d frequency %lu\n", priv->config->bus, (unsigned long)frequency);

  if (priv->frequency == frequency)
    {
      /* We are already at this frequency. Return the actual. */
      return priv->actual;
    }

  /* Limit to maximum frequency - applications can implement their own limits */
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

  spiinfo("SPI%d SPBR=%d BRDV=%d actual=%lu\n",
    priv->config->bus, spbr, brdv, (unsigned long)priv->actual);

  /* Set the bit rate field in SPCR3 register */
  uint32_t spcr3 = ra_spi_getreg32(priv, RA_SPI_SPCR3_OFFSET);
  spcr3 &= ~RA_SPI_SPCR3_SPBR_MASK;
  spcr3 |= ((spbr & 0xFF) << RA_SPI_SPCR3_SPBR_SHIFT);
  ra_spi_putreg32(priv, RA_SPI_SPCR3_OFFSET, spcr3);

  /* Update SPCMD0 with new BRDV */
  uint32_t spcmd0 = ra_spi_getreg32(priv, RA_SPI_SPCMD0_OFFSET);
  spcmd0 &= ~RA_SPI_SPCMD_BRDV_MASK;
  spcmd0 |= (brdv << RA_SPI_SPCMD_BRDV_SHIFT);
  ra_spi_putreg32(priv, RA_SPI_SPCMD0_OFFSET, spcmd0);

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

  spcmd0 = ra_spi_getreg32(priv, RA_SPI_SPCMD0_OFFSET);
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

  ra_spi_putreg32(priv, RA_SPI_SPCMD0_OFFSET, spcmd0);
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
  uint32_t spcmd0;
  uint32_t spb_bits;

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

  spcmd0 = ra_spi_getreg32(priv, RA_SPI_SPCMD0_OFFSET);
  spcmd0 &= ~RA_SPI_SPCMD_SPB_MASK;
  spcmd0 |= spb_bits;
  ra_spi_putreg32(priv, RA_SPI_SPCMD0_OFFSET, spcmd0);

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

  spiinfo("SPI%d features %08lx\n", priv->config->bus, (unsigned long)features);

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
  uint32_t ret;

  DEBUGASSERT(priv != NULL);

  /* Write the word to transmit */
  ra_spi_writeword(priv, wd);

  /* Read the received word */
  ret = ra_spi_readword(priv);

  spiinfo("SPI%d sent %08lx received %08lx\n",
    priv->config->bus, (unsigned long)wd, (unsigned long)ret);

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

      /* Wait for DTC completion (TEI interrupt will signal completion) */
      nxsem_wait_uninterruptible(&priv->waitsem);

      /* DTC stop is handled by TEI interrupt - don't call it here */
    }
  else
    {
      /* Use interrupt-driven transfer */
      flags = enter_critical_section();

      /* Enable interrupts */
      spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
      spcr |= (RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE | RA_SPI_SPCR_SPEIE);
      ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);

      leave_critical_section(flags);

      /* Wait for transfer completion */
      nxsem_wait_uninterruptible(&priv->waitsem);

      /* Disable interrupts */
      flags = enter_critical_section();

      spcr = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
      spcr &= ~(RA_SPI_SPCR_SPRIE | RA_SPI_SPCR_SPTIE | RA_SPI_SPCR_SPEIE);
      ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, spcr);

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
  uint32_t regval;
  uint32_t regval16;

  /* Configure GPIO pins for SPI */
  ra_configgpio(priv->config->sck_pin);
  ra_configgpio(priv->config->miso_pin);
  ra_configgpio(priv->config->mosi_pin);

  /* Note: CS pin initialization is now handled by application-specific */
  /* ra_spi_select() function implementations or via CS configuration */

  /* Enable SPI module */
  if (priv->config->base == RA_SPI0_BASE)
    {
      ra_mstp_start(RA_MSTP_SPI0);
    }
  else if (priv->config->base == RA_SPI1_BASE)
    {
      ra_mstp_start(RA_MSTP_SPI1);
    }
  else
    {
      spierr("SPI%d invalid module\n", priv->config->bus);
      return;
    }
  /* Disable SPI function */
  ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, 0);

  /* Configure as master mode with auto-stop for overflow prevention */
  //regval = RA_SPI_SPCR_MSTR | RA_SPI_SPCR_MODFEN | RA_SPI_SPCR_SCKASE; // MODFEN is set in slave mode only
  regval = RA_SPI_SPCR_MSTR | RA_SPI_SPCR_SCKASE;
  ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, regval);

  /* SSL polarity is already set to 0 (active low) in SPCR3 above */
  /* Pin control register is already set to 0 in SPCR2 above */

  /* Set sequence length to 1 (using SPCMD0 only) - sequence control is in SPCR3 */
  ra_spi_putreg32(priv, RA_SPI_SPCR3_OFFSET, 0);

  /* Configure data control register for 32-bit access */
  ra_spi_putreg32(priv, RA_SPI_SPDCR_OFFSET, 0);

  /* Set clock delays in SPDECR register - all delays in one 32-bit write */
  uint32_t spdecr_init = ((CS_SETUP_DELAY & 0x07) << RA_SPI_SPDECR_SCKDL_SHIFT) |
                         ((CS_NEGATION_DELAY & 0x07) << RA_SPI_SPDECR_SLNDL_SHIFT) |
                         ((CS_HOLD_DELAY & 0x07) << RA_SPI_SPDECR_SPNDL_SHIFT);
  ra_spi_putreg32(priv, RA_SPI_SPDECR_OFFSET, spdecr_init);

  /* Configure SPCR2 */
  ra_spi_putreg32(priv, RA_SPI_SPCR2_OFFSET, 0);

  /* Configure SPCMD0 for default settings */
  regval16 = RA_SPI_SPCMD_SPB_8        |  /* 8-bit data */
             RA_SPI_SPCMD_BRDV_1       |  /* No division */
             RA_SPI_SPCMD_SSLA_0       |  /* Use SSL0 */
             RA_SPI_SPCMD_SCKDEN       |  /* Enable setup delay */
             RA_SPI_SPCMD_SLNDEN       |  /* Enable negation delay */
             RA_SPI_SPCMD_SPNDEN;         /* Enable hold delay */
  ra_spi_putreg32(priv, RA_SPI_SPCMD0_OFFSET, regval16);

  /* Set default bit rate to 1MHz */
  ra_spi_setfrequency(&priv->spidev, 1000000);

  /* Set default configuration */
  priv->mode = SPIDEV_MODE0;
  priv->nbits = 8;
  priv->frequency = 1000000;

  /* Setup DTC if configured */
  ra_spi_dtc_setup(priv);

  /* Enable SPI function */
  regval = ra_spi_getreg32(priv, RA_SPI_SPCR_OFFSET);
  regval |= RA_SPI_SPCR_SPE;
  ra_spi_putreg32(priv, RA_SPI_SPCR_OFFSET, regval);
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
  int ret;

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
      /* Initialize hardware first, before attaching interrupts */
      ra_spi_bus_initialize(priv);

      /* Attach interrupts but immediately disable them to prevent spurious interrupts */
      ret = ra_icu_attach(priv->config->elc_rxi, ra_spi_rxi_interrupt, priv, false);
      if (ret < 0)
        {
          return NULL;
        }
      priv->irq_rxi = ret; /* Store the assigned IRQ number */
      ret = ra_icu_attach(priv->config->elc_txi, ra_spi_txi_interrupt, priv, false);
      if (ret < 0)
        {
          return NULL;
        }
      priv->irq_txi = ret; /* Store the assigned IRQ number */

      ret = ra_icu_attach(priv->config->elc_tei, ra_spi_tei_interrupt, priv, false);
      if (ret < 0)
        {
          return NULL;
        }
      priv->irq_tei = ret; /* Store the assigned IRQ number */

      ret = ra_icu_attach(priv->config->elc_eri, ra_spi_eri_interrupt, priv, false);
      if (ret < 0)
        {
          return NULL;
        }
      priv->irq_eri = ret; /* Store the assigned IRQ number */
      spiinfo("SPI%d interrupts attached: RXI=%d TXI=%d TEI=%d ERI=%d (all disabled until transfer)\n",
              priv->config->bus, priv->irq_rxi, priv->irq_txi, priv->irq_tei, priv->irq_eri);
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

  spiinfo("SPI%d devid=0x%08lx selected=%d\n",
    priv->config->bus, (unsigned long)devid, selected);

  if (selected)
    {
      /* Configure CS and timing for this device */
      ra_spi_cs_configure(priv, devid);

      /* Apply device-specific SPI configuration (mode, bits, frequency) */
      ra_spi_apply_cs_config(dev, devid);

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

/****************************************************************************
 * Name: ra_spi_select
 *
 * Description:
 *   Board-specific SPI device select function called by the SPI driver.
 *   This is a weak function that can be overridden by board-specific
 *   implementations.
 *
 ****************************************************************************/

void weak_function ra_spi_select(struct spi_dev_s *dev, uint32_t devid,
                                 bool selected)
{
  /* Default implementation does nothing */
  UNUSED(dev);
  UNUSED(devid);
  UNUSED(selected);
}

/****************************************************************************
 * Name: ra_spi_status
 *
 * Description:
 *   Board-specific SPI device status function called by the SPI driver.
 *   This is a weak function that can be overridden by board-specific
 *   implementations.
 *
 ****************************************************************************/

uint8_t weak_function ra_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  /* Default implementation returns no status */
  UNUSED(dev);
  UNUSED(devid);
  return 0;
}

/****************************************************************************
 * Name: ra_spi_cmddata
 *
 * Description:
 *   Board-specific SPI command/data function called by the SPI driver.
 *   This is a weak function that can be overridden by board-specific
 *   implementations.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int weak_function ra_spi_cmddata(struct spi_dev_s *dev, uint32_t devid,
                                 bool cmd)
{
  /* Default implementation does nothing */
  UNUSED(dev);
  UNUSED(devid);
  UNUSED(cmd);
  return OK;
}
#endif

/****************************************************************************
 * Name: ra_spi_register_callback
 *
 * Description:
 *   Board-specific SPI register callback function called by the SPI driver.
 *   This is a weak function that can be overridden by board-specific
 *   implementations.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spi_get_cs_config
 *
 * Description:
 *   Board-specific function to get CS configuration for a device.
 *   This is a weak function that can be overridden by board-specific
 *   implementations to provide device-specific CS configurations.
 *
 ****************************************************************************/

const struct ra_spi_cs_config_s * weak_function ra_spi_get_cs_config(struct spi_dev_s *dev, uint32_t devid)
{
  /* Default implementation returns NULL - use default settings */
  UNUSED(dev);
  UNUSED(devid);
  return NULL;
}

#ifdef CONFIG_SPI_CALLBACK
int weak_function ra_spi_register_callback(struct spi_dev_s *dev,
                                           spi_callback_t callback, void *arg)
{
  /* Default implementation does nothing */
  UNUSED(dev);
  UNUSED(callback);
  UNUSED(arg);
  return OK;
}
#endif

#endif /* CONFIG_RA_SPI */
