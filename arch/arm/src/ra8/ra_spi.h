/****************************************************************************
 * arch/arm/src/ra8/ra_spi.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_SPI_H
#define __ARCH_ARM_SRC_RA8_RA_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Configuration */
#define RA_SPI_MAX_FREQUENCY     8000000  /* Maximum SPI frequency */
#define RA_SPI_MIN_FREQUENCY     1000     /* Minimum SPI frequency */

/* SPI Bus numbers */
#define RA_SPI_BUS_0             0
#define RA_SPI_BUS_1             1

/* Chip Select definitions for GY-912 sensors */
#define RA_SPI_CS_IMU            0        /* P612 - ICM20948 IMU */
#define RA_SPI_CS_BMP            1        /* P605 - BMP388 pressure sensor */

/* DMA Channel assignments */
#define RA_SPI_DMA_TX_CHANNEL    0        /* TX DMA channel */
#define RA_SPI_DMA_RX_CHANNEL    1        /* RX DMA channel */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SPI device configuration */
struct ra_spi_config_s
{
  uint8_t  bus;              /* SPI bus number */
  uint32_t frequency;        /* SPI frequency */
  uint8_t  mode;            /* SPI mode (0-3) */
  uint8_t  nbits;           /* Number of bits per word */
  uint8_t  cs;              /* Chip select line */
  bool     use_dma;         /* Use DMA for transfers */
};

/* SPI driver state */
struct ra_spi_priv_s
{
  struct spi_dev_s spidev;   /* Externally visible part of the SPI interface */
  uint32_t         base;     /* SPI controller register base address */
  uint8_t          bus;      /* SPI bus number */
  uint8_t          irq_rxi;  /* RX interrupt number */
  uint8_t          irq_txi;  /* TX interrupt number */
  uint8_t          irq_tei;  /* Transfer end interrupt number */
  uint8_t          irq_eri;  /* Error interrupt number */
  sem_t            exclsem;  /* Holds exclusive access to SPI */
  sem_t            waitsem;  /* Wait for transfer completion */
  
  /* DMA support */
  bool             use_dma;   /* DMA enabled flag */
  uint8_t          dma_tx_ch; /* TX DMA channel */
  uint8_t          dma_rx_ch; /* RX DMA channel */
  
  /* Transfer state */
  const uint8_t   *txbuffer;  /* Source data for SPI output */
  uint8_t         *rxbuffer;  /* Sink for SPI input */
  size_t           ntxwords;  /* Number of words left to transfer */
  size_t           nrxwords;  /* Number of words left to receive */
  
  /* Configuration */
  uint32_t         frequency; /* Requested clock frequency */
  uint32_t         actual;    /* Actual clock frequency */
  uint8_t          mode;      /* Mode 0,1,2,3 */
  uint8_t          nbits;     /* Width of word in bits (4-16) */
  
  /* Debug */
  uint32_t         debug_flags;
};

/****************************************************************************
 * Public Function Prototypes
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

struct spi_dev_s *ra_spibus_initialize(int bus);

/****************************************************************************
 * Name: ra_spi_register_callback
 *
 * Description:
 *   Register a callback that will be invoked on any SPI completion or error
 *
 ****************************************************************************/

int ra_spi_register_callback(struct spi_dev_s *dev, spi_callback_t callback,
                             void *arg);

/****************************************************************************
 * Name: ra_spi_exchange (optional)
 *
 * Description:
 *   Exchange a block of data from SPI. This is alternative interface when
 *   a complex SPI device requires both sending and receiving data
 *   simultaneously.
 *
 ****************************************************************************/

void ra_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                     void *rxbuffer, size_t nwords);

/****************************************************************************
 * Name: ra_spi_select
 *
 * Description:
 *   Control the chip select
 *
 ****************************************************************************/

void ra_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected);

/****************************************************************************
 * Name: ra_spi_status
 *
 * Description:
 *   Return status information associated with the SPI device
 *
 ****************************************************************************/

uint8_t ra_spi_status(struct spi_dev_s *dev, uint32_t devid);

#ifdef CONFIG_SPI_CMDDATA
/****************************************************************************
 * Name: ra_spi_cmddata
 *
 * Description:
 *   Control the SPI CMD/DATA line if supported by the device
 *
 ****************************************************************************/

int ra_spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd);
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_SPI_H */
