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
#include <nuttx/spi/spi.h>
#include <nuttx/semaphore.h>
#include "ra_gpio.h"

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

/* DTC Channel assignments for SPI transfers */
#define RA_SPI_DTC_TX_CHANNEL    0        /* TX DTC channel */
#define RA_SPI_DTC_RX_CHANNEL    1        /* RX DTC channel */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SPI callback function type */
typedef void (*spi_callback_t)(void *arg);

/* DTC transfer modes for SPI */
#define RA_SPI_DTC_MODE_DISABLED 0        /* DTC disabled */
#define RA_SPI_DTC_MODE_TX_ONLY  1        /* TX only DTC */
#define RA_SPI_DTC_MODE_RX_ONLY  2        /* RX only DTC */
#define RA_SPI_DTC_MODE_FULL     3        /* Full duplex DTC */

/****************************************************************************
 * Public Types
 ****************************************************************************/

 /* Chip Select Configuration */
struct ra_spi_cs_config_s
{
  uint32_t devid;           /* Device ID */
  uint32_t max_frequency;   /* Maximum frequency for this device */
  uint8_t  mode;            /* SPI mode */
  uint8_t  bits;            /* Data bits per transfer */
  gpio_pinset_t cs_gpio;    /* GPIO Chip Select and Slave Select pin definitions */
  bool     use_hardware;    /* Use hardware SS0 or GPIO */
  uint8_t  ssl_select;      /* SSL select value (0-3) */
  uint8_t  setup_delay;     /* CS setup delay */
  uint8_t  hold_delay;      /* CS hold delay */
  uint8_t  negation_delay;  /* CS negation delay */
  bool     active_low;      /* CS active low */
  const char *name;         /* Device name for debugging */
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

/****************************************************************************
 * Name: ra_spi_get_cs_config
 *
 * Description:
 *   Get CS configuration for a specific device (weak function)
 *
 ****************************************************************************/

const struct ra_spi_cs_config_s *ra_spi_get_cs_config(struct spi_dev_s *dev, uint32_t devid);

#endif /* __ARCH_ARM_SRC_RA8_RA_SPI_H */
