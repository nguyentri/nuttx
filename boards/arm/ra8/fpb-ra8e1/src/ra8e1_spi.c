/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "fpb-ra8e1.h"
#include "spi_gy912.h"

#ifdef CONFIG_RA8_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */
#ifdef CONFIG_DEBUG_SPI_INFO
#  define spiinfo _info
#else
#  define spiinfo(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_WARN
#  define spiwarn _warn
#else
#  define spiwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SPI_ERROR
#  define spierr _err
#else
#  define spierr(x...)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spi0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spi_select
 *
 * Description:
 *   Select/deselect the SPI device specified by 'devid'
 *
 * Input Parameters:
 *   dev    - SPI device instance
 *   devid  - Device ID (chip select)
 *   selected - True if selecting device
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  switch (devid)
    {
      case SPIDEV_ICM20948:
        /* ICM-20948 on P612 (CS0) */
        ra_gpio_write(GY912_ICM20948_CS_PIN, !selected);
        break;

      case SPIDEV_BMP388:
        /* BMP388 on P605 (CS1) */
        ra_gpio_write(GY912_BMP388_CS_PIN, !selected);
        break;

      default:
        spierr("Unknown device ID: %d\n", (int)devid);
        break;
    }
}

/****************************************************************************
 * Name: ra_spi_status
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   devid - Identifies the device to report status on
 *
 * Returned Value:
 *   Returns a bitset of status values (see SPI_STATUS_* defines)
 *
 ****************************************************************************/

uint8_t ra_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
      case SPIDEV_ICM20948:
        /* ICM-20948 is always present on GY-912 module */
        status |= SPI_STATUS_PRESENT;
        break;

      case SPIDEV_BMP388:
        /* BMP388 is always present on GY-912 module */
        status |= SPI_STATUS_PRESENT;
        break;

      default:
        break;
    }

  spiinfo("devid: %d status: 0x%02x\n", (int)devid, status);
  return status;
}

#ifdef CONFIG_SPI_CMDDATA
/****************************************************************************
 * Name: ra_spi_cmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data. If CONFIG_SPI_CMDDATA then
 *   this function will be called to different be command data and other data.
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   devid - Identifies the device to see CMD or DATA
 *   cmd - true: The following word is a command; false: the following words
 *         are data.
 *
 * Returned Value:
 *   OK unless an error occurs.  Then a negated errno value is returned
 *
 ****************************************************************************/

int ra_spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  spiinfo("devid: %d CMD: %s\n", (int)devid, cmd ? "command" : "data");

  switch (devid)
    {
      case SPIDEV_ICM20948:
      case SPIDEV_BMP388:
        /* These devices don't use separate CMD/DATA lines */
        return OK;

      default:
        return -ENODEV;
    }
}
#endif

#ifdef CONFIG_SPI_CALLBACK
/****************************************************************************
 * Name: ra_spi_register_callback
 *
 * Description:
 *   Register a callback that will be invoked on any SPI completion or error
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   callback - The function to call on the completion of any SPI DMA
 *   arg      - An argument that will be provided when the callback is invoked
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_spi_register_callback(struct spi_dev_s *dev, spi_callback_t callback,
                             void *arg)
{
  /* This function would be implemented if we supported DMA completion callbacks */
  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: fpb_ra8e1_spi_initialize
 *
 * Description:
 *   Initialize SPI driver and register devices
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fpb_ra8e1_spi_initialize(void)
{
  spiinfo("Initializing SPI for FPB-RA8E1\n");

  /* Configure CS pins as outputs (initially high - deselected) */
  ra_gpio_config(GY912_ICM20948_CS_PIN);
  ra_gpio_config(GY912_BMP388_CS_PIN);

  /* Initialize SPI0 */
  g_spi0 = ra_spibus_initialize(0);
  if (g_spi0 == NULL)
    {
      spierr("Failed to initialize SPI0\n");
      return -ENODEV;
    }

  spiinfo("SPI0 initialized successfully\n");

  /* Test SPI by reading chip IDs */
  struct spi_dev_s *spi = g_spi0;
  uint8_t icm_id, bmp_id;

  /* Test ICM-20948 */
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, GY912_SPI_FREQUENCY);
  SPI_SETMODE(spi, GY912_SPI_MODE);
  SPI_SETBITS(spi, GY912_SPI_BITS);

  SPI_SELECT(spi, SPIDEV_ICM20948, true);
  SPI_SEND(spi, ICM20948_WHO_AM_I | 0x80);  /* Read bit set */
  icm_id = SPI_SEND(spi, 0xFF);
  SPI_SELECT(spi, SPIDEV_ICM20948, false);

  /* Test BMP388 */
  SPI_SELECT(spi, SPIDEV_BMP388, true);
  SPI_SEND(spi, BMP388_CHIP_ID | 0x80);  /* Read bit set */
  bmp_id = SPI_SEND(spi, 0xFF);
  SPI_SELECT(spi, SPIDEV_BMP388, false);

  SPI_LOCK(spi, false);

  spiinfo("ICM-20948 ID: 0x%02x (expected: 0x%02x)\n", icm_id, ICM20948_WHO_AM_I_VAL);
  spiinfo("BMP388 ID: 0x%02x (expected: 0x%02x)\n", bmp_id, BMP388_CHIP_ID_VAL);

  if (icm_id != ICM20948_WHO_AM_I_VAL)
    {
      spiwarn("ICM-20948 not detected or communication failed\n");
    }

  if (bmp_id != BMP388_CHIP_ID_VAL)
    {
      spiwarn("BMP388 not detected or communication failed\n");
    }

  return OK;
}

/****************************************************************************
 * Name: fpb_ra8e1_spi_bus_test
 *
 * Description:
 *   Test SPI bus functionality
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int fpb_ra8e1_spi_bus_test(void)
{
  struct spi_dev_s *spi = g_spi0;
  uint8_t test_data[] = {0x55, 0xAA, 0xFF, 0x00};
  uint8_t read_data[sizeof(test_data)];
  int i;

  if (spi == NULL)
    {
      spierr("SPI0 not initialized\n");
      return -ENODEV;
    }

  spiinfo("Testing SPI bus functionality\n");

  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, 1000000);  /* 1MHz for testing */
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);

  /* Test with ICM-20948 */
  SPI_SELECT(spi, SPIDEV_ICM20948, true);
  
  for (i = 0; i < sizeof(test_data); i++)
    {
      read_data[i] = SPI_SEND(spi, test_data[i]);
      spiinfo("Sent: 0x%02x, Received: 0x%02x\n", test_data[i], read_data[i]);
    }
  
  SPI_SELECT(spi, SPIDEV_ICM20948, false);
  SPI_LOCK(spi, false);

  return OK;
}

#endif /* CONFIG_RA8_SPI */
