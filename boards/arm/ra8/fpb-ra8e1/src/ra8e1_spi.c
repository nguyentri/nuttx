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
#include <syslog.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fpb_ra8e1_spi_initialize
 *
 * Description:
 *   Initialize SPI buses for the FPB-RA8E1 board
 *   This function initializes both SPI0 and SPI1 for loopback demo
 *
 ****************************************************************************/

int fpb_ra8e1_spi_initialize(void)
{
#ifdef CONFIG_RA8_SPI
  struct spi_dev_s *spi0;
  struct spi_dev_s *spi1;
  int ret;

  /* Initialize SPI0 (Master) */
  spi0 = ra8_spibus_initialize(0);
  if (!spi0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI0 (Master)\n");
      return -ENODEV;
    }

  /* Register SPI0 device */
  spi_register(spi0, 0);
  syslog(LOG_INFO, "SPI0 (Master) initialized successfully\n");

#ifdef CONFIG_RA_SPI1
  /* Initialize SPI1 (Slave) */
  spi1 = ra8_spibus_initialize(1);
  if (!spi1)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI1 (Slave)\n");
      return -ENODEV;
    }

  /* Register SPI1 device */
  spi_register(spi1, 1);
  syslog(LOG_INFO, "SPI1 (Slave) initialized successfully\n");
#endif

#ifdef CONFIG_SPI_DRIVER
  /* Register SPI character drivers */
  ret = spi_register_driver("/dev/spi0", spi0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register SPI0 driver: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_RA_SPI1
  ret = spi_register_driver("/dev/spi1", spi1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register SPI1 driver: %d\n", ret);
      return ret;
    }
#endif
#endif

  syslog(LOG_INFO, "All SPI interfaces initialized for loopback demo\n");

#endif /* CONFIG_RA8_SPI */

  return 0;
}

/****************************************************************************
 * Name: ra8_spi0select
 *
 * Description:
 *   Select or deselect the SPI device specified by 'devid' for SPI0
 *
 ****************************************************************************/

#ifdef CONFIG_RA8_SPI
void ra8_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("SPI0 devid: %" PRIu32 " CS: %s\n",
          devid, selected ? "assert" : "de-assert");

  /* Handle device selection based on device ID */
  switch (devid)
    {
#ifdef CONFIG_RA8_SPI_GY912
      case SPIDEV_GY912(0):
        /* Handle GY-912 sensor chip select on SPI0 */
        /* Implementation would go here based on hardware pinout */
        break;
#endif

#ifdef CONFIG_RA8_SPI_LOOPBACK_DEMO
      case SPIDEV_USER(0):
        /* Handle loopback demo device selection */
        /* For loopback demo, CS handling may be simplified */
        break;
#endif

      default:
        break;
    }
}

/****************************************************************************
 * Name: ra8_spi1select
 *
 * Description:
 *   Select or deselect the SPI device specified by 'devid' for SPI1
 *
 ****************************************************************************/

#ifdef CONFIG_RA_SPI1
void ra8_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  spiinfo("SPI1 devid: %" PRIu32 " CS: %s\n",
          devid, selected ? "assert" : "de-assert");

  /* Handle device selection based on device ID */
  switch (devid)
    {
#ifdef CONFIG_RA8_SPI_LOOPBACK_DEMO
      case SPIDEV_USER(1):
        /* Handle loopback demo device selection for SPI1 */
        /* For loopback demo between SPI0 and SPI1 */
        break;
#endif

      default:
        break;
    }
}
#endif

/****************************************************************************
 * Name: ra8_spi0status
 *
 * Description:
 *   Return status information associated with the SPI0 device.
 *
 ****************************************************************************/

uint8_t ra8_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
#ifdef CONFIG_RA8_SPI_GY912
      case SPIDEV_GY912(0):
        /* Return status for GY-912 sensor */
        status = SPI_STATUS_PRESENT;
        break;
#endif

#ifdef CONFIG_RA8_SPI_LOOPBACK_DEMO
      case SPIDEV_USER(0):
        /* Return status for loopback demo */
        status = SPI_STATUS_PRESENT;
        break;
#endif

      default:
        break;
    }

  return status;
}

/****************************************************************************
 * Name: ra8_spi1status
 *
 * Description:
 *   Return status information associated with the SPI1 device.
 *
 ****************************************************************************/

#ifdef CONFIG_RA_SPI1
uint8_t ra8_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
#ifdef CONFIG_RA8_SPI_LOOPBACK_DEMO
      case SPIDEV_USER(1):
        /* Return status for loopback demo */
        status = SPI_STATUS_PRESENT;
        break;
#endif

      default:
        break;
    }

  return status;
}
#endif

#endif /* CONFIG_RA8_SPI */
