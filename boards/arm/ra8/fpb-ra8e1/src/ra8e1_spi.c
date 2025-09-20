
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
#include "ra_gpio.h"
#include "ra_spi.h"
#include "fpb-ra8e1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Note: SPIDEV_IMU and SPIDEV_BAROMETER are already defined in spi.h */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_spi_initialize
 *
 * Description:
 *   Initialize SPI buses for the FPB-RA8E1 board with dual CS support
 *
 ****************************************************************************/

int ra8e1_spi_initialize(void)
{
#ifdef CONFIG_RA_SPI
  struct spi_dev_s *spi0;
  int ret;

  /* Initialize SPI0 with dual CS support */
  spi0 = ra_spibus_initialize(0);
  if (!spi0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI0\n");
      return -ENODEV;
    }

  /* Register SPI0 device */
  spi_register(spi0, 0);
  syslog(LOG_INFO, "SPI0 initialized with dual CS support\n");

#ifdef CONFIG_SPI_DRIVER
  /* Register SPI character driver */
  ret = spi_register(spi0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register SPI0 driver: %d\n", ret);
      return ret;
    }
#endif

  syslog(LOG_INFO, "SPI interfaces initialized for GY-912 sensors\n");

#endif /* CONFIG_RA_SPI */

  return 0;
}

#ifdef CONFIG_RA_SPI

/****************************************************************************
 * Name: ra_spi_select
 *
 * Description:
 *   Board-specific SPI device select function. This overrides the weak
 *   function in ra_spi.c and provides proper chip select control.
 *
 ****************************************************************************/

void ra_spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                   bool selected)
{
  /* For now, just call the SPI0 select function since we're primarily
   * using SPI0 for the current implementation
   */
  ra_spi0select(dev, devid, selected);
}

/****************************************************************************
 * Name: ra_spi_status
 *
 * Description:
 *   Board-specific SPI status function. This overrides the weak
 *   function in ra_spi.c.
 *
 ****************************************************************************/

uint8_t ra_spi_status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  /* For now, just call the SPI0 status function */
  return ra_spi0status(dev, devid);
}

/****************************************************************************
 * Name: ra_spi_cmddata
 *
 * Description:
 *   Board-specific SPI command/data function. This overrides the weak
 *   function in ra_spi.c.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int ra_spi_cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  /* For now, just call the SPI0 cmddata function */
  return ra_spi0cmddata(dev, devid, cmd);
}
#endif

/****************************************************************************
 * Name: ra_spi0select
 *
 * Description:
 *   Select or deselect the SPI device specified by 'devid' for SPI0.
 *   Enhanced for GY-912 sensor support with proper CS control
 *
 ****************************************************************************/

void ra_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                   bool selected)
{
  spiinfo("SPI0 devid: %" PRIu32 " CS: %s\n",
          devid, selected ? "assert" : "de-assert");

  /* Configure CS pins for GY-912 sensor modules */
  switch (devid)
    {
      case SPIDEV_IMU(0):
        /* ICM-20948 IMU sensor CS control */
        ra_gpiowrite(GPIO_IMU_CS, !selected);
        break;

      case SPIDEV_BAROMETER(0):
        /* BMP388 Barometer sensor CS control */
        ra_gpiowrite(GPIO_BMP_CS, !selected);
        break;

      default:
        spierr("SPI0 unknown device: %" PRIu32 "\n", devid);
        break;
    }
}

/****************************************************************************
 * Name: ra_spi0status
 *
 * Description:
 *   Return status information associated with the SPI0 device.
 *   Enhanced for GY-912 sensor support
 *
 ****************************************************************************/

uint8_t ra_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
      case SPIDEV_IMU(0):
        /* ICM-20948 IMU sensor */
        status = SPI_STATUS_PRESENT;
        spiinfo("SPI0 IMU sensor status: present\n");
        break;

      case SPIDEV_BAROMETER(0):
        /* BMP388 Barometer sensor */
        status = SPI_STATUS_PRESENT;
        spiinfo("SPI0 Barometer sensor status: present\n");
        break;

      default:
        spiinfo("SPI0 unknown device: %" PRIu32 "\n", devid);
        break;
    }

  return status;
}

/****************************************************************************
 * Name: ra_spi0cmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data. This is typically used for
 *   displays where the first byte is a command and subsequent bytes are data.
 *   For GY-912 sensors, this is typically not used.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int ra_spi0cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  switch (devid)
    {
      case SPIDEV_IMU(0):
      case SPIDEV_BAROMETER(0):
        /* GY-912 sensors don't typically use command/data distinction */
        return OK;

      default:
        return -ENODEV;
    }
}
#endif

#ifdef CONFIG_RA_SPI1
/****************************************************************************
 * Name: ra_spi1select
 *
 * Description:
 *   Select or deselect the SPI device specified by 'devid' for SPI1
 *
 ****************************************************************************/

void ra_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                   bool selected)
{
  spiinfo("SPI1 devid: %" PRIu32 " CS: %s\n",
          devid, selected ? "assert" : "de-assert");

  /* SPI1 device selection - add device-specific CS control as needed */
  switch (devid)
    {
      default:
        spierr("SPI1 unknown device: %" PRIu32 "\n", devid);
        break;
    }
}

/****************************************************************************
 * Name: ra_spi1status
 *
 * Description:
 *   Return status information associated with the SPI1 device.
 *
 ****************************************************************************/

uint8_t ra_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
      default:
        spiinfo("SPI1 unknown device: %" PRIu32 "\n", devid);
        break;
    }

  return status;
}

/****************************************************************************
 * Name: ra_spi1cmddata
 *
 * Description:
 *   Some SPI devices require an additional control to determine if the SPI
 *   data being sent is a command or is data.
 *
 ****************************************************************************/

#ifdef CONFIG_SPI_CMDDATA
int ra_spi1cmddata(FAR struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  switch (devid)
    {
      default:
        return -ENODEV;
    }
}
#endif

#endif /* CONFIG_RA_SPI1 */

#endif /* CONFIG_RA_SPI */
