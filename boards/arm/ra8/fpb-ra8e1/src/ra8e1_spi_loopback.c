/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_spi_loopback.c
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

#ifdef CONFIG_RA8E1_SPI_LOOPBACK_EXAMPLE

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include <arch/board/board.h>
#include "ra_spi.h"
#include "ra_gpio.h"
#include "fpb-ra8e1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Configuration for loopback test */
#define SPI_LOOPBACK_BUFFER_SIZE    32
#define SPI_LOOPBACK_FREQUENCY      1000000  /* 1 MHz */
#define SPI_LOOPBACK_MODE           0        /* Mode 0 */

/* SPI buffer length for loopback test */
#define SPI_BUFF_LEN        32
#define SPI_FREQUENCY       1000000  /* 1 MHz */
#define SPI_MODE            0        /* Mode 0 */

/* Test patterns */
#define TEST_PATTERN_1      0x12345678
#define TEST_PATTERN_2      0xABCDEF00
#define TEST_PATTERN_3      0x55AA55AA
#define TEST_PATTERN_4      0xFF00FF00

/* SPI Device IDs for loopback test */
#define SPI_DEVICE_SPI0     0x0000
#define SPI_DEVICE_SPI1     0x0001

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_loopback_s
{
  FAR struct spi_dev_s *spi0;           /* SPI0 master */
  FAR struct spi_dev_s *spi1;           /* SPI1 master */
  uint8_t spi0_tx_buff[SPI_BUFF_LEN];   /* SPI0 transmit buffer */
  uint8_t spi0_rx_buff[SPI_BUFF_LEN];   /* SPI0 receive buffer */
  uint8_t spi1_tx_buff[SPI_BUFF_LEN];   /* SPI1 transmit buffer */
  uint8_t spi1_rx_buff[SPI_BUFF_LEN];   /* SPI1 receive buffer */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_loopback_s g_spi_loopback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_spi_select (strong override for loopback)
 *
 * Description:
 *   Enable/disable the SPI chip select for loopback test
 *   For loopback testing, CS is not used
 *
 ****************************************************************************/

void ra_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  /* No CS control needed for loopback testing */
  /* MOSI is connected directly to MISO for each SPI controller */
  UNUSED(dev);
  UNUSED(devid);
  UNUSED(selected);
}

/****************************************************************************
 * Name: ra_spi_status (strong override for loopback)
 *
 * Description:
 *   Return status information for loopback test
 *
 ****************************************************************************/

uint8_t ra_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  /* For loopback test, device is always present */
  UNUSED(dev);
  UNUSED(devid);
  return SPI_STATUS_PRESENT;
}

/****************************************************************************
 * Name: ra_spi_cmddata (strong override for loopback)
 *
 * Description:
 *   Control the SPI CMD/DATA GPIO for loopback test
 *
 ****************************************************************************/

int ra_spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  /* Loopback test doesn't use CMD/DATA line */
  return 0;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_prepare_test_data
 *
 * Description:
 *   Prepare test data buffers with known patterns for loopback test
 *
 ****************************************************************************/

static void spi_prepare_test_data(void)
{
  int i;

  /* Clear all buffers */
  memset(g_spi_loopback.spi0_tx_buff, 0, sizeof(g_spi_loopback.spi0_tx_buff));
  memset(g_spi_loopback.spi0_rx_buff, 0, sizeof(g_spi_loopback.spi0_rx_buff));
  memset(g_spi_loopback.spi1_tx_buff, 0, sizeof(g_spi_loopback.spi1_tx_buff));
  memset(g_spi_loopback.spi1_rx_buff, 0, sizeof(g_spi_loopback.spi1_rx_buff));

  /* Fill SPI0 TX buffer with test patterns */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      switch (i % 4)
        {
          case 0:
            g_spi_loopback.spi0_tx_buff[i] = (TEST_PATTERN_1 + i) & 0xFF;
            break;
          case 1:
            g_spi_loopback.spi0_tx_buff[i] = (TEST_PATTERN_2 + i) & 0xFF;
            break;
          case 2:
            g_spi_loopback.spi0_tx_buff[i] = (TEST_PATTERN_3 + i) & 0xFF;
            break;
          case 3:
            g_spi_loopback.spi0_tx_buff[i] = (TEST_PATTERN_4 + i) & 0xFF;
            break;
        }
    }

  /* Fill SPI1 TX buffer with different test patterns */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      switch (i % 4)
        {
          case 0:
            g_spi_loopback.spi1_tx_buff[i] = (~TEST_PATTERN_1 + i) & 0xFF;
            break;
          case 1:
            g_spi_loopback.spi1_tx_buff[i] = (~TEST_PATTERN_2 + i) & 0xFF;
            break;
          case 2:
            g_spi_loopback.spi1_tx_buff[i] = (~TEST_PATTERN_3 + i) & 0xFF;
            break;
          case 3:
            g_spi_loopback.spi1_tx_buff[i] = (~TEST_PATTERN_4 + i) & 0xFF;
            break;
        }
    }

  spiinfo("Test data prepared: SPI0 TX[0]=0x%02x, SPI1 TX[0]=0x%02x\n",
          g_spi_loopback.spi0_tx_buff[0], g_spi_loopback.spi1_tx_buff[0]);
}

/****************************************************************************
 * Name: spi_verify_loopback_data
 *
 * Description:
 *   Verify loopback data - TX data should equal RX data for each SPI
 *
 ****************************************************************************/

static int spi_verify_loopback_data(void)
{
  int i;
  int errors = 0;

  spiinfo("Verifying SPI loopback data...\n");

  /* Check SPI0: TX data should equal RX data (loopback) */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      if (g_spi_loopback.spi0_tx_buff[i] != g_spi_loopback.spi0_rx_buff[i])
        {
          spierr("SPI0 loopback mismatch at index %d: TX=0x%02x != RX=0x%02x\n",
                 i, g_spi_loopback.spi0_tx_buff[i], g_spi_loopback.spi0_rx_buff[i]);
          errors++;
        }
    }

  /* Check SPI1: TX data should equal RX data (loopback) */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      if (g_spi_loopback.spi1_tx_buff[i] != g_spi_loopback.spi1_rx_buff[i])
        {
          spierr("SPI1 loopback mismatch at index %d: TX=0x%02x != RX=0x%02x\n",
                 i, g_spi_loopback.spi1_tx_buff[i], g_spi_loopback.spi1_rx_buff[i]);
          errors++;
        }
    }

  if (errors == 0)
    {
      spiinfo("✓ SPI loopback test PASSED - all data verified successfully\n");
      spiinfo("  SPI0: %d bytes looped back correctly\n", SPI_BUFF_LEN);
      spiinfo("  SPI1: %d bytes looped back correctly\n", SPI_BUFF_LEN);
      return OK;
    }
  else
    {
      spierr("✗ SPI loopback test FAILED - %d errors found\n", errors);
      return -EIO;
    }
}

/****************************************************************************
 * Name: spi_configure_devices
 *
 * Description:
 *   Configure both SPI devices as masters for loopback testing
 *
 ****************************************************************************/

static int spi_configure_devices(void)
{
  /* Configure SPI0 as master */
  SPI_LOCK(g_spi_loopback.spi0, true);
  SPI_SETMODE(g_spi_loopback.spi0, SPI_MODE);
  SPI_SETBITS(g_spi_loopback.spi0, 8);   /* 8-bit transfers */
  SPI_SETFREQUENCY(g_spi_loopback.spi0, SPI_FREQUENCY);
  SPI_LOCK(g_spi_loopback.spi0, false);

  /* Configure SPI1 as master */
  SPI_LOCK(g_spi_loopback.spi1, true);
  SPI_SETMODE(g_spi_loopback.spi1, SPI_MODE);
  SPI_SETBITS(g_spi_loopback.spi1, 8);   /* 8-bit transfers */
  SPI_SETFREQUENCY(g_spi_loopback.spi1, SPI_FREQUENCY);
  SPI_LOCK(g_spi_loopback.spi1, false);

  spiinfo("SPI devices configured: SPI0=%p, SPI1=%p (both as masters)\n",
          g_spi_loopback.spi0, g_spi_loopback.spi1);

  return OK;
}

/****************************************************************************
 * Name: spi_test_loopback
 *
 * Description:
 *   Test SPI loopback where MOSI is connected to MISO for each SPI
 *
 ****************************************************************************/

static int spi_test_loopback(void)
{
  spiinfo("Starting SPI loopback test...\n");
  spiinfo("Hardware connections required:\n");
  spiinfo("  SPI0: Connect P609 (MOSI) to P610 (MISO)\n");
  spiinfo("  SPI1: Connect P411 (MOSI) to P410 (MISO)\n\n");

  /* Test SPI0 loopback */
  spiinfo("Testing SPI0 loopback (MOSI->MISO)...\n");
  SPI_LOCK(g_spi_loopback.spi0, true);
  SPI_EXCHANGE(g_spi_loopback.spi0, g_spi_loopback.spi0_tx_buff,
               g_spi_loopback.spi0_rx_buff, SPI_BUFF_LEN);
  SPI_LOCK(g_spi_loopback.spi0, false);

  /* Test SPI1 loopback */
  spiinfo("Testing SPI1 loopback (MOSI->MISO)...\n");
  SPI_LOCK(g_spi_loopback.spi1, true);
  SPI_EXCHANGE(g_spi_loopback.spi1, g_spi_loopback.spi1_tx_buff,
               g_spi_loopback.spi1_rx_buff, SPI_BUFF_LEN);
  SPI_LOCK(g_spi_loopback.spi1, false);

  spiinfo("Loopback transfers completed\n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_spi_loopback_init
 *
 * Description:
 *   Initialize SPI loopback demo
 *
 ****************************************************************************/

int ra8e1_spi_loopback_init(void)
{
  int ret;

  spiinfo("Initializing SPI loopback demo...\n");

  /* Clear the demo structure */
  memset(&g_spi_loopback, 0, sizeof(g_spi_loopback));

  /* Get SPI0 as master */
  g_spi_loopback.spi0 = ra_spibus_initialize(0);
  if (!g_spi_loopback.spi0)
    {
      spierr("Failed to initialize SPI0\n");
      return -ENODEV;
    }

  /* Get SPI1 as master */
  g_spi_loopback.spi1 = ra_spibus_initialize(1);
  if (!g_spi_loopback.spi1)
    {
      spierr("Failed to initialize SPI1\n");
      return -ENODEV;
    }

  /* Configure both SPI devices */
  ret = spi_configure_devices();
  if (ret < 0)
    {
      spierr("Failed to configure SPI devices: %d\n", ret);
      return ret;
    }

  spiinfo("SPI loopback demo initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_spi_loopback_test
 *
 * Description:
 *   Run SPI loopback tests
 *
 ****************************************************************************/

int ra8e1_spi_loopback_test(void)
{
  int ret;

  spiinfo("=== Starting SPI Loopback Test ===\n");

  if (!g_spi_loopback.spi0 || !g_spi_loopback.spi1)
    {
      spierr("SPI devices not initialized. Call ra8e1_spi_loopback_init() first.\n");
      return -EINVAL;
    }

  /* Prepare test data */
  spi_prepare_test_data();

  /* Run loopback test */
  ret = spi_test_loopback();
  if (ret < 0)
    {
      spierr("Loopback test failed: %d\n", ret);
      return ret;
    }

  /* Verify results */
  ret = spi_verify_loopback_data();
  if (ret < 0)
    {
      return ret;
    }

  spiinfo("\n=== SPI Loopback Test COMPLETED SUCCESSFULLY ===\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_spi_loopback_main
 *
 * Description:
 *   Main entry point for SPI loopback demo
 *
 ****************************************************************************/

int ra8e1_spi_loopback_main(int argc, char *argv[])
{
  int ret;

  syslog(LOG_INFO, "RA8E1 SPI Loopback Test\n");
  syslog(LOG_INFO, "=======================\n");
  syslog(LOG_INFO, "This test verifies SPI loopback functionality:\n");
  syslog(LOG_INFO, "- SPI0 and SPI1 both configured as masters\n");
  syslog(LOG_INFO, "- Hardware connections required:\n");
  syslog(LOG_INFO, "  * SPI0: Connect P609 (MOSI) to P610 (MISO)\n");
  syslog(LOG_INFO, "  * SPI1: Connect P411 (MOSI) to P410 (MISO)\n");
  syslog(LOG_INFO, "- Verification: TX data should equal RX data\n\n");

  /* Initialize the test */
  ret = ra8e1_spi_loopback_init();
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test initialization failed: %d\n", ret);
      return ret;
    }

  /* Run the test */
  ret = ra8e1_spi_loopback_test();
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "\nSPI Loopback Test completed successfully!\n");
  return OK;
}

#endif /* CONFIG_RA8E1_SPI_LOOPBACK_EXAMPLE */
