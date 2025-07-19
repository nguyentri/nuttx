/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_spi_loopback_demo.c
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
#include "ra8e1_demo_log.h"

/* SPI Configuration */
#define SPI_LOOPBACK_BUFFER_SIZE    32
#define SPI_LOOPBACK_FREQUENCY      1000000  /* 1 MHz */
#define SPI_LOOPBACK_MODE           SPIDEV_MODE0

/* Test Commands */
#define SPI_TEST_WRITE_AND_READ     1
#define SPI_TEST_WRITE_READ         2
#define SPI_TEST_EXIT               3

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI buffer length - matching FSP example */
#define SPI_BUFF_LEN        32
#define SPI_FREQUENCY       1000000  /* 1 MHz */
#define SPI_MODE            SPIDEV_MODE0

/* Test patterns */
#define TEST_PATTERN_1      0x12345678
#define TEST_PATTERN_2      0xABCDEF00
#define TEST_PATTERN_3      0x55AA55AA
#define TEST_PATTERN_4      0xFF00FF00

/* Max wait count for synchronization */
#define MAX_WAIT_COUNT      1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spi_loopback_s
{
  FAR struct spi_dev_s *master;
  FAR struct spi_dev_s *slave;
  uint32_t master_tx_buff[SPI_BUFF_LEN];
  uint32_t master_rx_buff[SPI_BUFF_LEN];
  uint32_t slave_tx_buff[SPI_BUFF_LEN];
  uint32_t slave_rx_buff[SPI_BUFF_LEN];
  volatile bool master_complete;
  volatile bool slave_complete;
  volatile bool test_running;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_loopback_s g_spi_loopback;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_prepare_test_data
 *
 * Description:
 *   Prepare test data buffers with known patterns
 *
 ****************************************************************************/

static void spi_prepare_test_data(void)
{
  int i;

  /* Clear all buffers */
  memset(g_spi_loopback.master_tx_buff, 0, sizeof(g_spi_loopback.master_tx_buff));
  memset(g_spi_loopback.master_rx_buff, 0, sizeof(g_spi_loopback.master_rx_buff));
  memset(g_spi_loopback.slave_tx_buff, 0, sizeof(g_spi_loopback.slave_tx_buff));
  memset(g_spi_loopback.slave_rx_buff, 0, sizeof(g_spi_loopback.slave_rx_buff));

  /* Fill master TX buffer with test patterns */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      switch (i % 4)
        {
          case 0:
            g_spi_loopback.master_tx_buff[i] = TEST_PATTERN_1 + i;
            break;
          case 1:
            g_spi_loopback.master_tx_buff[i] = TEST_PATTERN_2 + i;
            break;
          case 2:
            g_spi_loopback.master_tx_buff[i] = TEST_PATTERN_3 + i;
            break;
          case 3:
            g_spi_loopback.master_tx_buff[i] = TEST_PATTERN_4 + i;
            break;
        }
    }

  /* Fill slave TX buffer with inverted patterns for loopback response */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      g_spi_loopback.slave_tx_buff[i] = ~g_spi_loopback.master_tx_buff[i];
    }

  spiinfo("Test data prepared: Master TX[0]=0x%08lx, Slave TX[0]=0x%08lx\n",
          g_spi_loopback.master_tx_buff[0], g_spi_loopback.slave_tx_buff[0]);
}

/****************************************************************************
 * Name: spi_verify_data
 *
 * Description:
 *   Verify received data against transmitted data
 *
 ****************************************************************************/

static int spi_verify_data(void)
{
  int i;
  int errors = 0;

  spiinfo("Verifying SPI loopback data...\n");

  /* Check if master TX data matches slave RX data */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      if (g_spi_loopback.master_tx_buff[i] != g_spi_loopback.slave_rx_buff[i])
        {
          spierr("Master TX / Slave RX mismatch at index %d: 0x%08lx != 0x%08lx\n",
                 i, g_spi_loopback.master_tx_buff[i], g_spi_loopback.slave_rx_buff[i]);
          errors++;
        }
    }

  /* Check if slave TX data matches master RX data */
  for (i = 0; i < SPI_BUFF_LEN; i++)
    {
      if (g_spi_loopback.slave_tx_buff[i] != g_spi_loopback.master_rx_buff[i])
        {
          spierr("Slave TX / Master RX mismatch at index %d: 0x%08lx != 0x%08lx\n",
                 i, g_spi_loopback.slave_tx_buff[i], g_spi_loopback.master_rx_buff[i]);
          errors++;
        }
    }

  if (errors == 0)
    {
      spiinfo("✓ SPI loopback test PASSED - all data verified successfully\n");
      return OK;
    }
  else
    {
      spierr("✗ SPI loopback test FAILED - %d data mismatches found\n", errors);
      return -EIO;
    }
}

/****************************************************************************
 * Name: spi_configure_devices
 *
 * Description:
 *   Configure SPI master and slave devices
 *
 ****************************************************************************/

static int spi_configure_devices(void)
{
  /* Configure SPI Master (SPI0) */
  SPI_LOCK(g_spi_loopback.master, true);
  SPI_SETMODE(g_spi_loopback.master, SPI_MODE);
  SPI_SETBITS(g_spi_loopback.master, 32);  /* 32-bit transfers */
  SPI_SETFREQUENCY(g_spi_loopback.master, SPI_FREQUENCY);
  SPI_LOCK(g_spi_loopback.master, false);

  /* Configure SPI Slave (SPI1) */
  SPI_LOCK(g_spi_loopback.slave, true);
  SPI_SETMODE(g_spi_loopback.slave, SPI_MODE);
  SPI_SETBITS(g_spi_loopback.slave, 32);   /* 32-bit transfers */
  SPI_SETFREQUENCY(g_spi_loopback.slave, SPI_FREQUENCY);
  SPI_LOCK(g_spi_loopback.slave, false);

  spiinfo("SPI devices configured: Master=%p, Slave=%p\n",
          g_spi_loopback.master, g_spi_loopback.slave);

  return OK;
}

/****************************************************************************
 * Name: spi_test_write_and_read
 *
 * Description:
 *   Test separate write and read operations (matching FSP example)
 *
 ****************************************************************************/

static int spi_test_write_and_read(void)
{
  int ret;
  
  spiinfo("Starting SPI write-and-read test...\n");
  
  /* Reset completion flags */
  g_spi_loopback.master_complete = false;
  g_spi_loopback.slave_complete = false;

  /* Step 1: Slave prepares to receive data from Master */
  SPI_LOCK(g_spi_loopback.slave, true);
  ret = SPI_RECVBLOCK(g_spi_loopback.slave, g_spi_loopback.slave_rx_buff, 
                      SPI_BUFF_LEN * sizeof(uint32_t));
  if (ret < 0)
    {
      spierr("Slave receive setup failed: %d\n", ret);
      SPI_LOCK(g_spi_loopback.slave, false);
      return ret;
    }
  SPI_LOCK(g_spi_loopback.slave, false);

  /* Step 2: Master sends data to Slave */
  SPI_LOCK(g_spi_loopback.master, true);
  ret = SPI_SNDBLOCK(g_spi_loopback.master, g_spi_loopback.master_tx_buff,
                     SPI_BUFF_LEN * sizeof(uint32_t));
  if (ret < 0)
    {
      spierr("Master send failed: %d\n", ret);
      SPI_LOCK(g_spi_loopback.master, false);
      return ret;
    }
  SPI_LOCK(g_spi_loopback.master, false);

  /* Small delay to ensure first transfer completes */
  usleep(10000);  /* 10ms */

  /* Step 3: Slave sends response data to Master */
  SPI_LOCK(g_spi_loopback.slave, true);
  ret = SPI_SNDBLOCK(g_spi_loopback.slave, g_spi_loopback.slave_tx_buff,
                     SPI_BUFF_LEN * sizeof(uint32_t));
  if (ret < 0)
    {
      spierr("Slave send failed: %d\n", ret);
      SPI_LOCK(g_spi_loopback.slave, false);
      return ret;
    }
  SPI_LOCK(g_spi_loopback.slave, false);

  /* Step 4: Master receives response from Slave */
  SPI_LOCK(g_spi_loopback.master, true);
  ret = SPI_RECVBLOCK(g_spi_loopback.master, g_spi_loopback.master_rx_buff,
                      SPI_BUFF_LEN * sizeof(uint32_t));
  if (ret < 0)
    {
      spierr("Master receive failed: %d\n", ret);
      SPI_LOCK(g_spi_loopback.master, false);
      return ret;
    }
  SPI_LOCK(g_spi_loopback.master, false);

  spiinfo("Write-and-read test completed successfully\n");
  return OK;
}

/****************************************************************************
 * Name: spi_test_write_read
 *
 * Description:
 *   Test simultaneous write/read operations (matching FSP example)
 *
 ****************************************************************************/

static int spi_test_write_read(void)
{
  int ret;
  
  spiinfo("Starting SPI write-read (simultaneous) test...\n");
  
  /* Reset completion flags */
  g_spi_loopback.master_complete = false;
  g_spi_loopback.slave_complete = false;

  /* Reset RX buffers for this test */
  memset(g_spi_loopback.master_rx_buff, 0, sizeof(g_spi_loopback.master_rx_buff));
  memset(g_spi_loopback.slave_rx_buff, 0, sizeof(g_spi_loopback.slave_rx_buff));

  /* Slave performs simultaneous write/read */
  SPI_LOCK(g_spi_loopback.slave, true);
  ret = SPI_EXCHANGE(g_spi_loopback.slave, g_spi_loopback.slave_tx_buff,
                     g_spi_loopback.slave_rx_buff, SPI_BUFF_LEN * sizeof(uint32_t));
  if (ret < 0)
    {
      spierr("Slave exchange setup failed: %d\n", ret);
      SPI_LOCK(g_spi_loopback.slave, false);
      return ret;
    }
  SPI_LOCK(g_spi_loopback.slave, false);

  /* Master performs simultaneous write/read */
  SPI_LOCK(g_spi_loopback.master, true);
  ret = SPI_EXCHANGE(g_spi_loopback.master, g_spi_loopback.master_tx_buff,
                     g_spi_loopback.master_rx_buff, SPI_BUFF_LEN * sizeof(uint32_t));
  if (ret < 0)
    {
      spierr("Master exchange failed: %d\n", ret);
      SPI_LOCK(g_spi_loopback.master, false);
      return ret;
    }
  SPI_LOCK(g_spi_loopback.master, false);

  spiinfo("Write-read (simultaneous) test completed successfully\n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_spi_loopback_demo_init
 *
 * Description:
 *   Initialize SPI loopback demo
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_init(void)
{
  int ret;

  spiinfo("Initializing SPI loopback demo...\n");

  /* Clear the demo structure */
  memset(&g_spi_loopback, 0, sizeof(g_spi_loopback));

  /* Get SPI0 as master */
  g_spi_loopback.master = ra8_spibus_initialize(0);
  if (!g_spi_loopback.master)
    {
      spierr("Failed to initialize SPI0 (master)\n");
      return -ENODEV;
    }

  /* Get SPI1 as slave */
  g_spi_loopback.slave = ra8_spibus_initialize(1);
  if (!g_spi_loopback.slave)
    {
      spierr("Failed to initialize SPI1 (slave)\n");
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
 * Name: ra8e1_spi_loopback_demo_test
 *
 * Description:
 *   Run SPI loopback tests
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_test(void)
{
  int ret;

  spiinfo("=== Starting SPI Loopback Demo Tests ===\n");

  if (!g_spi_loopback.master || !g_spi_loopback.slave)
    {
      spierr("SPI devices not initialized. Call ra8e1_spi_loopback_demo_init() first.\n");
      return -EINVAL;
    }

  /* Prepare test data */
  spi_prepare_test_data();

  /* Test 1: Write-and-Read (separate operations) */
  spiinfo("\n--- Test 1: Write-and-Read ---\n");
  ret = spi_test_write_and_read();
  if (ret < 0)
    {
      spierr("Write-and-read test failed: %d\n", ret);
      return ret;
    }

  /* Verify Test 1 results */
  ret = spi_verify_data();
  if (ret < 0)
    {
      return ret;
    }

  /* Prepare fresh test data for Test 2 */
  spi_prepare_test_data();

  /* Test 2: Write-Read (simultaneous operations) */
  spiinfo("\n--- Test 2: Write-Read (Simultaneous) ---\n");
  ret = spi_test_write_read();
  if (ret < 0)
    {
      spierr("Write-read test failed: %d\n", ret);
      return ret;
    }

  /* Verify Test 2 results */
  ret = spi_verify_data();
  if (ret < 0)
    {
      return ret;
    }

  spiinfo("\n=== SPI Loopback Demo Tests COMPLETED SUCCESSFULLY ===\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_spi_loopback_demo_main
 *
 * Description:
 *   Main entry point for SPI loopback demo
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_main(int argc, char *argv[])
{
  int ret;

  demoprintf("RA8E1 SPI Loopback Demo\n");
  demoprintf("=======================\n");
  demoprintf("This demo tests SPI communication between two SPI units:\n");
  demoprintf("- SPI0 as Master\n");
  demoprintf("- SPI1 as Slave\n");
  demoprintf("Tests both separate and simultaneous write/read operations.\n\n");

  /* Initialize the demo */
  ret = ra8e1_spi_loopback_demo_init();
  if (ret < 0)
    {
      demoprintf("Demo initialization failed: %d\n", ret);
      return ret;
    }

  /* Run the tests */
  ret = ra8e1_spi_loopback_demo_test();
  if (ret < 0)
    {
      demoprintf("Demo tests failed: %d\n", ret);
      return ret;
    }

  demoprintf("\nSPI Loopback Demo completed successfully!\n");
  return OK;
}
