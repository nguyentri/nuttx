/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_spi_masterslave.c
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

#ifdef CONFIG_RA8E1_SPI_MASTERSLAVE_EXAMPLE

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <pthread.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include <arch/board/board.h>
#include "ra_spi.h"
#include "ra_gpio.h"
#include "fpb-ra8e1.h"

/* SPI Configuration for Master-Slave Communication */
#define SPI_MS_BUFFER_SIZE          32
#define SPI_MS_FREQUENCY            1000000  /* 1 MHz */
#define SPI_MS_MODE                 0        /* Mode 0 */

/* Test Commands and Patterns */
/* Test Commands and Patterns */
#define MS_CMD_PING                 0x01
#define MS_CMD_ECHO                 0x02
#define MS_CMD_DATA_TRANSFER        0x03
#define MS_CMD_STATUS               0x04

#define MS_RESP_ACK                 0xA0
#define MS_RESP_NAK                 0xF0

/* Test patterns */
#define TEST_PATTERN_1              0x12345678
#define TEST_PATTERN_2              0xABCDEF00
#define TEST_PATTERN_3              0x55AA55AA
#define TEST_PATTERN_4              0xFF00FF00

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI buffer length for master-slave communication */
#define SPI_BUFF_LEN               32
#define SPI_FREQUENCY              1000000  /* 1 MHz */
#define SPI_MODE                   0         /* Mode 0 */

/* Synchronization constants */
#define MAX_SYNC_RETRIES           100
#define SYNC_DELAY_MS              10

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Master-Slave communication structure */
struct spi_masterslave_s
{
  FAR struct spi_dev_s *master;        /* SPI0 as master */
  FAR struct spi_dev_s *slave;         /* SPI1 as slave */
  uint8_t master_tx_buff[SPI_BUFF_LEN];
  uint8_t master_rx_buff[SPI_BUFF_LEN];
  uint8_t slave_tx_buff[SPI_BUFF_LEN];
  uint8_t slave_rx_buff[SPI_BUFF_LEN];
  volatile bool master_ready;
  volatile bool slave_ready;
  volatile bool test_complete;
  pthread_t slave_thread;
  pthread_mutex_t sync_mutex;
  pthread_cond_t sync_cond;
};


/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device IDs for master-slave test */
#define MASTER_DEVICE_ID    0x1000
#define SLAVE_DEVICE_ID     0x2000

/* SPI device configurations for master-slave test */
static const struct ra_spi_cs_config_s g_masterslave_spi_devices[] =
{
  {
    .devid      = MASTER_DEVICE_ID, // SPI0 as Master
    .cs_gpio    = GPIO_SPI0_SS0,
    .frequency  = 1000000,
    .mode       = 0,
    .bits       = 8,
    .active_low = true,
    .name       = "SPI Master (SPI0)"
  },
  {
    .devid      = SLAVE_DEVICE_ID, // SPI1 as Slave
    .cs_gpio    = GPIO_SPI1_SSL0,
    .frequency  = 1000000,
    .mode       = 0,
    .bits       = 8,
    .active_low = true,
    .name       = "SPI Slave (SPI1)"
  }
};

#define NUM_MASTERSLAVE_DEVICES (sizeof(g_masterslave_spi_devices) / sizeof(g_masterslave_spi_devices[0]))

static struct spi_masterslave_s g_spi_ms;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: masterslave_spi_get_device_config
 *
 * Description:
 *   Get device configuration for master-slave test
 *
 ****************************************************************************/

static const struct masterslave_spi_config_s *
masterslave_spi_get_device_config(uint32_t devid)
{
  int i;

  for (i = 0; i < NUM_MASTERSLAVE_DEVICES; i++)
    {
      if (g_masterslave_spi_devices[i].devid == devid)
        {
          return &g_masterslave_spi_devices[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: ra_spi_select (strong override for master-slave)
 *
 * Description:
 *   Enable/disable the SPI chip select for master-slave test
 *
 ****************************************************************************/

void ra_spi_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  const struct masterslave_spi_config_s *config = masterslave_spi_get_device_config(devid);

  if (config)
    {
      if (config->active_low)
        {
          ra_gpiowrite(config->cs_gpio, !selected);
        }
      else
        {
          ra_gpiowrite(config->cs_gpio, selected);
        }
    }
}

/****************************************************************************
 * Name: ra_spi_status (strong override for master-slave)
 *
 * Description:
 *   Return status information associated with the SPI device for master-slave
 *
 ****************************************************************************/

uint8_t ra_spi_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  /* For master-slave test, devices are always present */
  status |= SPI_STATUS_PRESENT;

  return status;
}

/****************************************************************************
 * Name: ra_spi_cmddata (strong override for master-slave)
 *
 * Description:
 *   Control the SPI CMD/DATA GPIO for master-slave test
 *
 ****************************************************************************/

int ra_spi_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  /* Master-slave test doesn't use CMD/DATA line */
  return 0;
}

/****************************************************************************
 * Name: spi_ms_prepare_test_data
 *
 * Description:
 *   Prepare test data buffers for master-slave communication
 *
 ****************************************************************************/

static void spi_ms_prepare_test_data(void)
{
  int i;

  /* Clear all buffers */
  memset(g_spi_ms.master_tx_buff, 0, sizeof(g_spi_ms.master_tx_buff));
  memset(g_spi_ms.master_rx_buff, 0, sizeof(g_spi_ms.master_rx_buff));
  memset(g_spi_ms.slave_tx_buff, 0, sizeof(g_spi_ms.slave_tx_buff));
  memset(g_spi_ms.slave_rx_buff, 0, sizeof(g_spi_ms.slave_rx_buff));

  /* Fill master TX buffer with command and test data */
  g_spi_ms.master_tx_buff[0] = MS_CMD_DATA_TRANSFER;
  for (i = 1; i < SPI_BUFF_LEN; i++)
    {
      switch (i % 4)
        {
          case 1:
            g_spi_ms.master_tx_buff[i] = (TEST_PATTERN_1 >> (8 * (i % 4))) & 0xFF;
            break;
          case 2:
            g_spi_ms.master_tx_buff[i] = (TEST_PATTERN_2 >> (8 * (i % 4))) & 0xFF;
            break;
          case 3:
            g_spi_ms.master_tx_buff[i] = (TEST_PATTERN_3 >> (8 * (i % 4))) & 0xFF;
            break;
          case 0:
            g_spi_ms.master_tx_buff[i] = (TEST_PATTERN_4 >> (8 * (i % 4))) & 0xFF;
            break;
        }
    }

  /* Prepare slave response buffer */
  g_spi_ms.slave_tx_buff[0] = MS_RESP_ACK;
  for (i = 1; i < SPI_BUFF_LEN; i++)
    {
      g_spi_ms.slave_tx_buff[i] = ~g_spi_ms.master_tx_buff[i]; /* Invert pattern */
    }

  spiinfo("Master-Slave test data prepared\n");
}

/****************************************************************************
 * Name: spi_ms_slave_thread
 *
 * Description:
 *   Slave thread function that handles SPI1 slave operations
 *
 ****************************************************************************/

static FAR void *spi_ms_slave_thread(FAR void *arg)
{
  int retries = 0;

  spiinfo("SPI1 Slave thread started\n");

  /* Configure SPI1 as slave */
  SPI_LOCK(g_spi_ms.slave, true);
  SPI_SETMODE(g_spi_ms.slave, SPI_MODE);
  SPI_SETBITS(g_spi_ms.slave, 8);
  SPI_SETFREQUENCY(g_spi_ms.slave, SPI_FREQUENCY);
  SPI_LOCK(g_spi_ms.slave, false);

  /* Signal slave is ready */
  pthread_mutex_lock(&g_spi_ms.sync_mutex);
  g_spi_ms.slave_ready = true;
  pthread_cond_signal(&g_spi_ms.sync_cond);
  pthread_mutex_unlock(&g_spi_ms.sync_mutex);

  /* Wait for master to initiate communication */
  while (!g_spi_ms.test_complete && retries < MAX_SYNC_RETRIES)
    {
      /* Slave listens for incoming data */
      SPI_LOCK(g_spi_ms.slave, true);
      SPI_EXCHANGE(g_spi_ms.slave, g_spi_ms.slave_tx_buff,
                   g_spi_ms.slave_rx_buff, SPI_BUFF_LEN);
      SPI_LOCK(g_spi_ms.slave, false);

      /* Check if we received valid command */
      if (g_spi_ms.slave_rx_buff[0] == MS_CMD_DATA_TRANSFER)
        {
          spiinfo("Slave received command: 0x%02x\n", g_spi_ms.slave_rx_buff[0]);
          break;
        }

      usleep(SYNC_DELAY_MS * 1000);
      retries++;
    }

  spiinfo("SPI1 Slave thread completed\n");
  return NULL;
}

/****************************************************************************
 * Name: spi_ms_verify_data
 *
 * Description:
 *   Verify master-slave communication data
 *
 ****************************************************************************/

static int spi_ms_verify_data(void)
{
  int i;
  int errors = 0;

  spiinfo("Verifying SPI master-slave communication...\n");

  /* Verify command was received correctly */
  if (g_spi_ms.slave_rx_buff[0] != MS_CMD_DATA_TRANSFER)
    {
      spierr("Command verification failed: expected 0x%02x, got 0x%02x\n",
             MS_CMD_DATA_TRANSFER, g_spi_ms.slave_rx_buff[0]);
      errors++;
    }

  /* Verify master received slave response */
  if (g_spi_ms.master_rx_buff[0] != MS_RESP_ACK)
    {
      spierr("Response verification failed: expected 0x%02x, got 0x%02x\n",
             MS_RESP_ACK, g_spi_ms.master_rx_buff[0]);
      errors++;
    }

  /* Check data integrity (skip first byte which is command/response) */
  for (i = 1; i < SPI_BUFF_LEN; i++)
    {
      if (g_spi_ms.master_tx_buff[i] != g_spi_ms.slave_rx_buff[i])
        {
          spierr("Master->Slave data mismatch at index %d: 0x%02x != 0x%02x\n",
                 i, g_spi_ms.master_tx_buff[i], g_spi_ms.slave_rx_buff[i]);
          errors++;
        }

      if (g_spi_ms.slave_tx_buff[i] != g_spi_ms.master_rx_buff[i])
        {
          spierr("Slave->Master data mismatch at index %d: 0x%02x != 0x%02x\n",
                 i, g_spi_ms.slave_tx_buff[i], g_spi_ms.master_rx_buff[i]);
          errors++;
        }
    }

  if (errors == 0)
    {
      spiinfo("✓ SPI Master-Slave test PASSED - all data verified successfully\n");
      return OK;
    }
  else
    {
      spierr("✗ SPI Master-Slave test FAILED - %d errors found\n", errors);
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
  spiinfo("Starting SPI write-and-read test...\n");

  /* Reset completion flags */
  g_spi_loopback.master_complete = false;
  g_spi_loopback.slave_complete = false;

  /* Step 1: Slave prepares to receive data from Master */
  SPI_LOCK(g_spi_loopback.slave, true);
  SPI_RECVBLOCK(g_spi_loopback.slave, g_spi_loopback.slave_rx_buff,
                SPI_BUFF_LEN * sizeof(uint32_t));
  SPI_LOCK(g_spi_loopback.slave, false);

  /* Step 2: Master sends data to Slave */
  SPI_LOCK(g_spi_loopback.master, true);
  SPI_SNDBLOCK(g_spi_loopback.master, g_spi_loopback.master_tx_buff,
               SPI_BUFF_LEN * sizeof(uint32_t));
  SPI_LOCK(g_spi_loopback.master, false);

  /* Small delay to ensure first transfer completes */
  usleep(10000);  /* 10ms */

  /* Step 3: Slave sends response data to Master */
  SPI_LOCK(g_spi_loopback.slave, true);
  SPI_SNDBLOCK(g_spi_loopback.slave, g_spi_loopback.slave_tx_buff,
               SPI_BUFF_LEN * sizeof(uint32_t));
  SPI_LOCK(g_spi_loopback.slave, false);

  /* Step 4: Master receives response from Slave */
  SPI_LOCK(g_spi_loopback.master, true);
  SPI_RECVBLOCK(g_spi_loopback.master, g_spi_loopback.master_rx_buff,
                SPI_BUFF_LEN * sizeof(uint32_t));
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
  spiinfo("Starting SPI write-read (simultaneous) test...\n");

  /* Reset completion flags */
  g_spi_loopback.master_complete = false;
  g_spi_loopback.slave_complete = false;

  /* Reset RX buffers for this test */
  memset(g_spi_loopback.master_rx_buff, 0, sizeof(g_spi_loopback.master_rx_buff));
  memset(g_spi_loopback.slave_rx_buff, 0, sizeof(g_spi_loopback.slave_rx_buff));

  /* Slave performs simultaneous write/read */
  SPI_LOCK(g_spi_loopback.slave, true);
  SPI_EXCHANGE(g_spi_loopback.slave, g_spi_loopback.slave_tx_buff,
               g_spi_loopback.slave_rx_buff, SPI_BUFF_LEN * sizeof(uint32_t));
  SPI_LOCK(g_spi_loopback.slave, false);

  /* Master performs simultaneous write/read */
  SPI_LOCK(g_spi_loopback.master, true);
  SPI_EXCHANGE(g_spi_loopback.master, g_spi_loopback.master_tx_buff,
               g_spi_loopback.master_rx_buff, SPI_BUFF_LEN * sizeof(uint32_t));
  SPI_LOCK(g_spi_loopback.master, false);

  spiinfo("Write-read (simultaneous) test completed successfully\n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_spi_masterslave_init
 *
 * Description:
 *   Initialize SPI master-slave communication demo
 *
 ****************************************************************************/

int ra8e1_spi_masterslave_init(void)
{
  int ret;

  spiinfo("Initializing SPI Master-Slave demo...\n");

  /* Clear the demo structure */
  memset(&g_spi_ms, 0, sizeof(g_spi_ms));

  /* Initialize synchronization primitives */
  ret = pthread_mutex_init(&g_spi_ms.sync_mutex, NULL);
  if (ret != 0)
    {
      spierr("Failed to initialize mutex: %d\n", ret);
      return -EINVAL;
    }

  ret = pthread_cond_init(&g_spi_ms.sync_cond, NULL);
  if (ret != 0)
    {
      spierr("Failed to initialize condition variable: %d\n", ret);
      pthread_mutex_destroy(&g_spi_ms.sync_mutex);
      return -EINVAL;
    }

  /* Get SPI0 as master */
  g_spi_ms.master = ra_spibus_initialize(0);
  if (!g_spi_ms.master)
    {
      spierr("Failed to initialize SPI0 (master)\n");
      goto cleanup;
    }

  /* Get SPI1 as slave */
  g_spi_ms.slave = ra_spibus_initialize(1);
  if (!g_spi_ms.slave)
    {
      spierr("Failed to initialize SPI1 (slave)\n");
      goto cleanup;
    }

  /* Configure SPI Master (SPI0) */
  SPI_LOCK(g_spi_ms.master, true);
  SPI_SETMODE(g_spi_ms.master, SPI_MODE);
  SPI_SETBITS(g_spi_ms.master, 8);
  SPI_SETFREQUENCY(g_spi_ms.master, SPI_FREQUENCY);
  SPI_LOCK(g_spi_ms.master, false);

  spiinfo("SPI Master-Slave demo initialized successfully\n");
  return OK;

cleanup:
  pthread_cond_destroy(&g_spi_ms.sync_cond);
  pthread_mutex_destroy(&g_spi_ms.sync_mutex);
  return -ENODEV;
}

/****************************************************************************
 * Name: ra8e1_spi_masterslave_test
 *
 * Description:
 *   Run SPI master-slave communication tests
 *
 ****************************************************************************/

int ra8e1_spi_masterslave_test(void)
{
  pthread_attr_t attr;
  struct timespec timeout;
  int ret;

  spiinfo("=== Starting SPI Master-Slave Communication Test ===\n");

  if (!g_spi_ms.master || !g_spi_ms.slave)
    {
      spierr("SPI devices not initialized. Call ra8e1_spi_masterslave_init() first.\n");
      return -EINVAL;
    }

  /* Prepare test data */
  spi_ms_prepare_test_data();

  /* Initialize thread attributes */
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 2048);

  /* Start slave thread */
  ret = pthread_create(&g_spi_ms.slave_thread, &attr, spi_ms_slave_thread, NULL);
  if (ret != 0)
    {
      spierr("Failed to create slave thread: %d\n", ret);
      return -EINVAL;
    }

  /* Wait for slave to be ready */
  pthread_mutex_lock(&g_spi_ms.sync_mutex);
  clock_gettime(CLOCK_REALTIME, &timeout);
  timeout.tv_sec += 5; /* 5 second timeout */

  while (!g_spi_ms.slave_ready)
    {
      ret = pthread_cond_timedwait(&g_spi_ms.sync_cond, &g_spi_ms.sync_mutex, &timeout);
      if (ret == ETIMEDOUT)
        {
          pthread_mutex_unlock(&g_spi_ms.sync_mutex);
          spierr("Timeout waiting for slave to be ready\n");
          g_spi_ms.test_complete = true;
          pthread_join(g_spi_ms.slave_thread, NULL);
          return -ETIMEDOUT;
        }
    }
  pthread_mutex_unlock(&g_spi_ms.sync_mutex);

  spiinfo("Both master and slave are ready, starting communication...\n");

  /* Small delay to ensure slave is listening */
  usleep(100000); /* 100ms */

  /* Master initiates communication */
  SPI_LOCK(g_spi_ms.master, true);
  SPI_EXCHANGE(g_spi_ms.master, g_spi_ms.master_tx_buff,
               g_spi_ms.master_rx_buff, SPI_BUFF_LEN);
  SPI_LOCK(g_spi_ms.master, false);

  spiinfo("Master completed data exchange\n");

  /* Signal test completion and wait for slave thread */
  g_spi_ms.test_complete = true;
  pthread_join(g_spi_ms.slave_thread, NULL);

  /* Verify the communication results */
  ret = spi_ms_verify_data();
  if (ret < 0)
    {
      return ret;
    }

  spiinfo("\n=== SPI Master-Slave Communication Test COMPLETED SUCCESSFULLY ===\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_spi_masterslave_main
 *
 * Description:
 *   Main entry point for SPI master-slave communication demo
 *
 ****************************************************************************/

int ra8e1_spi_masterslave_main(int argc, char *argv[])
{
  int ret;

  syslog(LOG_INFO, "RA8E1 SPI Master-Slave Communication Demo\n");
  syslog(LOG_INFO, "=========================================\n");
  syslog(LOG_INFO, "This demo tests SPI communication between:\n");
  syslog(LOG_INFO, "- SPI0 as Master\n");
  syslog(LOG_INFO, "- SPI1 as Slave\n");
  syslog(LOG_INFO, "Communication uses command/response protocol with data verification.\n\n");

  /* Initialize the demo */
  ret = ra8e1_spi_masterslave_init();
  if (ret < 0)
    {
      syslog(LOG_INFO, "Demo initialization failed: %d\n", ret);
      return ret;
    }

  /* Run the tests */
  ret = ra8e1_spi_masterslave_test();
  if (ret < 0)
    {
      syslog(LOG_INFO, "Demo tests failed: %d\n", ret);
      return ret;
    }

  /* Cleanup */
  pthread_cond_destroy(&g_spi_ms.sync_cond);
  pthread_mutex_destroy(&g_spi_ms.sync_mutex);

  syslog(LOG_INFO, "\nSPI Master-Slave Communication Demo completed successfully!\n");
  return OK;
}

#endif /* CONFIG_RA8E1_SPI_MASTERSLAVE_EXAMPLE */
