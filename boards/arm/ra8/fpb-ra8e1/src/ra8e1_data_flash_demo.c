/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_data_flash_demo.c
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
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/nxffs.h>

#include "ra_flash.h"
#include "ra8e1_data_flash_demo.h"
#include "ra8e1_demo_log.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DATA_FLASH_TEST_PATTERN    0xA5
#define DATA_FLASH_TEST_SIZE       256
#define DATA_FLASH_MOUNT_POINT     "/data"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mtd_dev_s *g_data_flash_mtd = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_data_flash_demo_init
 ****************************************************************************/

int ra8e1_data_flash_demo_init(void)
{
  int ret;

  demoprintf("RA8E1 Data Flash Demo Initialization\n");

  /* Initialize the data flash MTD device */

  g_data_flash_mtd = ra_flash_initialize(true);
  if (g_data_flash_mtd == NULL)
    {
      demoprintf("ERROR: Failed to initialize data flash MTD\n");
      return -ENODEV;
    }

  demoprintf("Data flash MTD device initialized successfully\n");

#ifdef CONFIG_FS_NXFFS
  /* Initialize NXFFS on the data flash */

  ret = nxffs_initialize(g_data_flash_mtd);
  if (ret < 0)
    {
      demoprintf("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  /* Mount the NXFFS file system */

  ret = nx_mount(NULL, DATA_FLASH_MOUNT_POINT, "nxffs", 0, NULL);
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to mount NXFFS: %d\n", ret);
      return ret;
    }

  demoprintf("NXFFS mounted on %s\n", DATA_FLASH_MOUNT_POINT);
#endif

  return OK;
}

/****************************************************************************
 * Name: ra8e1_data_flash_demo_test
 ****************************************************************************/

int ra8e1_data_flash_demo_test(void)
{
  uint8_t write_buffer[DATA_FLASH_TEST_SIZE];
  uint8_t read_buffer[DATA_FLASH_TEST_SIZE];
  struct mtd_geometry_s geo;
  int ret;
  int i;

  demoprintf("\n=== RA8E1 Data Flash Demo Test ===\n");

  if (g_data_flash_mtd == NULL)
    {
      demoprintf("ERROR: Data flash MTD not initialized\n");
      return -ENODEV;
    }

  /* Get flash geometry */

  ret = MTD_IOCTL(g_data_flash_mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      demoprintf("ERROR: MTD_IOCTL failed: %d\n", ret);
      return ret;
    }

  demoprintf("Data Flash Geometry:\n");
  demoprintf("  Block size: %lu bytes\n", (unsigned long)geo.blocksize);
  demoprintf("  Erase size: %lu bytes\n", (unsigned long)geo.erasesize);
  demoprintf("  Blocks: %lu\n", (unsigned long)geo.neraseblocks);
  demoprintf("  Total size: %lu bytes\n", 
         (unsigned long)(geo.blocksize * geo.neraseblocks));

  /* Prepare test data */

  for (i = 0; i < DATA_FLASH_TEST_SIZE; i++)
    {
      write_buffer[i] = (uint8_t)(DATA_FLASH_TEST_PATTERN + i);
    }

  /* Erase first block */

  demoprintf("\nErasing first block...\n");
  ret = MTD_ERASE(g_data_flash_mtd, 0, 1);
  if (ret < 0)
    {
      demoprintf("ERROR: MTD_ERASE failed: %d\n", ret);
      return ret;
    }

  demoprintf("Erase completed successfully\n");

  /* Write test data */

  demoprintf("Writing test data...\n");
  ret = MTD_WRITE(g_data_flash_mtd, 0, DATA_FLASH_TEST_SIZE, write_buffer);
  if (ret != DATA_FLASH_TEST_SIZE)
    {
      demoprintf("ERROR: MTD_WRITE failed: %d\n", ret);
      return ret;
    }

  demoprintf("Write completed successfully\n");

  /* Read back and verify */

  demoprintf("Reading back data...\n");
  memset(read_buffer, 0, sizeof(read_buffer));
  ret = MTD_READ(g_data_flash_mtd, 0, DATA_FLASH_TEST_SIZE, read_buffer);
  if (ret != DATA_FLASH_TEST_SIZE)
    {
      demoprintf("ERROR: MTD_READ failed: %d\n", ret);
      return ret;
    }

  /* Verify data */

  demoprintf("Verifying data...\n");
  for (i = 0; i < DATA_FLASH_TEST_SIZE; i++)
    {
      if (read_buffer[i] != write_buffer[i])
        {
          demoprintf("ERROR: Data mismatch at offset %d: expected 0x%02x, got 0x%02x\n",
                 i, write_buffer[i], read_buffer[i]);
          return -EIO;
        }
    }

  demoprintf("Data verification successful!\n");

#ifdef CONFIG_FS_NXFFS
  /* Test file system operations */

  demoprintf("\nTesting file system operations...\n");
  
  const char *test_file = DATA_FLASH_MOUNT_POINT "/test.txt";
  const char *test_data = "Hello, RA8E1 Data Flash!";
  char read_data[64];
  int fd;

  /* Write to file */

  fd = open(test_file, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0)
    {
      demoprintf("ERROR: Failed to open file for writing: %d\n", errno);
      return -errno;
    }

  ret = write(fd, test_data, strlen(test_data));
  if (ret != (int)strlen(test_data))
    {
      demoprintf("ERROR: Failed to write to file: %d\n", ret);
      close(fd);
      return ret;
    }

  close(fd);
  demoprintf("File write successful\n");

  /* Read from file */

  fd = open(test_file, O_RDONLY);
  if (fd < 0)
    {
      demoprintf("ERROR: Failed to open file for reading: %d\n", errno);
      return -errno;
    }

  memset(read_data, 0, sizeof(read_data));
  ret = read(fd, read_data, sizeof(read_data) - 1);
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to read from file: %d\n", ret);
      close(fd);
      return ret;
    }

  close(fd);

  if (strcmp(test_data, read_data) != 0)
    {
      demoprintf("ERROR: File data mismatch\n");
      demoprintf("  Expected: %s\n", test_data);
      demoprintf("  Got: %s\n", read_data);
      return -EIO;
    }

  demoprintf("File read successful: %s\n", read_data);
#endif

  demoprintf("\n=== Data Flash Demo Test Completed Successfully ===\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_data_flash_demo_main
 ****************************************************************************/

int ra8e1_data_flash_demo_main(int argc, char *argv[])
{
  int ret;

  demoprintf("Starting RA8E1 Data Flash Demo\n");

  /* Initialize data flash */

  ret = ra8e1_data_flash_demo_init();
  if (ret < 0)
    {
      demoprintf("ERROR: Data flash initialization failed: %d\n", ret);
      return ret;
    }

  /* Run test */

  ret = ra8e1_data_flash_demo_test();
  if (ret < 0)
    {
      demoprintf("ERROR: Data flash test failed: %d\n", ret);
      return ret;
    }

  return OK;
}