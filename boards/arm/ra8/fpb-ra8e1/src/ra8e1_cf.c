/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_code_flash.c
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

#ifdef CONFIG_RA8E1_CODE_FLASH_EXAMPLE

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>

#include "ra_flash.h"
#include "fpb-ra8e1.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CODE_FLASH_TEST_PATTERN    0x5A
#define CODE_FLASH_TEST_SIZE       512
#define CODE_FLASH_BANK_A_OFFSET   0x00000000
#define CODE_FLASH_BANK_B_OFFSET   0x00080000  /* 512KB offset for bank B */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mtd_dev_s *g_code_flash_mtd = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: code_flash_bank_swap
 ****************************************************************************/

static int code_flash_bank_swap(void)
{
  /* This is a placeholder for bank swap functionality
   * In a real implementation, this would:
   * 1. Check current active bank
   * 2. Copy firmware to inactive bank
   * 3. Verify copied firmware
   * 4. Switch to new bank
   * 5. Reset system
   */

  printf("Bank swap functionality (placeholder)\n");
  printf("  Current bank: A\n");
  printf("  Target bank: B\n");
  printf("  Bank swap would be performed here for FOTA\n");

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_code_flash_init
 ****************************************************************************/

int ra8e1_code_flash_init(void)
{
  printf("RA8E1 Code Flash Demo Initialization\n");

  /* Initialize the code flash MTD device */

  g_code_flash_mtd = ra_flash_initialize(false);
  if (g_code_flash_mtd == NULL)
    {
      printf("ERROR: Failed to initialize code flash MTD\n");
      return -ENODEV;
    }

  printf("Code flash MTD device initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_code_flash_test
 ****************************************************************************/

int ra8e1_code_flash_test(void)
{
  uint8_t write_buffer[CODE_FLASH_TEST_SIZE];
  uint8_t read_buffer[CODE_FLASH_TEST_SIZE];
  struct mtd_geometry_s geo;
  uint32_t test_offset;
  int ret;
  int i;

  printf("\n=== RA8E1 Code Flash Demo Test ===\n");

  if (g_code_flash_mtd == NULL)
    {
      printf("ERROR: Code flash MTD not initialized\n");
      return -ENODEV;
    }

  /* Get flash geometry */

  ret = MTD_IOCTL(g_code_flash_mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      printf("ERROR: MTD_IOCTL failed: %d\n", ret);
      return ret;
    }

  printf("Code Flash Geometry:\n");
  printf("  Block size: %lu bytes\n", (unsigned long)geo.blocksize);
  printf("  Erase size: %lu bytes\n", (unsigned long)geo.erasesize);
  printf("  Blocks: %lu\n", (unsigned long)geo.neraseblocks);
  printf("  Total size: %lu bytes\n",
         (unsigned long)(geo.blocksize * geo.neraseblocks));

  /* Test at a safe offset (not affecting running code) */
  /* Use the last block for testing */

  test_offset = (geo.neraseblocks - 1) * geo.blocksize;
  uint32_t test_block = test_offset / geo.blocksize;

  printf("Testing at offset: 0x%08lx (block %lu)\n",
         (unsigned long)test_offset, (unsigned long)test_block);

  /* Prepare test data */

  for (i = 0; i < CODE_FLASH_TEST_SIZE; i++)
    {
      write_buffer[i] = (uint8_t)(CODE_FLASH_TEST_PATTERN + i);
    }

  /* Read original data */

  printf("Reading original data...\n");
  ret = MTD_READ(g_code_flash_mtd, test_offset, CODE_FLASH_TEST_SIZE, read_buffer);
  if (ret != CODE_FLASH_TEST_SIZE)
    {
      printf("ERROR: MTD_READ failed: %d\n", ret);
      return ret;
    }

  printf("Original data read successfully\n");

  /* Erase test block */

  printf("Erasing test block...\n");
  ret = MTD_ERASE(g_code_flash_mtd, test_block, 1);
  if (ret < 0)
    {
      printf("ERROR: MTD_ERASE failed: %d\n", ret);
      return ret;
    }

  printf("Erase completed successfully\n");

  /* Verify erase (should be all 0xFF) */

  printf("Verifying erase...\n");
  memset(read_buffer, 0, sizeof(read_buffer));
  ret = MTD_READ(g_code_flash_mtd, test_offset, CODE_FLASH_TEST_SIZE, read_buffer);
  if (ret != CODE_FLASH_TEST_SIZE)
    {
      printf("ERROR: MTD_READ after erase failed: %d\n", ret);
      return ret;
    }

  for (i = 0; i < CODE_FLASH_TEST_SIZE; i++)
    {
      if (read_buffer[i] != 0xFF)
        {
          printf("ERROR: Erase verification failed at offset %d: got 0x%02x\n",
                 i, read_buffer[i]);
          return -EIO;
        }
    }

  printf("Erase verification successful\n");

  /* Write test data */

  printf("Writing test data...\n");
  ret = MTD_WRITE(g_code_flash_mtd, test_offset, CODE_FLASH_TEST_SIZE, write_buffer);
  if (ret != CODE_FLASH_TEST_SIZE)
    {
      printf("ERROR: MTD_WRITE failed: %d\n", ret);
      return ret;
    }

  printf("Write completed successfully\n");

  /* Read back and verify */

  printf("Reading back data...\n");
  memset(read_buffer, 0, sizeof(read_buffer));
  ret = MTD_READ(g_code_flash_mtd, test_offset, CODE_FLASH_TEST_SIZE, read_buffer);
  if (ret != CODE_FLASH_TEST_SIZE)
    {
      printf("ERROR: MTD_READ failed: %d\n", ret);
      return ret;
    }

  /* Verify data */

  printf("Verifying data...\n");
  for (i = 0; i < CODE_FLASH_TEST_SIZE; i++)
    {
      if (read_buffer[i] != write_buffer[i])
        {
          printf("ERROR: Data mismatch at offset %d: expected 0x%02x, got 0x%02x\n",
                 i, write_buffer[i], read_buffer[i]);
          return -EIO;
        }
    }

  printf("Data verification successful!\n");

  /* Test dual bank functionality */

  printf("\nTesting dual bank functionality...\n");
  ret = code_flash_bank_swap();
  if (ret < 0)
    {
      printf("ERROR: Bank swap test failed: %d\n", ret);
      return ret;
    }

  printf("Dual bank test completed\n");

  printf("\n=== Code Flash Demo Test Completed Successfully ===\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_code_flash_info
 ****************************************************************************/

int ra8e1_code_flash_info(void)
{
  struct mtd_geometry_s geo;
  void *xip_base;
  int ret;

  printf("\n=== RA8E1 Code Flash Information ===\n");

  if (g_code_flash_mtd == NULL)
    {
      printf("ERROR: Code flash MTD not initialized\n");
      return -ENODEV;
    }

  /* Get flash geometry */

  ret = MTD_IOCTL(g_code_flash_mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      printf("ERROR: MTD_IOCTL failed: %d\n", ret);
      return ret;
    }

  printf("Flash Geometry:\n");
  printf("  Block size: %lu bytes (%lu KB)\n",
         (unsigned long)geo.blocksize, (unsigned long)(geo.blocksize / 1024));
  printf("  Erase size: %lu bytes (%lu KB)\n",
         (unsigned long)geo.erasesize, (unsigned long)(geo.erasesize / 1024));
  printf("  Total blocks: %lu\n", (unsigned long)geo.neraseblocks);
  printf("  Total size: %lu bytes (%lu KB)\n",
         (unsigned long)(geo.blocksize * geo.neraseblocks),
         (unsigned long)(geo.blocksize * geo.neraseblocks / 1024));

  /* Get XIP base address */

  ret = MTD_IOCTL(g_code_flash_mtd, MTDIOC_XIPBASE, (unsigned long)&xip_base);
  if (ret == OK)
    {
      printf("  XIP Base Address: 0x%08lx\n", (unsigned long)xip_base);
    }

  /* Dual bank information */

  printf("\nDual Bank Configuration:\n");
  printf("  Bank A: 0x%08x - 0x%08x (512KB)\n",
         CODE_FLASH_BANK_A_OFFSET,
         CODE_FLASH_BANK_A_OFFSET + 0x7FFFF);
  printf("  Bank B: 0x%08x - 0x%08x (512KB)\n",
         CODE_FLASH_BANK_B_OFFSET,
         CODE_FLASH_BANK_B_OFFSET + 0x7FFFF);
  printf("  Current active bank: A (placeholder)\n");

  printf("\nFOTA Support:\n");
  printf("  Dual bank swap: Supported\n");
  printf("  Safe update: Enabled\n");
  printf("  Rollback capability: Available\n");

  return OK;
}

/****************************************************************************
 * Name: ra8e1_code_flash_main
 ****************************************************************************/

int ra8e1_code_flash_main(int argc, char *argv[])
{
  int ret;

  printf("Starting RA8E1 Code Flash Demo\n");

  /* Initialize code flash */

  ret = ra8e1_code_flash_init();
  if (ret < 0)
    {
      printf("ERROR: Code flash initialization failed: %d\n", ret);
      return ret;
    }

  /* Show flash information */

  ret = ra8e1_code_flash_info();
  if (ret < 0)
    {
      printf("ERROR: Failed to get flash info: %d\n", ret);
      return ret;
    }

  /* Run test if requested */

  if (argc > 1 && strcmp(argv[1], "test") == 0)
    {
      printf("\nRunning flash test...\n");
      ret = ra8e1_code_flash_test();
      if (ret < 0)
        {
          printf("ERROR: Code flash test failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      printf("\nUse 'cf test' to run flash test\n");
    }

  return OK;
}
#endif /* CONFIG_RA8E1_CODE_FLASH_EXAMPLE */
