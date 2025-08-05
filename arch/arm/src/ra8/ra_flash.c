/****************************************************************************
 * arch/arm/src/ra8/ra_flash.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/ioctl.h>

#include "arm_internal.h"
#include "ra_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_TIMEOUT_MS                10000
#define FLASH_KEY_CODE                  0x0000FFFFU

/* Return codes */
#define OK                              0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_flash_dev_s
{
  struct mtd_dev_s mtd;        /* MTD interface */
  uint32_t base;               /* Flash base address */
  uint32_t size;               /* Flash size */
  uint32_t blocksize;          /* Block size */
  uint32_t nblocks;            /* Number of blocks */
  bool     data_flash;         /* True if data flash, false if code flash */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks);
static ssize_t ra_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, uint8_t *buffer);
static ssize_t ra_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, const uint8_t *buffer);
static ssize_t ra_flash_read(struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, uint8_t *buffer);
static ssize_t ra_flash_write(struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, const uint8_t *buffer);
static int ra_flash_ioctl(struct mtd_dev_s *dev, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Code Flash MTD device */

static struct ra_flash_dev_s g_code_flash =
{
  .mtd =
  {
    .erase  = ra_flash_erase,
    .bread  = ra_flash_bread,
    .bwrite = ra_flash_bwrite,
    .read   = ra_flash_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    .write  = ra_flash_write,
#endif
    .ioctl  = ra_flash_ioctl,
  },
  .base       = RA_FLASH_CODE_START,
  .size       = RA_FLASH_CODE_SIZE,
  .blocksize  = RA_FLASH_CODE_BLOCK_SIZE,
  .nblocks    = RA_FLASH_CODE_SIZE / RA_FLASH_CODE_BLOCK_SIZE,
  .data_flash = false,
};

/* Data Flash MTD device */

static struct ra_flash_dev_s g_data_flash =
{
  .mtd =
  {
    .erase  = ra_flash_erase,
    .bread  = ra_flash_bread,
    .bwrite = ra_flash_bwrite,
    .read   = ra_flash_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    .write  = ra_flash_write,
#endif
    .ioctl  = ra_flash_ioctl,
  },
  .base       = RA_FLASH_DATA_START,
  .size       = RA_FLASH_DATA_SIZE,
  .blocksize  = RA_FLASH_DATA_BLOCK_SIZE,
  .nblocks    = RA_FLASH_DATA_SIZE / RA_FLASH_DATA_BLOCK_SIZE,
  .data_flash = true,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_flash_wait_ready
 ****************************************************************************/

static int ra_flash_wait_ready(void)
{
  uint32_t timeout = FLASH_TIMEOUT_MS * 1000;  /* Convert to microseconds */
  uint32_t status;

  while (timeout > 0)
    {
      status = getreg32(RA8_FLASH_FSTATR);
      if (status & FLASH_FSTATR_FRDY)
        {
          return OK;
        }

      up_udelay(1);
      timeout--;
    }

  ferr("Flash timeout waiting for ready\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: ra_flash_enter_pe_mode
 ****************************************************************************/

static int ra_flash_enter_pe_mode(bool data_flash)
{
  uint32_t entry_bit;
  int ret;

  /* Wait for flash to be ready */

  ret = ra_flash_wait_ready();
  if (ret < 0)
    {
      return ret;
    }

  /* Set entry bit based on flash type */

  entry_bit = data_flash ? FLASH_FENTRYR_FENTRYD : FLASH_FENTRYR_FENTRYC;

  /* Enter P/E mode */

  putreg32(FLASH_KEY_CODE | entry_bit, RA8_FLASH_FENTRYR);

  /* Wait for entry to complete */

  ret = ra_flash_wait_ready();
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_flash_exit_pe_mode
 ****************************************************************************/

static int ra_flash_exit_pe_mode(bool data_flash)
{
  int ret;

  /* Exit P/E mode */

  putreg32(FLASH_KEY_CODE, RA8_FLASH_FENTRYR);

  /* Wait for exit to complete */

  ret = ra_flash_wait_ready();
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_flash_erase_block
 ****************************************************************************/

static int ra_flash_erase_block(struct ra_flash_dev_s *priv, uint32_t addr)
{
  int ret;

  /* Enter P/E mode */

  ret = ra_flash_enter_pe_mode(priv->data_flash);
  if (ret < 0)
    {
      return ret;
    }

  /* Set erase address */

  putreg32(addr, RA8_FLASH_FSADDR);

  /* Issue erase command */

  putreg16(FLASH_CMD_BLOCK_ERASE, addr);

  /* Wait for operation to complete */

  ret = ra_flash_wait_ready();
  if (ret < 0)
    {
      goto exit;
    }

  /* Check for errors */

  uint32_t status = getreg32(RA8_FLASH_FSTATR);
  if (status & (FLASH_FSTATR_ILGLERR | FLASH_FSTATR_ERSERR))
    {
      ferr("Flash erase error: 0x%08" PRIx32 "\n", status);
      ret = -EIO;
    }

exit:
  /* Exit P/E mode */

  ra_flash_exit_pe_mode(priv->data_flash);
  return ret;
}

/****************************************************************************
 * Name: ra_flash_program_page
 ****************************************************************************/

static int ra_flash_program_page(struct ra_flash_dev_s *priv, uint32_t addr,
                                 const uint8_t *buffer, size_t len)
{
  int ret;
  size_t i;

  /* Enter P/E mode */

  ret = ra_flash_enter_pe_mode(priv->data_flash);
  if (ret < 0)
    {
      return ret;
    }

  /* Set program address */

  putreg32(addr, RA8_FLASH_FSADDR);

  /* Issue program command */

  putreg16(FLASH_CMD_PROGRAM, addr);

  /* Program data */

  for (i = 0; i < len; i += 2)
    {
      uint16_t data = buffer[i] | (buffer[i + 1] << 8);
      putreg16(data, addr + i);

      /* Wait for buffer to be ready */

      while ((getreg32(RA8_FLASH_FSTATR) & FLASH_FSTATR_DBFULL) != 0)
        {
          /* Wait */
        }
    }

  /* Issue confirm command */

  putreg16(0xD0, addr);

  /* Wait for operation to complete */

  ret = ra_flash_wait_ready();
  if (ret < 0)
    {
      goto exit;
    }

  /* Check for errors */

  uint32_t status = getreg32(RA8_FLASH_FSTATR);
  if (status & (FLASH_FSTATR_ILGLERR | FLASH_FSTATR_PRGERR))
    {
      ferr("Flash program error: 0x%08" PRIx32 "\n", status);
      ret = -EIO;
    }

exit:
  /* Exit P/E mode */

  ra_flash_exit_pe_mode(priv->data_flash);
  return ret;
}

/****************************************************************************
 * Name: ra_flash_erase
 ****************************************************************************/

static int ra_flash_erase(struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks)
{
  struct ra_flash_dev_s *priv = (struct ra_flash_dev_s *)dev;
  size_t blocksleft = nblocks;
  uint32_t addr;
  int ret = OK;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* Lock access to the flash */

  while (blocksleft-- > 0)
    {
      addr = priv->base + startblock * priv->blocksize;

      /* Erase the block */

      ret = ra_flash_erase_block(priv, addr);
      if (ret < 0)
        {
          ferr("ra_flash_erase_block failed: %d\n", ret);
          goto errout;
        }

      startblock++;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: ra_flash_bread
 ****************************************************************************/

static ssize_t ra_flash_bread(struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, uint8_t *buffer)
{
  struct ra_flash_dev_s *priv = (struct ra_flash_dev_s *)dev;
  ssize_t nbytes;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can use the normal memory read operation */

  nbytes = nblocks * priv->blocksize;
  memcpy(buffer, (void *)(priv->base + startblock * priv->blocksize), nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: ra_flash_bwrite
 ****************************************************************************/

static ssize_t ra_flash_bwrite(struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, const uint8_t *buffer)
{
  struct ra_flash_dev_s *priv = (struct ra_flash_dev_s *)dev;
  size_t blocksleft = nblocks;
  uint32_t addr;
  int ret;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  while (blocksleft-- > 0)
    {
      addr = priv->base + startblock * priv->blocksize;

      /* Program the block */

      ret = ra_flash_program_page(priv, addr, buffer, priv->blocksize);
      if (ret < 0)
        {
          ferr("ra_flash_program_page failed: %d\n", ret);
          return ret;
        }

      startblock++;
      buffer += priv->blocksize;
    }

  return nblocks;
}

/****************************************************************************
 * Name: ra_flash_read
 ****************************************************************************/

static ssize_t ra_flash_read(struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, uint8_t *buffer)
{
  struct ra_flash_dev_s *priv = (struct ra_flash_dev_s *)dev;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Bounds check */

  if (offset + nbytes > priv->size)
    {
      ferr("Read beyond end of flash\n");
      return -EINVAL;
    }

  /* Read the data */

  memcpy(buffer, (void *)(priv->base + offset), nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: ra_flash_write
 ****************************************************************************/
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t ra_flash_write(struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, const uint8_t *buffer)
{
  struct ra_flash_dev_s *priv = (struct ra_flash_dev_s *)dev;
  uint32_t addr;
  size_t remaining = nbytes;
  size_t written = 0;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  /* Bounds check */

  if (offset + nbytes > priv->size)
    {
      ferr("Write beyond end of flash\n");
      return -EINVAL;
    }

  while (remaining > 0)
    {
      addr = priv->base + offset;
      size_t chunk = (remaining > priv->blocksize) ? priv->blocksize : remaining;

      /* Program the chunk */

      ret = ra_flash_program_page(priv, addr, buffer, chunk);
      if (ret < 0)
        {
          ferr("ra_flash_program_page failed: %d\n", ret);
          return ret;
        }

      offset += chunk;
      buffer += chunk;
      written += chunk;
      remaining -= chunk;
    }

  return written;
}
#endif /* CONFIG_MTD_BYTE_WRITE */

/****************************************************************************
 * Name: ra_flash_ioctl
 ****************************************************************************/

static int ra_flash_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct ra_flash_dev_s *priv = (struct ra_flash_dev_s *)dev;
  int ret = -EINVAL;

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo = (struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->blocksize;
              geo->neraseblocks = priv->nblocks;
              ret = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = ra_flash_erase(dev, 0, priv->nblocks);
        }
        break;

      case BIOC_XIPBASE:
        {
          void **ppv = (void**)((uintptr_t)arg);

          if (ppv)
            {
              /* Return the base address of the flash memory */

              *ppv = (void*)priv->base;
              ret = OK;
            }
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_flash_initialize
 ****************************************************************************/

struct mtd_dev_s *ra_flash_initialize(bool data_flash)
{
  struct ra_flash_dev_s *priv;

  finfo("data_flash: %d\n", data_flash);

  /* Select the appropriate flash device */

  if (data_flash)
    {
      priv = &g_data_flash;
    }
  else
    {
      priv = &g_code_flash;
    }

  /* Initialize flash controller */

  /* Disable flash cache if needed */

  /* Set up flash access mode */

  /* Clear any error flags */

  putreg32(FLASH_CMD_STATUS_CLEAR, RA8_FLASH_FCMDR);

  return (struct mtd_dev_s *)priv;
}