/****************************************************************************
 * arch/arm/src/ra8/ra_flash.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_FLASH_H
#define __ARCH_ARM_SRC_RA8_RA_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "hardware/ra_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash block sizes */

#define RA_FLASH_CODE_BLOCK_SIZE     8192   /* 8KB Code Flash block size */
#define RA_FLASH_DATA_BLOCK_SIZE     64     /* 64B Data Flash block size */

/* Flash memory areas - Updated to match RA8E1 FSP specification */

#define RA_FLASH_CODE_START         0x00000000
#define RA_FLASH_CODE_SIZE          0x00100000  /* 1MB Code Flash */
#define RA_FLASH_DATA_START         0x08000000  /* Data Flash starts at different address */
#define RA_FLASH_DATA_SIZE          0x00003000  /* 12KB Data Flash (RA8E1 has 12KB) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mtd_dev_s;  /* Forward declaration */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ra_flash_initialize
 *
 * Description:
 *   Initialize the Flash MTD device driver for either Code Flash or 
 *   Data Flash.
 *
 * Input Parameters:
 *   data_flash - true for Data Flash, false for Code Flash
 *
 * Returned Value:
 *   Pointer to MTD device structure on success; NULL on failure
 *
 ****************************************************************************/

struct mtd_dev_s *ra_flash_initialize(bool data_flash);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_FLASH_H */
