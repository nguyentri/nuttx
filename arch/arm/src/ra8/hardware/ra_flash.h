/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_flash.h
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

#ifndef __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8E1_FLASH_H
#define __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8E1_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/ra8/chip.h>
#include "ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Flash Cache Register Offsets */
#define R_FCACHE_FCACHEE_OFFSET           0x0000 /* Flash Cache Enable Register (16-Bits) */
#define R_FCACHE_FCACHEIV_OFFSET          0x0004 /* Flash Cache Invalidate Register (16-Bits) */
#define R_FCACHE_FLWT_OFFSET              0x001C /* Flash Cache FLWT (8-Bits) */

/* Flash Access Control Interface (FACI) Register Offsets */
#define R_FACI_HP_FSADDR_OFFSET           0x0030 /* Flash Start Address Register */
#define R_FACI_HP_FSTATR_OFFSET           0x0080 /* Flash Status Register */
#define R_FACI_HP_FENTRYR_OFFSET          0x0084 /* Program/Erase Mode Entry Register */
#define R_FACI_HP_FCMDR_OFFSET            0x00A0 /* Flash Command Register */

/* Register Addresses *******************************************************/

/* Flash Cache Registers */
# define R_FCACHE_FCACHEE                 (R_FCACHE_BASE + R_FCACHE_FCACHEE_OFFSET)
# define R_FCACHE_FCACHEIV                (R_FCACHE_BASE + R_FCACHE_FCACHEIV_OFFSET)
# define R_FCACHE_FLWT                    (R_FCACHE_BASE + R_FCACHE_FLWT_OFFSET)

/* Flash Access Control Interface (FACI) Registers */
# define R_FLASH_FSADDR                 (R_FACI_HP_BASE + R_FACI_HP_FSADDR_OFFSET)
# define R_FLASH_FSTATR                 (R_FACI_HP_BASE + R_FACI_HP_FSTATR_OFFSET)
# define R_FLASH_FENTRYR                (R_FACI_HP_BASE + R_FACI_HP_FENTRYR_OFFSET)
# define R_FLASH_FCMDR                  (R_FACI_HP_BASE + R_FACI_HP_FCMDR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Flash Cache Enable Register (16-Bits) */

#define R_FCACHE_FCACHEE_FCACHEEN         (1 <<  0) /* 01: FCACHE Enable */

/* Flash Cache Invalidate Register (16-Bits) */

#define R_FCACHE_FCACHEIV_FCACHEIV        (1 <<  0) /* 01: FCACHE Invalidation */

/* Flash Cache FLWT (8-Bits) */

#define R_FCACHE_FLWT_FLWT                (3 <<  0) /* 01: These bits represent the ratio of the CPU clock period to the Flash memory access time. */
#define R_FCACHE_FLWT_FLWT_MASK           (0x07)

/* Flash Status Register (FSTATR) Bitfield Definitions */

#define FLASH_FSTATR_FRDY                 (1 << 15) /* Flash Ready Flag */
#define FLASH_FSTATR_ILGLERR              (1 << 14) /* Illegal Command Error Flag */
#define FLASH_FSTATR_ERSERR               (1 << 13) /* Erase Error Flag */
#define FLASH_FSTATR_PRGERR               (1 << 12) /* Program Error Flag */
#define FLASH_FSTATR_SUSRDY               (1 << 11) /* Suspend Ready Flag */
#define FLASH_FSTATR_DBFULL               (1 << 10) /* Data Buffer Full Flag */
#define FLASH_FSTATR_ERSSUS               (1 <<  9) /* Erase Suspend Flag */
#define FLASH_FSTATR_PRGCOF               (1 <<  8) /* Programming Complete Flag */
#define FLASH_FSTATR_ERSCOF               (1 <<  7) /* Erase Complete Flag */

/* Program/Erase Mode Entry Register (FENTRYR) Bitfield Definitions */

#define FLASH_FENTRYR_FENTRYD             (1 <<  7) /* Data Flash P/E Mode Entry */
#define FLASH_FENTRYR_FENTRYC             (1 <<  0) /* Code Flash P/E Mode Entry */

/* Flash Command Definitions */

#define FLASH_CMD_BLOCK_ERASE             0x20D0    /* Block Erase Command */
#define FLASH_CMD_PROGRAM                 0xE8D0    /* Program Command */
#define FLASH_CMD_STATUS_CLEAR            0x50      /* Status Clear Command */

#endif /* __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8E1_FLASH_H */
