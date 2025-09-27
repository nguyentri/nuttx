/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_dtc.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA_DTC_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA_DTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "chip.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DTC Register Offsets - Based on R7FA8E1AF SVD file */
#define RA_DTC_DTCST_OFFSET      0x0C  /* DTC Module Start Register */
#define RA_DTC_DTCSTS_OFFSET     0x0E  /* DTC Status Register */
#define RA_DTC_DTEVR_OFFSET      0x20  /* DTC Error Vector Register */

#define RA_DTC_DTCCR_OFFSET      0x00  /* DTC Control Register for Non-secure region */
#define RA_DTC_DTCVBR_OFFSET     0x40  /* DTC Vector Base Register for Non-secure region */

#define RA_DTC_DTCCR_SEC_OFFSET  0x10  /* DTC Control Register for Secure Region */
#define RA_DTC_DTCVBR_SEC_OFFSET 0x14  /* DTC Vector Base Register for Secure Region */

/* DTC Register Addresses */
#define RA_DTC_DTCST             (R_DTC_BASE + RA_DTC_DTCST_OFFSET)
#define RA_DTC_DTCSTS            (R_DTC_BASE + RA_DTC_DTCSTS_OFFSET)
#define RA_DTC_DTEVR             (R_DTC_BASE + RA_DTC_DTEVR_OFFSET)

#define RA_DTC_DTCCR_NC          (R_DTC_BASE + RA_DTC_DTCCR_OFFSET)
#define RA_DTC_DTCVBR_NC         (R_DTC_BASE + RA_DTC_DTCVBR_OFFSET)

#define RA_DTC_DTCCR_SEC         (R_DTC_BASE + RA_DTC_DTCCR_SEC_OFFSET)
#define RA_DTC_DTCVBR_SEC        (R_DTC_BASE + RA_DTC_DTCVBR_SEC_OFFSET)

/* DTC Module Start Register (DTCST) bit definitions */
#define RA_DTC_DTCST_DTCST       (1 << 0)  /* DTC Module Start */

/* DTC Status Register (DTCSTS) bit definitions */
#define RA_DTC_DTCSTS_VECN_SHIFT (0)       /* Vector Number */
#define RA_DTC_DTCSTS_VECN_MASK  (0xFF << RA_DTC_DTCSTS_VECN_SHIFT)
#define RA_DTC_DTCSTS_ACT        (1 << 15) /* DTC Active Flag */

/* DTC Error Vector Register (DTEVR) bit definitions */
#define RA_DTC_DTEVR_DTEV_SHIFT  (0)       /* DTC Error Vector Number */
#define RA_DTC_DTEVR_DTEV_MASK   (0xFF << RA_DTC_DTEVR_DTEV_SHIFT)
#define RA_DTC_DTEVR_DTEVSAM     (1 << 8)  /* DTC Error Vector Number SA Monitor */
#define RA_DTC_DTEVR_DTESTA      (1 << 16) /* DTC Error Status Flag */

/* Transfer Information Register Layout (16 bytes) - Based on RA8E1 Hardware Manual */
#define RA_DTC_INFO_MRB_OFFSET   0x02  /* Mode Register B */
#define RA_DTC_INFO_MRA_OFFSET   0x03  /* Mode Register A */
#define RA_DTC_INFO_SAR_OFFSET   0x04  /* Source Address Register */
#define RA_DTC_INFO_DAR_OFFSET   0x08  /* Destination Address Register */
#define RA_DTC_INFO_CRB_OFFSET   0x0C  /* Transfer Count Register B */
#define RA_DTC_INFO_CRA_OFFSET   0x0E  /* Transfer Count Register A */

/* Mode Register A (MRA) bit definitions */
#define RA_DTC_MRA_MD_SHIFT      (6)       /* Transfer Mode */
#define RA_DTC_MRA_MD_MASK       (0x3 << RA_DTC_MRA_MD_SHIFT)
#define RA_DTC_MRA_MD_NORMAL     (0x0 << RA_DTC_MRA_MD_SHIFT)
#define RA_DTC_MRA_MD_REPEAT     (0x1 << RA_DTC_MRA_MD_SHIFT)
#define RA_DTC_MRA_MD_BLOCK      (0x2 << RA_DTC_MRA_MD_SHIFT)

#define RA_DTC_MRA_SZ_SHIFT      (4)       /* Transfer Data Size */
#define RA_DTC_MRA_SZ_MASK       (0x3 << RA_DTC_MRA_SZ_SHIFT)
#define RA_DTC_MRA_SZ_BYTE       (0x0 << RA_DTC_MRA_SZ_SHIFT)
#define RA_DTC_MRA_SZ_WORD       (0x1 << RA_DTC_MRA_SZ_SHIFT)
#define RA_DTC_MRA_SZ_LONG       (0x2 << RA_DTC_MRA_SZ_SHIFT)

#define RA_DTC_MRA_SM_SHIFT      (2)       /* Source Address Mode */
#define RA_DTC_MRA_SM_MASK       (0x3 << RA_DTC_MRA_SM_SHIFT)
#define RA_DTC_MRA_SM_FIXED      (0x0 << RA_DTC_MRA_SM_SHIFT)
#define RA_DTC_MRA_SM_INCREMENT  (0x2 << RA_DTC_MRA_SM_SHIFT)
#define RA_DTC_MRA_SM_DECREMENT  (0x3 << RA_DTC_MRA_SM_SHIFT)

/* Mode Register B (MRB) bit definitions */
#define RA_DTC_MRB_CHNE          (1 << 7)  /* Chain Transfer Enable */
#define RA_DTC_MRB_CHNS          (1 << 6)  /* Chain Transfer Select */
#define RA_DTC_MRB_DISEL         (1 << 5)  /* DTC Interrupt Select */
#define RA_DTC_MRB_DTS           (1 << 4)  /* DTC Transfer Mode Select */

#define RA_DTC_MRB_DM_SHIFT      (2)       /* Destination Address Mode */
#define RA_DTC_MRB_DM_MASK       (0x3 << RA_DTC_MRB_DM_SHIFT)
#define RA_DTC_MRB_DM_FIXED      (0x0 << RA_DTC_MRB_DM_SHIFT)
#define RA_DTC_MRB_DM_INCREMENT  (0x2 << RA_DTC_MRB_DM_SHIFT)
#define RA_DTC_MRB_DM_DECREMENT  (0x3 << RA_DTC_MRB_DM_SHIFT)

/* Transfer size constants */
#define RA_DTC_MAX_NORMAL_LENGTH    (0x10000)  /* 65536 transfers max */
#define RA_DTC_MAX_REPEAT_LENGTH    (0x400)    /* 1024 transfers max */
#define RA_DTC_MAX_BLOCK_LENGTH     (0x400)    /* 1024 transfers max */

/* Vector table alignment */
#define RA_DTC_VECTOR_TABLE_ALIGN   (1024)

/* Transfer info structure size (must be 16 bytes) */
#define RA_DTC_TRANSFER_INFO_SIZE   (16)

/* Vector table size for DTC */
#define RA_DTC_VECTOR_TABLE_ENTRIES  (32)

 #define DTC_PRV_DTCSAR_DTCSTSA       ((R_CPSCU->DTCSAR & R_CPSCU_DTCSAR_DTCSTSA_Msk) >> R_CPSCU_DTCSAR_DTCSTSA_Pos)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DTC Transfer Mode */
typedef enum
{
  RA_DTC_MODE_NORMAL = 0,  /* Normal mode */
  RA_DTC_MODE_REPEAT,      /* Repeat mode */
  RA_DTC_MODE_BLOCK        /* Block mode */
} ra_dtc_mode_t;

/* DTC Transfer Size */
typedef enum
{
  RA_DTC_SIZE_BYTE = 0,    /* 8-bit transfer */
  RA_DTC_SIZE_WORD,        /* 16-bit transfer */
  RA_DTC_SIZE_LONG         /* 32-bit transfer */
} ra_dtc_size_t;

/* DTC Address Mode */
typedef enum
{
  RA_DTC_ADDR_FIXED = 0,   /* Fixed address */
  RA_DTC_ADDR_INCR,        /* Increment address */
  RA_DTC_ADDR_DECR         /* Decrement address */
} ra_dtc_addr_mode_t;

/* DTC Transfer Information Structure (16 bytes) */
typedef struct
{
  uint8_t  reserved0[2];  /* Offset 0x00-0x01: Reserved */
  uint8_t  mrb;           /* Offset 0x02: Mode Register B */
  uint8_t  mra;           /* Offset 0x03: Mode Register A */
  uint32_t sar;           /* Offset 0x04: Source Address Register */
  uint32_t dar;           /* Offset 0x08: Destination Address Register */
  uint16_t crb;           /* Offset 0x0C: Transfer Count Register B */
  uint16_t cra;           /* Offset 0x0E: Transfer Count Register A */
} ra_dtc_info_t;

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_DTC_H */