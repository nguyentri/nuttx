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

/* DTC Register Offsets */
#define RA_DTC_DTCCR_OFFSET      0x00  /* DTC Control Register */
#define RA_DTC_DTCVBR_OFFSET     0x04  /* DTC Vector Base Register */
#define RA_DTC_DTCADMOD_OFFSET   0x08  /* DTC Address Mode Register */
#define RA_DTC_DTCST_OFFSET      0x0C  /* DTC Start Register */
#define RA_DTC_DTCSTS_OFFSET     0x10  /* DTC Status Register */
#define RA_DTC_DTCIBR_OFFSET     0x14  /* DTC Index Table Base Register */
#define RA_DTC_DTCOR_OFFSET      0x18  /* DTC Operation Register */

/* DTC Register Addresses */
#define RA_DTC_DTCCR             (R_DTC_BASE + RA_DTC_DTCCR_OFFSET)
#define RA_DTC_DTCVBR            (R_DTC_BASE + RA_DTC_DTCVBR_OFFSET)
#define RA_DTC_DTCADMOD          (R_DTC_BASE + RA_DTC_DTCADMOD_OFFSET)
#define RA_DTC_DTCST             (R_DTC_BASE + RA_DTC_DTCST_OFFSET)
#define RA_DTC_DTCSTS            (R_DTC_BASE + RA_DTC_DTCSTS_OFFSET)
#define RA_DTC_DTCIBR            (R_DTC_BASE + RA_DTC_DTCIBR_OFFSET)
#define RA_DTC_DTCOR             (R_DTC_BASE + RA_DTC_DTCOR_OFFSET)

/* DTC Control Register (DTCCR) bit definitions */
#define RA_DTC_DTCCR_RRS         (1 << 4)  /* DTC Transfer Information Read Skip Enable */

/* DTC Status Register (DTCSTS) bit definitions */
#define RA_DTC_DTCSTS_VECN_SHIFT (0)       /* Vector Number */
#define RA_DTC_DTCSTS_VECN_MASK  (0xFF << RA_DTC_DTCSTS_VECN_SHIFT)
#define RA_DTC_DTCSTS_ACT        (1 << 15) /* DTC Active Flag */

/* Transfer Information Register Layout (16 bytes) */
#define RA_DTC_INFO_MRA_OFFSET   0x00  /* Mode Register A */
#define RA_DTC_INFO_MRB_OFFSET   0x01  /* Mode Register B */
#define RA_DTC_INFO_MRC_OFFSET   0x02  /* Mode Register C */
#define RA_DTC_INFO_MRD_OFFSET   0x03  /* Mode Register D */
#define RA_DTC_INFO_SAR_OFFSET   0x04  /* Source Address Register */
#define RA_DTC_INFO_DAR_OFFSET   0x08  /* Destination Address Register */
#define RA_DTC_INFO_CRA_OFFSET   0x0C  /* Transfer Count Register A */
#define RA_DTC_INFO_CRB_OFFSET   0x0E  /* Transfer Count Register B */

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
#define RA_DTC_MRA_SM_INCR       (0x1 << RA_DTC_MRA_SM_SHIFT)
#define RA_DTC_MRA_SM_DECR       (0x2 << RA_DTC_MRA_SM_SHIFT)

#define RA_DTC_MRA_DM_SHIFT      (0)       /* Destination Address Mode */
#define RA_DTC_MRA_DM_MASK       (0x3 << RA_DTC_MRA_DM_SHIFT)
#define RA_DTC_MRA_DM_FIXED      (0x0 << RA_DTC_MRA_DM_SHIFT)
#define RA_DTC_MRA_DM_INCR       (0x1 << RA_DTC_MRA_DM_SHIFT)
#define RA_DTC_MRA_DM_DECR       (0x2 << RA_DTC_MRA_DM_SHIFT)

/* Mode Register B (MRB) bit definitions */
#define RA_DTC_MRB_CHNE          (1 << 7)  /* Chain Transfer Enable */
#define RA_DTC_MRB_CHNS          (1 << 6)  /* Chain Transfer Select */
#define RA_DTC_MRB_DISEL         (1 << 5)  /* Destination Increment Select */
#define RA_DTC_MRB_DTS           (1 << 4)  /* Destination Transfer Select */

/* Mode Register C (MRC) bit definitions */
#define RA_DTC_MRC_WDSEL         (1 << 0)  /* Write-back Disable Select */

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
  uint8_t  mra;            /* Mode Register A */
  uint8_t  mrb;            /* Mode Register B */
  uint8_t  mrc;            /* Mode Register C */
  uint8_t  mrd;            /* Mode Register D */
  uint32_t sar;            /* Source Address Register */
  uint32_t dar;            /* Destination Address Register */
  uint16_t cra;            /* Transfer Count Register A */
  uint16_t crb;            /* Transfer Count Register B */
} ra_dtc_info_t;

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_DTC_H */