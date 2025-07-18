/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_dmac.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA_DMAC_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAC Register Offsets */
#define RA_DMAC_DMSAR_OFFSET     0x00  /* DMA Source Address Register */
#define RA_DMAC_DMDAR_OFFSET     0x04  /* DMA Destination Address Register */
#define RA_DMAC_DMCRA_OFFSET     0x08  /* DMA Transfer Count Register A */
#define RA_DMAC_DMCRB_OFFSET     0x0C  /* DMA Transfer Count Register B */
#define RA_DMAC_DMTMD_OFFSET     0x10  /* DMA Transfer Mode Register */
#define RA_DMAC_DMINT_OFFSET     0x13  /* DMA Interrupt Setting Register */
#define RA_DMAC_DMAMD_OFFSET     0x14  /* DMA Address Mode Register */
#define RA_DMAC_DMOFR_OFFSET     0x18  /* DMA Offset Register */
#define RA_DMAC_DMCNT_OFFSET     0x1C  /* DMA Software Start Register */

/* DMAC Control Register Offsets */
#define RA_DMAC_DMAST_OFFSET     0x200  /* DMA Module Start Register */
#define RA_DMAC_DMIST_OFFSET     0x204  /* DMA Module Information Register */

/* DMA Transfer Mode Register (DMTMD) bits */
#define RA_DMAC_DMTMD_DCTG_SHIFT (0)    /* DMA Transfer Request Source Select */
#define RA_DMAC_DMTMD_DCTG_MASK  (3 << RA_DMAC_DMTMD_DCTG_SHIFT)
#define RA_DMAC_DMTMD_DCTG_SW    (0 << RA_DMAC_DMTMD_DCTG_SHIFT)
#define RA_DMAC_DMTMD_DCTG_HW    (1 << RA_DMAC_DMTMD_DCTG_SHIFT)

#define RA_DMAC_DMTMD_SZ_SHIFT   (8)    /* DMA Transfer Data Size Select */
#define RA_DMAC_DMTMD_SZ_MASK    (3 << RA_DMAC_DMTMD_SZ_SHIFT)
#define RA_DMAC_DMTMD_SZ_8       (0 << RA_DMAC_DMTMD_SZ_SHIFT)
#define RA_DMAC_DMTMD_SZ_16      (1 << RA_DMAC_DMTMD_SZ_SHIFT)
#define RA_DMAC_DMTMD_SZ_32      (2 << RA_DMAC_DMTMD_SZ_SHIFT)

#define RA_DMAC_DMTMD_DTS_SHIFT  (12)   /* DMA Transfer Request Source Select */
#define RA_DMAC_DMTMD_DTS_MASK   (3 << RA_DMAC_DMTMD_DTS_SHIFT)

#define RA_DMAC_DMTMD_MD_SHIFT   (14)   /* DMA Transfer Mode Select */
#define RA_DMAC_DMTMD_MD_MASK    (3 << RA_DMAC_DMTMD_MD_SHIFT)
#define RA_DMAC_DMTMD_MD_NORMAL  (0 << RA_DMAC_DMTMD_MD_SHIFT)
#define RA_DMAC_DMTMD_MD_REPEAT  (1 << RA_DMAC_DMTMD_MD_SHIFT)
#define RA_DMAC_DMTMD_MD_BLOCK   (2 << RA_DMAC_DMTMD_MD_SHIFT)

/* DMA Interrupt Setting Register (DMINT) bits */
#define RA_DMAC_DMINT_DARIE      (1 << 0)  /* Destination Address Extended Repeat Area Overflow Interrupt Enable */
#define RA_DMAC_DMINT_SARIE      (1 << 1)  /* Source Address Extended Repeat Area Overflow Interrupt Enable */
#define RA_DMAC_DMINT_RPTIE      (1 << 2)  /* Repeat Size End Interrupt Enable */
#define RA_DMAC_DMINT_ESIE       (1 << 3)  /* Transfer Escape End Interrupt Enable */
#define RA_DMAC_DMINT_DTIE       (1 << 4)  /* Transfer End Interrupt Enable */

/* DMA Address Mode Register (DMAMD) bits */
#define RA_DMAC_DMAMD_DARA_SHIFT (0)    /* Destination Address Update Mode */
#define RA_DMAC_DMAMD_DARA_MASK  (0x1F << RA_DMAC_DMAMD_DARA_SHIFT)
#define RA_DMAC_DMAMD_DARA_FIXED (0x00 << RA_DMAC_DMAMD_DARA_SHIFT)
#define RA_DMAC_DMAMD_DARA_INC   (0x08 << RA_DMAC_DMAMD_DARA_SHIFT)
#define RA_DMAC_DMAMD_DARA_DEC   (0x0C << RA_DMAC_DMAMD_DARA_SHIFT)

#define RA_DMAC_DMAMD_SARA_SHIFT (8)    /* Source Address Update Mode */
#define RA_DMAC_DMAMD_SARA_MASK  (0x1F << RA_DMAC_DMAMD_SARA_SHIFT)
#define RA_DMAC_DMAMD_SARA_FIXED (0x00 << RA_DMAC_DMAMD_SARA_SHIFT)
#define RA_DMAC_DMAMD_SARA_INC   (0x08 << RA_DMAC_DMAMD_SARA_SHIFT)
#define RA_DMAC_DMAMD_SARA_DEC   (0x0C << RA_DMAC_DMAMD_SARA_SHIFT)

#define RA_DMAC_DMAMD_SM_SHIFT   (14)   /* Source Address Extended Repeat Area */
#define RA_DMAC_DMAMD_SM_MASK    (3 << RA_DMAC_DMAMD_SM_SHIFT)

#define RA_DMAC_DMAMD_DM_SHIFT   (6)    /* Destination Address Extended Repeat Area */
#define RA_DMAC_DMAMD_DM_MASK    (3 << RA_DMAC_DMAMD_DM_SHIFT)

/* DMA Software Start Register (DMCNT) bits */
#define RA_DMAC_DMCNT_DTE        (1 << 0)  /* DMA Transfer Enable */

/* DMA Module Start Register (DMAST) bits */
#define RA_DMAC_DMAST_DMST       (1 << 0)  /* DMA Module Start */

/* Register addresses */
#define RA_DMAC_DMSAR(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMSAR_OFFSET)
#define RA_DMAC_DMDAR(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMDAR_OFFSET)
#define RA_DMAC_DMCRA(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMCRA_OFFSET)
#define RA_DMAC_DMCRB(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMCRB_OFFSET)
#define RA_DMAC_DMTMD(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMTMD_OFFSET)
#define RA_DMAC_DMINT(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMINT_OFFSET)
#define RA_DMAC_DMAMD(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMAMD_OFFSET)
#define RA_DMAC_DMOFR(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMOFR_OFFSET)
#define RA_DMAC_DMCNT(n)         (R_DMAC0_BASE + (n)*0x40 + RA_DMAC_DMCNT_OFFSET)

#define RA_DMAC_DMAST            (R_DMAC0_BASE + RA_DMAC_DMAST_OFFSET)
#define RA_DMAC_DMIST            (R_DMAC0_BASE + RA_DMAC_DMIST_OFFSET)

/* Maximum number of DMAC channels */
#define RA_DMAC_NUM_CHANNELS     8

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_DMAC_H */
