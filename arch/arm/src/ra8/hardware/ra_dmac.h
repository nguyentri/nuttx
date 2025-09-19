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

/* DMAC Register Offsets (per channel) */
#define RA_DMAC_DMSAR_OFFSET     0x00  /* Source Address Register */
#define RA_DMAC_DMDAR_OFFSET     0x04  /* Destination Address Register */
#define RA_DMAC_DMCRA_OFFSET     0x08  /* Transfer Count Register A */
#define RA_DMAC_DMCRB_OFFSET     0x0C  /* Transfer Count Register B */
#define RA_DMAC_DMTMD_OFFSET     0x10  /* Transfer Mode Register */
#define RA_DMAC_DMINT_OFFSET     0x13  /* Interrupt Setting Register */
#define RA_DMAC_DMAMD_OFFSET     0x14  /* Address Mode Register */
#define RA_DMAC_DMOFR_OFFSET     0x18  /* Offset Register */
#define RA_DMAC_DMCNT_OFFSET     0x1C  /* Count Register */
#define RA_DMAC_DMREQ_OFFSET     0x20  /* Software Start Register */
#define RA_DMAC_DMSTS_OFFSET     0x24  /* Status Register */
#define RA_DMAC_DMCSL_OFFSET     0x28  /* Channel Status Low Register */
#define RA_DMAC_DMCSH_OFFSET     0x2C  /* Channel Status High Register */

/* DMAC Control Register Offsets */
#define RA_DMAC_DMAST_OFFSET     0x200  /* DMA Start Register */
#define RA_DMAC_DMIST_OFFSET     0x204  /* DMA Interrupt Status Register */

/* Channel stride (each channel is 0x40 bytes apart) */
#define RA_DMAC_CHANNEL_STRIDE   0x40

/* DMA Transfer Mode Register (DMTMD) bits */
#define RA_DMAC_DMTMD_DCTG_SHIFT (0)    /* Data Transfer Trigger */
#define RA_DMAC_DMTMD_DCTG_MASK  (3 << RA_DMAC_DMTMD_DCTG_SHIFT)
#define RA_DMAC_DMTMD_DCTG_SW    (0 << RA_DMAC_DMTMD_DCTG_SHIFT)  /* Software trigger */
#define RA_DMAC_DMTMD_DCTG_HW    (1 << RA_DMAC_DMTMD_DCTG_SHIFT)  /* Hardware trigger */

#define RA_DMAC_DMTMD_SZ_SHIFT   (8)    /* Transfer Data Size */
#define RA_DMAC_DMTMD_SZ_MASK    (3 << RA_DMAC_DMTMD_SZ_SHIFT)
#define RA_DMAC_DMTMD_SZ_8       (0 << RA_DMAC_DMTMD_SZ_SHIFT)    /* 8-bit */
#define RA_DMAC_DMTMD_SZ_16      (1 << RA_DMAC_DMTMD_SZ_SHIFT)    /* 16-bit */
#define RA_DMAC_DMTMD_SZ_32      (2 << RA_DMAC_DMTMD_SZ_SHIFT)    /* 32-bit */

#define RA_DMAC_DMTMD_DTS_SHIFT  (12)   /* Destination Transfer Select */
#define RA_DMAC_DMTMD_DTS_MASK   (3 << RA_DMAC_DMTMD_DTS_SHIFT)
#define RA_DMAC_DMTMD_DTS_NORMAL (0 << RA_DMAC_DMTMD_DTS_SHIFT)   /* Normal mode */
#define RA_DMAC_DMTMD_DTS_REPEAT (1 << RA_DMAC_DMTMD_DTS_SHIFT)   /* Repeat mode */
#define RA_DMAC_DMTMD_DTS_BLOCK  (2 << RA_DMAC_DMTMD_DTS_SHIFT)   /* Block mode */

#define RA_DMAC_DMTMD_MD_SHIFT   (14)   /* Transfer Mode */
#define RA_DMAC_DMTMD_MD_MASK    (3 << RA_DMAC_DMTMD_MD_SHIFT)
#define RA_DMAC_DMTMD_MD_NORMAL  (0 << RA_DMAC_DMTMD_MD_SHIFT)    /* Normal mode */
#define RA_DMAC_DMTMD_MD_REPEAT  (1 << RA_DMAC_DMTMD_MD_SHIFT)    /* Repeat mode */
#define RA_DMAC_DMTMD_MD_BLOCK   (2 << RA_DMAC_DMTMD_MD_SHIFT)    /* Block mode */

/* DMA Interrupt Setting Register (DMINT) bits */
#define RA_DMAC_DMINT_DARIE      (1 << 0)  /* Destination Address Reload Interrupt Enable */
#define RA_DMAC_DMINT_SARIE      (1 << 1)  /* Source Address Reload Interrupt Enable */
#define RA_DMAC_DMINT_RPTIE      (1 << 2)  /* Repeat Transfer Interrupt Enable */
#define RA_DMAC_DMINT_ESIE       (1 << 3)  /* Extension Section Interrupt Enable */
#define RA_DMAC_DMINT_DTIE       (1 << 4)  /* Data Transfer Interrupt Enable */

/* DMA Address Mode Register (DMAMD) bits */
#define RA_DMAC_DMAMD_DARA_SHIFT (0)       /* Destination Address Reload Area */
#define RA_DMAC_DMAMD_DARA_MASK  (0x1F << RA_DMAC_DMAMD_DARA_SHIFT)

#define RA_DMAC_DMAMD_DM_SHIFT   (6)       /* Destination Address Mode */
#define RA_DMAC_DMAMD_DM_MASK    (3 << RA_DMAC_DMAMD_DM_SHIFT)
#define RA_DMAC_DMAMD_DM_FIXED   (0 << RA_DMAC_DMAMD_DM_SHIFT)    /* Fixed */
#define RA_DMAC_DMAMD_DM_OFFSET  (1 << RA_DMAC_DMAMD_DM_SHIFT)    /* Offset */
#define RA_DMAC_DMAMD_DM_INCR    (2 << RA_DMAC_DMAMD_DM_SHIFT)    /* Increment */
#define RA_DMAC_DMAMD_DM_DECR    (3 << RA_DMAC_DMAMD_DM_SHIFT)    /* Decrement */

#define RA_DMAC_DMAMD_SARA_SHIFT (8)       /* Source Address Reload Area */
#define RA_DMAC_DMAMD_SARA_MASK  (0x1F << RA_DMAC_DMAMD_SARA_SHIFT)

#define RA_DMAC_DMAMD_SM_SHIFT   (14)      /* Source Address Mode */
#define RA_DMAC_DMAMD_SM_MASK    (3 << RA_DMAC_DMAMD_SM_SHIFT)
#define RA_DMAC_DMAMD_SM_FIXED   (0 << RA_DMAC_DMAMD_SM_SHIFT)    /* Fixed */
#define RA_DMAC_DMAMD_SM_OFFSET  (1 << RA_DMAC_DMAMD_SM_SHIFT)    /* Offset */
#define RA_DMAC_DMAMD_SM_INCR    (2 << RA_DMAC_DMAMD_SM_SHIFT)    /* Increment */
#define RA_DMAC_DMAMD_SM_DECR    (3 << RA_DMAC_DMAMD_SM_SHIFT)    /* Decrement */

/* DMA Software Start Register (DMREQ) bits */
#define RA_DMAC_DMREQ_SWREQ      (1 << 0)  /* Software Request */
#define RA_DMAC_DMREQ_CLRS       (1 << 4)  /* Clear Source */

/* DMA Status Register (DMSTS) bits */
#define RA_DMAC_DMSTS_DTIF       (1 << 0)  /* Data Transfer Interrupt Flag */
#define RA_DMAC_DMSTS_ESIF       (1 << 4)  /* Extension Section Interrupt Flag */
#define RA_DMAC_DMSTS_ACT        (1 << 7)  /* DMA Transfer Active */

/* Maximum transfer lengths */
#define RA_DMAC_MAX_NORMAL_LENGTH    (0x10000)  /* 65536 transfers */
#define RA_DMAC_MAX_REPEAT_LENGTH    (0x400)    /* 1024 transfers */
#define RA_DMAC_MAX_BLOCK_LENGTH     (0x400)    /* 1024 transfers */

/* Number of DMAC channels */
#define RA_DMAC_NUM_CHANNELS     (8)

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
/* Register access macros */
#define RA_DMAC_DMSAR(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMSAR_OFFSET)
#define RA_DMAC_DMDAR(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMDAR_OFFSET)
#define RA_DMAC_DMCRA(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMCRA_OFFSET)
#define RA_DMAC_DMCRB(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMCRB_OFFSET)
#define RA_DMAC_DMTMD(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMTMD_OFFSET)
#define RA_DMAC_DMINT(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMINT_OFFSET)
#define RA_DMAC_DMAMD(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMAMD_OFFSET)
#define RA_DMAC_DMOFR(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMOFR_OFFSET)
#define RA_DMAC_DMCNT(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMCNT_OFFSET)
#define RA_DMAC_DMREQ(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMREQ_OFFSET)
#define RA_DMAC_DMSTS(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMSTS_OFFSET)
#define RA_DMAC_DMCSL(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMCSL_OFFSET)
#define RA_DMAC_DMCSH(n)         (R_DMAC0_BASE + (n)*RA_DMAC_CHANNEL_STRIDE + RA_DMAC_DMCSH_OFFSET)

/* Control registers */
#define RA_DMAC_DMAST            (R_DMAC0_BASE + RA_DMAC_DMAST_OFFSET)
#define RA_DMAC_DMIST            (R_DMAC0_BASE + RA_DMAC_DMIST_OFFSET)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DMAC Transfer Mode */
typedef enum
{
  RA_DMAC_MODE_NORMAL = 0,  /* Normal mode */
  RA_DMAC_MODE_REPEAT,      /* Repeat mode */
  RA_DMAC_MODE_BLOCK        /* Block mode */
} ra_dmac_mode_t;

/* DMAC Transfer Size */
typedef enum
{
  RA_DMAC_SIZE_8BIT = 0,    /* 8-bit transfer */
  RA_DMAC_SIZE_16BIT,       /* 16-bit transfer */
  RA_DMAC_SIZE_32BIT        /* 32-bit transfer */
} ra_dmac_size_t;

/* DMAC Address Mode */
typedef enum
{
  RA_DMAC_ADDR_FIXED = 0,   /* Fixed address */
  RA_DMAC_ADDR_OFFSET,      /* Offset address */
  RA_DMAC_ADDR_INCR,        /* Increment address */
  RA_DMAC_ADDR_DECR         /* Decrement address */
} ra_dmac_addr_mode_t;

/* DMAC Trigger Mode */
typedef enum
{
  RA_DMAC_TRIGGER_SW = 0,   /* Software trigger */
  RA_DMAC_TRIGGER_HW        /* Hardware trigger */
} ra_dmac_trigger_t;

#define RA_DMAC_DMAST            (R_DMAC0_BASE + RA_DMAC_DMAST_OFFSET)
#define RA_DMAC_DMIST            (R_DMAC0_BASE + RA_DMAC_DMIST_OFFSET)

/* Maximum number of DMAC channels */
#define RA_DMAC_NUM_CHANNELS     8

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_DMAC_H */
