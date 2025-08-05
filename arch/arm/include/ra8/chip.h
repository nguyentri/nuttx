/****************************************************************************
 * arch/arm/include/ra8/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RA_CHIP_H
#define __ARCH_ARM_INCLUDE_RA_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* RA8E1 Family */

/* Memory
 * - 1 MB code flash memory
 * - 12 KB data flash memory (100,000 program/erase (P/E) cycles)
 * - 544 KB SRAM including 32 KB of TCM
 */

/* Internal memory sizes for RA8E1 */
#define RA8E1_CODE_FLASH_SIZE        (1024*1024)   /* 1MB Code Flash */
#define RA8E1_DATA_FLASH_SIZE        (12*1024)     /* 12KB Data Flash */
#define RA8E1_SRAM_SIZE              (512*1024)    /* 512KB SRAM */
#define RA8E1_TCM_SIZE               (32*1024)     /* 32KB SRAM TCM */

#define RA_FLASH_SIZE                 RA8E1_CODE_FLASH_SIZE
#define RA_SRAM0_SIZE                 RA8E1_SRAM_SIZE + RA8E1_TCM_SIZE

/* Supported packages:
 * R7FA8E1AFDCFB PLQP0144KA-B (LQFP144)
 * R7FA8E1AFDCFP PLQP0100KP-A (LQFP100)
 */

/* NVIC priority levels *****************************************************/

/* Each priority field holds a priority value, 0-15. The lower the value, the
 * greater the priority of the corresponding interrupt. The processor
 * implements only bits[7:4] of each field, bits[3:0] read as zero and ignore
 * writes.
 */

#define NVIC_SYSH_PRIORITY_MIN        0xf0 /* All bits[7:4] set is minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT    0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX        0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP       0x10 /* Four bits of interrupt priority used */

/* Vector Table Entries *****************************************************/
/* RA8E1 has 96 external interrupt vectors + 16 Cortex-M85 core vectors = 112 total */

#define RA8E1_IRQ_NEXTERNAL          96    /* 96 external interrupt vectors */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_RA_CHIP_H */
