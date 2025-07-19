/****************************************************************************
 * arch/arm/src/ra8/ra_fsp_integration.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_FSP_INTEGRATION_H
#define __ARCH_ARM_SRC_RA8_RA_FSP_INTEGRATION_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FSP-Based Clock Configuration Defaults */

#ifndef CONFIG_RA8_HOCO_FREQUENCY
#  define CONFIG_RA8_HOCO_FREQUENCY    48000000  /* 48MHz HOCO */
#endif

#ifndef CONFIG_RA8_MOCO_FREQUENCY
#  define CONFIG_RA8_MOCO_FREQUENCY    8000000   /* 8MHz MOCO */
#endif

#ifndef CONFIG_RA8_LOCO_FREQUENCY
#  define CONFIG_RA8_LOCO_FREQUENCY    32768     /* 32.768kHz LOCO */
#endif

/* FSP-Based System Clock Selection */

#ifndef RA_CKSEL
#  ifdef CONFIG_RA8_CLOCK_PLL
#    define RA_CKSEL                   5         /* PLL */
#  elif defined(CONFIG_RA8_CLOCK_HOCO)
#    define RA_CKSEL                   0         /* HOCO */
#  elif defined(CONFIG_RA8_CLOCK_MOCO)
#    define RA_CKSEL                   1         /* MOCO */
#  else
#    define RA_CKSEL                   0         /* Default to HOCO */
#  endif
#endif

/* FSP-Based PLL Configuration */

#ifndef CONFIG_RA8_PLL_SOURCE
#  define CONFIG_RA8_PLL_SOURCE        0         /* HOCO as PLL source */
#endif

#ifndef CONFIG_RA8_PLL_DIV
#  define CONFIG_RA8_PLL_DIV           0         /* PLL input divider */
#endif

#ifndef CONFIG_RA8_PLL_MUL
#  define CONFIG_RA8_PLL_MUL           20        /* PLL multiplier for 240MHz */
#endif

#define RA_PLL_DIV                     CONFIG_RA8_PLL_DIV
#define RA_PLL_MUL                     CONFIG_RA8_PLL_MUL

/* FSP-Based Clock Frequencies */

#if defined(CONFIG_RA8_CLOCK_PLL)
#  define RA_SYSTEM_CLOCK_FREQUENCY    (CONFIG_RA8_HOCO_FREQUENCY * RA_PLL_MUL / (RA_PLL_DIV + 1))
#elif defined(CONFIG_RA8_CLOCK_HOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA8_HOCO_FREQUENCY
#elif defined(CONFIG_RA8_CLOCK_MOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA8_MOCO_FREQUENCY
#else
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA8_HOCO_FREQUENCY
#endif

/* FSP-Based Clock Dividers */

#ifndef CONFIG_RA8_ICK_DIV
#  define CONFIG_RA8_ICK_DIV           0         /* ICLK = System clock / 1 */
#endif

#ifndef CONFIG_RA8_PCKB_DIV
#  define CONFIG_RA8_PCKB_DIV          1         /* PCKB = System clock / 2 */
#endif

#ifndef CONFIG_RA8_PCKD_DIV
#  define CONFIG_RA8_PCKD_DIV          2         /* PCKD = System clock / 4 */
#endif

#define RA_ICK_DIV                     (CONFIG_RA8_ICK_DIV << 24)
#define RA_FCK_DIV                     (CONFIG_RA8_ICK_DIV << 28)
#define RA_PCKA_DIV                    (CONFIG_RA8_ICK_DIV << 12)
#define RA_PCKB_DIV                    (CONFIG_RA8_PCKB_DIV << 8)
#define RA_PCKC_DIV                    (CONFIG_RA8_PCKB_DIV << 4)
#define RA_PCKD_DIV                    (CONFIG_RA8_PCKD_DIV << 0)

/* FSP-Based Derived Frequencies */

#define RA_ICLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY >> CONFIG_RA8_ICK_DIV)
#define RA_PCLKB_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY >> CONFIG_RA8_PCKB_DIV)
#define RA_PCLKD_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY >> CONFIG_RA8_PCKD_DIV)

/* FSP-Based Timer Configuration */

#ifdef CONFIG_RA8_SYSTICK_GPT
#  define RA_TIMER_SOURCE_FREQ         RA_PCLKD_FREQUENCY
#  ifndef RA_GPT_CHANNEL
#    define RA_GPT_CHANNEL             0
#  endif
#endif

/* FSP-Based Memory Configuration - Matched to Linker Script */

#ifndef CONFIG_RA8_DTCM_BASE
#  define CONFIG_RA8_DTCM_BASE         0x20000000  /* DTCM base from linker */
#endif

#ifndef CONFIG_RA8_DTCM_SIZE
#  define CONFIG_RA8_DTCM_SIZE         0x4000      /* 16KB - matches linker */
#endif

#ifndef CONFIG_RA8_ITCM_BASE
#  define CONFIG_RA8_ITCM_BASE         0x00000000  /* ITCM base from linker */
#endif

#ifndef CONFIG_RA8_ITCM_SIZE
#  define CONFIG_RA8_ITCM_SIZE         0x4000      /* 16KB - matches linker */
#endif

#ifndef CONFIG_RA8_SRAM_BASE
#  define CONFIG_RA8_SRAM_BASE         0x22060000  /* Main SRAM base */
#endif

#ifndef CONFIG_RA8_SRAM_SIZE
#  define CONFIG_RA8_SRAM_SIZE         0x80000     /* 512KB - matches linker */
#endif

#ifndef CONFIG_RA8_EXTERNAL_RAM_BASE
#  define CONFIG_RA8_EXTERNAL_RAM_BASE 0x80000000  /* OSPI0 CS0 from linker */
#endif

#ifndef CONFIG_RA8_EXTERNAL_RAM_SIZE
#  define CONFIG_RA8_EXTERNAL_RAM_SIZE 0x1000000   /* 16MB OSPI0 CS0 */
#endif

#ifndef CONFIG_RA8_HEAP_ALIGNMENT
#  define CONFIG_RA8_HEAP_ALIGNMENT    8          /* 8-byte alignment */
#endif

#ifndef CONFIG_RA8_STACK_GUARD_SIZE
#  define CONFIG_RA8_STACK_GUARD_SIZE  1024       /* 1KB guard */
#endif

/* Derived memory definitions for compatibility */
#define CONFIG_RAM_END                  (CONFIG_RA8_SRAM_BASE + CONFIG_RA8_SRAM_SIZE)

/* FSP-Based Option Function Select Register Settings */

#ifndef RA_HOCO_FREQUENCY
#  if CONFIG_RA8_HOCO_FREQUENCY == 16000000
#    define RA_HOCO_FREQUENCY          0
#  elif CONFIG_RA8_HOCO_FREQUENCY == 18000000
#    define RA_HOCO_FREQUENCY          1
#  elif CONFIG_RA8_HOCO_FREQUENCY == 20000000
#    define RA_HOCO_FREQUENCY          2
#  elif CONFIG_RA8_HOCO_FREQUENCY == 24000000
#    define RA_HOCO_FREQUENCY          4
#  elif CONFIG_RA8_HOCO_FREQUENCY == 32000000
#    define RA_HOCO_FREQUENCY          5
#  elif CONFIG_RA8_HOCO_FREQUENCY == 48000000
#    define RA_HOCO_FREQUENCY          7
#  elif CONFIG_RA8_HOCO_FREQUENCY == 64000000
#    define RA_HOCO_FREQUENCY          8
#  else
#    define RA_HOCO_FREQUENCY          7         /* Default 48MHz */
#  endif
#endif

#ifndef RA_HOCOEN
#  ifdef CONFIG_RA8_HOCO_ENABLE
#    define RA_HOCOEN                  0         /* HOCO enabled */
#  else
#    define RA_HOCOEN                  1         /* HOCO disabled */
#  endif
#endif

/* FSP-Based ID Code Configuration */

#ifndef IDCODE1
#  define IDCODE1                      0xFFFFFFFF
#endif

#ifndef IDCODE2
#  define IDCODE2                      0xFFFFFFFF
#endif

#ifndef IDCODE3
#  define IDCODE3                      0xFFFFFFFF
#endif

#ifndef IDCODE4
#  define IDCODE4                      0xFFFFFFFF
#endif

/* FSP-Based Hardware Register Base Addresses */

#ifndef RA_GPT_BASE
#  define RA_GPT_BASE                  0x40078000
#endif

/* FSP-Based GPT Register Offsets */

#define RA_GPT_GTSTR_OFFSET            0x004
#define RA_GPT_GTSTP_OFFSET            0x008
#define RA_GPT_GTWP_OFFSET             0x00C
#define RA_GPT_GTUD_OFFSET             0x400
#define RA_GPT_GTCR_OFFSET             0x500
#define RA_GPT_GTPR_OFFSET             0x600
#define RA_GPT_GTIER_OFFSET            0x700

/* FSP-Based GPT Control Values */

#define RA_GPT_GTWP_WP_KEY             0xA500
#define RA_GPT_GTWP_WP_SHIFT           16

/* FSP-Based IRQ Numbers */

#ifndef RA_IRQ_GPT0_COMPARE_A
#  define RA_IRQ_GPT0_COMPARE_A        32        /* GPT0 Compare A IRQ */
#endif

/* FSP-Based Feature Flags */

#ifdef CONFIG_RA8_FSP_COMPATIBILITY
#  define RA8_FSP_COMPATIBLE           1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FSP-Compatible Clock Configuration Structure */

typedef struct
{
  uint32_t system_clock_freq;
  uint32_t iclk_freq;
  uint32_t pclkb_freq;
  uint32_t pclkd_freq;
  uint8_t  clock_source;
  uint8_t  hoco_frequency;
  bool     pll_enabled;
  bool     hoco_enabled;
  bool     moco_enabled;
} ra_fsp_clock_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ra_fsp_get_clock_config
 *
 * Description:
 *   Get current FSP-compatible clock configuration
 *
 ****************************************************************************/

void ra_fsp_get_clock_config(ra_fsp_clock_config_t *config);

/****************************************************************************
 * Name: ra_fsp_validate_memory_map
 *
 * Description:
 *   Validate memory configuration against FSP requirements
 *
 ****************************************************************************/

bool ra_fsp_validate_memory_map(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_FSP_INTEGRATION_H */
