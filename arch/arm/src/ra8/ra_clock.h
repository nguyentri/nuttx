/****************************************************************************
 * arch/arm/src/ra8/ra_clock.h
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

#ifndef __ARCH_ARM_SRC_RA_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_RA_CLOCKCONFIG_H

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

#ifndef CONFIG_RA_HOCO_FREQUENCY
#  define CONFIG_RA_HOCO_FREQUENCY    48000000  /* 48MHz HOCO */
#endif

#ifndef CONFIG_RA_MOCO_FREQUENCY
#  define CONFIG_RA_MOCO_FREQUENCY    8000000   /* 8MHz MOCO */
#endif

#ifndef CONFIG_RA_LOCO_FREQUENCY
#  define CONFIG_RA_LOCO_FREQUENCY    32768     /* 32.768kHz LOCO */
#endif

/* FSP-Based System Clock Selection */

#ifndef RA_CKSEL
#  ifdef CONFIG_RA_CLOCK_PLL
#    define RA_CKSEL                   5         /* PLL */
#  elif defined(CONFIG_RA_CLOCK_HOCO)
#    define RA_CKSEL                   0         /* HOCO */
#  elif defined(CONFIG_RA_CLOCK_MOCO)
#    define RA_CKSEL                   1         /* MOCO */
#  else
#    define RA_CKSEL                   0         /* Default to HOCO */
#  endif
#endif

/* FSP-Based PLL Configuration */

#ifndef CONFIG_RA_PLL_SOURCE
#  define CONFIG_RA_PLL_SOURCE        0         /* HOCO as PLL source */
#endif

#ifndef CONFIG_RA_PLL_DIV
#  define CONFIG_RA_PLL_DIV           0         /* PLL input divider */
#endif

#ifndef CONFIG_RA_PLL_MUL
#  define CONFIG_RA_PLL_MUL           20        /* PLL multiplier for 240MHz */
#endif

#define RA_PLL_DIV                     CONFIG_RA_PLL_DIV
#define RA_PLL_MUL                     CONFIG_RA_PLL_MUL

/* FSP-Based Clock Frequencies */

#if defined(CONFIG_RA_CLOCK_PLL)
#  define RA_SYSTEM_CLOCK_FREQUENCY    (CONFIG_RA_HOCO_FREQUENCY * RA_PLL_MUL / (RA_PLL_DIV + 1))
#elif defined(CONFIG_RA_CLOCK_HOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_HOCO_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_MOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_MOCO_FREQUENCY
#else
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_HOCO_FREQUENCY
#endif

/* FSP-Based Clock Dividers */

#ifndef CONFIG_RA_ICK_DIV
#  define CONFIG_RA_ICK_DIV           0         /* ICLK = System clock / 1 */
#endif

#ifndef CONFIG_RA_PCKB_DIV
#  define CONFIG_RA_PCKB_DIV          1         /* PCKB = System clock / 2 */
#endif

#ifndef CONFIG_RA_PCKD_DIV
#  define CONFIG_RA_PCKD_DIV          2         /* PCKD = System clock / 4 */
#endif

#define RA_ICK_DIV                     (CONFIG_RA_ICK_DIV << 24)
#define RA_FCK_DIV                     (CONFIG_RA_ICK_DIV << 28)
#define RA_PCKA_DIV                    (CONFIG_RA_ICK_DIV << 12)
#define RA_PCKB_DIV                    (CONFIG_RA_PCKB_DIV << 8)
#define RA_PCKC_DIV                    (CONFIG_RA_PCKB_DIV << 4)
#define RA_PCKD_DIV                    (CONFIG_RA_PCKD_DIV << 0)

/* FSP-Based Derived Frequencies */

#define RA_ICLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY >> CONFIG_RA_ICK_DIV)
#define RA_PCLKB_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY >> CONFIG_RA_PCKB_DIV)
#define RA_PCLKD_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY >> CONFIG_RA_PCKD_DIV)

/* FSP-Based Option Function Select Register Settings */

#ifndef RA_HOCO_FREQUENCY
#  if CONFIG_RA_HOCO_FREQUENCY == 16000000
#    define RA_HOCO_FREQUENCY          0
#  elif CONFIG_RA_HOCO_FREQUENCY == 18000000
#    define RA_HOCO_FREQUENCY          1
#  elif CONFIG_RA_HOCO_FREQUENCY == 20000000
#    define RA_HOCO_FREQUENCY          2
#  elif CONFIG_RA_HOCO_FREQUENCY == 24000000
#    define RA_HOCO_FREQUENCY          4
#  elif CONFIG_RA_HOCO_FREQUENCY == 32000000
#    define RA_HOCO_FREQUENCY          5
#  elif CONFIG_RA_HOCO_FREQUENCY == 48000000
#    define RA_HOCO_FREQUENCY          7
#  elif CONFIG_RA_HOCO_FREQUENCY == 64000000
#    define RA_HOCO_FREQUENCY          8
#  else
#    define RA_HOCO_FREQUENCY          7         /* Default 48MHz */
#  endif
#endif

#ifndef RA_HOCOEN
#  ifdef CONFIG_RA_HOCO_ENABLE
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
} ra_clock_config_t;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra_clock
 *
 * Description:
 *   Called to initialize the RA.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void ra_clock(void);

/****************************************************************************
 * Name: ra_get_clock_config
 *
 * Description:
 *   Get current FSP-compatible clock configuration
 *
 ****************************************************************************/

void ra_get_clock_config(ra_clock_config_t *config);

/****************************************************************************
 * Name: ra_print_clock_info
 *
 * Description:
 *   Print FSP-compatible clock information for debugging
 *
 ****************************************************************************/

void ra_print_clock_info(void);

/****************************************************************************
 * Name: ra_get_peripheral_clock
 *
 * Description:
 *   Get peripheral clock frequency for FSP compatibility
 *
 ****************************************************************************/

uint32_t ra_get_peripheral_clock(int peripheral_id);

/****************************************************************************
 * Name: ra_mem_validate
 *
 * Description:
 *   Validate memory configuration against FSP requirements
 *
 ****************************************************************************/

bool ra_mem_validate(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA_CLOCKCONFIG_H */
