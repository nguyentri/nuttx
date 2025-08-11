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
#  define CONFIG_RA_HOCO_FREQUENCY    20000000  /* 20MHz HOCO (default for RA8E1) */
#endif

#ifndef CONFIG_RA_MOCO_FREQUENCY
#  define CONFIG_RA_MOCO_FREQUENCY    8000000   /* 8MHz MOCO */
#endif

#ifndef CONFIG_RA_LOCO_FREQUENCY
#  define CONFIG_RA_LOCO_FREQUENCY    32768     /* 32.768kHz LOCO */
#endif

#ifndef CONFIG_RA_XTAL_FREQUENCY
#  define CONFIG_RA_XTAL_FREQUENCY    20000000  /* 20MHz External Crystal */
#endif

#ifndef CONFIG_RA_MAIN_OSC_FREQUENCY
#  define CONFIG_RA_MAIN_OSC_FREQUENCY CONFIG_RA_XTAL_FREQUENCY
#endif

/* FSP-Based System Clock Selection */

#ifndef RA_CKSEL
#  ifdef CONFIG_RA_CLOCK_PLL1P
#    define RA_CKSEL                   5         /* PLL1P */
#  elif defined(CONFIG_RA_CLOCK_PLL)
#    define RA_CKSEL                   5         /* PLL */
#  elif defined(CONFIG_RA_CLOCK_HOCO)
#    define RA_CKSEL                   0         /* HOCO */
#  elif defined(CONFIG_RA_CLOCK_MOCO)
#    define RA_CKSEL                   1         /* MOCO */
#  elif defined(CONFIG_RA_CLOCK_MAIN_OSC)
#    define RA_CKSEL                   3         /* Main OSC */
#  else
#    define RA_CKSEL                   0         /* Default to HOCO */
#  endif
#endif

/* Clock Source Definitions (FSP Compatible) */
#define RA_CLOCKS_SOURCE_CLOCK_HOCO      0     /* HOCO */
#define RA_CLOCKS_SOURCE_CLOCK_MOCO      1     /* MOCO */
#define RA_CLOCKS_SOURCE_CLOCK_LOCO      2     /* LOCO */
#define RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC  3     /* Main OSC */
#define RA_CLOCKS_SOURCE_CLOCK_SUBCLOCK  4     /* Sub-clock */
#define RA_CLOCKS_SOURCE_CLOCK_PLL       5     /* PLL */
#define RA_CLOCKS_SOURCE_CLOCK_PLL1P     5     /* PLL1P (alias for PLL) */
#define RA_CLOCKS_SOURCE_CLOCK_PLL1Q     7     /* PLL1Q */
#define RA_CLOCKS_SOURCE_CLOCK_PLL1R     8     /* PLL1R */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2      6     /* PLL2 */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2P     6     /* PLL2P */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2Q     9     /* PLL2Q */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2R    10     /* PLL2R */
#define RA_CLOCKS_CLOCK_DISABLED        15     /* Clock disabled */

/* FSP-Based PLL Configuration */

#ifndef CONFIG_RA_PLL_SOURCE
#  define CONFIG_RA_PLL_SOURCE        RA_CLOCKS_SOURCE_CLOCK_HOCO  /* HOCO as PLL source */
#endif

#ifndef CONFIG_RA_PLL_DIV
#  define CONFIG_RA_PLL_DIV           1         /* PLL input divider /2 (FSP value 1 = /2) */
#endif

#ifndef CONFIG_RA_PLL_MUL
#  define CONFIG_RA_PLL_MUL           72        /* PLL multiplier x72 for 720MHz */
#endif

/* PLL Output Dividers (RA8E1 specific) */
#ifndef CONFIG_RA_PLL1P_DIV
#  define CONFIG_RA_PLL1P_DIV         1         /* PLL1P divider /2 (FSP value 1 = /2) */
#endif

#ifndef CONFIG_RA_PLL1Q_DIV
#  define CONFIG_RA_PLL1Q_DIV         1         /* PLL1Q divider /2 */
#endif

#ifndef CONFIG_RA_PLL1R_DIV
#  define CONFIG_RA_PLL1R_DIV         1         /* PLL1R divider /2 */
#endif

/* PLL2 Configuration */
#ifndef CONFIG_RA_PLL2_SOURCE
#  define CONFIG_RA_PLL2_SOURCE       RA_CLOCKS_CLOCK_DISABLED  /* PLL2 disabled by default */
#endif

#ifndef CONFIG_RA_PLL2_DIV
#  define CONFIG_RA_PLL2_DIV          1         /* PLL2 input divider */
#endif

#ifndef CONFIG_RA_PLL2_MUL
#  define CONFIG_RA_PLL2_MUL          96        /* PLL2 multiplier */
#endif

#define RA_PLL_DIV                     CONFIG_RA_PLL_DIV
#define RA_PLL_MUL                     CONFIG_RA_PLL_MUL

/* PLL Frequency Calculations */
#if CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_HOCO
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_HOCO_FREQUENCY
#elif CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_MAIN_OSC_FREQUENCY
#else
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_HOCO_FREQUENCY  /* Default */
#endif

#define RA_PLL_FREQUENCY              (RA_PLL_SOURCE_FREQ * CONFIG_RA_PLL_MUL / (CONFIG_RA_PLL_DIV + 1))
#define RA_PLL1P_FREQUENCY            (RA_PLL_FREQUENCY / (CONFIG_RA_PLL1P_DIV + 1))
#define RA_PLL1Q_FREQUENCY            (RA_PLL_FREQUENCY / (CONFIG_RA_PLL1Q_DIV + 1))
#define RA_PLL1R_FREQUENCY            (RA_PLL_FREQUENCY / (CONFIG_RA_PLL1R_DIV + 1))

/* FSP-Based Clock Frequencies */

#if defined(CONFIG_RA_CLOCK_PLL1P) || defined(CONFIG_RA_CLOCK_PLL)
#  define RA_SYSTEM_CLOCK_FREQUENCY    RA_PLL1P_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_HOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_HOCO_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_MOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_MOCO_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_MAIN_OSC)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_MAIN_OSC_FREQUENCY
#else
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_HOCO_FREQUENCY
#endif

/* System Clock Divider Values (FSP Compatible) */
#define RA_CLOCKS_SYS_CLOCK_DIV_1        0    /* /1 */
#define RA_CLOCKS_SYS_CLOCK_DIV_2        1    /* /2 */
#define RA_CLOCKS_SYS_CLOCK_DIV_4        2    /* /4 */
#define RA_CLOCKS_SYS_CLOCK_DIV_8        3    /* /8 */
#define RA_CLOCKS_SYS_CLOCK_DIV_16       4    /* /16 */
#define RA_CLOCKS_SYS_CLOCK_DIV_32       5    /* /32 */
#define RA_CLOCKS_SYS_CLOCK_DIV_64       6    /* /64 */
#define RA_CLOCKS_SYS_CLOCK_DIV_128      7    /* /128 */
#define RA_CLOCKS_SYS_CLOCK_DIV_3        8    /* /3 */
#define RA_CLOCKS_SYS_CLOCK_DIV_6        9    /* /6 */
#define RA_CLOCKS_SYS_CLOCK_DIV_12      10    /* /12 */
#define RA_CLOCKS_SYS_CLOCK_DIV_24      11    /* /24 */

/* FSP-Based Clock Dividers */

#ifndef CONFIG_RA_CPUCLK_DIV
#  define CONFIG_RA_CPUCLK_DIV        RA_CLOCKS_SYS_CLOCK_DIV_1   /* CPU clock = System clock / 1 */
#endif

#ifndef CONFIG_RA_ICK_DIV
#  define CONFIG_RA_ICK_DIV           RA_CLOCKS_SYS_CLOCK_DIV_3   /* ICLK = System clock / 3 */
#endif

#ifndef CONFIG_RA_PCKA_DIV
#  define CONFIG_RA_PCKA_DIV          RA_CLOCKS_SYS_CLOCK_DIV_3   /* PCKA = System clock / 3 */
#endif

#ifndef CONFIG_RA_PCKB_DIV
#  define CONFIG_RA_PCKB_DIV          RA_CLOCKS_SYS_CLOCK_DIV_6   /* PCKB = System clock / 6 */
#endif

#ifndef CONFIG_RA_PCKC_DIV
#  define CONFIG_RA_PCKC_DIV          RA_CLOCKS_SYS_CLOCK_DIV_6   /* PCKC = System clock / 6 */
#endif

#ifndef CONFIG_RA_PCKD_DIV
#  define CONFIG_RA_PCKD_DIV          RA_CLOCKS_SYS_CLOCK_DIV_3   /* PCKD = System clock / 3 */
#endif

#ifndef CONFIG_RA_PCKЕ_DIV
#  define CONFIG_RA_PCKЕ_DIV          RA_CLOCKS_SYS_CLOCK_DIV_3   /* PCKE = System clock / 3 */
#endif

#ifndef CONFIG_RA_BCLK_DIV
#  define CONFIG_RA_BCLK_DIV          RA_CLOCKS_SYS_CLOCK_DIV_12  /* BCLK = System clock / 12 */
#endif

#ifndef CONFIG_RA_FCLK_DIV
#  define CONFIG_RA_FCLK_DIV          RA_CLOCKS_SYS_CLOCK_DIV_6   /* FCLK = System clock / 6 */
#endif

/* Peripheral Clock Sources and Dividers */
#ifndef CONFIG_RA_SCICLK_SOURCE
#  define CONFIG_RA_SCICLK_SOURCE     RA_CLOCKS_SOURCE_CLOCK_PLL1P /* SCI clock source */
#endif

#ifndef CONFIG_RA_SCICLK_DIV
#  define CONFIG_RA_SCICLK_DIV        2                           /* SCI clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_SPICLK_SOURCE
#  define CONFIG_RA_SPICLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* SPI clock disabled */
#endif

#ifndef CONFIG_RA_CANFDCLK_SOURCE
#  define CONFIG_RA_CANFDCLK_SOURCE   RA_CLOCKS_CLOCK_DISABLED    /* CANFD clock disabled */
#endif

#ifndef CONFIG_RA_USBCLK_SOURCE
#  define CONFIG_RA_USBCLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* USB clock disabled */
#endif

#ifndef CONFIG_RA_OCTACLK_SOURCE
#  define CONFIG_RA_OCTACLK_SOURCE    RA_CLOCKS_CLOCK_DISABLED    /* OCTA clock disabled */
#endif

/* Clock Divider Shift Positions for Register Settings */
#define RA_ICK_DIV                     (CONFIG_RA_ICK_DIV << 24)
#define RA_FCK_DIV                     (CONFIG_RA_FCLK_DIV << 28)
#define RA_PCKA_DIV                    (CONFIG_RA_PCKA_DIV << 12)
#define RA_PCKB_DIV                    (CONFIG_RA_PCKB_DIV << 8)
#define RA_PCKC_DIV                    (CONFIG_RA_PCKC_DIV << 4)
#define RA_PCKD_DIV                    (CONFIG_RA_PCKD_DIV << 0)

/* Helper macro to convert divider value to actual divisor */
#define RA_DIV_TO_DIVISOR(div) \
  (((div) >= 8) ? (((div) == 8) ? 3 : \
                   ((div) == 9) ? 6 : \
                   ((div) == 10) ? 12 : \
                   ((div) == 11) ? 24 : (1 << (div))) : (1 << (div)))

/* FSP-Based Derived Frequencies */

#define RA_CPUCLK_FREQUENCY            (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_CPUCLK_DIV))
#define RA_ICLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_ICK_DIV))
#define RA_PCLKA_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKA_DIV))
#define RA_PCLKB_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKB_DIV))
#define RA_PCLKC_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKC_DIV))
#define RA_PCLKD_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKD_DIV))
#define RA_PCLKE_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKЕ_DIV))
#define RA_BCLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_BCLK_DIV))
#define RA_FCLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_FCLK_DIV))

/* Peripheral Clock Frequencies */
#if CONFIG_RA_SCICLK_SOURCE == RA_CLOCKS_SOURCE_CLOCK_PLL1P
#  define RA_SCICLK_FREQUENCY          (RA_PLL1P_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_SCICLK_DIV))
#elif CONFIG_RA_SCICLK_SOURCE == RA_CLOCKS_CLOCK_DISABLED
#  define RA_SCICLK_FREQUENCY          0
#else
#  define RA_SCICLK_FREQUENCY          RA_PCLKB_FREQUENCY  /* Default to PCLKB */
#endif

/* FSP-Based Option Function Select Register Settings */

/* HOCO Frequency Options for RA8E1 */
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
#    define RA_HOCO_FREQUENCY          2         /* Default 20MHz for RA8E1 */
#  endif
#endif

#ifndef RA_HOCOEN
#  ifdef CONFIG_RA_HOCO_ENABLE
#    define RA_HOCOEN                  0         /* HOCO enabled */
#  else
#    define RA_HOCOEN                  1         /* HOCO disabled */
#  endif
#endif

/* PLL Divider Values (FSP Compatible) */
#define RA_CLOCKS_PLL_DIV_1            0
#define RA_CLOCKS_PLL_DIV_2            1
#define RA_CLOCKS_PLL_DIV_3            2
#define RA_CLOCKS_PLL_DIV_4            3
#define RA_CLOCKS_PLL_DIV_5            4
#define RA_CLOCKS_PLL_DIV_6            5
#define RA_CLOCKS_PLL_DIV_8            7
#define RA_CLOCKS_PLL_DIV_9            8
#define RA_CLOCKS_PLL_DIV_16          15

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
  uint32_t cpu_clock_freq;
  uint32_t iclk_freq;
  uint32_t pclka_freq;
  uint32_t pclkb_freq;
  uint32_t pclkc_freq;
  uint32_t pclkd_freq;
  uint32_t pclke_freq;
  uint32_t bclk_freq;
  uint32_t fclk_freq;
  uint32_t sciclk_freq;
  uint32_t pll_freq;
  uint32_t pll1p_freq;
  uint32_t pll1q_freq;
  uint32_t pll1r_freq;
  uint8_t  clock_source;
  uint8_t  hoco_frequency;
  uint8_t  pll_source;
  uint8_t  pll_div;
  uint8_t  pll_mul;
  bool     pll_enabled;
  bool     pll2_enabled;
  bool     hoco_enabled;
  bool     moco_enabled;
  bool     main_osc_enabled;
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
