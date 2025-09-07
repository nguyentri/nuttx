/****************************************************************************
 * arch/arm/src/ra8/ra_start.h
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

#ifndef __ARCH_ARM_SRC_RA_START_H
#define __ARCH_ARM_SRC_RA_START_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Compiler-specific macros */
#if defined(__ARMCC_VERSION)
  #define RA_UNINIT_SECTION_PREFIX         ".bss"
  #define RA_DONT_REMOVE                   __attribute__((used))
  #define RA_FORCE_INLINE                  __attribute__((always_inline))
#elif defined(__GNUC__)
  #define RA_UNINIT_SECTION_PREFIX
  #define RA_DONT_REMOVE                   __attribute__((used))
  #define RA_ATTRIBUTE_STACKLESS           __attribute__((naked))
  #define RA_FORCE_INLINE                  __attribute__((always_inline))
#elif defined(__ICCARM__)
  #define RA_UNINIT_SECTION_PREFIX
  #define RA_DONT_REMOVE                   __root
  #define RA_FORCE_INLINE                  _Pragma("inline=forced")
#endif

/* Linker section macros */
#define RA_PLACE_IN_SECTION(x)    __attribute__((section(x))) __attribute__((__used__))
#define RA_ALIGN_VARIABLE(x)      __attribute__((aligned(x)))

/* Stack and heap alignment */
#define RA_STACK_ALIGNMENT        (8)

/* TrustZone build configuration macros */
#ifndef CONFIG_RA_TZ_SECURE_BUILD
#  define CONFIG_RA_TZ_SECURE_BUILD 0
#endif

#ifndef CONFIG_RA_TZ_NONSECURE_BUILD
#  define CONFIG_RA_TZ_NONSECURE_BUILD 0
#endif

/* Option setting register definitions */
#define RA_OFS1_HOCOFRQ_OFFSET    (9UL)

/* IWDT configuration based on Kconfig */
#ifdef CONFIG_RA_IWDT_ENABLE
#  define RA_OFS_IWDT_START       (1 << 1)  /* Auto-start mode */
#  define RA_OFS_IWDT_TIMEOUT     (CONFIG_RA_IWDT_TIMEOUT << 2)
#  define RA_OFS_IWDT_CLK_DIV     (CONFIG_RA_IWDT_CLK_DIV << 4)
#  define RA_OFS_IWDT_WINDOW_END  (CONFIG_RA_IWDT_WINDOW_END << 8)
#  define RA_OFS_IWDT_WINDOW_START (CONFIG_RA_IWDT_WINDOW_START << 10)
#  define RA_OFS_IWDT_RESET_IRQ   (CONFIG_RA_IWDT_RESET_IRQ << 12)
#  define RA_OFS_IWDT_STOP_CTRL   (CONFIG_RA_IWDT_STOP_CTRL << 14)
#else
#  define RA_OFS_IWDT_START       (0 << 1)  /* Register-start mode */
#  define RA_OFS_IWDT_TIMEOUT     (3 << 2)  /* 2048 cycles */
#  define RA_OFS_IWDT_CLK_DIV     (15 << 4) /* Divide by 128 */
#  define RA_OFS_IWDT_WINDOW_END  (3 << 8)  /* 0% (no window) */
#  define RA_OFS_IWDT_WINDOW_START (3 << 10) /* 100% (no window) */
#  define RA_OFS_IWDT_RESET_IRQ   (1 << 12) /* Reset enabled */
#  define RA_OFS_IWDT_STOP_CTRL   (1 << 14) /* Stop in low power modes */
#endif

/* WDT configuration based on Kconfig */
#ifdef CONFIG_RA_WDT_ENABLE
#  define RA_OFS_WDT_START        (1 << 17) /* Auto-start mode */
#  define RA_OFS_WDT_TIMEOUT      (CONFIG_RA_WDT_TIMEOUT << 18)
#  define RA_OFS_WDT_CLK_DIV      (CONFIG_RA_WDT_CLK_DIV << 20)
#  define RA_OFS_WDT_WINDOW_END   (CONFIG_RA_WDT_WINDOW_END << 24)
#  define RA_OFS_WDT_WINDOW_START (CONFIG_RA_WDT_WINDOW_START << 26)
#  define RA_OFS_WDT_RESET_IRQ    (CONFIG_RA_WDT_RESET_IRQ << 28)
#  define RA_OFS_WDT_STOP_CTRL    (CONFIG_RA_WDT_STOP_CTRL << 30)
#else
#  define RA_OFS_WDT_START        (0 << 17) /* Register-start mode */
#  define RA_OFS_WDT_TIMEOUT      (3 << 18) /* 16384 cycles */
#  define RA_OFS_WDT_CLK_DIV      (15 << 20) /* Divide by 128 */
#  define RA_OFS_WDT_WINDOW_END   (3 << 24) /* 0% (no window) */
#  define RA_OFS_WDT_WINDOW_START (3 << 26) /* 100% (no window) */
#  define RA_OFS_WDT_RESET_IRQ    (1 << 28) /* Reset enabled */
#  define RA_OFS_WDT_STOP_CTRL    (1 << 30) /* Stop in low power modes */
#endif

/* Computed option setting values */
#define RA_OPTION_SETTING_OFS0 (0xA001A001 | RA_OFS_IWDT_START | RA_OFS_IWDT_TIMEOUT | \
                                  RA_OFS_IWDT_CLK_DIV | RA_OFS_IWDT_WINDOW_END | RA_OFS_IWDT_WINDOW_START | \
                                  RA_OFS_IWDT_RESET_IRQ | RA_OFS_IWDT_STOP_CTRL | RA_OFS_WDT_START | \
                                  RA_OFS_WDT_TIMEOUT | RA_OFS_WDT_CLK_DIV | RA_OFS_WDT_WINDOW_END | \
                                  RA_OFS_WDT_WINDOW_START | RA_OFS_WDT_RESET_IRQ | RA_OFS_WDT_STOP_CTRL)

#define RA_OPTION_SETTING_OFS2 ((1 << 0) | (0xFFFFFFFE)) /* DCDC enabled */

/* OFS1_SEC based on Kconfig */
#ifdef CONFIG_RA_HOCO_ENABLE
#  define RA_OFS1_SEC_HOCO_FREQ  (RA_HOCO_FREQUENCY << RA_OFS1_HOCOFRQ_OFFSET)
#else
#  define RA_OFS1_SEC_HOCO_FREQ  (0)
#endif

/* #define RA_OPTION_SETTING_OFS1_SEC (0xFCFFF0D0 | (1 << 3) | 7 | (1 << 5) | (1 << 8) | \
                                 (1 << 24) | (0 << 25) | RA_OFS1_SEC_HOCO_FREQ) */
#define RA_OPTION_SETTING_OFS1_SEC 0xFDFFF5FF /* Default OFS1_SEC value, can be overridden by Kconfig */

/* OFS1_SEL for TrustZone security attribution */
#if CONFIG_RA_TZ_SECURE_BUILD || CONFIG_RA_TZ_NONSECURE_BUILD
#  define RA_OPTION_SETTING_OFS1_SEL (0x0F00) /* Load from secure settings */
#else
#  define RA_OPTION_SETTING_OFS1_SEL (0x00000000)
#endif

/* option setting default values */
#define RA_OPTION_SETTING_OFS1      (0xFFFFFFFF) /* Default OFS1 value */
#define RA_OPTION_SETTING_DUALSEL   (0xFFFFFFFF) /* Default dual bank select */
#define RA_OPTION_SETTING_BANKSEL   (0xFFFFFFFF) /* Default bank select */
#define RA_OPTION_SETTING_BPS       (0xFFFFFFFF) /* Default boot protection */
#define RA_OPTION_SETTING_PBPS      (0xFFFFFFFF) /* Default P/E boot protection */

/* Secure versions of option settings (for TrustZone secure builds) */
#define RA_OPTION_SETTING_BANKSEL_SEC  (0xFFFFFFFF)
#define RA_OPTION_SETTING_BPS_SEC      (0xFFFFFFFF)
#define RA_OPTION_SETTING_PBPS_SEC     (0xFFFFFFFF)

/* Selection registers for TrustZone security attribution */
#define RA_OPTION_SETTING_BANKSEL_SEL  (0x0000)
#define RA_OPTION_SETTING_BPS_SEL      (0x0000)

/* NuttX-style configuration macros - use CONFIG settings when available, else defaults */
#ifdef CONFIG_RA_OFS0_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_OFS0
#    define CONFIG_RA_OPTION_SETTING_OFS0      RA_OPTION_SETTING_OFS0
#  endif
#endif

#ifdef CONFIG_RA_OFS1_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_OFS1
#    define CONFIG_RA_OPTION_SETTING_OFS1      RA_OPTION_SETTING_OFS1
#  endif
#endif

#ifdef CONFIG_RA_OFS2_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_OFS2
#    define CONFIG_RA_OPTION_SETTING_OFS2      RA_OPTION_SETTING_OFS2
#  endif
#endif

#ifdef CONFIG_RA_DUAL_BANK_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_DUALSEL
#    define CONFIG_RA_OPTION_SETTING_DUALSEL   RA_OPTION_SETTING_DUALSEL
#  endif
#endif

#ifdef CONFIG_RA_BANK_SELECT_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_BANKSEL
#    define CONFIG_RA_OPTION_SETTING_BANKSEL   RA_OPTION_SETTING_BANKSEL
#  endif
#  ifndef CONFIG_RA_OPTION_SETTING_BANKSEL_SEC
#    define CONFIG_RA_OPTION_SETTING_BANKSEL_SEC   RA_OPTION_SETTING_BANKSEL_SEC
#  endif
#  ifndef CONFIG_RA_OPTION_SETTING_BANKSEL_SEL
#    define CONFIG_RA_OPTION_SETTING_BANKSEL_SEL   RA_OPTION_SETTING_BANKSEL_SEL
#  endif
#endif

#ifdef CONFIG_RA_BOOT_PROTECT_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_BPS
#    define CONFIG_RA_OPTION_SETTING_BPS       RA_OPTION_SETTING_BPS
#  endif
#  ifndef CONFIG_RA_OPTION_SETTING_PBPS
#    define CONFIG_RA_OPTION_SETTING_PBPS      RA_OPTION_SETTING_PBPS
#  endif
#  ifndef CONFIG_RA_OPTION_SETTING_BPS_SEC
#    define CONFIG_RA_OPTION_SETTING_BPS_SEC       RA_OPTION_SETTING_BPS_SEC
#  endif
#  ifndef CONFIG_RA_OPTION_SETTING_PBPS_SEC
#    define CONFIG_RA_OPTION_SETTING_PBPS_SEC      RA_OPTION_SETTING_PBPS_SEC
#  endif
#  ifndef CONFIG_RA_OPTION_SETTING_BPS_SEL
#    define CONFIG_RA_OPTION_SETTING_BPS_SEL       RA_OPTION_SETTING_BPS_SEL
#  endif
#endif

#ifdef CONFIG_RA_OFS1_SEC_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_OFS1_SEC
#    define CONFIG_RA_OPTION_SETTING_OFS1_SEC      RA_OPTION_SETTING_OFS1_SEC
#  endif
#endif

#ifdef CONFIG_RA_OFS1_SEL_SETTING
#  ifndef CONFIG_RA_OPTION_SETTING_OFS1_SEL
#    define CONFIG_RA_OPTION_SETTING_OFS1_SEL     RA_OPTION_SETTING_OFS1_SEL
#  endif
#endif


/* SYSTEM Control Register Bits */
#define R_SYSTEM_PRCR_PRKEY               (0xA500)      /* Protection Key */
#define R_SYSTEM_PRCR_PRC0                (1 << 0)      /* Protect bit 0 */
#define R_SYSTEM_PRCR_PRC1                (1 << 1)      /* Protect bit 1 */
#define R_SYSTEM_PRCR_PRC3                (1 << 3)      /* Protect bit 3 */
#define R_SYSTEM_PRCR_PRC4                (1 << 4)      /* Protect bit 4 */

/* Register Protection Types */
typedef enum
{
    RA_REG_PROTECT_CGC = 0,            /* PRC0: Clock generation circuit */
    RA_REG_PROTECT_OM_LPC_BATT,        /* PRC1: Operating mode, LPC, battery backup */
    RA_REG_PROTECT_LVD,                /* PRC3: LVD */
    RA_REG_PROTECT_SAR                 /* PRC4: SAR registers */
} ra_reg_protect_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ra_board_initialize(void);
void ra_option_bytes_init(void);
void ra_trustzone_init(void);
void ra_vector_table_init(void);
void ra_ram_init (const uint32_t external);
void ra_delay_us(uint32_t delay_us);

/****************************************************************************
 * Name: ra_register_protect_enable
 *
 * Description:
 *   Enable register protection (FSP-compatible)
 *
 ****************************************************************************/

void ra_register_protect_enable(ra_reg_protect_t regs_to_protect);

/****************************************************************************
 * Name: ra_register_protect_disable
 *
 * Description:
 *   Disable register protection (FSP-compatible)
 *
 ****************************************************************************/

void ra_register_protect_disable(ra_reg_protect_t regs_to_unprotect);


/****************************************************************************
 * Name: ra_gpio_init_security_attribution
 *
 * Description:
 *   Initialize PMSAR and PSCU registers to their default values.
 *   Sets all port pins to secure mode (0) as per FSP implementation.
 *   Must be called before configuring any port pins.
 *
 ****************************************************************************/

void ra_gpio_init_security_attribution(void);


#endif /* __ARCH_ARM_SRC_RA_START_H */
