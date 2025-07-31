/****************************************************************************
 * arch/arm/src/ra8/ra_clockconfig.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "ra_clockconfig.h"
#include "hardware/ra_flash.h"
#include "hardware/ra_system.h"
#include "hardware/ra_option_setting.h"

//const uint32_t option_settings[] __attribute__((section(".rom_registers")))
//__attribute__((__used__)) =
//{
//  /* Option Function Select Register 0 */
//
//  (
//  R_OFS0_RESERVED_31 | R_OFS0_WDTSTPCTL | R_OFS0_RESERVED_29 |
//  R_OFS0_WDTRSTIRQS | R_OFS0_WDTRPSS_MASK | R_OFS0_WDTRPES_MASK |
//  R_OFS0_WDTCKS_MASK | R_OFS0_WDTTOPS_MASK | R_OFS0_WDTSTRT |
//  R_OFS0_RESERVED_16_15_MASK | R_OFS0_IWDTSTPCTL | R_OFS0_RESERVED_13 |
//  R_OFS0_IWDTRSTIRQS | R_OFS0_IWDTRPSS_MASK | R_OFS0_IWDTRPES_MASK |
//  R_OFS0_IWDTCKS_MASK | R_OFS0_IWDTTOPS_MASK | R_OFS0_IWDTSTRT |
//  R_OFS0_RESERVED_0
//  ),
//
//  /* Option Function Select Register 1 */
//
//  (
//  R_OFS1_RESERVED_16_15_MASK | RA_HOCO_FREQUENCY |
//  R_OFS1_RESERVED_11_9_MASK | RA_HOCOEN | R_OFS1_RESERVED_7_6_MASK |
//  R_OFS1_VDSEL1_MASK | R_OFS1_LVDAS | R_OFS1_RESERVED_1_0_MASK),
//
//  (uint32_t)0x00fffffc,         /* Security MPU Program Counter Start Address
//                                 * Register (SECMPUPCS0) */
//  (uint32_t)0x00ffffff,         /* Security MPU Program Counter End Address
//                                 * Register (SECMPUPCE0)  */
//  (uint32_t)0x00fffffc,         /* Security MPU Program Counter Start Address
//                                 * Register (SECMPUPCS1) */
//  (uint32_t)0x00ffffff,         /* Security MPU Program Counter End Address
//                                 * Register (SECMPUPCE1)  */
//  (uint32_t)0x00fffffc,         /* Security MPU Region 0 Start Address
//                                 * Register (SECMPUS0) */
//  (uint32_t)0x00ffffff,         /* Security MPU Region 0 END Address Register
//                                 * (SECMPUE0) */
//  (uint32_t)0x200ffffc,         /* Security MPU Region 0 Start Address
//                                 * Register (SECMPUS1) */
//  (uint32_t)0x200fffff,         /* Security MPU Region 0 END Address Register
//                                 * (SECMPUE1) */
//  (uint32_t)0x407ffffc,         /* Security MPU Region 0 Start Address
//                                 * Register (SECMPUS2) */
//  (uint32_t)0x407fffff,         /* Security MPU Region 0 END Address Register
//                                 * (SECMPUE2) */
//  (uint32_t)0x40dffffc,         /* Security MPU Region 0 Start Address
//                                 * Register (SECMPUS3) */
//  (uint32_t)0x40dfffff,         /* Security MPU Region 0 END Address Register
//                                 * (SECMPUE3) */
//  (uint32_t)0xffffffff,         /* Security MPU Access Control Register
//                                 * (SECMPUAC) */
//};
//
///** ID code definitions defined here. */
//
//static const uint32_t g_bsp_id_codes[] __attribute__((section(".id_code")))
//__attribute__((__used__)) =
//{
//  IDCODE1, IDCODE2, IDCODE3, IDCODE4
//};
#define BSP_FEATURE_BSP_OFS1_HOCOFRQ_OFFSET                       (9UL)             // Offset to the OFS1.HOCOFRQx bitfield.
#ifndef BSP_CFG_OPTION_SETTING_OFS0
#define OFS_IWDT (0xA001A001 | 1 << 1 | 3 << 2 | 15 << 4 | 3 << 8 | 3 << 10 | 1 << 12 | 1 << 14)
#define OFS_WDT  (1 << 17 | 3 << 18 | 15 << 20 | 3 << 24 | 3 << 26 | 1 << 28 | 1 << 30)
#define BSP_CFG_OPTION_SETTING_OFS0  (OFS_IWDT | OFS_WDT)
#endif
#ifndef BSP_CFG_OPTION_SETTING_OFS2
#define BSP_CFG_OPTION_SETTING_OFS2  ((1 << 0) | (0xFFFFFFFE))
#endif
#ifndef BSP_CFG_OPTION_SETTING_OFS1_SEC
#define BSP_CFG_OPTION_SETTING_OFS1_SEC_NO_HOCOFRQ (0xFCFFF0D0 | 1 <<3 | 7 | 1 << 5 | 1 << 8 | 1 << 24 | 0 << 25)

#define BSP_CFG_OPTION_SETTING_OFS1_SEC  ((uint32_t) BSP_CFG_OPTION_SETTING_OFS1_SEC_NO_HOCOFRQ | ((uint32_t) BSP_CFG_HOCO_FREQUENCY << BSP_FEATURE_BSP_OFS1_HOCOFRQ_OFFSET))
#endif
#ifndef BSP_CFG_OPTION_SETTING_OFS1_SEL
#if defined(_RA_TZ_SECURE) || defined(_RA_TZ_NONSECURE)
  #define BSP_CFG_OPTION_SETTING_OFS1_SEL  (0 | ((0U << 0U)) | ((0U << 3U)) | ((0U << 5U)) | ((BSP_CFG_CLOCKS_SECURE == 0) ? 0xF00 : 0U) | ((0U << 24U)) | ((0U << 25U)))
#else
#define BSP_CFG_OPTION_SETTING_OFS1_SEL  (0)
#endif
#endif

#define BSP_CFG_CLOCKS_SECURE (0)
#define BSP_CFG_CLOCKS_OVERRIDE (0)
#define BSP_CFG_XTAL_HZ (20000000) /* XTAL 20000000Hz */
#define BSP_CFG_HOCO_FREQUENCY (2) /* HOCO 20MHz */
#define BSP_CFG_PLL_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_HOCO) /* PLL Src: HOCO */
#define BSP_CFG_PLL_DIV (BSP_CLOCKS_PLL_DIV_2) /* PLL Div /2 */
#define BSP_CFG_PLL_MUL BSP_CLOCKS_PLL_MUL(72,0) /* PLL Mul x60-79|Mul x72|PLL Mul x72.00 */
#define BSP_CFG_PLL_FREQUENCY_HZ (720000000) /* PLL 720000000Hz */
#define BSP_CFG_PLODIVP (BSP_CLOCKS_PLL_DIV_2) /* PLL1P Div /2 */
#define BSP_CFG_PLL1P_FREQUENCY_HZ (360000000) /* PLL1P 360000000Hz */
#define BSP_CFG_PLODIVQ (BSP_CLOCKS_PLL_DIV_2) /* PLL1Q Div /2 */
#define BSP_CFG_PLL1Q_FREQUENCY_HZ (360000000) /* PLL1Q 360000000Hz */
#define BSP_CFG_PLODIVR (BSP_CLOCKS_PLL_DIV_2) /* PLL1R Div /2 */
#define BSP_CFG_PLL1R_FREQUENCY_HZ (360000000) /* PLL1R 360000000Hz */
#define BSP_CFG_PLL2_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* PLL2 Disabled */
#define BSP_CFG_PLL2_DIV (BSP_CLOCKS_PLL_DIV_2) /* PLL2 Div /2 */
#define BSP_CFG_PLL2_MUL BSP_CLOCKS_PLL_MUL(96,0) /* PLL2 Mul x80-99|Mul x96|PLL2 Mul x96.00 */
#define BSP_CFG_PLL2_FREQUENCY_HZ (0) /* PLL2 0Hz */
#define BSP_CFG_PL2ODIVP (BSP_CLOCKS_PLL_DIV_2) /* PLL2P Div /2 */
#define BSP_CFG_PLL2P_FREQUENCY_HZ (0) /* PLL2P 0Hz */
#define BSP_CFG_PL2ODIVQ (BSP_CLOCKS_PLL_DIV_2) /* PLL2Q Div /2 */
#define BSP_CFG_PLL2Q_FREQUENCY_HZ (0) /* PLL2Q 0Hz */
#define BSP_CFG_PL2ODIVR (BSP_CLOCKS_PLL_DIV_2) /* PLL2R Div /2 */
#define BSP_CFG_PLL2R_FREQUENCY_HZ (0) /* PLL2R 0Hz */
#define BSP_CFG_CLOCK_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_PLL1P) /* Clock Src: PLL1P */
#define BSP_CFG_CLKOUT_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* CLKOUT Disabled */
#define BSP_CFG_SCICLK_SOURCE (BSP_CLOCKS_SOURCE_CLOCK_PLL1P) /* SCICLK Src: PLL1P */
#define BSP_CFG_SPICLK_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* SPICLK Disabled */
#define BSP_CFG_CANFDCLK_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* CANFDCLK Disabled */
#define BSP_CFG_UCLK_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* USBCLK Disabled */
#define BSP_CFG_OCTACLK_SOURCE (BSP_CLOCKS_CLOCK_DISABLED) /* OCTACLK Disabled */
#define BSP_CFG_CPUCLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_1) /* CPUCLK Div /1 */
#define BSP_CFG_ICLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_3) /* ICLK Div /3 */
#define BSP_CFG_PCLKA_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_3) /* PCLKA Div /3 */
#define BSP_CFG_PCLKB_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_6) /* PCLKB Div /6 */
#define BSP_CFG_PCLKC_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_6) /* PCLKC Div /6 */
#define BSP_CFG_PCLKD_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_3) /* PCLKD Div /3 */
#define BSP_CFG_PCLKE_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_3) /* PCLKE Div /3 */
#define BSP_CFG_BCLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_12) /* BCLK Div /12 */
#define BSP_CFG_FCLK_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_6) /* FCLK Div /6 */
#define BSP_CFG_CLKOUT_DIV (BSP_CLOCKS_SYS_CLOCK_DIV_1) /* CLKOUT Div /1 */
#define BSP_CFG_SCICLK_DIV (BSP_CLOCKS_SCI_CLOCK_DIV_4) /* SCICLK Div /4 */
#define BSP_CFG_SPICLK_DIV (BSP_CLOCKS_SPI_CLOCK_DIV_4) /* SPICLK Div /4 */
#define BSP_CFG_CANFDCLK_DIV (BSP_CLOCKS_CANFD_CLOCK_DIV_8) /* CANFDCLK Div /8 */
#define BSP_CFG_UCLK_DIV (BSP_CLOCKS_USB_CLOCK_DIV_5) /* USBCLK Div /5 */
#define BSP_CFG_OCTACLK_DIV (BSP_CLOCKS_OCTA_CLOCK_DIV_4) /* OCTACLK Div /4 */

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
 #if defined(__ARMCC_VERSION)          /* AC6 compiler */

/* The AC6 linker requires uninitialized code to be placed in a section that starts with ".bss." Without this, load
 * memory (ROM) is reserved unnecessarily. */
  #define BSP_UNINIT_SECTION_PREFIX         ".bss"
  #define BSP_DONT_REMOVE                   __attribute__((used))
  #define BSP_ATTRIBUTE_STACKLESS           __attribute__((naked))
  #define BSP_FORCE_INLINE                  __attribute__((always_inline))
 #elif   defined(__GNUC__)             /* GCC compiler */
  #define BSP_UNINIT_SECTION_PREFIX
  #define BSP_DONT_REMOVE                   __attribute__((used))
  #define BSP_ATTRIBUTE_STACKLESS           __attribute__((naked))
  #define BSP_FORCE_INLINE                  __attribute__((always_inline))
 #elif defined(__ICCARM__)             /* IAR compiler */
  #define BSP_UNINIT_SECTION_PREFIX
  #define BSP_DONT_REMOVE                   __root
  #define BSP_ATTRIBUTE_STACKLESS           __stackless
  #define BSP_FORCE_INLINE                  _Pragma("inline=forced")
 #endif

 #define BSP_SECTION_NOINIT                 BSP_UNINIT_SECTION_PREFIX ".ram_noinit"
 #define BSP_SECTION_FIXED_VECTORS          ".fixed_vectors"
 #define BSP_SECTION_APPLICATION_VECTORS    ".application_vectors"

/* Compiler neutral macros. */
 #define BSP_PLACE_IN_SECTION(x)    __attribute__((section(x))) __attribute__((__used__))

 #define BSP_ALIGN_VARIABLE(x)      __attribute__((aligned(x)))

 #define BSP_WEAK_REFERENCE            __attribute__((weak))

/** Stacks (and heap) must be sized and aligned to an integer multiple of this number. */
 #define BSP_STACK_ALIGNMENT           (8)

/* boot loaded applications cannot set ofs registers (only do so in the boot loader) */
#ifndef BSP_BOOTLOADED_APPLICATION
/** configuration register output to sections */
#if defined BSP_CFG_OPTION_SETTING_OFS0 && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_ofs0") g_bsp_cfg_option_setting_ofs0[] = {BSP_CFG_OPTION_SETTING_OFS0};
#endif
#if defined BSP_CFG_OPTION_SETTING_OFS2 && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_ofs2") g_bsp_cfg_option_setting_ofs2[] = {BSP_CFG_OPTION_SETTING_OFS2};
#endif
#if defined BSP_CFG_OPTION_SETTING_DUALSEL && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_dualsel") g_bsp_cfg_option_setting_dualsel[] = {BSP_CFG_OPTION_SETTING_DUALSEL};
#endif
#if defined BSP_CFG_OPTION_SETTING_OFS1
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_ofs1") g_bsp_cfg_option_setting_ofs1[] = {BSP_CFG_OPTION_SETTING_OFS1};
#endif
#if defined BSP_CFG_OPTION_SETTING_BANKSEL
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_banksel") g_bsp_cfg_option_setting_banksel[] = {BSP_CFG_OPTION_SETTING_BANKSEL};
#endif
#if defined BSP_CFG_OPTION_SETTING_BPS
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_bps") g_bsp_cfg_option_setting_bps[] = {BSP_CFG_OPTION_SETTING_BPS};
#endif
#if defined BSP_CFG_OPTION_SETTING_PBPS
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_pbps") g_bsp_cfg_option_setting_pbps[] = {BSP_CFG_OPTION_SETTING_PBPS};
#endif
#if defined BSP_CFG_OPTION_SETTING_OFS1_SEC && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_ofs1_sec") g_bsp_cfg_option_setting_ofs1_sec[] = {BSP_CFG_OPTION_SETTING_OFS1_SEC};
#endif
#if defined BSP_CFG_OPTION_SETTING_BANKSEL_SEC && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_banksel_sec") g_bsp_cfg_option_setting_banksel_sec[] = {BSP_CFG_OPTION_SETTING_BANKSEL_SEC};
#endif
#if defined BSP_CFG_OPTION_SETTING_BPS_SEC && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_bps_sec") g_bsp_cfg_option_setting_bps_sec[] = {BSP_CFG_OPTION_SETTING_BPS_SEC};
#endif
#if defined BSP_CFG_OPTION_SETTING_PBPS_SEC && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_pbps_sec") g_bsp_cfg_option_setting_pbps_sec[] = {BSP_CFG_OPTION_SETTING_PBPS_SEC};
#endif
#if defined BSP_CFG_OPTION_SETTING_OFS1_SEL && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_ofs1_sel") g_bsp_cfg_option_setting_ofs1_sel[] = {BSP_CFG_OPTION_SETTING_OFS1_SEL};
#endif
#if defined BSP_CFG_OPTION_SETTING_BANKSEL_SEL && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_banksel_sel") g_bsp_cfg_option_setting_banksel_sel[] = {BSP_CFG_OPTION_SETTING_BANKSEL_SEL};
#endif
#if defined BSP_CFG_OPTION_SETTING_BPS_SEL && !BSP_TZ_NONSECURE_BUILD
BSP_DONT_REMOVE static const uint32_t BSP_PLACE_IN_SECTION(".option_setting_bps_sel") g_bsp_cfg_option_setting_bps_sel[] = {BSP_CFG_OPTION_SETTING_BPS_SEL};
#endif
#endif // BSP_BOOTLOADED_APPLICATION

/******************************/
/* the init tables are located in bsp_linker_info.h */
#define BSP_LINKER_C
#include "bsp_linker_info.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Key code for writing PRCR register. */

#define BSP_PRV_PRCR_KEY            (0xA500U)
#define BSP_PRV_PRCR_PRC1_UNLOCK    ((BSP_PRV_PRCR_KEY) | 0x2U)
#define BSP_PRV_PRCR_UNLOCK         ((BSP_PRV_PRCR_KEY) | 0x3U)
#define BSP_PRV_PRCR_LOCK           ((BSP_PRV_PRCR_KEY) | 0x0U)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_clockconfig
 *
 * Description:
 *   Called to initialize the RA.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void ra_clockconfig(void)
{
  /* Unlock VBTCR1 register. */

  putreg16((BSP_PRV_PRCR_KEY | R_SYSTEM_PRCR_PRC0 | R_SYSTEM_PRCR_PRC1),
           R_SYSTEM_PRCR);

  /* The VBTCR1.BPWSWSTP must be set after reset on MCUs that have
   * VBTCR1.BPWSWSTP.
   * Reference section 11.2.1 "VBATT Control Register 1 (VBTCR1)" and Figure
   * 11.2
   * "Setting flow of the VBTCR1.BPWSWSTP bit" in the RA manual
   * R01UM0007EU0110. This must be done before bsp_clock_init because LOCOCR,
   * LOCOUTCR, SOSCCR, and SOMCR cannot be accessed until VBTSR.VBTRVLD is
   * set.
   * */

  modifyreg8(R_SYSTEM_VBTCR1, 0, R_SYSTEM_VBTCR1_BPWSWSTP);
  while ((getreg8(R_SYSTEM_VBTSR) & R_SYSTEM_VBTSR_VBTRVLD) == 0)
    {
    }

  /* Disable FCache. */

  modifyreg16(R_FCACHE_FCACHEE, R_FCACHE_FCACHEE_FCACHEEN, 0);

  modifyreg8(R_SYSTEM_SCKSCR, R_SYSTEM_SCKSCR_CKSEL_MASK, RA_CKSEL);

  /* lock VBTCR1 register. */

  putreg16(0, R_SYSTEM_PRCR);

#if (RA_ICLK_FREQUENCY > 32000000)
  modifyreg8(R_SYSTEM_MEMWAIT, 0, R_SYSTEM_MEMWAIT_MEMWAIT);
#endif

  modifyreg32(R_SYSTEM_SCKDIVCR,
              (R_SYSTEM_SCKDIVCR_FCK_MASK | R_SYSTEM_SCKDIVCR_ICK_MASK |
               R_SYSTEM_SCKDIVCR_PCKA_MASK | R_SYSTEM_SCKDIVCR_PCKB_MASK |
               R_SYSTEM_SCKDIVCR_PCKC_MASK | R_SYSTEM_SCKDIVCR_PCKD_MASK),
              (RA_FCK_DIV | RA_ICK_DIV | RA_PCKA_DIV | RA_PCKB_DIV |
               RA_PCKC_DIV |
               RA_PCKD_DIV));
}
