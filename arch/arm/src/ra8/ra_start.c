/****************************************************************************
 * arch/arm/src/ra8/ra_start.c
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
#include <assert.h>
#include <debug.h>
#include <string.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include "arch/board/board.h"
#include "arm_internal.h"
#include "nvic.h"
#include "ra_clock.h"
#include "ra_lowputc.h"
#include "ra_start.h"
#include "hardware/ra_flash.h"
#include "hardware/ra_system.h"
#include "hardware/ra_option_setting.h"


/* Function prototype for nx_start */
void nx_start(void);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use the standard NuttX approach for heap base - will be defined in config */
#ifndef CONFIG_IDLETHREAD_STACKSIZE
#  define CONFIG_IDLETHREAD_STACKSIZE 2048
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The top of the idle thread stack.  This is used to initialize the
 * initial stack pointer for the idle thread.  The idle thread stack is
 * allocated in the .bss section, so it will be zero initialized.
 * The idle thread stack is allocated in the .bss section, so it will be
 * zero initialized.
 */
extern uint32_t __ram_thread_stack$$Limit;
const uintptr_t g_idle_topstack = (uintptr_t)&__ram_thread_stack$$Limit + CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * ID Code Definitions
 * Following Renesas FSP pattern for device identification
 ****************************************************************************/

/** ID code definitions defined here. */
static const uint32_t g_ra_id_codes[] __attribute__((section(".id_code")))
__attribute__((__used__)) =
{
  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF
};

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/

/** ID code definitions already defined above */

/* Key constants for option bytes */
#define RA_TZ_STACK_SEAL_VALUE     (0xFEF5EDA5)
#define RA_CCR_CACHE_ENABLE        (0x000E0201) /* Enable instruction cache, branch prediction and LOB extension */

/* PRCR register unlock keys */
#define RA_PRCR_KEY                (0xA500U)
#define RA_PRCR_PRC1_UNLOCK        ((RA_PRCR_KEY) | 0x2U)
#define RA_PRCR_LOCK               ((RA_PRCR_KEY) | 0x0U)

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/* We use RA_ prefixed macros from ra_start.h to avoid duplicate definitions */
/* Any usage of macros in this file should be converted to RA_ equivalents */
#if defined (CONFIG_RA_LINKER_C) && CONFIG_RA_LINKER_C
/* boot loaded applications cannot set ofs registers (only do so in the boot loader) */
#if !defined(CONFIG_RA_BOOTLOADED_APPLICATION) || !CONFIG_RA_BOOTLOADED_APPLICATION

/** configuration register output to sections */
#if defined CONFIG_RA_OPTION_SETTING_OFS0 && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs0") g_ra_cfg_option_setting_ofs0[] = {CONFIG_RA_OPTION_SETTING_OFS0};
#endif
#if defined CONFIG_RA_OPTION_SETTING_OFS2 && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs2") g_ra_cfg_option_setting_ofs2[] = {CONFIG_RA_OPTION_SETTING_OFS2};
#endif
#if defined CONFIG_RA_OPTION_SETTING_DUALSEL && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_dualsel") g_ra_cfg_option_setting_dualsel[] = {CONFIG_RA_OPTION_SETTING_DUALSEL};
#endif
#if defined CONFIG_RA_OPTION_SETTING_OFS1
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs1") g_ra_cfg_option_setting_ofs1[] = {CONFIG_RA_OPTION_SETTING_OFS1};
#endif
#if defined CONFIG_RA_OPTION_SETTING_BANKSEL
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_banksel") g_ra_cfg_option_setting_banksel[] = {CONFIG_RA_OPTION_SETTING_BANKSEL};
#endif
#if defined CONFIG_RA_OPTION_SETTING_BPS
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_bps") g_ra_cfg_option_setting_bps[] = {CONFIG_RA_OPTION_SETTING_BPS};
#endif
#if defined CONFIG_RA_OPTION_SETTING_PBPS
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_pbps") g_ra_cfg_option_setting_pbps[] = {CONFIG_RA_OPTION_SETTING_PBPS};
#endif
#if defined CONFIG_RA_OPTION_SETTING_OFS1_SEC && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs1_sec") g_ra_cfg_option_setting_ofs1_sec[] = {CONFIG_RA_OPTION_SETTING_OFS1_SEC};
#endif
#if defined CONFIG_RA_OPTION_SETTING_BANKSEL_SEC && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_banksel_sec") g_ra_cfg_option_setting_banksel_sec[] = {CONFIG_RA_OPTION_SETTING_BANKSEL_SEC};
#endif
#if defined CONFIG_RA_OPTION_SETTING_BPS_SEC && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_bps_sec") g_ra_cfg_option_setting_bps_sec[] = {CONFIG_RA_OPTION_SETTING_BPS_SEC};
#endif
#if defined CONFIG_RA_OPTION_SETTING_PBPS_SEC && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_pbps_sec") g_ra_cfg_option_setting_pbps_sec[] = {CONFIG_RA_OPTION_SETTING_PBPS_SEC};
#endif
#if defined CONFIG_RA_OPTION_SETTING_OFS1_SEL && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs1_sel") g_ra_cfg_option_setting_ofs1_sel[] = {CONFIG_RA_OPTION_SETTING_OFS1_SEL};
#endif
#if defined CONFIG_RA_OPTION_SETTING_BANKSEL_SEL && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_banksel_sel") g_ra_cfg_option_setting_banksel_sel[] = {CONFIG_RA_OPTION_SETTING_BANKSEL_SEL};
#endif
#if defined CONFIG_RA_OPTION_SETTING_BPS_SEL && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_bps_sel") g_ra_cfg_option_setting_bps_sel[] = {CONFIG_RA_OPTION_SETTING_BPS_SEL};
#endif

#endif /* CONFIG_RA_BOOTLOADED_APPLICATION */

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
/* FSP linker generated initialization table data structures types */
/* These are only used for reference with FSP initialization approach */
/* NuttX has its own memory initialization in arm_head.S */
typedef enum e_ra_init_mem
{
    INIT_MEM_ZERO,
    INIT_MEM_FLASH,
    INIT_MEM_DATA_FLASH,
    INIT_MEM_RAM,
    INIT_MEM_DTCM,
    INIT_MEM_ITCM,
    INIT_MEM_CTCM,
    INIT_MEM_STCM,
    INIT_MEM_OSPI0_CS0,
    INIT_MEM_OSPI0_CS1,
    INIT_MEM_OSPI1_CS0,
    INIT_MEM_OSPI1_CS1,
    INIT_MEM_QSPI_FLASH,
    INIT_MEM_SDRAM,
} ra_init_mem_t;

typedef struct st_ra_init_type
{
    uint32_t copy_64 :8; /* if 1, must use 64 bit copy operation (to keep ecc happy) */
    uint32_t external :8; /* =1 if either source or destination is external, else 0  */
    uint32_t source_type :8;
    uint32_t destination_type :8;
} ra_init_type_t;

typedef struct st_ra_init_zero_info
{
    uint32_t *const p_base;
    uint32_t *const p_limit;
    ra_init_type_t type;
} ra_init_zero_info_t;

typedef struct st_ra_init_copy_info
{
    uint32_t *const p_base;
    uint32_t *const p_limit;
    uint32_t *const p_load;
    ra_init_type_t type;
} ra_init_copy_info_t;

typedef struct st_ra_init_nocache_info
{
    uint32_t *const p_base;
    uint32_t *const p_limit;
} ra_mpu_nocache_info_t;

typedef struct st_ra_init_info
{
    uint32_t zero_count;
    ra_init_zero_info_t const *const p_zero_list;
    uint32_t copy_count;
    ra_init_copy_info_t const *const p_copy_list;
    uint32_t nocache_count;
    ra_mpu_nocache_info_t const *const p_nocache_list;
} ra_init_info_t;

/***********************************************************************************************************************
 * Exported global variables
 **********************************************************************************************************************/

extern ra_init_info_t const g_init_info;
/* These symbols are used for sau/idau configuration in a secure project */

/***********************************************************************************************************************
 * Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Objects allocated by bsp_linker.c
 **********************************************************************************************************************/
/* DDSC symbol definitions */
/* Zero initialization tables */
extern uint32_t __ospi0_cs0_zero_nocache$$Base;
extern uint32_t __ospi0_cs0_zero_nocache$$Limit;
extern uint32_t __ospi0_cs0_zero$$Base;
extern uint32_t __ospi0_cs0_zero$$Limit;
extern uint32_t __itcm_zero$$Base;
extern uint32_t __itcm_zero$$Limit;
extern uint32_t __dtcm_zero$$Base;
extern uint32_t __dtcm_zero$$Limit;
extern uint32_t __ram_zero_nocache$$Base;
extern uint32_t __ram_zero_nocache$$Limit;
extern uint32_t __ram_zero$$Base;
extern uint32_t __ram_zero$$Limit;
extern uint32_t __ram_tbss$$Base;
extern uint32_t __ram_tbss$$Limit;
static const ra_init_zero_info_t zero_list[] =
{
  {.p_base = &__ospi0_cs0_zero_nocache$$Base, .p_limit = &__ospi0_cs0_zero_nocache$$Limit,.type={.copy_64 = 0, .external = 1, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_OSPI0_CS0}},
  {.p_base = &__ospi0_cs0_zero$$Base, .p_limit = &__ospi0_cs0_zero$$Limit,.type={.copy_64 = 0, .external = 1, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_OSPI0_CS0}},
  {.p_base = &__itcm_zero$$Base, .p_limit = &__itcm_zero$$Limit,.type={.copy_64 = 1, .external = 0, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_ITCM}},
  {.p_base = &__dtcm_zero$$Base, .p_limit = &__dtcm_zero$$Limit,.type={.copy_64 = 1, .external = 0, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_DTCM}},
  {.p_base = &__ram_zero_nocache$$Base, .p_limit = &__ram_zero_nocache$$Limit,.type={.copy_64 = 0, .external = 0, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_RAM}},
  {.p_base = &__ram_zero$$Base, .p_limit = &__ram_zero$$Limit,.type={.copy_64 = 0, .external = 0, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_RAM}},
  {.p_base = &__ram_tbss$$Base, .p_limit = &__ram_tbss$$Limit,.type={.copy_64 = 0, .external = 0, .source_type = INIT_MEM_ZERO, .destination_type = INIT_MEM_RAM}}
};
/* Load initialization tables */
extern uint32_t __ospi0_cs0_from_ospi0_cs1$$Base;
extern uint32_t __ospi0_cs0_from_ospi0_cs1$$Limit;
extern uint32_t __ospi0_cs0_from_ospi0_cs1$$Load;
extern uint32_t __ospi0_cs0_from_data_flash$$Base;
extern uint32_t __ospi0_cs0_from_data_flash$$Limit;
extern uint32_t __ospi0_cs0_from_data_flash$$Load;
extern uint32_t __ospi0_cs0_from_flash$$Base;
extern uint32_t __ospi0_cs0_from_flash$$Limit;
extern uint32_t __ospi0_cs0_from_flash$$Load;
extern uint32_t __itcm_from_ospi0_cs1$$Base;
extern uint32_t __itcm_from_ospi0_cs1$$Limit;
extern uint32_t __itcm_from_ospi0_cs1$$Load;
extern uint32_t __itcm_from_data_flash$$Base;
extern uint32_t __itcm_from_data_flash$$Limit;
extern uint32_t __itcm_from_data_flash$$Load;
extern uint32_t __itcm_from_flash$$Base;
extern uint32_t __itcm_from_flash$$Limit;
extern uint32_t __itcm_from_flash$$Load;
extern uint32_t __dtcm_from_ospi0_cs1$$Base;
extern uint32_t __dtcm_from_ospi0_cs1$$Limit;
extern uint32_t __dtcm_from_ospi0_cs1$$Load;
extern uint32_t __dtcm_from_data_flash$$Base;
extern uint32_t __dtcm_from_data_flash$$Limit;
extern uint32_t __dtcm_from_data_flash$$Load;
extern uint32_t __dtcm_from_flash$$Base;
extern uint32_t __dtcm_from_flash$$Limit;
extern uint32_t __dtcm_from_flash$$Load;
extern uint32_t __ram_from_ospi0_cs1$$Base;
extern uint32_t __ram_from_ospi0_cs1$$Limit;
extern uint32_t __ram_from_ospi0_cs1$$Load;
extern uint32_t __ram_from_data_flash$$Base;
extern uint32_t __ram_from_data_flash$$Limit;
extern uint32_t __ram_from_data_flash$$Load;
extern uint32_t __ram_from_flash$$Base;
extern uint32_t __ram_from_flash$$Limit;
extern uint32_t __ram_from_flash$$Load;
extern uint32_t __ram_tdata$$Base;
extern uint32_t __ram_tdata$$Limit;
extern uint32_t __ram_tdata$$Load;
static const ra_init_copy_info_t copy_list[] =
{
  {.p_base = &__ospi0_cs0_from_ospi0_cs1$$Base, .p_limit = &__ospi0_cs0_from_ospi0_cs1$$Limit, .p_load = &__ospi0_cs0_from_ospi0_cs1$$Load,.type={.copy_64 = 0, .external = 1, .source_type = INIT_MEM_OSPI0_CS1, .destination_type = INIT_MEM_OSPI0_CS0}},
  {.p_base = &__ospi0_cs0_from_data_flash$$Base, .p_limit = &__ospi0_cs0_from_data_flash$$Limit, .p_load = &__ospi0_cs0_from_data_flash$$Load,.type={.copy_64 = 0, .external = 1, .source_type = INIT_MEM_DATA_FLASH, .destination_type = INIT_MEM_OSPI0_CS0}},
  {.p_base = &__ospi0_cs0_from_flash$$Base, .p_limit = &__ospi0_cs0_from_flash$$Limit, .p_load = &__ospi0_cs0_from_flash$$Load,.type={.copy_64 = 0, .external = 1, .source_type = INIT_MEM_FLASH, .destination_type = INIT_MEM_OSPI0_CS0}},
  {.p_base = &__itcm_from_ospi0_cs1$$Base, .p_limit = &__itcm_from_ospi0_cs1$$Limit, .p_load = &__itcm_from_ospi0_cs1$$Load,.type={.copy_64 = 1, .external = 1, .source_type = INIT_MEM_OSPI0_CS1, .destination_type = INIT_MEM_ITCM}},
  {.p_base = &__itcm_from_data_flash$$Base, .p_limit = &__itcm_from_data_flash$$Limit, .p_load = &__itcm_from_data_flash$$Load,.type={.copy_64 = 1, .external = 0, .source_type = INIT_MEM_DATA_FLASH, .destination_type = INIT_MEM_ITCM}},
  {.p_base = &__itcm_from_flash$$Base, .p_limit = &__itcm_from_flash$$Limit, .p_load = &__itcm_from_flash$$Load,.type={.copy_64 = 1, .external = 0, .source_type = INIT_MEM_FLASH, .destination_type = INIT_MEM_ITCM}},
  {.p_base = &__dtcm_from_ospi0_cs1$$Base, .p_limit = &__dtcm_from_ospi0_cs1$$Limit, .p_load = &__dtcm_from_ospi0_cs1$$Load,.type={.copy_64 = 1, .external = 1, .source_type = INIT_MEM_OSPI0_CS1, .destination_type = INIT_MEM_DTCM}},
  {.p_base = &__dtcm_from_data_flash$$Base, .p_limit = &__dtcm_from_data_flash$$Limit, .p_load = &__dtcm_from_data_flash$$Load,.type={.copy_64 = 1, .external = 0, .source_type = INIT_MEM_DATA_FLASH, .destination_type = INIT_MEM_DTCM}},
  {.p_base = &__dtcm_from_flash$$Base, .p_limit = &__dtcm_from_flash$$Limit, .p_load = &__dtcm_from_flash$$Load,.type={.copy_64 = 1, .external = 0, .source_type = INIT_MEM_FLASH, .destination_type = INIT_MEM_DTCM}},
  {.p_base = &__ram_from_ospi0_cs1$$Base, .p_limit = &__ram_from_ospi0_cs1$$Limit, .p_load = &__ram_from_ospi0_cs1$$Load,.type={.copy_64 = 0, .external = 1, .source_type = INIT_MEM_OSPI0_CS1, .destination_type = INIT_MEM_RAM}},
  {.p_base = &__ram_from_data_flash$$Base, .p_limit = &__ram_from_data_flash$$Limit, .p_load = &__ram_from_data_flash$$Load,.type={.copy_64 = 0, .external = 0, .source_type = INIT_MEM_DATA_FLASH, .destination_type = INIT_MEM_RAM}},
  {.p_base = &__ram_from_flash$$Base, .p_limit = &__ram_from_flash$$Limit, .p_load = &__ram_from_flash$$Load,.type={.copy_64 = 0, .external = 0, .source_type = INIT_MEM_FLASH, .destination_type = INIT_MEM_RAM}},
  {.p_base = &__ram_tdata$$Base, .p_limit = &__ram_tdata$$Limit, .p_load = &__ram_tdata$$Load,.type={.copy_64 = 0, .external = 0, .source_type = INIT_MEM_FLASH, .destination_type = INIT_MEM_RAM}}
};
/* nocache regions */
extern uint32_t __ospi0_cs0_noinit_nocache$$Base;
extern uint32_t __ospi0_cs0_noinit_nocache$$Limit;
extern uint32_t __ospi0_cs0_zero_nocache$$Base;
extern uint32_t __ospi0_cs0_zero_nocache$$Limit;
extern uint32_t __ram_noinit_nocache$$Base;
extern uint32_t __ram_noinit_nocache$$Limit;
extern uint32_t __ram_zero_nocache$$Base;
extern uint32_t __ram_zero_nocache$$Limit;
static const ra_mpu_nocache_info_t nocache_list[] =
{
  {.p_base = &__ospi0_cs0_noinit_nocache$$Base, .p_limit = &__ospi0_cs0_zero_nocache$$Limit},
  {.p_base = &__ram_noinit_nocache$$Base, .p_limit = &__ram_zero_nocache$$Limit},
};

/* initialization data structure */
const ra_init_info_t g_init_info =
{
    .zero_count  = sizeof(zero_list) / sizeof(zero_list[0]),
    .p_zero_list = zero_list,
    .copy_count  = sizeof(copy_list) / sizeof(copy_list[0]),
    .p_copy_list = copy_list,
    .nocache_count  = sizeof(nocache_list) / sizeof(nocache_list[0]),
    .p_nocache_list = nocache_list
};

#endif /* CONFIG_RA_LINKER_C */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the CONSOLE USART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c)  arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_cortex_m85_init
 *
 * Description:
 *   Initialize Cortex-M85 specific features following Renesas SystemInit
 *
 ****************************************************************************/
static void ra_cortex_m85_init(void)
{
#ifdef CONFIG_ARCH_CORTEXM85
  /* Following Renesas SystemInit for Cortex-M85:
   * Enable instruction cache, branch prediction, and LOB extension
   * This will be handled by NuttX ARM-specific initialization
   * See sections 6.5, 6.6, and 6.7 in Arm Cortex-M85 Technical Reference Manual
   */

  /* D-Cache configuration and errata handling will be done by NuttX */

  /* FPU configuration will be done by arm_fpuconfig() */

  /* Enable flash cache and wait for it to be ready */
  //putreg16(1U, R_FCACHE_FCACHEIV);
  //RA_HARDWARE_REGISTER_WAIT(getreg16(R_FCACHE_FCACHEIV), 1U);
  //putreg16(1U, R_FCACHE_FCACHEE);
#endif
}

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void __start(void)
{
    /* Enable the instruction cache, branch prediction, and the branch cache (required for Low Overhead Branch (LOB) extension).
     * See sections 6.5, 6.6, and 6.7 in the Arm Cortex-M85 Processor Technical Reference Manual (Document ID: 101924_0002_05_en, Issue: 05)
     * See section D1.2.9 in the Armv8-M Architecture Reference Manual (Document number: DDI0553B.w, Document version: ID07072023) */

  /* 1. Cortex-M85 Initialization */
  ra_cortex_m85_init();

  /* 2. TrustZone Configuration - early security setup */
#if CONFIG_RA_TZ_SECURE_BUILD || CONFIG_RA_TZ_NONSECURE_BUILD
  ra_trustzone_init();
#endif

  /* Phase 2: Core and Clock Initialization */
  /* 3. Setup System Clocks (includes Cortex-M85 core features) */
  ra_clock();

  /* 4. Set Vector Table Base Address */
  ra_vector_table_init();

  /* Phase 3: Memory Initialization */
  /* 5. Initialize RAM Sections (BSS, data, TCM) */
  ra_ram_init(0);

  /* Phase 4: Low-level Hardware Setup */
  /* 6. Configure the uart so that we can get debug output as soon as possible */
  ra_lowsetup();
  showprogress('A');

  /* 7. Perform early serial initialization */
#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('B');

  /* Phase 5: Board-level Initialization */
  /* 8. Initialize onboard resources */
  ra_board_initialize();
  showprogress('C');

  /* Phase 6: Start NuttX */
  /* 9. Then start NuttX main initialization */
  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */
  for (; ; )
    {
    }
}

/* Additional initialization functions for RA8E1 startup */

/****************************************************************************
 * Name: ra_option_bytes_init
 *
 * Description:
 *   Initialize option bytes (security and boot configuration)
 *   Following Renesas FSP bsp_linker.c approach
 *
 ****************************************************************************/

void ra_option_bytes_init(void)
{
  /* Configure option bytes for security, boot, etc. */
  /* Reference Renesas FSP logic for OFS registers */
  /* Option bytes are typically handled by linker sections and bootloader */
  /* On RA8E1, these are defined in ra_start.h and placed by linker */

  /* Option bytes include:
   * - RA_OPTION_SETTING_OFS0: IWDT and WDT settings
   * - RA_OPTION_SETTING_OFS2: Boot mode selection
   * - RA_OPTION_SETTING_OFS1_SEC: Security settings for TrustZone
   * - RA_OPTION_SETTING_OFS1_SEL: Secure/Non-secure selection
   */

  /* These are handled automatically by the linker script and bootloader */
  /* No runtime configuration needed here */
}

/****************************************************************************
 * Name: ra_trustzone_init
 *
 * Description:
 *   Initialize ARM TrustZone features following Renesas SystemInit
 *
 ****************************************************************************/

void ra_trustzone_init(void)
{
#if defined(CONFIG_RA_TZ_SECURE_BUILD)
  /* Enable TrustZone Secure settings following Renesas SystemInit */
  /* Seal the main stack for secure projects */
  /* Reference: https://developer.arm.com/documentation/100720/0300 */
  /* uint32_t * p_main_stack_top = (uint32_t *) &g_main_stack[CONFIG_RA_SECURE_STACK_BYTES]; */
  /* *p_main_stack_top = RA_TZ_STACK_SEAL_VALUE; */

  /* Configure SAU, IDAU, and secure memory regions */
  /* RA_SecurityInit(); */

#elif defined(CONFIG_RA_TZ_NONSECURE_BUILD)
  /* Configure non-secure memory regions and permissions */
  /* Non-secure VTOR is set by secure project, skip here */
#endif
}

/****************************************************************************
 * Name: ra_vector_table_init
 *
 * Description:
 *   Initialize vector table following Renesas SystemInit
 *
 ****************************************************************************/

void ra_vector_table_init(void)
{
  /* Set VTOR to point to the vector table base address */
  /* Following Renesas SystemInit: SCB->VTOR = (uint32_t) &__VECTOR_TABLE; */
#if !CONFIG_RA_TZ_NONSECURE_BUILD
  /* VTOR is in undefined state out of RESET, set it explicitly */
  /* Use NuttX standard method to set vector table */
  /* Note: This will be handled by the ARM core initialization later */
  /* For now, just ensure the vector table is properly set in linker script */
#endif
}

/****************************************************************************
 * Name: ra_tcm_init
 *
 * Description:
 *   Initialize TCM memories following Renesas SystemInit
 *
 ****************************************************************************/

void ra_tcm_init(void)
{
#if defined(CONFIG_ARMV8M_HAVE_ITCM) || defined(CONFIG_ARMV8M_HAVE_DTCM)
  /* Following Renesas SystemInit:
   * Zero initialize TCM memory if ECC is enabled and this is the first project
   * This prevents ECC errors on first access
   */

  /* TCM initialization will be handled by NuttX memory management */
  /* The linker script should properly configure TCM regions */
#endif
}

/****************************************************************************
 * Name: ra_ram_init
 *
 * Description:
 *   Initialize RAM sections following standard NuttX ARM startup
 *
 ****************************************************************************/
void ra_ram_init (const uint32_t external)
{
#if defined (CONFIG_RA_LINKER_C) && CONFIG_RA_LINKER_C
    /* Initialize C runtime environment. */
    for (uint32_t i = 0; i < g_init_info.zero_count; i++)
    {
        if (external == g_init_info.p_zero_list[i].type.external)
        {
            memset(g_init_info.p_zero_list[i].p_base, 0U,
                   ((uintptr_t) g_init_info.p_zero_list[i].p_limit - (uintptr_t) g_init_info.p_zero_list[i].p_base));
        }
    }

    for (uint32_t i = 0; i < g_init_info.copy_count; i++)
    {
        if (external == g_init_info.p_copy_list[i].type.external)
        {
            memcpy(g_init_info.p_copy_list[i].p_base, g_init_info.p_copy_list[i].p_load,
                   ((uintptr_t) g_init_info.p_copy_list[i].p_limit - (uintptr_t) g_init_info.p_copy_list[i].p_base));
        }
    }
#else
  const register uint32_t *src;
  register uint32_t *dest;
  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in OCRAM.  The correct place in OCRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

#endif
}
