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
#include <nuttx/cache.h>
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

 /* Time conversion macros */
#define RA_PRV_NS_PER_SECOND    (1000000000)
#define RA_PRV_US_PER_SECOND    (1000000)
#define RA_PRV_NS_PER_US        (1000)
#define RA_PRV_LOOP_CYCLES      (4)
#define RA_PRV_LOOPS_CALCULATE  (cycles)    (((cycles) / RA_PRV_LOOP_CYCLES) + 1U)

/* Key constants for option bytes */
#define RA_TZ_STACK_SEAL_VALUE     (0xFEF5EDA5)
#define RA_CCR_CACHE_ENABLE        (0x000E0201) /* Enable instruction cache, branch prediction and LOB extension */

/* PRCR register unlock keys */
#define RA_PRCR_KEY                (0xA500U)
#define RA_PRCR_PRC1_UNLOCK        ((RA_PRCR_KEY) | 0x2U)
#define RA_PRCR_LOCK               ((RA_PRCR_KEY) | 0x0U)

/* We use RA_ prefixed macros from ra_start.h to avoid duplicate definitions */
/* Any usage of macros in this file should be converted to RA_ equivalents */
#if defined (CONFIG_RA_OPTION_SETTING_ENABLE) && CONFIG_RA_OPTION_SETTING_ENABLE
/* boot loaded applications cannot set ofs registers (only do so in the boot loader) */
#if !defined(CONFIG_RA_BOOTLOADED_APPLICATION) || !CONFIG_RA_BOOTLOADED_APPLICATION

/** configuration register output to sections */
#if defined CONFIG_RA_OFS0_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs0") g_ra_cfg_option_setting_ofs0[] = {CONFIG_RA_OPTION_SETTING_OFS0};
#endif
#if defined CONFIG_RA_OFS2_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs2") g_ra_cfg_option_setting_ofs2[] = {CONFIG_RA_OPTION_SETTING_OFS2};
#endif
#if defined CONFIG_RA_DUAL_BANK_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_dualsel") g_ra_cfg_option_setting_dualsel[] = {CONFIG_RA_OPTION_SETTING_DUALSEL};
#endif
#if defined CONFIG_RA_OFS1_SETTING
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs1") g_ra_cfg_option_setting_ofs1[] = {CONFIG_RA_OPTION_SETTING_OFS1};
#endif
#if defined CONFIG_RA_BANK_SELECT_SETTING
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_banksel") g_ra_cfg_option_setting_banksel[] = {CONFIG_RA_OPTION_SETTING_BANKSEL};
#endif
#if defined CONFIG_RA_BOOT_PROTECT_SETTING
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_bps") g_ra_cfg_option_setting_bps[] = {CONFIG_RA_OPTION_SETTING_BPS};
#endif
#if defined CONFIG_RA_BOOT_PROTECT_SETTING
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_pbps") g_ra_cfg_option_setting_pbps[] = {CONFIG_RA_OPTION_SETTING_PBPS};
#endif
#if defined CONFIG_RA_OFS1_SEC_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs1_sec") g_ra_cfg_option_setting_ofs1_sec[] = {CONFIG_RA_OPTION_SETTING_OFS1_SEC};
#endif
#if defined CONFIG_RA_BANK_SELECT_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_banksel_sec") g_ra_cfg_option_setting_banksel_sec[] = {CONFIG_RA_OPTION_SETTING_BANKSEL_SEC};
#endif
#if defined CONFIG_RA_BANK_SELECT_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_bps_sec") g_ra_cfg_option_setting_bps_sec[] = {CONFIG_RA_OPTION_SETTING_BPS_SEC};
#endif
#if defined CONFIG_RA_BOOT_PROTECT_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_pbps_sec") g_ra_cfg_option_setting_pbps_sec[] = {CONFIG_RA_OPTION_SETTING_PBPS_SEC};
#endif
#if defined CONFIG_RA_OFS1_SEL_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_ofs1_sel") g_ra_cfg_option_setting_ofs1_sel[] = {CONFIG_RA_OPTION_SETTING_OFS1_SEL};
#endif
#if defined CONFIG_RA_BANK_SELECT_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
RA_DONT_REMOVE static const uint32_t RA_PLACE_IN_SECTION(".option_setting_banksel_sel") g_ra_cfg_option_setting_banksel_sel[] = {CONFIG_RA_OPTION_SETTING_BANKSEL_SEL};
#endif
#if defined CONFIG_RA_BOOT_PROTECT_SETTING && !CONFIG_RA_TZ_NONSECURE_BUILD
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

/* Register protection counters (FSP-compatible) */
static volatile uint16_t g_register_protect_counters[4] = {0};

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
#ifdef CONFIG_ARMV8M_ICACHE
  up_enable_icache();
#endif
#ifdef CONFIG_ARMV8M_DCACHE
  up_enable_dcache();
#endif
#ifdef CONFIG_ARCH_FPU
  arm_fpuconfig();
#endif
#ifdef CONFIG_ARCH_RAMVECTORS
  /* Initialize RAM vectors and set VTOR */
  arm_ramvec_initialize();
#else
  /* Set VTOR to point to the vector table using NuttX symbol */
#if defined(__ICCARM__)
  putreg32((uint32_t)__vector_table, NVIC_VECTAB);
#else
  putreg32((uint32_t)_vectors, NVIC_VECTAB);
#endif
#endif
  /* Enable flash cache and wait for it to be ready */
  putreg16(1U, R_FCACHE_FCACHEIV);
  RA_HARDWARE_REGISTER_WAIT(getreg16(R_FCACHE_FCACHEIV), 0U);
  putreg16(1U, R_FCACHE_FCACHEE);
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

  /* Initialize GPIO security attribution */
  ra_gpio_security_init();

  /* Phase 4: Low-level Hardware Setup */
  /* 6. Configure the uart so that we can get debug output as soon as possible */
  ra_lowsetup();

  /* 7. Perform early serial initialization */
#ifdef USE_EARLYSERIALINIT
  /* The 'A' character is not displayed because the UART hardware is not fully ready */
  showprogress('A');
  arm_earlyserialinit();
#else
  /* The 'A' character should be displayed if the ra_lowsetup() is responsible for minimal UART setup */
  showprogress('A');
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
    //const register uint32_t *src;
    //register uint32_t *dest;
   /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
    * certain that there are no issues with the state of global variables.
    */

    // for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    // {
    //   *dest++ = 0;
    // }

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in OCRAM.  The correct place in OCRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

    // for (src = (const uint32_t *)_eronly,
    //    dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
    //   )
    // {
    //   *dest++ = *src++;
    // }

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
}


/****************************************************************************
 * Name: ra_delay_us
 *
 * Description:
 *   Delay for a specified number of microseconds.
 *
 ****************************************************************************/

void ra_delay_us(uint32_t delay_us)
{
    uint32_t iclk_hz;
    uint32_t loops_required = 0;
    uint32_t total_us       = delay_us; /** Convert the requested time to microseconds. */

    iclk_hz = g_sys_core_clock;                 /** Get the system clock frequency in Hz. */


    if (iclk_hz >= 8000000)
    {
        /* For larger system clock values the below calculation in the else causes inaccurate delays due to rounding errors:
         *
         * ns_per_cycle = RA_PRV_NS_PER_SECOND / iclk_hz
         *
         * For system clock values greater than the MOCO speed the following delay calculation is used instead.
         * The value is always rounded up to ensure the delay is at least the supplied value.
         */
        uint32_t cycles_per_us = (iclk_hz + (RA_PRV_US_PER_SECOND * RA_PRV_LOOP_CYCLES) - 1) /
                                 (RA_PRV_US_PER_SECOND * RA_PRV_LOOP_CYCLES);

        uint64_t loops_required_u64 = ((uint64_t) total_us) * cycles_per_us;

        if (loops_required_u64 > UINT32_MAX)
        {
            loops_required = UINT32_MAX;
        }
        else
        {
            loops_required = (uint32_t) loops_required_u64;
        }
    }
    else
    {
        uint32_t cycles_requested;
        uint32_t ns_per_cycle;
        uint64_t ns_64bits;

        /* Running on the Sub-clock (32768 Hz) there are 30517 ns/cycle. This means one cycle takes 31 us. One execution
         * loop of the delay_loop takes 6 cycles which at 32768 Hz is 180 us. That does not include the overhead below prior to even getting
         * to the delay loop. Given this, at this frequency anything less then a delay request of 122 us will not even generate a single
         * pass through the delay loop.  For this reason small delays (<=~200 us) at this slow clock rate will not be possible and such a request
         * will generate a minimum delay of ~200 us.*/
        ns_per_cycle = RA_PRV_NS_PER_SECOND / iclk_hz;                 /** Get the # of nanoseconds/cycle. */

        /* We want to get the time in total nanoseconds but need to be conscious of overflowing 32 bits. We also do not want to do 64 bit */
        /* division as that pulls in a division library. */
        ns_64bits = (uint64_t) total_us * (uint64_t) RA_PRV_NS_PER_US; // Convert to ns.

        /* Have we overflowed 32 bits? */
        if (ns_64bits <= UINT32_MAX)
        {
            /* No, we will not overflow. */
            cycles_requested = ((uint32_t) ns_64bits / ns_per_cycle);
            loops_required   = cycles_requested / RA_PRV_LOOP_CYCLES;
        }
        else
        {
            /* We did overflow. Try dividing down first. */
            total_us  = (total_us / (ns_per_cycle * RA_PRV_LOOP_CYCLES));
            ns_64bits = (uint64_t) total_us * (uint64_t) RA_PRV_NS_PER_US; // Convert to ns.

            /* Have we overflowed 32 bits? */
            if (ns_64bits <= UINT32_MAX)
            {
                /* No, we will not overflow. */
                loops_required = (uint32_t) ns_64bits;
            }
            else
            {
                /* We still overflowed, use the max count for cycles */
                loops_required = UINT32_MAX;
            }
        }
    }

    /** Only delay if the supplied parameters constitute a delay. */
    if (loops_required > (uint32_t) 0)
    {
            __asm volatile (
        #if defined(RENESAS_CORTEX_M85) && (defined(__ARMCC_VERSION) || defined(__GNUC__))

                /* Align the branch target to a 64-bit boundary, a CM85 specific optimization. */
                /* IAR does not support alignment control within inline assembly. */
                ".balign 8\n"
        #endif
                "sw_delay_loop:         \n"
        #if defined(__ICCARM__) || defined(__ARMCC_VERSION) || (defined(__llvm__) && !defined(__CLANG_TIDY__))
                "   subs r0, #1         \n"    ///< 1 cycle
        #elif defined(__GNUC__)
                "   sub r0, r0, #1      \n"    ///< 1 cycle
        #endif

                "   cmp r0, #0          \n"    ///< 1 cycle

        /* CM0 and CM23 have a different instruction set */
        #if defined(__CORE_CM0PLUS_H_GENERIC) || defined(__CORE_CM23_H_GENERIC)
                "   bne sw_delay_loop   \n"    ///< 2 cycles
        #else
                "   bne.n sw_delay_loop \n"    ///< 2 cycles
        #endif
                "   bx lr               \n");  ///< 2 cycles
    }
}


/****************************************************************************
 * Name: ra_register_protect_enable
 *
 * Description:
 *   Enable register protection (FSP-compatible implementation)
 *
 * Input Parameters:
 *   regs_to_protect - Registers which have write protection enabled
 *
 ****************************************************************************/

void ra_register_protect_enable(ra_reg_protect_t regs_to_protect)
{
  irqstate_t flags;
  uint16_t prcr_masks[] = {0x0001, 0x0002, 0x0008, 0x0010};

  flags = enter_critical_section();

  /* Is it safe to disable write access? */
  if (g_register_protect_counters[regs_to_protect] != 0)
    {
      g_register_protect_counters[regs_to_protect]--;
    }

  /* If counter reaches zero, enable protection */
  if (g_register_protect_counters[regs_to_protect] == 0)
    {
      uint16_t prcr_value = getreg16(R_SYSTEM_PRCR);
      prcr_value = (prcr_value | R_SYSTEM_PRCR_PRKEY) &
                   (~prcr_masks[regs_to_protect]);
      putreg16(prcr_value, R_SYSTEM_PRCR);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra_register_protect_disable
 *
 * Description:
 *   Disable register protection (FSP-compatible implementation)
 *
 * Input Parameters:
 *   regs_to_unprotect - Registers which have write protection disabled
 *
 ****************************************************************************/

void ra_register_protect_disable(ra_reg_protect_t regs_to_unprotect)
{
  irqstate_t flags;
  uint16_t prcr_masks[] = {0x0001, 0x0002, 0x0008, 0x0010};

  flags = enter_critical_section();

  /* If this is first entry then disable protection */
  if (g_register_protect_counters[regs_to_unprotect] == 0)
    {
      uint16_t prcr_value = getreg16(R_SYSTEM_PRCR);
      prcr_value = (prcr_value | R_SYSTEM_PRCR_PRKEY) |
                   prcr_masks[regs_to_unprotect];
      putreg16(prcr_value, R_SYSTEM_PRCR);
    }

  /* Increment the protect counter */
  g_register_protect_counters[regs_to_unprotect]++;

  leave_critical_section(flags);
}


/****************************************************************************
 * Name: ra_gpio_security_init
 *
 * Description:
 *   Initialize PMSAR and PSCU registers to their default values.
 *   Sets all port pins to secure mode (0) as per FSP implementation.
 *   Must be called before configuring any port pins.
 *
 ****************************************************************************/

void ra_gpio_security_init(void)
{
  uint32_t i;

  /* Disable register protection for SAR (Security Attribution Registers) */
  ra_register_protect_disable(RA_REG_PROTECT_SAR);

  /* Set all PMSAR registers to 0 (secure mode for all pins) */
  for (i = 0; i < R_PMSAR_NUM; i++)
    {
      putreg16(0U, R_PMSAR(i));
    }

  /* Set all PSCU registers to 0 (secure mode) */
  putreg32(0U, R_PSCU_PSARB);
  putreg32(0U, R_PSCU_PSARC);
  putreg32(0U, R_PSCU_PSARD);
  putreg32(0U, R_PSCU_PSARE);

  /* Re-enable register protection for SAR */
  ra_register_protect_enable(RA_REG_PROTECT_SAR);
}
