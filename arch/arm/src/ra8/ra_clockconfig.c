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
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/syslog/syslog.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "ra_clockconfig.h"
#include "hardware/ra_flash.h"
#include "hardware/ra_system.h"
#include "hardware/ra_option_setting.h"

const uint32_t option_settings[] __attribute__((section(".rom_registers")))
__attribute__((__used__)) =
{
  /* Option Function Select Register 0 */

  (
  R_OFS0_RESERVED_31 | R_OFS0_WDTSTPCTL | R_OFS0_RESERVED_29 |
  R_OFS0_WDTRSTIRQS | R_OFS0_WDTRPSS_MASK | R_OFS0_WDTRPES_MASK |
  R_OFS0_WDTCKS_MASK | R_OFS0_WDTTOPS_MASK | R_OFS0_WDTSTRT |
  R_OFS0_RESERVED_16_15_MASK | R_OFS0_IWDTSTPCTL | R_OFS0_RESERVED_13 |
  R_OFS0_IWDTRSTIRQS | R_OFS0_IWDTRPSS_MASK | R_OFS0_IWDTRPES_MASK |
  R_OFS0_IWDTCKS_MASK | R_OFS0_IWDTTOPS_MASK | R_OFS0_IWDTSTRT |
  R_OFS0_RESERVED_0
  ),

  /* Option Function Select Register 1 */

  (
  R_OFS1_RESERVED_16_15_MASK | RA_HOCO_FREQUENCY |
  R_OFS1_RESERVED_11_9_MASK | RA_HOCOEN | R_OFS1_RESERVED_7_6_MASK |
  R_OFS1_VDSEL1_MASK | R_OFS1_LVDAS | R_OFS1_RESERVED_1_0_MASK),

  (uint32_t)0x00fffffc,         /* Security MPU Program Counter Start Address
                                 * Register (SECMPUPCS0) */
  (uint32_t)0x00ffffff,         /* Security MPU Program Counter End Address
                                 * Register (SECMPUPCE0)  */
  (uint32_t)0x00fffffc,         /* Security MPU Program Counter Start Address
                                 * Register (SECMPUPCS1) */
  (uint32_t)0x00ffffff,         /* Security MPU Program Counter End Address
                                 * Register (SECMPUPCE1)  */
  (uint32_t)0x00fffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS0) */
  (uint32_t)0x00ffffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE0) */
  (uint32_t)0x200ffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS1) */
  (uint32_t)0x200fffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE1) */
  (uint32_t)0x407ffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS2) */
  (uint32_t)0x407fffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE2) */
  (uint32_t)0x40dffffc,         /* Security MPU Region 0 Start Address
                                 * Register (SECMPUS3) */
  (uint32_t)0x40dfffff,         /* Security MPU Region 0 END Address Register
                                 * (SECMPUE3) */
  (uint32_t)0xffffffff,         /* Security MPU Access Control Register
                                 * (SECMPUAC) */
};

/** ID code definitions defined here. */

static const uint32_t g_bsp_id_codes[] __attribute__((section(".id_code")))
__attribute__((__used__)) =
{
  IDCODE1, IDCODE2, IDCODE3, IDCODE4
};

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
  /* FSP-based clock configuration sequence */
  
  /* Step 1: Unlock system registers */
  putreg16((BSP_PRV_PRCR_KEY | R_SYSTEM_PRCR_PRC0 | R_SYSTEM_PRCR_PRC1),
           R_SYSTEM_PRCR);

  /* Step 2: Configure VBATT Control - FSP requirement */
  /* The VBTCR1.BPWSWSTP must be set after reset on MCUs that have
   * VBTCR1.BPWSWSTP.
   * Reference section 11.2.1 "VBATT Control Register 1 (VBTCR1)" and Figure
   * 11.2 "Setting flow of the VBTCR1.BPWSWSTP bit" in the RA manual
   * R01UM0007EU0110. This must be done before clock init because LOCOCR,
   * LOCOUTCR, SOSCCR, and SOMCR cannot be accessed until VBTSR.VBTRVLD is
   * set.
   */
  modifyreg8(R_SYSTEM_VBTCR1, 0, R_SYSTEM_VBTCR1_BPWSWSTP);
  while ((getreg8(R_SYSTEM_VBTSR) & R_SYSTEM_VBTSR_VBTRVLD) == 0)
    {
    }

#ifdef CONFIG_RA8_HOCO_ENABLE
  /* Step 3: Configure HOCO (High-Speed On-Chip Oscillator) */
  /* FSP-style HOCO configuration */
  modifyreg8(R_SYSTEM_HOCOCR, R_SYSTEM_HOCOCR_HCSTP, 0);
  while ((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_HOCOSF) == 0)
    {
    }
#endif

#ifdef CONFIG_RA8_MOCO_ENABLE
  /* Step 4: Configure MOCO (Medium-Speed On-Chip Oscillator) */
  modifyreg8(R_SYSTEM_MOCOCR, R_SYSTEM_MOCOCR_MCSTP, 0);
  while ((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_MOCOSF) == 0)
    {
    }
#endif

#ifdef CONFIG_RA8_PLL_ENABLE
  /* Step 5: Configure PLL - FSP-based PLL setup */
  /* Set PLL source clock and multiplication factor */
  putreg16(RA_PLL_DIV | (RA_PLL_MUL << 8), R_SYSTEM_PLLCCR);
  
  /* Enable PLL */
  modifyreg8(R_SYSTEM_PLLCR, R_SYSTEM_PLLCR_PLLSTP, 0);
  while ((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_PLLSF) == 0)
    {
    }
#endif

  /* Step 6: Disable Flash Cache during clock transition - FSP requirement */
  modifyreg16(R_FCACHE_FCACHEE, R_FCACHE_FCACHEE_FCACHEEN, 0);

  /* Step 7: Set system clock source - FSP-based clock selection */
  modifyreg8(R_SYSTEM_SCKSCR, R_SYSTEM_SCKSCR_CKSEL_MASK, RA_CKSEL);

  /* Step 8: Configure memory wait states for high-speed operation */
#if (RA_ICLK_FREQUENCY > 32000000)
  modifyreg8(R_SYSTEM_MEMWAIT, 0, R_SYSTEM_MEMWAIT_MEMWAIT);
#endif

  /* Step 9: Configure system clock dividers - FSP-based divider setup */
  modifyreg32(R_SYSTEM_SCKDIVCR,
              (R_SYSTEM_SCKDIVCR_FCK_MASK | R_SYSTEM_SCKDIVCR_ICK_MASK |
               R_SYSTEM_SCKDIVCR_PCKA_MASK | R_SYSTEM_SCKDIVCR_PCKB_MASK |
               R_SYSTEM_SCKDIVCR_PCKC_MASK | R_SYSTEM_SCKDIVCR_PCKD_MASK),
              (RA_FCK_DIV | RA_ICK_DIV | RA_PCKA_DIV | RA_PCKB_DIV |
               RA_PCKC_DIV | RA_PCKD_DIV));

#ifdef CONFIG_RA8_FLASH_CACHE_ENABLE
  /* Step 10: Re-enable Flash Cache after clock configuration */
  modifyreg16(R_FCACHE_FCACHEE, 0, R_FCACHE_FCACHEE_FCACHEEN);
#endif

  /* Step 11: Lock system registers */
  putreg16(BSP_PRV_PRCR_LOCK, R_SYSTEM_PRCR);
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ra_clock_config_t g_fsp_clock_config;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_update_clock_config
 *
 * Description:
 *   Update FSP clock configuration from hardware registers
 *
 ****************************************************************************/

static void ra_update_clock_config(void)
{
  uint32_t sckscr;
  uint32_t sckdivcr;
  uint8_t clock_source;

  /* Read current clock source */
  sckscr = getreg8(R_SYSTEM_SCKSCR);
  clock_source = sckscr & R_SYSTEM_SCKSCR_CKSEL_MASK;

  /* Read clock dividers */
  sckdivcr = getreg32(R_SYSTEM_SCKDIVCR);

  /* Update configuration structure */
  g_fsp_clock_config.clock_source = clock_source;
  
  /* Calculate frequencies based on clock source */
  switch (clock_source)
    {
      case 0: /* HOCO */
        g_fsp_clock_config.system_clock_freq = CONFIG_RA8_HOCO_FREQUENCY;
        g_fsp_clock_config.hoco_enabled = true;
        break;
        
      case 1: /* MOCO */
        g_fsp_clock_config.system_clock_freq = CONFIG_RA8_MOCO_FREQUENCY;
        g_fsp_clock_config.moco_enabled = true;
        break;
        
      case 5: /* PLL */
        g_fsp_clock_config.system_clock_freq = RA_SYSTEM_CLOCK_FREQUENCY;
        g_fsp_clock_config.pll_enabled = true;
        break;
        
      default:
        g_fsp_clock_config.system_clock_freq = CONFIG_RA8_HOCO_FREQUENCY;
        break;
    }

  /* Calculate derived frequencies */
  uint8_t ick_div = (sckdivcr & R_SYSTEM_SCKDIVCR_ICK_MASK) >> 24;
  uint8_t pckb_div = (sckdivcr & R_SYSTEM_SCKDIVCR_PCKB_MASK) >> 8;
  uint8_t pckd_div = (sckdivcr & R_SYSTEM_SCKDIVCR_PCKD_MASK) >> 0;

  g_fsp_clock_config.iclk_freq = g_fsp_clock_config.system_clock_freq >> ick_div;
  g_fsp_clock_config.pclkb_freq = g_fsp_clock_config.system_clock_freq >> pckb_div;
  g_fsp_clock_config.pclkd_freq = g_fsp_clock_config.system_clock_freq >> pckd_div;
  g_fsp_clock_config.hoco_frequency = RA_HOCO_FREQUENCY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_get_clock_config
 *
 * Description:
 *   Get current FSP-compatible clock configuration
 *
 ****************************************************************************/

void ra_get_clock_config(ra_clock_config_t *config)
{
  DEBUGASSERT(config != NULL);

  /* Update configuration from hardware */
  ra_update_clock_config();

  /* Copy to user structure */
  memcpy(config, &g_fsp_clock_config, sizeof(ra_clock_config_t));
}

/****************************************************************************
 * Name: ra_print_clock_info
 *
 * Description:
 *   Print FSP-compatible clock information for debugging
 *
 ****************************************************************************/

void ra_print_clock_info(void)
{
#ifdef CONFIG_DEBUG_INFO
  ra_clock_config_t config;
  
  ra_get_clock_config(&config);
  
  syslog(LOG_INFO, "FSP Clock Configuration:\n");
  syslog(LOG_INFO, "  System Clock: %lu Hz\n", config.system_clock_freq);
  syslog(LOG_INFO, "  ICLK: %lu Hz\n", config.iclk_freq);
  syslog(LOG_INFO, "  PCLKB: %lu Hz\n", config.pclkb_freq);
  syslog(LOG_INFO, "  PCLKD: %lu Hz\n", config.pclkd_freq);
  syslog(LOG_INFO, "  Clock Source: %d\n", config.clock_source);
  syslog(LOG_INFO, "  HOCO: %s\n", config.hoco_enabled ? "Enabled" : "Disabled");
  syslog(LOG_INFO, "  MOCO: %s\n", config.moco_enabled ? "Enabled" : "Disabled");
  syslog(LOG_INFO, "  PLL: %s\n", config.pll_enabled ? "Enabled" : "Disabled");
#endif
}

/****************************************************************************
 * Name: ra_get_peripheral_clock
 *
 * Description:
 *   Get peripheral clock frequency for FSP compatibility
 *
 ****************************************************************************/

uint32_t ra_get_peripheral_clock(int peripheral_id)
{
  ra_clock_config_t config;
  
  ra_get_clock_config(&config);
  
  switch (peripheral_id)
    {
      case 0: /* PCLKA - same as ICLK */
        return config.iclk_freq;
        
      case 1: /* PCLKB */
        return config.pclkb_freq;
        
      case 2: /* PCLKC - same as PCLKB */
        return config.pclkb_freq;
        
      case 3: /* PCLKD */
        return config.pclkd_freq;
        
      default:
        return config.system_clock_freq;
    }
}
