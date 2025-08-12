/****************************************************************************
 * arch/arm/src/ra8/ra_clock.c
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
#include "ra_clock.h"
#include "hardware/ra_flash.h"
#include "hardware/ra_system.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Key code for writing PRCR register. */

#define RA_PRCR_KEY            (0xA500U)
#define RA_PRCR_PRC1_UNLOCK    ((RA_PRCR_KEY) | 0x2U)
#define RA_PRCR_UNLOCK         ((RA_PRCR_KEY) | 0x3U)
#define RA_PRCR_LOCK           ((RA_PRCR_KEY) | 0x0U)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ra_clock_config_t g_fsp_clock_config;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_update_clock_config(void);

/****************************************************************************
 * Public Functions
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

void ra_clock(void)
{
  uint32_t sckdivcr_value;

  /* FSP-based clock configuration sequence */

  /* Step 1: Unlock system registers */
  putreg16((RA_PRCR_KEY | R_SYSTEM_PRCR_PRC0 | R_SYSTEM_PRCR_PRC1),
           R_SYSTEM_PRCR);

  /* Step 2: Configure VBATT Control - FSP requirement */
#if defined (CONFIG_RA_VBATT_SWITCH) && (CONFIG_RA_VBATT_SWITCH)
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
#endif

#if defined(CONFIG_RA_CLOCK_HOCO) || defined(CONFIG_RA_CLOCK_PLL) || defined(CONFIG_RA_CLOCK_PLL1P)
  /* Step 3: Configure HOCO (High-Speed On-Chip Oscillator) */
  /* Set HOCO frequency first if needed */
  #if CONFIG_RA_HOCO_FREQUENCY != 20000000
  modifyreg8(R_SYSTEM_HOCOCR, R_SYSTEM_HOCOCR_HCOFRQ_MASK, RA_HOCO_FREQUENCY);
  #endif

  /* Enable HOCO */
  modifyreg8(R_SYSTEM_HOCOCR, R_SYSTEM_HOCOCR_HCSTP, 0);
  while ((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_HOCOSF) == 0)
    {
    }
#endif

#ifdef CONFIG_RA_CLOCK_MOCO
  /* Step 4: Configure MOCO (Medium-Speed On-Chip Oscillator) */
  modifyreg8(R_SYSTEM_MOCOCR, R_SYSTEM_MOCOCR_MCSTP, 0);
  while ((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_MOCOSF) == 0)
    {
    }
#endif

#if defined(CONFIG_RA_CLOCK_PLL) || defined(CONFIG_RA_CLOCK_PLL1P)
  /* Step 5: Configure PLL - FSP-based PLL setup */
  /* Set PLL source (HOCO or Main OSC) */
  #if CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_HOCO
  /* PLL source is HOCO - no register setting needed (default) */
  #elif CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC
  /* PLL source is Main OSC - configure PLLCCR2 if available */
  modifyreg8(R_SYSTEM_PLLCCR2, R_SYSTEM_PLLCCR2_PLLSEL, R_SYSTEM_PLLCCR2_PLLSEL);
  #endif

  /* Set PLL multiplier, divider, and source selection (PLLCCR) for RA8E1 */
  /* Based on FSP: PLLCCR_TYPE=3 uses PLLCCR register with specific bit layout */
  /* PLIDIV: bits 1-0, PLSRCSEL: bit 4, PLLMUL: bits 15-8 */
#if CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_HOCO
  /* HOCO source: PLSRCSEL=1 */
  putreg16((CONFIG_RA_PLL_DIV & 0x03) | (1 << 4) | ((CONFIG_RA_PLL_MUL & 0xFF) << 8), R_SYSTEM_PLLCCR);
#else
  /* Main OSC source: PLSRCSEL=0 */
  putreg16((CONFIG_RA_PLL_DIV & 0x03) | (0 << 4) | ((CONFIG_RA_PLL_MUL & 0xFF) << 8), R_SYSTEM_PLLCCR);
#endif

  /* Set PLL output dividers (PLLCCR2) for RA8E1 */
  modifyreg8(R_SYSTEM_PLLCCR2,
             (R_SYSTEM_PLLCCR2_PLODIV0_MASK | R_SYSTEM_PLLCCR2_PLODIV1_MASK | R_SYSTEM_PLLCCR2_PLODIV2_MASK),
             ((CONFIG_RA_PLL1P_DIV << R_SYSTEM_PLLCCR2_PLODIV0_Pos) |
              (CONFIG_RA_PLL1Q_DIV << R_SYSTEM_PLLCCR2_PLODIV1_Pos) |
              (CONFIG_RA_PLL1R_DIV << R_SYSTEM_PLLCCR2_PLODIV2_Pos)));

  /* Enable PLL */
  modifyreg8(R_SYSTEM_PLLCR, R_SYSTEM_PLLCR_PLLSTP, 0);
  while ((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_PLLSF) == 0)
    {
    }
#endif

  /* Step 6: Disable Flash Cache during clock transition - FSP requirement */
  modifyreg16(R_FCACHE_FCACHEE, R_FCACHE_FCACHEE_FCACHEEN, 0);

  /* Step 7: Configure memory wait states BEFORE clock switch for high frequencies */
#if (RA_ICLK_FREQUENCY > 32000000)
  modifyreg8(R_SYSTEM_MEMWAIT, 0, R_SYSTEM_MEMWAIT_MEMWAIT);
#endif

  /* Step 8: Configure system clock dividers - FSP-based divider setup */
  sckdivcr_value = ((CONFIG_RA_ICK_DIV << 24) |     /* ICLK */
                    (CONFIG_RA_FCLK_DIV << 28) |    /* FCLK */
                    (CONFIG_RA_PCKA_DIV << 12) |    /* PCKA */
                    (CONFIG_RA_PCKB_DIV << 8) |     /* PCKB */
                    (CONFIG_RA_PCKC_DIV << 4) |     /* PCKC */
                    (CONFIG_RA_PCKD_DIV << 0));     /* PCKD */

  putreg32(sckdivcr_value, R_SYSTEM_SCKDIVCR);

  /* Step 9: Set system clock source - FSP-based clock selection */
  modifyreg8(R_SYSTEM_SCKSCR, R_SYSTEM_SCKSCR_CKSEL_MASK, RA_CKSEL);

  /* Step 10: Configure peripheral clock sources if available */
#if CONFIG_RA_SCICLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED
  /* Configure SCI clock source and divider */
  /* This is implementation-specific based on RA8E1 registers */
#endif

#ifdef CONFIG_RA_FLASH_CACHE_ENABLE
  /* Step 11: Re-enable Flash Cache after clock configuration */
  modifyreg16(R_FCACHE_FCACHEE, 0, R_FCACHE_FCACHEE_FCACHEEN);
#endif

  /* Step 12: Lock system registers */
  putreg16(RA_PRCR_LOCK, R_SYSTEM_PRCR);

  /* Update internal clock configuration */
  ra_update_clock_config();
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
        g_fsp_clock_config.system_clock_freq = CONFIG_RA_HOCO_FREQUENCY;
        g_fsp_clock_config.hoco_enabled = true;
        break;

      case 1: /* MOCO */
        g_fsp_clock_config.system_clock_freq = CONFIG_RA_MOCO_FREQUENCY;
        g_fsp_clock_config.moco_enabled = true;
        break;

      case 5: /* PLL */
        g_fsp_clock_config.system_clock_freq = RA_SYSTEM_CLOCK_FREQUENCY;
        g_fsp_clock_config.pll_enabled = true;
        break;

      default:
        g_fsp_clock_config.system_clock_freq = CONFIG_RA_HOCO_FREQUENCY;
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
