/****************************************************************************
 * arch/arm/src/ra8/ra_fsp_integration.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_fsp_integration.h"
#include "hardware/ra_system.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ra_fsp_clock_config_t g_fsp_clock_config;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_fsp_update_clock_config
 *
 * Description:
 *   Update FSP clock configuration from hardware registers
 *
 ****************************************************************************/

static void ra_fsp_update_clock_config(void)
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
 * Name: ra_fsp_get_clock_config
 *
 * Description:
 *   Get current FSP-compatible clock configuration
 *
 ****************************************************************************/

void ra_fsp_get_clock_config(ra_fsp_clock_config_t *config)
{
  DEBUGASSERT(config != NULL);

  /* Update configuration from hardware */
  ra_fsp_update_clock_config();

  /* Copy to user structure */
  memcpy(config, &g_fsp_clock_config, sizeof(ra_fsp_clock_config_t));
}

/****************************************************************************
 * Name: ra_fsp_validate_memory_map
 *
 * Description:
 *   Validate memory configuration against FSP requirements and linker script
 *
 ****************************************************************************/

bool ra_fsp_validate_memory_map(void)
{
  bool valid = true;

#ifdef CONFIG_RA_DTCM_HEAP
  /* Validate DTCM configuration against linker script */
  /* Linker: dtcm (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00004000 */
  if (CONFIG_RA_DTCM_BASE != 0x20000000)
    {
      syslog(LOG_ERR, "DTCM base mismatch: config=0x%08x linker=0x20000000\n", 
             CONFIG_RA_DTCM_BASE);
      valid = false;
    }

  if (CONFIG_RA_DTCM_SIZE != 0x4000)
    {
      syslog(LOG_ERR, "DTCM size mismatch: config=%d linker=16384 bytes\n", 
             CONFIG_RA_DTCM_SIZE);
      valid = false;
    }
#endif

#ifdef CONFIG_RA_ITCM_HEAP
  /* Validate ITCM configuration against linker script */
  /* Linker: itcm (rwx) : ORIGIN = 0x00000000, LENGTH = 0x00004000 */
  if (CONFIG_RA_ITCM_BASE != 0x00000000)
    {
      syslog(LOG_ERR, "ITCM base mismatch: config=0x%08x linker=0x00000000\n", 
             CONFIG_RA_ITCM_BASE);
      valid = false;
    }

  if (CONFIG_RA_ITCM_SIZE != 0x4000)
    {
      syslog(LOG_ERR, "ITCM size mismatch: config=%d linker=16384 bytes\n", 
             CONFIG_RA_ITCM_SIZE);
      valid = false;
    }
#endif

#ifdef CONFIG_RA_EXTERNAL_RAM_HEAP
  /* Validate external RAM configuration against linker script */
  /* Linker: ospi0_cs0 (rwx) : ORIGIN = 0x80000000, LENGTH = 0x10000000 */
  if (CONFIG_RA_EXTERNAL_RAM_BASE != 0x80000000)
    {
      syslog(LOG_ERR, "External RAM base mismatch: config=0x%08x linker=0x80000000\n", 
             CONFIG_RA_EXTERNAL_RAM_BASE);
      valid = false;
    }

  if (CONFIG_RA_EXTERNAL_RAM_SIZE > 0x10000000)
    {
      syslog(LOG_ERR, "External RAM size exceeds linker: config=%d max=268435456 bytes\n", 
             CONFIG_RA_EXTERNAL_RAM_SIZE);
      valid = false;
    }
#endif

  /* Validate main SRAM configuration */
  /* Linker: sram (rwx) : ORIGIN = 0x22060000, LENGTH = 0x00080000 */
  if (CONFIG_RA_SRAM_BASE != 0x22060000)
    {
      syslog(LOG_ERR, "SRAM base mismatch: config=0x%08x linker=0x22060000\n", 
             CONFIG_RA_SRAM_BASE);
      valid = false;
    }

  if (CONFIG_RA8_SRAM_SIZE != 0x80000)
    {
      syslog(LOG_ERR, "SRAM size mismatch: config=%d linker=524288 bytes\n", 
             CONFIG_RA8_SRAM_SIZE);
      valid = false;
    }

  /* Validate heap alignment is power of 2 */
  if ((CONFIG_RA_HEAP_ALIGNMENT & (CONFIG_RA_HEAP_ALIGNMENT - 1)) != 0)
    {
      syslog(LOG_ERR, "Heap alignment must be power of 2: %d\n", 
             CONFIG_RA_HEAP_ALIGNMENT);
      valid = false;
    }

  return valid;
}

/****************************************************************************
 * Name: ra_fsp_print_clock_info
 *
 * Description:
 *   Print FSP-compatible clock information for debugging
 *
 ****************************************************************************/

void ra_fsp_print_clock_info(void)
{
#ifdef CONFIG_DEBUG_INFO
  ra_fsp_clock_config_t config;
  
  ra_fsp_get_clock_config(&config);
  
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
 * Name: ra_fsp_get_peripheral_clock
 *
 * Description:
 *   Get peripheral clock frequency for FSP compatibility
 *
 ****************************************************************************/

uint32_t ra_fsp_get_peripheral_clock(int peripheral_id)
{
  ra_fsp_clock_config_t config;
  
  ra_fsp_get_clock_config(&config);
  
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
        return config.iclk_freq;
    }
}

/****************************************************************************
 * Name: ra_fsp_initialize
 *
 * Description:
 *   Initialize FSP integration layer
 *
 ****************************************************************************/

void ra_fsp_initialize(void)
{
  /* Initialize clock configuration */
  memset(&g_fsp_clock_config, 0, sizeof(ra_fsp_clock_config_t));
  ra_fsp_update_clock_config();
  
  /* Validate memory configuration */
  if (!ra_fsp_validate_memory_map())
    {
      syslog(LOG_ERR, "FSP memory map validation failed\n");
    }
  
#ifdef CONFIG_DEBUG_INFO
  /* Print FSP information */
  ra_fsp_print_clock_info();
#endif
}
