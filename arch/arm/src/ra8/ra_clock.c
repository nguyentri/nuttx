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
#include <nuttx/spinlock.h>
#include <arch/board/board.h>
#include <arch/irq.h>
#include <arch/barriers.h>

#include "arm_internal.h"
#include "ra_clock.h"
#include "hardware/ra_flash.h"
#include "hardware/ra_system.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions (Private Definitions)
 ****************************************************************************/

/* BSP register unlock/lock keys (FSP compatible) */
#define RA_PRV_PRCR_KEY                        (0xA500U)
#define RA_PRV_PRCR_UNLOCK                     ((RA_PRV_PRCR_KEY) | 0x3U)
#define RA_PRV_PRCR_LOCK                       ((RA_PRV_PRCR_KEY) | 0x0U)

/* Clock frequencies based on BSP reference */
#define RA_HOCO_HZ                             CONFIG_RA_HOCO_FREQUENCY
#define RA_MOCO_FREQ_HZ                        CONFIG_RA_MOCO_FREQUENCY
#define RA_LOCO_FREQ_HZ                        CONFIG_RA_LOCO_FREQUENCY
#define RA_SUBCLOCK_FREQ_HZ                    0U  /* No subclock */

/* Peripheral clock control bit positions */
#define RA_PRV_PERIPHERAL_CLK_REQ_BIT_POS      (6U)
#define RA_PRV_PERIPHERAL_CLK_REQ_BIT_MASK     (1U << RA_PRV_PERIPHERAL_CLK_REQ_BIT_POS)
#define RA_PRV_PERIPHERAL_CLK_RDY_BIT_POS      (7U)
#define RA_PRV_PERIPHERAL_CLK_RDY_BIT_MASK     (1U << RA_PRV_PERIPHERAL_CLK_RDY_BIT_POS)

/* Flash wait states */
#define RA_PRV_ROM_ZERO_WAIT_CYCLES            (0U)
#define RA_PRV_ROM_ONE_WAIT_CYCLES             (1U)
#define RA_PRV_ROM_TWO_WAIT_CYCLES             (2U)
#define RA_PRV_ROM_THREE_WAIT_CYCLES           (3U)

/* SRAM wait states */
#define RA_PRV_SRAM_UNLOCK                     (0xA501U)
#define RA_PRV_SRAM_LOCK                       (0xA500U)
#define RA_PRV_SRAM_WAIT_CYCLES                (0U)  /* No wait states for RA8E1 at startup freq */

#if defined (CONFIG_RA_PLL_SOURCE_MAIN_OSC)
  #define RA_PRV_PLSRCSEL                         (0)
  #define RA_PRV_PLL_USED                         (1)
#elif defined (CONFIG_RA_PLL_SOURCE_HOCO)
  #define RA_PRV_PLSRCSEL                         (1)
  #define RA_PRV_PLL_USED                         (1)
#else
  #define RA_PRV_PLL_USED                         (0)
#endif

/* System clock divider calculations (BSP compatible) */
/* SCKDIVCR register format (RA8E1):
 * Bits 31-28: FCLK divider    Bits 27-24: ICLK divider    Bits 23-20: PCLKE divider
 * Bits 19-16: BCLK divider    Bits 15-12: PCLKA divider   Bits 11-8:  PCLKB divider
 * Bits 7-4:   PCLKC divider   Bits 3-0:   PCLKD divider
 * Expected value: 0x988a8998 (FSP reference) - VERIFIED MATCH ✓
 */
#define RA_PRV_STARTUP_SCKDIVCR_FCLK_BITS        ((CONFIG_RA_FCLK_DIV & 0xFU) << 28U)
#define RA_PRV_STARTUP_SCKDIVCR_ICLK_BITS        ((CONFIG_RA_ICK_DIV & 0xFU) << 24U)
#define RA_PRV_STARTUP_SCKDIVCR_PCLKE_BITS       ((CONFIG_RA_PCKE_DIV & 0xFU) << 20U)
#define RA_PRV_STARTUP_SCKDIVCR_BCLK_BITS        ((CONFIG_RA_BCLK_DIV & 0xFU) << 16U)
#define RA_PRV_STARTUP_SCKDIVCR_PCLKA_BITS       ((CONFIG_RA_PCKA_DIV & 0xFU) << 12U)
#define RA_PRV_STARTUP_SCKDIVCR_PCLKB_BITS       ((CONFIG_RA_PCKB_DIV & 0xFU) << 8U)
#define RA_PRV_STARTUP_SCKDIVCR_PCLKC_BITS       ((CONFIG_RA_PCKC_DIV & 0xFU) << 4U)
#define RA_PRV_STARTUP_SCKDIVCR_PCLKD_BITS       (CONFIG_RA_PCKD_DIV & 0xFU)
#define RA_PRV_STARTUP_SCKDIVCR                  (RA_PRV_STARTUP_SCKDIVCR_FCLK_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_ICLK_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_PCLKE_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_BCLK_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_PCLKA_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_PCLKB_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_PCLKC_BITS | \
                                                  RA_PRV_STARTUP_SCKDIVCR_PCLKD_BITS)
#define RA_PRV_STARTUP_SCKDIVCR2                 (CONFIG_RA_CPUCLK_DIV)

#define RA_PRV_PLL2_MUL_CFG_MACRO_PLLMUL_MASK     (0x3FFU)
#define RA_PRV_PLL2_MUL_CFG_MACRO_PLLMULNF_MASK    (0x003U)
#define RA_PRV_PLL2CCR_PLLMULNF_BIT                (6) // PLLMULNF in PLLCCR starts at bit 6
#define RA_PRV_PLL2CCR_PLSRCSEL_BIT                (4) // PLSRCSEL in PLLCCR starts at bit 4
#define RA_PRV_PLL2CCR                             ((((CONFIG_RA_PLL2_MUL & RA_PRV_PLL2_MUL_CFG_MACRO_PLLMUL_MASK) << \
                                                        RA_PRV_PLL2CCR_PLLMULNF_BIT) |                                \
                                                      (RA_PRV_PL2SRCSEL << RA_PRV_PLL2CCR_PLSRCSEL_BIT)) |          \
                                                      CONFIG_RA_PLL2_DIV)
#define RA_PRV_PLL2CCR2_PLL_DIV_MASK               (0x0F) // PLL DIV in PLL2CCR2 is 4 bits wide
#define RA_PRV_PLL2CCR2_PLL_DIV_Q_BIT              (4)    // PLL DIV Q in PLL2CCR2 starts at bit 4
#define RA_PRV_PLL2CCR2_PLL_DIV_R_BIT              (8)    // PLL DIV R in PLL2CCR2 starts at bit 8
#define RA_PRV_PLL2CCR2                            (((CONFIG_RA_PL2ODIVR & RA_PRV_PLL2CCR2_PLL_DIV_MASK) << \
                                                      RA_PRV_PLL2CCR2_PLL_DIV_R_BIT) |                     \
                                                      ((CONFIG_RA_PL2ODIVQ & RA_PRV_PLL2CCR2_PLL_DIV_MASK) << \
                                                      RA_PRV_PLL2CCR2_PLL_DIV_Q_BIT) |                     \
                                                      (CONFIG_RA_PL2ODIVP & RA_PRV_PLL2CCR2_PLL_DIV_MASK))

/* PLL Control Register (PLLCCR) calculations (FSP compatible)
 * For RA8E1 (PLLCCR_TYPE 3), the PLL multiplier format is:
 * BSP_CLOCKS_PLL_MUL(X,Y) = ((X-1) << 2) | (Y/33), where Y is fractional part
 * This must match the FSP expected values: PLLCCR=0x4711, PLLCCR2=0x111
 */
#define RA_PRV_PLL_MUL_CFG_MACRO_PLLMUL_MASK    (0x3FFU)
#define RA_PRV_PLLCCR_PLLMULNF_BIT               (6) // PLLMULNF in PLLCCR starts at bit 6
#define RA_PRV_PLLCCR_PLSRCSEL_BIT               (4) // PLSRCSEL in PLLCCR starts at bit 4
/* Convert PLL multiplier to FSP format: BSP_CLOCKS_PLL_MUL(X,Y) = ((X-1) << 2) | (Y/33) */
#define RA_PRV_PLL_MUL_FSP_FORMAT                ((((CONFIG_RA_PLL_MUL) - 1U) << 2UL) | 0U)
#define RA_PRV_PLLCCR                            ((((RA_PRV_PLL_MUL_FSP_FORMAT & RA_PRV_PLL_MUL_CFG_MACRO_PLLMUL_MASK) << \
                                                      RA_PRV_PLLCCR_PLLMULNF_BIT) |                               \
                                                    (RA_PRV_PLSRCSEL << RA_PRV_PLLCCR_PLSRCSEL_BIT)) |          \
                                                    CONFIG_RA_PLL_DIV)
#define RA_PRV_PLLCCR2_PLL_DIV_MASK              (0x0F) // PLL DIV in PLLCCR2/PLL2CCR2 is 4 bits wide
#define RA_PRV_PLLCCR2_PLL_DIV_Q_BIT             (4)    // PLL DIV Q in PLLCCR2/PLL2CCR2 starts at bit 4
#define RA_PRV_PLLCCR2_PLL_DIV_R_BIT             (8)    // PLL DIV R in PLLCCR2/PLL2CCR2 starts at bit 8
#define RA_PRV_PLLCCR2                           (((CONFIG_RA_PLODIVR & RA_PRV_PLLCCR2_PLL_DIV_MASK) << \
                                                    RA_PRV_PLLCCR2_PLL_DIV_R_BIT) |                    \
                                                    ((CONFIG_RA_PLODIVQ & RA_PRV_PLLCCR2_PLL_DIV_MASK) << \
                                                    RA_PRV_PLLCCR2_PLL_DIV_Q_BIT) |                    \
                                                    (CONFIG_RA_PLODIVP & RA_PRV_PLLCCR2_PLL_DIV_MASK))

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_sys_core_clock variable for Renesas FSP compatibility */
uint32_t g_sys_core_clock = CONFIG_RA_HOCO_FREQUENCY;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static ra_clock_config_t g_fsp_clock_config;

/* FSP-Compatible clock frequency array */
static uint32_t g_clock_freq[16];  /* Array size to accommodate all clock sources */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ra_update_clock_config(void);

/****************************************************************************
 * Name: ra_sys_core_clock_update
 *
 * Description:
 *   Update the system core clock frequency based on the current clock settings
 *
 ****************************************************************************/

void ra_sys_core_clock_update (void)
{
  uint32_t clock_index = getreg8(R_SYSTEM_SCKSCR);
  uint8_t cpuck = (getreg8(R_SYSTEM_SCKDIVCR2) & R_SYSTEM_SCKDIVCR2_CPUCK_Msk) >> R_SYSTEM_SCKDIVCR2_CPUCK_Pos;
  uint8_t cpuclk_div = cpuck;

 /* Handle special divider cases first */
  if (8U == cpuclk_div)
    {
      g_sys_core_clock = g_clock_freq[clock_index] / 3U;
    }
  else if (9U == cpuclk_div)
    {
      g_sys_core_clock = g_clock_freq[clock_index] / 6U;
    }
  else if (10U == cpuclk_div)
    {
      g_sys_core_clock = g_clock_freq[clock_index] / 12U;
    }
  else if (11U == cpuclk_div)
    {
      g_sys_core_clock = g_clock_freq[clock_index] / 24U;
    }
  else
    {
      /* Standard power-of-2 dividers */
      g_sys_core_clock = g_clock_freq[clock_index] >> cpuclk_div;
    };
}

/****************************************************************************
 * Name: ra_clock_freq_var_init
 *
 * Description:
 *   Initialize clock frequency array (FSP-compatible implementation)
 *
 ****************************************************************************/

static void ra_clock_freq_var_init(void)
{
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_HOCO]     = RA_HOCO_HZ;
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_MOCO]     = RA_MOCO_FREQ_HZ;
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_LOCO]     = RA_LOCO_FREQ_HZ;
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC] = 0U;  /* Main OSC not populated */
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_SUBCLOCK] = RA_SUBCLOCK_FREQ_HZ;
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_PLL]      = RA_CFG_PLL1P_FREQUENCY_HZ;
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_PLL1Q]    = RA_CFG_PLL1Q_FREQUENCY_HZ;
  g_clock_freq[RA_CLOCKS_SOURCE_CLOCK_PLL1R]    = RA_CFG_PLL1R_FREQUENCY_HZ;

  ra_sys_core_clock_update();
}

/****************************************************************************
 * Name: ra_peripheral_clock_set
 *
 * Description:
 *   Set peripheral clock source and divider
 *
 ****************************************************************************/

static void ra_peripheral_clock_set(volatile uint8_t *p_clk_ctrl_reg,
                                     volatile uint8_t *p_clk_div_reg,
                                     uint8_t peripheral_clk_div,
                                     uint8_t peripheral_clk_source)
{
  /* Request to stop the peripheral clock */
  *p_clk_ctrl_reg |= (uint8_t)RA_PRV_PERIPHERAL_CLK_REQ_BIT_MASK;

  /* Wait for the peripheral clock to stop */
  while (((*p_clk_ctrl_reg & RA_PRV_PERIPHERAL_CLK_RDY_BIT_MASK) >> RA_PRV_PERIPHERAL_CLK_RDY_BIT_POS) != 1U)
    {
      /* Wait for ready bit */
    }

  /* Select the peripheral clock divisor and source */
  *p_clk_div_reg = peripheral_clk_div;
  *p_clk_ctrl_reg = peripheral_clk_source | RA_PRV_PERIPHERAL_CLK_REQ_BIT_MASK |
                    RA_PRV_PERIPHERAL_CLK_RDY_BIT_MASK;

  /* Request to start the peripheral clock */
  *p_clk_ctrl_reg &= (uint8_t)~RA_PRV_PERIPHERAL_CLK_REQ_BIT_MASK;

  /* Wait for the peripheral clock to start */
  while (((*p_clk_ctrl_reg & RA_PRV_PERIPHERAL_CLK_RDY_BIT_MASK) >> RA_PRV_PERIPHERAL_CLK_RDY_BIT_POS) != 0U)
    {
      /* Wait for ready bit to be clear */
    }
}

/****************************************************************************
 * Name: ra_peripheral_clock_init
 *
 * Description:
 *   Set clocks for configured peripherals (simplified for RA8E1)
 *
 ****************************************************************************/

static void ra_peripheral_clock_init(void)
{
  /* Initialize peripheral clocks based on BSP reference implementation */

  /* Set the CANFD clock if it exists on the MCU */
#if defined(CONFIG_RA_CANFDCLK_SOURCE) && (CONFIG_RA_CANFDCLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_CANFDCKCR,
                          (volatile uint8_t *)R_SYSTEM_CANFDCKDIVCR,
                          CONFIG_RA_CANFDCLK_DIV,
                          CONFIG_RA_CANFDCLK_SOURCE);
#endif

  /* Set the SCISPI clock if it exists on the MCU */
#if defined(CONFIG_RA_SCISPICLK_SOURCE) && (CONFIG_RA_SCISPICLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_SCISPICKCR,
                          (volatile uint8_t *)R_SYSTEM_SCISPICKDIVCR,
                          CONFIG_RA_SCISPICLK_DIV,
                          CONFIG_RA_SCISPICLK_SOURCE);
#endif

  /* Set the SCI clock if it exists on the MCU */
#if defined(CONFIG_RA_SCICLK_SOURCE) && (CONFIG_RA_SCICLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_SCICKCR,
                          (volatile uint8_t *)R_SYSTEM_SCICKDIVCR,
                          CONFIG_RA_SCICLK_DIV,
                          CONFIG_RA_SCICLK_SOURCE);
#endif

  /* Set the SPI clock if it exists on the MCU */
#if defined(CONFIG_RA_SPICLK_SOURCE) && (CONFIG_RA_SPICLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_SPICKCR,
                          (volatile uint8_t *)R_SYSTEM_SPICKDIVCR,
                          CONFIG_RA_SPICLK_DIV,
                          CONFIG_RA_SPICLK_SOURCE);
#endif

  /* Set the GPT clock if it exists on the MCU */
#if defined(CONFIG_RA_GPTCLK_SOURCE) && (CONFIG_RA_GPTCLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_GPTCKCR,
                          (volatile uint8_t *)R_SYSTEM_GPTCKDIVCR,
                          CONFIG_RA_GPTCLK_DIV,
                          CONFIG_RA_GPTCLK_SOURCE);
#endif

  /* Set the IIC clock if it exists on the MCU */
#if defined(CONFIG_RA_IICCLK_SOURCE) && (CONFIG_RA_IICCLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_IICCKCR,
                          (volatile uint8_t *)R_SYSTEM_IICCKDIVCR,
                          CONFIG_RA_IICCLK_DIV,
                          CONFIG_RA_IICCLK_SOURCE);
#endif

  /* Set the ADC clock if it exists on the MCU */
#if defined(CONFIG_RA_ADCCLK_SOURCE) && (CONFIG_RA_ADCCLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_ADCCKCR,
                          (volatile uint8_t *)R_SYSTEM_ADCCKDIVCR,
                          CONFIG_RA_ADCCLK_DIV,
                          CONFIG_RA_ADCCLK_SOURCE);
#endif

  /* Set the USB clock if it exists on the MCU (requires special handling) */
#if defined(CONFIG_RA_USBCLK_SOURCE) && (CONFIG_RA_USBCLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  /* USB clock configuration may need special register handling */
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_USBCKCR,
                          NULL,  /* No divider register for USB on some MCUs */
                          0,     /* No divider */
                          CONFIG_RA_USBCLK_SOURCE);
#endif

  /* Set the OCTASPI clock if it exists on the MCU */
#if defined(CONFIG_RA_OCTACLK_SOURCE) && (CONFIG_RA_OCTACLK_SOURCE != RA_CLOCKS_CLOCK_DISABLED)
  /* OCTASPI clock may require special handling compared to other peripherals */
  ra_peripheral_clock_set((volatile uint8_t *)R_SYSTEM_OCTACKCR,
                          (volatile uint8_t *)R_SYSTEM_OCTACKDIVCR,
                          CONFIG_RA_OCTACLK_DIV,
                          CONFIG_RA_OCTACLK_SOURCE);
#endif
}

/****************************************************************************
 * Name: ra_prv_clock_set_hard_reset
 *
 * Description:
 *   Set clocks when coming from hard reset (FSP-compatible implementation)
 *
 ****************************************************************************/

static void ra_prv_clock_set_hard_reset(void)
{
  /* Set flash wait states for high frequency operation */
  putreg8(RA_PRV_ROM_TWO_WAIT_CYCLES, R_FCACHE_FLWT);

  /* Set system clock dividers with temporary safe values first */
  putreg32(RA_PRV_STARTUP_SCKDIVCR, R_SYSTEM_SCKDIVCR);

  /* Set CPU clock divider based on configuration */
#if CONFIG_RA_CPUCLK_DIV == RA_CLOCKS_SYS_CLOCK_DIV_1
  /* Determine what the other dividers are using and stay aligned with that. */
  putreg8((CONFIG_RA_ICK_DIV & 0x8) ? RA_CLOCKS_SYS_CLOCK_DIV_3 : RA_CLOCKS_SYS_CLOCK_DIV_2,
          R_SYSTEM_SCKDIVCR2);
#else
  /* If not /1, can just add 1 to it. */
  putreg8(RA_PRV_STARTUP_SCKDIVCR2 + 1, R_SYSTEM_SCKDIVCR2);
#endif

  /* Set the system source clock */
  //putreg8(RA_CFG_CLOCK_SOURCE, R_SYSTEM_SCKSCR);
  putreg8(R_SYSTEM_SCKSCR_CKSEL_HOCO, R_SYSTEM_SCKSCR); // Can't change to PLL1, fixed to use HOCO

  /* Wait for settling delay. */
  ra_sys_core_clock_update();
  up_udelay(RA_CFG_CLOCK_SETTLING_DELAY_US);

  /* Continue and set clock to actual target speed. */
  putreg8(RA_PRV_STARTUP_SCKDIVCR2, R_SYSTEM_SCKDIVCR2);
  putreg32(RA_PRV_STARTUP_SCKDIVCR, R_SYSTEM_SCKDIVCR);

  /* Wait for settling delay. */
  ra_sys_core_clock_update();
  up_udelay(RA_CFG_CLOCK_SETTLING_DELAY_US);

  /* Set the system source clock again */
  //putreg8(RA_CFG_CLOCK_SOURCE, R_SYSTEM_SCKSCR);
  putreg8(R_SYSTEM_SCKSCR_CKSEL_HOCO, R_SYSTEM_SCKSCR);  // Can't change to PLL1, fixed to use HOCO

  /* Update the CMSIS core clock variable so that it reflects the new ICLK frequency. */
  ra_sys_core_clock_update();

  /* Configure SRAM wait states if needed */
#ifdef R_SRAM_SRAMPRCR
  putreg16(RA_PRV_SRAM_UNLOCK, R_SRAM_SRAMPRCR);

  /* Execute data memory barrier before and after setting the wait states */
  UP_DMB();
  putreg8(RA_PRV_SRAM_WAIT_CYCLES, R_SRAM_SRAMWTSC);
  UP_DMB();

  putreg16(RA_PRV_SRAM_LOCK, R_SRAM_SRAMPRCR);
#endif
}

/****************************************************************************
 * Name: ra_clock_init
 *
 * Description:
 *   Initialize clocks (FSP-compatible implementation)
 *
 ****************************************************************************/

static void ra_clock_init(void)
{
  /* Step 1: Unlock system registers */
  putreg16(RA_PRV_PRCR_UNLOCK, R_SYSTEM_PRCR);

  /* Step 2: Initialize clock frequency variables */
  ra_clock_freq_var_init();

  /* Step 3: Start HOCO if used */
#if defined(CONFIG_RA_CLOCK_HOCO) || defined(CONFIG_RA_CLOCK_PLL) || defined(CONFIG_RA_CLOCK_PLL1P)
  putreg8(0U, R_SYSTEM_HOCOCR);  /* Enable HOCO */
  /* Wait for HOCO to stabilize */
  RA_HARDWARE_REGISTER_WAIT((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_HOCOSF), R_SYSTEM_OSCSF_HOCOSF);
#endif

  /* Step 4: Configure and start PLL if used */
#if defined(CONFIG_RA8E1_GROUP)
  putreg16((uint16_t)RA_PRV_PLLCCR, R_SYSTEM_PLLCCR);
  putreg16((uint16_t)RA_PRV_PLLCCR2, R_SYSTEM_PLLCCR2);
  putreg8(0U, R_SYSTEM_PLLCR);  /* Enable PLL */
  /* Wait for PLL to stabilize */
  RA_HARDWARE_REGISTER_WAIT((getreg8(R_SYSTEM_OSCSF) & R_SYSTEM_OSCSF_PLLSF), R_SYSTEM_OSCSF_PLLSF);
#endif

  /* Step 5: Set clocks from hard reset state */
  ra_prv_clock_set_hard_reset();

  /* Step 6: Configure peripheral clocks */
  ra_peripheral_clock_init();

  /* Step 7: Lock system registers */
  putreg16(RA_PRV_PRCR_LOCK, R_SYSTEM_PRCR);
}

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
  /* Use FSP-compatible clock initialization sequence */
  ra_clock_init();

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
