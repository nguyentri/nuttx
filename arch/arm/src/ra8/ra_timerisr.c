/****************************************************************************
 * arch/arm/src/ra8/ra_timerisr.c
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
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>
#include <arch/irq.h>

#include "nvic.h"
#include "hardware/ra_gpt.h"
#include "hardware/ra8e1/ra8e1_icu.h"
#include "hardware/ra8e1/ra8e1_memorymap.h"
#include "ra_clock.h"
#include "ra_icu.h"
#include "ra_mstp.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  GPT timer configuration */
#ifdef CONFIG_RA_SYSTICK_GPT
#  include "ra_mstp.h"
#  define RA_TIMER_CLOCK    (RA_PCLKD_FREQUENCY)
#  define RA_TIMER_RELOAD   ((RA_TIMER_CLOCK / CLK_TCK) - 1)
#  define RA_GPT_CHANNEL    3  /* Use GPT3 for system timer */
#  define RA_GPT_CHANNEL_BASE  R_GPT3_BASE
#  define RA_GPT_IRQ_EVENT     EVENT_GPT3_COUNTER_OVERFLOW
#else
#  include "ra_clock.h"
#  define SYSTICK_CLOCK     (RA_ICLK_FREQUENCY)
#  define SYSTICK_RELOAD    ((SYSTICK_CLOCK / CLK_TCK) - 1)
#endif

/* The size of the reload field is 24 bits.  Verify that the reload value
 * will fit in the reload register.
 */

#define SYSTICK_MAX 0x00ffffff
#if defined(SYSTICK_RELOAD) && SYSTICK_RELOAD > SYSTICK_MAX
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/* FSP-based timer validation */
#ifdef CONFIG_RA_SYSTICK_GPT
#  if RA_TIMER_RELOAD > 0xFFFFFFFF
#    error GPT timer reload value exceeds 32-bit range
#  endif
#endif

/* GPT Register Addresses for specific channel */
#ifdef CONFIG_RA_SYSTICK_GPT
#  define RA_GPT_GTWP       (RA_GPT_CHANNEL_BASE + RA_GPT_GTWP_OFFSET)
#  define RA_GPT_GTSTR      (RA_GPT_CHANNEL_BASE + RA_GPT_GTSTR_OFFSET)
#  define RA_GPT_GTSTP      (RA_GPT_CHANNEL_BASE + RA_GPT_GTSTP_OFFSET)
#  define RA_GPT_GTCLR      (RA_GPT_CHANNEL_BASE + RA_GPT_GTCLR_OFFSET)
#  define RA_GPT_GTCR       (RA_GPT_CHANNEL_BASE + RA_GPT_GTCR_OFFSET)
#  define RA_GPT_GTPR       (RA_GPT_CHANNEL_BASE + RA_GPT_GTPR_OFFSET)
#  define RA_GPT_GTINTAD    (RA_GPT_CHANNEL_BASE + RA_GPT_GTINTAD_OFFSET)
#  define RA_GPT_GTST       (RA_GPT_CHANNEL_BASE + RA_GPT_GTST_OFFSET)
#  define RA_GPT_GTCNT      (RA_GPT_CHANNEL_BASE + RA_GPT_GTCNT_OFFSET)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ra_timer
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARMV8M_SYSTICK) && !defined(CONFIG_TIMER_ARCH)
int ra_timer_arch_isr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}
#endif

#ifdef CONFIG_RA_SYSTICK_GPT
/****************************************************************************
 * Function:  ra_timer_gpt_isr
 *
 * Description:
 *   GPT-based timer interrupt handler. Based on the working FSP GPT driver
 *   implementation and adapted for NuttX system timer.
 *
 ****************************************************************************/

static int ra_timer_gpt_isr(int irq, uint32_t *regs, void *arg)
{
  uint32_t status;

  /* Read and clear GPT overflow status flag */
  status = getreg32(RA_GPT_GTST);

  /* Check if overflow interrupt occurred */
  if (status & GPT_GTST_TCFPO)
    {
      /* Clear overflow flag by writing 0 to it */
      putreg32(status & ~GPT_GTST_TCFPO, RA_GPT_GTST);

      /* Process timer interrupt */
      nxsched_process_timer();
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize
 *   the timer interrupt. Implementation based on working FSP GPT driver
 *   and adapted for NuttX system timer requirements.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

#ifdef CONFIG_RA_SYSTICK_GPT
  /* GPT-based timer initialization using GPT3 */

  /* Enable GPT module clock */
  ra_mstp_start(RA_MSTP_GPT_1);

  /* Disable write protection to configure GPT */
  regval = (GPT_GTWP_PRKEY | GPT_GTWP_WP);
  putreg32(regval, RA_GPT_GTWP);

  /* Stop GPT channel if running */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_GTSTP);

  /* Clear GPT counter */
  putreg32(0, RA_GPT_GTCNT);

  /* Configure GPT control register for periodic mode */
  regval = GPT_GTCR_MD_SAW_WAVE_UP |      /* Saw-wave PWM mode (up-counting) */
           GPT_GTCR_TPCS_PCLKD_1;         /* Use PCLKD as clock source */
  putreg32(regval, RA_GPT_GTCR);

  /* Set period register for desired interrupt frequency */
  putreg32(RA_TIMER_RELOAD, RA_GPT_GTPR);

  /* Enable overflow interrupt */
  regval = GPT_GTINTAD_GTINTV;  /* Overflow interrupt enable */
  putreg32(regval, RA_GPT_GTINTAD);

  /* Clear any pending interrupt flags */
  regval = getreg32(RA_GPT_GTST);
  regval &= ~GPT_GTST_TCFPO;  /* Clear overflow flag */
  putreg32(regval, RA_GPT_GTST);

  /* Set up ICU event linking for GPT3 overflow interrupt */
  ra_icu_set_event(0, RA_GPT_IRQ_EVENT);  /* Use ICU slot 0 for GPT3 overflow */

  /* Attach the GPT interrupt vector */
  irq_attach(0, (xcpt_t)ra_timer_gpt_isr, NULL);  /* ICU slot 0 */

  /* Enable GPT interrupt */
  up_enable_irq(0);  /* ICU slot 0 */

  /* Re-enable write protection */
  regval = (GPT_GTWP_PRKEY);  /* Remove WP bit but keep key */
  putreg32(regval, RA_GPT_GTWP);

  /* Start GPT timer */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_GTSTR);

#else
  /* Standard ARM Cortex-M SysTick configuration */
  /* TODO: Implement proper SysTick fallback if needed */
  /* For now, GPT is the primary timer method for RA8E1 */
#endif
}

#ifdef CONFIG_RA_SYSTICK_GPT
#  ifndef EVENT_GPT3_COUNTER_OVERFLOW
#    warning "EVENT_GPT3_COUNTER_OVERFLOW not defined; verify IRQ mapping for GPT3"
#  endif
#endif
