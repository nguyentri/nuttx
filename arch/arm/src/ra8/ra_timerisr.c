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
#include <time.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FSP-based timer configuration */
#ifdef CONFIG_RA_SYSTICK_GPT
#  include "hardware/ra_gpt.h"
#  include "ra_mstp.h"
#  define RA_TIMER_CLOCK    (RA_PCLKD_FREQUENCY)
#  define RA_TIMER_RELOAD   ((RA_TIMER_CLOCK / CLK_TCK) - 1)
#  define RA_GPT_CHANNEL    0
#else
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ra_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARMV8M_SYSTICK) && !defined(CONFIG_TIMER_ARCH)
static int ra_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}
#endif

#ifdef CONFIG_RA_SYSTICK_GPT
/****************************************************************************
 * Function:  ra_gpt_timerisr
 *
 * Description:
 *   GPT-based timer interrupt handler for FSP compatibility
 *
 ****************************************************************************/

static int ra_gpt_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear GPT compare match flag */
  uint32_t regaddr = RA_GPT_BASE + RA_GPT_GTSTR_OFFSET;
  uint32_t channel_mask = (1 << RA_GPT_CHANNEL);
  
  putreg32(channel_mask, regaddr);
  
  /* Process timer interrupt */
  nxsched_process_timer();
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
 *   the timer interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

#ifdef CONFIG_RA_SYSTICK_GPT
  /* FSP-based GPT timer initialization */
  
  /* Enable GPT module clock */
  ra_mstp_start(RA_MSTP_GPT_1);
  
  /* Stop GPT channel */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_BASE + RA_GPT_GTSTP_OFFSET);
  
  /* Configure GPT channel for periodic mode */
  regval = (RA_GPT_GTWP_WP_KEY << RA_GPT_GTWP_WP_SHIFT) |
           (1 << RA_GPT_CHANNEL);
  putreg32(regval, RA_GPT_BASE + RA_GPT_GTWP_OFFSET);
  
  /* Set count direction and mode */
  putreg32(0, RA_GPT_BASE + RA_GPT_GTUD_OFFSET + (RA_GPT_CHANNEL * 4));
  putreg32(0, RA_GPT_BASE + RA_GPT_GTCR_OFFSET + (RA_GPT_CHANNEL * 4));
  
  /* Set period (compare match A) */
  putreg32(RA_TIMER_RELOAD, 
           RA_GPT_BASE + RA_GPT_GTPR_OFFSET + (RA_GPT_CHANNEL * 4));
  
  /* Enable compare match A interrupt */
  regval = (1 << 16); /* GTCCRA interrupt enable */
  putreg32(regval, RA_GPT_BASE + RA_GPT_GTIER_OFFSET + (RA_GPT_CHANNEL * 4));
  
  /* Attach the GPT interrupt vector */
  irq_attach(RA_IRQ_GPT0_COMPARE_A + RA_GPT_CHANNEL, 
             (xcpt_t)ra_gpt_timerisr, NULL);
  
  /* Enable GPT interrupt */
  up_enable_irq(RA_IRQ_GPT0_COMPARE_A + RA_GPT_CHANNEL);
  
  /* Start GPT timer */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_BASE + RA_GPT_GTSTR_OFFSET);
  
#elif defined(CONFIG_ARMV8M_SYSTICK) && defined(CONFIG_TIMER_ARCH)
  /* ARM Cortex-M systick with timer arch */
  up_timer_set_lowerhalf(systick_initialize(true, SYSTICK_CLOCK, -1));
  
#else
  /* Standard ARM Cortex-M SysTick configuration */
  
  /* Set the SysTick interrupt to the default priority */
  regval = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

  /* Configure SysTick to interrupt at the requested rate */
  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the timer interrupt vector */
  irq_attach(RA_IRQ_SYSTICK, (xcpt_t)ra_timerisr, NULL);

  /* Enable SysTick interrupts */
  putreg32((NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
            NVIC_SYSTICK_CTRL_ENABLE), NVIC_SYSTICK_CTRL);

  /* And enable the timer interrupt */
  up_enable_irq(RA_IRQ_SYSTICK);
#endif
}
