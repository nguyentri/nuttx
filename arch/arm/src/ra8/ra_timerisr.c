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
#include "chip.h"
#include "arm_internal.h"
#include "ra_clock.h"
#include "ra_icu.h"
#include "ra_mstp.h"
#include "hardware/ra_gpt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  GPT timer configuration */
#ifdef CONFIG_RA_SYSTICK_GPT
#  include "ra_mstp.h"
#define RA_GPT_CHANNEL        (0)  /* Use GPT0 for system timer */
#define RA_MSTP_GPT(n)           (RA_MSTP_GPT##n)
#define RA_EL_GPT(n)          (RA_EL_GPT##n##COUNTER_OVERFLOW) /* Event link for GPTn overflow */
/* Use the new channel-based register macros */
#define RA_GPT_SYSTICK_GTWP           RA_GPT_GTWP(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTSTR          RA_GPT_GTSTR(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTSTP          RA_GPT_GTSTP(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCLR          RA_GPT_GTCLR(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCR           RA_GPT_GTCR(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTPR           RA_GPT_GTPR(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTINTAD        RA_GPT_GTINTAD(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTST           RA_GPT_GTST_REG(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCNT          RA_GPT_GTCNT(RA_GPT_CHANNEL)
#define RA_TIMER_CLOCK                (RA_PCLKD_FREQUENCY)
#define SYSTICK_RELOAD                ((RA_TIMER_CLOCK / CLK_TCK) - 1)
#if SYSTICK_RELOAD > 0xFFFFFFFF
#  error GPT timer reload value exceeds 32-bit range
#endif
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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  ra_timer_arch_isr
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
 * Function:  ra_systick_isr
 *
 * Description:
 *   GPT-based timer interrupt handler. Based on the working FSP GPT driver
 *   implementation and adapted for NuttX system timer.
 *
 ****************************************************************************/

static int ra_systick_isr(int irq, uint32_t *regs, void *arg)
{
  uint32_t status;

  /* Read and clear GPT overflow status flag - Following FSP best practices */
  status = getreg32(RA_GPT_SYSTICK_GTST);

  /* Check if overflow interrupt occurred */
  if (status & GPT_GTST_TCFPO)
    {
      /* Clear overflow flag by writing 0 to it - FSP compatible approach */
      putreg32(status & ~GPT_GTST_TCFPO, RA_GPT_SYSTICK_GTST);

      /* Process timer interrupt */
      nxsched_process_timer();
    }

  /* Also check for any other enabled interrupt flags and clear them
   * to prevent spurious interrupts - defensive programming from FSP example
   */
  if (status & (GPT_GTST_TCFA | GPT_GTST_TCFB | GPT_GTST_TCFC |
                GPT_GTST_TCFD | GPT_GTST_TCFE | GPT_GTST_TCFF |
                GPT_GTST_TCFPU))
    {
      /* Clear all other potential flags */
      putreg32(0, RA_GPT_SYSTICK_GTST);
    }

  return 0;
}
#else
/* ARM Cortex-M85 SysTick interrupt handler for NuttX system timer */
static int ra_systick_isr(int irq, uint32_t *regs, void *arg)
{
  /* SysTick interrupt is acknowledged automatically by reading the
   * SYST_CSR register or writing to the SYST_CVR register.
   * The COUNTFLAG bit is cleared automatically when SYST_CSR is read.
   */

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
 *   the timer interrupt. Implementation based on working FSP GPT driver
 *   and adapted for NuttX system timer requirements.
 *
 ****************************************************************************/

#ifdef CONFIG_RA_SYSTICK_GPT
void up_timer_initialize(void)
{
  uint32_t regval;

  /* Enable GPT module clock */
  ra_mstp_start(RA_MSTP_GPT(RA_GPT_CHANNEL));

  /* Disable write protection to configure GPT - not supported */
  //regval = (GPT_GTWP_PRKEY | GPT_GTWP_WP);
  //putreg32(regval, RA_GPT_SYSTICK_GTWP);

  /* Stop GPT channel if running */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTP);

  /* Clear GPT counter */
  putreg32(0, RA_GPT_SYSTICK_GTCNT);

  /* Configure GPT control register for periodic mode - Enhanced from FSP */
  regval = GPT_GTCR_MD_SAW_WAVE_UP |      /* Saw-wave PWM mode (up-counting) */
           GPT_GTCR_TPCS_PCLKD_1;         /* Use PCLKD as clock source */
  putreg32(regval, RA_GPT_SYSTICK_GTCR);

  /* Set period register for desired interrupt frequency */
  putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTPR);

  /* Enable overflow interrupt - Key for system timer */
  regval = GPT_GTINTAD_GTINTV;  /* Overflow interrupt enable */
  putreg32(regval, RA_GPT_SYSTICK_GTINTAD);

  /* Clear any pending interrupt flags before enabling interrupts */
  putreg32(0, RA_GPT_SYSTICK_GTST);

  /* Set up ICU event linking for  overflow interrupt */
  ra_icu_set_event(RA_IRQ_SYSTICK_GPT, RA_EL_GPT(RA_GPT_CHANNEL));

  /* Attach the GPT interrupt vector */
  irq_attach(RA_IRQ_SYSTICK_GPT, (xcpt_t)ra_systick_isr, NULL);

  /* Enable GPT interrupt */
  up_enable_irq(RA_IRQ_SYSTICK_GPT);

  /* Re-enable write protection - not supported */
  //regval = GPT_GTWP_PRKEY;  /* Remove WP bit but keep key */
  //putreg32(regval, RA_GPT_SYSTICK_GTWP);

  /* Start GPT timer */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTR);

  tmrinfo("GPT3 timer configured: reload=0x%08x, clock=%u Hz, rate=%u Hz\n",
          SYSTICK_RELOAD, RA_TIMER_CLOCK, CLK_TCK);
}
#else
/* Standard ARM Cortex-M85 SysTick configuration */
void up_timer_initialize(void)
{
  uint32_t regval;

  /* Disable SysTick during setup */
  putreg32(0, NVIC_SYSTICK_CTRL);

  /* Clear current value register */
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Set reload register value for desired tick rate */
  /* SYSTICK_RELOAD should be calculated as (SYSTICK_CLOCK / CLK_TCK) - 1 */
  DEBUGASSERT(SYSTICK_RELOAD > 0 && SYSTICK_RELOAD <= SYSTICK_MAX);
  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Attach the SysTick interrupt handler */
  /* SysTick uses a fixed exception number (-1) which maps to a specific IRQ */
  irq_attach(RA_IRQ_SYSTICK, (xcpt_t)ra_systick_isr, NULL);

  /* Configure and enable SysTick:
   * - ENABLE: Enable the counter
   * - TICKINT: Enable SysTick exception request
   * - CLKSOURCE: Use processor clock as source (set to 1)
   *   If CLKSOURCE = 0, uses external reference clock
   *   If CLKSOURCE = 1, uses processor clock
   */
  regval = NVIC_SYSTICK_CTRL_ENABLE | NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);

  /* SysTick is now configured and will generate periodic interrupts
   * at the rate specified by CLK_TCK (typically 100Hz for NuttX)
   */

  tmrinfo("SysTick configured: reload=0x%08x, clock=%u Hz, rate=%u Hz\n",
          SYSTICK_RELOAD, SYSTICK_CLOCK, CLK_TCK);
}
#endif

/****************************************************************************
 * Function:  up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   up_timer_initialize() was called).  This function is functionally
 *   equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

#ifdef CONFIG_RA_SYSTICK_GPT
int up_timer_gettime(struct timespec *ts)
{
  uint64_t usecs;
  uint32_t period;
  uint32_t current;
  uint32_t elapsed;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL);

  /* Get the period and current counter values and the tick count, being
   * careful that we get a coherent set of values.
   */

  flags = enter_critical_section();

  /* Get GPT registers - counter counts up in our configuration */
  period  = getreg32(RA_GPT_SYSTICK_GTPR) + 1;
  current = getreg32(RA_GPT_SYSTICK_GTCNT);

  /* Get the number of full ticks elapsed */
  usecs = clock_systime_ticks() * USEC_PER_TICK;

  /* Add the partial tick time - current counts from 0 to period */
  elapsed = current;
  usecs  += (elapsed * USEC_PER_TICK) / period;

  leave_critical_section(flags);

  /* Convert to timespec */
  ts->tv_sec  = usecs / USEC_PER_SEC;
  ts->tv_nsec = (usecs % USEC_PER_SEC) * NSEC_PER_USEC;

  return OK;
}
#else
int up_timer_gettime(struct timespec *ts)
{
  uint64_t usecs;
  uint32_t reload;
  uint32_t current;
  uint32_t elapsed;
  irqstate_t flags;

  DEBUGASSERT(ts != NULL);

  /* Get the reload and current counter values and the tick count, being
   * careful that we get a coherent set of values.
   */

  flags = enter_critical_section();

  /* Get SysTick registers */
  reload  = getreg32(NVIC_SYSTICK_RELOAD) + 1;
  current = getreg32(NVIC_SYSTICK_CURRENT);

  /* Get the number of full ticks elapsed */
  usecs = clock_systime_ticks() * USEC_PER_TICK;

  /* Add the partial tick time */
  elapsed = reload - current;
  usecs  += (elapsed * USEC_PER_TICK) / reload;

  leave_critical_section(flags);

  /* Convert to timespec */
  ts->tv_sec  = usecs / USEC_PER_SEC;
  ts->tv_nsec = (usecs % USEC_PER_SEC) * NSEC_PER_USEC;

  return OK;
}
#endif

/****************************************************************************
 * Function:  up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_timer_expiration() will not be called unless the timer is
 *   restarted with up_timer_start().
 *
 *   If, as a race condition, the timer has already expired when this
 *   function is called, then that pending interrupt must be cleared so
 *   that up_timer_start() and the remaining time of zero should be
 *   returned.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 * Input Parameters:
 *   ts - Location to return the remaining time.  Zero should be returned
 *        if the timer is not active.  ts may be zero in which case the
 *        time remaining is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_RA_SYSTICK_GPT
int up_timer_cancel(struct timespec *ts)
{
  uint32_t period;
  uint32_t current;
  uint32_t remaining;

  if (ts != NULL)
    {
      /* Get the current GPT state */
      period  = getreg32(RA_GPT_SYSTICK_GTPR) + 1;
      current = getreg32(RA_GPT_SYSTICK_GTCNT);

      /* Calculate remaining time in this tick period */
      remaining = period - current;

      /* Convert to microseconds */
      uint32_t remaining_usecs = (remaining * USEC_PER_TICK) / period;

      /* Convert to timespec */
      ts->tv_sec  = 0;
      ts->tv_nsec = remaining_usecs * NSEC_PER_USEC;
    }

  /* Stop GPT timer */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTP);

  /* Clear counter */
  putreg32(0, RA_GPT_SYSTICK_GTCNT);

  /* Disable overflow interrupt */
  putreg32(0, RA_GPT_SYSTICK_GTINTAD);

  return OK;
}
#else
int up_timer_cancel(struct timespec *ts)
{
  uint32_t reload;
  uint32_t current;
  uint32_t remaining;

  if (ts != NULL)
    {
      /* Get the current SysTick state */
      reload  = getreg32(NVIC_SYSTICK_RELOAD) + 1;
      current = getreg32(NVIC_SYSTICK_CURRENT);

      /* Calculate remaining time in this tick period */
      remaining = current;

      /* Convert to microseconds */
      uint32_t remaining_usecs = (remaining * USEC_PER_TICK) / reload;

      /* Convert to timespec */
      ts->tv_sec  = 0;
      ts->tv_nsec = remaining_usecs * NSEC_PER_USEC;
    }

  /* Disable SysTick temporarily */
  putreg32(0, NVIC_SYSTICK_CTRL);

  /* Clear current counter */
  putreg32(0, NVIC_SYSTICK_CURRENT);

  return OK;
}
#endif

/****************************************************************************
 * Function:  up_timer_start
 *
 * Description:
 *   Start the interval timer.  nxsched_timer_expiration() will be called at
 *   the completion of the timeout (unless up_timer_cancel is called to stop
 *   the timing).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until nxsched_timer_expiration() is
 *        called.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_RA_SYSTICK_GPT
int up_timer_start(const struct timespec *ts)
{
  uint64_t usecs;
  uint32_t period_val;
  uint32_t regval;

  DEBUGASSERT(ts != NULL);

  /* Convert timespec to microseconds */
  usecs = (uint64_t)ts->tv_sec * USEC_PER_SEC +
          (uint64_t)ts->tv_nsec / NSEC_PER_USEC;

  /* Convert microseconds to GPT counts */
  period_val = (usecs * RA_TIMER_CLOCK) / USEC_PER_SEC;

  /* Ensure period value is within valid range */
  if (period_val == 0)
    {
      period_val = 1;
    }
  else if (period_val > 0xFFFFFFFF)
    {
      period_val = 0xFFFFFFFF;
    }

  /* Stop timer during setup */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTP);

  /* Clear counter */
  putreg32(0, RA_GPT_SYSTICK_GTCNT);

  /* Set new period value */
  putreg32(period_val - 1, RA_GPT_SYSTICK_GTPR);

  /* Enable overflow interrupt */
  regval = GPT_GTINTAD_GTINTV;
  putreg32(regval, RA_GPT_SYSTICK_GTINTAD);

  /* Start timer */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTR);

  return OK;
}
#else
int up_timer_start(const struct timespec *ts)
{
  uint64_t usecs;
  uint32_t reload_val;
  uint32_t regval;

  DEBUGASSERT(ts != NULL);

  /* Convert timespec to microseconds */
  usecs = (uint64_t)ts->tv_sec * USEC_PER_SEC +
          (uint64_t)ts->tv_nsec / NSEC_PER_USEC;

  /* Convert microseconds to SysTick counts */
  reload_val = (usecs * SYSTICK_CLOCK) / USEC_PER_SEC;

  /* Ensure reload value is within valid range */
  if (reload_val == 0)
    {
      reload_val = 1;
    }
  else if (reload_val > SYSTICK_MAX)
    {
      reload_val = SYSTICK_MAX;
    }

  /* Disable SysTick during setup */
  putreg32(0, NVIC_SYSTICK_CTRL);

  /* Clear current value */
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Set new reload value */
  putreg32(reload_val - 1, NVIC_SYSTICK_RELOAD);

  /* Enable SysTick with interrupt and processor clock */
  regval = NVIC_SYSTICK_CTRL_ENABLE | NVIC_SYSTICK_CTRL_TICKINT | NVIC_SYSTICK_CTRL_CLKSOURCE;
  putreg32(regval, NVIC_SYSTICK_CTRL);

  return OK;
}
#endif
