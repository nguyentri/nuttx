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
#define RA_GPT_CHANNEL        (0)  /* Use GPT0 for system timer to match FSP example */

#if RA_GPT_CHANNEL == 0
#  define RA_MSTP_GPT_SYSTICK       RA_MSTP_GPT0
#elif RA_GPT_CHANNEL == 1
#  define RA_MSTP_GPT_SYSTICK       RA_MSTP_GPT1
#elif RA_GPT_CHANNEL == 2
#  define RA_MSTP_GPT_SYSTICK       RA_MSTP_GPT2
#elif RA_GPT_CHANNEL == 3
#  define RA_MSTP_GPT_SYSTICK       RA_MSTP_GPT3
#elif RA_GPT_CHANNEL == 4
#  define RA_MSTP_GPT_SYSTICK       RA_MSTP_GPT4
#elif RA_GPT_CHANNEL == 5
#  define RA_MSTP_GPT_SYSTICK       RA_MSTP_GPT5
#else
#  error "Unsupported GPT channel for system timer"
#endif
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
#define RA_GPT_SYSTICK_GTIOR          RA_GPT_GTIOR(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTPBR          RA_GPT_GTPBR(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCCRA         RA_GPT_GTCCRA(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCCRB         RA_GPT_GTCCRB(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCCRC         RA_GPT_GTCCRC(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTCCRE         RA_GPT_GTCCRE(RA_GPT_CHANNEL)
#define RA_GPT_SYSTICK_GTSSR          RA_GPT_REG(RA_GPT_CHANNEL, RA_GPT_GTSSR_OFFSET)
#define RA_GPT_SYSTICK_GTPSR          RA_GPT_REG(RA_GPT_CHANNEL, RA_GPT_GTPSR_OFFSET)
#define RA_GPT_SYSTICK_GTCSR          RA_GPT_REG(RA_GPT_CHANNEL, RA_GPT_GTCSR_OFFSET)
#define RA_TIMER_CLOCK                (RA_PCLKD_FREQUENCY)
#define SYSTICK_RELOAD                ((RA_TIMER_CLOCK / CLK_TCK) - 1)
#if SYSTICK_RELOAD > 0xFFFFFFFF
#  error GPT timer reload value exceeds 32-bit range
#endif
#else
#  include "ra_clock.h"
#  define SYSTICK_CLOCK     RA_CPUCLK_FREQUENCY
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
int ra_timer_arch_isr(int irq, void *context, void *arg)
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

static int ra_systick_isr(int irq, void *context, void *arg)
{
  uint32_t status;

  /* Read GPT status flags */
  status = getreg32(RA_GPT_SYSTICK_GTST);

  /* Check if overflow interrupt occurred - this is what we want for system timer */
  if (status & GPT_GTST_TCFPO)
    {
      /* Clear overflow flag by writing 0 to it - FSP style */
      putreg32(status & ~GPT_GTST_TCFPO, RA_GPT_SYSTICK_GTST);

      /* Process timer interrupt */
      nxsched_process_timer();
    }

  /* Clear any other potential flags to prevent spurious interrupts */
  if (status & (GPT_GTST_TCFA | GPT_GTST_TCFB | GPT_GTST_TCFC | GPT_GTST_TCFD |
                GPT_GTST_TCFE | GPT_GTST_TCFF | GPT_GTST_TCFPU))
    {
      /* Clear all other flags */
      putreg32(status & ~(GPT_GTST_TCFA | GPT_GTST_TCFB | GPT_GTST_TCFC | GPT_GTST_TCFD |
                          GPT_GTST_TCFE | GPT_GTST_TCFF | GPT_GTST_TCFPU),
               RA_GPT_SYSTICK_GTST);
    }

  return 0;
}
#else
/* ARM Cortex-M85 SysTick interrupt handler for NuttX system timer */
static int ra_systick_isr(int irq, void *context, void *arg)
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
  ra_mstp_start(RA_MSTP_GPT_SYSTICK);

  /* Disable write protection to configure GPT registers */
  regval = GPT_GTWP_PRKEY;  /* Write protection key without WP bit */
  putreg32(regval, RA_GPT_SYSTICK_GTWP);

  /* Stop GPT channel if running */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTP);

  /* Clear GPT counter */
  putreg32(0, RA_GPT_SYSTICK_GTCNT);

  /* Set count direction to up-counting for periodic mode */
  /* Set count direction (count up for normal operation) */
  regval = GPT_GTUDDTYC_UD;  /* Count up */
  putreg32(regval, RA_GPT_GTUDDTYC(RA_GPT_CHANNEL));

  /* Set period register for desired interrupt frequency */
  putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTPR);

  /* Set period buffer register (GTPBR) - FSP requirement for double buffering */
  putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTPBR);

  /* Configure compare match registers to zero - not used */
  putreg32(0, RA_GPT_SYSTICK_GTCCRA);
  putreg32(0, RA_GPT_SYSTICK_GTCCRB);
  putreg32(0, RA_GPT_SYSTICK_GTCCRC);
  putreg32(0, RA_GPT_SYSTICK_GTCCRE);

  /* Set GTCCRA to the full period for system tick timing */
  //putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTCCRA);
  //putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTCCRB);
  //putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTCCRC);
  //putreg32(SYSTICK_RELOAD, RA_GPT_SYSTICK_GTCCRE);

  /* Configure GPT control register for simple timer mode */
  regval = GPT_GTCR_MD_SAW_WAVE_UP |      /* Simple up-counting timer mode */
           GPT_GTCR_TPCS_PCLKD_1;         /* Use PCLKD as clock source */
  putreg32(regval, RA_GPT_SYSTICK_GTCR);

  /* Configure start/stop/clear sources to match FSP */
  regval = (1 << 31);  /* CSTRT: Software start enable */
  putreg32(regval, RA_GPT_SYSTICK_GTSSR);

  regval = (1 << 31);  /* CSTOP: Software stop enable */
  putreg32(regval, RA_GPT_SYSTICK_GTPSR);

  regval = (1 << 31);  /* CCLR: Software clear enable */
  putreg32(regval, RA_GPT_SYSTICK_GTCSR);

  /* Enable overflow interrupt for system timer */
  //regval = GPT_GTINTAD_GTINTV;            /* Overflow interrupt */
  //putreg32(regval, RA_GPT_SYSTICK_GTINTAD);

  /* Configure GTINTAD register - FSP style (no direct interrupt enables) */
  /* Clear GTINTAD completely - overflow interrupt is routed via ICU events */
  putreg32(0, RA_GPT_SYSTICK_GTINTAD);

  /* Clear any residual interrupt flags again after configuration */
  putreg32(0, RA_GPT_SYSTICK_GTST);

  /* Attach the timer interrupt handler through ICU */
  ra_icu_attach(RA_EL_GPT0_COUNTER_OVERFLOW, ra_systick_isr, NULL);

  /* Start GPT timer - this must be done BEFORE re-enabling write protection */
  putreg32((1 << RA_GPT_CHANNEL), RA_GPT_SYSTICK_GTSTR);

  /* Re-enable write protection after configuration */
  regval = GPT_GTWP_PRKEY | GPT_GTWP_WP;
  putreg32(regval, RA_GPT_SYSTICK_GTWP);
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
   * at the rate specified by CLK_TCK (typically 1000Hz for NuttX)
   */
  _info("SysTick configured: CLOCK = %u MHz, Tick rate = %u Hz (reload = %u)\n",
        SYSTICK_CLOCK / 1000000, CLK_TCK, SYSTICK_RELOAD + 1);
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
