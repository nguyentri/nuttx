/****************************************************************************
 * arch/arm/src/ra8/ra_gpt.c
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/timers/timer.h>
#include <nuttx/clock.h>
#include <nuttx/timers/arch_timer.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "nvic.h"
#include "hardware/ra_gpt.h"
#include "ra_gpt.h"
#include "ra_mstp.h"
#include "ra_clock.h"
#include "ra_gpio.h"
#include <syslog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default PCLKD frequency - typically 120MHz for RA8E1 */

#ifndef CONFIG_RA_PCLKD_FREQUENCY
#  define CONFIG_RA_PCLKD_FREQUENCY    120000000
#endif

/* Buffer enable mask for PWM mode */
#define GPT_GTBER_PWM_ENABLE            (GPT_GTBER_CCRA | GPT_GTBER_CCRB | GPT_GTBER_PR)

/* Prescaler values */
#define GPT_PRESCALER_1                 1
#define GPT_PRESCALER_4                 4
#define GPT_PRESCALER_16                16
#define GPT_PRESCALER_64                64
#define GPT_PRESCALER_256               256
#define GPT_PRESCALER_1024              1024

#ifdef CONFIG_RA_GPT
/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GPT channel configuration structure */
struct ra_gpt_channel_config_s
{
  uint32_t base;                   /* GPT peripheral base address */
  ra_mstp_module_t mstp;           /* Module stop control bit */
  uint32_t pclkd_freq;             /* PCLKD frequency */
  uint32_t max_period;            /* Maximum period in timer counts, 16-bit timer: 65535 and 32-bit timer: 4294967295 */
  uint32_t channel;                /* GPT channel (0-13) */
  uint32_t elc;                    /* ELC event input number */
  gpio_pinset_t pin_a;             /* GTIOCA pin configuration */
  gpio_pinset_t pin_b;             /* GTIOCB pin configuration */
};

/* GPT device state structure */
struct ra_gpt_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  const struct timer_ops_s *timer_ops; /* Timer operations */
  const struct ra_gpt_channel_config_s *config; /* GPT configuration */
  uint32_t frequency;             /* Current frequency */
  uint32_t period;                /* Period in timer counts */
  uint32_t prescaler;             /* Current prescaler setting */
  /* Current Duty cycle values stored as timer ticks for compare A/B.
   * These reflect the last started configuration and are used to answer
   * PWMIOC_GETCHARACTERISTICS and for logging. duty_b is only present
   * when multi-channel support is enabled.
   */
  uint32_t duty_a;                /* Current Duty cycle for channel A in timer counts */
#ifdef CONFIG_PWM_MULTICHAN
  uint32_t duty_b;                /* Current Duty cycle for channel B in timer counts */
#endif
  int       irq;                  /* Timer interrupt slot IRQ number assigned in the runtime */
  uint8_t mode;                   /* GPT mode (PWM/Timer) */
  bool started;                   /* True: Started */
  bool pwm_mode;                  /* True: PWM mode, False: Timer mode */
  void (*callback)(void *arg);    /* Timer callback function */
  void *arg;                      /* Timer callback argument */
#ifdef CONFIG_PWM_MULTICHAN
  uint8_t  nchannels;             /* Number of channels */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access helpers */
static inline uint32_t gpt_getreg(struct ra_gpt_s *priv, int offset);
static inline void gpt_putreg(struct ra_gpt_s *priv, int offset, uint32_t value);

/* Low-level GPT operations */
static int gpt_configure(struct ra_gpt_s *priv);
static uint32_t gpt_calculate_prescaler(uint32_t frequency, uint32_t pclkd);
static void gpt_dumpregs(struct ra_gpt_s *priv, const char *msg);
static void gpt_log_channel(uint8_t ch,
                            uint32_t freq_hz,
                            uint32_t prescaler,
                            uint32_t pclkd,
                            uint32_t reload_ticks,
                            uint32_t duty_ticks);

/* PWM driver methods */
static int gpt_setup(struct pwm_lowerhalf_s *dev);
static int gpt_shutdown(struct pwm_lowerhalf_s *dev);
static int gpt_start(struct pwm_lowerhalf_s *dev, const struct pwm_info_s *info);
static int gpt_stop(struct pwm_lowerhalf_s *dev);
static int gpt_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */
static const struct pwm_ops_s g_gpt_ops =
{
  .setup      = gpt_setup,
  .shutdown   = gpt_shutdown,
  .start      = gpt_start,
  .stop       = gpt_stop,
  .ioctl      = gpt_ioctl,
};

/* GPT device configurations */
static const struct ra_gpt_channel_config_s g_gpt_configs[] =
{
#ifdef CONFIG_RA_GPT0
  {
    .base       = R_GPT0_BASE,
    .mstp       = RA_MSTP_GPT0,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .max_period = UINT32_MAX, /* 32-bit timer */
    .channel    = 0,
    .elc        = RA_ELC_GPT0_CAPTURE_COMPARE_A,  /* GPT0 capture/compare A IRQ, typically not used when control ECS */
    .pin_a      = GPIO_GPT0_A,  /* Configure based on board */
    .pin_b      = {0},  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT1 // not configured
  {
    .base       = R_GPT1_BASE,
    .mstp       = RA_MSTP_GPT1,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .max_period = UINT32_MAX, /* 32-bit timer */
    .channel    = 1,
    .elc        = RA_ELC_GPT1_COUNTER_OVERFLOW,  /* GPT1 overflow IRQ */
    .pin_a      = {0},  /* Configure based on board */
    .pin_b      = {0},  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT2
  {
    .base       = R_GPT2_BASE,
    .mstp       = RA_MSTP_GPT2,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .max_period = UINT32_MAX, /* 32-bit timer */
    .channel    = 2,
    .elc        = RA_ELC_GPT2_CAPTURE_COMPARE_A,  /* GPT2 capture/compare A IRQ */
    .pin_a      = GPIO_GPT2_A,  /* Configure based on board */
    .pin_b      = {0},  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT3
  {
    .base       = R_GPT3_BASE,
    .mstp       = RA_MSTP_GPT3,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .max_period = UINT32_MAX, /* 32-bit timer */
    .channel    = 3,
    .elc        = RA_ELC_GPT3_CAPTURE_COMPARE_A,  /* GPT3 capture/compare A IRQ */
    .pin_a      = GPIO_GPT3_A,  /* Configure based on board */
    .pin_b      = {0},  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT4
  {
    .base       = R_GPT4_BASE,
    .mstp       = RA_MSTP_GPT4,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .max_period = UINT32_MAX, /* 32-bit timer */
    .channel    = 4,
    .elc        = RA_ELC_GPT4_CAPTURE_COMPARE_A,  /* GPT4 capture/compare A IRQ */
    .pin_a      = GPIO_GPT4_A,  /* Configure based on board */
    .pin_b      = {0},  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT5
  {
    .base       = R_GPT5_BASE,
    .mstp       = RA_MSTP_GPT5,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .max_period = UINT32_MAX, /* 32-bit timer */
    .channel    = 5,
    .elc        = RA_ELC_GPT5_CAPTURE_COMPARE_A,  /* GPT5 capture/compare A IRQ */
    .pin_a      = GPIO_GPT5_A,  /* Configure based on board */
    .pin_b      = {0},  /* Configure based on board */
  },
#endif
/* Add more channels as needed */
};

#define NGPT_CONFIGS (sizeof(g_gpt_configs) / sizeof(struct ra_gpt_channel_config_s))

/* GPT device instances */
static struct ra_gpt_s g_gpt_devs[NGPT_CONFIGS];

#define NGPT_DEVS (sizeof(g_gpt_devs) / sizeof(struct ra_gpt_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpt_getreg
 *
 * Description:
 *   Read the value of a GPT timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the GPT structure
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The value of the register
 *
 ****************************************************************************/

static inline uint32_t gpt_getreg(struct ra_gpt_s *priv, int offset)
{
  return getreg32(priv->config->base + offset);
}

/****************************************************************************
 * Name: gpt_putreg
 *
 * Description:
 *   Write a value to a GPT timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the GPT structure
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void gpt_putreg(struct ra_gpt_s *priv, int offset,
                              uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

/****************************************************************************
 * Name: gpt_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the GPT structure
 *   msg  - Message to print before the register dump
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gpt_dumpregs(struct ra_gpt_s *priv, const char *msg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  pwminfo("%s:\n", msg);
  pwminfo("  GTCR:    %08x  GTPR:    %08x  GTCNT:   %08x\n",
          gpt_getreg(priv, RA_GPT_GTCR_OFFSET),
          gpt_getreg(priv, RA_GPT_GTPR_OFFSET),
          gpt_getreg(priv, RA_GPT_GTCNT_OFFSET));
  pwminfo("  GTCCRA:  %08x  GTCCRB:  %08x  GTIOR:   %08x\n",
          gpt_getreg(priv, RA_GPT_GTCCRA_OFFSET),
          gpt_getreg(priv, RA_GPT_GTCCRB_OFFSET),
          gpt_getreg(priv, RA_GPT_GTIOR_OFFSET));
  pwminfo("  GTINTAD: %08x  GTST:    %08x\n",
          gpt_getreg(priv, RA_GPT_GTINTAD_OFFSET),
          gpt_getreg(priv, RA_GPT_GTST_OFFSET));
#endif
}

/*
 * Log helper that prints channel timing info to syslog and low-level debug.
 */
#ifdef CONFIG_DEBUG_PWM_INFO
static void gpt_log_channel(uint8_t ch,
                            uint32_t freq_hz,
                            uint32_t prescaler,
                            uint32_t pclkd,
                            uint32_t reload_ticks,
                            uint32_t duty_ticks)
{
  uint32_t period_us = 0;
  uint32_t duty_us = 0;
  uint32_t duty_pct = 0;
  uint64_t timer_tick_freq = 0;
  const uint32_t prescaler_divs[] = {1, 4, 16, 64, 256, 1024};

  /* Validate prescaler index and pclkd */
  if (prescaler < (sizeof(prescaler_divs) / sizeof(prescaler_divs[0])) && pclkd > 0)
    {
      timer_tick_freq = (uint64_t)pclkd / prescaler_divs[prescaler];
    }

  /* If the reload register contains the default/unprogrammed value
   * (UINT32_MAX for 32-bit, 0xFFFF for 16-bit) then this GPT has not been
   * programmed yet; avoid computing and printing misleading large us values.
   */
  if (reload_ticks == UINT32_MAX || reload_ticks == 0xFFFF)
    {
      syslog(LOG_INFO,
             "GPT ch%u: unprogrammed reload=0x%" PRIx32 "\n",
             (unsigned)ch, reload_ticks);
      pwminfo("GPT ch%u: unprogrammed\n", (unsigned)ch);
      return;
    }

  if (timer_tick_freq > 0 && reload_ticks > 0)
    {
      /* reload_ticks are timer ticks at prescaled timer clock. Convert to us
       * Use integer rounding: add half of divisor before divide.
       */
      period_us = (uint32_t)((((uint64_t)reload_ticks * 1000000ULL) + (timer_tick_freq >> 1)) / timer_tick_freq);
      duty_us   = (uint32_t)((((uint64_t)duty_ticks * 1000000ULL) + (timer_tick_freq >> 1)) / timer_tick_freq);

      /* Percent with rounding: (duty*100 + reload/2)/reload */
      duty_pct  = (uint32_t)((((uint64_t)duty_ticks * 100ULL) + (reload_ticks >> 1)) / reload_ticks);
    }

  syslog(LOG_INFO,
    "GPT ch%u: freq=%" PRIu32 "Hz presc=%" PRIu32 " reload=%" PRIu32 " ticks period~%" PRIu32 "us duty=%" PRIu32 " ticks ~%" PRIu32 "us (%" PRIu32 "%%)\n",
    (unsigned)ch, freq_hz, prescaler, reload_ticks, period_us, duty_ticks, duty_us, duty_pct);

  /* Use pwminfo for low-level PWM info logging (lldbg may be unavailable
   * in some build configs). */
  pwminfo("GPT ch%u: period~%" PRIu32 "us duty~%" PRIu32 "us (%" PRIu32 "%%)\n",
     (unsigned)ch, period_us, duty_us, duty_pct);
}
#else
#  define gpt_log_channel(ch,freq_hz,prescaler,pclkd,reload_ticks,duty_ticks)
#endif

/****************************************************************************
 * Name: gpt_calculate_prescaler
 *
 * Description:
 *   Calculate the appropriate prescaler for the given frequency.
 *
 * Input Parameters:
 *   frequency - The desired PWM frequency
 *   pclkd     - The PCLKD frequency
 *
 * Returned Value:
 *   The prescaler value (0-5) or -1 on error
 *
 ****************************************************************************/

static uint32_t gpt_calculate_prescaler(uint32_t frequency, uint32_t pclkd)
{
  uint32_t prescaler_divs[] = {1, 4, 16, 64, 256, 1024};
  uint32_t i;
  uint32_t timer_freq;
  uint32_t period;

  for (i = 0; i < sizeof(prescaler_divs) / sizeof(prescaler_divs[0]); i++)
    {
      timer_freq = pclkd / prescaler_divs[i];

      /* Compute period rounded to nearest tick: period = round(timer_freq / frequency)
       * Use 64-bit math to avoid overflow.
       */
  period = (uint32_t)((((uint64_t)timer_freq) + ((uint64_t)frequency >> 1)) / (uint64_t)frequency);

      /* Check if period fits in 32-bit counter and is reasonable */
      if (period > 1 && period <= UINT32_MAX)
        {
          return i;
        }
    }

  return UINT32_MAX; /* Error */
}

/****************************************************************************
 * Name: gpt_configure
 *
 * Description:
 *   Configure the GPT timer for PWM operation.
 *
 * Input Parameters:
 *   priv - A reference to the GPT structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gpt_configure(struct ra_gpt_s *priv)
{
  uint32_t regval;

  pwminfo("Configuring GPT%d\n", priv->config->channel);

  /* Perform the multi-register configuration atomically to avoid races */
  irqstate_t flags = enter_critical_section();

  /* Disable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET, GPT_GTWP_PRKEY);

  /* Stop the timer if it's running */
  regval = gpt_getreg(priv, RA_GPT_GTCR_OFFSET);
  regval &= ~GPT_GTCR_CST;
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Configure timer for saw-wave PWM mode (up-counting) */
  regval = GPT_GTCR_MD_SAW_WAVE_UP | GPT_GTCR_TPCS_PCLKD_1;
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Configure I/O pins for PWM output - Start with low output */
  regval = GPT_GTIOR_GTIOA_INITIAL_LOW | GPT_GTIOR_GTIOB_INITIAL_LOW;
  gpt_putreg(priv, RA_GPT_GTIOR_OFFSET, regval);

  /* Initialize counter and period */
  gpt_putreg(priv, RA_GPT_GTCNT_OFFSET, 0);
  gpt_putreg(priv, RA_GPT_GTPR_OFFSET, 0xffff);

  /* Initialize compare registers */
  gpt_putreg(priv, RA_GPT_GTCCRA_OFFSET, 0);
  gpt_putreg(priv, RA_GPT_GTCCRB_OFFSET, 0);

  /* Clear all interrupt flags */
  regval = gpt_getreg(priv, RA_GPT_GTST_OFFSET);
  gpt_putreg(priv, RA_GPT_GTST_OFFSET, regval);

  /* Re-enable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET,
             GPT_GTWP_PRKEY | GPT_GTWP_WP | GPT_GTWP_CMNWP);

  leave_critical_section(flags);

  gpt_dumpregs(priv, "After configuration");

  /* Mark as PWM mode */
  priv->pwm_mode = true;

  /* Log channel state */
  gpt_log_channel((uint8_t)priv->config->channel,
                  priv->frequency,
                  priv->prescaler,
                  priv->config->pclkd_freq,
                  gpt_getreg(priv, RA_GPT_GTPR_OFFSET),
                  gpt_getreg(priv, RA_GPT_GTCCRA_OFFSET));

  return 0;
}

/****************************************************************************
 * Name: gpt_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gpt_setup(struct pwm_lowerhalf_s *dev)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)dev;

  pwminfo("GPT%d setup\n", priv->config->channel);

  return gpt_configure(priv);
}

/****************************************************************************
 * Name: gpt_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gpt_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)dev;
  uint32_t regval;

  pwminfo("GPT%d shutdown\n", priv->config->channel);

  /* Make shutdown sequence atomic */
  irqstate_t flags = enter_critical_section();

  /* Disable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET, GPT_GTWP_PRKEY);

  /* Stop the timer */
  regval = gpt_getreg(priv, RA_GPT_GTCR_OFFSET);
  regval &= ~GPT_GTCR_CST;
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Reset the timer to its default state */
  gpt_putreg(priv, RA_GPT_GTCNT_OFFSET, 0);
  gpt_putreg(priv, RA_GPT_GTCCRA_OFFSET, 0);
  gpt_putreg(priv, RA_GPT_GTCCRB_OFFSET, 0);
  gpt_putreg(priv, RA_GPT_GTIOR_OFFSET, 0);

  /* Re-enable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET,
             GPT_GTWP_PRKEY | GPT_GTWP_WP | GPT_GTWP_CMNWP);

  leave_critical_section(flags);

  priv->started = false;
  return 0;
}

/****************************************************************************
 * Name: gpt_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gpt_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)dev;
  uint32_t prescaler;
  uint32_t timer_freq;
  uint32_t period;
  uint32_t duty_a, duty_b;
  uint32_t regval;

#ifdef CONFIG_PWM_MULTICHAN
  pwminfo("GPT%d start: frequency=%lu (multichan)\n",
          priv->config->channel, info->frequency);
#else
  pwminfo("GPT%d start: frequency=%lu duty=%08lx\n",
          priv->config->channel, info->frequency, info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);

  /* Calculate the prescaler and period */

  prescaler = gpt_calculate_prescaler(info->frequency, priv->config->pclkd_freq);
  if (prescaler == UINT32_MAX)
    {
      pwmerr("ERROR: Cannot achieve frequency %lu\n", info->frequency);
      return -ERANGE;
    }

  /* Compute timer tick frequency using prescaler divisor table and round
   * the period (timer ticks) to the nearest integer to avoid systematic
   * truncation biases.
   */
  {
    const uint32_t prescaler_divs[] = {1, 4, 16, 64, 256, 1024};

    if (prescaler >= (sizeof(prescaler_divs) / sizeof(prescaler_divs[0])))
      {
        pwmerr("ERROR: invalid prescaler index %lu\n", prescaler);
        return -EINVAL;
      }

    timer_freq = priv->config->pclkd_freq / prescaler_divs[prescaler];

    /* period = round(timer_freq / frequency) -> (timer_freq + freq >> 1)/freq */
    period = (uint32_t)((((uint64_t)timer_freq) + ((uint64_t)info->frequency >> 1)) /
            (uint64_t)info->frequency);
  }

  /* Verify period fits in the channel's maximum period */
  if (period == 0 || period > priv->config->max_period)
    {
      pwmerr("ERROR: period %lu out of range for GPT%lu (max %lu)\n",
             period, priv->config->channel, priv->config->max_period);
      return -ERANGE;
    }

  pwminfo("prescaler=%lu, timer_freq=%lu, period=%lu\n",
          prescaler, timer_freq, period);

  /* Calculate duty cycle values */

#ifdef CONFIG_PWM_MULTICHAN
  /* Map per-channel requests to GTIOCA (A) and GTIOCB (B).
   * pwm.h channel numbers start at 1. We treat channel==1 -> A, ==2 -> B.
   * channel==0 indicates unused and negative channel indicates end.
   */
  duty_a = duty_b = 0;
  for (int i = 0; i < CONFIG_PWM_NCHANNELS; i++)
    {
      int8_t ch = info->channels[i].channel;
      if (ch == 0)
        {
          /* channel 0: not used */
          continue;
        }
      if (ch < 0)
        {
          /* negative channel indicates no more channels */
          break;
        }

      /* Convert ub16 duty to timer ticks with rounding: add 0.5 (0x8000) before shift */
      uint32_t chduty = (uint32_t)((((uint64_t)period * (uint64_t)info->channels[i].duty) + 0x8000ULL) >> 16);
      if (ch == 1)
        {
          duty_a = chduty;
        }
      else if (ch == 2)
        {
          duty_b = chduty;
        }
      else
        {
          pwmerr("GPT%u: Unsupported channel number %d\n",
                 priv->config->channel, ch);
          return -EINVAL;
        }
    }

  /* If only one channel was provided, duplicate to the other compare */
  if (duty_b == 0 && duty_a != 0)
    {
      duty_b = duty_a;
    }
#else
  /* Convert ub16 duty to timer ticks with rounding */
  duty_a = (uint32_t)((((uint64_t)period * (uint64_t)info->duty) + 0x8000ULL) >> 16);
  duty_b = duty_a;
#endif


  /* Make the start sequence atomic */
  irqstate_t flags = enter_critical_section();

  /* Disable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET, GPT_GTWP_PRKEY);

  /* Stop the timer */
  regval = gpt_getreg(priv, RA_GPT_GTCR_OFFSET);
  regval &= ~GPT_GTCR_CST;
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Configure the prescaler */
  regval = GPT_GTCR_MD_SAW_WAVE_UP | (prescaler << GPT_GTCR_TPCS_SHIFT);
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Set the period */
  gpt_putreg(priv, RA_GPT_GTPR_OFFSET, period - 1);

  /* Set the duty cycles */
  gpt_putreg(priv, RA_GPT_GTCCRA_OFFSET, duty_a);
  gpt_putreg(priv, RA_GPT_GTCCRB_OFFSET, duty_b);

  /* Reset the counter */
  gpt_putreg(priv, RA_GPT_GTCNT_OFFSET, 0);

  /* Configure I/O pins for PWM output */
  regval = GPT_GTIOR_GTIOA_INITIAL_LOW | GPT_GTIOR_GTIOB_INITIAL_LOW;
  gpt_putreg(priv, RA_GPT_GTIOR_OFFSET, regval);

  /* Start the timer */
  regval = gpt_getreg(priv, RA_GPT_GTCR_OFFSET);
  regval |= GPT_GTCR_CST;
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Re-enable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET,
             GPT_GTWP_PRKEY | GPT_GTWP_WP | GPT_GTWP_CMNWP);

  leave_critical_section(flags);

  gpt_dumpregs(priv, "After start");

  priv->started = true;
  priv->frequency = info->frequency;
  priv->period = period;
  priv->prescaler = prescaler;
  priv->duty_a = duty_a;
#ifdef CONFIG_PWM_MULTICHAN
  priv->duty_b = duty_b;
#endif

  /* Log the channel timing after start */
  gpt_log_channel((uint8_t)priv->config->channel,
                  info->frequency,
                  prescaler,
                  priv->config->pclkd_freq,
                  period,
                  duty_a);

  return 0;
}

/****************************************************************************
 * Name: gpt_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gpt_stop(struct pwm_lowerhalf_s *dev)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)dev;
  uint32_t regval;

  pwminfo("GPT%d stop\n", priv->config->channel);


  irqstate_t flags = enter_critical_section();

  /* Disable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET, GPT_GTWP_PRKEY);

  /* Stop the timer */
  regval = gpt_getreg(priv, RA_GPT_GTCR_OFFSET);
  regval &= ~GPT_GTCR_CST;
  gpt_putreg(priv, RA_GPT_GTCR_OFFSET, regval);

  /* Disable PWM outputs */
  gpt_putreg(priv, RA_GPT_GTIOR_OFFSET, 0);

  /* Re-enable write protection */
  gpt_putreg(priv, RA_GPT_GTWP_OFFSET,
             GPT_GTWP_PRKEY | GPT_GTWP_WP | GPT_GTWP_CMNWP);

  leave_critical_section(flags);

  gpt_dumpregs(priv, "After stop");

  priv->started = false;
  return 0;
}

/****************************************************************************
 * Name: gpt_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int gpt_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  struct ra_gpt_s *priv = (struct ra_gpt_s *)dev;
  int ret = 0;

  pwminfo("GPT%d ioctl: cmd=%d arg=%08lx\n", priv->config->channel, cmd, arg);

  switch (cmd)
    {
      case PWMIOC_GETCHARACTERISTICS:
        {
          /* Populate a pwm_info_s with the current settings */
          struct pwm_info_s info;
          memset(&info, 0, sizeof(info));

          info.frequency = priv->frequency;
#ifdef CONFIG_PWM_MULTICHAN
          /* Fill channels array: channel numbers 1->A, 2->B */
          info.channels[0].channel = 1;
          /* Convert ticks to ub16 (16.16) with rounding: add half period before divide */
          info.channels[0].duty = (ub16_t)((priv->period == 0) ? 0 :
                                    (ub16_t)((((uint64_t)priv->duty_a << 16) + (priv->period >> 1)) / priv->period));
          info.channels[1].channel = 2;
          info.channels[1].duty = (ub16_t)((priv->period == 0) ? 0 :
                                    (ub16_t)((((uint64_t)priv->duty_b << 16) + (priv->period >> 1)) / priv->period));
#else
          /* Convert ticks to ub16 (16.16) with rounding */
          info.duty = (ub16_t)((priv->period == 0) ? 0 :
                                (ub16_t)((((uint64_t)priv->duty_a << 16) + (priv->period >> 1)) / priv->period));
#endif
          /* Copy into caller-provided buffer (arg is a pointer in kernel space)
           * Since ioctl() is called from kernel context, simple assignment is OK.
           */
          struct pwm_info_s *user = (struct pwm_info_s *)((FAR void *)arg);
          if (user == NULL)
            {
              ret = -EFAULT;
            }
          else
            {
              memcpy(user, &info, sizeof(info));
              ret = 0;
            }
        }
        break;

      /* Add any custom ioctl commands here */

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpt_initialize
 *
 * Description:
 *   Initialize one GPT timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the timer channel.
 *
 * Returned Value:
 *   On success, a pointer to the RA8 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ra_gpt_initialize(int channel)
{
  struct ra_gpt_s *lower;
  int i;

  pwminfo("GPT%d PWM initialize\n", channel);

  /* Find the matching configuration */

  for (i = 0; i < NGPT_CONFIGS; i++)
    {
      if (g_gpt_configs[i].channel == channel)
        {
          lower = &g_gpt_devs[i];

          /* Initialize the device structure */
          memset(lower, 0, sizeof(struct ra_gpt_s));

          lower->ops = &g_gpt_ops;
          lower->config = &g_gpt_configs[i];
          lower->pwm_mode = true;
          lower->started = false;

          /* Configure GPIO pins for PWM output */
          if( lower->config->pin_a.cfg != 0 )
            {
              ra_configgpio(lower->config->pin_a);
            }
          if( lower->config->pin_b.cfg != 0 )
            {
              ra_configgpio(lower->config->pin_b);
            }

          /* Take the GPT out of module stop state */
          ra_mstp_start(lower->config->mstp);
#ifdef CONFIG_PWM_MULTICHAN
          lower->nchannels = 2;  /* GTIOCA and GTIOCB */
#endif
          break;
        }
    }

  if (i >= NGPT_CONFIGS)
    {
      pwmerr("ERROR: No such timer configured: %d\n", channel);
      return NULL;
    }

  /* Emit a log for the newly-initialized channel (registers may be default)
   * Use register reads for period/duty if available.
   */
  gpt_log_channel((uint8_t)lower->config->channel,
                  lower->frequency,
                  lower->prescaler,
                  lower->config->pclkd_freq,
                  gpt_getreg(lower, RA_GPT_GTPR_OFFSET),
                  gpt_getreg(lower, RA_GPT_GTCCRA_OFFSET));

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_RA_GPT */

/****************************************************************************
 * End of file
 ****************************************************************************/
