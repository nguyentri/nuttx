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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default PCLKD frequency - typically 120MHz for RA8E1 */

#ifndef CONFIG_RA_PCLKD_FREQUENCY
#  define CONFIG_RA_PCLKD_FREQUENCY    120000000
#endif

/* GPT Write Protection Key */
#define GPT_GTWP_PRKEY                  0xA500

/* Buffer enable mask for PWM mode */
#define GPT_GTBER_PWM_ENABLE            (GPT_GTBER_CCRA | GPT_GTBER_CCRB | GPT_GTBER_PR)

/* Prescaler values */
#define GPT_PRESCALER_1                 1
#define GPT_PRESCALER_4                 4
#define GPT_PRESCALER_16                16
#define GPT_PRESCALER_64                64
#define GPT_PRESCALER_256               256
#define GPT_PRESCALER_1024              1024

#ifdef CONFIG_RA_GPT_PWM
/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GPT channel configuration structure */
struct ra_gpt_config_s
{
  uint32_t base;                   /* GPT peripheral base address */
  uint32_t pclkd_freq;            /* PCLKD frequency */
  uint8_t  channel;                /* GPT channel (0-13) */
  uint8_t  irq;                    /* Timer interrupt IRQ number */
  uint32_t pin_a;                  /* GTIOCA pin configuration */
  uint32_t pin_b;                  /* GTIOCB pin configuration */
};

/* GPT device state structure */
struct ra_gpt_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  const struct timer_ops_s *timer_ops; /* Timer operations */
  const struct ra_gpt_config_s *config; /* GPT configuration */
  uint32_t frequency;             /* Current frequency */
  uint32_t period;                /* Period in timer counts */
  uint32_t prescaler;             /* Current prescaler setting */
  uint8_t mode;                   /* GPT mode (PWM/Timer) */
  bool started;                   /* True: Started */
  timer_callback_t callback;      /* Timer callback function */
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
static void gpt_disable_write_protection(int channel);
static void gpt_enable_write_protection(int channel);
static void gpt_dumpregs(struct ra_gpt_s *priv, const char *msg);

/* PWM driver methods */
static int gpt_pwm_setup(struct pwm_lowerhalf_s *dev);
static int gpt_pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int gpt_pwm_start(struct pwm_lowerhalf_s *dev, const struct pwm_info_s *info);
static int gpt_pwm_stop(struct pwm_lowerhalf_s *dev);
static int gpt_pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);

/* Timer driver methods */
static int gpt_timer_start(struct timer_lowerhalf_s *lower);
static int gpt_timer_stop(struct timer_lowerhalf_s *lower);
static int gpt_timer_getstatus(struct timer_lowerhalf_s *lower, struct timer_status_s *status);
static int gpt_timer_settimeout(struct timer_lowerhalf_s *lower, uint32_t timeout);
static void gpt_timer_setcallback(struct timer_lowerhalf_s *lower,
                                 tccb_t callback, void *arg);
static int gpt_timer_ioctl(struct timer_lowerhalf_s *lower, int cmd, unsigned long arg);

/* Interrupt handling */
static int gpt_interrupt_handler(int irq, void *context, void *arg);

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access helpers */

static inline uint32_t gpt_getreg(struct ra_gpt_s *priv, int offset);
static inline void gpt_putreg(struct ra_gpt_s *priv, int offset,
                              uint32_t value);

/* PWM driver methods */

static int gpt_setup(struct pwm_lowerhalf_s *dev);
static int gpt_shutdown(struct pwm_lowerhalf_s *dev);
static int gpt_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int gpt_stop(struct pwm_lowerhalf_s *dev);
static int gpt_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

/* Initialization helpers */

static int gpt_configure(struct ra_gpt_s *priv);
static uint32_t gpt_calculate_prescaler(uint32_t frequency, uint32_t pclkd);
static void gpt_dumpregs(struct ra_gpt_s *priv, const char *msg);


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
static const struct ra_gpt_config_s g_gpt_configs[] =
{
#ifdef CONFIG_RA_GPT0_PWM
  {
    .base       = R_GPT0_BASE,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .channel    = 0,
    .irq        = RA_IRQ_FIRST + 48,  /* GPT0 overflow IRQ */
    .pin_a      = 0,  /* Configure based on board */
    .pin_b      = 0,  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT1_PWM
  {
    .base       = R_GPT1_BASE,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .channel    = 1,
    .irq        = RA_IRQ_FIRST + 49,  /* GPT1 overflow IRQ */
    .pin_a      = 0,  /* Configure based on board */
    .pin_b      = 0,  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT2_PWM
  {
    .base       = R_GPT2_BASE,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .channel    = 2,
    .irq        = RA_IRQ_FIRST + 50,  /* GPT2 overflow IRQ */
    .pin_a      = 0,  /* Configure based on board */
    .pin_b      = 0,  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT3_PWM
  {
    .base       = R_GPT3_BASE,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .channel    = 3,
    .irq        = RA_IRQ_FIRST + 51,  /* GPT3 overflow IRQ */
    .pin_a      = 0,  /* Configure based on board */
    .pin_b      = 0,  /* Configure based on board */
  },
#endif
#ifdef CONFIG_RA_GPT4_PWM
  {
    .base       = R_GPT4_BASE,
    .pclkd_freq = CONFIG_RA_PCLKD_FREQUENCY,
    .channel    = 4,
    .irq        = RA_IRQ_FIRST + 52,  /* GPT4 overflow IRQ */
    .pin_a      = 0,  /* Configure based on board */
    .pin_b      = 0,  /* Configure based on board */
  },
#endif
/* Add more channels as needed */
};

#define NGPT_CONFIGS (sizeof(g_gpt_configs) / sizeof(struct ra_gpt_config_s))

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
      period = timer_freq / frequency;

      /* Check if period fits in 32-bit counter and is reasonable */

      if (period > 1 && period <= 0xffffffff)
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

  gpt_dumpregs(priv, "After configuration");

  /* Mark as PWM mode */
  priv->pwm_mode = true;

  return OK;
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

  priv->started = false;
  return OK;
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

  pwminfo("GPT%d start: frequency=%lu duty=%08lx\n",
          priv->config->channel, info->frequency, info->duty);

  DEBUGASSERT(info->frequency > 0);

  /* Calculate the prescaler and period */

  prescaler = gpt_calculate_prescaler(info->frequency, priv->config->pclkd_freq);
  if (prescaler == UINT32_MAX)
    {
      pwmerr("ERROR: Cannot achieve frequency %lu\n", info->frequency);
      return -ERANGE;
    }

  timer_freq = priv->config->pclkd_freq / (1 << (prescaler * 2));
  period = timer_freq / info->frequency;

  pwminfo("prescaler=%lu, timer_freq=%lu, period=%lu\n",
          prescaler, timer_freq, period);

  /* Calculate duty cycle values */

  duty_a = (period * info->duty) >> 16;

#ifdef CONFIG_PWM_MULTICHAN
  if (info->count > 1)
    {
      duty_b = (period * info->channels[1].duty) >> 16;
    }
  else
#endif
    {
      duty_b = duty_a;
    }

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

  gpt_dumpregs(priv, "After start");

  priv->started = true;
  priv->frequency = info->frequency;
  priv->period = period;
  priv->prescaler = prescaler;

  return OK;
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

  gpt_dumpregs(priv, "After stop");

  priv->started = false;
  return OK;
}

  gpt_dumpregs(priv, "After stop");

  priv->started = false;
  return OK;
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
  int ret = OK;

  pwminfo("GPT%d ioctl: cmd=%d arg=%08lx\n", priv->config->channel, cmd, arg);

  switch (cmd)
    {
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

  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_RA_GPT */

/****************************************************************************
 * End of file
 ****************************************************************************/
