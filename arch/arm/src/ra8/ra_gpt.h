/****************************************************************************
 * arch/arm/src/ra8/ra_gpt.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_GPT_H
#define __ARCH_ARM_SRC_RA8_RA_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include <nuttx/timers/timer.h>
#include "hardware/ra_gpt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of GPT channels */
#define RA_GPT_NCHANNELS               14

/* GPT modes */
#define RA_GPT_MODE_TIMER              0   /* Timer mode */
#define RA_GPT_MODE_PWM                1   /* PWM mode */
#define RA_GPT_MODE_INPUT_CAPTURE      2   /* Input capture mode */

/* GPT timer modes */
#define RA_GPT_TIMER_PERIODIC          0   /* Periodic timer */
#define RA_GPT_TIMER_ONE_SHOT          1   /* One-shot timer */

/* GPT PWM output modes */
#define RA_GPT_PWM_SINGLE_OUTPUT       0   /* Single output (GTIOCA only) */
#define RA_GPT_PWM_COMPLEMENTARY       1   /* Complementary output (GTIOCA + GTIOCB) */
#define RA_GPT_PWM_INDEPENDENT         2   /* Independent outputs */

/* GPT interrupt sources */
#define RA_GPT_INT_OVERFLOW            (1 << 0)
#define RA_GPT_INT_UNDERFLOW           (1 << 1)
#define RA_GPT_INT_COMPARE_A           (1 << 2)
#define RA_GPT_INT_COMPARE_B           (1 << 3)
#define RA_GPT_INT_COMPARE_C           (1 << 4)
#define RA_GPT_INT_COMPARE_D           (1 << 5)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPT timer configuration */
struct ra_gpt_timer_config_s
{
  uint8_t  channel;                /* GPT channel number */
  uint8_t  mode;                   /* Timer mode (periodic/one-shot) */
  uint32_t frequency;              /* Timer frequency in Hz */
  uint32_t timeout_us;             /* Timeout in microseconds (for one-shot) */
  void     (*callback)(void *arg); /* Timer callback function */
  void     *arg;                   /* Callback argument */
};

/* GPT PWM configuration */
struct ra_gpt_pwm_config_s
{
  uint8_t  channel;                /* GPT channel number */
  uint8_t  output_mode;            /* PWM output mode */
  uint32_t frequency;              /* PWM frequency in Hz */
  uint32_t duty_a;                 /* Duty cycle for GTIOCA (0-65535) */
  uint32_t duty_b;                 /* Duty cycle for GTIOCB (0-65535) */
  bool     enable_deadtime;        /* Enable dead time control */
  uint16_t deadtime_up;            /* Dead time for rising edge */
  uint16_t deadtime_down;          /* Dead time for falling edge */
};

/* GPT channel configuration */
struct ra_gpt_channel_s
{
  uint8_t channel;                 /* GPT channel number (0-13) */
  uint8_t mode;                    /* GPT mode flags */
  uint32_t pin_a;                  /* GTIOA pin configuration */
  uint32_t pin_b;                  /* GTIOB pin configuration (for complementary mode) */
  uint16_t deadtime_up;            /* Dead time for rising edge (in timer counts) */
  uint16_t deadtime_down;          /* Dead time for falling edge (in timer counts) */
};

/* GPT driver configuration */
struct ra_gpt_config_s
{
  uint8_t nchannels;               /* Number of GPT channels */
  const struct ra_gpt_channel_s *channels; /* GPT channel configurations */
  uint32_t base_frequency;         /* Base timer frequency in Hz */
  uint32_t default_frequency;      /* Default PWM frequency in Hz */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpt_initialize
 *
 * Description:
 *   Initialize the GPT subsystem
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_initialize(void);

/****************************************************************************
 * Name: ra_gpt_pwm_setup
 *
 * Description:
 *   Initialize one PWM channel for the given GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *
 * Returned Value:
 *   On success, a pointer to the RA GPT lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ra_gpt_pwm_setup(int channel);

/****************************************************************************
 * Name: ra_gpt_timer_setup
 *
 * Description:
 *   Initialize one timer for the given GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *
 * Returned Value:
 *   On success, a pointer to the RA GPT lower half timer driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct timer_lowerhalf_s *ra_gpt_timer_setup(int channel);

/****************************************************************************
 * Name: ra_gpt_start
 *
 * Description:
 *   Start the specified GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_start(int channel);

/****************************************************************************
 * Name: ra_gpt_stop
 *
 * Description:
 *   Stop the specified GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_stop(int channel);

/****************************************************************************
 * Name: ra_gpt_pwm_config
 *
 * Description:
 *   Configure PWM parameters for the specified GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *   config  - PWM configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_pwm_config(int channel, const struct ra_gpt_pwm_config_s *config);

/****************************************************************************
 * Name: ra_gpt_timer_config
 *
 * Description:
 *   Configure timer parameters for the specified GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *   config  - Timer configuration
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_timer_config(int channel, const struct ra_gpt_timer_config_s *config);

/****************************************************************************
 * Name: ra_gpt_get_counter
 *
 * Description:
 *   Get the current counter value for the specified GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *
 * Returned Value:
 *   Current counter value, or 0 on error.
 *
 ****************************************************************************/

uint32_t ra_gpt_get_counter(int channel);

/****************************************************************************
 * Name: ra_gpt_set_period
 *
 * Description:
 *   Set the period for the specified GPT channel
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *   period  - Period value in timer counts
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_set_period(int channel, uint32_t period);

/****************************************************************************
 * Name: ra_gpt_set_compare
 *
 * Description:
 *   Set the compare value for the specified GPT channel and compare register
 *
 * Input Parameters:
 *   channel - GPT channel number (0-13)
 *   compare - Compare register (0=A, 1=B, 2=C, 3=D)
 *   value   - Compare value in timer counts
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpt_set_compare(int channel, int compare, uint32_t value);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8_RA_GPT_H */
