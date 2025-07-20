/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_pwm.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA_PWM_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/timers/pwm.h>
#include "hardware/ra_gpt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of PWM channels */

#define RA8_PWM_NCHANNELS               14

/* PWM output modes */

#define RA8_PWM_MODE_COMPLEMENTARY      0x01
#define RA8_PWM_MODE_DEADTIME           0x02

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* PWM channel configuration */

struct ra_pwm_channel_s
{
  uint8_t channel;                 /* PWM channel number (0-13) */
  uint8_t mode;                    /* PWM mode flags */
  uint32_t pin_a;                  /* GTIOA pin configuration */
  uint32_t pin_b;                  /* GTIOB pin configuration (for complementary mode) */
  uint16_t deadtime_up;            /* Dead time for rising edge (in timer counts) */
  uint16_t deadtime_down;          /* Dead time for falling edge (in timer counts) */
};

/* PWM driver configuration */

struct ra_pwm_config_s
{
  uint8_t nchannels;               /* Number of PWM channels */
  const struct ra_pwm_channel_s *channels; /* PWM channel configurations */
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
 * Name: ra_pwm_initialize
 *
 * Description:
 *   Initialize the PWM driver for the specified timer channel.
 *
 * Input Parameters:
 *   channel - The PWM channel to initialize (0-13)
 *
 * Returned Value:
 *   On success, a pointer to the PWM lower half driver is returned.
 *   NULL is returned on failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ra_pwm_initialize(int channel);

/****************************************************************************
 * Name: ra_pwm_setup
 *
 * Description:
 *   Configure the PWM channel.
 *
 * Input Parameters:
 *   dev  - A reference to the PWM device structure
 *   info - PWM configuration information
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_pwm_setup(struct pwm_lowerhalf_s *dev, 
                  const struct pwm_info_s *info);

/****************************************************************************
 * Name: ra_pwm_shutdown
 *
 * Description:
 *   Disable the PWM channel.
 *
 * Input Parameters:
 *   dev - A reference to the PWM device structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_pwm_shutdown(struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Name: ra_pwm_start
 *
 * Description:
 *   Start the PWM output.
 *
 * Input Parameters:
 *   dev  - A reference to the PWM device structure
 *   info - PWM configuration information
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_pwm_start(struct pwm_lowerhalf_s *dev,
                  const struct pwm_info_s *info);

/****************************************************************************
 * Name: ra_pwm_stop
 *
 * Description:
 *   Stop the PWM output.
 *
 * Input Parameters:
 *   dev - A reference to the PWM device structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_pwm_stop(struct pwm_lowerhalf_s *dev);

/****************************************************************************
 * Name: ra_pwm_ioctl
 *
 * Description:
 *   Handle PWM ioctl commands.
 *
 * Input Parameters:
 *   dev - A reference to the PWM device structure
 *   cmd - The ioctl command
 *   arg - The argument of the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_PWM_H */
