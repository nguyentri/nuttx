/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_gpt_pwm.c
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
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <syslog.h>

#include <fixedmath.h>
#include <nuttx/timers/pwm.h>

#include "ra_gpt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PWM_FREQUENCY     400        /* 400 Hz ESC frequency */
#define PWM_MIN_PULSE_US  1000       /* 1000 us minimum pulse width */
#define PWM_MAX_PULSE_US  2000       /* 2000 us maximum pulse width */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_gpt_pwm_test_device
 *
 * Description:
 *   Test a single PWM device with various pulse widths
 *
 ****************************************************************************/

static int ra8e1_gpt_pwm_test_device(const char *devpath, int channel)
{
  int fd;
  int ret;
  struct pwm_info_s info;
  uint32_t pulse_widths[] = {1000, 1250, 1500, 1750, 2000}; /* Test pulse widths in us */
  int num_tests = sizeof(pulse_widths) / sizeof(pulse_widths[0]);

  syslog(LOG_INFO, "Testing PWM device: %s (Channel %d)\n", devpath, channel);

  /* Open the PWM device */
  fd = open(devpath, O_RDONLY);
  if (fd < 0)
    {
      syslog(LOG_INFO, "ERROR: Failed to open %s: %d\n", devpath, fd);
      return fd;
    }

  /* Configure PWM characteristics */
  info.frequency = PWM_FREQUENCY;
  info.duty = 0; /* Start with 0% duty cycle */

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);
  if (ret < 0)
    {
      syslog(LOG_INFO, "ERROR: Failed to set characteristics for %s: %d\n", devpath, ret);
      close(fd);
      return ret;
    }

  syslog(LOG_INFO, "PWM configured: %d Hz\n", PWM_FREQUENCY);

  /* Test different pulse widths */
  for (int i = 0; i < num_tests; i++)
    {
      uint32_t pulse_us = pulse_widths[i];

      /* Calculate duty cycle from pulse width */
      /* duty = (pulse_us * 65536) / (1000000 / frequency) */
      uint32_t period_us = 1000000 / PWM_FREQUENCY; /* Period in microseconds */
      uint32_t duty = (pulse_us * 65536) / period_us;

      info.frequency = PWM_FREQUENCY;
      info.duty = duty;

      ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);
      if (ret < 0)
        {
          syslog(LOG_INFO, "ERROR: Failed to set duty cycle for %s: %d\n", devpath, ret);
          break;
        }

      /* Start PWM */
      ret = ioctl(fd, PWMIOC_START, 0);
      if (ret < 0)
        {
          syslog(LOG_INFO, "ERROR: Failed to start PWM for %s: %d\n", devpath, ret);
          break;
        }

      syslog(LOG_INFO, "  Pulse width: %" PRIu32 " us, Duty: %" PRIu32 "/65536 (%" PRIu32 "%%) - ACTIVE\n",
        pulse_us, duty, (duty * 100) / 65536);

      /* Keep this setting for 2 seconds */
      sleep(2);

      /* Stop PWM */
      ret = ioctl(fd, PWMIOC_STOP, 0);
      if (ret < 0)
        {
          syslog(LOG_INFO, "ERROR: Failed to stop PWM for %s: %d\n", devpath, ret);
          break;
        }

      syslog(LOG_INFO, "  PWM stopped\n");
      sleep(1);
    }

  close(fd);
  syslog(LOG_INFO, "Test completed for %s\n\n", devpath);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_gpt_pwm_main
 *
 * Description:
 *   Test all available PWM devices for ESC control
 *
 ****************************************************************************/

int ra8e1_gpt_pwm_main(int argc, char *argv[])
{
  int ret = OK;

  syslog(LOG_INFO, "RA8E1 PWM ESC Test Application\n");
  syslog(LOG_INFO, "==============================\n\n");

  syslog(LOG_INFO, "Testing ESC PWM channels:\n");
  syslog(LOG_INFO, "ESC1: /dev/pwm3 (P300/GPT3A)\n");
  syslog(LOG_INFO, "ESC2: /dev/pwm0 (P415/GPT0A)\n");
  syslog(LOG_INFO, "ESC3: /dev/pwm2 (P114/GPT2B)\n");
  syslog(LOG_INFO, "ESC4: /dev/pwm4 (P302/GPT4A)\n\n");

  syslog(LOG_INFO, "Test sequence:\n");
  syslog(LOG_INFO, "- 1000us (0%% throttle, armed)\n");
  syslog(LOG_INFO, "- 1250us (25%% throttle)\n");
  syslog(LOG_INFO, "- 1500us (50%% throttle)\n");
  syslog(LOG_INFO, "- 1750us (75%% throttle)\n");
  syslog(LOG_INFO, "- 2000us (100%% throttle)\n\n");

  /* Test ESC1 (GPT3A) */
  ret = ra8e1_gpt_pwm_test_device("/dev/pwm3", 3);
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test failed for ESC1\n");
      return ret;
    }

  /* Test ESC2 (GPT0A) */
  ret = ra8e1_gpt_pwm_test_device("/dev/pwm0", 0);
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test failed for ESC2\n");
      return ret;
    }

  /* Test ESC3 (GPT2B) */
  ret = ra8e1_gpt_pwm_test_device("/dev/pwm2", 2);
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test failed for ESC3\n");
      return ret;
    }

  /* Test ESC4 (GPT4A) */
  ret = ra8e1_gpt_pwm_test_device("/dev/pwm4", 4);
  if (ret < 0)
    {
      syslog(LOG_INFO, "Test failed for ESC4\n");
      return ret;
    }

  syslog(LOG_INFO, "All PWM ESC tests completed successfully!\n");
  syslog(LOG_INFO, "You can now use the interactive ESC demo: ra8e1_gpt_escs\n");

  return OK;
}


/****************************************************************************
 * Name: ra8e1_gpt_pwm_initialize
 *
 * Description:
 *   Initialize GPT-based PWM devices for ESC control
 *
 ****************************************************************************/

int ra8e1_gpt_pwm_initialize(void)
{
  struct pwm_lowerhalf_s *pwm;
  int ret;

  syslog(LOG_INFO, "Initializing GPT-based PWM devices for ESC control\n");

  /* Initialize GPT3 for ESC1 (P300 - ch 3A) */
  pwm = ra_gpt_initialize(3);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup GPT3 PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm3", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm3: %d\n", ret);
      return ret;
    }
  syslog(LOG_INFO, "ESC1 PWM registered at /dev/pwm3 (GPT3A - P300)\n");

  /* Initialize GPT0 for ESC2 (P415 - ch 0A) */
  pwm = ra_gpt_initialize(0);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup GPT0 PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm0: %d\n", ret);
      return ret;
    }
  syslog(LOG_INFO, "ESC2 PWM registered at /dev/pwm0 (GPT0A - P415)\n");

  /* Initialize GPT5 for ESC3 (P905 - alt PWM) */
  pwm = ra_gpt_initialize(5);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup GPT5 PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm5", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm5: %d\n", ret);
      return ret;
    }
  syslog(LOG_INFO, "ESC3 PWM registered at /dev/pwm5 (GPT5A - P905)\n");

  /* Initialize GPT2 for ESC4 (P114 - ch 2B) */
  pwm = ra_gpt_initialize(2);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup GPT2 PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm2", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm2: %d\n", ret);
      return ret;
    }
  syslog(LOG_INFO, "ESC4 PWM registered at /dev/pwm2 (GPT2B - P114)\n");

  /* Initialize GPT2 for ESC5 (P113 - ch 2A) - Note: GPT2 supports both A and B outputs */
  /* ESC5 uses the same GPT2 channel but different output pin (2A vs 2B) */
  syslog(LOG_INFO, "ESC5 PWM uses GPT2A (P113) - shared with GPT2 channel\n");

  /* Initialize GPT4 for ESC6 (P302 - ch 4A) */
  pwm = ra_gpt_initialize(4);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to setup GPT4 PWM\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm4", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm4: %d\n", ret);
      return ret;
    }
  syslog(LOG_INFO, "ESC6 PWM registered at /dev/pwm4 (GPT4A - P302)\n");

  syslog(LOG_INFO, "All GPT-based PWM devices initialized for 400Hz ESC control\n");
  return OK;
}
