/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_pwm_test.c
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

/* PWM driver definitions */
#define PWMIOC_SETCHARACTERISTICS  1
#define PWMIOC_START              2  
#define PWMIOC_STOP               3

/* PWM info structure */
struct pwm_info_s
{
  uint32_t frequency;             /* PWM frequency */
  uint32_t duty;                  /* PWM duty cycle (0-65536) */
};

#ifndef OK
#define OK 0
#endif

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
 * Name: test_pwm_device
 *
 * Description:
 *   Test a single PWM device with various pulse widths
 *
 ****************************************************************************/

static int test_pwm_device(const char *devpath, int channel)
{
  int fd;
  int ret;
  struct pwm_info_s info;
  uint32_t pulse_widths[] = {1000, 1250, 1500, 1750, 2000}; /* Test pulse widths in us */
  int num_tests = sizeof(pulse_widths) / sizeof(pulse_widths[0]);

  printf("Testing PWM device: %s (Channel %d)\n", devpath, channel);

  /* Open the PWM device */
  fd = open(devpath, O_RDONLY);
  if (fd < 0)
    {
      printf("ERROR: Failed to open %s: %d\n", devpath, fd);
      return fd;
    }

  /* Configure PWM characteristics */
  info.frequency = PWM_FREQUENCY;
  info.duty = 0; /* Start with 0% duty cycle */

  ret = ioctl(fd, PWMIOC_SETCHARACTERISTICS, (unsigned long)&info);
  if (ret < 0)
    {
      printf("ERROR: Failed to set characteristics for %s: %d\n", devpath, ret);
      close(fd);
      return ret;
    }

  printf("PWM configured: %d Hz\n", PWM_FREQUENCY);

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
          printf("ERROR: Failed to set duty cycle for %s: %d\n", devpath, ret);
          break;
        }

      /* Start PWM */
      ret = ioctl(fd, PWMIOC_START, 0);
      if (ret < 0)
        {
          printf("ERROR: Failed to start PWM for %s: %d\n", devpath, ret);
          break;
        }

      printf("  Pulse width: %u us, Duty: %u/65536 (%u%%) - ACTIVE\n", 
             pulse_us, duty, (duty * 100) / 65536);

      /* Keep this setting for 2 seconds */
      sleep(2);

      /* Stop PWM */
      ret = ioctl(fd, PWMIOC_STOP, 0);
      if (ret < 0)
        {
          printf("ERROR: Failed to stop PWM for %s: %d\n", devpath, ret);
          break;
        }

      printf("  PWM stopped\n");
      sleep(1);
    }

  close(fd);
  printf("Test completed for %s\n\n", devpath);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_pwm_test_main
 *
 * Description:
 *   Test all available PWM devices for ESC control
 *
 ****************************************************************************/

int ra8e1_pwm_test_main(int argc, char *argv[])
{
  int ret = OK;

  printf("RA8E1 PWM ESC Test Application\n");
  printf("==============================\n\n");
  
  printf("Testing ESC PWM channels:\n");
  printf("ESC1: /dev/pwm3 (P300/GPT3A)\n");
  printf("ESC2: /dev/pwm0 (P415/GPT0A)\n"); 
  printf("ESC3: /dev/pwm2 (P114/GPT2B)\n");
  printf("ESC4: /dev/pwm4 (P302/GPT4A)\n\n");

  printf("Test sequence:\n");
  printf("- 1000us (0%% throttle, armed)\n");
  printf("- 1250us (25%% throttle)\n");
  printf("- 1500us (50%% throttle)\n");
  printf("- 1750us (75%% throttle)\n");
  printf("- 2000us (100%% throttle)\n\n");

  /* Test ESC1 (GPT3A) */
  ret = test_pwm_device("/dev/pwm3", 3);
  if (ret < 0)
    {
      printf("Test failed for ESC1\n");
      return ret;
    }

  /* Test ESC2 (GPT0A) */
  ret = test_pwm_device("/dev/pwm0", 0);
  if (ret < 0)
    {
      printf("Test failed for ESC2\n");
      return ret;
    }

  /* Test ESC3 (GPT2B) */
  ret = test_pwm_device("/dev/pwm2", 2);
  if (ret < 0)
    {
      printf("Test failed for ESC3\n");
      return ret;
    }

  /* Test ESC4 (GPT4A) */
  ret = test_pwm_device("/dev/pwm4", 4);
  if (ret < 0)
    {
      printf("Test failed for ESC4\n");
      return ret;
    }

  printf("All PWM ESC tests completed successfully!\n");
  printf("You can now use the interactive ESC demo: ra8e1_pwm_escs_demo\n");

  return OK;
}
