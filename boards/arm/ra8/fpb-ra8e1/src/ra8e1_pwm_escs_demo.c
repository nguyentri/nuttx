/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_pwm_escs_demo.c
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
#ifdef CONFIG_RA8E1_PWM_ESCS_DEMO

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/timers/pwm.h>
#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra_gpt.h"
#include "ra8e1_demo_log.h"
#include "fpb-ra8e1.h"


#ifdef CONFIG_RA8E1_PWM_ESCS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESC PWM Channel Definitions - Using 4 ESCs only for simplicity */

#define NUM_ESC_CHANNELS        4
#define ESC_PWM_FREQUENCY       400      /* 400Hz for ESC control */
#define ESC_PWM_MIN_US          1000     /* Minimum pulse width (microseconds) */
#define ESC_PWM_MAX_US          2000     /* Maximum pulse width (microseconds) */
#define ESC_PWM_ARM_US          1000     /* Arming pulse width */

/* PWM Device Paths */

#define PWM_DEVICE_PATH         "/dev/pwm"

/* RTT Command Buffer */
#define RTT_COMMAND_MAX         64

/* GPIO Configuration for ESC PWM pins */

static gpio_pinset_t g_pwm_esc1 = {3, 0, 0};   /* P300 - GPT3A */
static gpio_pinset_t g_pwm_esc2 = {4, 15, 0};  /* P415 - GPT0A */
static gpio_pinset_t g_pwm_esc3 = {1, 14, 0};  /* P114 - GPT2B */
static gpio_pinset_t g_pwm_esc4 = {3, 2, 0};   /* P302 - GPT4A */

/* ESC Status Structure */

struct esc_status_s
{
  bool armed;                   /* ESC armed status */
  uint16_t throttle_percent;    /* Current throttle percentage */
  uint32_t pulse_width_us;      /* Current pulse width in microseconds */
};

/* ESC Channel Configuration */

struct esc_channel_s
{
  int pwm_channel;              /* PWM channel number */
  const char *device_path;      /* PWM device path */
  int fd;                       /* File descriptor */
  uint16_t current_throttle;    /* Current throttle percentage */
  bool armed;                   /* ESC armed status */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PWM device descriptors for ESC channels */

static struct pwm_lowerhalf_s *g_pwm_esc_devs[4];

/* Demo control variables */

static bool g_demo_running = false;
static char g_rtt_command[RTT_COMMAND_MAX];

/* ESC channel configurations */

static struct esc_channel_s g_esc_channels[NUM_ESC_CHANNELS] =
{
  { 3, "/dev/pwm3", -1, 0, false },  /* ESC1 - P300, GPT3A */
  { 0, "/dev/pwm0", -1, 0, false },  /* ESC2 - P415, GPT0A */
  { 2, "/dev/pwm2", -1, 0, false },  /* ESC3 - P114, GPT2B */
  { 4, "/dev/pwm4", -1, 0, false },  /* ESC4 - P302, GPT4A */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra8e1_pwm_setup_pins(void);
static int ra8e1_pwm_initialize(void);
static int ra8e1_pwm_setup(void);
static struct pwm_lowerhalf_s *ra8e1_esc_get_pwm_dev(int esc_index);
static int ra8e1_esc_set_duty_us(int esc_index, uint32_t pulse_us, uint32_t frequency);
static int ra8e1_esc_stop_all(void);

static int esc_pwm_open(int esc_index);
static int esc_pwm_close(int esc_index);
static int esc_set_throttle_us(int esc_index, uint32_t pulse_us);
static int esc_set_throttle_percent(int esc_index, uint16_t throttle_percent);
static int esc_arm(int esc_index);
static int esc_disarm(int esc_index);
static int esc_arm_all(void);
static int esc_disarm_all(void);

static void print_menu(void);
static void process_rtt_command(const char *command);
static int parse_esc_command(const char *command, int *esc_index, int *value);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_pwm_setup_pins
 *
 * Description:
 *   Setup GPIO pins for PWM ESC output
 *
 ****************************************************************************/

static int ra8e1_pwm_setup_pins(void)
{
  /* Configure ESC1 pin (P300 - GPT3A) */
  ra_configgpio(g_pwm_esc1);

  /* Configure ESC2 pin (P415 - GPT0A) */
  ra_configgpio(g_pwm_esc2);

  /* Configure ESC3 pin (P114 - GPT2B) */
  ra_configgpio(g_pwm_esc3);

  /* Configure ESC4 pin (P302 - GPT4A) */
  ra_configgpio(g_pwm_esc4);

  pwminfo("ESC PWM pins configured successfully\n");
  return 0;
}

/****************************************************************************
 * Name: ra8e1_pwm_initialize
 *
 * Description:
 *   Initialize PWM for ESC control and register the PWM devices.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

static int ra8e1_pwm_initialize(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;
  int i;

  /* Have we already initialized? */

  if (initialized)
    {
      return OK;
    }

  pwminfo("Initializing ESC PWM drivers\n");

  /* Setup GPIO pins for PWM */

  ret = ra8e1_pwm_setup_pins();
  if (ret < 0)
    {
      pwmerr("ERROR: PWM pin setup failed: %d\n", ret);
      return ret;
    }

  /* Initialize GPT0 for ESC2 (P415) */

  pwm = ra_pwm_initialize(0);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get GPT0 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: Failed to register /dev/pwm0: %d\n", ret);
      return ret;
    }

  g_pwm_esc_devs[1] = pwm;  /* ESC2 maps to PWM0/GPT0 */
  pwminfo("ESC2 PWM driver registered at /dev/pwm0\n");

  /* Initialize GPT2 for ESC3 (P114) */

  pwm = ra_pwm_initialize(2);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get GPT2 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm2", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: Failed to register /dev/pwm2: %d\n", ret);
      return ret;
    }

  g_pwm_esc_devs[2] = pwm;  /* ESC3 maps to PWM2/GPT2 */
  pwminfo("ESC3 PWM driver registered at /dev/pwm2\n");

  /* Initialize GPT3 for ESC1 (P300) */

  pwm = ra_pwm_initialize(3);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get GPT3 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm3", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: Failed to register /dev/pwm3: %d\n", ret);
      return ret;
    }

  g_pwm_esc_devs[0] = pwm;  /* ESC1 maps to PWM3/GPT3 */
  pwminfo("ESC1 PWM driver registered at /dev/pwm3\n");

  /* Initialize GPT4 for ESC4 (P302) */

  pwm = ra_pwm_initialize(4);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get GPT4 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm4", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: Failed to register /dev/pwm4: %d\n", ret);
      return ret;
    }

  g_pwm_esc_devs[3] = pwm;  /* ESC4 maps to PWM4/GPT4 */
  pwminfo("ESC4 PWM driver registered at /dev/pwm4\n");

  initialized = true;
  pwminfo("All ESC PWM drivers initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

static int ra8e1_pwm_setup(void)
{
  return ra8e1_pwm_initialize();
}

/****************************************************************************
 * Name: ra8e1_esc_get_pwm_dev
 *
 * Description:
 *   Get PWM lower half device for specified ESC
 *
 * Input Parameters:
 *   esc_index - ESC index (0-3)
 *
 * Returned Value:
 *   PWM device pointer on success, NULL on failure.
 *
 ****************************************************************************/

static struct pwm_lowerhalf_s *ra8e1_esc_get_pwm_dev(int esc_index)
{
  if (esc_index < 0 || esc_index >= 4)
    {
      return NULL;
    }

  return g_pwm_esc_devs[esc_index];
}

#ifdef CONFIG_PWM_HAVE_DUTY16
/****************************************************************************
 * Name: ra8e1_esc_set_duty_us
 *
 * Description:
 *   Set ESC PWM duty cycle using microsecond pulse width
 *
 * Input Parameters:
 *   esc_index - ESC index (0-3)
 *   pulse_us  - Pulse width in microseconds (1000-2000 for ESCs)
 *   frequency - PWM frequency in Hz (typically 400)
 *
 * Returned Value:
 *   Zero on success; negated errno on failure.
 *
 ****************************************************************************/

static int ra8e1_esc_set_duty_us(int esc_index, uint32_t pulse_us, uint32_t frequency)
{
  struct pwm_lowerhalf_s *pwm;
  struct pwm_info_s info;
  uint32_t period_us;
  uint32_t duty_frac;

  if (esc_index < 0 || esc_index >= 4)
    {
      return -EINVAL;
    }

  pwm = g_pwm_esc_devs[esc_index];
  if (!pwm)
    {
      return -ENODEV;
    }

  /* Calculate duty cycle as fraction of period */

  period_us = 1000000 / frequency;  /* Period in microseconds */
  duty_frac = (pulse_us * 65536) / period_us;  /* 16-bit duty fraction */

  /* Prepare PWM info */

  info.frequency = frequency;
  info.duty = duty_frac;

  /* Set the PWM */

  return PWM_START(pwm, &info);
}
#endif

/****************************************************************************
 * Name: ra8e1_esc_stop_all
 *
 * Description:
 *   Stop all ESC PWM outputs
 *
 * Returned Value:
 *   Zero on success; negated errno on failure.
 *
 ****************************************************************************/

static int ra8e1_esc_stop_all(void)
{
  int ret = OK;
  int i;

  for (i = 0; i < 4; i++)
    {
      if (g_pwm_esc_devs[i])
        {
          if (PWM_STOP(g_pwm_esc_devs[i]) < 0)
            {
              pwmerr("ERROR: Failed to stop ESC%d PWM\n", i + 1);
              ret = -EIO;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esc_pwm_open
 *
 * Description:
 *   Open PWM device for specified ESC channel
 *
 ****************************************************************************/

static int esc_pwm_open(int esc_index)
{
  struct pwm_info_s info;
  int ret;

  if (esc_index < 0 || esc_index >= NUM_ESC_CHANNELS)
    {
      return -EINVAL;
    }

  if (g_esc_channels[esc_index].fd >= 0)
    {
      /* Already open */
      return OK;
    }

  /* Open PWM device */

  g_esc_channels[esc_index].fd = open(g_esc_channels[esc_index].device_path,
                                      O_RDONLY);
  if (g_esc_channels[esc_index].fd < 0)
    {
      demoprintf("ERROR: Failed to open %s: %d\n",
             g_esc_channels[esc_index].device_path, errno);
      return -errno;
    }

  /* Configure PWM */

  info.frequency = ESC_PWM_FREQUENCY;
  info.duty = 0;

  ret = ioctl(g_esc_channels[esc_index].fd, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)&info);
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to configure PWM%d: %d\n", esc_index, errno);
      close(g_esc_channels[esc_index].fd);
      g_esc_channels[esc_index].fd = -1;
      return -errno;
    }

  demoprintf("ESC%d: PWM device opened and configured\n", esc_index + 1);
  return OK;
}

/****************************************************************************
 * Name: esc_pwm_close
 *
 * Description:
 *   Close PWM device for specified ESC channel
 *
 ****************************************************************************/

static int esc_pwm_close(int esc_index)
{
  if (esc_index < 0 || esc_index >= NUM_ESC_CHANNELS)
    {
      return -EINVAL;
    }

  if (g_esc_channels[esc_index].fd >= 0)
    {
      /* Stop PWM before closing */
      ioctl(g_esc_channels[esc_index].fd, PWMIOC_STOP, 0);
      
      close(g_esc_channels[esc_index].fd);
      g_esc_channels[esc_index].fd = -1;
      g_esc_channels[esc_index].armed = false;
      g_esc_channels[esc_index].current_throttle = 0;
    }

  return OK;
}

/****************************************************************************
 * Name: esc_set_throttle_us
 *
 * Description:
 *   Set ESC throttle using pulse width in microseconds
 *
 ****************************************************************************/

static int esc_set_throttle_us(int esc_index, uint32_t pulse_us)
{
  struct pwm_info_s info;
  uint32_t duty_percentage;
  int ret;

  if (esc_index < 0 || esc_index >= NUM_ESC_CHANNELS)
    {
      return -EINVAL;
    }

  if (g_esc_channels[esc_index].fd < 0)
    {
      demoprintf("ERROR: ESC%d PWM device not open\n", esc_index + 1);
      return -ENODEV;
    }

  /* Clamp pulse width to valid range */
  if (pulse_us < ESC_PWM_MIN_US)
    {
      pulse_us = ESC_PWM_MIN_US;
    }
  else if (pulse_us > ESC_PWM_MAX_US)
    {
      pulse_us = ESC_PWM_MAX_US;
    }

  /* Convert pulse width to duty cycle percentage
   * For 400Hz: period = 2500us, so duty = (pulse_us / 2500) * 100
   */
  duty_percentage = (pulse_us * 100) / (1000000 / ESC_PWM_FREQUENCY);

  info.frequency = ESC_PWM_FREQUENCY;
  info.duty = (duty_percentage * 65536) / 100;  /* Convert to 16-bit duty */

  ret = ioctl(g_esc_channels[esc_index].fd, PWMIOC_SETCHARACTERISTICS,
              (unsigned long)&info);
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to set PWM duty for ESC%d: %d\n",
             esc_index + 1, errno);
      return -errno;
    }

  /* Start PWM if not already started */
  ret = ioctl(g_esc_channels[esc_index].fd, PWMIOC_START, 0);
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to start PWM for ESC%d: %d\n",
             esc_index + 1, errno);
      return -errno;
    }

  demoprintf("ESC%d: Set to %lu µs (duty: %lu%%)\n",
         esc_index + 1, pulse_us, duty_percentage);

  return OK;
}

/****************************************************************************
 * Name: esc_set_throttle_percent
 *
 * Description:
 *   Set ESC throttle using percentage (0-100%)
 *
 ****************************************************************************/

static int esc_set_throttle_percent(int esc_index, uint16_t throttle_percent)
{
  uint32_t pulse_us;

  if (throttle_percent > 100)
    {
      throttle_percent = 100;
    }

  /* Convert percentage to pulse width */
  pulse_us = ESC_PWM_MIN_US + 
             ((ESC_PWM_MAX_US - ESC_PWM_MIN_US) * throttle_percent) / 100;

  g_esc_channels[esc_index].current_throttle = throttle_percent;

  return esc_set_throttle_us(esc_index, pulse_us);
}

/****************************************************************************
 * Name: esc_arm
 *
 * Description:
 *   Arm specified ESC by sending minimum throttle signal
 *
 ****************************************************************************/

static int esc_arm(int esc_index)
{
  int ret;

  ret = esc_set_throttle_us(esc_index, ESC_PWM_ARM_US);
  if (ret == OK)
    {
      g_esc_channels[esc_index].armed = true;
      demoprintf("ESC%d: Armed\n", esc_index + 1);
    }

  return ret;
}

/****************************************************************************
 * Name: esc_disarm
 *
 * Description:
 *   Disarm specified ESC by stopping PWM output
 *
 ****************************************************************************/

static int esc_disarm(int esc_index)
{
  int ret = OK;

  if (g_esc_channels[esc_index].fd >= 0)
    {
      ret = ioctl(g_esc_channels[esc_index].fd, PWMIOC_STOP, 0);
      if (ret < 0)
        {
          demoprintf("ERROR: Failed to stop PWM for ESC%d: %d\n",
                 esc_index + 1, errno);
        }
      else
        {
          g_esc_channels[esc_index].armed = false;
          g_esc_channels[esc_index].current_throttle = 0;
          demoprintf("ESC%d: Disarmed\n", esc_index + 1);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esc_arm_all
 *
 * Description:
 *   Arm all ESCs simultaneously
 *
 ****************************************************************************/

static int esc_arm_all(void)
{
  int ret = OK;
  int i;

  demoprintf("Arming all ESCs...\n");

  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      ret = esc_arm(i);
      if (ret < 0)
        {
          demoprintf("ERROR: Failed to arm ESC%d\n", i + 1);
          break;
        }
      usleep(100000);  /* 100ms delay between arming */
    }

  if (ret == OK)
    {
      demoprintf("All ESCs armed successfully\n");
    }

  return ret;
}

/****************************************************************************
 * Name: esc_disarm_all
 *
 * Description:
 *   Disarm all ESCs simultaneously
 *
 ****************************************************************************/

static int esc_disarm_all(void)
{
  int i;

  demoprintf("Disarming all ESCs...\n");

  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      esc_disarm(i);
    }

  demoprintf("All ESCs disarmed\n");
  return OK;
}

/****************************************************************************
 * Name: print_menu
 *
 * Description:
 *   Print RTT command menu
 *
 ****************************************************************************/

static void print_menu(void)
{
  demoprintf("\n=== RA8E1 ESC PWM Control Demo ===\n");
  demoprintf("Commands:\n");
  demoprintf("  help      - Show this menu\n");
  demoprintf("  status    - Show ESC status\n");
  demoprintf("  arm       - Arm all ESCs\n");
  demoprintf("  disarm    - Disarm all ESCs\n");
  demoprintf("  arm<N>    - Arm ESC N (1-4)\n");
  demoprintf("  esc<N> <%%> - Set ESC N throttle (0-100%%)\n");
  demoprintf("  test      - Run automatic test sequence\n");
  demoprintf("  stop      - Stop demo\n");
  demoprintf("\nExamples:\n");
  demoprintf("  esc1 50   - Set ESC1 to 50%% throttle\n");
  demoprintf("  esc2 0    - Set ESC2 to 0%% throttle\n");
  demoprintf("  arm1      - Arm ESC1 only\n");
  demoprintf("\nCurrent Status:\n");
  
  for (int i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      demoprintf("  ESC%d: %s, %d%% throttle\n", 
             i + 1,
             g_esc_channels[i].armed ? "ARMED" : "DISARMED",
             g_esc_channels[i].current_throttle);
    }
  demoprintf("\n");
}

/****************************************************************************
 * Name: parse_esc_command
 *
 * Description:
 *   Parse ESC command (e.g., "esc1 50" or "arm2")
 *
 ****************************************************************************/

static int parse_esc_command(const char *command, int *esc_index, int *value)
{
  int parsed_items;
  char cmd_type[8];
  int channel;

  /* Try to parse "esc<N> <value>" format */
  parsed_items = sscanf(command, "%3s%d %d", cmd_type, &channel, value);
  
  if (parsed_items >= 2 && strcmp(cmd_type, "esc") == 0)
    {
      if (channel >= 1 && channel <= NUM_ESC_CHANNELS)
        {
          *esc_index = channel - 1;
          if (parsed_items == 2)
            {
              *value = 0;  /* Default value */
            }
          return 1;  /* ESC command */
        }
    }

  /* Try to parse "arm<N>" format */
  parsed_items = sscanf(command, "%3s%d", cmd_type, &channel);
  
  if (parsed_items == 2 && strcmp(cmd_type, "arm") == 0)
    {
      if (channel >= 1 && channel <= NUM_ESC_CHANNELS)
        {
          *esc_index = channel - 1;
          *value = 0;
          return 2;  /* ARM command */
        }
    }

  return 0;  /* Not recognized */
}

/****************************************************************************
 * Name: process_rtt_command
 *
 * Description:
 *   Process RTT command input
 *
 ****************************************************************************/

static void process_rtt_command(const char *command)
{
  int esc_index, value;
  int cmd_type;

  if (strlen(command) == 0)
    {
      return;
    }

  demoprintf("Command: %s\n", command);

  /* Check for simple commands */
  if (strcmp(command, "help") == 0)
    {
      print_menu();
    }
  else if (strcmp(command, "status") == 0)
    {
      print_menu();
    }
  else if (strcmp(command, "arm") == 0)
    {
      esc_arm_all();
    }
  else if (strcmp(command, "disarm") == 0)
    {
      esc_disarm_all();
    }
  else if (strcmp(command, "test") == 0)
    {
      ra8e1_pwm_escs_demo_test();
    }
  else if (strcmp(command, "stop") == 0)
    {
      demoprintf("Stopping demo...\n");
      g_demo_running = false;
    }
  else
    {
      /* Try to parse ESC-specific commands */
      cmd_type = parse_esc_command(command, &esc_index, &value);
      
      if (cmd_type == 1)  /* ESC throttle command */
        {
          if (value >= 0 && value <= 100)
            {
              esc_set_throttle_percent(esc_index, value);
            }
          else
            {
              demoprintf("ERROR: Throttle value must be 0-100%%\n");
            }
        }
      else if (cmd_type == 2)  /* ARM command */
        {
          esc_arm(esc_index);
        }
      else
        {
          demoprintf("ERROR: Unknown command '%s'. Type 'help' for commands.\n", 
                 command);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_pwm_escs_demo_init
 *
 * Description:
 *   Initialize the ESCs demo
 *
 ****************************************************************************/

int ra8e1_pwm_escs_demo_init(void)
{
  /* ESCs demo initialization is done within main function */
  return 0;
}

/****************************************************************************
 * Name: ra8e1_pwm_escs_demo_main
 *
 * Description:
 *   Main entry point for PWM ESC control demo
 *
 ****************************************************************************/

int ra8e1_pwm_escs_demo_main(int argc, char *argv[])
{
  int ret;
  int i;
  int cmd_len = 0;
  char ch;

  demoprintf("\n=== RA8E1 ESC PWM Control Demo ===\n");
  demoprintf("Initializing PWM devices...\n");

  /* Initialize PWM subsystem first */
  ret = ra8e1_pwm_initialize();
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to initialize PWM subsystem: %d\n", ret);
      return ret;
    }

  /* Initialize RTT */
  demoprintf("RTT initialized - ready for commands\n");

  /* Initialize all ESC channels */
  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      ret = esc_pwm_open(i);
      if (ret < 0)
        {
          demoprintf("ERROR: Failed to initialize ESC%d: %d\n", i + 1, ret);
          goto cleanup;
        }
    }

  demoprintf("All PWM devices initialized successfully\n");
  
  /* Safety: Disarm all ESCs initially */
  esc_disarm_all();

  print_menu();

  g_demo_running = true;

  /* Main command loop */
  demoprintf("Ready for commands (type 'help' for menu):\n> ");
  
  while (g_demo_running)
    {
      /* Check for input */
      if (demo_haskey())
        {
          ch = demo_getkey();
          if (ch == '\n' || ch == '\r')
            {
              if (cmd_len > 0)
                {
                  g_rtt_command[cmd_len] = '\0';
                  process_rtt_command(g_rtt_command);
                  cmd_len = 0;
                  demoprintf("> ");
                }
            }
          else if (ch == '\b' || ch == 127)  /* Backspace */
            {
              if (cmd_len > 0)
                {
                  cmd_len--;
                  demoprintf("\b \b");
                }
            }
          else if (cmd_len < RTT_COMMAND_MAX - 1)
            {
              g_rtt_command[cmd_len++] = ch;
              demoprintf("%c", ch);
            }
        }
      
      usleep(10000);  /* 10ms delay */
    }

cleanup:
  /* Cleanup: Disarm all ESCs and close devices */
  demoprintf("Cleaning up...\n");
  esc_disarm_all();
  
  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      esc_pwm_close(i);
    }

  demoprintf("Demo stopped\n");
  return ret;
}

/****************************************************************************
 * Name: ra8e1_esc_get_status
 *
 * Description:
 *   Get current status of specified ESC channel
 *
 ****************************************************************************/

int ra8e1_esc_get_status(int esc_index, struct esc_status_s *status)
{
  if (esc_index < 0 || esc_index >= NUM_ESC_CHANNELS || status == NULL)
    {
      return -EINVAL;
    }

  status->armed = g_esc_channels[esc_index].armed;
  status->throttle_percent = g_esc_channels[esc_index].current_throttle;
  
  /* Calculate current pulse width */
  status->pulse_width_us = ESC_PWM_MIN_US + 
                          ((ESC_PWM_MAX_US - ESC_PWM_MIN_US) * 
                           status->throttle_percent) / 100;

  return OK;
}

/****************************************************************************
 * Name: ra8e1_pwm_escs_demo_test
 *
 * Description:
 *   Simple test sequence for ESC control
 *
 ****************************************************************************/

int ra8e1_pwm_escs_demo_test(void)
{
  int ret;
  int i;

  demoprintf("Starting ESC test sequence...\n");

  /* Test sequence:
   * 1. Arm all ESCs
   * 2. Gradually increase throttle
   * 3. Gradually decrease throttle
   * 4. Disarm all ESCs
   */

  ret = esc_arm_all();
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to arm ESCs\n");
      return ret;
    }

  sleep(2);  /* Wait 2 seconds */

  /* Gradually increase throttle to 25% */
  for (i = 0; i <= 25; i += 5)
    {
      demoprintf("Setting all ESCs to %d%% throttle\n", i);
      
      for (int j = 0; j < NUM_ESC_CHANNELS; j++)
        {
          esc_set_throttle_percent(j, i);
        }
      
      sleep(1);  /* Wait 1 second between steps */
    }

  sleep(3);  /* Hold at 25% for 3 seconds */

  /* Gradually decrease throttle to 0% */
  for (i = 25; i >= 0; i -= 5)
    {
      demoprintf("Setting all ESCs to %d%% throttle\n", i);
      
      for (int j = 0; j < NUM_ESC_CHANNELS; j++)
        {
          esc_set_throttle_percent(j, i);
        }
      
      sleep(1);  /* Wait 1 second between steps */
    }

  sleep(2);  /* Wait 2 seconds */

  /* Disarm all ESCs */
  esc_disarm_all();

  demoprintf("ESC test sequence completed\n");
  return OK;
}

#endif /* CONFIG_RA8E1_PWM_ESCS */

#endif /* CONFIG_RA8E1_PWM_ESCS_DEMO */
