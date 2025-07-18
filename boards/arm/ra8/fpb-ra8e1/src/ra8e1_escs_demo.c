
/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_pwm_demo.c
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
#include "ra8e1_escs_demo.h"

/* Conditional RTT support */
#ifdef CONFIG_SEGGER_RTT
#include "SEGGER_RTT.h"
#define RTT_PRINTF(...)     SEGGER_RTT_printf(0, __VA_ARGS__)
#define RTT_HASKEY()        SEGGER_RTT_HasKey()
#define RTT_GETKEY()        SEGGER_RTT_GetKey()
#else
#define RTT_PRINTF(...)     printf(__VA_ARGS__)
#define RTT_HASKEY()        (false)
#define RTT_GETKEY()        (0)
#endif

#ifdef CONFIG_EXAMPLES_ESCS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESC PWM Channel Definitions - Using 4 ESCs only for simplicity */

#define NUM_ESC_CHANNELS        4
#define ESC_PWM_FREQUENCY       400      /* 400Hz for ESC control */
#define ESC_PWM_MIN_US          1000     /* Minimum pulse width (microseconds) */
#define ESC_PWM_MAX_US          2000     /* Maximum pulse width (microseconds) */
#define ESC_PWM_ARM_US          1000     /* Arming pulse width */

/* RTT Command Buffer Size */

#define RTT_BUFFER_SIZE         64
#define RTT_COMMAND_MAX         16

/* PWM Device Paths */

#define PWM_DEVICE_PATH         "/dev/pwm"

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

/* ESC channel configurations */

static struct esc_channel_s g_esc_channels[NUM_ESC_CHANNELS] =
{
  { 0, "/dev/pwm0", -1, 0, false },  /* ESC1 - GPT0 */
  { 1, "/dev/pwm1", -1, 0, false },  /* ESC2 - GPT1 */
  { 2, "/dev/pwm2", -1, 0, false },  /* ESC3 - GPT2 */
  { 3, "/dev/pwm3", -1, 0, false },  /* ESC4 - GPT3 */
};

/* RTT command buffer and status */

static char g_rtt_buffer[RTT_BUFFER_SIZE];
static char g_rtt_command[RTT_COMMAND_MAX];
static volatile bool g_demo_running = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

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
      RTT_PRINTF("ERROR: Failed to open %s: %d\n",
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
      RTT_PRINTF("ERROR: Failed to configure PWM%d: %d\n", esc_index, errno);
      close(g_esc_channels[esc_index].fd);
      g_esc_channels[esc_index].fd = -1;
      return -errno;
    }

  RTT_PRINTF("ESC%d: PWM device opened and configured\n", esc_index + 1);
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
      RTT_PRINTF("ERROR: ESC%d PWM device not open\n", esc_index + 1);
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
      RTT_PRINTF("ERROR: Failed to set PWM duty for ESC%d: %d\n",
             esc_index + 1, errno);
      return -errno;
    }

  /* Start PWM if not already started */
  ret = ioctl(g_esc_channels[esc_index].fd, PWMIOC_START, 0);
  if (ret < 0)
    {
      RTT_PRINTF("ERROR: Failed to start PWM for ESC%d: %d\n",
             esc_index + 1, errno);
      return -errno;
    }

  RTT_PRINTF("ESC%d: Set to %lu µs (duty: %lu%%)\n",
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
      RTT_PRINTF("ESC%d: Armed\n", esc_index + 1);
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
          RTT_PRINTF("ERROR: Failed to stop PWM for ESC%d: %d\n",
                 esc_index + 1, errno);
        }
      else
        {
          g_esc_channels[esc_index].armed = false;
          g_esc_channels[esc_index].current_throttle = 0;
          RTT_PRINTF("ESC%d: Disarmed\n", esc_index + 1);
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

  RTT_PRINTF("Arming all ESCs...\n");

  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      ret = esc_arm(i);
      if (ret < 0)
        {
          RTT_PRINTF("ERROR: Failed to arm ESC%d\n", i + 1);
          break;
        }
      usleep(100000);  /* 100ms delay between arming */
    }

  if (ret == OK)
    {
      RTT_PRINTF("All ESCs armed successfully\n");
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

  RTT_PRINTF("Disarming all ESCs...\n");

  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      esc_disarm(i);
    }

  RTT_PRINTF("All ESCs disarmed\n");
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
  RTT_PRINTF("\n=== RA8E1 ESC PWM Control Demo ===\n");
  RTT_PRINTF("Commands:\n");
  RTT_PRINTF("  help      - Show this menu\n");
  RTT_PRINTF("  status    - Show ESC status\n");
  RTT_PRINTF("  arm       - Arm all ESCs\n");
  RTT_PRINTF("  disarm    - Disarm all ESCs\n");
  RTT_PRINTF("  arm<N>    - Arm ESC N (1-4)\n");
  RTT_PRINTF("  esc<N> <%%> - Set ESC N throttle (0-100%%)\n");
  RTT_PRINTF("  test      - Run automatic test sequence\n");
  RTT_PRINTF("  stop      - Stop demo\n");
  RTT_PRINTF("\nExamples:\n");
  RTT_PRINTF("  esc1 50   - Set ESC1 to 50%% throttle\n");
  RTT_PRINTF("  esc2 0    - Set ESC2 to 0%% throttle\n");
  RTT_PRINTF("  arm1      - Arm ESC1 only\n");
  RTT_PRINTF("\nCurrent Status:\n");
  
  for (int i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      RTT_PRINTF("  ESC%d: %s, %d%% throttle\n", 
             i + 1,
             g_esc_channels[i].armed ? "ARMED" : "DISARMED",
             g_esc_channels[i].current_throttle);
    }
  RTT_PRINTF("\n");
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

  RTT_PRINTF("Command: %s\n", command);

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
      ra8e1_pwm_demo_test();
    }
  else if (strcmp(command, "stop") == 0)
    {
      RTT_PRINTF("Stopping demo...\n");
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
              RTT_PRINTF("ERROR: Throttle value must be 0-100%%\n");
            }
        }
      else if (cmd_type == 2)  /* ARM command */
        {
          esc_arm(esc_index);
        }
      else
        {
          RTT_PRINTF("ERROR: Unknown command '%s'. Type 'help' for commands.\n", 
                 command);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_pwm_demo_main
 *
 * Description:
 *   Main entry point for PWM ESC control demo
 *
 ****************************************************************************/

int ra8e1_pwm_demo_main(int argc, char *argv[])
{
  int ret;
  int i;
  int cmd_len = 0;
  char ch;

  RTT_PRINTF("\n=== RA8E1 ESC PWM Control Demo ===\n");
  RTT_PRINTF("Initializing PWM devices...\n");

#ifdef CONFIG_SEGGER_RTT
  /* Initialize RTT */
  SEGGER_RTT_Init();
  RTT_PRINTF("RTT initialized - ready for commands\n");
#endif

  /* Initialize all ESC channels */
  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      ret = esc_pwm_open(i);
      if (ret < 0)
        {
          RTT_PRINTF("ERROR: Failed to initialize ESC%d: %d\n", i + 1, ret);
          goto cleanup;
        }
    }

  RTT_PRINTF("All PWM devices initialized successfully\n");
  
  /* Safety: Disarm all ESCs initially */
  esc_disarm_all();

  print_menu();

  g_demo_running = true;

  /* Main command loop */
  RTT_PRINTF("Ready for commands (type 'help' for menu):\n> ");
  
  while (g_demo_running)
    {
      /* Check for RTT input */
#ifdef CONFIG_SEGGER_RTT
      if (RTT_HASKEY())
        {
          ch = RTT_GETKEY();
#else
      /* Fallback to standard input for testing */
      if (read(STDIN_FILENO, &ch, 1) == 1)
        {
#endif
          if (ch == '\n' || ch == '\r')
            {
              if (cmd_len > 0)
                {
                  g_rtt_command[cmd_len] = '\0';
                  process_rtt_command(g_rtt_command);
                  cmd_len = 0;
                  RTT_PRINTF("> ");
                }
            }
          else if (ch == '\b' || ch == 127)  /* Backspace */
            {
              if (cmd_len > 0)
                {
                  cmd_len--;
                  RTT_PRINTF("\b \b");
                }
            }
          else if (cmd_len < RTT_COMMAND_MAX - 1)
            {
              g_rtt_command[cmd_len++] = ch;
              RTT_PRINTF("%c", ch);
            }
        }
      
      usleep(10000);  /* 10ms delay */
    }

cleanup:
  /* Cleanup: Disarm all ESCs and close devices */
  RTT_PRINTF("Cleaning up...\n");
  esc_disarm_all();
  
  for (i = 0; i < NUM_ESC_CHANNELS; i++)
    {
      esc_pwm_close(i);
    }

  RTT_PRINTF("Demo stopped\n");
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
 * Name: ra8e1_pwm_demo_test
 *
 * Description:
 *   Simple test sequence for ESC control
 *
 ****************************************************************************/

int ra8e1_pwm_demo_test(void)
{
  int ret;
  int i;

  RTT_PRINTF("Starting ESC test sequence...\n");

  /* Test sequence:
   * 1. Arm all ESCs
   * 2. Gradually increase throttle
   * 3. Gradually decrease throttle
   * 4. Disarm all ESCs
   */

  ret = esc_arm_all();
  if (ret < 0)
    {
      RTT_PRINTF("ERROR: Failed to arm ESCs\n");
      return ret;
    }

  sleep(2);  /* Wait 2 seconds */

  /* Gradually increase throttle to 25% */
  for (i = 0; i <= 25; i += 5)
    {
      RTT_PRINTF("Setting all ESCs to %d%% throttle\n", i);
      
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
      RTT_PRINTF("Setting all ESCs to %d%% throttle\n", i);
      
      for (int j = 0; j < NUM_ESC_CHANNELS; j++)
        {
          esc_set_throttle_percent(j, i);
        }
      
      sleep(1);  /* Wait 1 second between steps */
    }

  sleep(2);  /* Wait 2 seconds */

  /* Disarm all ESCs */
  esc_disarm_all();

  RTT_PRINTF("ESC test sequence completed\n");
  return OK;
}

#endif /* CONFIG_EXAMPLES_ESCS */
