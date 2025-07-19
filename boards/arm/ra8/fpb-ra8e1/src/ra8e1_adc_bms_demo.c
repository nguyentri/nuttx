/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_adc_demo.c
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
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/analog/adc.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "board.h"
#include "ra8e1_demo_log.h"

#ifdef CONFIG_RA8_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
/* ADC Channels *************************************************************/

/* Battery monitoring pins on FPB-RA8E1:
 * - P004 (AN000): Battery voltage through 5.7:1 voltage divider
 * - P003 (AN104): Battery current through ACS712-05B current sensor
 */

#define GPIO_ADC_BATT_VOLTAGE   (PORT_0 | PIN_4)
#define GPIO_ADC_BATT_CURRENT   (PORT_0 | PIN_3)

/* ADC Channel Numbers */

#define ADC_BATTERY_VOLTAGE_CHANNEL    0
#define ADC_BATTERY_CURRENT_CHANNEL    104

/* Board ADC Configuration **************************************************/

#define BOARD_ADC_VREF              3300
#define BOARD_ADC_RESOLUTION        4096

/* ADC Channel Definitions for FPB-RA8E1 */

#define ADC_BATTERY_VOLTAGE_CHANNEL   0   /* AN000 - P004 (Arduino A0) */
#define ADC_BATTERY_CURRENT_CHANNEL   104 /* AN104 - P003 (Arduino A1) */

/* Number of ADC channels used for battery monitoring */

#define BOARD_ADC_CHANNELS           2

/* Battery monitoring constants */

#define BATTERY_VOLTAGE_DIVIDER_RATIO  5.7f  /* 5.7:1 voltage divider */
#define CURRENT_SENSOR_SENSITIVITY    185   /* ACS712-05B: 185mV/A */
#define ADC_VREF_MV                   3300  /* 3.3V reference voltage */
#define ADC_RESOLUTION_BITS          12    /* 12-bit ADC */
#define ADC_MAX_VALUE                ((1 << ADC_RESOLUTION_BITS) - 1)

/* Battery voltage thresholds (in mV) */

#define BATTERY_VOLTAGE_MAX          4200  /* 4.2V fully charged */
#define BATTERY_VOLTAGE_MIN          3200  /* 3.2V discharged */

/* GPIO pin definitions for ADC channels */

#define GPIO_ADC_BATTERY_VOLTAGE     GPIO_P004_AN000  /* P004 as AN000 */
#define GPIO_ADC_BATTERY_CURRENT     GPIO_P003_AN104  /* P003 as AN104 */

/* DTC buffer size for batch ADC transfers */

#define ADC_DTC_BUFFER_SIZE          8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Battery status structure for comprehensive monitoring */

struct battery_status_s
{
  uint32_t voltage_mv;          /* Battery voltage in millivolts */
  int32_t  current_ma;          /* Battery current in milliamps (+ = charging, - = discharging) */
  uint32_t power_mw;            /* Instantaneous power in milliwatts */
  uint8_t  percentage;          /* Battery percentage (0-100) */
  bool     is_charging;         /* True if battery is charging */
  bool     is_valid;            /* True if readings are valid */
};

/* ADC channel configuration structure */

struct adc_chan_s
{
  uint8_t  channel;             /* ADC channel number */
  uint32_t pinset;              /* GPIO pin configuration */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC device handle */

static FAR struct adc_dev_s *g_adc_dev = NULL;

/* ADC channel configuration for FPB-RA8E1 battery monitoring */

static const struct adc_chan_s g_adc_channels[BOARD_ADC_CHANNELS] =
{
  {
    .channel = ADC_BATTERY_VOLTAGE_CHANNEL,
    .pinset  = GPIO_ADC_BATTERY_VOLTAGE,
  },
  {
    .channel = ADC_BATTERY_CURRENT_CHANNEL,
    .pinset  = GPIO_ADC_BATTERY_CURRENT,
  },
};

/* Battery status cache */

static struct battery_status_s g_battery_status;
static bool g_adc_initialized = false;

/* DTC buffer for batch ADC operations */

#ifdef CONFIG_RA8_ADC_DTC
static uint32_t g_adc_buffer[ADC_DTC_BUFFER_SIZE];
static uint8_t g_channel_buffer[ADC_DTC_BUFFER_SIZE];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_adc_setup
 *
 * Description:
 *   Setup ADC peripheral and GPIO pins for battery monitoring
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ra8e1_adc_setup(void)
{
  int ret;
  uint32_t chanlist = 0;

  ainfo("Setting up ADC for battery monitoring\n");

  /* Configure GPIO pins for ADC channels */

  for (int i = 0; i < BOARD_ADC_CHANNELS; i++)
    {
      ret = ra_gpio_config(g_adc_channels[i].pinset);
      if (ret < 0)
        {
          aerr("ERROR: Failed to configure GPIO for ADC channel %d: %d\n",
               g_adc_channels[i].channel, ret);
          return ret;
        }

      /* Build channel list mask */

      chanlist |= (1 << g_adc_channels[i].channel);
      ainfo("Configured ADC channel %d (pin 0x%08lx)\n",
            g_adc_channels[i].channel, g_adc_channels[i].pinset);
    }

  /* Initialize ADC driver */

  g_adc_dev = ra8_adc_initialize(0, chanlist, BOARD_ADC_CHANNELS);
  if (g_adc_dev == NULL)
    {
      aerr("ERROR: Failed to initialize ADC\n");
      return -ENODEV;
    }

  /* Register ADC driver */

  ret = adc_register("/dev/adc0", g_adc_dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to register ADC driver: %d\n", ret);
      return ret;
    }

  ainfo("ADC setup complete\n");
  return OK;
}

/****************************************************************************
 * Name: ra8e1_adc_convert_voltage
 *
 * Description:
 *   Convert ADC reading to actual battery voltage in millivolts
 *
 * Input Parameters:
 *   adc_value - Raw ADC value (12-bit)
 *
 * Returned Value:
 *   Battery voltage in millivolts
 *
 ****************************************************************************/

static uint32_t ra8e1_adc_convert_voltage(uint32_t adc_value)
{
  uint32_t voltage_mv;

  /* Convert ADC value to voltage considering the voltage divider */

  voltage_mv = (adc_value * ADC_VREF_MV) / ADC_MAX_VALUE;
  voltage_mv = (uint32_t)(voltage_mv * BATTERY_VOLTAGE_DIVIDER_RATIO);

  return voltage_mv;
}

/****************************************************************************
 * Name: ra8e1_adc_convert_current
 *
 * Description:
 *   Convert ADC reading to actual battery current in milliamps
 *
 * Input Parameters:
 *   adc_value - Raw ADC value (12-bit)
 *
 * Returned Value:
 *   Battery current in milliamps (positive = charging, negative = discharging)
 *
 ****************************************************************************/

static int32_t ra8e1_adc_convert_current(uint32_t adc_value)
{
  int32_t voltage_mv;
  int32_t current_ma;
  int32_t offset_mv = ADC_VREF_MV / 2;  /* ACS712 offset (VCC/2) */

  /* Convert ADC value to voltage */

  voltage_mv = (adc_value * ADC_VREF_MV) / ADC_MAX_VALUE;

  /* Convert to current using ACS712 sensitivity (185mV/A) */

  current_ma = ((voltage_mv - offset_mv) * 1000) / CURRENT_SENSOR_SENSITIVITY;

  return current_ma;
}

/****************************************************************************
 * Name: ra8e1_calculate_battery_percentage
 *
 * Description:
 *   Calculate battery percentage based on voltage
 *
 * Input Parameters:
 *   voltage_mv - Battery voltage in millivolts
 *
 * Returned Value:
 *   Battery percentage (0-100)
 *
 ****************************************************************************/

static uint8_t ra8e1_calculate_battery_percentage(uint32_t voltage_mv)
{
  if (voltage_mv >= BATTERY_VOLTAGE_MAX)
    {
      return 100;
    }
  else if (voltage_mv <= BATTERY_VOLTAGE_MIN)
    {
      return 0;
    }
  else
    {
      /* Linear interpolation between min and max voltage */

      return (uint8_t)(((voltage_mv - BATTERY_VOLTAGE_MIN) * 100) /
                       (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN));
    }
}

/****************************************************************************
 * Name: ra8e1_adc_read_channel
 *
 * Description:
 *   Read a single ADC channel value
 *
 * Input Parameters:
 *   channel - ADC channel number
 *   value   - Pointer to store the read value
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ra8e1_adc_read_channel(uint8_t channel, uint32_t *value)
{
  struct adc_msg_s sample;
  ssize_t nbytes;
  int fd;
  int ret;

  /* Open ADC device */

  fd = open("/dev/adc0", O_RDONLY);
  if (fd < 0)
    {
      aerr("ERROR: Failed to open ADC device: %d\n", errno);
      return -errno;
    }

  /* Trigger ADC conversion */

  ret = ioctl(fd, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      aerr("ERROR: Failed to trigger ADC conversion: %d\n", errno);
      close(fd);
      return -errno;
    }

  /* Read ADC sample */

  nbytes = read(fd, &sample, sizeof(sample));
  if (nbytes != sizeof(sample))
    {
      aerr("ERROR: Failed to read ADC sample: %zd\n", nbytes);
      close(fd);
      return -EIO;
    }

  /* Verify channel matches */

  if (sample.am_channel != channel)
    {
      awarn("WARNING: Expected channel %d, got %d\n", channel, sample.am_channel);
    }

  *value = sample.am_data;
  close(fd);

  return OK;
}

/****************************************************************************
 * Name: ra8e1_adc_read_battery_data
 *
 * Description:
 *   Read both voltage and current channels and update battery status
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ra8e1_adc_read_battery_data(void)
{
  uint32_t voltage_raw, current_raw;
  int ret;

  /* Read battery voltage channel */

  ret = ra8e1_adc_read_channel(ADC_BATTERY_VOLTAGE_CHANNEL, &voltage_raw);
  if (ret < 0)
    {
      aerr("ERROR: Failed to read voltage channel: %d\n", ret);
      return ret;
    }

  /* Read battery current channel */

  ret = ra8e1_adc_read_channel(ADC_BATTERY_CURRENT_CHANNEL, &current_raw);
  if (ret < 0)
    {
      aerr("ERROR: Failed to read current channel: %d\n", ret);
      return ret;
    }

  /* Convert raw values to actual measurements */

  g_battery_status.voltage_mv = ra8e1_adc_convert_voltage(voltage_raw);
  g_battery_status.current_ma = ra8e1_adc_convert_current(current_raw);
  g_battery_status.percentage = ra8e1_calculate_battery_percentage(g_battery_status.voltage_mv);

  /* Calculate power consumption */

  g_battery_status.power_mw = (g_battery_status.voltage_mv * abs(g_battery_status.current_ma)) / 1000;

  /* Determine charging status */

  g_battery_status.is_charging = (g_battery_status.current_ma > 50);  /* > 50mA considered charging */
  g_battery_status.is_valid = true;

  ainfo("Battery Status: %lu mV, %ld mA, %lu mW, %d%%, %s\n",
        g_battery_status.voltage_mv, g_battery_status.current_ma,
        g_battery_status.power_mw, g_battery_status.percentage,
        g_battery_status.is_charging ? "Charging" : "Discharging");

  return OK;
}

#ifdef CONFIG_RA8_ADC_DTC
/****************************************************************************
 * Name: ra8e1_adc_read_batch_dtc
 *
 * Description:
 *   Read multiple ADC channels using DTC (Data Transfer Controller)
 *
 * Input Parameters:
 *   channels - Array of channel numbers to read
 *   values   - Array to store read values
 *   count    - Number of channels to read
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ra8e1_adc_read_batch_dtc(FAR const uint8_t *channels,
                                    FAR uint32_t *values, size_t count)
{
  int fd;
  int ret;

  if (count > ADC_DTC_BUFFER_SIZE)
    {
      aerr("ERROR: Channel count %zu exceeds buffer size %d\n", count, ADC_DTC_BUFFER_SIZE);
      return -EINVAL;
    }

  /* Open ADC device */

  fd = open("/dev/adc0", O_RDONLY);
  if (fd < 0)
    {
      aerr("ERROR: Failed to open ADC device: %d\n", errno);
      return -errno;
    }

  /* Setup channel list in buffer */

  memcpy(g_channel_buffer, channels, count);

  /* Trigger batch conversion */

  ret = ioctl(fd, ANIOC_TRIGGER, 0);
  if (ret < 0)
    {
      aerr("ERROR: Failed to trigger batch ADC conversion: %d\n", errno);
      close(fd);
      return -errno;
    }

  /* Read batch data - this would be implementation specific for DTC */

  /* For now, fall back to individual reads */

  for (size_t i = 0; i < count; i++)
    {
      ret = ra8e1_adc_read_channel(channels[i], &values[i]);
      if (ret < 0)
        {
          aerr("ERROR: Failed to read channel %d in batch: %d\n", channels[i], ret);
          close(fd);
          return ret;
        }
    }

  close(fd);
  return OK;
}

/****************************************************************************
 * Name: ra8e1_adc_read_battery_data_dtc
 *
 * Description:
 *   Read battery data using DTC for efficient multi-channel sampling
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

static int ra8e1_adc_read_battery_data_dtc(void)
{
  uint8_t channels[2] = {ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL};
  uint32_t values[2];
  int ret;

  /* Read both channels simultaneously using DTC */

  ret = ra8e1_adc_read_batch_dtc(channels, values, 2);
  if (ret < 0)
    {
      aerr("ERROR: Failed to read battery data with DTC: %d\n", ret);
      return ret;
    }

  /* Convert raw values to actual measurements */

  g_battery_status.voltage_mv = ra8e1_adc_convert_voltage(values[0]);
  g_battery_status.current_ma = ra8e1_adc_convert_current(values[1]);
  g_battery_status.percentage = ra8e1_calculate_battery_percentage(g_battery_status.voltage_mv);

  /* Calculate power consumption */

  g_battery_status.power_mw = (g_battery_status.voltage_mv * abs(g_battery_status.current_ma)) / 1000;

  /* Determine charging status */

  g_battery_status.is_charging = (g_battery_status.current_ma > 50);
  g_battery_status.is_valid = true;

  ainfo("Battery Status (DTC): %lu mV, %ld mA, %lu mW, %d%%, %s\n",
        g_battery_status.voltage_mv, g_battery_status.current_ma,
        g_battery_status.power_mw, g_battery_status.percentage,
        g_battery_status.is_charging ? "Charging" : "Discharging");

  return OK;
}
#endif /* CONFIG_RA8_ADC_DTC */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_adc_initialize
 *
 * Description:
 *   Initialize ADC subsystem for battery monitoring on FPB-RA8E1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

void ra8e1_adc_initialize(void)
{
  int ret;

  if (g_adc_initialized)
    {
      ainfo("ADC already initialized\n");
      return;
    }

  ainfo("Initializing FPB-RA8E1 ADC for battery monitoring\n");

  /* Setup ADC hardware and drivers */

  ret = ra8e1_adc_setup();
  if (ret < 0)
    {
      aerr("ERROR: ADC setup failed: %d\n", ret);
      return;
    }

  /* Initialize battery status structure */

  memset(&g_battery_status, 0, sizeof(g_battery_status));

  /* Perform initial battery reading */

#ifdef CONFIG_RA8_ADC_DTC
  ret = ra8e1_adc_read_battery_data_dtc();
#else
  ret = ra8e1_adc_read_battery_data();
#endif

  if (ret < 0)
    {
      awarn("WARNING: Initial battery reading failed: %d\n", ret);
    }

  g_adc_initialized = true;
  ainfo("FPB-RA8E1 ADC initialization complete\n");
}

/****************************************************************************
 * Name: ra8e1_read_battery_voltage
 *
 * Description:
 *   Read battery voltage in millivolts
 *
 * Input Parameters:
 *   voltage_mv - Pointer to store voltage value
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_read_battery_voltage(uint32_t *voltage_mv)
{
  uint32_t raw_value;
  int ret;

  if (!g_adc_initialized)
    {
      aerr("ERROR: ADC not initialized\n");
      return -ENODEV;
    }

  if (voltage_mv == NULL)
    {
      return -EINVAL;
    }

  ret = ra8e1_adc_read_channel(ADC_BATTERY_VOLTAGE_CHANNEL, &raw_value);
  if (ret < 0)
    {
      return ret;
    }

  *voltage_mv = ra8e1_adc_convert_voltage(raw_value);
  return OK;
}

/****************************************************************************
 * Name: ra8e1_read_battery_current
 *
 * Description:
 *   Read battery current in milliamps
 *
 * Input Parameters:
 *   current_ma - Pointer to store current value
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_read_battery_current(int32_t *current_ma)
{
  uint32_t raw_value;
  int ret;

  if (!g_adc_initialized)
    {
      aerr("ERROR: ADC not initialized\n");
      return -ENODEV;
    }

  if (current_ma == NULL)
    {
      return -EINVAL;
    }

  ret = ra8e1_adc_read_channel(ADC_BATTERY_CURRENT_CHANNEL, &raw_value);
  if (ret < 0)
    {
      return ret;
    }

  *current_ma = ra8e1_adc_convert_current(raw_value);
  return OK;
}

/****************************************************************************
 * Name: ra8e1_get_battery_status
 *
 * Description:
 *   Get complete battery status information
 *
 * Input Parameters:
 *   status - Pointer to store battery status
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_get_battery_status(struct battery_status_s *status)
{
  int ret;

  if (!g_adc_initialized)
    {
      aerr("ERROR: ADC not initialized\n");
      return -ENODEV;
    }

  if (status == NULL)
    {
      return -EINVAL;
    }

  /* Update battery data */

#ifdef CONFIG_RA8_ADC_DTC
  ret = ra8e1_adc_read_battery_data_dtc();
#else
  ret = ra8e1_adc_read_battery_data();
#endif

  if (ret < 0)
    {
      return ret;
    }

  /* Copy status to output structure */

  memcpy(status, &g_battery_status, sizeof(struct battery_status_s));
  return OK;
}

/****************************************************************************
 * Name: ra8e1_adc_sample_demo
 *
 * Description:
 *   Demonstration function showing ADC usage for battery monitoring
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_adc_sample_demo(void)
{
  struct battery_status_s status;
  int ret;
  int sample_count = 10;

  demoprintf("\n=== FPB-RA8E1 ADC Battery Monitoring Demo ===\n");

  /* Initialize ADC if not already done */

  if (!g_adc_initialized)
    {
      ra8e1_adc_initialize();
    }

  demoprintf("Taking %d battery measurements...\n\n", sample_count);

  for (int i = 0; i < sample_count; i++)
    {
      ret = ra8e1_get_battery_status(&status);
      if (ret < 0)
        {
          demoprintf("Sample %d: ERROR reading battery status (%d)\n", i + 1, ret);
          continue;
        }

      demoprintf("Sample %d:\n", i + 1);
      demoprintf("  Voltage:    %lu mV\n", status.voltage_mv);
      demoprintf("  Current:    %ld mA %s\n", abs(status.current_ma),
             status.is_charging ? "(Charging)" : "(Discharging)");
      demoprintf("  Power:      %lu mW\n", status.power_mw);
      demoprintf("  Percentage: %d%%\n", status.percentage);
      demoprintf("  Status:     %s\n\n", status.is_valid ? "Valid" : "Invalid");

      /* Wait between samples */

      usleep(500000);  /* 500ms delay */
    }

  demoprintf("=== Demo Complete ===\n");
  return OK;
}

#endif /* CONFIG_RA8_ADC */



#ifdef CONFIG_RA8E1_ADC_BMS_DEMO
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_BATTERY_PROGNAME "adc_battery"
#define ADC_BATTERY_STACKSIZE 2048
#define ADC_BATTERY_PRIORITY 100

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_usage
 ****************************************************************************/

static void show_usage(FAR const char *progname, int exitcode)
{
  demoprintf("USAGE: %s [OPTIONS]\n", progname);
  demoprintf("\nWhere OPTIONS include:\n");
  demoprintf("  -h      Show this help message and exit\n");
  demoprintf("  -c <n>  Number of samples to take (default: 10)\n");
  demoprintf("  -d <ms> Delay between samples in milliseconds (default: 1000)\n");
  demoprintf("  -v      Verbose output\n");
  demoprintf("  -s      Single sample mode\n");
  demoprintf("  -m      Continuous monitoring mode\n");
  demoprintf("\nExamples:\n");
  demoprintf("  %s -c 5 -d 500    Take 5 samples with 500ms delay\n", progname);
  demoprintf("  %s -s             Take a single sample\n", progname);
  demoprintf("  %s -m             Continuous monitoring (Ctrl+C to stop)\n", progname);
  exit(exitcode);
}

/****************************************************************************
 * Name: print_battery_status
 ****************************************************************************/

#ifdef CONFIG_RA8_ADC_BATTERY_MONITOR
static void print_battery_status(FAR struct battery_status_s *status, bool verbose)
{
  const char *charge_level = "Unknown";
  
  /* Determine charge level description */
  
  if (status->percentage >= 80)
    {
      charge_level = "High";
    }
  else if (status->percentage >= 50)
    {
      charge_level = "Medium";
    }
  else if (status->percentage >= 20)
    {
      charge_level = "Low";
    }
  else
    {
      charge_level = "Critical";
    }

  if (verbose)
    {
      demoprintf("=== Battery Status ===\n");
      demoprintf("Voltage:     %lu mV\n", status->voltage_mv);
      demoprintf("Current:     %ld mA (%s)\n", 
             abs(status->current_ma),
             status->is_charging ? "Charging" : "Discharging");
      demoprintf("Power:       %lu mW\n", status->power_mw);
      demoprintf("Charge:      %d%% (%s)\n", status->percentage, charge_level);
      demoprintf("Status:      %s\n", status->is_valid ? "Valid" : "Invalid");
      demoprintf("Charging:    %s\n", status->is_charging ? "Yes" : "No");
      demoprintf("======================\n");
    }
  else
    {
      demoprintf("%lu mV | %ld mA | %lu mW | %d%% | %s\n",
             status->voltage_mv, status->current_ma, status->power_mw,
             status->percentage, status->is_charging ? "CHG" : "DIS");
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_adc_bms_demo_init
 *
 * Description:
 *   Initialize the ADC BMS demo
 *
 ****************************************************************************/

int ra8e1_adc_bms_demo_init(void)
{
  /* Initialize ADC for battery monitoring */
  return battery_adc_initialize();
}

/****************************************************************************
 * Name: ra8e1_adc_bms_demo_main
 *
 * Description:
 *   ADC Battery Monitoring Demo main function
 *
 ****************************************************************************/

int ra8e1_adc_bms_demo_main(int argc, FAR char *argv[])
{
#ifdef CONFIG_RA8_ADC_BATTERY_MONITOR
  struct battery_status_s status;
  int sample_count = 10;
  int delay_ms = 1000;
  bool verbose = false;
  bool single_mode = false;
  bool continuous_mode = false;
  int option;
  int ret;
  int i;

  /* Parse command line arguments */

  while ((option = getopt(argc, argv, "hc:d:vsm")) != ERROR)
    {
      switch (option)
        {
          case 'h':
            show_usage(argv[0], EXIT_SUCCESS);
            break;

          case 'c':
            sample_count = atoi(optarg);
            if (sample_count <= 0)
              {
                fdemoprintf(stderr, "Error: Sample count must be positive\n");
                show_usage(argv[0], EXIT_FAILURE);
              }
            break;

          case 'd':
            delay_ms = atoi(optarg);
            if (delay_ms < 0)
              {
                fdemoprintf(stderr, "Error: Delay must be non-negative\n");
                show_usage(argv[0], EXIT_FAILURE);
              }
            break;

          case 'v':
            verbose = true;
            break;

          case 's':
            single_mode = true;
            break;

          case 'm':
            continuous_mode = true;
            break;

          case '?':
          default:
            fdemoprintf(stderr, "Error: Unknown option\n");
            show_usage(argv[0], EXIT_FAILURE);
            break;
        }
    }

  /* Validate mode combinations */

  if (single_mode && continuous_mode)
    {
      fdemoprintf(stderr, "Error: Cannot use both single and continuous mode\n");
      show_usage(argv[0], EXIT_FAILURE);
    }

  demoprintf("FPB-RA8E1 ADC Battery Monitor\n");
  demoprintf("=====================================\n");

  /* Initialize ADC */

  ra8e1_adc_initialize();

  if (single_mode)
    {
      /* Single sample mode */

      demoprintf("Taking single battery measurement...\n");
      ret = ra8e1_get_battery_status(&status);
      if (ret < 0)
        {
          fdemoprintf(stderr, "Error: Failed to read battery status: %d\n", ret);
          return EXIT_FAILURE;
        }

      print_battery_status(&status, true);  /* Always verbose for single mode */
    }
  else if (continuous_mode)
    {
      /* Continuous monitoring mode */

      demoprintf("Continuous monitoring mode (Ctrl+C to stop)\n");
      if (!verbose)
        {
          demoprintf("Format: Voltage | Current | Power | Charge | Status\n");
          demoprintf("--------|--------|-------|-------|--------\n");
        }

      i = 0;
      while (true)
        {
          ret = ra8e1_get_battery_status(&status);
          if (ret < 0)
            {
              fdemoprintf(stderr, "Error: Failed to read battery status: %d\n", ret);
              usleep(delay_ms * 1000);
              continue;
            }

          if (verbose)
            {
              demoprintf("\n--- Sample %d ---\n", ++i);
            }
          else
            {
              demoprintf("%3d: ", ++i);
            }

          print_battery_status(&status, verbose);

          usleep(delay_ms * 1000);
        }
    }
  else
    {
      /* Multiple sample mode */

      demoprintf("Taking %d battery measurements with %dms delay...\n", 
             sample_count, delay_ms);

      if (!verbose)
        {
          demoprintf("Sample | Voltage | Current | Power | Charge | Status\n");
          demoprintf("-------|---------|---------|-------|--------|--------\n");
        }

      for (i = 0; i < sample_count; i++)
        {
          ret = ra8e1_get_battery_status(&status);
          if (ret < 0)
            {
              fdemoprintf(stderr, "Sample %d: Failed to read battery status: %d\n", 
                      i + 1, ret);
              continue;
            }

          if (verbose)
            {
              demoprintf("\n--- Sample %d of %d ---\n", i + 1, sample_count);
            }
          else
            {
              demoprintf("%3d:   ", i + 1);
            }

          print_battery_status(&status, verbose);

          if (i < sample_count - 1)
            {
              usleep(delay_ms * 1000);
            }
        }
    }

  demoprintf("\nBattery monitoring complete.\n");
  return EXIT_SUCCESS;

#else
  demoprintf("Error: ADC battery monitoring not enabled in configuration\n");
  demoprintf("Please enable CONFIG_RA8_ADC_BATTERY_MONITOR\n");
  return EXIT_FAILURE;
#endif
}

#endif /* CONFIG_RA8E1_ADC_BMS_DEMO */
