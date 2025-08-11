/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_bringup.c
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>

#include "fpb-ra8e1.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#undef HAVE_LEDS

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
#  define HAVE_LEDS 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int ra8e1_bringup(void)
{
  int ret = OK;

  syslog(LOG_INFO, "RA8E1 Board bring-up starting...\n");

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Mounted procfs at /proc\n");
    }
#endif

#ifdef CONFIG_RA_SCI_UART
  /* Initialize UART drivers */

  ret = ra8e1_uart_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize UART: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "UART initialized successfully\n");
    }
#endif

#ifdef HAVE_LEDS
  /* Initialize LED support */

  board_userled_initialize();

  /* Register the LED driver */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "LED driver initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_GPIO
  /* Initialize GPIO drivers */

  ret = ra8e1_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize GPIO: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "GPIO drivers initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_ADC_BMS_DEMO
  /* Initialize ADC BMS demo */
  ret = ra8e1_adc_bms_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ADC BMS demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "ADC BMS demo initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_CODE_FLASH_DEMO
  /* Initialize Code Flash */
  ret = ra8e1_code_flash_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Code Flash: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "Code Flash initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_DATA_FLASH_DEMO
  /* Initialize Data Flash */
  ret = ra8e1_data_flash_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Data Flash: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "Data Flash initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_PWM_ESCS_DEMO
  /* Initialize ESCs demo */
  ret = ra8e1_pwm_escs_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize ESCs demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "ESCs demo initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_GPS_DEMO
  /* Initialize GPS demo */
  ret = ra8e1_gps_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize GPS demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "GPS demo initialized successfully\n");
    }
#endif
#ifdef CONFIG_RA8E1_SBUS_DEMO
  /* Initialize SBUS demo */
  ret = ra8e1_sbus_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SBUS demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "SBUS demo initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_I2C_GY912_DEMO
  /* Initialize I2C GY-912 demo */
  ret = ra8e1_i2c_gy912_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C GY-912 demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "I2C GY-912 demo initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_I2C_ACC_DEMO
  /* Initialize I2C ACC demo */
  ret = ra8e1_i2c_acc_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C ACC demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "I2C ACC demo initialized successfully\n");
    }

#ifdef CONFIG_RA8E1_I2C_TEST
  /* Initialize I2C Simple demo */
  ret = ra8e1_i2c_test_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C Simple demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "I2C Simple demo initialized successfully\n");
    }
#endif
#endif

#ifdef CONFIG_RA_SPI
  /* Initialize SPI */
  ret = fpb_ra8e1_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI: %d\n", ret);
      return ret;
    }
  else
    {
      syslog(LOG_INFO, "SPI initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_SPI_LOOPBACK_DEMO
  /* Initialize SPI loopback demo */
  ret = ra8e1_spi_loopback_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI loopback demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "SPI loopback demo initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_SPI_GY912_DEMO
  /* Initialize SPI GY-912 demo */
  ret = ra8e1_spi_gy912_demo_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI GY-912 demo: %d\n", ret);
      /* Don't return error, continue with other initialization */
    }
  else
    {
      syslog(LOG_INFO, "SPI GY-912 demo initialized successfully\n");
    }
#endif

#ifdef CONFIG_RA8E1_SWITCH
  /* Initialize buttons */
  board_button_initialize();
#endif

  /* Suppress unused variable warning */
  (void)ret;
  return 0;
}
