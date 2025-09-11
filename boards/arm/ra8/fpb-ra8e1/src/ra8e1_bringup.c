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

#include <syslog.h>
#include <debug.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>

#include <arch/board/board.h>

#include "fpb-ra8e1.h"

/****************************************************************************
 * Rust Data Structures (mirrored from Rust)
 ****************************************************************************/

typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint64_t timestamp;
    float battery_voltage;
    uint32_t status_flags;
} shared_sensor_data_t;

typedef struct {
    float avg_temperature;
    int32_t trend_direction;  // -1: decreasing, 0: stable, 1: increasing
    uint32_t alert_level;     // 0: normal, 1: warning, 2: critical
    uint32_t data_quality_score; // 0-100
} rust_processed_data_t;

typedef struct {
    uint32_t score;
    uint32_t issues;
    uint32_t battery_status;
    uint32_t temperature_status;
} health_data_t;

typedef struct {
    uint32_t checksum;
    uint32_t vowel_count;
    uint32_t length;
    uint32_t status;
} string_process_result_t;

typedef struct {
    int32_t average;
    int32_t min_val;
    int32_t max_val;
    uint32_t count;
} array_process_result_t;

typedef union {
    shared_sensor_data_t sensor;
    rust_processed_data_t processed;
    health_data_t health;
    string_process_result_t string_result;
    array_process_result_t array_result;
} message_data_t;

typedef struct {
    uint32_t msg_type;
    uint64_t timestamp;
    message_data_t data;
} queue_message_t;

/* Message types */
#define MSG_TYPE_SENSOR_DATA    1
#define MSG_TYPE_PROCESSED_DATA 2
#define MSG_TYPE_HEALTH_CHECK   3
#define MSG_TYPE_STRING_PROCESS 4
#define MSG_TYPE_ARRAY_PROCESS  5

/* Global message queue descriptors */
static mqd_t g_rust_to_c_queue = -1;
static mqd_t g_c_to_rust_queue = -1;

/****************************************************************************
 * External Function Declarations (Rust functions)
 ****************************************************************************/

extern int rust_calculate(int a, int b);
extern int rust_advanced_math(int x, int y, unsigned char operation);
extern int rust_system_init(void);

/* Legacy Rust functions (kept for backward compatibility) */
extern int rust_store_sensor_data(float temp, float humidity, float pressure,
                                  uint64_t timestamp, float battery);
extern const rust_processed_data_t* rust_process_sensor_data(void);
extern const shared_sensor_data_t* rust_get_sensor_data(void);
extern int rust_validate_sensor_data(float temp, float humidity, float pressure);
extern uint32_t rust_system_health_check(void);
extern uint32_t rust_process_string(const uint8_t* input, size_t len);
extern int32_t rust_process_array(const int32_t* data, size_t len);

/* New thread-safe Rust functions using message queues */
extern int rust_set_queues(int rust_to_c, int c_to_rust);
extern int rust_send_sensor_data(float temp, float humidity, float pressure,
                                 uint64_t timestamp, float battery);
extern int rust_send_processed_data(void);
extern int rust_send_health_check(void);
extern int rust_receive_message(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: init_thread_safe_queues
 *
 * Description:
 *   Initialize message queues for thread-safe communication between C and Rust
 *
 ****************************************************************************/

static int init_thread_safe_queues(void)
{
  struct mq_attr attr;

#ifndef CONFIG_DISABLE_MQUEUE
  /* Configure message queue attributes */
  attr.mq_maxmsg = 16;  /* Maximum number of messages in queue */
  attr.mq_msgsize = sizeof(queue_message_t);  /* Maximum message size */
  attr.mq_flags = 0;    /* Blocking queue */
  attr.mq_curmsgs = 0;  /* Number of messages currently in queue */

  /* Create Rust-to-C message queue */
  g_rust_to_c_queue = mq_open("/rust_to_c", O_CREAT | O_RDWR, 0666, &attr);
  if (g_rust_to_c_queue < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to create Rust-to-C message queue: %d (errno: %d)\n",
             g_rust_to_c_queue, errno);
      syslog(LOG_ERR, "ERROR: Message queue support may not be enabled in NuttX configuration\n");
      syslog(LOG_INFO, "INFO: Falling back to legacy Rust-C communication without queues\n");
      g_rust_to_c_queue = -1;
      g_c_to_rust_queue = -1;
      return 0;  /* Continue without message queues */
    }

  /* Create C-to-Rust message queue */
  g_c_to_rust_queue = mq_open("/c_to_rust", O_CREAT | O_RDWR, 0666, &attr);
  if (g_c_to_rust_queue < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to create C-to-Rust message queue: %d (errno: %d)\n",
             g_c_to_rust_queue, errno);
      mq_close(g_rust_to_c_queue);
      g_rust_to_c_queue = -1;
      g_c_to_rust_queue = -1;
      syslog(LOG_INFO, "INFO: Falling back to legacy Rust-C communication without queues\n");
      return 0;  /* Continue without message queues */
    }

  /* Pass queue descriptors to Rust */
  rust_set_queues(g_rust_to_c_queue, g_c_to_rust_queue);

  syslog(LOG_INFO, "Thread-safe message queues initialized successfully\n");
  syslog(LOG_INFO, "Rust-to-C queue: %d, C-to-Rust queue: %d\n",
         g_rust_to_c_queue, g_c_to_rust_queue);
#else
  syslog(LOG_WARNING, "WARNING: POSIX message queues disabled in configuration\n");
  syslog(LOG_INFO, "INFO: Using legacy Rust-C communication without message queues\n");
  g_rust_to_c_queue = -1;
  g_c_to_rust_queue = -1;
#endif

  return 0;
}

/****************************************************************************
 * Name: cleanup_thread_safe_queues
 *
 * Description:
 *   Cleanup message queues
 *
 ****************************************************************************/

static void cleanup_thread_safe_queues(void)
{
  if (g_rust_to_c_queue >= 0)
    {
      mq_close(g_rust_to_c_queue);
      mq_unlink("/rust_to_c");
    }

  if (g_c_to_rust_queue >= 0)
    {
      mq_close(g_c_to_rust_queue);
      mq_unlink("/c_to_rust");
    }
}

/****************************************************************************
 * Name: simulate_sensor_reading
 *
 * Description:
 *   Simulate sensor readings for demonstration
 *
 ****************************************************************************/

static void simulate_sensor_reading(float *temp, float *humidity, float *pressure, float *battery)
{
  static float base_temp = 25.0f;
  static float base_humidity = 50.0f;
  static float base_pressure = 1013.25f;
  static float base_battery = 4.2f;

  /* Add some variation to simulate real sensor readings */
  *temp = base_temp + (float)(rand() % 20 - 10) * 0.1f;
  *humidity = base_humidity + (float)(rand() % 40 - 20) * 0.1f;
  *pressure = base_pressure + (float)(rand() % 100 - 50) * 0.1f;
  *battery = base_battery - (float)(rand() % 20) * 0.01f;

  /* Ensure reasonable ranges */
  if (*humidity < 0.0f) *humidity = 0.0f;
  if (*humidity > 100.0f) *humidity = 100.0f;
  if (*battery < 3.0f) *battery = 3.0f;
}

/****************************************************************************
 * Name: ra8e1_cyclic_rust_thread
 *
 * Description:
 *   Enhanced cyclic rust thread with thread-safe message queue communication
 *
 ****************************************************************************/

static int ra8e1_cyclic_rust_thread(int argc, char *argv[])
{
  struct timespec start_time;
  uint32_t counter = 0;

  syslog(LOG_INFO, "[Rust Thread Init] Thread-safe Rust Thread started\n");

  /* Initialize Rust subsystem */
  syslog(LOG_INFO, "[Rust Thread Init] Initializing Rust Subsystem...\n");
  rust_system_init();

  /* Get start time for reference */
  clock_gettime(CLOCK_REALTIME, &start_time);

  while (1)
    {
      struct timespec current_time;
      clock_gettime(CLOCK_REALTIME, &current_time);

      /* Calculate elapsed time since start */
      long elapsed_sec = current_time.tv_sec - start_time.tv_sec;
      long elapsed_nsec = current_time.tv_nsec - start_time.tv_nsec;

      if (elapsed_nsec < 0)
        {
          elapsed_sec--;
          elapsed_nsec += 1000000000;
        }

      double elapsed_total = elapsed_sec + elapsed_nsec / 1000000000.0;

      /* Print log message with timing information */
      syslog(LOG_INFO, "[Rust Thread] Count: %lu, Elapsed: %.3fs\n",
             (unsigned long)counter, elapsed_total);

      /* Check for messages from C thread */
      if (g_c_to_rust_queue >= 0)
        {
          rust_receive_message();
        }

      /* Simulate and store sensor data every cycle using thread-safe queues */
      float temp, humidity, pressure, battery;
      simulate_sensor_reading(&temp, &humidity, &pressure, &battery);

      /* Validate sensor data using Rust */
      if (rust_validate_sensor_data(temp, humidity, pressure))
        {
          if (g_rust_to_c_queue >= 0)
            {
              /* Send validated data via message queue */
              rust_send_sensor_data(temp, humidity, pressure,
                                   (uint64_t)current_time.tv_sec, battery);
            }
          else
            {
              /* Use legacy method when queues are not available */
              rust_store_sensor_data(temp, humidity, pressure,
                                    (uint64_t)current_time.tv_sec, battery);
            }
        }

      /* Call Rust calculation functions every few iterations */
      if (counter % 3 == 0)
        {
          int calc_result = rust_calculate((int)counter, 5);
          syslog(LOG_INFO, "[Rust Application] Basic calculation result: %d\n", calc_result);
        }

      if (counter % 5 == 0 && counter > 0)
        {
          /* Test different math operations */
          int operations[] = {0, 1, 2, 3, 4, 5, 6}; // All available operations
          int op = operations[counter % 7];
          int math_result = rust_advanced_math((int)counter, 3, op);
          syslog(LOG_INFO, "[Rust Application] Advanced math (op=%d) result: %d\n", op, math_result);
        }

      /* Process sensor data and send analytics every 4th iteration */
      if (counter % 4 == 0)
        {
          if (g_rust_to_c_queue >= 0)
            {
              rust_send_processed_data();
            }
          else
            {
              /* Use legacy method when queues are not available */
              const rust_processed_data_t* processed = rust_process_sensor_data();
              syslog(LOG_INFO, "[Rust Legacy] Processed data: AvgT=%.1f, Trend=%d, Alert=%d\n",
                     processed->avg_temperature, processed->trend_direction, processed->alert_level);
            }
        }

      /* System health check every 6th iteration */
      if (counter % 6 == 0)
        {
          if (g_rust_to_c_queue >= 0)
            {
              rust_send_health_check();
            }
          else
            {
              /* Use legacy method when queues are not available */
              uint32_t health = rust_system_health_check();
              syslog(LOG_INFO, "[Rust Legacy] Health check result: 0x%08X\n", health);
            }
        }

      /* Test string processing every 8th iteration */
      if (counter % 8 == 0)
        {
          const char *test_string = "Hello from C to Rust!";
          uint32_t string_result = rust_process_string((const uint8_t*)test_string, strlen(test_string));
          uint32_t checksum = (string_result >> 16) & 0xFFFF;
          uint32_t vowels = string_result & 0xFFFF;
          syslog(LOG_INFO, "[Rust String] Checksum: %u, Vowels: %u\n", checksum, vowels);
        }

      /* Test array processing every 10th iteration */
      if (counter % 10 == 0 && counter > 0)
        {
          int32_t test_array[] = {1, 2, 3, 4, 5, (int32_t)counter};
          int32_t avg_result = rust_process_array(test_array, sizeof(test_array)/sizeof(test_array[0]));
          syslog(LOG_INFO, "[Rust Array] Average of test array: %d\n", avg_result);
        }

      counter++;
      sleep(5);
    }

  return 0;
}

/****************************************************************************
 * Name: ra8e1_cyclic_c_thread
 *
 * Description:
 *   Enhanced C thread that receives data from Rust via thread-safe message queues
 *
 ****************************************************************************/

static int ra8e1_cyclic_c_thread(int argc, char *argv[])
{
  struct timespec start_time;
  uint32_t counter = 0;
  queue_message_t msg;
  unsigned int prio;

  syslog(LOG_INFO, "[C Thread Init] Thread-safe C Thread started\n");

  /* Get start time for reference */
  clock_gettime(CLOCK_REALTIME, &start_time);

  while (1)
    {
      struct timespec current_time;
      clock_gettime(CLOCK_REALTIME, &current_time);

      /* Calculate elapsed time since start */
      long elapsed_sec = current_time.tv_sec - start_time.tv_sec;
      long elapsed_nsec = current_time.tv_nsec - start_time.tv_nsec;

      if (elapsed_nsec < 0)
        {
          elapsed_sec--;
          elapsed_nsec += 1000000000;
        }

      double elapsed_total = elapsed_sec + elapsed_nsec / 1000000000.0;

      /* Print log message with timing information */
      syslog(LOG_INFO, "[C Thread] Count: %lu, Elapsed: %.3fs\n",
             (unsigned long)counter, elapsed_total);

      /* Check for messages from Rust thread (non-blocking) */
      if (g_rust_to_c_queue >= 0)
        {
          struct timespec timeout = {0, 100000000}; /* 100ms timeout */
          ssize_t result = mq_timedreceive(g_rust_to_c_queue, (char*)&msg,
                                          sizeof(queue_message_t), &prio, &timeout);

          if (result > 0)
            {
              /* Process received message based on type */
              switch (msg.msg_type)
                {
                  case MSG_TYPE_SENSOR_DATA:
                    {
                      shared_sensor_data_t sensor_data = msg.data.sensor;
                      syslog(LOG_INFO, "[C Reading Queue] Sensor Data - Temp: %.1f°C, "
                                      "Humidity: %.1f%%, Pressure: %.1fhPa, Battery: %.2fV\n",
                             sensor_data.temperature, sensor_data.humidity,
                             sensor_data.pressure, sensor_data.battery_voltage);

                      /* Demonstrate C-side processing of Rust data */
                      if (sensor_data.temperature > 30.0f)
                        {
                          syslog(LOG_WARNING, "[C Alert] High temperature detected: %.1f°C\n",
                                 sensor_data.temperature);
                        }

                      if (sensor_data.battery_voltage < 3.5f)
                        {
                          syslog(LOG_WARNING, "[C Alert] Low battery detected: %.2fV\n",
                                 sensor_data.battery_voltage);
                        }
                      break;
                    }

                  case MSG_TYPE_PROCESSED_DATA:
                    {
                      rust_processed_data_t processed = msg.data.processed;
                      const char *trend_str = (processed.trend_direction == 1) ? "Rising" :
                                             (processed.trend_direction == -1) ? "Falling" : "Stable";

                      syslog(LOG_INFO, "[C Queue Analytics] AvgTemp: %.1f°C, Trend: %s, "
                                      "Alert: %u, Quality: %u%%\n",
                             processed.avg_temperature, trend_str,
                             processed.alert_level, processed.data_quality_score);

                      if (processed.alert_level >= 2)
                        {
                          syslog(LOG_ERR, "[C CRITICAL] System requires immediate attention!\n");
                        }
                      break;
                    }

                  case MSG_TYPE_HEALTH_CHECK:
                    {
                      health_data_t health = msg.data.health;
                      syslog(LOG_INFO, "[C Queue Health] Score: %u%%, Issues: 0x%08X, "
                                      "Battery: %u, Temperature: %u\n",
                             health.score, health.issues,
                             health.battery_status, health.temperature_status);

                      if (health.score < 70)
                        {
                          syslog(LOG_WARNING, "[C Health Warning] System health degraded\n");
                        }
                      break;
                    }

                  default:
                    syslog(LOG_WARNING, "[C Thread] Unknown message type: %u\n", msg.msg_type);
                    break;
                }
            }
          else if (result < 0 && errno != ETIMEDOUT)
            {
              syslog(LOG_ERR, "[C Thread] Error receiving message: %d\n", errno);
            }

          /* Send occasional test messages to Rust thread */
          if (counter % 7 == 0 && counter > 0)
            {
              queue_message_t test_msg;
              test_msg.msg_type = MSG_TYPE_STRING_PROCESS;
              test_msg.timestamp = (uint64_t)current_time.tv_sec;

              /* Send test message to Rust thread */
              if (mq_send(g_c_to_rust_queue, (const char*)&test_msg,
                         sizeof(queue_message_t), 0) == 0)
                {
                  syslog(LOG_INFO, "[C Thread] Sent test message to Rust\n");
                }
            }
        }
      else
        {
          /* Legacy mode - use direct function calls to get Rust data */
          if (counter % 3 == 0)
            {
              const shared_sensor_data_t* sensor_data = rust_get_sensor_data();
              if (sensor_data)
                {
                  syslog(LOG_INFO, "[C Legacy] Sensor Data - Temp: %.1f°C, "
                                  "Humidity: %.1f%%, Battery: %.2fV\n",
                         sensor_data->temperature, sensor_data->humidity,
                         sensor_data->battery_voltage);
                }
            }

          if (counter % 5 == 0)
            {
              const rust_processed_data_t* processed = rust_process_sensor_data();
              if (processed)
                {
                  syslog(LOG_INFO, "[C Legacy] Processed - AvgTemp: %.1f°C, Alert: %d\n",
                         processed->avg_temperature, processed->alert_level);
                }
            }
        }

      counter++;
      sleep(2);
    }

  return 0;
}

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
  int ret = 0;

  syslog(LOG_INFO, "Nuttx: RA8E1 Board bring-up with thread-safe Rust-C integration...\n");

  /* Initialize thread-safe message queues first */
  ret = init_thread_safe_queues();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize message queues: %d\n", ret);
      return ret;
    }

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

#ifdef CONFIG_RTC_DRIVER
  /* Initialize RTC driver */

  ret = board_rtc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize RTC: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "RTC initialized successfully\n");
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

#ifdef CONFIG_ARCH_BUTTONS
  /* Initialize buttons */
  board_button_initialize();
#endif

  /* Auto-start the cyclic rust task since UART RX is not available */
  syslog(LOG_INFO, "Starting Rust Thread automatically...\n");

  /* Use task_create so it builds across configurations where kthread_create
   * may not be available/enabled.
   */
  ret = kthread_create("cyclic_rust_thread",
                       100,
                       8192,
                       ra8e1_cyclic_rust_thread,
                       NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start cyclic rust thread: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Cyclic Rust Thread started successfully (PID: %d)\n", ret);
    }

  /* Auto-start the cyclic c task */
  syslog(LOG_INFO, "Starting C Thread automatically...\n");

  ret = kthread_create("cyclic_c_rust_thread",
                       100,
                       8192,
                       ra8e1_cyclic_c_thread,
                       NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start cyclic C thread: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "C Thread started successfully (PID: %d)\n", ret);
    }

  syslog(LOG_INFO, "Nuttx: RA8E1 Board bring-up is successful...\n");

  /* Suppress unused variable warning */
  (void)ret;
  return 0;
}
