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
#include <math.h>

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
#ifdef CONFIG_RA8E1_RUST_DEMO
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

extern int rust_gaussian_filter_imu(float accel_x, float accel_y, float accel_z,
                                    float gyro_x, float gyro_y, float gyro_z);
extern int rust_complementary_filter(uint64_t dt_us, unsigned char filter_type);
extern int rust_pid_controller(float setpoint, float measured_value, uint32_t dt_ms, unsigned char controller_type);
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
  *humidity = base_humidity + (float)(rand() % 10 - 5) * 0.5f;
  *pressure = base_pressure + (float)(rand() % 50 - 25) * 0.2f;
  *battery = base_battery - (float)(rand() % 10) * 0.05f;

  /* Ensure reasonable ranges and prevent zero/extreme values */
  if (*humidity < 30.0f) *humidity = 30.0f;
  if (*humidity > 80.0f) *humidity = 80.0f;
  if (*pressure < 950.0f) *pressure = 950.0f;
  if (*battery < 3.5f) *battery = 3.5f;
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

      /* Check for messages from C thread every 2nd iteration to reduce overhead */
      if (g_c_to_rust_queue >= 0 && counter % 2 == 0)
        {
          rust_receive_message();
        }

      /* Simulate and store sensor data every 3rd cycle to reduce processing load */
      if (counter % 3 == 0)
        {
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
        }

      /* Apply Gaussian filter to IMU data every 6 iterations instead of 3 */
      if (counter % 6 == 0)
        {
          /* Simulate IMU readings for demonstration */
          float sim_time = counter * 0.01f; // 10ms cycle time
          float accel_x = 0.5f * sinf(sim_time * 0.5f) + 0.1f * sinf(sim_time * 3.0f);
          float accel_y = 0.3f * cosf(sim_time * 0.7f) + 0.05f * sinf(sim_time * 5.0f);
          float accel_z = 9.81f + 0.2f * sinf(sim_time * 1.2f);
          float gyro_x = 0.1f * sinf(sim_time * 0.8f) + 0.02f * sinf(sim_time * 10.0f);
          float gyro_y = 0.08f * cosf(sim_time * 0.6f) + 0.015f * cosf(sim_time * 8.0f);
          float gyro_z = 0.05f * sinf(sim_time * 0.3f) + 0.01f * sinf(sim_time * 12.0f);

          int filter_result = rust_gaussian_filter_imu(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
          syslog(LOG_INFO, "[Rust Drone] Gaussian filter applied, result: %d\n", filter_result);
        }

      if (counter % 10 == 0 && counter > 0)
        {
          /* Test different complementary filter operations */
          unsigned char filter_types[] = {0, 1, 2, 3, 4, 5}; // Roll, Pitch, Yaw, Confidence, etc.
          unsigned char filter_type = filter_types[counter % 6];
          uint64_t dt_microseconds = 10000; // 10ms in microseconds
          int comp_result = rust_complementary_filter(dt_microseconds, filter_type);
          syslog(LOG_INFO, "[Rust Drone] Complementary filter (type=%d) result: %d\n", filter_type, comp_result);
        }

      /* Test PID controller every 12 iterations */
      if (counter % 12 == 0 && counter > 0)
        {
          /* Test PID controllers for Roll, Pitch, Yaw */
          float setpoint = 0.0f; // Target angle (level flight)
          float measured_angle = 0.1f * sinf(counter * 0.05f); // Simulated current angle
          uint32_t dt_ms = 10; // 10ms sample time
          unsigned char pid_type = (counter / 12) % 3; // Cycle through Roll, Pitch, Yaw PIDs

          int pid_result = rust_pid_controller(setpoint, measured_angle, dt_ms, pid_type);
          syslog(LOG_INFO, "[Rust Drone] PID controller[%d] result: %d (milli-units)\n", pid_type, pid_result);
        }

      /* Process sensor data and send analytics every 8th iteration instead of 4th */
      if (counter % 8 == 0)
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

      /* System health check every 12th iteration instead of 6th */
      if (counter % 12 == 0)
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

      /* Test string processing every 16th iteration instead of 8th */
      if (counter % 16 == 0)
        {
          const char *test_string = "Hello from C to Rust!";
          uint32_t string_result = rust_process_string((const uint8_t*)test_string, strlen(test_string));
          uint32_t checksum = (string_result >> 16) & 0xFFFF;
          uint32_t vowels = string_result & 0xFFFF;
          syslog(LOG_INFO, "[Rust String] Checksum: %u, Vowels: %u\n", checksum, vowels);
        }

      /* Test array processing every 20th iteration instead of 10th */
      if (counter % 20 == 0 && counter > 0)
        {
          int32_t test_array[] = {1, 2, 3, 4, 5, (int32_t)counter};
          int32_t avg_result = rust_process_array(test_array, sizeof(test_array)/sizeof(test_array[0]));
          syslog(LOG_INFO, "[Rust Array] Average of test array: %d\n", avg_result);
        }

      counter++;
      sleep(1);
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
      sleep(1);
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

int ra8e1_thread_init(void)
{
   int ret = 0;
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

  return ret;
}
#endif
