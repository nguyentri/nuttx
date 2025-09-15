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
#include <sched.h>
#include <sys/mman.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/kthread.h>
#include <nuttx/sched.h>

#include <arch/board/board.h>

#include "fpb-ra8e1.h"

/* Math constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Memory locking flags (fallback if not defined) */
#ifndef MCL_CURRENT
#define MCL_CURRENT 1
#endif
#ifndef MCL_FUTURE
#define MCL_FUTURE 2
#endif

/****************************************************************************
 * Flight Control Data Structures (Real-time C types)
 ****************************************************************************/
#ifdef CONFIG_RA8E1_RUST_DEMO

/* Hardware IMU data structure */
typedef struct {
    float accel_x, accel_y, accel_z;    /* Accelerometer (m/s²) */
    float gyro_x, gyro_y, gyro_z;       /* Gyroscope (rad/s) */
    float mag_x, mag_y, mag_z;          /* Magnetometer (μT) */
    uint64_t timestamp_us;              /* Timestamp in microseconds */
} imu_data_t;

/* Attitude estimation structure */
typedef struct {
    float roll, pitch, yaw;             /* Euler angles (rad) */
    float roll_rate, pitch_rate, yaw_rate; /* Angular rates (rad/s) */
    float confidence;                   /* Filter confidence (0.0-1.0) */
} attitude_t;

/* Control output structure */
typedef struct {
    float roll_output;                  /* Roll control output */
    float pitch_output;                 /* Pitch control output */
    float yaw_output;                   /* Yaw control output */
    float throttle;                     /* Throttle setting */
} control_output_t;

/* Motor commands structure */
typedef struct {
    uint32_t motor_fl, motor_fr;        /* Front motors (0-100%) */
    uint32_t motor_rl, motor_rr;        /* Rear motors (0-100%) */
} motor_commands_t;

/* PID controller structure */
typedef struct {
    float kp, ki, kd;                   /* PID gains */
    float integral, last_error;         /* State variables */
    float output_min, output_max;       /* Output limits */
} pid_controller_t;

/* Flight data for Rust logging */
typedef struct {
    attitude_t attitude;
    control_output_t control;
    imu_data_t imu_raw;
    motor_commands_t motors;
    uint64_t timestamp_us;
    uint32_t cycle_count;
} flight_data_t;

/* Legacy sensor data structure (for compatibility) */
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    uint64_t timestamp;
    float battery_voltage;
    uint32_t status_flags;
} shared_sensor_data_t;

/* Additional data structures for new architecture */
typedef struct {
    uint32_t packet_id;
    uint64_t timestamp_us;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_deg;
    float altitude_est;
    uint32_t battery_percent;
    uint32_t system_status;
} TelemetryPacket;

typedef struct {
    uint32_t overall_score;
    uint32_t imu_health;
    uint32_t control_health;
    uint32_t motor_health;
    uint32_t battery_health;
    uint32_t memory_usage;
    uint32_t cpu_load;
} SystemHealthStatus;

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

/* Rust low-priority task data structures */
typedef struct {
    uint32_t request_id;
    uint32_t data_type;     /* 1=flight_logs, 2=sensor_data, 3=health_data */
    uint32_t priority;      /* 1=low, 2=normal, 3=high */
    uint64_t timestamp_us;
} data_logging_request_t;

typedef struct {
    uint32_t analysis_type; /* 1=trend, 2=statistics, 3=anomaly_detection */
    uint32_t time_window;   /* Analysis window in seconds */
    uint32_t data_points;   /* Number of data points to analyze */
    float threshold;        /* Analysis threshold value */
} analytics_request_t;

typedef struct {
    uint32_t compression_level; /* 1=fast, 2=balanced, 3=max */
    uint32_t age_threshold;     /* Compress data older than X seconds */
    uint32_t size_limit;        /* Maximum size before compression */
    uint32_t preserve_recent;   /* Keep recent X entries uncompressed */
} compression_request_t;

typedef struct {
    uint32_t packet_id;
    uint64_t timestamp_us;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_deg;
    float altitude_est;
    uint32_t battery_percent;
    uint32_t system_status;
} telemetry_packet_t;

typedef union {
    data_logging_request_t logging_req;
    analytics_request_t analytics_req;
    compression_request_t compression_req;
    health_data_t health_analysis;
    flight_data_t flight_data;
    telemetry_packet_t telemetry;  /* Reserved for future use */
} message_data_t;

typedef struct {
    uint32_t msg_type;
    uint64_t timestamp;
    message_data_t data;
} queue_message_t;

/* Message types for Rust low-priority tasks */
#define MSG_TYPE_DATA_LOGGING   1   /* Flight data logging to Rust */
#define MSG_TYPE_DATA_ANALYTICS 2   /* Statistical analysis requests */
#define MSG_TYPE_LOG_COMPRESSION 3  /* Compress old flight logs */
#define MSG_TYPE_HEALTH_ANALYSIS 4  /* System health trend analysis */
#define MSG_TYPE_FLIGHT_STATS   5   /* Calculate flight statistics */
#define MSG_TYPE_TELEMETRY      6   /* Telemetry packets from C for external transmission */

/****************************************************************************
 * External Function Declarations (Rust data services functions)
 ****************************************************************************/

/* New Rust data services functions */
extern int rust_system_init(void);
extern int rust_set_queues(int rust_to_c, int c_to_rust);
extern int rust_log_flight_data(const flight_data_t* flight_data);
extern const TelemetryPacket* rust_generate_telemetry(void);
extern const SystemHealthStatus* rust_analyze_system_health(void);
extern int rust_compress_flight_logs(void);
extern int rust_calculate_flight_stats(void);
extern int rust_receive_flight_data(void);
extern int rust_send_telemetry(void);  /* DEPRECATED - telemetry now flows C->Rust */
extern int rust_transmit_telemetry(const telemetry_packet_t* telemetry);

/* Legacy Rust functions (simplified for compatibility) */
extern int rust_store_sensor_data(float temp, float humidity, float pressure,
                                  uint64_t timestamp, float battery);
extern const rust_processed_data_t* rust_process_sensor_data(void);
extern const shared_sensor_data_t* rust_get_sensor_data(void);
extern int rust_validate_sensor_data(float temp, float humidity, float pressure);
extern uint32_t rust_system_health_check(void);
extern uint32_t rust_process_string(const uint8_t* input, size_t len);
extern int32_t rust_process_array(const int32_t* data, size_t len);

/* Legacy thread-safe Rust functions using message queues */
extern int rust_send_sensor_data(float temp, float humidity, float pressure,
                                 uint64_t timestamp, float battery);
extern int rust_send_processed_data(void);
extern int rust_send_health_check(void);
extern int rust_receive_message(void);

/* Global message queue descriptors */
static mqd_t g_rust_to_c_queue = -1;
static mqd_t g_c_to_rust_queue = -1;

/* Global flight control state */
static attitude_t g_attitude = {0};
static pid_controller_t g_roll_pid = {2.0f, 0.1f, 0.05f, 0, 0, -45.0f, 45.0f};
static pid_controller_t g_pitch_pid = {2.0f, 0.1f, 0.05f, 0, 0, -45.0f, 45.0f};
static pid_controller_t g_yaw_pid = {1.5f, 0.05f, 0.02f, 0, 0, -30.0f, 30.0f};

/* Complementary filter state */
static float g_comp_angle_x = 0.0f;
static float g_comp_angle_y = 0.0f;
static uint64_t g_last_filter_time = 0;

/* Flight control setpoints */
static float g_roll_setpoint = 0.0f;
static float g_pitch_setpoint = 0.0f;
static float g_yaw_rate_setpoint = 0.0f;
static float g_throttle_setpoint = 50.0f;

/****************************************************************************
 * Hardware Access Functions (C Implementation)
 ****************************************************************************/

/****************************************************************************
 * Name: read_imu_hardware
 *
 * Description:
 *   Direct hardware access for IMU data (simulated for demo)
 *
 ****************************************************************************/

static void read_imu_hardware(imu_data_t *imu)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    static uint32_t sim_cycle = 0;
    float sim_time = sim_cycle * 0.001f; /* 1ms cycle time */

    /* Simulate realistic drone IMU data */
    imu->accel_x = 0.5f * sinf(sim_time * 0.5f) + 0.1f * sinf(sim_time * 3.0f);
    imu->accel_y = 0.3f * cosf(sim_time * 0.7f) + 0.05f * sinf(sim_time * 5.0f);
    imu->accel_z = 9.81f + 0.2f * sinf(sim_time * 1.2f);

    imu->gyro_x = 0.1f * sinf(sim_time * 0.8f) + 0.02f * sinf(sim_time * 10.0f);
    imu->gyro_y = 0.08f * cosf(sim_time * 0.6f) + 0.015f * cosf(sim_time * 8.0f);
    imu->gyro_z = 0.05f * sinf(sim_time * 0.3f) + 0.01f * sinf(sim_time * 12.0f);

    imu->mag_x = 25.0f + 2.0f * sinf(sim_time * 0.1f);
    imu->mag_y = 15.0f + 1.5f * cosf(sim_time * 0.15f);
    imu->mag_z = 45.0f + 3.0f * sinf(sim_time * 0.08f);

    imu->timestamp_us = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;
    sim_cycle++;
}

/****************************************************************************
 * Name: write_motor_hardware
 *
 * Description:
 *   Direct hardware access for motor control (simulated for demo)
 *
 ****************************************************************************/

static void write_motor_hardware(const motor_commands_t *motors)
{
    /* In real implementation, this would write to PWM registers */
    static uint32_t motor_log_counter = 0;

    /* Log motor commands every 100 cycles (10Hz at 1000Hz control rate) */
    if (motor_log_counter++ % 100 == 0)
    {
        syslog(LOG_INFO, "[C Flight Control] Motors: FL=%lu%%, FR=%lu%%, RL=%lu%%, RR=%lu%%\n",
               (unsigned long)motors->motor_fl, (unsigned long)motors->motor_fr,
               (unsigned long)motors->motor_rl, (unsigned long)motors->motor_rr);
    }
}

/****************************************************************************
 * Name: pid_update
 *
 * Description:
 *   Fast PID controller update (C implementation for real-time performance)
 *
 ****************************************************************************/

static float pid_update(pid_controller_t *pid, float error, float dt)
{
    /* Proportional term */
    float p_term = pid->kp * error;

    /* Integral term with windup protection */
    pid->integral += error * dt;
    float integral_limit = pid->output_max / pid->ki;
    if (pid->integral > integral_limit)
    {
        pid->integral = integral_limit;
    }
    else if (pid->integral < -integral_limit)
    {
        pid->integral = -integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    /* Derivative term */
    float d_term = 0.0f;
    if (dt > 0.0f)
    {
        d_term = pid->kd * (error - pid->last_error) / dt;
    }
    pid->last_error = error;

    /* Calculate total output */
    float output = p_term + i_term + d_term;

    /* Apply output limits */
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    return output;
}

/****************************************************************************
 * Name: sensor_fusion_update
 *
 * Description:
 *   Fast complementary filter for attitude estimation (C implementation)
 *
 ****************************************************************************/

static void sensor_fusion_update(const imu_data_t *imu, attitude_t *attitude)
{
    float dt = 0.001f; /* 1ms sample time for 1000Hz */

    if (g_last_filter_time > 0)
    {
        dt = (imu->timestamp_us - g_last_filter_time) / 1000000.0f;
    }
    g_last_filter_time = imu->timestamp_us;

    /* Calculate accelerometer angles */
    float accel_angle_x = atan2f(imu->accel_y, imu->accel_z);
    float accel_angle_y = atan2f(-imu->accel_x,
                                sqrtf(imu->accel_y * imu->accel_y +
                                      imu->accel_z * imu->accel_z));

    /* Complementary filter (98% gyro, 2% accel) */
    g_comp_angle_x = 0.98f * (g_comp_angle_x + imu->gyro_x * dt) + 0.02f * accel_angle_x;
    g_comp_angle_y = 0.98f * (g_comp_angle_y + imu->gyro_y * dt) + 0.02f * accel_angle_y;

    /* Update attitude structure */
    attitude->roll = g_comp_angle_x;
    attitude->pitch = g_comp_angle_y;
    attitude->yaw_rate = imu->gyro_z;
    attitude->roll_rate = imu->gyro_x;
    attitude->pitch_rate = imu->gyro_y;

    /* Simple confidence metric based on accelerometer magnitude */
    float accel_mag = sqrtf(imu->accel_x * imu->accel_x +
                           imu->accel_y * imu->accel_y +
                           imu->accel_z * imu->accel_z);
    attitude->confidence = 1.0f - fabsf(accel_mag - 9.81f) / 9.81f;
    if (attitude->confidence < 0.0f) attitude->confidence = 0.0f;
    if (attitude->confidence > 1.0f) attitude->confidence = 1.0f;
}

/****************************************************************************
 * Name: flight_controller_update
 *
 * Description:
 *   Main flight control loop (C implementation for real-time performance)
 *
 ****************************************************************************/

static void flight_controller_update(const attitude_t *attitude, control_output_t *control)
{
    const float dt = 0.001f; /* 1ms control loop */

    /* Calculate control errors */
    float roll_error = g_roll_setpoint - attitude->roll;
    float pitch_error = g_pitch_setpoint - attitude->pitch;
    float yaw_rate_error = g_yaw_rate_setpoint - attitude->yaw_rate;

    /* Update PID controllers */
    control->roll_output = pid_update(&g_roll_pid, roll_error, dt);
    control->pitch_output = pid_update(&g_pitch_pid, pitch_error, dt);
    control->yaw_output = pid_update(&g_yaw_pid, yaw_rate_error, dt);
    control->throttle = g_throttle_setpoint;
}

/****************************************************************************
 * Name: mixer_update
 *
 * Description:
 *   Quadcopter motor mixing (C implementation)
 *
 ****************************************************************************/

static uint32_t constrain_motor(float value)
{
    if (value < 0.0f) return 0;
    if (value > 100.0f) return 100;
    return (uint32_t)value;
}

static void mixer_update(const control_output_t *control, motor_commands_t *motors)
{
    /* Standard X-configuration quadcopter motor mixing */
    float motor_fl = control->throttle + control->pitch_output + control->roll_output - control->yaw_output;
    float motor_fr = control->throttle + control->pitch_output - control->roll_output + control->yaw_output;
    float motor_rl = control->throttle - control->pitch_output + control->roll_output + control->yaw_output;
    float motor_rr = control->throttle - control->pitch_output - control->roll_output - control->yaw_output;

    /* Constrain motor outputs to 0-100% range */
    motors->motor_fl = constrain_motor(motor_fl);
    motors->motor_fr = constrain_motor(motor_fr);
    motors->motor_rl = constrain_motor(motor_rl);
    motors->motor_rr = constrain_motor(motor_rr);
}

static void send_flight_data_to_rust(const attitude_t *attitude,
                                    const control_output_t *control,
                                    const imu_data_t *imu,
                                    const motor_commands_t *motors,
                                    uint32_t cycle_count)
{
  if (g_c_to_rust_queue >= 0)  /* C sends flight data TO Rust */
  {
        queue_message_t msg;
        msg.msg_type = MSG_TYPE_DATA_LOGGING;  /* Send flight data for low-priority logging */
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.timestamp = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

        /* Pack flight data using union member access */
        msg.data.flight_data.attitude = *attitude;
        msg.data.flight_data.control = *control;
        msg.data.flight_data.imu_raw = *imu;
        msg.data.flight_data.motors = *motors;
        msg.data.flight_data.timestamp_us = msg.timestamp;
        msg.data.flight_data.cycle_count = cycle_count;

    /* Non-blocking send to avoid real-time disruption */
    if (mq_send(g_c_to_rust_queue, (const char*)&msg, sizeof(msg), 0) < 0)
    {
      /* Queue full - drop message to maintain real-time performance */
    }
  }
  else
  {
    /* Fallback: call Rust directly when queues are unavailable */
    flight_data_t fd;
    fd.attitude = *attitude;
    fd.control = *control;
    fd.imu_raw = *imu;
    fd.motors = *motors;
    fd.timestamp_us = 0; /* Not critical in fallback */
    fd.cycle_count = cycle_count;
    rust_log_flight_data(&fd);
  }
}

/****************************************************************************
 * Name: send_analytics_request_to_rust
 *
 * Description:
 *   Send low-priority analytics request to Rust
 *
 ****************************************************************************/

static void send_analytics_request_to_rust(uint32_t analysis_type, uint32_t time_window)
{
    if (g_c_to_rust_queue >= 0)
    {
        queue_message_t msg;
        msg.msg_type = MSG_TYPE_DATA_ANALYTICS;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.timestamp = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

        /* Pack analytics request */
        msg.data.analytics_req.analysis_type = analysis_type;
        msg.data.analytics_req.time_window = time_window;
        msg.data.analytics_req.data_points = 100; /* Default */
        msg.data.analytics_req.threshold = 0.8f;  /* Default threshold */

        /* Non-blocking send */
        if (mq_send(g_c_to_rust_queue, (const char*)&msg, sizeof(msg), 0) < 0)
        {
            /* Queue full - drop message, low priority task */
        }
    }
}

/****************************************************************************
 * Name: send_compression_request_to_rust
 *
 * Description:
 *   Send low-priority compression request to Rust
 *
 ****************************************************************************/

static void send_compression_request_to_rust(uint32_t age_threshold)
{
    if (g_c_to_rust_queue >= 0)
    {
        queue_message_t msg;
        msg.msg_type = MSG_TYPE_LOG_COMPRESSION;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.timestamp = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

        /* Pack compression request */
        msg.data.compression_req.compression_level = 2; /* Balanced */
        msg.data.compression_req.age_threshold = age_threshold;
        msg.data.compression_req.size_limit = 1000;     /* Max entries */
        msg.data.compression_req.preserve_recent = 50;  /* Keep 50 recent */

        /* Non-blocking send */
        if (mq_send(g_c_to_rust_queue, (const char*)&msg, sizeof(msg), 0) < 0)
        {
            /* Queue full - drop message, low priority task */
        }
    }
}

/****************************************************************************
 * Name: send_telemetry_to_rust
 *
 * Description:
 *   Generate and send telemetry packet to Rust for external transmission
 *
 ****************************************************************************/

static void send_telemetry_to_rust(const attitude_t *attitude,
                                  const control_output_t *control,
                                  const imu_data_t *imu,
                                  const motor_commands_t *motors,
                                  uint32_t cycle_count)
{
  if (g_c_to_rust_queue >= 0)
  {
        queue_message_t msg;
        msg.msg_type = MSG_TYPE_TELEMETRY;  /* Telemetry for external transmission */

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        msg.timestamp = ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL;

        /* Generate telemetry packet from current flight state */
        static uint32_t telemetry_packet_id = 0;
        telemetry_packet_id++;

        msg.data.telemetry.packet_id = telemetry_packet_id;
        msg.data.telemetry.timestamp_us = msg.timestamp;
        msg.data.telemetry.roll_deg = attitude->roll * 180.0f / M_PI;
        msg.data.telemetry.pitch_deg = attitude->pitch * 180.0f / M_PI;
        msg.data.telemetry.yaw_rate_deg = attitude->yaw_rate * 180.0f / M_PI;
        msg.data.telemetry.altitude_est = 10.0f + sinf(cycle_count * 0.001f) * 2.0f; /* Simulated altitude */

        /* Calculate battery percentage (simulated) */
        msg.data.telemetry.battery_percent = 100 - (cycle_count / 10000); /* Slowly decreasing */
        if (msg.data.telemetry.battery_percent > 100) msg.data.telemetry.battery_percent = 100;
        if (msg.data.telemetry.battery_percent < 10) msg.data.telemetry.battery_percent = 10;

        /* System status based on attitude confidence */
        msg.data.telemetry.system_status = (attitude->confidence > 0.8f) ? 1 : 2; /* 1=OK, 2=WARNING */

    /* Non-blocking send to Rust for external transmission */
    if (mq_send(g_c_to_rust_queue, (const char*)&msg, sizeof(msg), 0) < 0)
    {
      /* Queue full - drop message to maintain real-time performance */
    }
  }
  else
  {
    /* Fallback: call Rust directly to transmit telemetry when queues are unavailable */
    telemetry_packet_t tp;
    static uint32_t telemetry_packet_id = 0;
    telemetry_packet_id++;

    tp.packet_id = telemetry_packet_id;
    tp.timestamp_us = 0;
    tp.roll_deg = attitude->roll * 180.0f / M_PI;
    tp.pitch_deg = attitude->pitch * 180.0f / M_PI;
    tp.yaw_rate_deg = attitude->yaw_rate * 180.0f / M_PI;
    tp.altitude_est = 10.0f + sinf(cycle_count * 0.001f) * 2.0f;
    tp.battery_percent = 100 - (cycle_count / 10000);
    if (tp.battery_percent > 100) tp.battery_percent = 100;
    if (tp.battery_percent < 10) tp.battery_percent = 10;
    tp.system_status = (attitude->confidence > 0.8f) ? 1 : 2;
    rust_transmit_telemetry(&tp);
  }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Forward declaration */
static void cleanup_thread_safe_queues(void);
static void send_telemetry_to_rust(const attitude_t *attitude,
                                  const control_output_t *control,
                                  const imu_data_t *imu,
                                  const motor_commands_t *motors,
                                  uint32_t cycle_count);

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

#ifdef CONFIG_MQ_MAXMSGSIZE
  if (attr.mq_msgsize > CONFIG_MQ_MAXMSGSIZE)
    {
      syslog(LOG_WARNING,
             "[C Queue Init] WARNING: msg size %d > CONFIG_MQ_MAXMSGSIZE %d; clamping\n",
             (int)attr.mq_msgsize, CONFIG_MQ_MAXMSGSIZE);
      attr.mq_msgsize = CONFIG_MQ_MAXMSGSIZE;
    }
#endif

  /* Create Rust-to-C message queue (UNUSED in new architecture - telemetry flows C->Rust) */
  g_rust_to_c_queue = mq_open("/rust2c", O_CREAT | O_RDWR, 0666, &attr);
  if (g_rust_to_c_queue < 0)
    {
      syslog(LOG_ERR, "[C Queue Init] ERROR: Failed to create Rust-to-C message queue: %d (errno: %d)\n",
             g_rust_to_c_queue, errno);
      syslog(LOG_ERR, "[C Queue Init] ERROR: Message queue support may not be enabled in NuttX configuration\n");
      syslog(LOG_INFO, "[C Queue Init] INFO: Falling back to legacy Rust-C communication without queues\n");
      g_rust_to_c_queue = -1;
      g_c_to_rust_queue = -1;
      return 0;  /* Continue without message queues */
    }

  /* Create C-to-Rust message queue (C writes telemetry/tasks, Rust reads) */
  g_c_to_rust_queue = mq_open("/c2rust", O_CREAT | O_RDWR, 0666, &attr);
  if (g_c_to_rust_queue < 0)
    {
      syslog(LOG_ERR, "[C Queue Init] ERROR: Failed to create C-to-Rust message queue: %d (errno: %d)\n",
             g_c_to_rust_queue, errno);
      mq_close(g_rust_to_c_queue);
      mq_unlink("/rust2c");
      g_rust_to_c_queue = -1;
      g_c_to_rust_queue = -1;
      syslog(LOG_INFO, "[C Queue Init] INFO: Falling back to legacy Rust-C communication without queues\n");
      return 0;  /* Continue without message queues */
    }

  /* Pass queue descriptors to Rust - NOTE: New architecture direction */
  /* C sends telemetry/tasks TO Rust, so C writes to c_to_rust_queue */
  /* rust_to_c_queue is unused in new architecture (kept for compatibility) */
  rust_set_queues(g_rust_to_c_queue, g_c_to_rust_queue);

  syslog(LOG_INFO, "[C Queue Init] Thread-safe message queues initialized successfully\n");
  syslog(LOG_INFO, "[C Queue Init] New architecture: C->Rust telemetry transmission\n");
  syslog(LOG_INFO, "[C Queue Init] Rust-to-C queue: %d (unused), C-to-Rust queue: %d\n",
         g_rust_to_c_queue, g_c_to_rust_queue);
  syslog(LOG_INFO, "[C Queue Init] Queue message size: %d bytes\n", (int)sizeof(queue_message_t));

  /* Register cleanup function for proper shutdown */
  atexit(cleanup_thread_safe_queues);
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
 *   Cleanup message queues (called on system shutdown)
 *
 ****************************************************************************/

static void cleanup_thread_safe_queues(void)
{
  syslog(LOG_INFO, "[C Cleanup] Cleaning up message queues\n");

  if (g_rust_to_c_queue >= 0)
    {
      mq_close(g_rust_to_c_queue);
      mq_unlink("/rust2c");
      g_rust_to_c_queue = -1;
    }

  if (g_c_to_rust_queue >= 0)
    {
      mq_close(g_c_to_rust_queue);
      mq_unlink("/c2rust");
      g_c_to_rust_queue = -1;
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
 * Name: ra8e1_flight_control_thread
 *
 * Description:
 *   High-priority real-time flight control thread running at 1000Hz
 *
 ****************************************************************************/

static int ra8e1_flight_control_thread(int argc, char *argv[])
{
    struct timespec next_cycle;
    const long cycle_time_ns = 1000000; /* 1ms = 1000Hz */
    uint32_t cycle_count = 0;

    syslog(LOG_INFO, "[C Flight Control] High-priority flight control thread started\n");

    /* Set high priority for real-time performance */
    struct sched_param param;
    param.sched_priority = 200; /* High priority */
    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0)
    {
        syslog(LOG_WARNING, "[C Flight Control] Failed to set FIFO scheduler, using default\n");
    }

    /* Lock memory to prevent page faults */
#ifdef CONFIG_MM_KERNEL_HEAP
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        syslog(LOG_WARNING, "[C Flight Control] Failed to lock memory, continuing\n");
    }
#endif

    clock_gettime(CLOCK_MONOTONIC, &next_cycle);

    while (1)
    {
        /* === CRITICAL FLIGHT CONTROL LOOP === */

        /* 1. Read sensors (hardware registers directly) */
        imu_data_t imu_raw;
        read_imu_hardware(&imu_raw);

        /* 2. Sensor fusion (complementary filter in C) */
        sensor_fusion_update(&imu_raw, &g_attitude);

        /* 3. PID control calculations */
        control_output_t control;
        flight_controller_update(&g_attitude, &control);

        /* 4. Motor mixing and output */
        motor_commands_t motors;
        mixer_update(&control, &motors);
        write_motor_hardware(&motors);

        /* 5. Send data to Rust (non-blocking) - only every 10th cycle (100Hz) */
        if (cycle_count % 2 == 0)
        {
            send_flight_data_to_rust(&g_attitude, &control, &imu_raw, &motors, cycle_count);

            /* Debug log every 10 sends (once per second) */
            if (cycle_count % 10 == 0)
            {
                syslog(LOG_INFO, "[C Flight Control] Sent %lu flight data packets to Rust\n",
                       (unsigned long)(cycle_count / 10));
            }
        }

        /* 6. Send telemetry to Rust for external transmission - every 5 cycle (10Hz) */
        if (cycle_count % 5 == 0)
        {
            syslog(LOG_INFO, "[C Flight Control] Sending telemetry packet #%lu to Rust\n",
                   (unsigned long)(cycle_count / 5));
            send_telemetry_to_rust(&g_attitude, &control, &imu_raw, &motors, cycle_count);
        }

        /* === END CRITICAL SECTION === */

        /* Log performance every 10 cycles (1Hz) */
        if (cycle_count % 10 == 0)
        {
            float roll_deg = g_attitude.roll * 180.0f / M_PI;
            float pitch_deg = g_attitude.pitch * 180.0f / M_PI;
            syslog(LOG_INFO, "[C Flight Control] Cycle: %lu, Roll: %.1fdegC, Pitch: %.1fdegC, Confidence: %.2f\n",
                   (unsigned long)cycle_count, roll_deg, pitch_deg, g_attitude.confidence);
        }

        cycle_count++;

        /* Wait for next cycle (precise timing) */
        next_cycle.tv_nsec += cycle_time_ns;
        if (next_cycle.tv_nsec >= 1000000000)
        {
            next_cycle.tv_sec++;
            next_cycle.tv_nsec -= 1000000000;
        }
        //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_cycle, NULL);

        sleep(1); /* For demo purposes, reduce CPU load - remove in real implementation */
    }

    return 0;
}

/****************************************************************************
 * Name: ra8e1_cyclic_rust_thread (LEGACY - keeping for compatibility)
 *
 * Description:
 *   Legacy rust thread - now reduced functionality for demo purposes
 *
 ****************************************************************************/

static int ra8e1_cyclic_rust_thread(int argc, char *argv[])
{
  struct timespec start_time;
  uint32_t counter = 0;

  syslog(LOG_INFO, "[Rust Data Services] Low-priority data services thread started\n");

  /* Initialize Rust data services subsystem */
  syslog(LOG_INFO, "[Rust Data Services] Initializing data logging and telemetry...\n");
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

      /* Print log message with timing information every 2 seconds */
      if (counter % 5 == 0)
        {
          syslog(LOG_INFO, "[Rust Data Services] Count: %lu, Elapsed: %.3fs\n",
                 (unsigned long)counter, elapsed_total);
        }

      /* Check for flight data messages from C flight control thread */
      rust_receive_flight_data();

      /* Note: Telemetry is now generated by C and sent to Rust for external transmission */
      /* Rust no longer generates telemetry - it receives and transmits it externally */

      /* System health analysis every 5 seconds (0.2Hz) */
      if (counter % 2 == 0 && counter > 0)
        {
          (void)rust_analyze_system_health();
          // Logged health status in Rust
          //syslog(LOG_INFO, "[Rust System Health] Overall=%lu%%, IMU=%lu%%, Battery=%lu%%\n",
          //       (unsigned long)health->overall_score, (unsigned long)health->imu_health, (unsigned long)health->battery_health);
        }

      /* Flight data analytics every 10 seconds */
      if (counter % 5 == 0 && counter > 0)
        {
          int stats_result = rust_calculate_flight_stats();
          syslog(LOG_INFO, "[Rust Data Analytics Service Result] Flight statistics calculated: %d samples\n", stats_result);
        }

      /* Data compression every 30 seconds */
      if (counter % 10 == 0 && counter > 0)
        {
          int compressed = rust_compress_flight_logs();
          syslog(LOG_INFO, "[Rust Compression Service Result] Compressed %d flight log entries\n", compressed);
        }

      /* Legacy compatibility - simulate and process sensor data every 20 iterations */
      if (counter % 20 == 0)
        {
          float temp, humidity, pressure, battery;
          simulate_sensor_reading(&temp, &humidity, &pressure, &battery);

          /* Validate sensor data using Rust */
          if (rust_validate_sensor_data(temp, humidity, pressure))
            {
              if (g_rust_to_c_queue >= 0)
                {
                  rust_send_sensor_data(temp, humidity, pressure,
                                       current_time.tv_sec * 1000000ULL + current_time.tv_nsec / 1000ULL,
                                       battery);
                }
              else
                {
                  /* Use legacy method when queues are not available */
                  rust_store_sensor_data(temp, humidity, pressure,
                                        current_time.tv_sec * 1000000ULL + current_time.tv_nsec / 1000ULL,
                                        battery);
                }
            }
        }

      /* Legacy string processing test every 60 iterations */
      //if (counter % 60 == 0)
      //  {
      //    const char *test_string = "NuttX Drone Telemetry Data";
      //    uint32_t string_result = rust_process_string((const uint8_t*)test_string, strlen(test_string));
      //    uint32_t checksum = (string_result >> 16) & 0xFFFF;
      //    uint32_t vowels = string_result & 0xFFFF;
      //    syslog(LOG_INFO, "[Rust String Testing Service] String processing: Checksum=%lu, Vowels=%lu\n", (unsigned long)checksum, (unsigned long)vowels);
      //  }

      /* Legacy array processing test every 80 iterations */
      //if (counter % 80 == 0 && counter > 0)
      //  {
      //    int32_t test_array[] = {10, 20, 30, 40, 50, (int32_t)counter};
      //    int32_t avg_result = rust_process_array(test_array, sizeof(test_array)/sizeof(test_array[0]));
      //    syslog(LOG_INFO, "[Rust Array Processing Service] Array analytics: Average=%ld\n", (long)avg_result);
      //  }

      counter++;
      sleep(2); // 2s
    }

  return 0;
}

/****************************************************************************
 * Name: ra8e1_cyclic_c_thread
 *
 * Description:
 *   Enhanced C thread that handles realtime sensor data directly in C
 *   and receives only telemetry data from Rust via thread-safe message queues
 *
 ****************************************************************************/

static int ra8e1_cyclic_c_thread(int argc, char *argv[])
{
  struct timespec start_time;
  uint32_t counter = 0;

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
      syslog(LOG_INFO, "[C Health Monitoring] Count: %lu, Elapsed: %.3fs\n",
             (unsigned long)counter, elapsed_total);

      /* Generate realtime sensor data directly in C */
      if (counter % 2 == 0)
        {
          float temp, humidity, pressure, battery;
          simulate_sensor_reading(&temp, &humidity, &pressure, &battery);

          syslog(LOG_INFO, "[C Sensor Simulation] Temp: %.1fdegC, "
                          "Humidity: %.1f%%, Pressure: %.1fhPa, Battery: %.2fV\n",
                 temp, humidity, pressure, battery);

          /* C-side realtime processing and alerts */
          if (temp > 30.0f)
            {
              syslog(LOG_WARNING, "[C Alert] High temperature detected: %.1fdegC\n", temp);
            }

          if (battery < 3.5f)
            {
              syslog(LOG_WARNING, "[C Alert] Low battery detected: %.2fV\n", battery);
            }

          if (pressure < 980.0f)
            {
              syslog(LOG_WARNING, "[C Alert] Low pressure detected: %.1fhPa\n", pressure);
            }
        }

      /* Generate C-based health monitoring data */
      if (counter % 3 == 0)
        {
          uint32_t health_score = 85 + (rand() % 30); /* Random health score 85-115 */
          if (health_score > 100) health_score = 100;

          uint32_t battery_status = (rand() % 3) + 1; /* 1=Good, 2=Warning, 3=Critical */
          uint32_t temp_status = (rand() % 3) + 1;

          syslog(LOG_INFO, "[C Direct Health] Score: %lu%%, Battery Status: %lu, Temp Status: %lu\n",
                 (unsigned long)health_score, (unsigned long)battery_status, (unsigned long)temp_status);

          if (health_score < 70)
            {
              syslog(LOG_WARNING, "[C Health Warning] System health degraded: %lu%%\n", (unsigned long)health_score);
            }
        }

      /* Send analytics requests to Rust every 30 seconds */
      if (counter % 5 == 0 && counter > 0)
        {
          send_analytics_request_to_rust(2, 300); /* Statistics analysis over 5-minute window */
          syslog(LOG_INFO, "[C Analytics Request] Sent analytics request to Rust\n");
        }

      /* Send compression requests to Rust every 60 seconds */
      if (counter % 6 == 0 && counter > 0)
        {
          send_compression_request_to_rust(1800); /* Compress data older than 30 minutes */
          syslog(LOG_INFO, "[C Compression Request] Sent compression request to Rust\n");
        }

      /* Note: Telemetry now flows C->Rust for external transmission, not Rust->C for logging */
      /* The C thread generates telemetry directly in flight control and sends it to Rust */
      if (counter % 10 == 0)
        {
          syslog(LOG_INFO, "[C Health Monitoring] System operating - telemetry flows C->Rust for external transmission\n");
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

  /* Initialize thread-safe message queues first */
  init_thread_safe_queues();

  /* Start high-priority flight control thread */
  syslog(LOG_INFO, "Starting High-Priority Flight Control Thread...\n");

  ret = kthread_create("flight_control",
                       200,           /* High priority */
                       4096,          /* Smaller stack for real-time */
                       ra8e1_flight_control_thread,
                       NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start flight control thread: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Flight Control Thread started successfully (PID: %d)\n", ret);
    }

  /* Start lower-priority Rust data services thread */
  syslog(LOG_INFO, "Starting Rust Data Services Thread...\n");

  ret = kthread_create("rust_data_services",
                       50,            /* Lower priority */
                       32768,         /* Larger stack for Rust */
                       ra8e1_cyclic_rust_thread,
                       NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start rust data services thread: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "Rust Data Services Thread started successfully (PID: %d)\n", ret);
    }

  /* Start C monitoring thread for legacy compatibility */
  syslog(LOG_INFO, "Starting C Monitoring Thread...\n");

  ret = kthread_create("c_monitor",
                       75,            /* Medium priority */
                       8192,
                       ra8e1_cyclic_c_thread,
                       NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to start C monitoring thread: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "C Monitoring Thread started successfully (PID: %d)\n", ret);
    }

  syslog(LOG_INFO, "Nuttx: RA8E1 Board bring-up with new drone architecture is successful...\n");

  return ret;
}
#endif
