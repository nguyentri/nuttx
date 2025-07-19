/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_i2c_gy912_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_GY912_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_GY912_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define I2C_GY912_DEMO_VERSION        "1.0.0"
#define I2C_GY912_BUFFER_SIZE         64
#define I2C_GY912_MAX_CHANNELS        2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* I2C GY912 sensor data structure */
struct i2c_gy912_data_s
{
  /* ICM-20948 data */
  int16_t accel_x;        /* Accelerometer X-axis */
  int16_t accel_y;        /* Accelerometer Y-axis */
  int16_t accel_z;        /* Accelerometer Z-axis */
  int16_t gyro_x;         /* Gyroscope X-axis */
  int16_t gyro_y;         /* Gyroscope Y-axis */
  int16_t gyro_z;         /* Gyroscope Z-axis */
  int16_t temperature_imu; /* Temperature from IMU */

  /* BMP388 data */
  uint32_t pressure;      /* Pressure data */
  int32_t temperature_bmp; /* Temperature from BMP388 */
};

/* I2C GY912 demo configuration structure */
struct i2c_gy912_demo_config_s
{
  uint8_t i2c_port;                     /* I2C port number */
  uint32_t frequency;                   /* I2C frequency */
  uint32_t timeout_ms;                  /* I2C timeout in milliseconds */
  bool icm20948_enabled;                /* Enable ICM-20948 */
  bool bmp388_enabled;                  /* Enable BMP388 */
};

/* I2C GY912 demo status structure */
struct i2c_gy912_demo_status_s
{
  bool initialized;                     /* Initialization status */
  bool icm20948_detected;               /* ICM-20948 detection status */
  bool bmp388_detected;                 /* BMP388 detection status */
  uint32_t read_count;                  /* Number of successful reads */
  uint32_t error_count;                 /* Number of errors */
  uint32_t last_read_time;              /* Last successful read timestamp */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ra8e1_i2c_gy912_demo_main
 *
 * Description:
 *   Main entry point for the I2C GY912 demo.
 *
 * Input Parameters:
 *   argc - Number of command line arguments
 *   argv - Array of command line arguments
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_gy912_demo_main(int argc, char *argv[]);

/****************************************************************************
 * Name: ra8e1_i2c_gy912_demo_init
 *
 * Description:
 *   Initialize the I2C GY912 demo.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_gy912_demo_init(void);

/****************************************************************************
 * Name: ra8e1_i2c_gy912_demo_test
 *
 * Description:
 *   Run the I2C GY912 demo test.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_gy912_demo_test(void);

/****************************************************************************
 * Name: ra8e1_i2c_gy912_demo_info
 *
 * Description:
 *   Display I2C GY912 demo information.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_gy912_demo_info(void);

/****************************************************************************
 * Name: ra8e1_i2c_gy912_demo_cleanup
 *
 * Description:
 *   Cleanup the I2C GY912 demo.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_gy912_demo_cleanup(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_GY912_DEMO_H */
