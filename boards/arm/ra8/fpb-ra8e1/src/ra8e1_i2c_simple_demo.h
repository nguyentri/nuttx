/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_i2c_simple_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_SIMPLE_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_SIMPLE_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Demo return codes */
#define I2C_DEMO_OK     0
#define I2C_DEMO_ERROR -1

/* ADXL345 Configuration */
#define ADXL345_I2C_ADDRESS     0x1D    /* 7-bit I2C address */
#define ADXL345_DEVICE_ID_VAL   0xE5    /* Expected device ID */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Accelerometer data structure */
struct adxl345_accel_data_s
{
  int16_t x_raw;      /* Raw X-axis data */
  int16_t y_raw;      /* Raw Y-axis data */
  int16_t z_raw;      /* Raw Z-axis data */
  float x_mg;         /* X-axis acceleration in mg */
  float y_mg;         /* Y-axis acceleration in mg */
  float z_mg;         /* Z-axis acceleration in mg */
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
 * Name: ra8e1_i2c_simple_demo_main
 *
 * Description:
 *   Main function for RA8E1 I2C simple demo based on FSP SCI I2C Master 
 *   example. Demonstrates I2C communication with ADXL345 accelerometer.
 *
 * Input Parameters:
 *   argc - Number of command line arguments
 *   argv - Array of command line argument strings
 *          argv[1] - Optional: number of readings (1-100, default 5)
 *
 * Returned Value:
 *   I2C_DEMO_OK (0) on success
 *   I2C_DEMO_ERROR (-1) on failure
 *
 * Example Usage:
 *   ra8e1_i2c_simple_demo_main 10    # Take 10 readings
 *   ra8e1_i2c_simple_demo_main       # Take 5 readings (default)
 *
 ****************************************************************************/

int ra8e1_i2c_simple_demo_main(int argc, char *argv[]);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_SIMPLE_DEMO_H */
