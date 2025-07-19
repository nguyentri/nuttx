/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_i2c_acc_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_ACCEL_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_ACCEL_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Accelerometer Configuration */
#define I2C_ACCEL_BUS               0
#define I2C_ACCEL_FREQUENCY         400000    /* 400kHz */
#define I2C_ACCEL_ADDR              0x1D      /* ADXL345 7-bit address */

/* ADXL345 Accelerometer Registers */
#define ADXL345_REG_DEVID           0x00      /* Device ID */
#define ADXL345_REG_BW_RATE         0x2C      /* Data rate and power mode control */
#define ADXL345_REG_POWER_CTL       0x2D      /* Power-saving features control */
#define ADXL345_REG_INT_ENABLE      0x2E      /* Interrupt enable control */
#define ADXL345_REG_INT_SOURCE      0x30      /* Source of interrupts */
#define ADXL345_REG_DATA_FORMAT     0x31      /* Data format control */
#define ADXL345_REG_DATAX0          0x32      /* X-Axis Data 0 */
#define ADXL345_REG_DATAX1          0x33      /* X-Axis Data 1 */
#define ADXL345_REG_DATAY0          0x34      /* Y-Axis Data 0 */
#define ADXL345_REG_DATAY1          0x35      /* Y-Axis Data 1 */
#define ADXL345_REG_DATAZ0          0x36      /* Z-Axis Data 0 */
#define ADXL345_REG_DATAZ1          0x37      /* Z-Axis Data 1 */

/* ADXL345 Configuration Values */
#define ADXL345_DEVICE_ID           0xE5      /* Expected device ID */
#define ADXL345_BW_RATE_12_5HZ      0x04      /* 12.5 Hz output data rate */
#define ADXL345_POWER_CTL_MEASURE   0x08      /* Enable measurement mode */
#define ADXL345_INT_DATA_READY      0x80      /* Data ready interrupt enable */
#define ADXL345_RANGE_2G            0x00      /* ±2g range */
#define ADXL345_RANGE_4G            0x01      /* ±4g range */
#define ADXL345_RANGE_8G            0x02      /* ±8g range */
#define ADXL345_RANGE_16G           0x03      /* ±16g range */

/* Sensitivity values for different ranges (mg/LSB) */
#define ADXL345_SENSITIVITY_2G      3.9f      /* mg/LSB for ±2g */
#define ADXL345_SENSITIVITY_4G      7.8f      /* mg/LSB for ±4g */
#define ADXL345_SENSITIVITY_8G      15.6f     /* mg/LSB for ±8g */
#define ADXL345_SENSITIVITY_16G     31.2f     /* mg/LSB for ±16g */

/* Default sensitivity (using ±2g range) */
#define ADXL345_DEFAULT_SENSITIVITY ADXL345_SENSITIVITY_2G

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Accelerometer data structure */
struct ra8e1_accel_data_s
{
  float x_g;        /* X-axis acceleration in g */
  float y_g;        /* Y-axis acceleration in g */
  float z_g;        /* Z-axis acceleration in g */
  int16_t x_raw;    /* X-axis raw data */
  int16_t y_raw;    /* Y-axis raw data */
  int16_t z_raw;    /* Z-axis raw data */
  bool valid;       /* Data validity flag */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_i2c_accel_initialize
 *
 * Description:
 *   Initialize I2C accelerometer demo
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_accel_initialize(void);

/****************************************************************************
 * Name: ra8e1_i2c_accel_read
 *
 * Description:
 *   Read accelerometer data
 *
 * Input Parameters:
 *   data - Pointer to structure to receive accelerometer data
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_accel_read(struct ra8e1_accel_data_s *data);

/****************************************************************************
 * Name: ra8e1_i2c_acc_demo_main
 *
 * Description:
 *   Main entry point for I2C accelerometer demonstration
 *
 * Input Parameters:
 *   argc - Number of arguments
 *   argv - Argument array
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_acc_demo_main(int argc, char *argv[]);

/****************************************************************************
 * Name: ra8e1_i2c_accel_test
 *
 * Description:
 *   Test I2C accelerometer functionality
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_accel_test(void);

/****************************************************************************
 * Name: ra8e1_i2c_accel_calibrate
 *
 * Description:
 *   Calibrate accelerometer offsets
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_i2c_accel_calibrate(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_I2C_ACCEL_DEMO_H */
