/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/spi_gy912.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_SPI_GY912_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_SPI_GY912_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GY-912 10DOF sensor module definitions */

/* SPI Bus number for GY-912 */
#define GY912_SPI_BUS           0

/* SPI device IDs for GY-912 sensors */
#define SPIDEV_ICM20948         0  /* ICM-20948 9DOF IMU */
#define SPIDEV_BMP388           1  /* BMP388 pressure sensor */

/* Chip select pins for GY-912 sensors */
#define GY912_ICM20948_CS_PIN   PIN_P612  /* ICM-20948 CS */
#define GY912_BMP388_CS_PIN     PIN_P605  /* BMP388 CS */

/* ICM-20948 Register Definitions */
#define ICM20948_WHO_AM_I       0x75
#define ICM20948_WHO_AM_I_VAL   0xEA
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07
#define ICM20948_ACCEL_XOUT_H   0x2D
#define ICM20948_ACCEL_XOUT_L   0x2E
#define ICM20948_ACCEL_YOUT_H   0x2F
#define ICM20948_ACCEL_YOUT_L   0x30
#define ICM20948_ACCEL_ZOUT_H   0x31
#define ICM20948_ACCEL_ZOUT_L   0x32
#define ICM20948_GYRO_XOUT_H    0x33
#define ICM20948_GYRO_XOUT_L    0x34
#define ICM20948_GYRO_YOUT_H    0x35
#define ICM20948_GYRO_YOUT_L    0x36
#define ICM20948_GYRO_ZOUT_H    0x37
#define ICM20948_GYRO_ZOUT_L    0x38

/* BMP388 Register Definitions */
#define BMP388_CHIP_ID          0x00
#define BMP388_CHIP_ID_VAL      0x50
#define BMP388_PWR_CTRL         0x1B
#define BMP388_OSR              0x1C
#define BMP388_ODR              0x1D
#define BMP388_CONFIG           0x1F
#define BMP388_PRESS_MSB        0x04
#define BMP388_PRESS_LSB        0x05
#define BMP388_PRESS_XLSB       0x06
#define BMP388_TEMP_MSB         0x07
#define BMP388_TEMP_LSB         0x08
#define BMP388_TEMP_XLSB        0x09

/* SPI transfer parameters */
#define GY912_SPI_FREQUENCY     1000000   /* 1MHz SPI clock */
#define GY912_SPI_MODE          SPIDEV_MODE0
#define GY912_SPI_BITS          8

/* Pin definitions for FPB-RA8E1 */
#define PIN_SPI0_SCK    (GPIO_OUTPUT | GPIO_PULLUP | \
                         GPIO_PORT6 | GPIO_PIN11 | GPIO_PSEL_SPI)
#define PIN_SPI0_MISO   (GPIO_INPUT  | GPIO_PULLUP | \
                         GPIO_PORT6 | GPIO_PIN10 | GPIO_PSEL_SPI)
#define PIN_SPI0_MOSI   (GPIO_OUTPUT | GPIO_PULLUP | \
                         GPIO_PORT6 | GPIO_PIN9  | GPIO_PSEL_SPI)
#define PIN_SPI0_CS0    (GPIO_OUTPUT | GPIO_PULLUP | GPIO_OUTPUT_ONE | \
                         GPIO_PORT6 | GPIO_PIN12)  /* P612 - ICM20948 CS */
#define PIN_SPI0_CS1    (GPIO_OUTPUT | GPIO_PULLUP | GPIO_OUTPUT_ONE | \
                         GPIO_PORT6 | GPIO_PIN5)   /* P605 - BMP388 CS */

#define PIN_P612        PIN_SPI0_CS0
#define PIN_P605        PIN_SPI0_CS1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ICM-20948 sensor data structure */
struct icm20948_data_s
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t temp;
};

/* BMP388 sensor data structure */
struct bmp388_data_s
{
  uint32_t pressure;    /* Pressure in Pa */
  int32_t temperature;  /* Temperature in 0.01°C */
};

/* GY-912 combined sensor data */
struct gy912_data_s
{
  struct icm20948_data_s imu;
  struct bmp388_data_s baro;
  uint32_t timestamp;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_initialize
 *
 * Description:
 *   Initialize the GY-912 10DOF sensor module
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy912_initialize(void);

/****************************************************************************
 * Name: icm20948_read_reg
 *
 * Description:
 *   Read a register from ICM-20948
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint8_t icm20948_read_reg(uint8_t reg);

/****************************************************************************
 * Name: icm20948_write_reg
 *
 * Description:
 *   Write a register to ICM-20948
 *
 * Input Parameters:
 *   reg - Register address
 *   val - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icm20948_write_reg(uint8_t reg, uint8_t val);

/****************************************************************************
 * Name: icm20948_read_data
 *
 * Description:
 *   Read sensor data from ICM-20948
 *
 * Input Parameters:
 *   data - Pointer to data structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int icm20948_read_data(struct icm20948_data_s *data);

/****************************************************************************
 * Name: bmp388_read_reg
 *
 * Description:
 *   Read a register from BMP388
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint8_t bmp388_read_reg(uint8_t reg);

/****************************************************************************
 * Name: bmp388_write_reg
 *
 * Description:
 *   Write a register to BMP388
 *
 * Input Parameters:
 *   reg - Register address
 *   val - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bmp388_write_reg(uint8_t reg, uint8_t val);

/****************************************************************************
 * Name: bmp388_read_data
 *
 * Description:
 *   Read sensor data from BMP388
 *
 * Input Parameters:
 *   data - Pointer to data structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp388_read_data(struct bmp388_data_s *data);

/****************************************************************************
 * Name: gy912_read_all
 *
 * Description:
 *   Read data from all GY-912 sensors
 *
 * Input Parameters:
 *   data - Pointer to combined data structure
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy912_read_all(struct gy912_data_s *data);

/****************************************************************************
 * Name: gy912_self_test
 *
 * Description:
 *   Perform self-test on GY-912 sensors
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy912_self_test(void);

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_SPI_GY912_H */
