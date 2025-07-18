/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_gy912.c
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
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <assert.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra8e1_fpb.h"

#ifdef CONFIG_RA8_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GY-912 Module contains ICM-20948 (9-axis IMU) and BMP388 (Pressure sensor) */

/* ICM-20948 I2C Address */
#define ICM20948_I2C_ADDR       0x68    /* Default I2C address */
#define ICM20948_I2C_ADDR_ALT   0x69    /* Alternative I2C address */

/* BMP388 I2C Address */
#define BMP388_I2C_ADDR         0x76    /* Default I2C address */
#define BMP388_I2C_ADDR_ALT     0x77    /* Alternative I2C address */

/* ICM-20948 Registers */
#define ICM20948_WHO_AM_I       0x00    /* Device ID register */
#define ICM20948_USER_CTRL      0x03    /* User control register */
#define ICM20948_PWR_MGMT_1     0x06    /* Power management 1 */
#define ICM20948_PWR_MGMT_2     0x07    /* Power management 2 */
#define ICM20948_INT_PIN_CFG    0x0F    /* Interrupt pin configuration */
#define ICM20948_INT_ENABLE     0x10    /* Interrupt enable */
#define ICM20948_INT_ENABLE_1   0x11    /* Interrupt enable 1 */
#define ICM20948_INT_ENABLE_2   0x12    /* Interrupt enable 2 */
#define ICM20948_INT_ENABLE_3   0x13    /* Interrupt enable 3 */
#define ICM20948_ACCEL_XOUT_H   0x2D    /* Accelerometer X-axis high byte */
#define ICM20948_ACCEL_XOUT_L   0x2E    /* Accelerometer X-axis low byte */
#define ICM20948_ACCEL_YOUT_H   0x2F    /* Accelerometer Y-axis high byte */
#define ICM20948_ACCEL_YOUT_L   0x30    /* Accelerometer Y-axis low byte */
#define ICM20948_ACCEL_ZOUT_H   0x31    /* Accelerometer Z-axis high byte */
#define ICM20948_ACCEL_ZOUT_L   0x32    /* Accelerometer Z-axis low byte */
#define ICM20948_GYRO_XOUT_H    0x33    /* Gyroscope X-axis high byte */
#define ICM20948_GYRO_XOUT_L    0x34    /* Gyroscope X-axis low byte */
#define ICM20948_GYRO_YOUT_H    0x35    /* Gyroscope Y-axis high byte */
#define ICM20948_GYRO_YOUT_L    0x36    /* Gyroscope Y-axis low byte */
#define ICM20948_GYRO_ZOUT_H    0x37    /* Gyroscope Z-axis high byte */
#define ICM20948_GYRO_ZOUT_L    0x38    /* Gyroscope Z-axis low byte */
#define ICM20948_TEMP_OUT_H     0x39    /* Temperature high byte */
#define ICM20948_TEMP_OUT_L     0x3A    /* Temperature low byte */

/* ICM-20948 Expected WHO_AM_I value */
#define ICM20948_WHO_AM_I_VALUE 0xEA

/* BMP388 Registers */
#define BMP388_CHIP_ID          0x00    /* Chip ID register */
#define BMP388_ERR_REG          0x02    /* Error register */
#define BMP388_STATUS           0x03    /* Status register */
#define BMP388_DATA_0           0x04    /* Pressure data 0 (LSB) */
#define BMP388_DATA_1           0x05    /* Pressure data 1 */
#define BMP388_DATA_2           0x06    /* Pressure data 2 */
#define BMP388_DATA_3           0x07    /* Temperature data 0 (LSB) */
#define BMP388_DATA_4           0x08    /* Temperature data 1 */
#define BMP388_DATA_5           0x09    /* Temperature data 2 (MSB) */
#define BMP388_PWR_CTRL         0x1B    /* Power control register */
#define BMP388_OSR              0x1C    /* Oversampling register */
#define BMP388_ODR              0x1D    /* Output data rate register */
#define BMP388_CONFIG           0x1F    /* Configuration register */

/* BMP388 Expected CHIP_ID value */
#define BMP388_CHIP_ID_VALUE    0x50

/* I2C1 Configuration for sensor expansion */
#define I2C1_PORT               1
#define I2C1_FREQUENCY          400000  /* 400 kHz fast mode */

/* GY-912 Data Structure */
struct gy912_data_s
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

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gy912_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
static int gy912_i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);
static int gy912_icm20948_init(void);
static int gy912_bmp388_init(void);
static int gy912_icm20948_read(struct gy912_data_s *data);
static int gy912_bmp388_read(struct gy912_data_s *data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_master_s *g_i2c_dev = NULL;
static bool g_gy912_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_i2c_read_reg
 *
 * Description:
 *   Read register from I2C device
 *
 ****************************************************************************/

static int gy912_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
  struct i2c_msg_s msg[2];
  int ret;

  if (g_i2c_dev == NULL || data == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Setup write message for register address */
  msg[0].frequency = I2C1_FREQUENCY;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  /* Setup read message for data */
  msg[1].frequency = I2C1_FREQUENCY;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = data;
  msg[1].length    = len;

  /* Perform transfer */
  ret = I2C_TRANSFER(g_i2c_dev, msg, 2);
  if (ret < 0)
    {
      i2cerr("I2C read failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: gy912_i2c_write_reg
 *
 * Description:
 *   Write register to I2C device
 *
 ****************************************************************************/

static int gy912_i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
  struct i2c_msg_s msg;
  uint8_t *buffer;
  int ret;

  if (g_i2c_dev == NULL || data == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Allocate buffer for register address + data */
  buffer = kmm_malloc(len + 1);
  if (buffer == NULL)
    {
      return -ENOMEM;
    }

  /* Prepare buffer: register address followed by data */
  buffer[0] = reg;
  memcpy(&buffer[1], data, len);

  /* Setup message for register write */
  msg.frequency = I2C1_FREQUENCY;
  msg.addr      = addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = len + 1;

  /* Perform transfer */
  ret = I2C_TRANSFER(g_i2c_dev, &msg, 1);
  if (ret < 0)
    {
      i2cerr("I2C write failed: %d\n", ret);
    }

  /* Free buffer */
  kmm_free(buffer);

  return ret;
}

/****************************************************************************
 * Name: gy912_icm20948_init
 *
 * Description:
 *   Initialize ICM-20948 IMU sensor
 *
 ****************************************************************************/

static int gy912_icm20948_init(void)
{
  uint8_t who_am_i;
  uint8_t reg_data;
  int ret;

  i2cinfo("Initializing ICM-20948 IMU\n");

  /* Read WHO_AM_I register to verify device */
  ret = gy912_i2c_read_reg(ICM20948_I2C_ADDR, ICM20948_WHO_AM_I, &who_am_i, 1);
  if (ret < 0)
    {
      /* Try alternative address */
      ret = gy912_i2c_read_reg(ICM20948_I2C_ADDR_ALT, ICM20948_WHO_AM_I, &who_am_i, 1);
      if (ret < 0)
        {
          i2cerr("ICM-20948 not found\n");
          return ret;
        }
    }

  if (who_am_i != ICM20948_WHO_AM_I_VALUE)
    {
      i2cerr("ICM-20948 WHO_AM_I mismatch: expected 0x%02X, got 0x%02X\n",
             ICM20948_WHO_AM_I_VALUE, who_am_i);
      return -ENODEV;
    }

  i2cinfo("ICM-20948 found, WHO_AM_I: 0x%02X\n", who_am_i);

  /* Reset device */
  reg_data = 0x80;  /* Device reset bit */
  ret = gy912_i2c_write_reg(ICM20948_I2C_ADDR, ICM20948_PWR_MGMT_1, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for reset to complete */
  up_mdelay(100);

  /* Wake up device (clear sleep mode) */
  reg_data = 0x01;  /* Auto select clock source */
  ret = gy912_i2c_write_reg(ICM20948_I2C_ADDR, ICM20948_PWR_MGMT_1, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable accelerometer and gyroscope */
  reg_data = 0x00;  /* Enable accel and gyro */
  ret = gy912_i2c_write_reg(ICM20948_I2C_ADDR, ICM20948_PWR_MGMT_2, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure user control */
  reg_data = 0x00;  /* Disable DMP, FIFO, and I2C master */
  ret = gy912_i2c_write_reg(ICM20948_I2C_ADDR, ICM20948_USER_CTRL, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  i2cinfo("ICM-20948 initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: gy912_bmp388_init
 *
 * Description:
 *   Initialize BMP388 pressure sensor
 *
 ****************************************************************************/

static int gy912_bmp388_init(void)
{
  uint8_t chip_id;
  uint8_t reg_data;
  int ret;

  i2cinfo("Initializing BMP388 pressure sensor\n");

  /* Read CHIP_ID register to verify device */
  ret = gy912_i2c_read_reg(BMP388_I2C_ADDR, BMP388_CHIP_ID, &chip_id, 1);
  if (ret < 0)
    {
      /* Try alternative address */
      ret = gy912_i2c_read_reg(BMP388_I2C_ADDR_ALT, BMP388_CHIP_ID, &chip_id, 1);
      if (ret < 0)
        {
          i2cerr("BMP388 not found\n");
          return ret;
        }
    }

  if (chip_id != BMP388_CHIP_ID_VALUE)
    {
      i2cerr("BMP388 CHIP_ID mismatch: expected 0x%02X, got 0x%02X\n",
             BMP388_CHIP_ID_VALUE, chip_id);
      return -ENODEV;
    }

  i2cinfo("BMP388 found, CHIP_ID: 0x%02X\n", chip_id);

  /* Enable pressure and temperature measurement */
  reg_data = 0x33;  /* Enable pressure and temperature, normal mode */
  ret = gy912_i2c_write_reg(BMP388_I2C_ADDR, BMP388_PWR_CTRL, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure oversampling */
  reg_data = 0x24;  /* Pressure oversampling x4, temperature oversampling x2 */
  ret = gy912_i2c_write_reg(BMP388_I2C_ADDR, BMP388_OSR, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure output data rate */
  reg_data = 0x12;  /* 50 Hz output data rate */
  ret = gy912_i2c_write_reg(BMP388_I2C_ADDR, BMP388_ODR, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  /* Configure IIR filter */
  reg_data = 0x04;  /* IIR filter coefficient 3 */
  ret = gy912_i2c_write_reg(BMP388_I2C_ADDR, BMP388_CONFIG, &reg_data, 1);
  if (ret < 0)
    {
      return ret;
    }

  i2cinfo("BMP388 initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: gy912_icm20948_read
 *
 * Description:
 *   Read ICM-20948 sensor data
 *
 ****************************************************************************/

static int gy912_icm20948_read(struct gy912_data_s *data)
{
  uint8_t buffer[14];  /* 6 bytes accel + 2 bytes temp + 6 bytes gyro */
  int ret;

  DEBUGASSERT(data != NULL);

  /* Read all sensor data in one transaction */
  ret = gy912_i2c_read_reg(ICM20948_I2C_ADDR, ICM20948_ACCEL_XOUT_H, buffer, 14);
  if (ret < 0)
    {
      return ret;
    }

  /* Parse accelerometer data (big endian) */
  data->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
  data->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
  data->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

  /* Parse temperature data (big endian) */
  data->temperature_imu = (int16_t)((buffer[6] << 8) | buffer[7]);

  /* Parse gyroscope data (big endian) */
  data->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
  data->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
  data->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

  return OK;
}

/****************************************************************************
 * Name: gy912_bmp388_read
 *
 * Description:
 *   Read BMP388 sensor data
 *
 ****************************************************************************/

static int gy912_bmp388_read(struct gy912_data_s *data)
{
  uint8_t buffer[6];  /* 3 bytes pressure + 3 bytes temperature */
  int ret;

  DEBUGASSERT(data != NULL);

  /* Read pressure and temperature data */
  ret = gy912_i2c_read_reg(BMP388_I2C_ADDR, BMP388_DATA_0, buffer, 6);
  if (ret < 0)
    {
      return ret;
    }

  /* Parse pressure data (little endian, 24-bit) */
  data->pressure = (uint32_t)(buffer[0] | (buffer[1] << 8) | (buffer[2] << 16));

  /* Parse temperature data (little endian, 24-bit) */
  data->temperature_bmp = (int32_t)(buffer[3] | (buffer[4] << 8) | (buffer[5] << 16));
  
  /* Sign extend 24-bit temperature to 32-bit */
  if (data->temperature_bmp & 0x800000)
    {
      data->temperature_bmp |= 0xFF000000;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_gy912_initialize
 *
 * Description:
 *   Initialize GY-912 sensor module (ICM-20948 + BMP388)
 *
 ****************************************************************************/

int ra8e1_gy912_initialize(void)
{
  int ret;

  if (g_gy912_initialized)
    {
      return OK;
    }

  i2cinfo("Initializing GY-912 sensor module\n");

  /* Get I2C interface */
  g_i2c_dev = ra_i2cbus_initialize(I2C1_PORT);
  if (g_i2c_dev == NULL)
    {
      i2cerr("Failed to initialize I2C%d\n", I2C1_PORT);
      return -ENODEV;
    }

  /* Initialize ICM-20948 IMU */
  ret = gy912_icm20948_init();
  if (ret < 0)
    {
      i2cerr("Failed to initialize ICM-20948: %d\n", ret);
      return ret;
    }

  /* Initialize BMP388 pressure sensor */
  ret = gy912_bmp388_init();
  if (ret < 0)
    {
      i2cerr("Failed to initialize BMP388: %d\n", ret);
      return ret;
    }

  g_gy912_initialized = true;
  i2cinfo("GY-912 sensor module initialized successfully\n");

  return OK;
}

/****************************************************************************
 * Name: ra8e1_gy912_read_data
 *
 * Description:
 *   Read data from GY-912 sensor module
 *
 ****************************************************************************/

int ra8e1_gy912_read_data(struct gy912_data_s *data)
{
  int ret;

  if (!g_gy912_initialized || data == NULL)
    {
      return -EINVAL;
    }

  /* Clear data structure */
  memset(data, 0, sizeof(struct gy912_data_s));

  /* Read ICM-20948 data */
  ret = gy912_icm20948_read(data);
  if (ret < 0)
    {
      i2cerr("Failed to read ICM-20948 data: %d\n", ret);
      return ret;
    }

  /* Read BMP388 data */
  ret = gy912_bmp388_read(data);
  if (ret < 0)
    {
      i2cerr("Failed to read BMP388 data: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ra8e1_gy912_print_data
 *
 * Description:
 *   Print GY-912 sensor data in a readable format
 *
 ****************************************************************************/

void ra8e1_gy912_print_data(const struct gy912_data_s *data)
{
  if (data == NULL)
    {
      return;
    }

  printf("GY-912 Sensor Data:\n");
  printf("  ICM-20948 IMU:\n");
  printf("    Accel: X=%6d, Y=%6d, Z=%6d\n", 
         data->accel_x, data->accel_y, data->accel_z);
  printf("    Gyro:  X=%6d, Y=%6d, Z=%6d\n", 
         data->gyro_x, data->gyro_y, data->gyro_z);
  printf("    Temp:  %d\n", data->temperature_imu);
  printf("  BMP388 Pressure Sensor:\n");
  printf("    Pressure: %lu\n", (unsigned long)data->pressure);
  printf("    Temp:     %ld\n", (long)data->temperature_bmp);
}

/****************************************************************************
 * Name: ra8e1_gy912_test
 *
 * Description:
 *   Test GY-912 sensor module functionality
 *
 ****************************************************************************/

int ra8e1_gy912_test(void)
{
  struct gy912_data_s data;
  int ret;
  int i;

  i2cinfo("Starting GY-912 sensor test\n");

  /* Initialize the sensor */
  ret = ra8e1_gy912_initialize();
  if (ret < 0)
    {
      i2cerr("Failed to initialize GY-912: %d\n", ret);
      return ret;
    }

  /* Read and display data 10 times */
  for (i = 0; i < 10; i++)
    {
      ret = ra8e1_gy912_read_data(&data);
      if (ret < 0)
        {
          i2cerr("Failed to read GY-912 data: %d\n", ret);
          return ret;
        }

      printf("\n--- Sample %d ---\n", i + 1);
      ra8e1_gy912_print_data(&data);

      /* Wait 500ms between readings */
      up_mdelay(500);
    }

  i2cinfo("GY-912 sensor test completed successfully\n");
  return OK;
}

#endif /* CONFIG_RA8_I2C */
