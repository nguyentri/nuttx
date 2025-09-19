/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_spi_gy912.c
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

#ifdef CONFIG_RA8E1_SPI_GY912_EXAMPLE

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <assert.h>
#include <time.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/sensors/ioctl.h>
#include <nuttx/uorb.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BMP388 Register Definitions */
#define BMP388_CHIP_ID_REG      0x00
#define BMP388_CHIP_ID_VALUE    0x50
#define BMP388_ERR_REG          0x02
#define BMP388_STATUS_REG       0x03
#define BMP388_DATA_0_REG       0x04
#define BMP388_DATA_1_REG       0x05
#define BMP388_DATA_2_REG       0x06
#define BMP388_DATA_3_REG       0x07
#define BMP388_DATA_4_REG       0x08
#define BMP388_DATA_5_REG       0x09
#define BMP388_SENSORTIME_0_REG 0x0C
#define BMP388_SENSORTIME_1_REG 0x0D
#define BMP388_SENSORTIME_2_REG 0x0E
#define BMP388_EVENT_REG        0x10
#define BMP388_INT_STATUS_REG   0x11
#define BMP388_FIFO_LENGTH_0_REG 0x12
#define BMP388_FIFO_LENGTH_1_REG 0x13
#define BMP388_FIFO_DATA_REG    0x14
#define BMP388_FIFO_WTM_0_REG   0x15
#define BMP388_FIFO_WTM_1_REG   0x16
#define BMP388_FIFO_CONFIG_1_REG 0x17
#define BMP388_FIFO_CONFIG_2_REG 0x18
#define BMP388_INT_CTRL_REG     0x19
#define BMP388_IF_CONF_REG      0x1A
#define BMP388_PWR_CTRL_REG     0x1B
#define BMP388_OSR_REG          0x1C
#define BMP388_ODR_REG          0x1D
#define BMP388_CONFIG_REG       0x1F
#define BMP388_CMD_REG          0x7E

/* BMP388 Commands */
#define BMP388_CMD_SOFT_RESET   0xB6
#define BMP388_CMD_FIFO_FLUSH   0xB0

/* BMP388 Power Control */
#define BMP388_PWR_CTRL_PRESS_EN (1 << 0)
#define BMP388_PWR_CTRL_TEMP_EN  (1 << 1)
#define BMP388_PWR_CTRL_MODE_MASK (0x03 << 4)
#define BMP388_PWR_CTRL_MODE_SLEEP (0x00 << 4)
#define BMP388_PWR_CTRL_MODE_FORCED (0x01 << 4)
#define BMP388_PWR_CTRL_MODE_NORMAL (0x03 << 4)

/* ICM20948 Register Definitions */
#define ICM20948_WHO_AM_I       0x00
#define ICM20948_WHO_AM_I_VALUE 0xEA
#define ICM20948_USER_CTRL      0x03
#define ICM20948_LP_CONFIG      0x05
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_PWR_MGMT_2     0x07
#define ICM20948_INT_PIN_CFG    0x0F
#define ICM20948_INT_ENABLE     0x10
#define ICM20948_INT_ENABLE_1   0x11
#define ICM20948_INT_ENABLE_2   0x12
#define ICM20948_INT_ENABLE_3   0x13
#define ICM20948_I2C_MST_STATUS 0x17
#define ICM20948_INT_STATUS     0x19
#define ICM20948_INT_STATUS_1   0x1A
#define ICM20948_INT_STATUS_2   0x1B
#define ICM20948_INT_STATUS_3   0x1C
#define ICM20948_DELAY_TIMEH    0x28
#define ICM20948_DELAY_TIMEL    0x29
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
#define ICM20948_TEMP_OUT_H     0x39
#define ICM20948_TEMP_OUT_L     0x3A

/* ICM20948 Power Management */
#define ICM20948_PWR_MGMT_1_DEVICE_RESET (1 << 7)
#define ICM20948_PWR_MGMT_1_SLEEP        (1 << 6)
#define ICM20948_PWR_MGMT_1_LP_EN        (1 << 5)
#define ICM20948_PWR_MGMT_1_TEMP_DIS     (1 << 3)
#define ICM20948_PWR_MGMT_1_CLKSEL_MASK  (0x07 << 0)
#define ICM20948_PWR_MGMT_1_CLKSEL_AUTO  (0x01 << 0)

#define ICM20948_PWR_MGMT_2_DISABLE_ACCEL (0x07 << 3)
#define ICM20948_PWR_MGMT_2_DISABLE_GYRO  (0x07 << 0)

/* SPI Device IDs */
#define GY912_SPI_BMP388_DEVID   0
#define GY912_SPI_ICM20948_DEVID 1

/* Sensor update intervals */
#define GY912_DEFAULT_INTERVAL_US 100000  /* 100ms */
#define GY912_MIN_INTERVAL_US     1000    /* 1ms */
#define GY912_MAX_INTERVAL_US     1000000 /* 1s */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GY-912 sensor device structure */
struct gy912_sensor_dev_s
{
  struct sensor_lowerhalf_s lower;  /* Lower half sensor interface */
  FAR struct spi_dev_s *spi;        /* SPI device */
  uint32_t devid;                   /* SPI device ID */
  uint32_t frequency;               /* SPI frequency */
  uint32_t interval_us;             /* Sampling interval in microseconds */
  bool enabled;                     /* Sensor enabled flag */
  sem_t exclsem;                    /* Mutual exclusion semaphore */

  /* Sensor-specific data */
  union
  {
    struct sensor_baro baro_data;   /* Barometer data */
    struct sensor_accel accel_data; /* Accelerometer data */
    struct sensor_gyro gyro_data;   /* Gyroscope data */
    struct sensor_mag mag_data;     /* Magnetometer data */
  } data;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI helper functions */
static int gy912_spi_read_reg(FAR struct gy912_sensor_dev_s *priv,
                              uint8_t reg, FAR uint8_t *value);
static int gy912_spi_write_reg(FAR struct gy912_sensor_dev_s *priv,
                               uint8_t reg, uint8_t value);
static int gy912_spi_read_block(FAR struct gy912_sensor_dev_s *priv,
                                uint8_t reg, FAR uint8_t *buffer, size_t len);

/* BMP388 functions */
static int gy912_bmp388_initialize(FAR struct gy912_sensor_dev_s *priv);
static int gy912_bmp388_read_data(FAR struct gy912_sensor_dev_s *priv);
static int gy912_bmp388_selftest(FAR struct gy912_sensor_dev_s *priv,
                                 FAR uint32_t *result);

/* ICM20948 functions */
static int gy912_icm20948_initialize(FAR struct gy912_sensor_dev_s *priv);
static int gy912_icm20948_read_accel(FAR struct gy912_sensor_dev_s *priv);
static int gy912_icm20948_read_gyro(FAR struct gy912_sensor_dev_s *priv);
static int gy912_icm20948_read_mag(FAR struct gy912_sensor_dev_s *priv);
static int gy912_icm20948_selftest(FAR struct gy912_sensor_dev_s *priv,
                                   FAR uint32_t *result);

/* Sensor interface functions */
static int gy912_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep, bool enable);
static int gy912_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us);
static int gy912_sensor_fetch(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR char *buffer, size_t buflen);
static int gy912_sensor_selftest(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *result);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sensor operations structure */
static const struct sensor_ops_s g_gy912_sensor_ops =
{
  .activate   = gy912_sensor_activate,
  .set_interval = gy912_sensor_set_interval,
  .fetch      = gy912_sensor_fetch,
  .selftest   = gy912_sensor_selftest,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_spi_read_reg
 *
 * Description:
 *   Read a single register from the sensor via SPI
 *
 ****************************************************************************/

static int gy912_spi_read_reg(FAR struct gy912_sensor_dev_s *priv,
                              uint8_t reg, FAR uint8_t *value)
{
  uint8_t txbuf[2];
  uint8_t rxbuf[2];
  int ret;

  DEBUGASSERT(priv != NULL && value != NULL);

  /* Lock the SPI bus */
  SPI_LOCK(priv->spi, true);

  /* Select the device */
  SPI_SELECT(priv->spi, priv->devid, true);

  /* Set up the transfer */
  SPI_SETFREQUENCY(priv->spi, priv->frequency);
  SPI_SETMODE(priv->spi, SPIDEV_MODE3);
  SPI_SETBITS(priv->spi, 8);

  /* Prepare the command (read bit set for most sensors) */
  txbuf[0] = reg | 0x80;  /* Read command */
  txbuf[1] = 0x00;        /* Dummy byte */

  /* Perform the transfer */
  SPI_EXCHANGE(priv->spi, txbuf, rxbuf, 2);

  /* Deselect the device */
  SPI_SELECT(priv->spi, priv->devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(priv->spi, false);

  *value = rxbuf[1];
  ret = OK;

  return ret;
}

/****************************************************************************
 * Name: gy912_spi_write_reg
 *
 * Description:
 *   Write a single register to the sensor via SPI
 *
 ****************************************************************************/

static int gy912_spi_write_reg(FAR struct gy912_sensor_dev_s *priv,
                               uint8_t reg, uint8_t value)
{
  uint8_t txbuf[2];
  int ret;

  DEBUGASSERT(priv != NULL);

  /* Lock the SPI bus */
  SPI_LOCK(priv->spi, true);

  /* Select the device */
  SPI_SELECT(priv->spi, priv->devid, true);

  /* Set up the transfer */
  SPI_SETFREQUENCY(priv->spi, priv->frequency);
  SPI_SETMODE(priv->spi, SPIDEV_MODE3);
  SPI_SETBITS(priv->spi, 8);

  /* Prepare the command (write bit clear) */
  txbuf[0] = reg & 0x7F;  /* Write command */
  txbuf[1] = value;

  /* Perform the transfer */
  SPI_SNDBLOCK(priv->spi, txbuf, 2);

  /* Deselect the device */
  SPI_SELECT(priv->spi, priv->devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(priv->spi, false);

  ret = OK;
  return ret;
}

/****************************************************************************
 * Name: gy912_spi_read_block
 *
 * Description:
 *   Read multiple registers from the sensor via SPI
 *
 ****************************************************************************/

static int gy912_spi_read_block(FAR struct gy912_sensor_dev_s *priv,
                                uint8_t reg, FAR uint8_t *buffer, size_t len)
{
  uint8_t *txbuf;
  uint8_t *rxbuf;
  int ret;
  size_t i;

  DEBUGASSERT(priv != NULL && buffer != NULL && len > 0);

  /* Allocate temporary buffers */
  txbuf = kmm_malloc(len + 1);
  rxbuf = kmm_malloc(len + 1);
  if (txbuf == NULL || rxbuf == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Lock the SPI bus */
  SPI_LOCK(priv->spi, true);

  /* Select the device */
  SPI_SELECT(priv->spi, priv->devid, true);

  /* Set up the transfer */
  SPI_SETFREQUENCY(priv->spi, priv->frequency);
  SPI_SETMODE(priv->spi, SPIDEV_MODE3);
  SPI_SETBITS(priv->spi, 8);

  /* Prepare the command */
  txbuf[0] = reg | 0x80;  /* Read command */
  for (i = 1; i <= len; i++)
    {
      txbuf[i] = 0x00;    /* Dummy bytes */
    }

  /* Perform the transfer */
  SPI_EXCHANGE(priv->spi, txbuf, rxbuf, len + 1);

  /* Deselect the device */
  SPI_SELECT(priv->spi, priv->devid, false);

  /* Unlock the SPI bus */
  SPI_LOCK(priv->spi, false);

  /* Copy the received data (skip the first dummy byte) */
  memcpy(buffer, &rxbuf[1], len);
  ret = OK;

errout:
  if (txbuf != NULL)
    {
      kmm_free(txbuf);
    }
  if (rxbuf != NULL)
    {
      kmm_free(rxbuf);
    }

  return ret;
}

/****************************************************************************
 * Name: gy912_bmp388_initialize
 *
 * Description:
 *   Initialize the BMP388 sensor
 *
 ****************************************************************************/

static int gy912_bmp388_initialize(FAR struct gy912_sensor_dev_s *priv)
{
  uint8_t chip_id;
  int ret;

  sinfo("Initializing BMP388 sensor\n");

  /* Read chip ID to verify communication */
  ret = gy912_spi_read_reg(priv, BMP388_CHIP_ID_REG, &chip_id);
  if (ret < 0)
    {
      serr("Failed to read BMP388 chip ID: %d\n", ret);
      return ret;
    }

  if (chip_id != BMP388_CHIP_ID_VALUE)
    {
      serr("Invalid BMP388 chip ID: 0x%02x (expected 0x%02x)\n",
           chip_id, BMP388_CHIP_ID_VALUE);
      return -ENODEV;
    }

  sinfo("BMP388 chip ID verified: 0x%02x\n", chip_id);

  /* Perform soft reset */
  ret = gy912_spi_write_reg(priv, BMP388_CMD_REG, BMP388_CMD_SOFT_RESET);
  if (ret < 0)
    {
      serr("Failed to reset BMP388: %d\n", ret);
      return ret;
    }

  /* Wait for reset to complete */
  usleep(10000);  /* 10ms */

  /* Configure power control - enable pressure and temperature */
  ret = gy912_spi_write_reg(priv, BMP388_PWR_CTRL_REG,
                           BMP388_PWR_CTRL_PRESS_EN | BMP388_PWR_CTRL_TEMP_EN |
                           BMP388_PWR_CTRL_MODE_NORMAL);
  if (ret < 0)
    {
      serr("Failed to configure BMP388 power control: %d\n", ret);
      return ret;
    }

  /* Configure oversampling (default settings) */
  ret = gy912_spi_write_reg(priv, BMP388_OSR_REG, 0x00);
  if (ret < 0)
    {
      serr("Failed to configure BMP388 oversampling: %d\n", ret);
      return ret;
    }

  /* Configure output data rate */
  ret = gy912_spi_write_reg(priv, BMP388_ODR_REG, 0x00);
  if (ret < 0)
    {
      serr("Failed to configure BMP388 ODR: %d\n", ret);
      return ret;
    }

  sinfo("BMP388 initialization completed successfully\n");
  return OK;
}

/****************************************************************************
 * Name: gy912_bmp388_read_data
 *
 * Description:
 *   Read pressure and temperature data from BMP388
 *
 ****************************************************************************/

static int gy912_bmp388_read_data(FAR struct gy912_sensor_dev_s *priv)
{
  uint8_t data[6];
  uint32_t raw_pressure;
  uint32_t raw_temperature;
  struct timespec ts;
  int ret;

  /* Read pressure and temperature data */
  ret = gy912_spi_read_block(priv, BMP388_DATA_0_REG, data, 6);
  if (ret < 0)
    {
      serr("Failed to read BMP388 data: %d\n", ret);
      return ret;
    }

  /* Extract raw values */
  raw_pressure = (uint32_t)data[2] << 16 | (uint32_t)data[1] << 8 | data[0];
  raw_temperature = (uint32_t)data[5] << 16 | (uint32_t)data[4] << 8 | data[3];

  /* Get current timestamp */
  clock_gettime(CLOCK_REALTIME, &ts);

  /* Convert and store data (simplified conversion for demo) */
  priv->data.baro_data.timestamp = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  priv->data.baro_data.pressure = (float)raw_pressure / 100.0f;  /* Convert to hPa */
  priv->data.baro_data.temperature = (float)raw_temperature / 100.0f;  /* Convert to °C */

  return OK;
}

/****************************************************************************
 * Name: gy912_bmp388_selftest
 *
 * Description:
 *   Perform BMP388 self-test
 *
 ****************************************************************************/

static int gy912_bmp388_selftest(FAR struct gy912_sensor_dev_s *priv,
                                 FAR uint32_t *result)
{
  uint8_t chip_id;
  uint8_t status;
  int ret;

  *result = 0;

  /* Test 1: Verify chip ID */
  ret = gy912_spi_read_reg(priv, BMP388_CHIP_ID_REG, &chip_id);
  if (ret < 0 || chip_id != BMP388_CHIP_ID_VALUE)
    {
      *result |= (1 << 0);  /* Chip ID test failed */
    }

  /* Test 2: Check status register */
  ret = gy912_spi_read_reg(priv, BMP388_STATUS_REG, &status);
  if (ret < 0)
    {
      *result |= (1 << 1);  /* Status read failed */
    }

  /* Test 3: Try to read sensor data */
  ret = gy912_bmp388_read_data(priv);
  if (ret < 0)
    {
      *result |= (1 << 2);  /* Data read failed */
    }

  return OK;
}

/****************************************************************************
 * Name: gy912_icm20948_initialize
 *
 * Description:
 *   Initialize the ICM20948 sensor
 *
 ****************************************************************************/

static int gy912_icm20948_initialize(FAR struct gy912_sensor_dev_s *priv)
{
  uint8_t who_am_i;
  int ret;

  sinfo("Initializing ICM20948 sensor\n");

  /* Read WHO_AM_I register to verify communication */
  ret = gy912_spi_read_reg(priv, ICM20948_WHO_AM_I, &who_am_i);
  if (ret < 0)
    {
      serr("Failed to read ICM20948 WHO_AM_I: %d\n", ret);
      return ret;
    }

  if (who_am_i != ICM20948_WHO_AM_I_VALUE)
    {
      serr("Invalid ICM20948 WHO_AM_I: 0x%02x (expected 0x%02x)\n",
           who_am_i, ICM20948_WHO_AM_I_VALUE);
      return -ENODEV;
    }

  sinfo("ICM20948 WHO_AM_I verified: 0x%02x\n", who_am_i);

  /* Perform device reset */
  ret = gy912_spi_write_reg(priv, ICM20948_PWR_MGMT_1,
                           ICM20948_PWR_MGMT_1_DEVICE_RESET);
  if (ret < 0)
    {
      serr("Failed to reset ICM20948: %d\n", ret);
      return ret;
    }

  /* Wait for reset to complete */
  usleep(100000);  /* 100ms */

  /* Wake up the device and select auto clock */
  ret = gy912_spi_write_reg(priv, ICM20948_PWR_MGMT_1,
                           ICM20948_PWR_MGMT_1_CLKSEL_AUTO);
  if (ret < 0)
    {
      serr("Failed to configure ICM20948 power management 1: %d\n", ret);
      return ret;
    }

  /* Enable accelerometer and gyroscope */
  ret = gy912_spi_write_reg(priv, ICM20948_PWR_MGMT_2, 0x00);
  if (ret < 0)
    {
      serr("Failed to configure ICM20948 power management 2: %d\n", ret);
      return ret;
    }

  sinfo("ICM20948 initialization completed successfully\n");
  return OK;
}

/****************************************************************************
 * Name: gy912_icm20948_read_accel
 *
 * Description:
 *   Read accelerometer data from ICM20948
 *
 ****************************************************************************/

static int gy912_icm20948_read_accel(FAR struct gy912_sensor_dev_s *priv)
{
  uint8_t data[6];
  int16_t raw_x, raw_y, raw_z;
  struct timespec ts;
  int ret;

  /* Read accelerometer data */
  ret = gy912_spi_read_block(priv, ICM20948_ACCEL_XOUT_H, data, 6);
  if (ret < 0)
    {
      serr("Failed to read ICM20948 accelerometer data: %d\n", ret);
      return ret;
    }

  /* Extract raw values */
  raw_x = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  raw_y = (int16_t)((uint16_t)data[2] << 8 | data[3]);
  raw_z = (int16_t)((uint16_t)data[4] << 8 | data[5]);

  /* Get current timestamp */
  clock_gettime(CLOCK_REALTIME, &ts);

  /* Convert and store data (simplified conversion for demo) */
  priv->data.accel_data.timestamp = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  priv->data.accel_data.x = (float)raw_x * 9.81f / 16384.0f;  /* Convert to m/s² */
  priv->data.accel_data.y = (float)raw_y * 9.81f / 16384.0f;
  priv->data.accel_data.z = (float)raw_z * 9.81f / 16384.0f;
  priv->data.accel_data.temperature = 25.0f;  /* Placeholder temperature */

  return OK;
}

/****************************************************************************
 * Name: gy912_icm20948_read_gyro
 *
 * Description:
 *   Read gyroscope data from ICM20948
 *
 ****************************************************************************/

static int gy912_icm20948_read_gyro(FAR struct gy912_sensor_dev_s *priv)
{
  uint8_t data[6];
  int16_t raw_x, raw_y, raw_z;
  struct timespec ts;
  int ret;

  /* Read gyroscope data */
  ret = gy912_spi_read_block(priv, ICM20948_GYRO_XOUT_H, data, 6);
  if (ret < 0)
    {
      serr("Failed to read ICM20948 gyroscope data: %d\n", ret);
      return ret;
    }

  /* Extract raw values */
  raw_x = (int16_t)((uint16_t)data[0] << 8 | data[1]);
  raw_y = (int16_t)((uint16_t)data[2] << 8 | data[3]);
  raw_z = (int16_t)((uint16_t)data[4] << 8 | data[5]);

  /* Get current timestamp */
  clock_gettime(CLOCK_REALTIME, &ts);

  /* Convert and store data (simplified conversion for demo) */
  priv->data.gyro_data.timestamp = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  priv->data.gyro_data.x = (float)raw_x * 3.14159f / (180.0f * 131.0f);  /* Convert to rad/s */
  priv->data.gyro_data.y = (float)raw_y * 3.14159f / (180.0f * 131.0f);
  priv->data.gyro_data.z = (float)raw_z * 3.14159f / (180.0f * 131.0f);
  priv->data.gyro_data.temperature = 25.0f;  /* Placeholder temperature */

  return OK;
}

/****************************************************************************
 * Name: gy912_icm20948_read_mag
 *
 * Description:
 *   Read magnetometer data from ICM20948 (placeholder implementation)
 *
 ****************************************************************************/

static int gy912_icm20948_read_mag(FAR struct gy912_sensor_dev_s *priv)
{
  struct timespec ts;

  /* Get current timestamp */
  clock_gettime(CLOCK_REALTIME, &ts);

  /* Placeholder magnetometer data (ICM20948 magnetometer requires I2C master setup) */
  priv->data.mag_data.timestamp = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
  priv->data.mag_data.x = 0.0f;
  priv->data.mag_data.y = 0.0f;
  priv->data.mag_data.z = 0.0f;
  priv->data.mag_data.temperature = 25.0f;
  priv->data.mag_data.status = 0;

  return OK;
}

/****************************************************************************
 * Name: gy912_icm20948_selftest
 *
 * Description:
 *   Perform ICM20948 self-test
 *
 ****************************************************************************/

static int gy912_icm20948_selftest(FAR struct gy912_sensor_dev_s *priv,
                                   FAR uint32_t *result)
{
  uint8_t who_am_i;
  int ret;

  *result = 0;

  /* Test 1: Verify WHO_AM_I */
  ret = gy912_spi_read_reg(priv, ICM20948_WHO_AM_I, &who_am_i);
  if (ret < 0 || who_am_i != ICM20948_WHO_AM_I_VALUE)
    {
      *result |= (1 << 0);  /* WHO_AM_I test failed */
    }

  /* Test 2: Try to read accelerometer data */
  ret = gy912_icm20948_read_accel(priv);
  if (ret < 0)
    {
      *result |= (1 << 1);  /* Accelerometer read failed */
    }

  /* Test 3: Try to read gyroscope data */
  ret = gy912_icm20948_read_gyro(priv);
  if (ret < 0)
    {
      *result |= (1 << 2);  /* Gyroscope read failed */
    }

  return OK;
}

/****************************************************************************
 * Name: gy912_sensor_activate
 *
 * Description:
 *   Activate or deactivate the sensor
 *
 ****************************************************************************/

static int gy912_sensor_activate(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep, bool enable)
{
  FAR struct gy912_sensor_dev_s *priv = (FAR struct gy912_sensor_dev_s *)lower;
  int ret;

  /* Take the semaphore */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  if (enable && !priv->enabled)
    {
      /* Initialize the sensor based on its type */
      switch (lower->type)
        {
          case SENSOR_TYPE_BAROMETER:
            ret = gy912_bmp388_initialize(priv);
            break;

          case SENSOR_TYPE_ACCELEROMETER:
          case SENSOR_TYPE_GYROSCOPE:
          case SENSOR_TYPE_MAGNETIC_FIELD:
            ret = gy912_icm20948_initialize(priv);
            break;

          default:
            ret = -EINVAL;
            break;
        }

      if (ret == OK)
        {
          priv->enabled = true;
          sinfo("Sensor type %d activated\n", lower->type);
        }
    }
  else if (!enable && priv->enabled)
    {
      priv->enabled = false;
      sinfo("Sensor type %d deactivated\n", lower->type);
      ret = OK;
    }
  else
    {
      ret = OK;  /* Already in the requested state */
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: gy912_sensor_set_interval
 *
 * Description:
 *   Set the sensor sampling interval
 *
 ****************************************************************************/

static int gy912_sensor_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                     FAR struct file *filep,
                                     FAR uint32_t *period_us)
{
  FAR struct gy912_sensor_dev_s *priv = (FAR struct gy912_sensor_dev_s *)lower;
  int ret;

  /* Validate the interval */
  if (*period_us < GY912_MIN_INTERVAL_US)
    {
      *period_us = GY912_MIN_INTERVAL_US;
    }
  else if (*period_us > GY912_MAX_INTERVAL_US)
    {
      *period_us = GY912_MAX_INTERVAL_US;
    }

  /* Take the semaphore */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  priv->interval_us = *period_us;
  sinfo("Sensor type %d interval set to %u us\n", lower->type, *period_us);

  nxsem_post(&priv->exclsem);
  return OK;
}

/****************************************************************************
 * Name: gy912_sensor_fetch
 *
 * Description:
 *   Fetch sensor data
 *
 ****************************************************************************/

static int gy912_sensor_fetch(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR char *buffer, size_t buflen)
{
  FAR struct gy912_sensor_dev_s *priv = (FAR struct gy912_sensor_dev_s *)lower;
  int ret;
  size_t data_size;

  if (!priv->enabled)
    {
      return -ENODEV;
    }

  /* Take the semaphore */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Read sensor data based on type */
  switch (lower->type)
    {
      case SENSOR_TYPE_BAROMETER:
        ret = gy912_bmp388_read_data(priv);
        data_size = sizeof(struct sensor_baro);
        break;

      case SENSOR_TYPE_ACCELEROMETER:
        ret = gy912_icm20948_read_accel(priv);
        data_size = sizeof(struct sensor_accel);
        break;

      case SENSOR_TYPE_GYROSCOPE:
        ret = gy912_icm20948_read_gyro(priv);
        data_size = sizeof(struct sensor_gyro);
        break;

      case SENSOR_TYPE_MAGNETIC_FIELD:
        ret = gy912_icm20948_read_mag(priv);
        data_size = sizeof(struct sensor_mag);
        break;

      default:
        ret = -EINVAL;
        data_size = 0;
        break;
    }

  if (ret == OK && buflen >= data_size)
    {
      memcpy(buffer, &priv->data, data_size);
      ret = data_size;
    }
  else if (ret == OK)
    {
      ret = -ENOBUFS;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Name: gy912_sensor_selftest
 *
 * Description:
 *   Perform sensor self-test
 *
 ****************************************************************************/

static int gy912_sensor_selftest(FAR struct sensor_lowerhalf_s *lower,
                                FAR struct file *filep,
                                FAR uint32_t *result)
{
  FAR struct gy912_sensor_dev_s *priv = (FAR struct gy912_sensor_dev_s *)lower;
  int ret;

  /* Take the semaphore */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Perform self-test based on sensor type */
  switch (lower->type)
    {
      case SENSOR_TYPE_BAROMETER:
        ret = gy912_bmp388_selftest(priv, result);
        break;

      case SENSOR_TYPE_ACCELEROMETER:
      case SENSOR_TYPE_GYROSCOPE:
      case SENSOR_TYPE_MAGNETIC_FIELD:
        ret = gy912_icm20948_selftest(priv, result);
        break;

      default:
        ret = -EINVAL;
        *result = 0xFFFFFFFF;
        break;
    }

  nxsem_post(&priv->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_register_sensors
 *
 * Description:
 *   Register GY-912 sensors with the sensor framework
 *
 ****************************************************************************/

int gy912_register_sensors(FAR struct spi_dev_s *spi)
{
  FAR struct gy912_sensor_dev_s *bmp388_dev;
  FAR struct gy912_sensor_dev_s *accel_dev;
  FAR struct gy912_sensor_dev_s *gyro_dev;
  FAR struct gy912_sensor_dev_s *mag_dev;
  int ret;

  DEBUGASSERT(spi != NULL);

  /* Register BMP388 barometer */
  bmp388_dev = kmm_zalloc(sizeof(struct gy912_sensor_dev_s));
  if (bmp388_dev == NULL)
    {
      serr("Failed to allocate BMP388 device structure\n");
      return -ENOMEM;
    }

  bmp388_dev->lower.ops = &g_gy912_sensor_ops;
  bmp388_dev->lower.type = SENSOR_TYPE_BAROMETER;
  bmp388_dev->lower.nbuffer = 1;
  bmp388_dev->spi = spi;
  bmp388_dev->devid = GY912_SPI_BMP388_DEVID;
  bmp388_dev->frequency = 1000000;  /* 1 MHz */
  bmp388_dev->interval_us = GY912_DEFAULT_INTERVAL_US;
  bmp388_dev->enabled = false;
  nxsem_init(&bmp388_dev->exclsem, 0, 1);

  ret = sensor_register(&bmp388_dev->lower, 0);
  if (ret < 0)
    {
      serr("Failed to register BMP388 sensor: %d\n", ret);
      kmm_free(bmp388_dev);
      return ret;
    }

  sinfo("BMP388 barometer registered successfully\n");

  /* Register ICM20948 accelerometer */
  accel_dev = kmm_zalloc(sizeof(struct gy912_sensor_dev_s));
  if (accel_dev == NULL)
    {
      serr("Failed to allocate ICM20948 accelerometer device structure\n");
      return -ENOMEM;
    }

  accel_dev->lower.ops = &g_gy912_sensor_ops;
  accel_dev->lower.type = SENSOR_TYPE_ACCELEROMETER;
  accel_dev->lower.nbuffer = 1;
  accel_dev->spi = spi;
  accel_dev->devid = GY912_SPI_ICM20948_DEVID;
  accel_dev->frequency = 1000000;  /* 1 MHz */
  accel_dev->interval_us = GY912_DEFAULT_INTERVAL_US;
  accel_dev->enabled = false;
  nxsem_init(&accel_dev->exclsem, 0, 1);

  ret = sensor_register(&accel_dev->lower, 0);
  if (ret < 0)
    {
      serr("Failed to register ICM20948 accelerometer: %d\n", ret);
      kmm_free(accel_dev);
      return ret;
    }

  sinfo("ICM20948 accelerometer registered successfully\n");

  /* Register ICM20948 gyroscope */
  gyro_dev = kmm_zalloc(sizeof(struct gy912_sensor_dev_s));
  if (gyro_dev == NULL)
    {
      serr("Failed to allocate ICM20948 gyroscope device structure\n");
      return -ENOMEM;
    }

  gyro_dev->lower.ops = &g_gy912_sensor_ops;
  gyro_dev->lower.type = SENSOR_TYPE_GYROSCOPE;
  gyro_dev->lower.nbuffer = 1;
  gyro_dev->spi = spi;
  gyro_dev->devid = GY912_SPI_ICM20948_DEVID;
  gyro_dev->frequency = 1000000;  /* 1 MHz */
  gyro_dev->interval_us = GY912_DEFAULT_INTERVAL_US;
  gyro_dev->enabled = false;
  nxsem_init(&gyro_dev->exclsem, 0, 1);

  ret = sensor_register(&gyro_dev->lower, 0);
  if (ret < 0)
    {
      serr("Failed to register ICM20948 gyroscope: %d\n", ret);
      kmm_free(gyro_dev);
      return ret;
    }

  sinfo("ICM20948 gyroscope registered successfully\n");

  /* Register ICM20948 magnetometer */
  mag_dev = kmm_zalloc(sizeof(struct gy912_sensor_dev_s));
  if (mag_dev == NULL)
    {
      serr("Failed to allocate ICM20948 magnetometer device structure\n");
      return -ENOMEM;
    }

  mag_dev->lower.ops = &g_gy912_sensor_ops;
  mag_dev->lower.type = SENSOR_TYPE_MAGNETIC_FIELD;
  mag_dev->lower.nbuffer = 1;
  mag_dev->spi = spi;
  mag_dev->devid = GY912_SPI_ICM20948_DEVID;
  mag_dev->frequency = 1000000;  /* 1 MHz */
  mag_dev->interval_us = GY912_DEFAULT_INTERVAL_US;
  mag_dev->enabled = false;
  nxsem_init(&mag_dev->exclsem, 0, 1);

  ret = sensor_register(&mag_dev->lower, 0);
  if (ret < 0)
    {
      serr("Failed to register ICM20948 magnetometer: %d\n", ret);
      kmm_free(mag_dev);
      return ret;
    }

  sinfo("ICM20948 magnetometer registered successfully\n");

  return OK;
}

#endif /* CONFIG_RA8E1_SPI_GY912_EXAMPLE */
