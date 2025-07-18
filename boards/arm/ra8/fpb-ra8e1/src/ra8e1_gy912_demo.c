/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_gy912_complete_demo.c
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
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "fpb-ra8e1.h"
#include "spi_gy912.h"

#ifdef CONFIG_RA8_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTT Logging via Segger RTT */
#ifdef CONFIG_SEGGER_RTT
#  include <SEGGER_RTT.h>
#  define gy912_info(fmt, ...)  SEGGER_RTT_printf(0, "INFO: " fmt "\n", ##__VA_ARGS__)
#  define gy912_warn(fmt, ...)  SEGGER_RTT_printf(0, "WARN: " fmt "\n", ##__VA_ARGS__)
#  define gy912_err(fmt, ...)   SEGGER_RTT_printf(0, "ERROR: " fmt "\n", ##__VA_ARGS__)
#else
#  define gy912_info(fmt, ...)  _info(fmt "\n", ##__VA_ARGS__)
#  define gy912_warn(fmt, ...)  _warn(fmt "\n", ##__VA_ARGS__)
#  define gy912_err(fmt, ...)   _err(fmt "\n", ##__VA_ARGS__)
#endif

/* Demo configuration */
#define GY912_DEMO_INTERVAL_MS    1000    /* 1 second reading interval */
#define GY912_DEMO_MAX_READINGS   100     /* Maximum readings to take */
#define GY912_DMA_BUFFER_SIZE     64      /* DMA buffer size for bulk transfers */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Demo state structure */
struct gy912_demo_state_s
{
  struct spi_dev_s *spi;
  bool initialized;
  uint32_t reading_count;
  uint32_t error_count;
  
  /* DMA buffers */
  uint8_t dma_tx_buffer[GY912_DMA_BUFFER_SIZE];
  uint8_t dma_rx_buffer[GY912_DMA_BUFFER_SIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct gy912_demo_state_s g_gy912_demo;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_spi_read_reg
 *
 * Description:
 *   Read a register from a sensor via SPI
 *
 * Input Parameters:
 *   devid - Device ID (SPIDEV_ICM20948 or SPIDEV_BMP388)
 *   reg   - Register address
 *
 * Returned Value:
 *   Register value or 0xFF on error
 *
 ****************************************************************************/

static uint8_t gy912_spi_read_reg(uint32_t devid, uint8_t reg)
{
  struct spi_dev_s *spi = g_gy912_demo.spi;
  uint8_t result = 0xFF;
  
  if (spi == NULL)
    {
      gy912_err("SPI not initialized");
      return 0xFF;
    }
  
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, GY912_SPI_FREQUENCY);
  SPI_SETMODE(spi, GY912_SPI_MODE);
  SPI_SETBITS(spi, GY912_SPI_BITS);
  
  SPI_SELECT(spi, devid, true);
  SPI_SEND(spi, reg | 0x80);  /* Read bit set */
  result = SPI_SEND(spi, 0xFF);
  SPI_SELECT(spi, devid, false);
  
  SPI_LOCK(spi, false);
  
  return result;
}

/****************************************************************************
 * Name: gy912_spi_write_reg
 *
 * Description:
 *   Write a register to a sensor via SPI
 *
 * Input Parameters:
 *   devid - Device ID (SPIDEV_ICM20948 or SPIDEV_BMP388)
 *   reg   - Register address
 *   val   - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_spi_write_reg(uint32_t devid, uint8_t reg, uint8_t val)
{
  struct spi_dev_s *spi = g_gy912_demo.spi;
  
  if (spi == NULL)
    {
      gy912_err("SPI not initialized");
      return;
    }
  
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, GY912_SPI_FREQUENCY);
  SPI_SETMODE(spi, GY912_SPI_MODE);
  SPI_SETBITS(spi, GY912_SPI_BITS);
  
  SPI_SELECT(spi, devid, true);
  SPI_SEND(spi, reg & 0x7F);  /* Write bit clear */
  SPI_SEND(spi, val);
  SPI_SELECT(spi, devid, false);
  
  SPI_LOCK(spi, false);
}

/****************************************************************************
 * Name: gy912_spi_read_burst
 *
 * Description:
 *   Read multiple registers from a sensor via SPI with DMA
 *
 * Input Parameters:
 *   devid   - Device ID (SPIDEV_ICM20948 or SPIDEV_BMP388)
 *   reg     - Starting register address
 *   buffer  - Buffer to store read data
 *   len     - Number of bytes to read
 *
 * Returned Value:
 *   Number of bytes read or negative error code
 *
 ****************************************************************************/

static int gy912_spi_read_burst(uint32_t devid, uint8_t reg, uint8_t *buffer, size_t len)
{
  struct spi_dev_s *spi = g_gy912_demo.spi;
  uint8_t tx_buffer[GY912_DMA_BUFFER_SIZE];
  uint8_t rx_buffer[GY912_DMA_BUFFER_SIZE];
  
  if (spi == NULL || buffer == NULL || len == 0 || len > (GY912_DMA_BUFFER_SIZE - 1))
    {
      gy912_err("Invalid parameters for burst read");
      return -EINVAL;
    }
  
  /* Setup TX buffer */
  tx_buffer[0] = reg | 0x80;  /* Read bit set */
  memset(&tx_buffer[1], 0xFF, len);  /* Dummy bytes for reading */
  
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, GY912_SPI_FREQUENCY);
  SPI_SETMODE(spi, GY912_SPI_MODE);
  SPI_SETBITS(spi, GY912_SPI_BITS);
  
  SPI_SELECT(spi, devid, true);
  
  /* Use SPI exchange for efficient DMA transfer */
  SPI_EXCHANGE(spi, tx_buffer, rx_buffer, len + 1);
  
  SPI_SELECT(spi, devid, false);
  SPI_LOCK(spi, false);
  
  /* Copy received data (skip first byte which is dummy) */
  memcpy(buffer, &rx_buffer[1], len);
  
  return len;
}

/****************************************************************************
 * Name: icm20948_initialize
 *
 * Description:
 *   Initialize ICM-20948 sensor
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int icm20948_initialize(void)
{
  uint8_t chip_id;
  
  gy912_info("Initializing ICM-20948...");
  
  /* Read chip ID */
  chip_id = gy912_spi_read_reg(SPIDEV_ICM20948, ICM20948_WHO_AM_I);
  gy912_info("ICM-20948 chip ID: 0x%02x (expected: 0x%02x)", chip_id, ICM20948_WHO_AM_I_VAL);
  
  if (chip_id != ICM20948_WHO_AM_I_VAL)
    {
      gy912_err("ICM-20948 chip ID mismatch");
      return -ENODEV;
    }
  
  /* Reset device */
  gy912_spi_write_reg(SPIDEV_ICM20948, ICM20948_PWR_MGMT_1, 0x80);
  usleep(100000);  /* Wait 100ms for reset */
  
  /* Wake up device */
  gy912_spi_write_reg(SPIDEV_ICM20948, ICM20948_PWR_MGMT_1, 0x01);
  usleep(10000);   /* Wait 10ms */
  
  /* Enable accelerometer and gyroscope */
  gy912_spi_write_reg(SPIDEV_ICM20948, ICM20948_PWR_MGMT_2, 0x00);
  usleep(10000);   /* Wait 10ms */
  
  gy912_info("ICM-20948 initialized successfully");
  return OK;
}

/****************************************************************************
 * Name: bmp388_initialize
 *
 * Description:
 *   Initialize BMP388 sensor
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int bmp388_initialize(void)
{
  uint8_t chip_id;
  
  gy912_info("Initializing BMP388...");
  
  /* Read chip ID */
  chip_id = gy912_spi_read_reg(SPIDEV_BMP388, BMP388_CHIP_ID);
  gy912_info("BMP388 chip ID: 0x%02x (expected: 0x%02x)", chip_id, BMP388_CHIP_ID_VAL);
  
  if (chip_id != BMP388_CHIP_ID_VAL)
    {
      gy912_err("BMP388 chip ID mismatch");
      return -ENODEV;
    }
  
  /* Configure oversampling and data rate */
  gy912_spi_write_reg(SPIDEV_BMP388, BMP388_OSR, 0x05);      /* OSR x4 for pressure and temperature */
  gy912_spi_write_reg(SPIDEV_BMP388, BMP388_ODR, 0x13);      /* 50Hz data rate */
  gy912_spi_write_reg(SPIDEV_BMP388, BMP388_CONFIG, 0x02);   /* IIR filter coeff 3 */
  
  /* Enable pressure and temperature measurement */
  gy912_spi_write_reg(SPIDEV_BMP388, BMP388_PWR_CTRL, 0x33);
  usleep(10000);   /* Wait 10ms */
  
  gy912_info("BMP388 initialized successfully");
  return OK;
}

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

static int icm20948_read_data(struct icm20948_data_s *data)
{
  uint8_t raw_data[14];
  int ret;
  
  if (data == NULL)
    {
      return -EINVAL;
    }
  
  /* Read all sensor data in one burst (14 bytes) */
  ret = gy912_spi_read_burst(SPIDEV_ICM20948, ICM20948_ACCEL_XOUT_H, raw_data, 14);
  if (ret < 0)
    {
      gy912_err("Failed to read ICM-20948 data");
      return ret;
    }
  
  /* Convert raw data to sensor values */
  data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]);
  data->temp    = (int16_t)((raw_data[6] << 8) | raw_data[7]);
  data->gyro_x  = (int16_t)((raw_data[8] << 8) | raw_data[9]);
  data->gyro_y  = (int16_t)((raw_data[10] << 8) | raw_data[11]);
  data->gyro_z  = (int16_t)((raw_data[12] << 8) | raw_data[13]);
  
  return OK;
}

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

static int bmp388_read_data(struct bmp388_data_s *data)
{
  uint8_t raw_data[6];
  int ret;
  
  if (data == NULL)
    {
      return -EINVAL;
    }
  
  /* Read pressure and temperature data (6 bytes) */
  ret = gy912_spi_read_burst(SPIDEV_BMP388, BMP388_PRESS_MSB, raw_data, 6);
  if (ret < 0)
    {
      gy912_err("Failed to read BMP388 data");
      return ret;
    }
  
  /* Convert raw data to sensor values */
  data->pressure = ((uint32_t)raw_data[0] << 16) | 
                   ((uint32_t)raw_data[1] << 8) | 
                   ((uint32_t)raw_data[2]);
  
  data->temperature = ((int32_t)raw_data[3] << 16) | 
                      ((int32_t)raw_data[4] << 8) | 
                      ((int32_t)raw_data[5]);
  
  return OK;
}

/****************************************************************************
 * Name: gy912_print_data
 *
 * Description:
 *   Print sensor data in a formatted way
 *
 * Input Parameters:
 *   data - Pointer to combined sensor data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_print_data(const struct gy912_data_s *data)
{
  gy912_info("=== GY-912 Sensor Reading #%lu ===", g_gy912_demo.reading_count);
  gy912_info("ICM-20948 IMU:");
  gy912_info("  Accel: X=%6d Y=%6d Z=%6d", 
             data->imu.accel_x, data->imu.accel_y, data->imu.accel_z);
  gy912_info("  Gyro:  X=%6d Y=%6d Z=%6d", 
             data->imu.gyro_x, data->imu.gyro_y, data->imu.gyro_z);
  gy912_info("  Temp:  %d", data->imu.temp);
  
  gy912_info("BMP388 Barometer:");
  gy912_info("  Pressure:    %lu Pa", data->baro.pressure);
  gy912_info("  Temperature: %ld (0.01°C)", data->baro.temperature);
  
  gy912_info("Timestamp: %lu ms", data->timestamp);
  gy912_info("Errors: %lu", g_gy912_demo.error_count);
  gy912_info("");
}

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

static int gy912_self_test(void)
{
  uint8_t icm_id, bmp_id;
  struct icm20948_data_s imu_data;
  struct bmp388_data_s baro_data;
  int ret;
  
  gy912_info("Starting GY-912 self-test...");
  
  /* Test communication with both sensors */
  icm_id = gy912_spi_read_reg(SPIDEV_ICM20948, ICM20948_WHO_AM_I);
  bmp_id = gy912_spi_read_reg(SPIDEV_BMP388, BMP388_CHIP_ID);
  
  gy912_info("ICM-20948 ID: 0x%02x %s", icm_id, 
             (icm_id == ICM20948_WHO_AM_I_VAL) ? "PASS" : "FAIL");
  gy912_info("BMP388 ID: 0x%02x %s", bmp_id,
             (bmp_id == BMP388_CHIP_ID_VAL) ? "PASS" : "FAIL");
  
  if (icm_id != ICM20948_WHO_AM_I_VAL || bmp_id != BMP388_CHIP_ID_VAL)
    {
      gy912_err("Self-test failed - sensor communication error");
      return -ENODEV;
    }
  
  /* Test data reading */
  ret = icm20948_read_data(&imu_data);
  if (ret < 0)
    {
      gy912_err("Self-test failed - ICM-20948 data read error");
      return ret;
    }
  
  ret = bmp388_read_data(&baro_data);
  if (ret < 0)
    {
      gy912_err("Self-test failed - BMP388 data read error");
      return ret;
    }
  
  gy912_info("Data read test:");
  gy912_info("  ICM-20948: Accel(%d,%d,%d) Gyro(%d,%d,%d) Temp(%d)",
             imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
             imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z, imu_data.temp);
  gy912_info("  BMP388: Pressure(%lu) Temperature(%ld)",
             baro_data.pressure, baro_data.temperature);
  
  gy912_info("Self-test PASSED");
  return OK;
}

/****************************************************************************
 * Public Functions
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

int gy912_initialize(void)
{
  int ret;
  
  if (g_gy912_demo.initialized)
    {
      gy912_info("GY-912 already initialized");
      return OK;
    }
  
  gy912_info("Initializing GY-912 10DOF sensor module...");
  
  /* Initialize SPI */
  g_gy912_demo.spi = ra_spibus_initialize(GY912_SPI_BUS);
  if (g_gy912_demo.spi == NULL)
    {
      gy912_err("Failed to initialize SPI bus %d", GY912_SPI_BUS);
      return -ENODEV;
    }
  
  /* Initialize sensors */
  ret = icm20948_initialize();
  if (ret < 0)
    {
      gy912_err("Failed to initialize ICM-20948");
      return ret;
    }
  
  ret = bmp388_initialize();
  if (ret < 0)
    {
      gy912_err("Failed to initialize BMP388");
      return ret;
    }
  
  /* Perform self-test */
  ret = gy912_self_test();
  if (ret < 0)
    {
      gy912_err("Self-test failed");
      return ret;
    }
  
  /* Reset counters */
  g_gy912_demo.reading_count = 0;
  g_gy912_demo.error_count = 0;
  g_gy912_demo.initialized = true;
  
  gy912_info("GY-912 initialization completed successfully");
  return OK;
}

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

int gy912_read_all(struct gy912_data_s *data)
{
  int ret;
  
  if (!g_gy912_demo.initialized || data == NULL)
    {
      return -EINVAL;
    }
  
  /* Read IMU data */
  ret = icm20948_read_data(&data->imu);
  if (ret < 0)
    {
      g_gy912_demo.error_count++;
      return ret;
    }
  
  /* Read barometer data */
  ret = bmp388_read_data(&data->baro);
  if (ret < 0)
    {
      g_gy912_demo.error_count++;
      return ret;
    }
  
  /* Add timestamp */
  data->timestamp = clock_systime_ticks() * MSEC_PER_TICK;
  
  return OK;
}

/****************************************************************************
 * Name: gy912_demo_run
 *
 * Description:
 *   Run the GY-912 demonstration
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy912_demo_run(void)
{
  struct gy912_data_s sensor_data;
  int ret;
  
  gy912_info("Starting GY-912 demonstration...");
  gy912_info("Will take %d readings at %d ms intervals", 
             GY912_DEMO_MAX_READINGS, GY912_DEMO_INTERVAL_MS);
  
  /* Initialize if not already done */
  if (!g_gy912_demo.initialized)
    {
      ret = gy912_initialize();
      if (ret < 0)
        {
          gy912_err("Initialization failed");
          return ret;
        }
    }
  
  /* Main demonstration loop */
  while (g_gy912_demo.reading_count < GY912_DEMO_MAX_READINGS)
    {
      /* Read all sensor data */
      ret = gy912_read_all(&sensor_data);
      if (ret == OK)
        {
          g_gy912_demo.reading_count++;
          gy912_print_data(&sensor_data);
        }
      else
        {
          gy912_err("Failed to read sensor data: %d", ret);
        }
      
      /* Wait for next reading */
      usleep(GY912_DEMO_INTERVAL_MS * 1000);
    }
  
  gy912_info("Demonstration completed:");
  gy912_info("  Total readings: %lu", g_gy912_demo.reading_count);
  gy912_info("  Total errors:   %lu", g_gy912_demo.error_count);
  gy912_info("  Success rate:   %.1f%%", 
             100.0 * (g_gy912_demo.reading_count - g_gy912_demo.error_count) / 
             g_gy912_demo.reading_count);
  
  return OK;
}

/****************************************************************************
 * Name: gy912_demo_single_read
 *
 * Description:
 *   Perform a single sensor reading (for testing)
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy912_demo_single_read(void)
{
  struct gy912_data_s sensor_data;
  int ret;
  
  /* Initialize if not already done */
  if (!g_gy912_demo.initialized)
    {
      ret = gy912_initialize();
      if (ret < 0)
        {
          gy912_err("Initialization failed");
          return ret;
        }
    }
  
  /* Read sensor data */
  ret = gy912_read_all(&sensor_data);
  if (ret == OK)
    {
      g_gy912_demo.reading_count++;
      gy912_print_data(&sensor_data);
    }
  else
    {
      gy912_err("Failed to read sensor data: %d", ret);
    }
  
  return ret;
}

/****************************************************************************
 * Name: icm20948_read_reg
 *
 * Description:
 *   Read a register from ICM-20948 (Public API)
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint8_t icm20948_read_reg(uint8_t reg)
{
  return gy912_spi_read_reg(SPIDEV_ICM20948, reg);
}

/****************************************************************************
 * Name: icm20948_write_reg
 *
 * Description:
 *   Write a register to ICM-20948 (Public API)
 *
 * Input Parameters:
 *   reg - Register address
 *   val - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icm20948_write_reg(uint8_t reg, uint8_t val)
{
  gy912_spi_write_reg(SPIDEV_ICM20948, reg, val);
}

/****************************************************************************
 * Name: bmp388_read_reg
 *
 * Description:
 *   Read a register from BMP388 (Public API)
 *
 * Input Parameters:
 *   reg - Register address
 *
 * Returned Value:
 *   Register value
 *
 ****************************************************************************/

uint8_t bmp388_read_reg(uint8_t reg)
{
  return gy912_spi_read_reg(SPIDEV_BMP388, reg);
}

/****************************************************************************
 * Name: bmp388_write_reg
 *
 * Description:
 *   Write a register to BMP388 (Public API)
 *
 * Input Parameters:
 *   reg - Register address
 *   val - Value to write
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void bmp388_write_reg(uint8_t reg, uint8_t val)
{
  gy912_spi_write_reg(SPIDEV_BMP388, reg, val);
}

#endif /* CONFIG_RA8_SPI */
