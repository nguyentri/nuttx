
/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_i2c_acc_demo.c
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

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <math.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "ra8e1_i2c_acc_demo.h"
#include "ra8e1_demo_log.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Define constants for compatibility if not already defined */
#ifndef OK
#  define OK 0
#endif

#ifndef ERROR  
#  define ERROR -1
#endif

#ifndef FAR
#  define FAR
#endif

/* I2C transfer and flag macros compatibility */
#ifndef I2C_TRANSFER
#  define I2C_TRANSFER(dev, msgs, count) i2c_transfer(dev, msgs, count)
#endif

#ifndef I2C_M_READ
#  define I2C_M_READ 0x0001
#endif

/* I2C Configuration */
#define I2C_BUS_NUM             0               /* SCI0 used as I2C */
#define I2C_FREQUENCY           400000          /* 400kHz - Fast mode */
#define I2C_TIMEOUT_MS          1000

/* ADXL345 Accelerometer Configuration (matching FSP example) */
#define ACCEL_I2C_ADDR          0x1D            /* 7-bit slave address */
#define ACCEL_SENSITIVITY       0.0039f         /* g/LSB sensitivity */

/* ADXL345 Register Addresses */
#define ACCEL_REG_DEVID         0x00            /* Device ID register */
#define ACCEL_REG_BW_RATE       0x2C            /* Bandwidth rate */
#define ACCEL_REG_POWER_CTL     0x2D            /* Power control */
#define ACCEL_REG_INT_ENABLE    0x2E            /* Interrupt enable */
#define ACCEL_REG_INT_SOURCE    0x30            /* Interrupt source */
#define ACCEL_REG_DATAX0        0x32            /* X-axis data start */
#define ACCEL_REG_DATAY0        0x34            /* Y-axis data start */
#define ACCEL_REG_DATAZ0        0x36            /* Z-axis data start */

/* ADXL345 Configuration Values */
#define ACCEL_DEVICE_ID         0xE5            /* Expected device ID */
#define ACCEL_BW_RATE_VALUE     0x04            /* 12.5 Hz data rate */
#define ACCEL_POWER_CTL_VALUE   0x08            /* Measurement mode */
#define ACCEL_INT_ENABLE_VALUE  0x80            /* Data ready interrupt */

/* Data processing */
#define ACCEL_DATA_SIZE         6               /* 6 bytes for X,Y,Z */
#define ACCEL_AXIS_COUNT        3               /* X, Y, Z axes */

/* GPIO for interrupt (P414 - IRQ9) */
#define ACCEL_INT_GPIO          (RA_PORT4 | RA_PIN14)
#define ACCEL_INT_IRQ           9

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct accel_data_s
{
  float x_g;      /* X-axis acceleration in g */
  float y_g;      /* Y-axis acceleration in g */
  float z_g;      /* Z-axis acceleration in g */
  int16_t x_raw;  /* X-axis raw data */
  int16_t y_raw;  /* Y-axis raw data */
  int16_t z_raw;  /* Z-axis raw data */
};

struct i2c_accel_dev_s
{
  FAR struct i2c_master_s *i2c;        /* I2C interface */
  bool initialized;                    /* Initialization state */
  volatile bool data_ready;            /* Data ready flag */
  struct accel_data_s last_data;       /* Last sensor reading */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct i2c_accel_dev_s g_accel_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: accel_interrupt_handler
 *
 * Description:
 *   Handle accelerometer data ready interrupt
 *
 ****************************************************************************/

static int accel_interrupt_handler(int irq, void *context, void *arg)
{
  /* Set data ready flag */
  g_accel_dev.data_ready = true;
  
  demoinfo("Accelerometer data ready interrupt\n");
  
  return OK;
}

/****************************************************************************
 * Name: accel_write_reg
 *
 * Description:
 *   Write to accelerometer register
 *
 ****************************************************************************/

static int accel_write_reg(uint8_t reg_addr, uint8_t value)
{
  struct i2c_msg_s msg[1];
  uint8_t buffer[2];
  int ret;

  if (!g_accel_dev.i2c)
    {
      return -ENODEV;
    }

  buffer[0] = reg_addr;
  buffer[1] = value;

  msg[0].frequency = I2C_FREQUENCY;
  msg[0].addr      = ACCEL_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = buffer;
  msg[0].length    = 2;

  ret = I2C_TRANSFER(g_accel_dev.i2c, msg, 1);
  if (ret < 0)
    {
      demoerr("Failed to write register 0x%02x: %d\n", reg_addr, ret);
      return ret;
    }

  demoinfo("Wrote 0x%02x to register 0x%02x\n", value, reg_addr);
  return OK;
}

/****************************************************************************
 * Name: accel_read_reg
 *
 * Description:
 *   Read from accelerometer register
 *
 ****************************************************************************/

static int accel_read_reg(uint8_t reg_addr, uint8_t *value)
{
  struct i2c_msg_s msg[2];
  int ret;

  if (!g_accel_dev.i2c || !value)
    {
      return -EINVAL;
    }

  /* Write register address */
  msg[0].frequency = I2C_FREQUENCY;
  msg[0].addr      = ACCEL_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  /* Read register value */
  msg[1].frequency = I2C_FREQUENCY;
  msg[1].addr      = ACCEL_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = value;
  msg[1].length    = 1;

  ret = I2C_TRANSFER(g_accel_dev.i2c, msg, 2);
  if (ret < 0)
    {
      demoerr("Failed to read register 0x%02x: %d\n", reg_addr, ret);
      return ret;
    }

  demoinfo("Read 0x%02x from register 0x%02x\n", *value, reg_addr);
  return OK;
}

/****************************************************************************
 * Name: accel_read_data
 *
 * Description:
 *   Read acceleration data from accelerometer
 *
 ****************************************************************************/

static int accel_read_data(uint8_t *data, size_t len)
{
  struct i2c_msg_s msg[2];
  uint8_t reg_addr = ACCEL_REG_DATAX0;
  int ret;

  if (!g_accel_dev.i2c || !data || len < ACCEL_DATA_SIZE)
    {
      return -EINVAL;
    }

  /* Write starting register address */
  msg[0].frequency = I2C_FREQUENCY;
  msg[0].addr      = ACCEL_I2C_ADDR;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg_addr;
  msg[0].length    = 1;

  /* Read 6 bytes of acceleration data */
  msg[1].frequency = I2C_FREQUENCY;
  msg[1].addr      = ACCEL_I2C_ADDR;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = data;
  msg[1].length    = ACCEL_DATA_SIZE;

  ret = I2C_TRANSFER(g_accel_dev.i2c, msg, 2);
  if (ret < 0)
    {
      demoerr("Failed to read acceleration data: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: accel_convert_data
 *
 * Description:
 *   Convert raw accelerometer data to g units
 *
 ****************************************************************************/

static int accel_convert_data(const uint8_t *raw_data, struct accel_data_s *accel)
{
  if (!raw_data || !accel)
    {
      return -EINVAL;
    }

  /* Convert 16-bit raw data (little-endian) */
  accel->x_raw = (int16_t)((raw_data[1] << 8) | raw_data[0]);
  accel->y_raw = (int16_t)((raw_data[3] << 8) | raw_data[2]);
  accel->z_raw = (int16_t)((raw_data[5] << 8) | raw_data[4]);

  /* Convert to g units using sensitivity */
  accel->x_g = (float)accel->x_raw * ACCEL_SENSITIVITY;
  accel->y_g = (float)accel->y_raw * ACCEL_SENSITIVITY;
  accel->z_g = (float)accel->z_raw * ACCEL_SENSITIVITY;

  return OK;
}

/****************************************************************************
 * Name: accel_wait_data_ready
 *
 * Description:
 *   Wait for accelerometer data to be ready
 *
 ****************************************************************************/

static int accel_wait_data_ready(uint32_t timeout_ms)
{
  uint32_t elapsed = 0;
  uint8_t int_source;
  int ret;

  /* Reset the data ready flag */
  g_accel_dev.data_ready = false;

  /* Wait for data ready interrupt or timeout */
  while (elapsed < timeout_ms)
    {
      /* Check if interrupt occurred */
      if (g_accel_dev.data_ready)
        {
          /* Clear interrupt by reading INT_SOURCE register */
          ret = accel_read_reg(ACCEL_REG_INT_SOURCE, &int_source);
          if (ret < 0)
            {
              return ret;
            }
          
          if (int_source & 0x80)  /* DATA_READY bit */
            {
              return OK;
            }
        }

      /* Wait 10ms and check again */
      usleep(10000);
      elapsed += 10;
    }

  demoerr("Timeout waiting for accelerometer data ready\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: accel_initialize_hardware
 *
 * Description:
 *   Initialize accelerometer hardware
 *
 ****************************************************************************/

static int accel_initialize_hardware(void)
{
  uint8_t device_id;
  int ret;

  demoinfo("Initializing accelerometer hardware\n");

  /* Read and verify device ID */
  ret = accel_read_reg(ACCEL_REG_DEVID, &device_id);
  if (ret < 0)
    {
      demoerr("Failed to read device ID: %d\n", ret);
      return ret;
    }

  if (device_id != ACCEL_DEVICE_ID)
    {
      demoerr("Invalid device ID: 0x%02x (expected 0x%02x)\n", 
             device_id, ACCEL_DEVICE_ID);
      return -ENODEV;
    }

  demoinfo("ADXL345 accelerometer detected (ID: 0x%02x)\n", device_id);

  /* Configure bandwidth/output data rate */
  ret = accel_write_reg(ACCEL_REG_BW_RATE, ACCEL_BW_RATE_VALUE);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable measurement mode */
  ret = accel_write_reg(ACCEL_REG_POWER_CTL, ACCEL_POWER_CTL_VALUE);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable data ready interrupt */
  ret = accel_write_reg(ACCEL_REG_INT_ENABLE, ACCEL_INT_ENABLE_VALUE);
  if (ret < 0)
    {
      return ret;
    }

  demoinfo("Accelerometer hardware initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: accel_setup_interrupt
 *
 * Description:
 *   Setup accelerometer data ready interrupt
 *
 ****************************************************************************/

static int accel_setup_interrupt(void)
{
  int ret;

  /* Configure GPIO pin for interrupt */
  ret = ra_configgpio(ACCEL_INT_GPIO);
  if (ret < 0)
    {
      demoerr("Failed to configure interrupt GPIO: %d\n", ret);
      return ret;
    }

  /* Attach interrupt handler */
  ret = irq_attach(RA_IRQ_ICU_IRQ0 + ACCEL_INT_IRQ, accel_interrupt_handler, NULL);
  if (ret < 0)
    {
      demoerr("Failed to attach interrupt handler: %d\n", ret);
      return ret;
    }

  /* Configure ICU for falling edge interrupt */
  ret = ra_icu_config(ACCEL_INT_IRQ, RA_ICU_IRQ_EDGE_FALLING, true, 
                      RA_ICU_FILTER_PCLK_DIV_64);
  if (ret < 0)
    {
      demoerr("Failed to configure ICU: %d\n", ret);
      return ret;
    }

  /* Enable the interrupt */
  up_enable_irq(RA_IRQ_ICU_IRQ0 + ACCEL_INT_IRQ);

  demoinfo("Accelerometer interrupt configured on IRQ%d\n", ACCEL_INT_IRQ);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_i2c_accel_initialize
 *
 * Description:
 *   Initialize I2C accelerometer demo
 *
 ****************************************************************************/

int ra8e1_i2c_accel_initialize(void)
{
  int ret;

  demoinfo("Initializing I2C accelerometer demo\n");

  /* Clear device structure */
  memset(&g_accel_dev, 0, sizeof(g_accel_dev));

  /* Get I2C interface */
  g_accel_dev.i2c = ra_i2cbus_initialize(I2C_BUS_NUM);
  if (!g_accel_dev.i2c)
    {
      demoerr("Failed to initialize I2C bus %d\n", I2C_BUS_NUM);
      return -ENODEV;
    }

  /* Set I2C frequency */
  I2C_SETFREQUENCY(g_accel_dev.i2c, I2C_FREQUENCY);

  demoinfo("I2C bus initialized at %d Hz\n", I2C_FREQUENCY);

  /* Initialize accelerometer hardware */
  ret = accel_initialize_hardware();
  if (ret < 0)
    {
      demoerr("Failed to initialize accelerometer hardware: %d\n", ret);
      return ret;
    }

  /* Setup interrupt */
  ret = accel_setup_interrupt();
  if (ret < 0)
    {
      demoerr("Failed to setup accelerometer interrupt: %d\n", ret);
      return ret;
    }

  g_accel_dev.initialized = true;
  demoinfo("I2C accelerometer demo initialized successfully\n");

  return OK;
}

/****************************************************************************
 * Name: ra8e1_i2c_accel_read
 *
 * Description:
 *   Read accelerometer data
 *
 ****************************************************************************/

int ra8e1_i2c_accel_read(struct accel_data_s *data)
{
  uint8_t raw_data[ACCEL_DATA_SIZE];
  int ret;

  if (!g_accel_dev.initialized || !data)
    {
      return -EINVAL;
    }

  /* Wait for data to be ready */
  ret = accel_wait_data_ready(I2C_TIMEOUT_MS);
  if (ret < 0)
    {
      return ret;
    }

  /* Read raw acceleration data */
  ret = accel_read_data(raw_data, ACCEL_DATA_SIZE);
  if (ret < 0)
    {
      return ret;
    }

  /* Convert to engineering units */
  ret = accel_convert_data(raw_data, data);
  if (ret < 0)
    {
      return ret;
    }

  /* Store last reading */
  memcpy(&g_accel_dev.last_data, data, sizeof(struct accel_data_s));

  demoinfo("Accel data: X=%.3fg, Y=%.3fg, Z=%.3fg\n", 
          data->x_g, data->y_g, data->z_g);

  return OK;
}

/****************************************************************************
 * Name: ra8e1_i2c_acc_demo_init
 *
 * Description:
 *   Initialize the I2C accelerometer demo
 *
 ****************************************************************************/

int ra8e1_i2c_acc_demo_init(void)
{
  /* I2C accelerometer demo initialization is done within main function */
  return 0;
}

/****************************************************************************
 * Name: ra8e1_i2c_acc_demo_main
 *
 * Description:
 *   Main entry point for I2C accelerometer demo
 *
 ****************************************************************************/

int ra8e1_i2c_acc_demo_main(int argc, char *argv[])
{
  struct accel_data_s accel_data;
  int ret;
  int sample_count = 0;

  demoprintf("RA8E1 I2C Accelerometer Demo\n");
  demoprintf("=============================\n");
  demoprintf("Reading ADXL345 accelerometer via I2C (SCI0)\n");
  demoprintf("I2C Address: 0x%02X\n", ACCEL_I2C_ADDR);
  demoprintf("Sample Rate: 12.5 Hz\n");
  demoprintf("Press Ctrl+C to stop...\n\n");

  /* Initialize if not already done */
  if (!g_accel_dev.initialized)
    {
      ret = ra8e1_i2c_accel_initialize();
      if (ret < 0)
        {
          demoprintf("Failed to initialize accelerometer: %d\n", ret);
          return ret;
        }
    }

  /* Continuous reading loop */
  while (sample_count < 100)  /* Read 100 samples for demo */
    {
      ret = ra8e1_i2c_accel_read(&accel_data);
      if (ret < 0)
        {
          demoprintf("Failed to read accelerometer data: %d\n", ret);
          continue;
        }

      /* Display acceleration data */
      demoprintf("Sample %3d: X=%6.3fg  Y=%6.3fg  Z=%6.3fg  ", 
             ++sample_count, accel_data.x_g, accel_data.y_g, accel_data.z_g);
      
      /* Calculate magnitude */
      float magnitude = sqrtf(accel_data.x_g * accel_data.x_g + 
                             accel_data.y_g * accel_data.y_g + 
                             accel_data.z_g * accel_data.z_g);
      demoprintf("Mag=%.3fg\n", magnitude);

      /* Wait for next sample (approximately 80ms for 12.5Hz) */
      usleep(80000);
    }

  demoprintf("\nI2C Accelerometer demo completed successfully!\n");
  return OK;
}

#endif /* CONFIG_RA8_I2C */
    {
      demoerr("ERROR: Failed to setup I2C: %d\n", ret);
      return;
    }

  /* Get the I2C1 interface */

  i2c = ra_i2cbus_initialize(I2C1_PORT);
  if (i2c == NULL)
    {
      demoerr("ERROR: Failed to get I2C1 interface\n");
      return;
    }

  /* Set I2C frequency */

  I2C_SETFREQUENCY(i2c, I2C1_FREQUENCY);

  /* Register the I2C driver */

  ret = i2c_register(i2c, I2C1_PORT);
  if (ret < 0)
    {
      demoerr("ERROR: Failed to register I2C1: %d\n", ret);
      return;
    }

  demoinfo("I2C1 initialized successfully at %d Hz\n", I2C1_FREQUENCY);
}

/****************************************************************************
 * Name: ra8e1_i2c_scan
 *
 * Description:
 *   Scan I2C bus for connected devices
 *
 ****************************************************************************/

int ra8e1_i2c_scan(uint8_t *devices, int max_devices)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msg;
  uint8_t buffer;
  int found = 0;
  int addr;
  int ret;

  if (devices == NULL || max_devices <= 0)
    {
      return -EINVAL;
    }

  /* Get I2C interface */

  i2c = ra_i2cbus_initialize(I2C1_PORT);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  demoinfo("Scanning I2C bus for devices...\n");

  /* Scan addresses 0x08 to 0x77 (standard 7-bit range) */

  for (addr = 0x08; addr <= 0x77 && found < max_devices; addr++)
    {
      /* Setup message for device detection */

      msg.frequency = I2C1_FREQUENCY;
      msg.addr      = addr;
      msg.flags     = 0;
      msg.buffer    = &buffer;
      msg.length    = 0;  /* Zero-length transfer for detection */

      /* Try to communicate with device */

      ret = I2C_TRANSFER(i2c, &msg, 1);
      if (ret == OK)
        {
          devices[found] = addr;
          found++;
          demoinfo("Found device at address 0x%02X\n", addr);
        }
    }

  demoinfo("I2C scan complete: found %d devices\n", found);

  return found;
}

/****************************************************************************
 * Name: ra8e1_i2c_read_reg
 *
 * Description:
 *   Read register from I2C device
 *
 ****************************************************************************/

int ra8e1_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msg[2];
  int ret;

  if (data == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Get I2C interface */

  i2c = ra_i2cbus_initialize(I2C1_PORT);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Setup messages for register read */

  /* First message: write register address */

  msg[0].frequency = I2C1_FREQUENCY;
  msg[0].addr      = addr;
  msg[0].flags     = 0;
  msg[0].buffer    = &reg;
  msg[0].length    = 1;

  /* Second message: read data */

  msg[1].frequency = I2C1_FREQUENCY;
  msg[1].addr      = addr;
  msg[1].flags     = I2C_M_READ;
  msg[1].buffer    = data;
  msg[1].length    = len;

  /* Perform transfer */

  ret = I2C_TRANSFER(i2c, msg, 2);
  if (ret < 0)
    {
      demoerr("ERROR: Failed to read from device 0x%02X reg 0x%02X: %d\n",
             addr, reg, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ra8e1_i2c_write_reg
 *
 * Description:
 *   Write register to I2C device
 *
 ****************************************************************************/

int ra8e1_i2c_write_reg(uint8_t addr, uint8_t reg, const uint8_t *data,
                        size_t len)
{
  struct i2c_master_s *i2c;
  struct i2c_msg_s msg;
  uint8_t *buffer;
  int ret;

  if (data == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Get I2C interface */

  i2c = ra_i2cbus_initialize(I2C1_PORT);
  if (i2c == NULL)
    {
      return -ENODEV;
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

  ret = I2C_TRANSFER(i2c, &msg, 1);
  if (ret < 0)
    {
      demoerr("ERROR: Failed to write to device 0x%02X reg 0x%02X: %d\n",
             addr, reg, ret);
    }

  /* Free buffer */

  kmm_free(buffer);

  return ret;
}
