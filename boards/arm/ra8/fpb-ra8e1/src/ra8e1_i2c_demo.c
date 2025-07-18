
/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_i2c.c
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

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra8e1_fpb.h"

#ifdef CONFIG_RA8_I2C

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C1 Configuration for sensor expansion */

#define I2C1_PORT               1
#define I2C1_FREQUENCY          BOARD_I2C1_FREQUENCY

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_i2c_setup
 *
 * Description:
 *   Setup I2C for sensor expansion
 *
 ****************************************************************************/

static int ra8e1_i2c_setup(void)
{
  /* TODO: Implement I2C setup
   * - Configure I2C clock
   * - Set frequency to 400kHz (fast mode)
   * - Configure pins for I2C function
   * - Enable I2C peripheral
   */

  i2cinfo("Setting up I2C1 for sensor expansion\n");

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_i2c_initialize
 *
 * Description:
 *   Initialize I2C drivers for sensor expansion.
 *
 ****************************************************************************/

void ra8e1_i2c_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  i2cinfo("Initializing I2C1 for sensor expansion\n");

  /* Setup I2C hardware */

  ret = ra8e1_i2c_setup();
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to setup I2C: %d\n", ret);
      return;
    }

  /* Get the I2C1 interface */

  i2c = ra_i2cbus_initialize(I2C1_PORT);
  if (i2c == NULL)
    {
      i2cerr("ERROR: Failed to get I2C1 interface\n");
      return;
    }

  /* Set I2C frequency */

  I2C_SETFREQUENCY(i2c, I2C1_FREQUENCY);

  /* Register the I2C driver */

  ret = i2c_register(i2c, I2C1_PORT);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to register I2C1: %d\n", ret);
      return;
    }

  i2cinfo("I2C1 initialized successfully at %d Hz\n", I2C1_FREQUENCY);
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

  i2cinfo("Scanning I2C bus for devices...\n");

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
          i2cinfo("Found device at address 0x%02X\n", addr);
        }
    }

  i2cinfo("I2C scan complete: found %d devices\n", found);

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
      i2cerr("ERROR: Failed to read from device 0x%02X reg 0x%02X: %d\n",
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
      i2cerr("ERROR: Failed to write to device 0x%02X reg 0x%02X: %d\n",
             addr, reg, ret);
    }

  /* Free buffer */

  kmm_free(buffer);

  return ret;
}

/****************************************************************************
 * Name: ra8e1_optical_flow_init
 *
 * Description:
 *   Initialize optical flow sensor (stub implementation)
 *
 ****************************************************************************/

int ra8e1_optical_flow_init(void)
{
  /* Initialize GY-912 sensor module instead of generic optical flow */
  i2cinfo("Initializing GY-912 sensor module (ICM-20948 + BMP388)\n");

  return ra8e1_gy912_initialize();
}

#endif /* CONFIG_RA8_I2C */
