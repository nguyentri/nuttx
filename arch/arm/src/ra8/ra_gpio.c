/****************************************************************************
 * arch/arm/src/ra8/ra_gpio.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "hardware/ra_gpio.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* PFS protection counter for safe register access */
static volatile uint32_t g_pfs_protect_counter = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpio_pfs_enable
 *
 * Description:
 *   Enable access to PFS registers with protection counter
 *
 ****************************************************************************/

static void ra_gpio_pfs_enable(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* If this is the first entry, enable PFS writing */
  if (g_pfs_protect_counter == 0)
    {
      /* Clear BOWI bit - enable writing to PFSWE bit */
      putreg8(0, R_PMISC_PWPR);

      /* Set PFSWE bit - enable writing to PFS registers */
      putreg8(R_PMISC_PWPR_PFSWE, R_PMISC_PWPR);
    }

  /* Increment protect counter */
  g_pfs_protect_counter++;

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra_gpio_pfs_disable
 *
 * Description:
 *   Disable access to PFS registers with protection counter
 *
 ****************************************************************************/

static void ra_gpio_pfs_disable(void)
{
  irqstate_t flags;

  flags = enter_critical_section();

  /* Is it safe to disable PFS register access? */
  if (g_pfs_protect_counter > 0)
    {
      /* Decrement the protect counter */
      g_pfs_protect_counter--;
    }

  /* If counter reaches zero, disable PFS writing */
  if (g_pfs_protect_counter == 0)
    {
      /* Clear PFSWE bit - disable writing to PFS registers */
      putreg8(0, R_PMISC_PWPR);

      /* Set BOWI bit - disable writing to PFSWE bit */
      putreg8(R_PMISC_PWPR_B0WI, R_PMISC_PWPR);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra_gpio_pfs_write
 *
 * Description:
 *   Write to PFS register for a specific pin
 *
 * Input Parameters:
 *   port  - Port number (0-14)
 *   pin   - Pin number (0-15)
 *   value - PFS register value to write
 *
 ****************************************************************************/

static void ra_gpio_pfs_write(uint8_t port, uint8_t pin, uint32_t value)
{
  uint32_t pfs_addr;

  /* Calculate PFS register address */
  pfs_addr = R_PFS_BASE + (port * R_PFS_PSEL_PORT_OFFSET) +
             (pin * R_PFS_PSEL_PIN_OFFSET);

  /* For peripheral functions, clear PMR first */
  if ((value & R_PFS_PMR) != 0)
    {
      /* Clear PMR bit first, keeping other settings */
      putreg32(value & ~R_PFS_PMR, pfs_addr);
    }

  /* Write the complete configuration */
  putreg32(value, pfs_addr);
}

/****************************************************************************
 * Name: ra_gpio_get_pfs_config
 *
 * Description:
 *   Convert NuttX GPIO configuration to PFS register value
 *
 * Input Parameters:
 *   cfgset - Encoded GPIO configuration
 *
 * Returned Value:
 *   PFS register value
 *
 ****************************************************************************/

static uint32_t ra_gpio_get_pfs_config(gpio_pinset_t cfgset)
{
  /* Simply return the cfg field from the structure */
  return cfgset.cfg;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_gpio_config
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin configuration.
 *
 * Input Parameters:
 *   cfgset - GPIO configuration encoding
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_gpio_config(gpio_pinset_t cfgset)
{
  uint8_t port;
  uint8_t pin;
  uint32_t pfs_value;

  /* Extract port and pin from struct */
  port = cfgset.port;
  pin = cfgset.pin;

  /* Validate port and pin numbers */
  if (port > 14 || pin > 15)
    {
      return -EINVAL;
    }

  /* Convert to PFS configuration */
  pfs_value = ra_gpio_get_pfs_config(cfgset);

  /* Configure the pin with PFS protection */
  ra_gpio_pfs_enable();
  ra_gpio_pfs_write(port, pin, pfs_value);
  ra_gpio_pfs_disable();

  return OK;
}

/****************************************************************************
 * Name: ra_gpio_write
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Input Parameters:
 *   pinset - GPIO pin configuration (port/pin encoded)
 *   value  - Output value (true=high, false=low)
 *
 ****************************************************************************/

void ra_gpio_write(gpio_pinset_t pinset, bool value)
{
  uint8_t port;
  uint8_t pin;

  /* Extract port and pin from struct */
  port = pinset.port;
  pin = pinset.pin;

  /* Validate port and pin numbers */
  if (port > 14 || pin > 15)
    {
      return;
    }

  /* Use atomic set/reset registers for thread-safe operation */
  if (value)
    {
      putreg16((1U << pin), R_PORT_POSR(port));
    }
  else
    {
      putreg16((1U << pin), R_PORT_PORR(port));
    }
}

/****************************************************************************
 * Name: ra_gpio_read
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Input Parameters:
 *   pinset - GPIO pin configuration (port/pin encoded)
 *
 * Returned Value:
 *   The boolean state of the input pin
 *
 ****************************************************************************/

bool ra_gpio_read(gpio_pinset_t pinset)
{
  uint8_t port;
  uint8_t pin;
  uint16_t port_data;

  /* Extract port and pin from struct */
  port = pinset.port;
  pin = pinset.pin;

  /* Validate port and pin numbers */
  if (port > 14 || pin > 15)
    {
      return false;
    }

  /* Read from PIDR register */
  port_data = getreg16(R_PORT_PIDR(port));
  return ((port_data & (1U << pin)) != 0);
}

/****************************************************************************
 * Name: ra_gpio_set_direction
 *
 * Description:
 *   Set GPIO pin direction (input/output)
 *
 * Input Parameters:
 *   pinset    - GPIO pin configuration (port/pin encoded)
 *   direction - Direction (true=output, false=input)
 *
 ****************************************************************************/

void ra_gpio_set_direction(gpio_pinset_t pinset, bool direction)
{
  uint8_t port;
  uint8_t pin;
  uint32_t pfs_addr;
  uint32_t pfs_value;

  /* Extract port and pin from struct */
  port = pinset.port;
  pin = pinset.pin;

  /* Validate port and pin numbers */
  if (port > 14 || pin > 15)
    {
      return;
    }

  /* Calculate PFS register address */
  pfs_addr = R_PFS_BASE + (port * R_PFS_PSEL_PORT_OFFSET) +
             (pin * R_PFS_PSEL_PIN_OFFSET);

  /* Modify direction bit with PFS protection */
  ra_gpio_pfs_enable();

  pfs_value = getreg32(pfs_addr);
  if (direction)
    {
      pfs_value |= RA_PFS_PDR;  /* Output */
    }
  else
    {
      pfs_value &= ~RA_PFS_PDR; /* Input */
    }
  putreg32(pfs_value, pfs_addr);

  ra_gpio_pfs_disable();
}

/****************************************************************************
 * Name: ra_gpio_set_pullup
 *
 * Description:
 *   Enable/disable pull-up resistor on GPIO pin
 *
 * Input Parameters:
 *   pinset - GPIO pin configuration (port/pin encoded)
 *   enable - Enable pull-up (true=enable, false=disable)
 *
 ****************************************************************************/

void ra_gpio_set_pullup(gpio_pinset_t pinset, bool enable)
{
  uint8_t port;
  uint8_t pin;
  uint32_t pfs_addr;
  uint32_t pfs_value;

  /* Extract port and pin from struct */
  port = pinset.port;
  pin = pinset.pin;

  /* Validate port and pin numbers */
  if (port > 14 || pin > 15)
    {
      return;
    }

  /* Calculate PFS register address */
  pfs_addr = R_PFS_BASE + (port * R_PFS_PSEL_PORT_OFFSET) +
             (pin * R_PFS_PSEL_PIN_OFFSET);

  /* Modify pull-up bit with PFS protection */
  ra_gpio_pfs_enable();

  pfs_value = getreg32(pfs_addr);
  if (enable)
    {
      pfs_value |= RA_PFS_PCR;  /* Enable pull-up */
    }
  else
    {
      pfs_value &= ~RA_PFS_PCR; /* Disable pull-up */
    }
  putreg32(pfs_value, pfs_addr);

  ra_gpio_pfs_disable();
}

/****************************************************************************
 * Name: ra_gpio_set_drive_strength
 *
 * Description:
 *   Set GPIO pin drive strength
 *
 * Input Parameters:
 *   pinset   - GPIO pin configuration (port/pin encoded)
 *   strength - Drive strength (0=low, 1=mid, 2=high)
 *
 ****************************************************************************/

void ra_gpio_set_drive_strength(gpio_pinset_t pinset, uint8_t strength)
{
  uint8_t port;
  uint8_t pin;
  uint32_t pfs_addr;
  uint32_t pfs_value;

  /* Extract port and pin from struct */
  port = pinset.port;
  pin = pinset.pin;

  /* Validate port and pin numbers and strength level */
  if (port > 14 || pin > 15 || strength > 2)
    {
      return;
    }

  /* Calculate PFS register address */
  pfs_addr = R_PFS_BASE + (port * R_PFS_PSEL_PORT_OFFSET) +
             (pin * R_PFS_PSEL_PIN_OFFSET);

  /* Modify drive strength bits with PFS protection */
  ra_gpio_pfs_enable();

  pfs_value = getreg32(pfs_addr);

  /* Clear existing drive strength bits */
  pfs_value &= ~(RA_PFS_DSCR | RA_PFS_DSCR1);

  /* Set new drive strength */
  switch (strength)
    {
      case 1: /* Mid */
        pfs_value |= RA_PFS_DSCR;
        break;

      case 2: /* High */
        pfs_value |= RA_PFS_DSCR | RA_PFS_DSCR1;
        break;

      default: /* Low */
        break;
    }

  putreg32(pfs_value, pfs_addr);

  ra_gpio_pfs_disable();
}

/****************************************************************************
 * Legacy wrapper functions for compatibility
 ****************************************************************************/

/****************************************************************************
 * Name: ra_configgpio
 *
 * Description:
 *   Legacy wrapper for ra_gpio_config
 *
 ****************************************************************************/

void ra_configgpio(gpio_pinset_t cfgset)
{
  (void)ra_gpio_config(cfgset);
}

/****************************************************************************
 * Name: ra_gpiowrite
 *
 * Description:
 *   Legacy wrapper for ra_gpio_write
 *
 ****************************************************************************/

void ra_gpiowrite(gpio_pinset_t pinset, bool value)
{
  ra_gpio_write(pinset, value);
}

/****************************************************************************
 * Name: ra_gpioread
 *
 * Description:
 *   Legacy wrapper for ra_gpio_read
 *
 ****************************************************************************/

bool ra_gpioread(gpio_pinset_t pinset)
{
  return ra_gpio_read(pinset);
}
