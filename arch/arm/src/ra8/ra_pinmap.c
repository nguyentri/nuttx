/****************************************************************************
 * arch/arm/src/ra8/ra_pinmap.c
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
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ra_pinmap.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RA_PIN_WRITE_PROTECT_KEY   0xA5  /* PFS write protection key */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_pin_unlock
 *
 * Description:
 *   Unlock PFS register write protection
 *
 ****************************************************************************/

static inline void ra_pin_unlock(void)
{
  /* Disable PFS protection */
  putreg8(0, R_PMISC_PWPRS);
  putreg8(R_PMISC_PWPRS_PFSWE, R_PMISC_PWPRS);
}

/****************************************************************************
 * Name: ra_pin_lock
 *
 * Description:
 *   Lock PFS register write protection
 *
 ****************************************************************************/

static inline void ra_pin_lock(void)
{
  /* Enable PFS protection */
  putreg8(0, R_PMISC_PWPRS);
  putreg8(R_PMISC_PWPRS_B0WI, R_PMISC_PWPRS);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_pin_config
 *
 * Description:
 *   Configure a pin based on pin configuration
 *
 ****************************************************************************/

int ra_pin_config(gpio_pinset_t pinset)
{
  uint32_t regaddr;
  uint32_t regval;
  uint8_t port = pinset.port;
  uint8_t pin = pinset.pin;
  uint32_t cfg = pinset.cfg;

  /* Validate port and pin numbers */
  if (port > 14 || pin > 15)
    {
      return -EINVAL;
    }

  /* Calculate PFS register address */
  regaddr = R_PFS(port, pin);

  /* Unlock PFS registers */
  ra_pin_unlock();

  /* Configure the pin */
  putreg32(cfg, regaddr);

  /* Lock PFS registers */
  ra_pin_lock();

  return OK;
}

/****************************************************************************
 * Name: ra_pin_set
 *
 * Description:
 *   Set pin output value
 *
 ****************************************************************************/

void ra_pin_set(uint8_t port, uint8_t pin, bool value)
{
  uint32_t regaddr;

  if (port > 14 || pin > 15)
    {
      return;
    }

  regaddr = R_PFS(port, pin);

  if (value)
    {
      modifyreg32(regaddr, 0, R_PFS_PODR);
    }
  else
    {
      modifyreg32(regaddr, R_PFS_PODR, 0);
    }
}

/****************************************************************************
 * Name: ra_pin_get
 *
 * Description:
 *   Get pin input value
 *
 ****************************************************************************/

bool ra_pin_get(uint8_t port, uint8_t pin)
{
  uint32_t regaddr;
  uint32_t regval;

  if (port > 14 || pin > 15)
    {
      return false;
    }

  regaddr = R_PFS(port, pin);
  regval = getreg32(regaddr);

  return (regval & R_PFS_PIDR) != 0;
}

/****************************************************************************
 * Name: ra_pin_toggle
 *
 * Description:
 *   Toggle pin output value
 *
 ****************************************************************************/

void ra_pin_toggle(uint8_t port, uint8_t pin)
{
  uint32_t regaddr;
  uint32_t regval;

  if (port > 14 || pin > 15)
    {
      return;
    }

  regaddr = R_PFS(port, pin);
  regval = getreg32(regaddr);

  if (regval & R_PFS_PODR)
    {
      modifyreg32(regaddr, R_PFS_PODR, 0);
    }
  else
    {
      modifyreg32(regaddr, 0, R_PFS_PODR);
    }
}

/****************************************************************************
 * Name: ra_pin_set_direction
 *
 * Description:
 *   Set pin direction (input/output)
 *
 ****************************************************************************/

int ra_pin_set_direction(uint8_t port, uint8_t pin, bool output)
{
  uint32_t regaddr;

  if (port > 14 || pin > 15)
    {
      return -EINVAL;
    }

  regaddr = R_PFS(port, pin);

  ra_pin_unlock();

  if (output)
    {
      modifyreg32(regaddr, 0, R_PFS_PDR);
    }
  else
    {
      modifyreg32(regaddr, R_PFS_PDR, 0);
    }

  ra_pin_lock();

  return OK;
}

/****************************************************************************
 * Name: ra_pin_set_pullup
 *
 * Description:
 *   Set pin pull-up enable/disable
 *
 ****************************************************************************/

int ra_pin_set_pullup(uint8_t port, uint8_t pin, bool enable)
{
  uint32_t regaddr;

  if (port > 14 || pin > 15)
    {
      return -EINVAL;
    }

  regaddr = R_PFS(port, pin);

  ra_pin_unlock();

  if (enable)
    {
      modifyreg32(regaddr, 0, R_PFS_PCR);
    }
  else
    {
      modifyreg32(regaddr, R_PFS_PCR, 0);
    }

  ra_pin_lock();

  return OK;
}

/****************************************************************************
 * Name: ra_pin_set_function
 *
 * Description:
 *   Set pin peripheral function
 *
 ****************************************************************************/

int ra_pin_set_function(uint8_t port, uint8_t pin, uint8_t function)
{
  uint32_t regaddr;
  uint32_t regval;

  if (port > 14 || pin > 15)
    {
      return -EINVAL;
    }

  regaddr = R_PFS(port, pin);

  ra_pin_unlock();

  regval = getreg32(regaddr);
  regval &= ~(R_PFS_PSEL_MASK << R_PFS_PSEL_SHIFT);
  regval |= (function & R_PFS_PSEL_MASK) << R_PFS_PSEL_SHIFT;

  /* Enable peripheral mode if function is not GPIO */
  if (function != 0)
    {
      regval |= R_PFS_PMR;
    }
  else
    {
      regval &= ~R_PFS_PMR;
    }

  putreg32(regval, regaddr);

  ra_pin_lock();

  return OK;
}

/****************************************************************************
 * Name: ra_pin_set_drive_strength
 *
 * Description:
 *   Set pin drive strength
 *
 ****************************************************************************/

int ra_pin_set_drive_strength(uint8_t port, uint8_t pin, uint8_t strength)
{
  uint32_t regaddr;
  uint32_t regval;

  if (port > 14 || pin > 15 || strength > 3)
    {
      return -EINVAL;
    }

  regaddr = R_PFS(port, pin);

  ra_pin_unlock();

  regval = getreg32(regaddr);
  regval &= ~(R_PFS_DSCR | R_PFS_DSCR1);

  if (strength & 0x01)
    {
      regval |= R_PFS_DSCR;
    }
  if (strength & 0x02)
    {
      regval |= R_PFS_DSCR1;
    }

  putreg32(regval, regaddr);

  ra_pin_lock();

  return OK;
}
