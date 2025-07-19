/****************************************************************************
 * arch/arm/src/ra8/ra_pinmap.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_PINMAP_H
#define __ARCH_ARM_SRC_RA8_RA_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/ra_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin drive strength options */
#define RA_PIN_DRIVE_LOW      0x00    /* Low drive strength */
#define RA_PIN_DRIVE_MEDIUM   0x01    /* Medium drive strength */
#define RA_PIN_DRIVE_HIGH     0x02    /* High drive strength */
#define RA_PIN_DRIVE_MAX      0x03    /* Maximum drive strength */

/* Pin peripheral function macros */
#define RA_PIN_FUNCTION_GPIO   0x00   /* GPIO function */
#define RA_PIN_FUNCTION_AGT    0x01   /* AGT function */
#define RA_PIN_FUNCTION_GPT    0x02   /* GPT function */
#define RA_PIN_FUNCTION_SCI    0x04   /* SCI function */
#define RA_PIN_FUNCTION_SPI    0x06   /* SPI function */
#define RA_PIN_FUNCTION_IIC    0x07   /* I2C function */
#define RA_PIN_FUNCTION_CAN    0x10   /* CAN function */
#define RA_PIN_FUNCTION_USB    0x13   /* USB function */
#define RA_PIN_FUNCTION_DAC    0x14   /* DAC function */
#define RA_PIN_FUNCTION_ADC    0x15   /* ADC function (analog) */

/* Pin configuration helper macros */
#define RA_PIN_CFG_OUTPUT(port, pin) \
  ((gpio_pinset_t){port, pin, R_PFS_PDR})

#define RA_PIN_CFG_INPUT(port, pin) \
  ((gpio_pinset_t){port, pin, 0})

#define RA_PIN_CFG_INPUT_PULLUP(port, pin) \
  ((gpio_pinset_t){port, pin, R_PFS_PCR})

#define RA_PIN_CFG_PERIPHERAL(port, pin, func) \
  ((gpio_pinset_t){port, pin, R_PFS_PMR | ((func & R_PFS_PSEL_MASK) << R_PFS_PSEL_SHIFT)})

#define RA_PIN_CFG_ANALOG(port, pin) \
  ((gpio_pinset_t){port, pin, R_PFS_ASEL})

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ra_pin_config
 *
 * Description:
 *   Configure a pin based on pin configuration structure
 *
 * Input Parameters:
 *   pinset - Pin configuration structure
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int ra_pin_config(gpio_pinset_t pinset);

/****************************************************************************
 * Name: ra_pin_set
 *
 * Description:
 *   Set pin output value
 *
 * Input Parameters:
 *   port  - Port number (0-14)
 *   pin   - Pin number (0-15)
 *   value - Output value (true = high, false = low)
 *
 ****************************************************************************/

void ra_pin_set(uint8_t port, uint8_t pin, bool value);

/****************************************************************************
 * Name: ra_pin_get
 *
 * Description:
 *   Get pin input value
 *
 * Input Parameters:
 *   port - Port number (0-14)
 *   pin  - Pin number (0-15)
 *
 * Returned Value:
 *   Pin input value (true = high, false = low)
 *
 ****************************************************************************/

bool ra_pin_get(uint8_t port, uint8_t pin);

/****************************************************************************
 * Name: ra_pin_toggle
 *
 * Description:
 *   Toggle pin output value
 *
 * Input Parameters:
 *   port - Port number (0-14)
 *   pin  - Pin number (0-15)
 *
 ****************************************************************************/

void ra_pin_toggle(uint8_t port, uint8_t pin);

/****************************************************************************
 * Name: ra_pin_set_direction
 *
 * Description:
 *   Set pin direction (input/output)
 *
 * Input Parameters:
 *   port   - Port number (0-14)
 *   pin    - Pin number (0-15)
 *   output - Direction (true = output, false = input)
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int ra_pin_set_direction(uint8_t port, uint8_t pin, bool output);

/****************************************************************************
 * Name: ra_pin_set_pullup
 *
 * Description:
 *   Set pin pull-up enable/disable
 *
 * Input Parameters:
 *   port   - Port number (0-14)
 *   pin    - Pin number (0-15)
 *   enable - Pull-up enable (true = enable, false = disable)
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int ra_pin_set_pullup(uint8_t port, uint8_t pin, bool enable);

/****************************************************************************
 * Name: ra_pin_set_function
 *
 * Description:
 *   Set pin peripheral function
 *
 * Input Parameters:
 *   port     - Port number (0-14)
 *   pin      - Pin number (0-15)
 *   function - Peripheral function code
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int ra_pin_set_function(uint8_t port, uint8_t pin, uint8_t function);

/****************************************************************************
 * Name: ra_pin_set_drive_strength
 *
 * Description:
 *   Set pin drive strength
 *
 * Input Parameters:
 *   port     - Port number (0-14)
 *   pin      - Pin number (0-15)
 *   strength - Drive strength (0-3)
 *
 * Returned Value:
 *   OK on success, negative error code on failure
 *
 ****************************************************************************/

int ra_pin_set_drive_strength(uint8_t port, uint8_t pin, uint8_t strength);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_PINMAP_H */
