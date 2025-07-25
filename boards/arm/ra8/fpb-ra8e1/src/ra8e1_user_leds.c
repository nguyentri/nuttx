/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_user_leds.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include "chip.h"
#include "ra_gpio.h"

#include <arch/board/board.h>

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIOs for output */

  ra_configgpio(GPIO_LED1);
  ra_configgpio(GPIO_LED2);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  gpio_pinset_t ledcfg;

  if (led == BOARD_LED1)
    {
      ledcfg = GPIO_LED1;
      ledon = ledon;
    }
  else if (led == BOARD_LED2)
    {
      ledcfg = GPIO_LED2;
      ledon = !ledon;  /* Invert logic for LED2 */
    }
  else
    {
      return;
    }

  ra_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

  ledon = ((ledset & BOARD_LED1_BIT) != 0);
  ra_gpiowrite(GPIO_LED1, ledon);

  ledon = ((ledset & BOARD_LED2_BIT) != 0);
  ra_gpiowrite(GPIO_LED2, ledon);

}

#endif /* !CONFIG_ARCH_LEDS */
