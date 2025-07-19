/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_buttons.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include "ra_gpio.h"
#include "ra_icu.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_led1_state = true; /* Active low, true = off */
static bool g_led2_state = true; /* Active low, true = off */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_handler
 ****************************************************************************/

static int button_handler(int irq, void *context, void *arg)
{
  /* Toggle both LEDs */
  g_led1_state = !g_led1_state;
  g_led2_state = !g_led2_state;
  
  ra_gpiowrite(GPIO_LED1, g_led1_state);
  ra_gpiowrite(GPIO_LED2, g_led2_state);
  
  /* Clear the interrupt flag */
  ra_clear_ir(irq);
  
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Configure the button pin as an input with pullup and interrupt on falling edge */
  ra_configgpio(GPIO_SW1);
  
  /* Attach the button interrupt handler */
  irq_attach(SW1_IRQ, button_handler, NULL);
  
  /* Enable the interrupt */
  up_enable_irq(SW1_IRQ);
  
  return 1;
}