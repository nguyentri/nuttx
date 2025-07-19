/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_auto_leds.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>

#include "chip.h"
#include "ra_gpio.h"

/* The board.h file may override pin configurations defined in ra_pinmap.h */

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/*  There are two user-controllable LEDs on board the RA8E1 FPB board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     LED1 Green       P404
 *     LED2 Green       P408
 *
 * These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/ra8e1_auto_leds.c. The LEDs are used to encode 
 * OS-related events as follows:
 *
 *   SYMBOL                MEANING                         LED STATE
 *                                                   LED1       LED2
 *   -------------------  -----------------------  --------- ---------
 *   LED_STARTED          NuttX has been started     OFF       OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF       OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF       OFF
 *   LED_STACKCREATED     Idle stack created         ON        OFF
 *   LED_INIRQ            In an interrupt            N/C       ON
 *   LED_SIGNAL           In a signal handler        N/C       ON
 *   LED_ASSERTION        An assertion failed        N/C       ON
 *   LED_PANIC            The system has crashed     N/C     Blinking
 *   LED_IDLE             MCU is in sleep mode       ------ Not used ------
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GPIOs for output */
  ra_configgpio(GPIO_LED1);
  ra_configgpio(GPIO_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED: LED1=OFF LED2=OFF */
      default:
      case 0:
        ra_gpiowrite(GPIO_LED1, true);  /* LED off (active low) */
        ra_gpiowrite(GPIO_LED2, true);  /* LED off (active low) */
        break;

      /* 1: LED_STACKCREATED: LED1=ON LED2=OFF */
      case 1:
        ra_gpiowrite(GPIO_LED1, false); /* LED on (active low) */
        ra_gpiowrite(GPIO_LED2, true);  /* LED off (active low) */
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: LED1=N/C LED2=ON */
      case 2:
        ra_gpiowrite(GPIO_LED2, false); /* LED on (active low) */
        break;

      /* 3: LED_PANIC: LED2=Blinking */
      case 3:
        ra_gpiowrite(GPIO_LED2, false); /* LED on (active low) */
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      /* 0-1: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED, LED_STACKCREATED */
      default:
      case 0:
      case 1:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: LED1=N/C LED2=OFF */
      case 2:
        ra_gpiowrite(GPIO_LED2, true);  /* LED off (active low) */
        break;

      /* 3: LED_PANIC: LED2=Blinking */
      case 3:
        ra_gpiowrite(GPIO_LED2, true);  /* LED off (active low) */
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
