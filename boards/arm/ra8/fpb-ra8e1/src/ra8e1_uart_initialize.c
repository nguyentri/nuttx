/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_uart_initialize.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "hardware/ra_sci.h"
#include "fpb-ra8e1.h"

#ifdef CONFIG_RA_SCI_UART

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART pin configurations for FPB-RA8E1 board */
#define RA8E1_SCI2_TXD_PIN   GPIO_SCI2_TX  /* P103 */
#define RA8E1_SCI2_RXD_PIN   GPIO_SCI2_RX  /* P102 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_uart_gpio_config
 *
 * Description:
 *   Configure GPIO pins for UART/SCI operation
 *
 ****************************************************************************/

static void ra8e1_uart_gpio_config(void)
{
#ifdef CONFIG_RA_SCI2_UART
  /* Configure SCI2 pins for UART operation */
  ra_configgpio(RA8E1_SCI2_TXD_PIN);
  ra_configgpio(RA8E1_SCI2_RXD_PIN);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_uart_initialize
 *
 * Description:
 *   Initialize UART/SCI drivers for RA8E1
 *
 ****************************************************************************/

int ra8e1_uart_initialize(void)
{
  int ret = OK;

  /* Configure UART GPIO pins */
  ra8e1_uart_gpio_config();

#ifdef CONFIG_RA_SCI2_UART
  /* Initialize SCI2 as UART */
  _info("Initializing SCI2 UART\n");
  
  /* The low-level driver initialization is handled by the architecture-
   * specific code in ra_uart.c. Here we just configure the board-specific
   * aspects like GPIO pins and any board-level setup.
   */
  
#ifdef CONFIG_RA_SCI_UART_DMA_ENABLE
  _info("SCI2 UART with DTC/DMA support enabled\n");
#endif

#endif /* CONFIG_RA_SCI2_UART */

  return ret;
}

#endif /* CONFIG_RA_SCI_UART */
