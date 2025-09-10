/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/include/board.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_INCLUDE_BOARD_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Do not include RA8 driver header files here. */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ Configuration ************************************************************/
/* The RA ICU system automatically assigns IRQ numbers dynamically.
 * Configuration-time interrupts (GPT, buttons) are handled by ra_icu_initialize().
 * Runtime interrupts (UART) use ra_irq_attach() which returns the assigned IRQ number.
 * This eliminates the need for manual IRQ slot management in board.h.
 */

/* Alternate function pin selections */

#define GPIO_SCI2_RX   GPIO_RXD2_MISO2_SCL2_2  /* P801 */
#define GPIO_SCI2_TX   GPIO_TXD2_MOSI2_SDA2_2  /* P802 */

#define GPIO_SCI1_RX   GPIO_RXD1_MISO1_SCL1_3  /* P502 */
#define GPIO_SCI1_TX   GPIO_TXD1_MOSI1_SDA1_3  /* P501 */

#define GPIO_SCI9_RX   GPIO_RXD9_MISO9_SCL9_1  /* P110 */
#define GPIO_SCI9_TX   GPIO_TXD9_MOSI9_SDA9_1  /* P109 */

/* SCI3 and SCI4 pins - Update these with actual pin assignments when determined */
/* #define GPIO_SCI3_RX   GPIO_PIN_UNDEFINED */
/* #define GPIO_SCI3_TX   GPIO_PIN_UNDEFINED */
/* #define GPIO_SCI4_RX   GPIO_PIN_UNDEFINED */
/* #define GPIO_SCI4_TX   GPIO_PIN_UNDEFINED */

/* LED pin selections */

#define GPIO_LED1     (gpio_pinset_t){ PORT4, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}  /* P404 (Green LED1) */
#define GPIO_LED2     (gpio_pinset_t){ PORT4, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}  /* P408 (Green LED2) */

/* User Button - SW1 on P009 using IRQ13 */
#define GPIO_SW1         (gpio_pinset_t){ PORT0, PIN9, (GPIO_INPUT | GPIO_PULLUP | GPIO_INT_FALLING)}

/* GPIO Pin Definitions for Enhanced UART Driver ***********************/

/* SCI2 UART for SBUS (if configured) */
#ifdef CONFIG_RA_UART2_SBUS
#  define GPIO_UART2_RXD    IOPORT_CFG(PORT_5, PIN_2, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SCI1_3_5_7_9)
#  define GPIO_UART2_TXD    IOPORT_CFG(PORT_5, PIN_1, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SCI1_3_5_7_9)
#endif

/* SCI3 UART for GPS (if configured) */
#ifdef CONFIG_RA_UART3_GPS
#  define GPIO_UART3_RXD    IOPORT_CFG(PORT_2, PIN_1, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SCI1_3_5_7_9)
#  define GPIO_UART3_TXD    IOPORT_CFG(PORT_2, PIN_0, IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SCI1_3_5_7_9)
#endif

/* Board LED Definitions */
#define BOARD_LED1         0
#define BOARD_LED2         1
#define BOARD_NLEDS        2

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined. In that case, the usage by the board port is defined in
 * include/board.h and src/ra8e1_auto_leds.c. The LEDs are used to encode
 * OS-related events as follows:
 *
 *  SYMBOL                MEANING                         LED STATE
 *                                                      LED1   LED2
 *  -----------------------  --------------------------  ----  ----
 */
#define LED_STARTED       0  /* NuttX has been started     OFF   OFF  */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF   OFF  */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF   OFF  */
#define LED_STACKCREATED  1  /* Idle stack created         ON    OFF  */
#define LED_INIRQ         2  /* In an interrupt            N/C   ON   */
#define LED_SIGNAL        2  /* In a signal handler        N/C   ON   */
#define LED_ASSERTION     2  /* An assertion failed        N/C   ON   */
#define LED_PANIC         3  /* The system has crashed     N/C   BLINK */
#define LED_IDLE          3  /* MCU is in sleep mode       ----  Not used ---- */

/* LED bits for use with board_userled_all() */
#define BOARD_LED1_BIT     (1 << BOARD_LED1)
#define BOARD_LED2_BIT     (1 << BOARD_LED2)

/* Clock Configuration **************************************************/

/* SCI clock source and divider settings based on RA8E1 reference */
#define SCI_CLOCK_SOURCE_PCLKA    /* Use PCLKA as SCI clock source */
#define SCI_CLOCK_DIVIDER         4  /* Divide by 4 for optimal baud rates */

/* UART Configuration Defaults *****************************************/

/* SBUS configuration (100kbps, 8E2, inverted) */
#ifdef CONFIG_RA_UART2_SBUS
#  define UART2_DEFAULT_BAUD      100000
#  define UART2_DEFAULT_DATABITS  8
#  define UART2_DEFAULT_PARITY    2  /* Even parity */
#  define UART2_DEFAULT_STOPBITS  2
#  define UART2_DEFAULT_INVERTED  true
#endif

/* GPS configuration (9600bps, 8N1, non-inverted) */
#ifdef CONFIG_RA_UART3_GPS
#  define UART3_DEFAULT_BAUD      9600
#  define UART3_DEFAULT_DATABITS  8
#  define UART3_DEFAULT_PARITY    0  /* No parity */
#  define UART3_DEFAULT_STOPBITS  1
#  define UART3_DEFAULT_INVERTED  false
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_INCLUDE_BOARD_H */
