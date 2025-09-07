/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_gpio.h
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

/* Usage Example:
 *
 * // Configure P1_05 as output pin
 * gpio_pinset_t led_pin = RA_GPIO_OUTPUT_PIN(1, 5);
 * ra_gpio_config(led_pin);
 *
 * // Configure P0_03 as input with pull-up
 * gpio_pinset_t button_pin = RA_GPIO_INPUT_PULLUP_PIN(0, 3);
 * ra_gpio_config(button_pin);
 *
 * // Control GPIO pins
 * ra_gpio_write(led_pin, true);     // Turn on LED
 * bool button_state = ra_gpio_read(button_pin);  // Read button
 */

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA_GPIO_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra_memorymap.h"
#include "hardware/ra_system.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define R_PORT_PCNTR1_OFFSET            0x0000  /* Port Control Register 1 (32-bits) */
#define R_PORT_PODR_OFFSET              0x0000  /* Pmn Output Data (16-bits) */
#define R_PORT_PDR_OFFSET               0x0002  /* Pmn Direction (16-bits) */
#define R_PORT_PCNTR2_OFFSET            0x0004  /* Port Control Register 2 (32-bits) */
#define R_PORT_EIDR_OFFSET              0x0004  /* Port Event Input Data (16-bits) */
#define R_PORT_PIDR_OFFSET              0x0006  /* Pmn State (16-bits) */
#define R_PORT_PCNTR3_OFFSET            0x0008  /* Port Control Register 3 (32-bits) */
#define R_PORT_PORR_OFFSET              0x0008  /* Pmn Output Reset (16-bits) */
#define R_PORT_POSR_OFFSET              0x000a  /* Pmn Output Set (16-bits) */
#define R_PORT_PCNTR4_OFFSET            0x000c  /* Port Control Register 3 (32-bits) */
#define R_PORT_EORR_OFFSET              0x000c  /* Pmn Event Output Set (16-bits) */
#define R_PORT_EOSR_OFFSET              0x000e  /* Pmn Output Reset (16-bits) */

#define R_PORT_OFFSET                   0x0020  /* Relative Port Offset */

/* Register Addresses *******************************************************/

#define PORT0 (0)
#define PORT1 (1)
#define PORT2 (2)
#define PORT3 (3)
#define PORT4 (4)
#define PORT5 (5)
#define PORT6 (6)
#define PORT7 (7)
#define PORT8 (8)
#define PORT9 (9)

#define PIN0 (0)
#define PIN1 (1)
#define PIN2 (2)
#define PIN3 (3)
#define PIN4 (4)
#define PIN5 (5)
#define PIN6 (6)
#define PIN7 (7)
#define PIN8 (8)
#define PIN9 (9)
#define PIN10 (10)
#define PIN11 (11)
#define PIN12 (12)
#define PIN13 (13)
#define PIN14 (14)
#define PIN15 (15)

/* Relative PORT Registers */

#  define R_PORT_PCNTR1(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR1_OFFSET)
#  define R_PORT_PODR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PODR_OFFSET)
#  define R_PORT_PDR(port)             (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PDR_OFFSET)
#  define R_PORT_PCNTR2(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR2_OFFSET)
#  define R_PORT_EIDR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_EIDR_OFFSET)
#  define R_PORT_PIDR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PIDR_OFFSET)
#  define R_PORT_PCNTR3(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR3_OFFSET)
#  define R_PORT_PORR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PORR_OFFSET)
#  define R_PORT_POSR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_POSR_OFFSET)
#  define R_PORT_PCNTR4(port)          (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_PCNTR4_OFFSET)
#  define R_PORT_EORR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_EORR_OFFSET)
#  define R_PORT_EOSR(port)            (R_PORT0_BASE + (port)*R_PORT_OFFSET + R_PORT_EOSR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Port Control Register 1 (32-bits) */

#define R_PORT_PCNTR1_PODR_SHIFT              (16) /* 10000: Pmn Output Data */
#define R_PORT_PCNTR1_PODR_MASK               (0xffff)
#define R_PORT_PCNTR1_PDR_SHIFT               (0) /* 01: Pmn Direction */
#define R_PORT_PCNTR1_PDR_MASK                (0xffff)

/* Pmn Output Data (16-bits) */

#define R_PORT_PODR_PODR_SHIFT                (0) /* 01: Pmn Output Data */
#define R_PORT_PODR_PODR_MASK                 (0xffff)

/* Pmn Direction (16-bits) */

#define R_PORT_PDR_PDR_SHIFT                  (0) /* 01: Pmn Direction */
#define R_PORT_PDR_PDR_MASK                   (0xffff)

/* Port Control Register 2 (32-bits) */

#define R_PORT_PCNTR2_EIDR_SHIFT              (16) /* 10000: Pmn Event Input Data */
#define R_PORT_PCNTR2_EIDR_MASK               (0xffff)
#define R_PORT_PCNTR2_PIDR_SHIFT              (0) /* 01: Pmn Input Data */
#define R_PORT_PCNTR2_PIDR_MASK               (0xffff)

/* Port Event Input Data (16-bits) */

#define R_PORT_EIDR_EIDR_SHIFT                (0) /* 01: Pmn Event Input Data */
#define R_PORT_EIDR_EIDR_MASK                 (0xffff)

/* Pmn State (16-bits) */

#define R_PORT_PIDR_PIDR_SHIFT                (0) /* 01: Pmn Input Data */
#define R_PORT_PIDR_PIDR_MASK                 (0xffff)

/* Port Control Register 3 (32-bits) */

#define R_PORT_PCNTR3_PORR_SHIFT              (16) /* 10000: Pmn Output Reset */
#define R_PORT_PCNTR3_PORR_MASK               (0xffff)
#define R_PORT_PCNTR3_POSR_SHIFT              (0) /* 01: Pmn Output Set */
#define R_PORT_PCNTR3_POSR_MASK               (0xffff)

/* Pmn Output Reset (16-bits) */

#define R_PORT_PORR_PORR_SHIFT                (0) /* 01: Pmn Output Reset */
#define R_PORT_PORR_PORR_MASK                 (0xffff)

/* Pmn Output Set (16-bits) */

#define R_PORT_POSR_POSR_SHIFT                (0) /* 01: Pmn Output Set */
#define R_PORT_POSR_POSR_MASK                 (0xffff)

/* Port Control Register 3 (32-bits) */

#define R_PORT_PCNTR4_EORR_SHIFT              (16) /* 10000: Pmn Event Output Reset */
#define R_PORT_PCNTR4_EORR_MASK               (0xffff)
#define R_PORT_PCNTR4_EOSR_SHIFT              (0) /* 01: Pmn Event Output Set */
#define R_PORT_PCNTR4_EOSR_MASK               (0xffff)

/* Pmn Event Output Set (16-bits) */

#define R_PORT_EORR_EORR_SHIFT                (0) /* 01: Pmn Event Output Reset */
#define R_PORT_EORR_EORR_MASK                 (0xffff)

/* Pmn Output Reset (16-bits) */

#define R_PORT_EOSR_EOSR_SHIFT                (0) /* 01: Pmn Event Output Set */
#define R_PORT_EOSR_EOSR_MASK                 (0xffff)

#define RA_PRV_PFS_PSEL_OFFSET            (24)
#define RA_PRV_8BIT_MASK                  (0xFF)
#define RA_PFS_PDR_OUTPUT                 (4U)
#define RA_PRV_PIN_WRITE_MASK             (0xFFFE3FFE)

/* PFS Register Bit Definitions (avoid conflicts with ra8e1_pinmap.h) */

#define RA_PFS_PSEL_SHIFT                 (24)    /* PSEL field shift */
#define RA_PFS_PMR                        (1 << 16) /* Peripheral Mode Register */
#define RA_PFS_ASEL                       (1 << 15) /* Analog Select */
#define RA_PFS_ISEL                       (1 << 14) /* Interrupt Input Select */
#define RA_PFS_EOR                        (1 << 13) /* Event on Rising */
#define RA_PFS_EOF                        (1 << 12) /* Event on Falling */
#define RA_PFS_DSCR1                      (1 << 11) /* Drive Strength Control 1 */
#define RA_PFS_DSCR                       (1 << 10) /* Drive Strength Control 0 */
#define RA_PFS_NCODR                      (1 <<  6) /* N-Channel Open Drain */
#define RA_PFS_PCR                        (1 <<  4) /* Pull-up Control */
#define RA_PFS_PDR                        (1 <<  2) /* Port Direction */

/* GPIO Configuration Bit Fields */
#define RA_PFS_PIDR                       (1 <<  1) /* Port Input Data */
#define RA_PFS_PODR                       (1 <<  0) /* Port Output Data */

/* GPIO Configuration Options (matching Renesas FSP style) */

#define RA_GPIO_CFG_INPUT                 (0x00000000) /* Input (default) */
#define RA_GPIO_CFG_OUTPUT                (0x00000004) /* Output direction */

#define RA_GPIO_CFG_PULLUP                (0x00000010) /* Enable pull-up resistor */
#define RA_GPIO_CFG_OPENDRAIN             (0x00000040) /* Open-drain output */
#define RA_GPIO_CFG_DRIVE_LOW             (0x00000000) /* Low drive strength */
#define RA_GPIO_CFG_DRIVE_MID             (0x00000400) /* Mid drive strength */
#define RA_GPIO_CFG_DRIVE_HIGH            (0x00000C00) /* High drive strength */

/* GPIO Pin Configuration Macro */

#define RA_GPIO_PIN_CFG(port, pin, cfg)   \
  {                                       \
    .port = (port),                       \
    .pin = (pin),                         \
    .cfg = (cfg)                          \
  }

/* Common GPIO Pin Configurations */

#define RA_GPIO_INPUT_PIN(port, pin)      \
  RA_GPIO_PIN_CFG(port, pin, RA_GPIO_CFG_INPUT)

#define RA_GPIO_OUTPUT_PIN(port, pin)     \
  RA_GPIO_PIN_CFG(port, pin, RA_GPIO_CFG_OUTPUT)

#define RA_GPIO_INPUT_PULLUP_PIN(port, pin) \
  RA_GPIO_PIN_CFG(port, pin, RA_GPIO_CFG_INPUT | RA_GPIO_CFG_PULLUP)

#define RA_GPIO_OUTPUT_OPENDRAIN_PIN(port, pin) \
  RA_GPIO_PIN_CFG(port, pin, RA_GPIO_CFG_OUTPUT | RA_GPIO_CFG_OPENDRAIN)

#define RA_GPIO_CFG_OUTPUT_LOW            (0x00000000) /* Output low */
#define RA_GPIO_CFG_OUTPUT_HIGH           (0x00000001) /* Output high */
#define RA_GPIO_CFG_PULLUP_ENABLE         (0x00000010) /* Enable pull-up */
#define RA_GPIO_CFG_NMOS_ENABLE           (0x00000040) /* N-channel open drain */
#define RA_GPIO_CFG_PMOS_ENABLE           (0x00000080) /* P-channel open drain */
#define RA_GPIO_CFG_DRIVE_MID             (0x00000400) /* Medium drive strength */
#define RA_GPIO_CFG_DRIVE_HIGH            (0x00000C00) /* High drive strength */
#define RA_GPIO_CFG_EVENT_RISING_EDGE     (0x00001000) /* Rising edge trigger */
#define RA_GPIO_CFG_EVENT_FALLING_EDGE    (0x00002000) /* Falling edge trigger */
#define RA_GPIO_CFG_EVENT_BOTH_EDGES      (0x00003000) /* Both edge trigger */
#define RA_GPIO_CFG_IRQ_ENABLE            (0x00004000) /* Enable IRQ */
#define RA_GPIO_CFG_ANALOG_ENABLE         (0x00008000) /* Analog mode */
#define RA_GPIO_CFG_PERIPHERAL_PIN        (0x00010000) /* Peripheral mode */

/* Common GPIO Configuration Combinations */

#define RA_GPIO_INPUT                     (RA_GPIO_CFG_INPUT)
#define RA_GPIO_INPUT_PULLUP              (RA_GPIO_CFG_INPUT | RA_GPIO_CFG_PULLUP_ENABLE)
#define RA_GPIO_OUTPUT                    (RA_GPIO_CFG_OUTPUT)
#define RA_GPIO_OUTPUT_LOW                (RA_GPIO_CFG_OUTPUT | RA_GPIO_CFG_OUTPUT_LOW)
#define RA_GPIO_OUTPUT_HIGH               (RA_GPIO_CFG_OUTPUT | RA_GPIO_CFG_OUTPUT_HIGH)
#define RA_GPIO_OUTPUT_OPENDRAIN          (RA_GPIO_CFG_OUTPUT | RA_GPIO_CFG_NMOS_ENABLE)

/* Peripheral Selection Values (PSEL field) */

#define RA_GPIO_PSEL_IO                   (0x00) /* GPIO mode */
#define RA_GPIO_PSEL_AGT                  (0x01) /* AGT peripheral */
#define RA_GPIO_PSEL_GPT0                 (0x02) /* GPT0 peripheral */
#define RA_GPIO_PSEL_GPT1                 (0x03) /* GPT1 peripheral */
#define RA_GPIO_PSEL_SCI0_2_4_6_8         (0x04) /* SCI0/2/4/6/8 peripheral */
#define RA_GPIO_PSEL_SCI1_3_5_7_9         (0x05) /* SCI1/3/5/7/9 peripheral */
#define RA_GPIO_PSEL_SPI                  (0x06) /* SPI peripheral */
#define RA_GPIO_PSEL_IIC                  (0x07) /* IIC peripheral */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA_GPIO_H */
