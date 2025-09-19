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

/*
 * Alternate function pin selections
 */

/* UART/SCI Pin Definitions */
#define GPIO_SCI2_RX   GPIO_RXD2_MISO2_SCL2_2  /* P802 - SBUS RX (Drone RC) */
#define GPIO_SCI2_TX   GPIO_TXD2_MOSI2_SDA2_2  /* P801 */

#define GPIO_SCI1_RX   GPIO_RXD1_MISO1_SCL1_3  /* P502 */
#define GPIO_SCI1_TX   GPIO_TXD1_MOSI1_SDA1_3  /* P501 */

#define GPIO_SCI3_RX   GPIO_RXD3_MISO3_SCL3_B  /* P309 - GPS RX */
#define GPIO_SCI3_TX   GPIO_TXD3_MOSI3_SDA3_B  /* P310 - GPS TX */

#define GPIO_SCI9_RX   GPIO_RXD9_MISO9_SCL9_1  /* P110 */
#define GPIO_SCI9_TX   GPIO_TXD9_MOSI9_SDA9_1  /* P109 */

/* SPI0 Pin Definitions (IMU/Barometer on Pmod 1) - Master */
#define GPIO_SPI0_SCK   GPIO_SCK0_C            /* P611 - SPI Clock */
#define GPIO_SPI0_MOSI  GPIO_MOSI0_TXD0_SDA0_C /* P609 - SPI MOSI */
#define GPIO_SPI0_MISO  GPIO_MISO0_RXD0_SCL0_C /* P610 - SPI MISO */
#define GPIO_SPI0_SS0   GPIO_SS0_CTS_RTS0_C    /* P612 - IMU CS (ICM20948) */

/* SPI1 Pin Definitions (Expansion/Loopback) - Slave */
#define GPIO_SPI1_SCK   GPIO_SCK1_A            /* P500 - SPI1 Clock */
#define GPIO_SPI1_MOSI  GPIO_MOSI1_TXD1_SDA1_A /* P501 - SPI1 MOSI */
#define GPIO_SPI1_MISO  GPIO_MISO1_RXD1_SCL1_A /* P502 - SPI1 MISO */
#define GPIO_SPI1_SS0   GPIO_SS1_CTS_RTS1_A    /* P503 - SPI1 CS */

/* PWM/GPT Timer Pin Definitions for ESC Control */
#define GPIO_GPT0_A     GPIO_GTIOC0A_2         /* P415 - ESC 1 PWM (ch 0A) */
#define GPIO_GPT2_A     GPIO_GTIOC2A_3         /* P113 - ESC 2 PWM (ch 2A) */
#define GPIO_GPT2_B     GPIO_GTIOC2B_3         /* P114 - ESC 3 PWM (ch 2B) */
#define GPIO_GPT3_A     GPIO_GTIOC3A_1         /* P300 - ESC 4 PWM (ch 3A) */
#define GPIO_GPT3_B     GPIO_GTIOC3B_1         /* P404 - ESC 5 PWM */
#define GPIO_GPT4_A     GPIO_GTIOC4A_1         /* P302 - ESC 6 PWM (ch 4A) */
#define GPIO_GPT10_A    GPIO_GTIOC10A_1        /* P409 - Status LED 2 (alt function, corrected from P408) */

//#define GPIO_GPT5_A     GPIO_GTIOC5A_1         /* P905 - ESC 3 PWM (alt PWM) */

/* I2C Pin Definitions */
#define GPIO_I2C1_SDA   GPIO_SDA1_MOSI1_TXD1_C /* P511 - I2C SDA (Expansion) */
#define GPIO_I2C1_SCL   GPIO_SCL1_MISO1_RXD1_C /* P512 - I2C SCL (Expansion) */

/* ADC Pin Definitions for Battery Monitoring */
#define GPIO_ADC_AN000  GPIO_AN000_1           /* P004 - Battery Voltage */
#define GPIO_ADC_AN104  GPIO_AN104_1           /* P003 - Battery Current */

/* LED pin selections */

#define GPIO_LED1     GPIO_P404_OUTPUT_HIGH  /* P404 (Green LED1) */
#define GPIO_LED2     GPIO_P408_OUTPUT_HIGH  /* P408 (Green LED2) */

/* User Button - SW1 on P009 using IRQ13 */
#define GPIO_SW1         GPIO_IRQ13_P009

/* Drone-specific GPIO Pin Definitions */

/* IMU and Sensor Control Pins */
#define GPIO_BMP_CS      GPIO_P605_OUTPUT_HIGH  /* P605 - BMP388 CS */
#define GPIO_IMU_INT     GPIO_IRQ11_P006        /* P006 - IMU Data Ready (IRQ11) */

/* Buzzer Control */
#define GPIO_BUZZER      GPIO_P303_OUTPUT_LOW   /* P303 - Piezo Buzzer */

/* ESC PWM Alternate GPIO (for P905 with GPT5) */
//#define GPIO_ESC3_ALT    GPIO_GPT5_A            /* P905 - ESC 3 PWM (GPT5A) */
/* ESC PWM Alternate GPIO (for P905 without timer) */
#define GPIO_ESC3_ALT    GPIO_P905_OUTPUT_LOW   /* P905 - ESC 3 Alt PWM */

/* Additional GPIO Pin Definitions for General Use */

/* GPIO inputs with pullup for sensors/switches */
#define GPIO_SENSOR_INT1 GPIO_P100_INPUT_PULLUP /* P100 - General sensor interrupt */
#define GPIO_SENSOR_INT2 GPIO_P101_INPUT_PULLUP /* P101 - General sensor interrupt */
#define GPIO_SENSOR_INT3 GPIO_P104_INPUT_PULLUP /* P104 - General sensor interrupt */
#define GPIO_SENSOR_INT4 GPIO_P105_INPUT_PULLUP /* P105 - General sensor interrupt */

/* GPIO outputs for control signals */
#define GPIO_CTRL_OUT1   GPIO_P106_OUTPUT_LOW   /* P106 - General control output */
#define GPIO_CTRL_OUT2   GPIO_P107_OUTPUT_LOW   /* P107 - General control output */
#define GPIO_CTRL_OUT3   GPIO_P112_OUTPUT_LOW   /* P112 - General control output */
#define GPIO_CTRL_OUT4   GPIO_P400_OUTPUT_LOW   /* P400 - General control output */

/* Status/Debug pins */
#define GPIO_STATUS1     GPIO_P401_OUTPUT_LOW   /* P401 - Status indicator 1 */
#define GPIO_STATUS2     GPIO_P402_OUTPUT_LOW   /* P402 - Status indicator 2 */
#define GPIO_DEBUG1      GPIO_P403_OUTPUT_LOW   /* P403 - Debug signal 1 */
#define GPIO_DEBUG2      GPIO_P405_OUTPUT_LOW   /* P405 - Debug signal 2 */

/* Expansion/User pins */
#define GPIO_USER1       GPIO_P406_INPUT        /* P406 - User configurable pin 1 */
#define GPIO_USER2       GPIO_P407_INPUT        /* P407 - User configurable pin 2 */
#define GPIO_USER3       GPIO_P410_INPUT        /* P410 - User configurable pin 3 */
#define GPIO_USER4       GPIO_P411_INPUT        /* P411 - User configurable pin 4 */

/* Additional expansion pins */
#define GPIO_EXP1        GPIO_P412_INPUT        /* P412 - Expansion pin 1 */
#define GPIO_EXP2        GPIO_P413_INPUT        /* P413 - Expansion pin 2 */
#define GPIO_EXP3        GPIO_P414_INPUT        /* P414 - Expansion pin 3 */
#define GPIO_EXP4        GPIO_P415_INPUT        /* P415 - Expansion pin 4 */

/* SPI expansion pins (if not used for main SPI) */
#define GPIO_SPI_EXP1    GPIO_P600_INPUT        /* P600 - SPI expansion 1 */
#define GPIO_SPI_EXP2    GPIO_P601_INPUT        /* P601 - SPI expansion 2 */
#define GPIO_SPI_EXP3    GPIO_P602_INPUT        /* P602 - SPI expansion 3 */
#define GPIO_SPI_EXP4    GPIO_P603_INPUT        /* P603 - SPI expansion 4 */

/* Communication interface expansion */
#define GPIO_COMM_EXP1   GPIO_P800_INPUT        /* P800 - Communication expansion 1 */
#define GPIO_COMM_EXP2   GPIO_P801_INPUT        /* P801 - Communication expansion 2 */
#define GPIO_COMM_EXP3   GPIO_P803_INPUT        /* P803 - Communication expansion 3 */
#define GPIO_COMM_EXP4   GPIO_P804_INPUT        /* P804 - Communication expansion 4 */

/* GPIO Pin Definitions for Enhanced UART Driver ***********************/

/* SCI2 UART for SBUS RC Receiver (P802 RXD2) */
#ifdef CONFIG_RA_UART2_SBUS
#  define GPIO_UART2_RXD    GPIO_UART2_RXD_1
#  define GPIO_UART2_TXD    GPIO_UART2_TXD_1
#endif

/* SCI3 UART for GPS Module (P309 RXD3, P310 TXD3) */
#ifdef CONFIG_RA_UART3_GPS
#  define GPIO_UART3_RXD    GPIO_UART3_RXD_1
#  define GPIO_UART3_TXD    GPIO_UART3_TXD_1
#endif

/* SCI1 UART for Telemetry (MAVLink) - if configured */
#ifdef CONFIG_RA_UART1_TELEMETRY
#  define GPIO_UART1_RXD    GPIO_UART1_RXD_1
#  define GPIO_UART1_TXD    GPIO_UART1_TXD_1
#endif


/* Drone-specific Pin Name Mappings ************************************/

/* ESC PWM Output Pins - 400Hz ESC Control */
#define GPIO_ESC1_PWM    GPIO_GPT3_A    /* P300 - ESC 1 PWM (GPT ch 3A) */
#define GPIO_ESC2_PWM    GPIO_GPT0_A    /* P415 - ESC 2 PWM (GPT ch 0A) */
//#define GPIO_ESC3_PWM    GPIO_GPT5_A    /* P905 - ESC 3 PWM (GPT alt PWM) */
#define GPIO_ESC3_PWM    GPIO_ESC3_ALT  /* P905 - ESC 3 PWM (GPIO alt) */
#define GPIO_ESC4_PWM    GPIO_GPT2_B    /* P114 - ESC 4 PWM (GPT ch 2B) */
#define GPIO_ESC5_PWM    GPIO_GPT2_A    /* P113 - ESC 5 PWM (GPT ch 2A) */
#define GPIO_ESC6_PWM    GPIO_GPT4_A    /* P302 - ESC 6 PWM (GPT ch 4A) */

/* RC Receiver and GPS */
#define GPIO_SBUS_RX     GPIO_SCI2_RX   /* P802 - SBUS RC Receiver */
#define GPIO_GPS_RX      GPIO_SCI3_RX   /* P309 - GPS Module RX */
#define GPIO_GPS_TX      GPIO_SCI3_TX   /* P310 - GPS Module TX */

/* IMU and Barometer (SPI0) */
#define GPIO_IMU_SCK     GPIO_SPI0_SCK  /* P611 - SPI Clock */
#define GPIO_IMU_MOSI    GPIO_SPI0_MOSI /* P609 - SPI MOSI */
#define GPIO_IMU_MISO    GPIO_SPI0_MISO /* P610 - SPI MISO */
#define GPIO_IMU_CS      GPIO_SPI0_SS0  /* P612 - IMU Chip Select */
#define GPIO_IMU_DRDY    GPIO_IMU_INT   /* P006 - IMU Data Ready */

/* Battery Monitoring */
#define GPIO_BATT_VOLT   GPIO_ADC_AN000 /* P004 - Battery Voltage (5.7:1) */
#define GPIO_BATT_CURR   GPIO_ADC_AN104 /* P003 - Battery Current (ACS712) */

/* I2C Expansion Bus */
#define GPIO_EXP_SDA     GPIO_I2C1_SDA  /* P511 - Expansion I2C SDA */
#define GPIO_EXP_SCL     GPIO_I2C1_SCL  /* P512 - Expansion I2C SCL */

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

/* Board LED Definitions */
#define LED_1         0
#define LED_2         1
#define NLEDS         2

/* LED bits for use with board_userled_all() */
#define LED_1_BIT     (1 << LED_1)
#define LED_2_BIT     (1 << LED_2)

/*
 * UART Configuration Defaults
 */

/* SBUS configuration (100kbps, 8E2, inverted) */
#ifdef CONFIG_RA_UART2_SBUS
#  define UART2_DEFAULT_BAUD      100000
#  define UART2_DEFAULT_DATABITS  8
#  define UART2_DEFAULT_PARITY    2  /* Even parity */
#  define UART2_DEFAULT_STOPBITS  2
#  define UART2_DEFAULT_INVERTED  true
#endif

/* GPS configuration (38400bps, 8N1, non-inverted) */
#ifdef CONFIG_RA_UART3_GPS
#  define UART3_DEFAULT_BAUD      38400
#  define UART3_DEFAULT_DATABITS  8
#  define UART3_DEFAULT_PARITY    0  /* No parity */
#  define UART3_DEFAULT_STOPBITS  1
#  define UART3_DEFAULT_INVERTED  false
#endif

/* Telemetry MAVLink configuration (57600bps, 8N1, non-inverted) */
#ifdef CONFIG_RA_UART1_TELEMETRY
#  define UART1_DEFAULT_BAUD      57600
#  define UART1_DEFAULT_DATABITS  8
#  define UART1_DEFAULT_PARITY    0  /* No parity */
#  define UART1_DEFAULT_STOPBITS  1
#  define UART1_DEFAULT_INVERTED  false
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
