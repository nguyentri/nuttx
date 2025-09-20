/****************************************************************************
 * arch/arm/src/ra8/hardware/ra8e1_pinmap.h
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

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA8E1_PINMAP_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA8E1_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/
#define R_PFS_PSEL_PORT_OFFSET          0x40
#define R_PFS_PSEL_PIN_OFFSET           0x04
#define R_PMISC_PWPRS_OFFSET            0x14
#define R_PMISC_PMSAR_OFFSET            0x30

/* PSCU - Port Security Control Unit Registers */

#define R_PSCU_PSARB_OFFSET               0x04
#define R_PSCU_PSARC_OFFSET               0x08
#define R_PSCU_PSARD_OFFSET               0x0C
#define R_PSCU_PSARE_OFFSET               0x10

#define R_PSCU_PSARB                      (R_PSCU_BASE + R_PSCU_PSARB_OFFSET)
#define R_PSCU_PSARC                      (R_PSCU_BASE + R_PSCU_PSARC_OFFSET)
#define R_PSCU_PSARD                      (R_PSCU_BASE + R_PSCU_PSARD_OFFSET)
#define R_PSCU_PSARE                      (R_PSCU_BASE + R_PSCU_PSARE_OFFSET)

/* Register Addresses *******************************************************/

#define R_PFS(port,pin)                 (R_PFS_BASE + (port)*R_PFS_PSEL_PORT_OFFSET + (pin)*R_PFS_PSEL_PIN_OFFSET)
#define R_PMISC_PWPRS                    (R_PMISC_BASE + R_PMISC_PWPRS_OFFSET)

/* PMSAR - Port Security Attribution Registers */

#define R_PMSAR_BASE                      (R_PMISC_BASE + R_PMISC_PMSAR_OFFSET)
#define R_PMSAR(port)                     (R_PMSAR_BASE + (port) * 0x04)
#define R_PMSAR_NUM                       (10)  /* Ports 0-9 have PMSAR registers */

/* Register Bitfield Definitions ********************************************/

/* PFS - Pmn Pin Function Control Register */

#define R_PFS_PSEL_SHIFT          (24) /* 1000000: Port Function Select These bits select the peripheral function. For individual pin functions, see the MPC table */
#define R_PFS_PSEL_MASK           (0x1f)
#define R_PFS_PMR                 (1 << 16) /* Bit 16: Port Mode Control */
#define R_PFS_ASEL                (1 << 15) /* Bit 15: Analog Input enable */
#define R_PFS_ISEL                (1 << 14) /* Bit 14: IRQ input enable */
#define R_PFS_EOR                 (1 << 13) /* Bit 13: Event on Rising */
#define R_PFS_EOF                 (1 << 12) /* Bit 12: Event on Falling */
#define R_PFS_DSCR1               (1 << 11) /* Bit 11: Port Drive Capability 1 */
#define R_PFS_DSCR                (1 << 10) /* Bit 10: Port Drive Capability */
#define R_PFS_NCODR               (1 <<  6) /* Bit 6: N-Channel Open Drain Control */
#define R_PFS_PCR                 (1 <<  4) /* Bit 4: Pull-up Control */
#define R_PFS_PDR                 (1 <<  2) /* Bit 2: Port Direction */
#define R_PFS_PIDR                (1 <<  1) /* Bit 1: Port Input Data */
#define R_PFS_PODR                (1 <<  0) /* Bit 0: Port Output Data */

/* PMISC Register Bits */
#define R_PMISC_PWPRS_B0WI        (1 <<  7) /* 80: PFSWE Bit Write Disable */
#define R_PMISC_PWPRS_PFSWE       (1 <<  6) /* 40: PFS Register Write Enable */

/* Bit definitions for PWPRS are provided in ra_gpio.h to avoid duplication here. */

#define PFS_PSEL_HIZ                 (0x00 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_AGT                 (0x01 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_GPT                 (0x02 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_AGT1                (0x03 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SCI                 (0x04 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SCI1                (0x05 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SPI                 (0x06 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_IIC                 (0x07 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_KINT                (0x08 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CLKOUT_ACMPLP_RTC   (0x09 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CAC_ADC14           (0x0a << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CTSU                (0x0c << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SLCDC               (0x0d << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CAN                 (0x10 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_SSIE                (0x12 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_USBFS               (0x13 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_DAC                 (0x14 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_TRACE               (0x15 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_DEBUG               (0x16 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_OPAMP               (0x17 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_XSPI                (0x18 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_ETHERNET            (0x19 << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_CEU                 (0x1A << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_OSPI                (0x1B << R_PFS_PSEL_SHIFT)
#define PFS_PSEL_ULPT                (0x1C << R_PFS_PSEL_SHIFT)

/* SCI Alternative Functions */

/* SCI0 Alternative Functions */
#define GPIO_RXD0_MISO0_SCL0_A              (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_A              (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD0_MISO0_SCL0_B              (gpio_pinset_t){ PORT6,PIN2, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_B              (gpio_pinset_t){ PORT6,PIN3, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD0_MISO0_SCL0_C              (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD0_MOSI0_SDA0_C              (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SCI1 Alternative Functions */
#define GPIO_RXD1_MISO1_SCL1_A              (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_A              (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD1_MISO1_SCL1_B              (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_B              (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD1_MISO1_SCL1_C              (gpio_pinset_t){ PORT5,PIN12, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD1_MOSI1_SDA1_C              (gpio_pinset_t){ PORT5,PIN11, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SCI2 Alternative Functions */
#define GPIO_RXD2_MISO2_SCL2_A              (gpio_pinset_t){ PORT8,PIN2, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD2_MOSI2_SDA2_A              (gpio_pinset_t){ PORT8,PIN1, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD2_MISO2_SCL2_B              (gpio_pinset_t){ PORT7,PIN0, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD2_MOSI2_SDA2_B              (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SCI3 Alternative Functions */
#define GPIO_RXD3_MISO3_SCL3_A              (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD3_MOSI3_SDA3_A              (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD3_MISO3_SCL3_B              (gpio_pinset_t){ PORT3,PIN9, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD3_MOSI3_SDA3_B              (gpio_pinset_t){ PORT3,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SCI4 Alternative Functions */
#define GPIO_SCK4_A                         (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SCK4_B                         (gpio_pinset_t){ PORT7,PIN8, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_TXD4_MOSI4_SDA4_A              (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD4_MOSI4_SDA4_B              (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_RXD4_MISO4_SCL4_A              (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD4_MISO4_SCL4_B              (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_CTS_RTS4_SS4_A                 (gpio_pinset_t){ PORT4,PIN3, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS_RTS4_SS4_B                 (gpio_pinset_t){ PORT7,PIN9, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_CTS4_A                         (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS4_B                         (gpio_pinset_t){ PORT7,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SCI9 Alternative Functions */
#define GPIO_RXD9_MISO9_SCL9_A              (gpio_pinset_t){ PORT1,PIN1, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_A              (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_RXD9_MISO9_SCL9_B              (gpio_pinset_t){ PORT2,PIN8, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_TXD9_MOSI9_SDA9_B              (gpio_pinset_t){ PORT2,PIN9, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SCK9_A                         (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SCK9_B                         (gpio_pinset_t){ PORT2,PIN11, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS9_RTS9_A                    (gpio_pinset_t){ PORT1,PIN3, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS9_RTS9_B                    (gpio_pinset_t){ PORT2,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS9_A                         (gpio_pinset_t){ PORT1,PIN4, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS9_B                         (gpio_pinset_t){ PORT3,PIN8, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SPI/SCI Alternative Functions for SPI0 */
#define GPIO_SCK0_A                         (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SCK0_B                         (gpio_pinset_t){ PORT6,PIN1, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SCK0_C                         (gpio_pinset_t){ PORT6,PIN11, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_MOSI0_TXD0_SDA0_A              (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_MOSI0_TXD0_SDA0_B              (gpio_pinset_t){ PORT6,PIN3, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_MOSI0_TXD0_SDA0_C              (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_MISO0_RXD0_SCL0_A              (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_MISO0_RXD0_SCL0_B              (gpio_pinset_t){ PORT6,PIN2, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_MISO0_RXD0_SCL0_C              (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_SS0_CTS_RTS0_A                 (gpio_pinset_t){ PORT1,PIN14, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SS0_CTS_RTS0_B                 (gpio_pinset_t){ PORT6,PIN4, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_SS0_CTS_RTS0_C                 (gpio_pinset_t){ PORT6,PIN12, (PFS_PSEL_SCI | R_PFS_PMR)}

#define GPIO_CTS0_A                         (gpio_pinset_t){ PORT1,PIN15, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS0_B                         (gpio_pinset_t){ PORT6,PIN5, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_CTS0_C                         (gpio_pinset_t){ PORT6,PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}

/* SPI Alternative Functions for Group B */
#define GPIO_MISOB_A                        (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_MISOB_B                        (gpio_pinset_t){ PORT4,PIN10, (PFS_PSEL_SPI | R_PFS_PMR)}

#define GPIO_MOSIB_A                        (gpio_pinset_t){ PORT1,PIN15, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_MOSIB_B                        (gpio_pinset_t){ PORT4,PIN11, (PFS_PSEL_SPI | R_PFS_PMR)}

#define GPIO_RSPCKB_A                       (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_RSPCKB_B                       (gpio_pinset_t){ PORT4,PIN12, (PFS_PSEL_SPI | R_PFS_PMR)}

#define GPIO_SSLB0_B                        (gpio_pinset_t){ PORT4,PIN13, (PFS_PSEL_SPI | R_PFS_PMR)}

/* SPI SSL Alternative Functions */
#define GPIO_SSLA0_A                        (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA0_B                        (gpio_pinset_t){ PORT1,PIN14, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA0_C                        (gpio_pinset_t){ PORT7,PIN3, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA1_A                        (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA1_B                        (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA1_C                        (gpio_pinset_t){ PORT7,PIN4, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA2_A                        (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA2_B                        (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA2_C                        (gpio_pinset_t){ PORT7,PIN5, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA3_A                        (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA3_B                        (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLA3_C                        (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB0_A                        (gpio_pinset_t){ PORT1,PIN3, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB1_A                        (gpio_pinset_t){ PORT1,PIN4, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB1_B                        (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB2_A                        (gpio_pinset_t){ PORT1,PIN5, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB2_B                        (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB3_A                        (gpio_pinset_t){ PORT1,PIN6, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_SSLB3_B                        (gpio_pinset_t){ PORT7,PIN8, (PFS_PSEL_SPI | R_PFS_PMR)}

/* I2C Alternative Functions */
#define GPIO_SDA0_A                         (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SDA0_B                         (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL0_A                         (gpio_pinset_t){ PORT4,PIN10, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL0_B                         (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_IIC | R_PFS_PMR)}

#define GPIO_SDA1_A                         (gpio_pinset_t){ PORT5,PIN11, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SDA1_B                         (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL1_A                         (gpio_pinset_t){ PORT5,PIN12, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL1_B                         (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_IIC | R_PFS_PMR)}

/* Special Pin Functions */
#define GPIO_NMI                            (gpio_pinset_t){ PORT2,PIN0, (0)}
#define GPIO_MD                             (gpio_pinset_t){ PORT2,PIN1, (0)}

#define GPIO_GTIOC0A_1                      (gpio_pinset_t){ PORT2,PIN11, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC0A_2                      (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC0A_3                      (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC0A_4                      (gpio_pinset_t){ PORT5,PIN12, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC0B_1                      (gpio_pinset_t){ PORT2,PIN10, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC0B_2                      (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC0B_3                      (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC0B_4                      (gpio_pinset_t){ PORT5,PIN11, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC1A_1                      (gpio_pinset_t){ PORT1,PIN5, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC1A_2                      (gpio_pinset_t){ PORT2,PIN9, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC1A_3                      (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC1B_1                      (gpio_pinset_t){ PORT1,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC1B_2                      (gpio_pinset_t){ PORT2,PIN8, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC1B_3                      (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC2A_1                      (gpio_pinset_t){ PORT1, PIN3, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC2A_2                      (gpio_pinset_t){ PORT7, PIN13, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC2A_3                      (gpio_pinset_t){ PORT1, PIN13, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC2B_1                      (gpio_pinset_t){ PORT1, PIN2, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC2B_2                      (gpio_pinset_t){ PORT7, PIN12, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC2B_3                      (gpio_pinset_t){ PORT1, PIN14, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC3A_1                      (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC3A_2                      (gpio_pinset_t){ PORT4,PIN3, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC3B_1                      (gpio_pinset_t){ PORT4,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC3B_2                      (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC4A_1                      (gpio_pinset_t){ PORT3,PIN2, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC4A_2                      (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC4B_1                      (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC4B_2                      (gpio_pinset_t){ PORT3,PIN1, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC4B_3                      (gpio_pinset_t){ PORT6,PIN11, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC5A_1                      (gpio_pinset_t){ PORT2,PIN3, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC5A_2                      (gpio_pinset_t){ PORT7,PIN0, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC5A_3                      (gpio_pinset_t){ PORT1,PIN15, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC5B_1                      (gpio_pinset_t){ PORT2,PIN2, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC5B_2                      (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTIOC5B_3                      (gpio_pinset_t){ PORT7,PIN1, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC10A_1                     (gpio_pinset_t){ PORT4, PIN8, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC10B_1                     (gpio_pinset_t){ PORT4, PIN7, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC11A_1                     (gpio_pinset_t){ PORT8,PIN0, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC11B_1                     (gpio_pinset_t){ PORT8,PIN1, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC12A_1                     (gpio_pinset_t){ PORT8,PIN2, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC12B_1                     (gpio_pinset_t){ PORT8,PIN3, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC13A_1                     (gpio_pinset_t){ PORT8,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}

#define GPIO_GTIOC13B_1                     (gpio_pinset_t){ PORT8,PIN8, (PFS_PSEL_GPT | R_PFS_PMR)}

/* CAN Alternative Functions */
#define GPIO_CRX0_1                         (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX0_1                         (gpio_pinset_t){ PORT1,PIN3, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX0_2                         (gpio_pinset_t){ PORT2,PIN2, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX0_2                         (gpio_pinset_t){ PORT2,PIN3, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX0_3                         (gpio_pinset_t){ PORT3,PIN11, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX0_3                         (gpio_pinset_t){ PORT3,PIN12, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX0_4                         (gpio_pinset_t){ PORT4,PIN2, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX0_4                         (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX0_5                         (gpio_pinset_t){ PORT7,PIN5, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX0_5                         (gpio_pinset_t){ PORT7,PIN4, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX1_1                         (gpio_pinset_t){ PORT2,PIN8, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX1_1                         (gpio_pinset_t){ PORT2,PIN9, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX1_2                         (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX1_2                         (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX1_3                         (gpio_pinset_t){ PORT5,PIN11, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX1_3                         (gpio_pinset_t){ PORT5,PIN12, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CRX1_4                         (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_CAN | R_PFS_PMR)}
#define GPIO_CTX1_4                         (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_CAN | R_PFS_PMR)}

/* USB Alternative Functions */
#define GPIO_USB_VBUS                       (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_VBUSEN                     (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_OVRCURA                    (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_OVRCURB                    (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_ID                         (gpio_pinset_t){ PORT4,PIN11, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_EXICEN                     (gpio_pinset_t){ PORT4,PIN12, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_DP                         (gpio_pinset_t){ PORT8,PIN14, (PFS_PSEL_USBFS | R_PFS_PMR)}
#define GPIO_USB_DM                         (gpio_pinset_t){ PORT8,PIN15, (PFS_PSEL_USBFS | R_PFS_PMR)}

/* Ethernet Alternative Functions */
#define GPIO_ET0_CRS_A                      (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_CRS_B                      (gpio_pinset_t){ PORT7,PIN5, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_EXOUT                      (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_LINKSTA                    (gpio_pinset_t){ PORT1,PIN14, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_WOL_1                      (gpio_pinset_t){ PORT1,PIN15, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_WOL_2                      (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_RX_CLK_A                   (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_RX_CLK_B                   (gpio_pinset_t){ PORT7,PIN4, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ERXD0_A                    (gpio_pinset_t){ PORT3,PIN1, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ERXD0_B                    (gpio_pinset_t){ PORT7,PIN3, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ERXD1_A                    (gpio_pinset_t){ PORT3,PIN2, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ERXD1_B                    (gpio_pinset_t){ PORT7,PIN2, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ETXD0_A                    (gpio_pinset_t){ PORT3,PIN3, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ETXD0_B                    (gpio_pinset_t){ PORT7,PIN1, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ETXD1_A                    (gpio_pinset_t){ PORT3,PIN4, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ETXD1_B                    (gpio_pinset_t){ PORT7,PIN0, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_RX_ER_A                    (gpio_pinset_t){ PORT3,PIN5, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_RX_ER_B                    (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_TX_EN_A                    (gpio_pinset_t){ PORT3,PIN6, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_TX_EN_B                    (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_MDIO_1                     (gpio_pinset_t){ PORT3,PIN7, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_MDIO_2                     (gpio_pinset_t){ PORT4,PIN2, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_MDC_1                      (gpio_pinset_t){ PORT3,PIN8, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_MDC_2                      (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ETXD3                      (gpio_pinset_t){ PORT3,PIN9, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ETXD2                      (gpio_pinset_t){ PORT3,PIN10, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_TX_ER                      (gpio_pinset_t){ PORT3,PIN11, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_TX_CLK                     (gpio_pinset_t){ PORT3,PIN12, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ERXD2                      (gpio_pinset_t){ PORT6,PIN11, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_ERXD3                      (gpio_pinset_t){ PORT6,PIN12, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_COL                        (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_ET0_RX_DV                      (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_ETHERNET | R_PFS_PMR)}

/* RMII Alternative Functions */
#define GPIO_RMII0_CRS_DV_A                 (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_CRS_DV_B                 (gpio_pinset_t){ PORT7,PIN5, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_RX_ER_A                  (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_RX_ER_B                  (gpio_pinset_t){ PORT7,PIN4, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_RXD1_A                   (gpio_pinset_t){ PORT3,PIN1, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_RXD1_B                   (gpio_pinset_t){ PORT7,PIN3, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_RXD0_A                   (gpio_pinset_t){ PORT3,PIN2, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_RXD0_B                   (gpio_pinset_t){ PORT7,PIN2, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_TXD0_A                   (gpio_pinset_t){ PORT3,PIN4, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_TXD0_B                   (gpio_pinset_t){ PORT7,PIN0, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_TXD1_A                   (gpio_pinset_t){ PORT3,PIN5, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_TXD1_B                   (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_TXD_EN_A                 (gpio_pinset_t){ PORT3,PIN6, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_RMII0_TXD_EN_B                 (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_REF50CK0_A                     (gpio_pinset_t){ PORT3,PIN3, (PFS_PSEL_ETHERNET | R_PFS_PMR)}
#define GPIO_REF50CK0_B                     (gpio_pinset_t){ PORT7,PIN1, (PFS_PSEL_ETHERNET | R_PFS_PMR)}

/* Audio Clock Alternative Functions */
#define GPIO_AUDIO_CLK_1                    (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}
#define GPIO_AUDIO_CLK_2                    (gpio_pinset_t){ PORT4,PIN2, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}
#define GPIO_AUDIO_CLK_3                    (gpio_pinset_t){ PORT7,PIN8, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}

/* SSI (Synchronous Serial Interface) Alternative Functions */
#define GPIO_SSIBCK0_A                      (gpio_pinset_t){ PORT4,PIN3, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIBCK0_B                      (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSILRCK0_A                     (gpio_pinset_t){ PORT4,PIN4, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSILRCK0_B                     (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSITXD0_A                      (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSITXD0_B                      (gpio_pinset_t){ PORT1,PIN15, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIRXD0_A                      (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIRXD0_B                      (gpio_pinset_t){ PORT1,PIN14, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIBCK1_A                      (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIBCK1_B                      (gpio_pinset_t){ PORT7,PIN2, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSILRCK1_A                     (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSILRCK1_B                     (gpio_pinset_t){ PORT7,PIN1, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIDATA1_A                     (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_SSIE | R_PFS_PMR)}
#define GPIO_SSIDATA1_B                     (gpio_pinset_t){ PORT7,PIN0, (PFS_PSEL_SSIE | R_PFS_PMR)}

/* DAC Alternative Functions */
#define GPIO_DA0                            (gpio_pinset_t){ PORT0,PIN14, (PFS_PSEL_DAC | R_PFS_PMR)}

/* RTC Alternative Functions */
#define GPIO_RTCOUT_1                       (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}
#define GPIO_RTCOUT_2                       (gpio_pinset_t){ PORT6,PIN1, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}
#define GPIO_RTCIC0                         (gpio_pinset_t){ PORT4,PIN2, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}
#define GPIO_RTCIC1                         (gpio_pinset_t){ PORT4,PIN3, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}
#define GPIO_RTCIC2                         (gpio_pinset_t){ PORT4,PIN4, (PFS_PSEL_CLKOUT_ACMPLP_RTC | R_PFS_PMR)}

/* Comparator Alternative Functions */
#define GPIO_IVCMP0                         (gpio_pinset_t){ PORT0,PIN10, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_IVCMP2_1                       (gpio_pinset_t){ PORT0,PIN0, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_IVCMP2_2                       (gpio_pinset_t){ PORT0,PIN4, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_IVCMP3_1                       (gpio_pinset_t){ PORT0,PIN2, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_IVCMP3_2                       (gpio_pinset_t){ PORT0,PIN6, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_IVREF0                         (gpio_pinset_t){ PORT0,PIN1, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_IVREF1                         (gpio_pinset_t){ PORT0,PIN3, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_VCOUT_1                        (gpio_pinset_t){ PORT2,PIN8, (PFS_PSEL_OPAMP | R_PFS_PMR)}
#define GPIO_VCOUT_2                        (gpio_pinset_t){ PORT7,PIN3, (PFS_PSEL_OPAMP | R_PFS_PMR)}

/* AGT (Asynchronous General Purpose Timer) Alternative Functions */
#define GPIO_AGTIO0_1                       (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTIO0_2                       (gpio_pinset_t){ PORT3,PIN1, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTIO0_3                       (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTIO0_4                       (gpio_pinset_t){ PORT7,PIN5, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTIO1_1                       (gpio_pinset_t){ PORT2,PIN4, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTIO1_2                       (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTIO1_3                       (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTO0_1                        (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTO0_2                        (gpio_pinset_t){ PORT6,PIN14, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTO0_3                        (gpio_pinset_t){ PORT7,PIN4, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTO1_1                        (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTO1_2                        (gpio_pinset_t){ PORT6,PIN13, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTO1_3                        (gpio_pinset_t){ PORT7,PIN3, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTEE0_1                       (gpio_pinset_t){ PORT1,PIN1, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTEE0_2                       (gpio_pinset_t){ PORT7,PIN11, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTEE1_1                       (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTEE1_2                       (gpio_pinset_t){ PORT3,PIN10, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTEE1_3                       (gpio_pinset_t){ PORT4,PIN12, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOA0_1                       (gpio_pinset_t){ PORT1,PIN7, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOA0_2                       (gpio_pinset_t){ PORT7,PIN13, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOA0_3                       (gpio_pinset_t){ PORT8,PIN0, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOA1_1                       (gpio_pinset_t){ PORT3,PIN12, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOA1_2                       (gpio_pinset_t){ PORT4,PIN11, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOB0_1                       (gpio_pinset_t){ PORT1,PIN6, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOB0_2                       (gpio_pinset_t){ PORT7,PIN12, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOB0_3                       (gpio_pinset_t){ PORT8,PIN1, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOB1_1                       (gpio_pinset_t){ PORT3,PIN8, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOB1_2                       (gpio_pinset_t){ PORT3,PIN11, (PFS_PSEL_AGT | R_PFS_PMR)}
#define GPIO_AGTOB1_3                       (gpio_pinset_t){ PORT4,PIN10, (PFS_PSEL_AGT | R_PFS_PMR)}

/* GPT Trigger Functions */
#define GPIO_GTETRGA_1                      (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGA_2                      (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGA_3                      (gpio_pinset_t){ PORT6,PIN13, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGB_1                      (gpio_pinset_t){ PORT1,PIN1, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGB_2                      (gpio_pinset_t){ PORT1,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGB_3                      (gpio_pinset_t){ PORT6,PIN14, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGC_1                      (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGC_2                      (gpio_pinset_t){ PORT8,PIN3, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGD_1                      (gpio_pinset_t){ PORT2,PIN12, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTETRGD_2                      (gpio_pinset_t){ PORT8,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}

/* ADC Trigger Functions */
#define GPIO_ADTRG0_1                       (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_CAC_ADC14 | R_PFS_PMR)}
#define GPIO_ADTRG0_2                       (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_CAC_ADC14 | R_PFS_PMR)}
#define GPIO_ADTRG1_1                       (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_CAC_ADC14 | R_PFS_PMR)}
#define GPIO_ADTRG1_2                       (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_CAC_ADC14 | R_PFS_PMR)}

/* ULPT (Ultra-Low-Power Timer) Alternative Functions */
#define GPIO_ULPTOA0_1                      (gpio_pinset_t){ PORT1,PIN13, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOA0_2                      (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOA1_1                      (gpio_pinset_t){ PORT2,PIN3, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOA1_2                      (gpio_pinset_t){ PORT3,PIN7, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOA1_3                      (gpio_pinset_t){ PORT6,PIN9, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOB0_1                      (gpio_pinset_t){ PORT1,PIN12, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOB0_2                      (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOB1_1                      (gpio_pinset_t){ PORT2,PIN2, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOB1_2                      (gpio_pinset_t){ PORT3,PIN8, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTOB1_3                      (gpio_pinset_t){ PORT6,PIN10, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTO0_1                       (gpio_pinset_t){ PORT3,PIN2, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTO0_2                       (gpio_pinset_t){ PORT6,PIN3, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTO0_3                       (gpio_pinset_t){ PORT7,PIN2, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTO1_1                       (gpio_pinset_t){ PORT1,PIN5, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTO1_2                       (gpio_pinset_t){ PORT3,PIN4, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTO1_3                       (gpio_pinset_t){ PORT7,PIN1, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEE0_1                      (gpio_pinset_t){ PORT2,PIN13, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEE0_2                      (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEE0_3                      (gpio_pinset_t){ PORT6,PIN2, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEE1_1                      (gpio_pinset_t){ PORT1,PIN6, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEE1_2                      (gpio_pinset_t){ PORT3,PIN5, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEE1_3                      (gpio_pinset_t){ PORT4,PIN13, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEVI0_1                     (gpio_pinset_t){ PORT3,PIN0, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEVI0_2                     (gpio_pinset_t){ PORT6,PIN1, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEVI1_1                     (gpio_pinset_t){ PORT3,PIN6, (PFS_PSEL_ULPT | R_PFS_PMR)}
#define GPIO_ULPTEVI1_2                     (gpio_pinset_t){ PORT6,PIN0, (PFS_PSEL_ULPT | R_PFS_PMR)}

/* OctaRAM/OSPI Memory Interface Functions */
#define GPIO_OM_CS0                         (gpio_pinset_t){ PORT1,PIN7, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_CS1                         (gpio_pinset_t){ PORT1,PIN4, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_RESET                       (gpio_pinset_t){ PORT1,PIN6, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO0                        (gpio_pinset_t){ PORT1,PIN0, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO1                        (gpio_pinset_t){ PORT8,PIN3, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO2                        (gpio_pinset_t){ PORT1,PIN3, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO3                        (gpio_pinset_t){ PORT1,PIN1, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO4                        (gpio_pinset_t){ PORT1,PIN2, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO5                        (gpio_pinset_t){ PORT8,PIN0, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO6                        (gpio_pinset_t){ PORT8,PIN2, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SIO7                        (gpio_pinset_t){ PORT8,PIN4, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SCLK                        (gpio_pinset_t){ PORT8,PIN8, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_SCLKN                       (gpio_pinset_t){ PORT8,PIN9, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_DQS                         (gpio_pinset_t){ PORT8,PIN1, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_WP1                         (gpio_pinset_t){ PORT6,PIN1, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_RSTO1                       (gpio_pinset_t){ PORT6,PIN0, (PFS_PSEL_XSPI | R_PFS_PMR)}
#define GPIO_OM_ECSINT1                     (gpio_pinset_t){ PORT1,PIN5, (PFS_PSEL_XSPI | R_PFS_PMR)}

/* VIO (Video Input/Output) Alternative Functions */
#define GPIO_VIO_CLK                        (gpio_pinset_t){ PORT7,PIN8, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_HD                         (gpio_pinset_t){ PORT7,PIN9, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_VD                         (gpio_pinset_t){ PORT7,PIN10, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_FLD                        (gpio_pinset_t){ PORT5,PIN13, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D0                         (gpio_pinset_t){ PORT4,PIN0, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D1                         (gpio_pinset_t){ PORT4,PIN1, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D2                         (gpio_pinset_t){ PORT4,PIN5, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D3_1                       (gpio_pinset_t){ PORT4,PIN4, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D3_2                       (gpio_pinset_t){ PORT4,PIN6, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D4                         (gpio_pinset_t){ PORT7,PIN0, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D5                         (gpio_pinset_t){ PORT7,PIN1, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D6                         (gpio_pinset_t){ PORT7,PIN2, (PFS_PSEL_CEU | R_PFS_PMR)}
#define GPIO_VIO_D7                         (gpio_pinset_t){ PORT7,PIN3, (PFS_PSEL_CEU | R_PFS_PMR)}

/* GPT Delta-Sigma Modulator Functions */
#define GPIO_GTADSM0_1                      (gpio_pinset_t){ PORT3,PIN12, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTADSM0_2                      (gpio_pinset_t){ PORT4,PIN15, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTADSM0_3                      (gpio_pinset_t){ PORT7,PIN4, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTADSM1_1                      (gpio_pinset_t){ PORT3,PIN11, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTADSM1_2                      (gpio_pinset_t){ PORT4,PIN14, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_GTADSM1_3                      (gpio_pinset_t){ PORT7,PIN5, (PFS_PSEL_GPT | R_PFS_PMR)}
#define GPIO_SDA1_MOSI1_TXD1_A              (gpio_pinset_t){ PORT2,PIN5, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SDA1_MOSI1_TXD1_B              (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SDA1_MOSI1_TXD1_C              (gpio_pinset_t){ PORT5,PIN11, (PFS_PSEL_IIC | R_PFS_PMR)}

#define GPIO_SCL1_MISO1_RXD1_A              (gpio_pinset_t){ PORT2,PIN6, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL1_MISO1_RXD1_B              (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL1_MISO1_RXD1_C              (gpio_pinset_t){ PORT5,PIN12, (PFS_PSEL_IIC | R_PFS_PMR)}

#define GPIO_SDA0_A                         (gpio_pinset_t){ PORT4,PIN9, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SDA0_B                         (gpio_pinset_t){ PORT4,PIN7, (PFS_PSEL_IIC | R_PFS_PMR)}

#define GPIO_SCL0_A                         (gpio_pinset_t){ PORT4,PIN10, (PFS_PSEL_IIC | R_PFS_PMR)}
#define GPIO_SCL0_B                         (gpio_pinset_t){ PORT4,PIN8, (PFS_PSEL_IIC | R_PFS_PMR)}

/* ADC Alternative Functions */
#define GPIO_AN000_1                        (gpio_pinset_t){ PORT0,PIN4, (0 | R_PFS_ASEL)}
#define GPIO_AN001_1                        (gpio_pinset_t){ PORT0,PIN5, (0 | R_PFS_ASEL)}
#define GPIO_AN002_1                        (gpio_pinset_t){ PORT0,PIN6, (0 | R_PFS_ASEL)}
#define GPIO_AN004_1                        (gpio_pinset_t){ PORT0,PIN7, (0 | R_PFS_ASEL)}
#define GPIO_AN005_1                        (gpio_pinset_t){ PORT0,PIN10, (0 | R_PFS_ASEL)}
#define GPIO_AN006_1                        (gpio_pinset_t){ PORT0,PIN9, (0 | R_PFS_ASEL)}
#define GPIO_AN007_1                        (gpio_pinset_t){ PORT0,PIN14, (0 | R_PFS_ASEL)}
#define GPIO_AN008_1                        (gpio_pinset_t){ PORT0,PIN8, (0 | R_PFS_ASEL)}
#define GPIO_AN100_1                        (gpio_pinset_t){ PORT0,PIN0, (0 | R_PFS_ASEL)}
#define GPIO_AN101_1                        (gpio_pinset_t){ PORT0,PIN1, (0 | R_PFS_ASEL)}
#define GPIO_AN102_1                        (gpio_pinset_t){ PORT0,PIN2, (0 | R_PFS_ASEL)}
#define GPIO_AN104_1                        (gpio_pinset_t){ PORT0,PIN3, (0 | R_PFS_ASEL)}
#define GPIO_AN105_1                        (gpio_pinset_t){ PORT0,PIN15, (0 | R_PFS_ASEL)}

/* General Purpose GPIO Pin Definitions */
#define GPIO_P004_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P004_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P004_INPUT                     (gpio_pinset_t){ PORT0, PIN4, (GPIO_INPUT)}
#define GPIO_P004_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P006_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P006_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P006_INPUT                     (gpio_pinset_t){ PORT0, PIN6, (GPIO_INPUT)}
#define GPIO_P006_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN6, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P006_INT_RISING                (gpio_pinset_t){ PORT0, PIN6, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_P006_INT_FALLING               (gpio_pinset_t){ PORT0, PIN6, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL | R_PFS_EOF)}

#define GPIO_P009_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P009_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P009_INPUT                     (gpio_pinset_t){ PORT0, PIN9, (GPIO_INPUT)}
#define GPIO_P009_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN9, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P009_INT_FALLING               (gpio_pinset_t){ PORT0, PIN9, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL | R_PFS_EOF)}

#define GPIO_P303_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P303_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P303_INPUT                     (gpio_pinset_t){ PORT3, PIN3, (GPIO_INPUT)}
#define GPIO_P303_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P404_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P404_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P404_INPUT                     (gpio_pinset_t){ PORT4, PIN4, (GPIO_INPUT)}

#define GPIO_P408_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P408_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P408_INPUT                     (gpio_pinset_t){ PORT4, PIN8, (GPIO_INPUT)}

#define GPIO_P410_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P410_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P410_INPUT                     (gpio_pinset_t){ PORT4, PIN10, (GPIO_INPUT)}

#define GPIO_P411_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P411_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P411_INPUT                     (gpio_pinset_t){ PORT4, PIN11, (GPIO_INPUT)}

#define GPIO_P412_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P412_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P412_INPUT                     (gpio_pinset_t){ PORT4, PIN12, (GPIO_INPUT)}

#define GPIO_P605_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P605_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P605_INPUT                     (gpio_pinset_t){ PORT6, PIN5, (GPIO_INPUT)}

#define GPIO_P905_OUTPUT_HIGH               (gpio_pinset_t){ PORT9, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P905_OUTPUT_LOW                (gpio_pinset_t){ PORT9, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P905_INPUT                     (gpio_pinset_t){ PORT9, PIN5, (GPIO_INPUT)}

#define GPIO_P113_RXD0_A                    (gpio_pinset_t){ PORT1, PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_P113_MISO0_A                   (gpio_pinset_t){ PORT1, PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_P113_SCL0_A                    (gpio_pinset_t){ PORT1, PIN13, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_P113_SSLA1_B                   (gpio_pinset_t){ PORT1, PIN13, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_P113_SSILRCK0                  (gpio_pinset_t){ PORT1, PIN13, (PFS_PSEL_SSI | R_PFS_PMR)}

/* P114 Alternate Functions: CTS0_RTS0_A/SS0_A, DE0, SSLA0_B, SSIRXD0_B, ET0_LINKSTA */
#define GPIO_P114_CTS0_RTS0_A               (gpio_pinset_t){ PORT1, PIN14, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_P114_SS0_A                     (gpio_pinset_t){ PORT1, PIN14, (PFS_PSEL_SCI | R_PFS_PMR)}
#define GPIO_P114_SSLA0_B                   (gpio_pinset_t){ PORT1, PIN14, (PFS_PSEL_SPI | R_PFS_PMR)}
#define GPIO_P114_SSIRXD0_B                 (gpio_pinset_t){ PORT1, PIN14, (PFS_PSEL_SSI | R_PFS_PMR)}

/* External Interrupt Pin Definitions */
#define GPIO_IRQ0_P105                      (gpio_pinset_t){ PORT1, PIN5, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ0_P206                      (gpio_pinset_t){ PORT2, PIN6, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ0_P400                      (gpio_pinset_t){ PORT4, PIN0, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ0_P806                      (gpio_pinset_t){ PORT8, PIN6, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ1_P101                      (gpio_pinset_t){ PORT1, PIN1, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ1_P104                      (gpio_pinset_t){ PORT1, PIN4, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ1_P205                      (gpio_pinset_t){ PORT2, PIN5, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ2_P100                      (gpio_pinset_t){ PORT1, PIN0, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ2_P203                      (gpio_pinset_t){ PORT2, PIN3, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ2_P212                      (gpio_pinset_t){ PORT2, PIN12, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ3_P202                      (gpio_pinset_t){ PORT2, PIN2, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ3_P208                      (gpio_pinset_t){ PORT2, PIN8, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ3_P211                      (gpio_pinset_t){ PORT2, PIN11, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ4_P300                      (gpio_pinset_t){ PORT3, PIN0, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ4_P402                      (gpio_pinset_t){ PORT4, PIN2, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ4_P411                      (gpio_pinset_t){ PORT4, PIN11, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ5_P302                      (gpio_pinset_t){ PORT3, PIN2, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ5_P401                      (gpio_pinset_t){ PORT4, PIN1, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ5_P410                      (gpio_pinset_t){ PORT4, PIN10, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ6_P301                      (gpio_pinset_t){ PORT3, PIN1, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ6_P409                      (gpio_pinset_t){ PORT4, PIN9, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ7_P408                      (gpio_pinset_t){ PORT4, PIN8, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ8_P305                      (gpio_pinset_t){ PORT3, PIN5, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ8_P415                      (gpio_pinset_t){ PORT4, PIN15, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ9_P304                      (gpio_pinset_t){ PORT3, PIN4, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ9_P414                      (gpio_pinset_t){ PORT4, PIN14, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ10_P709                     (gpio_pinset_t){ PORT7, PIN9, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ11_P708                     (gpio_pinset_t){ PORT7, PIN8, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ11_P800                     (gpio_pinset_t){ PORT8, PIN0, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ12_P801                     (gpio_pinset_t){ PORT8, PIN1, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ13_P009                     (gpio_pinset_t){ PORT0, PIN9, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ13_P014                     (gpio_pinset_t){ PORT0, PIN14, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ14_P010                     (gpio_pinset_t){ PORT0, PIN10, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ14_P403                     (gpio_pinset_t){ PORT4, PIN3, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ14_P512                     (gpio_pinset_t){ PORT5, PIN12, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ14_P804                     (gpio_pinset_t){ PORT8, PIN4, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

#define GPIO_IRQ15_P404                     (gpio_pinset_t){ PORT4, PIN4, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ15_P511                     (gpio_pinset_t){ PORT5, PIN11, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ15_P808                     (gpio_pinset_t){ PORT8, PIN8, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

/* Missing IRQ definitions from CSV */
#define GPIO_IRQ6_P000                      (gpio_pinset_t){ PORT0, PIN0, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ7_P001                      (gpio_pinset_t){ PORT0, PIN1, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ8_P002                      (gpio_pinset_t){ PORT0, PIN2, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ8_P008                      (gpio_pinset_t){ PORT0, PIN8, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ9_P004                      (gpio_pinset_t){ PORT0, PIN4, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ10_P005                     (gpio_pinset_t){ PORT0, PIN5, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ11_P006                     (gpio_pinset_t){ PORT0, PIN6, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ12_P008                     (gpio_pinset_t){ PORT0, PIN8, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ15_P015                     (gpio_pinset_t){ PORT0, PIN15, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

/* IRQ with DS (Deep Standby) capability */
#define GPIO_IRQ1_P205_DS                   (gpio_pinset_t){ PORT2, PIN5, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ2_P203_DS                   (gpio_pinset_t){ PORT2, PIN3, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ3_P202_DS                   (gpio_pinset_t){ PORT2, PIN2, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ4_P402_DS                   (gpio_pinset_t){ PORT4, PIN2, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ5_P401_DS                   (gpio_pinset_t){ PORT4, PIN1, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ14_P403_DS                  (gpio_pinset_t){ PORT4, PIN3, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}
#define GPIO_IRQ15_P404_DS                  (gpio_pinset_t){ PORT4, PIN4, (GPIO_INPUT | R_PFS_PCR | R_PFS_ISEL)}

/* PORT 0 Pin Definitions */
#define GPIO_P000_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P000_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P000_INPUT                     (gpio_pinset_t){ PORT0, PIN0, (GPIO_INPUT)}
#define GPIO_P000_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN0, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P000_ANALOG                    (gpio_pinset_t){ PORT0, PIN0, (0 | R_PFS_ASEL)}

#define GPIO_P001_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P001_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P001_INPUT                     (gpio_pinset_t){ PORT0, PIN1, (GPIO_INPUT)}
#define GPIO_P001_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN1, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P001_ANALOG                    (gpio_pinset_t){ PORT0, PIN1, (0 | R_PFS_ASEL)}

#define GPIO_P002_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P002_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P002_INPUT                     (gpio_pinset_t){ PORT0, PIN2, (GPIO_INPUT)}
#define GPIO_P002_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN2, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P002_ANALOG                    (gpio_pinset_t){ PORT0, PIN2, (0 | R_PFS_ASEL)}

#define GPIO_P003_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P003_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P003_INPUT                     (gpio_pinset_t){ PORT0, PIN3, (GPIO_INPUT)}
#define GPIO_P003_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN3, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P003_ANALOG                    (gpio_pinset_t){ PORT0, PIN3, (0 | R_PFS_ASEL)}

#define GPIO_P005_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P005_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P005_INPUT                     (gpio_pinset_t){ PORT0, PIN5, (GPIO_INPUT)}
#define GPIO_P005_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN5, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P005_ANALOG                    (gpio_pinset_t){ PORT0, PIN5, (0 | R_PFS_ASEL)}

#define GPIO_P007_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P007_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P007_INPUT                     (gpio_pinset_t){ PORT0, PIN7, (GPIO_INPUT)}
#define GPIO_P007_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN7, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P007_ANALOG                    (gpio_pinset_t){ PORT0, PIN7, (0 | R_PFS_ASEL)}

#define GPIO_P008_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P008_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P008_INPUT                     (gpio_pinset_t){ PORT0, PIN8, (GPIO_INPUT)}
#define GPIO_P008_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN8, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P008_ANALOG                    (gpio_pinset_t){ PORT0, PIN8, (0 | R_PFS_ASEL)}

#define GPIO_P010_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P010_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P010_INPUT                     (gpio_pinset_t){ PORT0, PIN10, (GPIO_INPUT)}
#define GPIO_P010_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN10, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P010_ANALOG                    (gpio_pinset_t){ PORT0, PIN10, (0 | R_PFS_ASEL)}

#define GPIO_P014_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P014_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P014_INPUT                     (gpio_pinset_t){ PORT0, PIN14, (GPIO_INPUT)}
#define GPIO_P014_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN14, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P014_ANALOG                    (gpio_pinset_t){ PORT0, PIN14, (0 | R_PFS_ASEL)}

#define GPIO_P015_OUTPUT_HIGH               (gpio_pinset_t){ PORT0, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P015_OUTPUT_LOW                (gpio_pinset_t){ PORT0, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P015_INPUT                     (gpio_pinset_t){ PORT0, PIN15, (GPIO_INPUT)}
#define GPIO_P015_INPUT_PULLUP              (gpio_pinset_t){ PORT0, PIN15, (GPIO_INPUT | R_PFS_PCR)}
#define GPIO_P015_ANALOG                    (gpio_pinset_t){ PORT0, PIN15, (0 | R_PFS_ASEL)}

/* PORT 1 Pin Definitions */
#define GPIO_P100_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P100_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P100_INPUT                     (gpio_pinset_t){ PORT1, PIN0, (GPIO_INPUT)}
#define GPIO_P100_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P101_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P101_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P101_INPUT                     (gpio_pinset_t){ PORT1, PIN1, (GPIO_INPUT)}
#define GPIO_P101_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P102_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P102_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P102_INPUT                     (gpio_pinset_t){ PORT1, PIN2, (GPIO_INPUT)}
#define GPIO_P102_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P103_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P103_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P103_INPUT                     (gpio_pinset_t){ PORT1, PIN3, (GPIO_INPUT)}
#define GPIO_P103_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P104_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P104_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P104_INPUT                     (gpio_pinset_t){ PORT1, PIN4, (GPIO_INPUT)}
#define GPIO_P104_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P105_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P105_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P105_INPUT                     (gpio_pinset_t){ PORT1, PIN5, (GPIO_INPUT)}
#define GPIO_P105_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN5, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P106_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P106_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P106_INPUT                     (gpio_pinset_t){ PORT1, PIN6, (GPIO_INPUT)}
#define GPIO_P106_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN6, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P107_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P107_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P107_INPUT                     (gpio_pinset_t){ PORT1, PIN7, (GPIO_INPUT)}
#define GPIO_P107_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN7, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P112_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P112_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P112_INPUT                     (gpio_pinset_t){ PORT1, PIN12, (GPIO_INPUT)}
#define GPIO_P112_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P113_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P113_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P113_INPUT                     (gpio_pinset_t){ PORT1, PIN13, (GPIO_INPUT)}
#define GPIO_P113_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN13, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P114_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P114_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P114_INPUT                     (gpio_pinset_t){ PORT1, PIN14, (GPIO_INPUT)}
#define GPIO_P114_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN14, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P115_OUTPUT_HIGH               (gpio_pinset_t){ PORT1, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P115_OUTPUT_LOW                (gpio_pinset_t){ PORT1, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P115_INPUT                     (gpio_pinset_t){ PORT1, PIN15, (GPIO_INPUT)}
#define GPIO_P115_INPUT_PULLUP              (gpio_pinset_t){ PORT1, PIN15, (GPIO_INPUT | R_PFS_PCR)}

/* PORT 2 Pin Definitions */
#define GPIO_P200_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P200_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P200_INPUT                     (gpio_pinset_t){ PORT2, PIN0, (GPIO_INPUT)}
#define GPIO_P200_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P201_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P201_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P201_INPUT                     (gpio_pinset_t){ PORT2, PIN1, (GPIO_INPUT)}
#define GPIO_P201_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P202_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P202_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P202_INPUT                     (gpio_pinset_t){ PORT2, PIN2, (GPIO_INPUT)}
#define GPIO_P202_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P203_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P203_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P203_INPUT                     (gpio_pinset_t){ PORT2, PIN3, (GPIO_INPUT)}
#define GPIO_P203_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P204_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P204_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P204_INPUT                     (gpio_pinset_t){ PORT2, PIN4, (GPIO_INPUT)}
#define GPIO_P204_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P205_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P205_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P205_INPUT                     (gpio_pinset_t){ PORT2, PIN5, (GPIO_INPUT)}
#define GPIO_P205_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN5, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P206_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P206_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P206_INPUT                     (gpio_pinset_t){ PORT2, PIN6, (GPIO_INPUT)}
#define GPIO_P206_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN6, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P208_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P208_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P208_INPUT                     (gpio_pinset_t){ PORT2, PIN8, (GPIO_INPUT)}
#define GPIO_P208_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN8, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P209_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P209_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P209_INPUT                     (gpio_pinset_t){ PORT2, PIN9, (GPIO_INPUT)}
#define GPIO_P209_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN9, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P210_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P210_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P210_INPUT                     (gpio_pinset_t){ PORT2, PIN10, (GPIO_INPUT)}
#define GPIO_P210_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN10, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P211_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P211_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P211_INPUT                     (gpio_pinset_t){ PORT2, PIN11, (GPIO_INPUT)}
#define GPIO_P211_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN11, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P212_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P212_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P212_INPUT                     (gpio_pinset_t){ PORT2, PIN12, (GPIO_INPUT)}
#define GPIO_P212_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P213_OUTPUT_HIGH               (gpio_pinset_t){ PORT2, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P213_OUTPUT_LOW                (gpio_pinset_t){ PORT2, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P213_INPUT                     (gpio_pinset_t){ PORT2, PIN13, (GPIO_INPUT)}
#define GPIO_P213_INPUT_PULLUP              (gpio_pinset_t){ PORT2, PIN13, (GPIO_INPUT | R_PFS_PCR)}

/* PORT 3 Pin Definitions */
#define GPIO_P300_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P300_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P300_INPUT                     (gpio_pinset_t){ PORT3, PIN0, (GPIO_INPUT)}
#define GPIO_P300_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P301_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P301_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P301_INPUT                     (gpio_pinset_t){ PORT3, PIN1, (GPIO_INPUT)}
#define GPIO_P301_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P302_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P302_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P302_INPUT                     (gpio_pinset_t){ PORT3, PIN2, (GPIO_INPUT)}
#define GPIO_P302_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P304_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P304_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P304_INPUT                     (gpio_pinset_t){ PORT3, PIN4, (GPIO_INPUT)}
#define GPIO_P304_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P305_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P305_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P305_INPUT                     (gpio_pinset_t){ PORT3, PIN5, (GPIO_INPUT)}
#define GPIO_P305_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN5, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P306_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P306_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P306_INPUT                     (gpio_pinset_t){ PORT3, PIN6, (GPIO_INPUT)}
#define GPIO_P306_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN6, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P307_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P307_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P307_INPUT                     (gpio_pinset_t){ PORT3, PIN7, (GPIO_INPUT)}
#define GPIO_P307_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN7, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P308_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P308_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P308_INPUT                     (gpio_pinset_t){ PORT3, PIN8, (GPIO_INPUT)}
#define GPIO_P308_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN8, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P309_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P309_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P309_INPUT                     (gpio_pinset_t){ PORT3, PIN9, (GPIO_INPUT)}
#define GPIO_P309_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN9, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P310_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P310_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P310_INPUT                     (gpio_pinset_t){ PORT3, PIN10, (GPIO_INPUT)}
#define GPIO_P310_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN10, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P311_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P311_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P311_INPUT                     (gpio_pinset_t){ PORT3, PIN11, (GPIO_INPUT)}
#define GPIO_P311_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN11, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P312_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P312_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P312_INPUT                     (gpio_pinset_t){ PORT3, PIN12, (GPIO_INPUT)}
#define GPIO_P312_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P313_OUTPUT_HIGH               (gpio_pinset_t){ PORT3, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P313_OUTPUT_LOW                (gpio_pinset_t){ PORT3, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P313_INPUT                     (gpio_pinset_t){ PORT3, PIN13, (GPIO_INPUT)}
#define GPIO_P313_INPUT_PULLUP              (gpio_pinset_t){ PORT3, PIN13, (GPIO_INPUT | R_PFS_PCR)}

/* Additional PORT 4, 5, 6, 7, 8, 9 Pin Definitions */
#define GPIO_P400_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P400_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P400_INPUT                     (gpio_pinset_t){ PORT4, PIN0, (GPIO_INPUT)}
#define GPIO_P400_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P401_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P401_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P401_INPUT                     (gpio_pinset_t){ PORT4, PIN1, (GPIO_INPUT)}
#define GPIO_P401_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P402_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P402_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P402_INPUT                     (gpio_pinset_t){ PORT4, PIN2, (GPIO_INPUT)}
#define GPIO_P402_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P403_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P403_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P403_INPUT                     (gpio_pinset_t){ PORT4, PIN3, (GPIO_INPUT)}
#define GPIO_P403_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P405_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P405_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P405_INPUT                     (gpio_pinset_t){ PORT4, PIN5, (GPIO_INPUT)}
#define GPIO_P405_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN5, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P406_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P406_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P406_INPUT                     (gpio_pinset_t){ PORT4, PIN6, (GPIO_INPUT)}
#define GPIO_P406_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN6, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P407_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P407_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN7, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P407_INPUT                     (gpio_pinset_t){ PORT4, PIN7, (GPIO_INPUT)}
#define GPIO_P407_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN7, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P409_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P409_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P409_INPUT                     (gpio_pinset_t){ PORT4, PIN9, (GPIO_INPUT)}
#define GPIO_P409_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN9, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P410_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P410_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P410_INPUT                     (gpio_pinset_t){ PORT4, PIN10, (GPIO_INPUT)}
#define GPIO_P410_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN10, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P411_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P411_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P411_INPUT                     (gpio_pinset_t){ PORT4, PIN11, (GPIO_INPUT)}
#define GPIO_P411_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN11, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P412_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P412_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P412_INPUT                     (gpio_pinset_t){ PORT4, PIN12, (GPIO_INPUT)}
#define GPIO_P412_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P413_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P413_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P413_INPUT                     (gpio_pinset_t){ PORT4, PIN13, (GPIO_INPUT)}
#define GPIO_P413_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN13, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P414_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P414_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P414_INPUT                     (gpio_pinset_t){ PORT4, PIN14, (GPIO_INPUT)}
#define GPIO_P414_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN14, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P415_OUTPUT_HIGH               (gpio_pinset_t){ PORT4, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P415_OUTPUT_LOW                (gpio_pinset_t){ PORT4, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P415_INPUT                     (gpio_pinset_t){ PORT4, PIN15, (GPIO_INPUT)}
#define GPIO_P415_INPUT_PULLUP              (gpio_pinset_t){ PORT4, PIN15, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P511_OUTPUT_HIGH               (gpio_pinset_t){ PORT5, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P511_OUTPUT_LOW                (gpio_pinset_t){ PORT5, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P511_INPUT                     (gpio_pinset_t){ PORT5, PIN11, (GPIO_INPUT)}
#define GPIO_P511_INPUT_PULLUP              (gpio_pinset_t){ PORT5, PIN11, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P512_OUTPUT_HIGH               (gpio_pinset_t){ PORT5, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P512_OUTPUT_LOW                (gpio_pinset_t){ PORT5, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P512_INPUT                     (gpio_pinset_t){ PORT5, PIN12, (GPIO_INPUT)}
#define GPIO_P512_INPUT_PULLUP              (gpio_pinset_t){ PORT5, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P513_OUTPUT_HIGH               (gpio_pinset_t){ PORT5, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P513_OUTPUT_LOW                (gpio_pinset_t){ PORT5, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P513_INPUT                     (gpio_pinset_t){ PORT5, PIN13, (GPIO_INPUT)}
#define GPIO_P513_INPUT_PULLUP              (gpio_pinset_t){ PORT5, PIN13, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P600_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P600_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P600_INPUT                     (gpio_pinset_t){ PORT6, PIN0, (GPIO_INPUT)}
#define GPIO_P600_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P601_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P601_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P601_INPUT                     (gpio_pinset_t){ PORT6, PIN1, (GPIO_INPUT)}
#define GPIO_P601_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P602_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P602_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P602_INPUT                     (gpio_pinset_t){ PORT6, PIN2, (GPIO_INPUT)}
#define GPIO_P602_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P603_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P603_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P603_INPUT                     (gpio_pinset_t){ PORT6, PIN3, (GPIO_INPUT)}
#define GPIO_P603_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P604_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P604_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P604_INPUT                     (gpio_pinset_t){ PORT6, PIN4, (GPIO_INPUT)}
#define GPIO_P604_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P609_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P609_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P609_INPUT                     (gpio_pinset_t){ PORT6, PIN9, (GPIO_INPUT)}
#define GPIO_P609_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN9, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P610_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P610_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P610_INPUT                     (gpio_pinset_t){ PORT6, PIN10, (GPIO_INPUT)}
#define GPIO_P610_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN10, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P611_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P611_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P611_INPUT                     (gpio_pinset_t){ PORT6, PIN11, (GPIO_INPUT)}
#define GPIO_P611_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN11, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P612_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P612_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P612_INPUT                     (gpio_pinset_t){ PORT6, PIN12, (GPIO_INPUT)}
#define GPIO_P612_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P613_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P613_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P613_INPUT                     (gpio_pinset_t){ PORT6, PIN13, (GPIO_INPUT)}
#define GPIO_P613_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN13, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P614_OUTPUT_HIGH               (gpio_pinset_t){ PORT6, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P614_OUTPUT_LOW                (gpio_pinset_t){ PORT6, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P614_INPUT                     (gpio_pinset_t){ PORT6, PIN14, (GPIO_INPUT)}
#define GPIO_P614_INPUT_PULLUP              (gpio_pinset_t){ PORT6, PIN14, (GPIO_INPUT | R_PFS_PCR)}

/* PORT 7 Pin Definitions */
#define GPIO_P700_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P700_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P700_INPUT                     (gpio_pinset_t){ PORT7, PIN0, (GPIO_INPUT)}
#define GPIO_P700_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P701_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P701_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P701_INPUT                     (gpio_pinset_t){ PORT7, PIN1, (GPIO_INPUT)}
#define GPIO_P701_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P702_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P702_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P702_INPUT                     (gpio_pinset_t){ PORT7, PIN2, (GPIO_INPUT)}
#define GPIO_P702_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P703_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P703_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P703_INPUT                     (gpio_pinset_t){ PORT7, PIN3, (GPIO_INPUT)}
#define GPIO_P703_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P704_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P704_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P704_INPUT                     (gpio_pinset_t){ PORT7, PIN4, (GPIO_INPUT)}
#define GPIO_P704_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P705_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P705_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P705_INPUT                     (gpio_pinset_t){ PORT7, PIN5, (GPIO_INPUT)}
#define GPIO_P705_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN5, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P708_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P708_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P708_INPUT                     (gpio_pinset_t){ PORT7, PIN8, (GPIO_INPUT)}
#define GPIO_P708_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN8, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P709_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P709_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P709_INPUT                     (gpio_pinset_t){ PORT7, PIN9, (GPIO_INPUT)}
#define GPIO_P709_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN9, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P710_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P710_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN10, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P710_INPUT                     (gpio_pinset_t){ PORT7, PIN10, (GPIO_INPUT)}
#define GPIO_P710_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN10, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P711_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P711_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN11, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P711_INPUT                     (gpio_pinset_t){ PORT7, PIN11, (GPIO_INPUT)}
#define GPIO_P711_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN11, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P712_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P712_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN12, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P712_INPUT                     (gpio_pinset_t){ PORT7, PIN12, (GPIO_INPUT)}
#define GPIO_P712_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN12, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P713_OUTPUT_HIGH               (gpio_pinset_t){ PORT7, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P713_OUTPUT_LOW                (gpio_pinset_t){ PORT7, PIN13, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P713_INPUT                     (gpio_pinset_t){ PORT7, PIN13, (GPIO_INPUT)}
#define GPIO_P713_INPUT_PULLUP              (gpio_pinset_t){ PORT7, PIN13, (GPIO_INPUT | R_PFS_PCR)}

/* PORT 8 Pin Definitions */
#define GPIO_P800_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P800_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN0, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P800_INPUT                     (gpio_pinset_t){ PORT8, PIN0, (GPIO_INPUT)}
#define GPIO_P800_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN0, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P801_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P801_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN1, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P801_INPUT                     (gpio_pinset_t){ PORT8, PIN1, (GPIO_INPUT)}
#define GPIO_P801_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN1, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P802_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P802_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN2, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P802_INPUT                     (gpio_pinset_t){ PORT8, PIN2, (GPIO_INPUT)}
#define GPIO_P802_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN2, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P803_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P803_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN3, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P803_INPUT                     (gpio_pinset_t){ PORT8, PIN3, (GPIO_INPUT)}
#define GPIO_P803_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN3, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P804_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P804_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN4, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P804_INPUT                     (gpio_pinset_t){ PORT8, PIN4, (GPIO_INPUT)}
#define GPIO_P804_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN4, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P805_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P805_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN5, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P805_INPUT                     (gpio_pinset_t){ PORT8, PIN5, (GPIO_INPUT)}
#define GPIO_P805_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN5, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P806_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P806_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN6, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P806_INPUT                     (gpio_pinset_t){ PORT8, PIN6, (GPIO_INPUT)}
#define GPIO_P806_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN6, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P808_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P808_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN8, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P808_INPUT                     (gpio_pinset_t){ PORT8, PIN8, (GPIO_INPUT)}
#define GPIO_P808_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN8, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P809_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P809_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN9, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P809_INPUT                     (gpio_pinset_t){ PORT8, PIN9, (GPIO_INPUT)}
#define GPIO_P809_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN9, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P814_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P814_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN14, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P814_INPUT                     (gpio_pinset_t){ PORT8, PIN14, (GPIO_INPUT)}
#define GPIO_P814_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN14, (GPIO_INPUT | R_PFS_PCR)}

#define GPIO_P815_OUTPUT_HIGH               (gpio_pinset_t){ PORT8, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_HIGH)}
#define GPIO_P815_OUTPUT_LOW                (gpio_pinset_t){ PORT8, PIN15, (GPIO_OUTPUT | GPIO_LOW_DRIVE | GPIO_OUTPUT_LOW)}
#define GPIO_P815_INPUT                     (gpio_pinset_t){ PORT8, PIN15, (GPIO_INPUT)}
#define GPIO_P815_INPUT_PULLUP              (gpio_pinset_t){ PORT8, PIN15, (GPIO_INPUT | R_PFS_PCR)}

/* GPIO Configuration */

#define GPIO_OUTPUT               R_PFS_PDR
#define GPIO_INPUT               ~(R_PFS_PDR | 0xFFFFFFFF)

#define GPIO_LOW_DRIVE          ~(R_PFS_DSCR | 0xFFFFFFFF)
#define GPIO_MIDDLE_DRIVE       R_PFS_DSCR

#define GPIO_OUTPUT_HIGH         R_PFS_PODR
#define GPIO_OUTPUT_LOW         ~(R_PFS_PODR | 0xFFFFFFFF)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA8E1_PINMAP_H */
