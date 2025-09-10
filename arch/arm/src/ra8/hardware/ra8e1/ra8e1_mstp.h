/****************************************************************************
 * arch/arm/src/ra8/hardware/ra8e1_mstp.h
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

#ifndef __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8_MSTP_H
#define __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8_MSTP_H

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

#define R_MSTP_MSTPCRA_OFFSET           0x0000  /* Module Stop Control Register A (32-bits) */
#define R_MSTP_MSTPCRB_OFFSET           0x0004  /* Module Stop Control Register B (32-bits) */
#define R_MSTP_MSTPCRC_OFFSET           0x0008  /* Module Stop Control Register C (32-bits) */
#define R_MSTP_MSTPCRD_OFFSET           0x000C  /* Module Stop Control Register D (32-bits) */
#define R_MSTP_MSTPCRE_OFFSET           0x0010  /* Module Stop Control Register E (32-bits) */

/* Register Addresses *******************************************************/

#define R_MSTP_MSTPCRA                 (R_MSTP_BASE   + R_MSTP_MSTPCRA_OFFSET)
#define R_MSTP_MSTPCRB                 (R_MSTP_BASE   + R_MSTP_MSTPCRB_OFFSET)
#define R_MSTP_MSTPCRC                 (R_MSTP_BASE   + R_MSTP_MSTPCRC_OFFSET)
#define R_MSTP_MSTPCRD                 (R_MSTP_BASE   + R_MSTP_MSTPCRD_OFFSET)
#define R_MSTP_MSTPCRE                 (R_MSTP_BASE   + R_MSTP_MSTPCRE_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* MSTPCRA - Module Stop Control Register A */

#define R_MSTP_MSTPCRA_MSTPA0          (1 <<  0) /* 00000001: Unnecessary Circuit Module Stop */
#define R_MSTP_MSTPCRA_MSTPA1          (1 <<  1) /* 00000002: SRAM1 Module Stop */
#define R_MSTP_MSTPCRA_MSTPA15         (1 << 15) /* 00008000: Standby SRAM Module Stop */
#define R_MSTP_MSTPCRA_MSTPA22         (1 << 22) /* 00400000: DMA Controller and Data Transfer Controller Module Stop */

/* MSTPCRB - Module Stop Control Register B */

#define R_MSTP_MSTPCRB_MSTPB31         (1 << 31) /* 80000000: Serial Communication Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB30         (1 << 30) /* 40000000: Serial Communication Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB29         (1 << 29) /* 20000000: Serial Communication Interface 2 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB28         (1 << 28) /* 10000000: Serial Communication Interface 3 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB27         (1 << 27) /* 08000000: Serial Communication Interface 4 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB22         (1 << 22) /* 00400000: Serial Communication Interface 9 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB19         (1 << 19) /* 00080000: Serial Peripheral Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB18         (1 << 18) /* 00040000: Serial Peripheral Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB16         (1 << 16) /* 00010000: SCI Communication Interface 10 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB15         (1 << 15) /* 00008000: EtherCAT® Module Stop */
#define R_MSTP_MSTPCRB_MSTPB11         (1 << 11) /* 00000800: Universal Serial Bus 2.0 FS Interface Module Stop */
#define R_MSTP_MSTPCRB_MSTPB9          (1 <<  9) /* 00000200: I2C Bus Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_MSTPB8          (1 <<  8) /* 00000100: I2C Bus Interface 1 Module Stop */

/* Compatibility aliases */
#define R_MSTP_MSTPCRB_SCI0            R_MSTP_MSTPCRB_MSTPB31
#define R_MSTP_MSTPCRB_SCI1            R_MSTP_MSTPCRB_MSTPB30
#define R_MSTP_MSTPCRB_SCI2            R_MSTP_MSTPCRB_MSTPB29
#define R_MSTP_MSTPCRB_SCI3            R_MSTP_MSTPCRB_MSTPB28
#define R_MSTP_MSTPCRB_SCI4            R_MSTP_MSTPCRB_MSTPB27
#define R_MSTP_MSTPCRB_SCI9            R_MSTP_MSTPCRB_MSTPB22
#define R_MSTP_MSTPCRB_SPI0            R_MSTP_MSTPCRB_MSTPB19
#define R_MSTP_MSTPCRB_SPI1            R_MSTP_MSTPCRB_MSTPB18
#define R_MSTP_MSTPCRB_USBFS           R_MSTP_MSTPCRB_MSTPB11
#define R_MSTP_MSTPCRB_IIC0            R_MSTP_MSTPCRB_MSTPB9
#define R_MSTP_MSTPCRB_IIC1            R_MSTP_MSTPCRB_MSTPB8

/* MSTPCRC - Module Stop Control Register C */

#define R_MSTP_MSTPCRC_MSTPC31         (1 << 31) /* 80000000: Renesas Secure IP Module Stop */
#define R_MSTP_MSTPCRC_MSTPC27         (1 << 27) /* 08000000: Controller Area Network with Flexible Data-Rate 0 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC26         (1 << 26) /* 04000000: Controller Area Network with Flexible Data-Rate 1 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC16         (1 << 16) /* 00010000: Capture Engine Unit Module Stop */
#define R_MSTP_MSTPCRC_MSTPC14         (1 << 14) /* 00004000: Event Link Controller Module Stop */
#define R_MSTP_MSTPCRC_MSTPC13         (1 << 13) /* 00002000: Data Operation Circuit Module Stop */
#define R_MSTP_MSTPCRC_MSTPC8          (1 <<  8) /* 00000100: Serial Sound Interface Enhanced 0 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC7          (1 <<  7) /* 00000080: Serial Sound Interface Enhanced 1 Module Stop */
#define R_MSTP_MSTPCRC_MSTPC1          (1 <<  1) /* 00000002: Cyclic Redundancy Check Calculator Module Stop */
#define R_MSTP_MSTPCRC_MSTPC0          (1 <<  0) /* 00000001: Clock Frequency Accuracy Measurement Circuit Module Stop */

/* Compatibility aliases */
#define R_MSTP_MSTPCRC_SCE5            R_MSTP_MSTPCRC_MSTPC31
#define R_MSTP_MSTPCRC_CANFD0          R_MSTP_MSTPCRC_MSTPC27
#define R_MSTP_MSTPCRC_CANFD1          R_MSTP_MSTPCRC_MSTPC26
#define R_MSTP_MSTPCRC_CEU             R_MSTP_MSTPCRC_MSTPC16
#define R_MSTP_MSTPCRC_ELC             R_MSTP_MSTPCRC_MSTPC14
#define R_MSTP_MSTPCRC_DOC             R_MSTP_MSTPCRC_MSTPC13
#define R_MSTP_MSTPCRC_SSIE0           R_MSTP_MSTPCRC_MSTPC8
#define R_MSTP_MSTPCRC_SSI0            R_MSTP_MSTPCRC_MSTPC8
#define R_MSTP_MSTPCRC_SSIE1           R_MSTP_MSTPCRC_MSTPC7
#define R_MSTP_MSTPCRC_SSI1            R_MSTP_MSTPCRC_MSTPC7
#define R_MSTP_MSTPCRC_CRC             R_MSTP_MSTPCRC_MSTPC1
#define R_MSTP_MSTPCRC_CAC             R_MSTP_MSTPCRC_MSTPC0

/* MSTPCRD - Module Stop Control Register D */

#define R_MSTP_MSTPCRD_MSTPD28         (1 << 28) /* 10000000: High-Speed Analog Comparator 0 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD27         (1 << 27) /* 08000000: High-Speed Analog Comparator 1 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD22         (1 << 22) /* 00400000: Temperature Sensor Module Stop */
#define R_MSTP_MSTPCRD_MSTPD20         (1 << 20) /* 00100000: 12-bit D/A Converter Module Stop */
#define R_MSTP_MSTPCRD_MSTPD16         (1 << 16) /* 00010000: 12-bit A/D Converter 0 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD15         (1 << 15) /* 00008000: 12-bit A/D Converter 1 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD14         (1 << 14) /* 00004000: Port Output Enable for GPT Group A Module Stop */
#define R_MSTP_MSTPCRD_MSTPD13         (1 << 13) /* 00002000: Port Output Enable for GPT Group B Module Stop */
#define R_MSTP_MSTPCRD_MSTPD12         (1 << 12) /* 00001000: Port Output Enable for GPT Group C Module Stop */
#define R_MSTP_MSTPCRD_MSTPD11         (1 << 11) /* 00000800: Port Output Enable for GPT Group D Module Stop */
#define R_MSTP_MSTPCRD_MSTPD5          (1 <<  5) /* 00000020: Low Power Asynchronous General Purpose Timer 0 Module Stop */
#define R_MSTP_MSTPCRD_MSTPD4          (1 <<  4) /* 00000010: Low Power Asynchronous General Purpose Timer 1 Module Stop */

/* Compatibility aliases */
#define R_MSTP_MSTPCRD_ACMPHS0         R_MSTP_MSTPCRD_MSTPD28
#define R_MSTP_MSTPCRD_ACMPHS1         R_MSTP_MSTPCRD_MSTPD27
#define R_MSTP_MSTPCRD_ACMPHS          R_MSTP_MSTPCRD_MSTPD28  /* Default to ACMPHS0 */
#define R_MSTP_MSTPCRD_TSN             R_MSTP_MSTPCRD_MSTPD22
#define R_MSTP_MSTPCRD_DAC12           R_MSTP_MSTPCRD_MSTPD20
#define R_MSTP_MSTPCRD_DAC             R_MSTP_MSTPCRD_MSTPD20
#define R_MSTP_MSTPCRD_ADC0            R_MSTP_MSTPCRD_MSTPD16
#define R_MSTP_MSTPCRD_ADC1            R_MSTP_MSTPCRD_MSTPD15
#define R_MSTP_MSTPCRD_POEG0           R_MSTP_MSTPCRD_MSTPD14
#define R_MSTP_MSTPCRD_POEG1           R_MSTP_MSTPCRD_MSTPD13
#define R_MSTP_MSTPCRD_POEG2           R_MSTP_MSTPCRD_MSTPD12
#define R_MSTP_MSTPCRD_POEG3           R_MSTP_MSTPCRD_MSTPD11
#define R_MSTP_MSTPCRD_POEG            R_MSTP_MSTPCRD_MSTPD14  /* Default to POEG0 */
#define R_MSTP_MSTPCRD_AGT0            R_MSTP_MSTPCRD_MSTPD5
#define R_MSTP_MSTPCRD_AGT1            R_MSTP_MSTPCRD_MSTPD4

/* MSTPCRE - Module Stop Control Register E */

#define R_MSTP_MSTPCRE_MSTPE31         (1 << 31) /* 80000000: General PWM Timer 0 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE30         (1 << 30) /* 40000000: General PWM Timer 1 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE29         (1 << 29) /* 20000000: General PWM Timer 2 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE28         (1 << 28) /* 10000000: General PWM Timer 3 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE27         (1 << 27) /* 08000000: General PWM Timer 4 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE26         (1 << 26) /* 04000000: General PWM Timer 5 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE21         (1 << 21) /* 00200000: General PWM Timer 10 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE20         (1 << 20) /* 00100000: General PWM Timer 11 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE19         (1 << 19) /* 00080000: General PWM Timer 12 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE18         (1 << 18) /* 00040000: General PWM Timer 13 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE9          (1 <<  9) /* 00000200: Ultra-Low Power Timer 0 Module Stop */
#define R_MSTP_MSTPCRE_MSTPE8          (1 <<  8) /* 00000100: Ultra-Low Power Timer 1 Module Stop */

/* Compatibility aliases */
#define R_MSTP_MSTPCRE_GPT0            R_MSTP_MSTPCRE_MSTPE31
#define R_MSTP_MSTPCRE_GPT1            R_MSTP_MSTPCRE_MSTPE30
#define R_MSTP_MSTPCRE_GPT2            R_MSTP_MSTPCRE_MSTPE29
#define R_MSTP_MSTPCRE_GPT3            R_MSTP_MSTPCRE_MSTPE28
#define R_MSTP_MSTPCRE_GPT4            R_MSTP_MSTPCRE_MSTPE27
#define R_MSTP_MSTPCRE_GPT5            R_MSTP_MSTPCRE_MSTPE26
#define R_MSTP_MSTPCRE_GPT10           R_MSTP_MSTPCRE_MSTPE21
#define R_MSTP_MSTPCRE_GPT11           R_MSTP_MSTPCRE_MSTPE20
#define R_MSTP_MSTPCRE_GPT12           R_MSTP_MSTPCRE_MSTPE19
#define R_MSTP_MSTPCRE_GPT13           R_MSTP_MSTPCRE_MSTPE18
#define R_MSTP_MSTPCRE_ULPT0           R_MSTP_MSTPCRE_MSTPE9
#define R_MSTP_MSTPCRE_ULPT1           R_MSTP_MSTPCRE_MSTPE8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8_MSTP_H */
