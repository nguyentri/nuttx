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

#  define R_MSTP_MSTPCRA               (R_MSTP_BASE   + R_MSTP_MSTPCRA_OFFSET)
#  define R_MSTP_MSTPCRB               (R_MSTP_BASE   + R_MSTP_MSTPCRB_OFFSET)
#  define R_MSTP_MSTPCRC               (R_MSTP_BASE   + R_MSTP_MSTPCRC_OFFSET)
#  define R_MSTP_MSTPCRD               (R_MSTP_BASE   + R_MSTP_MSTPCRD_OFFSET)
#  define R_MSTP_MSTPCRE               (R_MSTP_BASE   + R_MSTP_MSTPCRE_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* MSTPCRB - Module Stop Control Register B */

#define R_MSTP_MSTPCRB_SCI0            (1 << 31) /* 80000000: Serial Communication Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_SCI1            (1 << 30) /* 40000000: Serial Communication Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_SCI2            (1 << 29) /* 20000000: Serial Communication Interface 2 Module Stop */
#define R_MSTP_MSTPCRB_SCI3            (1 << 28) /* 10000000: Serial Communication Interface 3 Module Stop */
#define R_MSTP_MSTPCRB_SCI4            (1 << 27) /* 08000000: Serial Communication Interface 4 Module Stop */
#define R_MSTP_MSTPCRB_SCI5            (1 << 26) /* 04000000: Serial Communication Interface 5 Module Stop */
#define R_MSTP_MSTPCRB_SCI6            (1 << 25) /* 02000000: Serial Communication Interface 6 Module Stop */
#define R_MSTP_MSTPCRB_SCI7            (1 << 24) /* 01000000: Serial Communication Interface 7 Module Stop */
#define R_MSTP_MSTPCRB_SCI8            (1 << 23) /* 00800000: Serial Communication Interface 8 Module Stop */
#define R_MSTP_MSTPCRB_SCI9            (1 << 22) /* 00400000: Serial Communication Interface 9 Module Stop */
#define R_MSTP_MSTPCRB_SPI0            (1 << 19) /* 00080000: Serial Peripheral Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_SPI1            (1 << 18) /* 00040000: Serial Peripheral Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_USBFS           (1 << 11) /* 00000800: Universal Serial Bus 2.0 FS Interface Module Stop */
#define R_MSTP_MSTPCRB_IIC0            (1 <<  9) /* 00000200: I2C Bus Interface 0 Module Stop */
#define R_MSTP_MSTPCRB_IIC1            (1 <<  8) /* 00000100: I2C Bus Interface 1 Module Stop */
#define R_MSTP_MSTPCRB_IIC2            (1 <<  7) /* 00000080: I2C Bus Interface 2 Module Stop */
#define R_MSTP_MSTPCRB_CAN             (1 <<  2) /* 00000004: Controller Area Network Module Stop */
#define R_MSTP_MSTPCRB_CANFD           (1 <<  2) /* 00000004: CAN-FD Module Stop (same bit as CAN) */

/* MSTPCRC - Module Stop Control Register C */

#define R_MSTP_MSTPCRC_SCE5            (1 << 31) /* 80000000: SCE5 Module Stop */
#define R_MSTP_MSTPCRC_TRNG            (1 << 28) /* 10000000: True Random Number Generator Module Stop */
#define R_MSTP_MSTPCRC_JPEG            (1 << 25) /* 02000000: JPEG Codec Module Stop */
#define R_MSTP_MSTPCRC_EDMAC0          (1 << 15) /* 00008000: Ethernet DMA Controller 0 Module Stop */
#define R_MSTP_MSTPCRC_ELC             (1 << 14) /* 00004000: Event Link Controller Module Stop */
#define R_MSTP_MSTPCRC_DOC             (1 << 13) /* 00002000: Data Operation Circuit Module Stop */
#define R_MSTP_MSTPCRC_SSIE0           (1 <<  8) /* 00000100: Synchronous Serial Interface 0 Module Stop */
#define R_MSTP_MSTPCRC_SLCDC           (1 <<  4) /* 00000010: Segment LCD Controller Module Stop */
#define R_MSTP_MSTPCRC_CTSU            (1 <<  3) /* 00000008: Capacitive Touch Sensing Unit Module Stop */
#define R_MSTP_MSTPCRC_CRC             (1 <<  1) /* 00000002: Cyclic Redundancy Check Calculator Module Stop */
#define R_MSTP_MSTPCRC_CAC             (1 <<  0) /* 00000001: Clock Frequency Accuracy Measurement Circuit Module Stop */

/* MSTPCRD - Module Stop Control Register D */

#define R_MSTP_MSTPCRD_OPAMP           (1 << 31) /* 80000000: Operational Amplifier Module Stop */
#define R_MSTP_MSTPCRD_ACMPLP          (1 << 29) /* 20000000: Low-Power Analog Comparator Module Stop */
#define R_MSTP_MSTPCRD_ACMPHS          (1 << 28) /* 10000000: High-Speed Analog Comparator Module Stop */
#define R_MSTP_MSTPCRD_ULPT1           (1 << 25) /* 02000000: Ultra-Low Power Timer 1 Module Stop */
#define R_MSTP_MSTPCRD_ULPT0           (1 << 24) /* 01000000: Ultra-Low Power Timer 0 Module Stop */
#define R_MSTP_MSTPCRD_CEU             (1 << 22) /* 00400000: Camera Engine Unit Module Stop */
#define R_MSTP_MSTPCRD_DAC12           (1 << 20) /* 00100000: 12-Bit D/A Converter Module Stop */
#define R_MSTP_MSTPCRD_DAC8            (1 << 19) /* 00080000: 8-bit D/A Converter Module Stop */
#define R_MSTP_MSTPCRD_TSN             (1 << 18) /* 00040000: Temperature Sensor Module Stop */
#define R_MSTP_MSTPCRD_ADC1            (1 << 17) /* 00020000: 14-Bit A/D Converter 1 Module Stop */
#define R_MSTP_MSTPCRD_ADC0            (1 << 16) /* 00010000: 14-Bit A/D Converter 0 Module Stop */
#define R_MSTP_MSTPCRD_POEG            (1 << 14) /* 00004000: Port Output Enable for GPT Module Stop */
#define R_MSTP_MSTPCRD_GPT_1           (1 <<  6) /* 00000040: General PWM Timer 169 to 164 Module Stop */
#define R_MSTP_MSTPCRD_GPT_2           (1 <<  5) /* 00000020: General PWM Timer 323 to 320 Module Stop */
#define R_MSTP_MSTPCRD_AGT0            (1 <<  3) /* 00000008: Asynchronous General Purpose Timer 0 Module Stop */
#define R_MSTP_MSTPCRD_AGT1            (1 <<  2) /* 00000004: Asynchronous General Purpose Timer 1 Module Stop */

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
