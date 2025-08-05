/****************************************************************************
 * arch/arm/src/ra8/hardware/ra8e1_icu.h
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

#ifndef __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8E1_ICU_H
#define __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8E1_ICU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ELC Event Definitions - Updated from FSP bsp_elc.h */

 #define  EVENT_NONE                       (0x00)  /* Link disabled */
 #define  EVENT_ICU_IRQ0                   (0x01)  /* External pin interrupt 0 */
 #define  EVENT_ICU_IRQ1                   (0x02)  /* External pin interrupt 1 */
 #define  EVENT_ICU_IRQ2                   (0x03)  /* External pin interrupt 2 */
 #define  EVENT_ICU_IRQ3                   (0x04)  /* External pin interrupt 3 */
 #define  EVENT_ICU_IRQ4                   (0x05)  /* External pin interrupt 4 */
 #define  EVENT_ICU_IRQ5                   (0x06)  /* External pin interrupt 5 */
 #define  EVENT_ICU_IRQ6                   (0x07)  /* External pin interrupt 6 */
 #define  EVENT_ICU_IRQ7                   (0x08)  /* External pin interrupt 7 */
 #define  EVENT_ICU_IRQ8                   (0x09)  /* External pin interrupt 8 */
 #define  EVENT_ICU_IRQ9                   (0x0A)  /* External pin interrupt 9 */
 #define  EVENT_ICU_IRQ10                  (0x0B)  /* External pin interrupt 10 */
 #define  EVENT_ICU_IRQ11                  (0x0C)  /* External pin interrupt 11 */
 #define  EVENT_ICU_IRQ12                  (0x0D)  /* External pin interrupt 12 */
 #define  EVENT_ICU_IRQ13                  (0x0E)  /* External pin interrupt 13 */
 #define  EVENT_ICU_IRQ14                  (0x0F)  /* External pin interrupt 14 */
 #define  EVENT_ICU_IRQ15                  (0x10)  /* External pin interrupt 15 */
 #define  EVENT_DMAC0_INT                  (0x11)  /* DMAC0 transfer end */
 #define  EVENT_DMAC1_INT                  (0x12)  /* DMAC1 transfer end */
 #define  EVENT_DMAC2_INT                  (0x13)  /* DMAC2 transfer end */
 #define  EVENT_DMAC3_INT                  (0x14)  /* DMAC3 transfer end */
 #define  EVENT_DMAC4_INT                  (0x15)  /* DMAC4 transfer end */
 #define  EVENT_DMAC5_INT                  (0x16)  /* DMAC5 transfer end */
 #define  EVENT_DMAC6_INT                  (0x17)  /* DMAC6 transfer end */
 #define  EVENT_DMAC7_INT                  (0x18)  /* DMAC7 transfer end */
 #define  EVENT_DTC_END                    (0x21)  /* DTC transfer end */
 #define  EVENT_DTC_COMPLETE               (0x22)  /* DTC transfer complete */
 #define  EVENT_DMA_TRANSERR               (0x27)  /* DMA/DTC transfer error */
 #define  EVENT_ICU_SNOOZE_CANCEL          (0x17)  /* Canceling from Snooze mode */
 #define  EVENT_FCU_FRDYI                  (0x31)  /* Flash ready interrupt */
 #define  EVENT_LVD_LVD1                   (0x38)  /* Voltage monitor 1 interrupt */
 #define  EVENT_LVD_LVD2                   (0x39)  /* Voltage monitor 2 interrupt */
 #define  EVENT_LVD_VBATT                  (0x3D)  /* VBATT low voltage detect */
 #define  EVENT_CGC_MOSC_STOP              (0x3E)  /* Main Clock oscillation stop */
 #define  EVENT_LPM_SNOOZE_REQUEST         (0x1D)  /* Snooze entry */
 #define  EVENT_ULPT0_INT                  (0x40)  /* ULPT0 Underflow */
 #define  EVENT_ULPT0_COMPARE_A            (0x41)  /* ULPT0 Compare match A */
 #define  EVENT_ULPT0_COMPARE_B            (0x42)  /* ULPT0 Compare match B */
 #define  EVENT_ULPT1_INT                  (0x43)  /* ULPT1 Underflow */
 #define  EVENT_ULPT1_COMPARE_A            (0x44)  /* ULPT1 Compare match A */
 #define  EVENT_ULPT1_COMPARE_B            (0x45)  /* ULPT1 Compare match B */
 #define  EVENT_AGT0_INT                   (0x46)  /* AGT interrupt */
 #define  EVENT_AGT0_COMPARE_A             (0x47)  /* Compare match A */
 #define  EVENT_AGT0_COMPARE_B             (0x48)  /* Compare match B */
 #define  EVENT_AGT1_INT                   (0x49)  /* AGT interrupt */
 #define  EVENT_AGT1_COMPARE_A             (0x4A)  /* Compare match A */
 #define  EVENT_AGT1_COMPARE_B             (0x4B)  /* Compare match B */
 #define  EVENT_IWDT_UNDERFLOW             (0x52)  /* IWDT underflow */
 #define  EVENT_WDT_UNDERFLOW              (0x53)  /* WDT underflow */
 #define  EVENT_RTC_ALARM                  (0x55)  /* Alarm interrupt */
 #define  EVENT_RTC_PERIOD                 (0x56)  /* Periodic interrupt */
 #define  EVENT_RTC_CARRY                  (0x57)  /* Carry interrupt */
 #define  EVENT_ADC0_SCAN_END              (0x1AE) /* End of A/D scanning operation */
 #define  EVENT_ADC0_SCAN_END_B            (0x1AF) /* A/D scan end interrupt for group B */
 #define  EVENT_ADC0_WINDOW_A              (0x1B0) /* Window A Compare match interrupt */
 #define  EVENT_ADC0_WINDOW_B              (0x1B1) /* Window B Compare match interrupt */
 #define  EVENT_ADC0_COMPARE_MATCH         (0x1B2) /* Compare match */
 #define  EVENT_ADC0_COMPARE_MISMATCH      (0x1B3) /* Compare mismatch */
 #define  EVENT_ADC1_SCAN_END              (0x1B4) /* End of A/D scanning operation */
 #define  EVENT_ADC1_SCAN_END_B            (0x1B5) /* A/D scan end interrupt for group B */
 #define  EVENT_ADC1_WINDOW_A              (0x1B6) /* Window A Compare match interrupt */
 #define  EVENT_ADC1_WINDOW_B              (0x1B7) /* Window B Compare match interrupt */
 #define  EVENT_ADC1_COMPARE_MATCH         (0x1B8) /* Compare match */
 #define  EVENT_ADC1_COMPARE_MISMATCH      (0x1B9) /* Compare mismatch */
 #define  EVENT_ACMPLP0_INT                (0x2F)  /* Low Power Comparator channel 0 interrupt */
 #define  EVENT_ACMPLP1_INT                (0x30)  /* Low Power Comparator channel 1 interrupt */
 #define  EVENT_ACMPHS0_INT                (0x7B)  /* High Speed Comparator channel 0 interrupt */
 #define  EVENT_ACMPHS1_INT                (0x7C)  /* High Speed Comparator channel 1 interrupt */
 #define  EVENT_USBFS_FIFO_0               (0x58)  /* DMA transfer request 0 */
 #define  EVENT_USBFS_FIFO_1               (0x59)  /* DMA transfer request 1 */
 #define  EVENT_USBFS_INT                  (0x5A)  /* USBFS interrupt */
 #define  EVENT_USBFS_RESUME               (0x5B)  /* USBFS resume interrupt */
 #define  EVENT_IIC0_RXI                   (0x5C)  /* Receive data full */
 #define  EVENT_IIC0_TXI                   (0x5D)  /* Transmit data empty */
 #define  EVENT_IIC0_TEI                   (0x5E)  /* Transmit end */
 #define  EVENT_IIC0_ERI                   (0x5F)  /* Transfer error */
 #define  EVENT_IIC0_WUI                   (0x60)  /* Wakeup interrupt */
 #define  EVENT_IIC1_RXI                   (0x61)  /* Receive data full */
 #define  EVENT_IIC1_TXI                   (0x62)  /* Transmit data empty */
 #define  EVENT_IIC1_TEI                   (0x63)  /* Transmit end */
 #define  EVENT_IIC1_ERI                   (0x64)  /* Transfer error */
 #define  EVENT_SSI0_TXI                   (0x73)  /* Transmit data empty */
 #define  EVENT_SSI0_RXI                   (0x74)  /* Receive data full */
 #define  EVENT_SSI0_INT                   (0x76)  /* Error interrupt */
 #define  EVENT_SSI1_TXI                   (0x79)  /* Transmit data empty */
 #define  EVENT_SSI1_RXI                   (0x79)  /* Receive data full */
 #define  EVENT_SSI1_INT                   (0x7A)  /* Error interrupt */
 #define  EVENT_CTSU_WRITE                 (0x42)  /* Write request interrupt */
 #define  EVENT_CTSU_READ                  (0x43)  /* Measurement data transfer request interrupt */
 #define  EVENT_CTSU_END                   (0x44)  /* Measurement end interrupt */
 #define  EVENT_KEY_INT                    (0x45)  /* Key interrupt */
 #define  EVENT_DOC_INT                    (0x1BA) /* Data operation circuit interrupt */
 #define  EVENT_CAC_FREQUENCY_ERROR        (0x8C)  /* Frequency error interrupt */
 #define  EVENT_CAC_MEASUREMENT_END        (0x8D)  /* Measurement end interrupt */
 #define  EVENT_CAC_OVERFLOW               (0x8E)  /* Overflow interrupt */
 #define  EVENT_CAN0_ERROR                 (0x186) /* Global error */
 #define  EVENT_CAN0_FIFO_RX               (0x185) /* Global receive FIFO interrupt */
 #define  EVENT_CAN0_FIFO_TX               (0x18F) /* Transmit interrupt */
 #define  EVENT_CAN0_MAILBOX_RX            (0x193) /* Receive message buffer interrupt */
 #define  EVENT_CAN0_MAILBOX_TX            (0x192) /* Transmit interrupt */
 #define  EVENT_CAN1_ERROR                 (0x195) /* Channel error */
 #define  EVENT_CAN1_FIFO_RX               (0x196) /* Common FIFO receive interrupt */
 #define  EVENT_CAN1_FIFO_TX               (0x194) /* Transmit interrupt */
 #define  EVENT_CAN1_MAILBOX_RX            (0x198) /* Receive message buffer interrupt */
 #define  EVENT_CAN1_MAILBOX_TX            (0x197) /* Channel DMA request */
 #define  EVENT_IOPORT_EVENT_1             (0x88)  /* Port 1 event */
 #define  EVENT_IOPORT_EVENT_2             (0x89)  /* Port 2 event */
 #define  EVENT_IOPORT_EVENT_3             (0x8A)  /* Port 3 event */
 #define  EVENT_IOPORT_EVENT_4             (0x8B)  /* Port 4 event */
 #define  EVENT_SOFTWARE_EVENT_0           (0x83)  /* Software event 0 */
 #define  EVENT_SOFTWARE_EVENT_1           (0x84)  /* Software event 1 */
 #define  EVENT_POEG0_EVENT                (0x8F)  /* Port Output disable 0 interrupt */
 #define  EVENT_POEG1_EVENT                (0x90)  /* Port Output disable 1 interrupt */
 #define  EVENT_POEG2_EVENT                (0x91)  /* Port Output disable 2 interrupt */
 #define  EVENT_POEG3_EVENT                (0x92)  /* Port Output disable 3 interrupt */
 #define  EVENT_GPT0_CAPTURE_COMPARE_A     (0xA1)  /* Capture/Compare match A */
 #define  EVENT_GPT0_CAPTURE_COMPARE_B     (0xA2)  /* Capture/Compare match B */
 #define  EVENT_GPT0_COMPARE_C             (0xA3)  /* Compare match C */
 #define  EVENT_GPT0_COMPARE_D             (0xA4)  /* Compare match D */
 #define  EVENT_GPT0_COMPARE_E             (0xA5)  /* Compare match E */
 #define  EVENT_GPT0_COMPARE_F             (0xA6)  /* Compare match F */
 #define  EVENT_GPT0_COUNTER_OVERFLOW      (0xA7)  /* Overflow */
 #define  EVENT_GPT0_COUNTER_UNDERFLOW     (0xA8)  /* Underflow */
 #define  EVENT_GPT0_PC                    (0xA9)  /* Period count function finish */
 #define  EVENT_GPT1_CAPTURE_COMPARE_A     (0xAA)  /* Capture/Compare match A */
 #define  EVENT_GPT1_CAPTURE_COMPARE_B     (0xAB)  /* Capture/Compare match B */
 #define  EVENT_GPT1_COMPARE_C             (0xAC)  /* Compare match C */
 #define  EVENT_GPT1_COMPARE_D             (0xAD)  /* Compare match D */
 #define  EVENT_GPT1_COMPARE_E             (0xAE)  /* Compare match E */
 #define  EVENT_GPT1_COMPARE_F             (0xAF)  /* Compare match F */
 #define  EVENT_GPT1_COUNTER_OVERFLOW      (0xB0)  /* Overflow */
 #define  EVENT_GPT1_COUNTER_UNDERFLOW     (0xB1)  /* Underflow */
 #define  EVENT_GPT1_PC                    (0xB2)  /* Period count function finish */
 #define  EVENT_GPT2_CAPTURE_COMPARE_A     (0xB3)  /* Capture/Compare match A */
 #define  EVENT_GPT2_CAPTURE_COMPARE_B     (0xB4)  /* Capture/Compare match B */
 #define  EVENT_GPT2_COMPARE_C             (0xB5)  /* Compare match C */
 #define  EVENT_GPT2_COMPARE_D             (0xB6)  /* Compare match D */
 #define  EVENT_GPT2_COMPARE_E             (0xB7)  /* Compare match E */
 #define  EVENT_GPT2_COMPARE_F             (0xB8)  /* Compare match F */
 #define  EVENT_GPT2_COUNTER_OVERFLOW      (0xB9)  /* Overflow */
 #define  EVENT_GPT2_COUNTER_UNDERFLOW     (0xBA)  /* Underflow */
 #define  EVENT_GPT2_PC                    (0xBB)  /* Period count function finish */
 #define  EVENT_GPT3_CAPTURE_COMPARE_A     (0xBC)  /* Capture/Compare match A */
 #define  EVENT_GPT3_CAPTURE_COMPARE_B     (0xBD)  /* Capture/Compare match B */
 #define  EVENT_GPT3_COMPARE_C             (0xBE)  /* Compare match C */
 #define  EVENT_GPT3_COMPARE_D             (0xBF)  /* Compare match D */
 #define  EVENT_GPT3_COMPARE_E             (0xC0)  /* Compare match E */
 #define  EVENT_GPT3_COMPARE_F             (0xC1)  /* Compare match F */
 #define  EVENT_GPT3_COUNTER_OVERFLOW      (0xC2)  /* Overflow */
 #define  EVENT_GPT3_COUNTER_UNDERFLOW     (0xC3)  /* Underflow */
 #define  EVENT_GPT3_PC                    (0xC4)  /* Period count function finish */
 #define  EVENT_GPT4_CAPTURE_COMPARE_A     (0xC5)  /* Capture/Compare match A */
 #define  EVENT_GPT4_CAPTURE_COMPARE_B     (0xC6)  /* Capture/Compare match B */
 #define  EVENT_GPT4_COMPARE_C             (0xC7)  /* Compare match C */
 #define  EVENT_GPT4_COMPARE_D             (0xC8)  /* Compare match D */
 #define  EVENT_GPT4_COMPARE_E             (0xC9)  /* Compare match E */
 #define  EVENT_GPT4_COMPARE_F             (0xCA)  /* Compare match F */
 #define  EVENT_GPT4_COUNTER_OVERFLOW      (0xCB)  /* Overflow */
 #define  EVENT_GPT4_COUNTER_UNDERFLOW     (0xCC)  /* Underflow */
 #define  EVENT_GPT5_CAPTURE_COMPARE_A     (0xCE)  /* Capture/Compare match A */
 #define  EVENT_GPT5_CAPTURE_COMPARE_B     (0xCF)  /* Capture/Compare match B */
 #define  EVENT_GPT5_COMPARE_C             (0xD0)  /* Compare match C */
 #define  EVENT_GPT5_COMPARE_D             (0xD1)  /* Compare match D */
 #define  EVENT_GPT5_COMPARE_E             (0xD2)  /* Compare match E */
 #define  EVENT_GPT5_COMPARE_F             (0xD3)  /* Compare match F */
 #define  EVENT_GPT5_COUNTER_OVERFLOW      (0xD4)  /* Overflow */
 #define  EVENT_GPT5_COUNTER_UNDERFLOW     (0xD5)  /* Underflow */
 #define  EVENT_GPT6_CAPTURE_COMPARE_A     (0x87)  /* Capture/Compare match A */
 #define  EVENT_GPT6_CAPTURE_COMPARE_B     (0x88)  /* Capture/Compare match B */
 #define  EVENT_GPT6_COMPARE_C             (0x89)  /* Compare match C */
 #define  EVENT_GPT6_COMPARE_D             (0x8A)  /* Compare match D */
 #define  EVENT_GPT6_COMPARE_E             (0x8B)  /* Compare match E */
 #define  EVENT_GPT6_COMPARE_F             (0x8C)  /* Compare match F */
 #define  EVENT_GPT6_COUNTER_OVERFLOW      (0x8D)  /* Overflow */
 #define  EVENT_GPT6_COUNTER_UNDERFLOW     (0x8E)  /* Underflow */
 #define  EVENT_GPT7_CAPTURE_COMPARE_A     (0x8F)  /* Capture/Compare match A */
 #define  EVENT_GPT7_CAPTURE_COMPARE_B     (0x90)  /* Capture/Compare match B */
 #define  EVENT_GPT7_COMPARE_C             (0x91)  /* Compare match C */
 #define  EVENT_GPT7_COMPARE_D             (0x92)  /* Compare match D */
 #define  EVENT_GPT7_COMPARE_E             (0x93)  /* Compare match E */
 #define  EVENT_GPT7_COMPARE_F             (0x94)  /* Compare match F */
 #define  EVENT_GPT7_COUNTER_OVERFLOW      (0x95)  /* Overflow */
 #define  EVENT_GPT7_COUNTER_UNDERFLOW     (0x96)  /* Underflow */
 #define  EVENT_GPT10_CAPTURE_COMPARE_A    (0xFB)  /* Capture/Compare match A */
 #define  EVENT_GPT10_CAPTURE_COMPARE_B    (0xFC)  /* Capture/Compare match B */
 #define  EVENT_GPT10_COMPARE_C            (0xFD)  /* Compare match C */
 #define  EVENT_GPT10_COMPARE_D            (0xFE)  /* Compare match D */
 #define  EVENT_GPT10_COMPARE_E            (0xFF)  /* Compare match E */
 #define  EVENT_GPT10_COMPARE_F            (0x100) /* Compare match F */
 #define  EVENT_GPT10_COUNTER_OVERFLOW     (0x101) /* Overflow */
 #define  EVENT_GPT10_COUNTER_UNDERFLOW    (0x102) /* Underflow */
 #define  EVENT_GPT10_PC                   (0x103) /* Period count function finish */
 #define  EVENT_GPT11_CAPTURE_COMPARE_A    (0x104) /* Capture/Compare match A */
 #define  EVENT_GPT11_CAPTURE_COMPARE_B    (0x105) /* Capture/Compare match B */
 #define  EVENT_GPT11_COMPARE_C            (0x106) /* Compare match C */
 #define  EVENT_GPT11_COMPARE_D            (0x107) /* Compare match D */
 #define  EVENT_GPT11_COMPARE_E            (0x108) /* Compare match E */
 #define  EVENT_GPT11_COMPARE_F            (0x109) /* Compare match F */
 #define  EVENT_GPT11_COUNTER_OVERFLOW     (0x10A) /* Overflow */
 #define  EVENT_GPT11_COUNTER_UNDERFLOW    (0x10B) /* Underflow */
 #define  EVENT_GPT12_CAPTURE_COMPARE_A    (0x10D) /* Capture/Compare match A */
 #define  EVENT_GPT12_CAPTURE_COMPARE_B    (0x10E) /* Capture/Compare match B */
 #define  EVENT_GPT12_COMPARE_C            (0x10F) /* Compare match C */
 #define  EVENT_GPT12_COMPARE_D            (0x110) /* Compare match D */
 #define  EVENT_GPT12_COMPARE_E            (0x111) /* Compare match E */
 #define  EVENT_GPT12_COMPARE_F            (0x112) /* Compare match F */
 #define  EVENT_GPT12_COUNTER_OVERFLOW     (0x113) /* Overflow */
 #define  EVENT_GPT12_COUNTER_UNDERFLOW    (0x114) /* Underflow */
 #define  EVENT_GPT13_CAPTURE_COMPARE_A    (0x116) /* Capture/Compare match A */
 #define  EVENT_GPT13_CAPTURE_COMPARE_B    (0x117) /* Capture/Compare match B */
 #define  EVENT_GPT13_COMPARE_C            (0x118) /* Compare match C */
 #define  EVENT_GPT13_COMPARE_D            (0x119) /* Compare match D */
 #define  EVENT_GPT13_COMPARE_E            (0x11A) /* Compare match E */
 #define  EVENT_GPT13_COMPARE_F            (0x11B) /* Compare match F */
 #define  EVENT_GPT13_COUNTER_OVERFLOW     (0x11C) /* Overflow */
 #define  EVENT_GPT13_COUNTER_UNDERFLOW    (0x11D) /* Underflow */
 #define  EVENT_OPS_UVW_EDGE               (0x97)  /* UVW edge event */
 #define  EVENT_SCI0_RXI                   (0x124) /* Receive data full */
 #define  EVENT_SCI0_TXI                   (0x125) /* Transmit data empty */
 #define  EVENT_SCI0_TEI                   (0x126) /* Transmit end */
 #define  EVENT_SCI0_ERI                   (0x127) /* Receive error */
 #define  EVENT_SCI0_AM                    (0x12A) /* Address match event */
 #define  EVENT_SCI0_RXI_OR_ERI            (0x9D)  /* Receive data full/Receive error */
 #define  EVENT_SCI1_RXI                   (0x12B) /* Receive data full */
 #define  EVENT_SCI1_TXI                   (0x12C) /* Transmit data empty */
 #define  EVENT_SCI1_TEI                   (0x12D) /* Transmit end */
 #define  EVENT_SCI1_ERI                   (0x12E) /* Receive error */
 #define  EVENT_SCI1_AM                    (0x131) /* Address match event */
 #define  EVENT_SCI2_RXI                   (0x132) /* Receive data full */
 #define  EVENT_SCI2_TXI                   (0x133) /* Transmit data empty */
 #define  EVENT_SCI2_TEI                   (0x134) /* Transmit end */
 #define  EVENT_SCI2_ERI                   (0x135) /* Receive error */
 #define  EVENT_SCI2_AM                    (0x138) /* Address match event */
 #define  EVENT_SCI3_RXI                   (0x139) /* Receive data full */
 #define  EVENT_SCI3_TXI                   (0x13A) /* Transmit data empty */
 #define  EVENT_SCI3_TEI                   (0x13B) /* Transmit end */
 #define  EVENT_SCI3_ERI                   (0x13C) /* Receive error */
 #define  EVENT_SCI3_AM                    (0x13F) /* Address match event */
 #define  EVENT_SCI4_RXI                   (0x140) /* Receive data full */
 #define  EVENT_SCI4_TXI                   (0x141) /* Transmit data empty */
 #define  EVENT_SCI4_TEI                   (0x142) /* Transmit end */
 #define  EVENT_SCI4_ERI                   (0x143) /* Receive error */
 #define  EVENT_SCI4_AM                    (0x146) /* Address match event */
 #define  EVENT_SCI9_RXI                   (0x163) /* Receive data full */
 #define  EVENT_SCI9_TXI                   (0x164) /* Transmit data empty */
 #define  EVENT_SCI9_TEI                   (0x165) /* Transmit end */
 #define  EVENT_SCI9_ERI                   (0x166) /* Receive error */
 #define  EVENT_SCI9_AM                    (0x169) /* Address match event */
 #define  EVENT_SPI0_RXI                   (0x178) /* Receive buffer full */
 #define  EVENT_SPI0_TXI                   (0x179) /* Transmit buffer empty */
 #define  EVENT_SPI0_IDLE                  (0x17A) /* Idle */
 #define  EVENT_SPI0_ERI                   (0x17B) /* Error */
 #define  EVENT_SPI0_TEI                   (0x17C) /* Transmission complete event */
 #define  EVENT_SPI1_RXI                   (0x17D) /* Receive buffer full */
 #define  EVENT_SPI1_TXI                   (0x17E) /* Transmit buffer empty */
 #define  EVENT_SPI1_IDLE                  (0x17F) /* Idle */
 #define  EVENT_SPI1_ERI                   (0x180) /* Error */
 #define  EVENT_SPI1_TEI                   (0x181) /* Transmission complete event */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8E1_ICU_H */
