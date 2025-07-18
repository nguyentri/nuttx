/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_i2c.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA_I2C_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA_I2C_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C Register Offsets *****************************************************/

#define RA_I2C_ICCR1_OFFSET      0x00    /* I2C Bus Control Register 1 */
#define RA_I2C_ICCR2_OFFSET      0x01    /* I2C Bus Control Register 2 */
#define RA_I2C_ICMR1_OFFSET      0x02    /* I2C Bus Mode Register 1 */
#define RA_I2C_ICMR2_OFFSET      0x03    /* I2C Bus Mode Register 2 */
#define RA_I2C_ICMR3_OFFSET      0x04    /* I2C Bus Mode Register 3 */
#define RA_I2C_ICFER_OFFSET      0x05    /* I2C Bus Function Enable Register */
#define RA_I2C_ICSER_OFFSET      0x06    /* I2C Bus Status Enable Register */
#define RA_I2C_ICIER_OFFSET      0x07    /* I2C Bus Interrupt Enable Register */
#define RA_I2C_ICSR1_OFFSET      0x08    /* I2C Bus Status Register 1 */
#define RA_I2C_ICSR2_OFFSET      0x09    /* I2C Bus Status Register 2 */
#define RA_I2C_SARL0_OFFSET      0x0A    /* Slave Address Register L0 */
#define RA_I2C_SARU0_OFFSET      0x0B    /* Slave Address Register U0 */
#define RA_I2C_SARL1_OFFSET      0x0C    /* Slave Address Register L1 */
#define RA_I2C_SARU1_OFFSET      0x0D    /* Slave Address Register U1 */
#define RA_I2C_SARL2_OFFSET      0x0E    /* Slave Address Register L2 */
#define RA_I2C_SARU2_OFFSET      0x0F    /* Slave Address Register U2 */
#define RA_I2C_ICBRL_OFFSET      0x10    /* I2C Bus Bit Rate Register L */
#define RA_I2C_ICBRH_OFFSET      0x11    /* I2C Bus Bit Rate Register H */
#define RA_I2C_ICDRT_OFFSET      0x12    /* I2C Bus Transmit Data Register */
#define RA_I2C_ICDRR_OFFSET      0x13    /* I2C Bus Receive Data Register */

/* I2C Register Addresses ***************************************************/

#define RA_I2C0_BASE             0x4025E000
#define RA_I2C1_BASE             0x4025E100
#define RA_I2C2_BASE             0x4025E200

#define RA_I2C_ICCR1(n)          (RA_I2C_BASE(n) + RA_I2C_ICCR1_OFFSET)
#define RA_I2C_ICCR2(n)          (RA_I2C_BASE(n) + RA_I2C_ICCR2_OFFSET)
#define RA_I2C_ICMR1(n)          (RA_I2C_BASE(n) + RA_I2C_ICMR1_OFFSET)
#define RA_I2C_ICMR2(n)          (RA_I2C_BASE(n) + RA_I2C_ICMR2_OFFSET)
#define RA_I2C_ICMR3(n)          (RA_I2C_BASE(n) + RA_I2C_ICMR3_OFFSET)
#define RA_I2C_ICFER(n)          (RA_I2C_BASE(n) + RA_I2C_ICFER_OFFSET)
#define RA_I2C_ICSER(n)          (RA_I2C_BASE(n) + RA_I2C_ICSER_OFFSET)
#define RA_I2C_ICIER(n)          (RA_I2C_BASE(n) + RA_I2C_ICIER_OFFSET)
#define RA_I2C_ICSR1(n)          (RA_I2C_BASE(n) + RA_I2C_ICSR1_OFFSET)
#define RA_I2C_ICSR2(n)          (RA_I2C_BASE(n) + RA_I2C_ICSR2_OFFSET)
#define RA_I2C_SARL0(n)          (RA_I2C_BASE(n) + RA_I2C_SARL0_OFFSET)
#define RA_I2C_SARU0(n)          (RA_I2C_BASE(n) + RA_I2C_SARU0_OFFSET)
#define RA_I2C_SARL1(n)          (RA_I2C_BASE(n) + RA_I2C_SARL1_OFFSET)
#define RA_I2C_SARU1(n)          (RA_I2C_BASE(n) + RA_I2C_SARU1_OFFSET)
#define RA_I2C_SARL2(n)          (RA_I2C_BASE(n) + RA_I2C_SARL2_OFFSET)
#define RA_I2C_SARU2(n)          (RA_I2C_BASE(n) + RA_I2C_SARU2_OFFSET)
#define RA_I2C_ICBRL(n)          (RA_I2C_BASE(n) + RA_I2C_ICBRL_OFFSET)
#define RA_I2C_ICBRH(n)          (RA_I2C_BASE(n) + RA_I2C_ICBRH_OFFSET)
#define RA_I2C_ICDRT(n)          (RA_I2C_BASE(n) + RA_I2C_ICDRT_OFFSET)
#define RA_I2C_ICDRR(n)          (RA_I2C_BASE(n) + RA_I2C_ICDRR_OFFSET)

/* Helper macro to get base address for I2C channel */
#define RA_I2C_BASE(n)           ((n) == 0 ? RA_I2C0_BASE : \
                                  (n) == 1 ? RA_I2C1_BASE : \
                                  RA_I2C2_BASE)

/* I2C Bus Control Register 1 (ICCR1) **************************************/

#define I2C_ICCR1_ICE            (1 << 7)  /* I2C Enable */
#define I2C_ICCR1_IICRST         (1 << 6)  /* I2C Reset */
#define I2C_ICCR1_CLO            (1 << 5)  /* Extra SCL Clock Cycle Output */
#define I2C_ICCR1_SOWP           (1 << 4)  /* SCL Output Wait */
#define I2C_ICCR1_SCLO           (1 << 3)  /* SCL Output Control */
#define I2C_ICCR1_SDAO           (1 << 2)  /* SDA Output Control */
#define I2C_ICCR1_SCLI           (1 << 1)  /* SCL Input Level Monitor */
#define I2C_ICCR1_SDAI           (1 << 0)  /* SDA Input Level Monitor */

/* I2C Bus Control Register 2 (ICCR2) **************************************/

#define I2C_ICCR2_BBSY           (1 << 7)  /* Bus Busy Detection Flag */
#define I2C_ICCR2_MST            (1 << 6)  /* Master/Slave Mode Select */
#define I2C_ICCR2_TRS            (1 << 5)  /* Transmit/Receive Mode Select */
#define I2C_ICCR2_SP             (1 << 3)  /* Stop Condition Issue Request */
#define I2C_ICCR2_RS             (1 << 2)  /* Restart Condition Issue Request */
#define I2C_ICCR2_ST             (1 << 1)  /* Start Condition Issue Request */

/* I2C Bus Mode Register 1 (ICMR1) *****************************************/

#define I2C_ICMR1_MTWP           (1 << 7)  /* Master Transmit Wait */
#define I2C_ICMR1_CKS_SHIFT      4         /* Internal Reference Clock Select */
#define I2C_ICMR1_CKS_MASK       (0x07 << I2C_ICMR1_CKS_SHIFT)
#define I2C_ICMR1_BCWP           (1 << 3)  /* BC Write Protect */
#define I2C_ICMR1_BC_SHIFT       0         /* Bit Counter */
#define I2C_ICMR1_BC_MASK        (0x07 << I2C_ICMR1_BC_SHIFT)

/* I2C Bus Mode Register 2 (ICMR2) *****************************************/

#define I2C_ICMR2_DLCS           (1 << 7)  /* SDA Output Delay Counter Select */
#define I2C_ICMR2_SDDL_SHIFT     4         /* SDA Output Delay Counter */
#define I2C_ICMR2_SDDL_MASK      (0x07 << I2C_ICMR2_SDDL_SHIFT)
#define I2C_ICMR2_DLSR           (1 << 3)  /* SDA Output Delay Clock Source Select */
#define I2C_ICMR2_TMOH           (1 << 2)  /* Timeout Function H Count Source Select */
#define I2C_ICMR2_TMOL           (1 << 1)  /* Timeout Function L Count Source Select */
#define I2C_ICMR2_TMOS           (1 << 0)  /* Timeout Detection Time Select */

/* I2C Bus Mode Register 3 (ICMR3) *****************************************/

#define I2C_ICMR3_SMBS           (1 << 7)  /* SMBus/I2C Bus Select */
#define I2C_ICMR3_WAIT           (1 << 6)  /* WAIT */
#define I2C_ICMR3_RDRFS          (1 << 5)  /* RDRF Flag Set Timing Select */
#define I2C_ICMR3_ACKWP          (1 << 4)  /* ACK Write Protect */
#define I2C_ICMR3_ACKBT          (1 << 3)  /* Transmit Acknowledge */
#define I2C_ICMR3_ACKBR          (1 << 2)  /* Receive Acknowledge */
#define I2C_ICMR3_NF_SHIFT       0         /* Noise Filter Stage Select */
#define I2C_ICMR3_NF_MASK        (0x03 << I2C_ICMR3_NF_SHIFT)

/* I2C Bus Function Enable Register (ICFER) ********************************/

#define I2C_ICFER_FMPE           (1 << 7)  /* Fast Mode Plus Enable */
#define I2C_ICFER_SCLE           (1 << 6)  /* SCL Synchronous Circuit Enable */
#define I2C_ICFER_NFE            (1 << 5)  /* Digital Noise Filter Circuit Enable */
#define I2C_ICFER_NACKE          (1 << 4)  /* NACK Transmission Arbitration-Lost Detection Enable */
#define I2C_ICFER_SALE           (1 << 3)  /* Slave Arbitration-Lost Detection Enable */
#define I2C_ICFER_NALE           (1 << 2)  /* NACK Arbitration-Lost Detection Enable */
#define I2C_ICFER_MALE           (1 << 1)  /* Master Arbitration-Lost Detection Enable */
#define I2C_ICFER_TMOE           (1 << 0)  /* Timeout Function Enable */

/* I2C Bus Status Enable Register (ICSER) **********************************/

#define I2C_ICSER_HOAE           (1 << 7)  /* Host Address Detection Enable */
#define I2C_ICSER_DIDE           (1 << 6)  /* Device-ID Address Detection Enable */
#define I2C_ICSER_GCAE           (1 << 5)  /* General Call Address Detection Enable */
#define I2C_ICSER_SAR2E          (1 << 2)  /* Slave Address Register 2 Enable */
#define I2C_ICSER_SAR1E          (1 << 1)  /* Slave Address Register 1 Enable */
#define I2C_ICSER_SAR0E          (1 << 0)  /* Slave Address Register 0 Enable */

/* I2C Bus Interrupt Enable Register (ICIER) *******************************/

#define I2C_ICIER_TIE            (1 << 7)  /* Transmit Data Empty Interrupt Enable */
#define I2C_ICIER_TEIE           (1 << 6)  /* Transmit End Interrupt Enable */
#define I2C_ICIER_RIE            (1 << 5)  /* Receive Data Full Interrupt Enable */
#define I2C_ICIER_NAKIE          (1 << 4)  /* NACK Detection Interrupt Enable */
#define I2C_ICIER_SPIE           (1 << 3)  /* Stop Condition Detection Interrupt Enable */
#define I2C_ICIER_STIE           (1 << 2)  /* Start Condition Detection Interrupt Enable */
#define I2C_ICIER_ALIE           (1 << 1)  /* Arbitration-Lost Detection Interrupt Enable */
#define I2C_ICIER_TMOIE          (1 << 0)  /* Timeout Detection Interrupt Enable */

/* I2C Bus Status Register 1 (ICSR1) ***************************************/

#define I2C_ICSR1_HOA            (1 << 7)  /* Host Address Detection Flag */
#define I2C_ICSR1_DID            (1 << 6)  /* Device-ID Address Detection Flag */
#define I2C_ICSR1_GCA            (1 << 5)  /* General Call Address Detection Flag */
#define I2C_ICSR1_AAS2           (1 << 4)  /* Slave Address 2 Detection Flag */
#define I2C_ICSR1_AAS1           (1 << 3)  /* Slave Address 1 Detection Flag */
#define I2C_ICSR1_AAS0           (1 << 2)  /* Slave Address 0 Detection Flag */
#define I2C_ICSR1_CEI            (1 << 1)  /* Communication Event Interrupt Flag */
#define I2C_ICSR1_AAS            (1 << 0)  /* Slave Address Detection Flag */

/* I2C Bus Status Register 2 (ICSR2) ***************************************/

#define I2C_ICSR2_TDRE           (1 << 7)  /* Transmit Data Empty Flag */
#define I2C_ICSR2_TEND           (1 << 6)  /* Transmit End Flag */
#define I2C_ICSR2_RDRF           (1 << 5)  /* Receive Data Full Flag */
#define I2C_ICSR2_NACKF          (1 << 4)  /* NACK Detection Flag */
#define I2C_ICSR2_STOP           (1 << 3)  /* Stop Condition Detection Flag */
#define I2C_ICSR2_START          (1 << 2)  /* Start Condition Detection Flag */
#define I2C_ICSR2_AL             (1 << 1)  /* Arbitration-Lost Detection Flag */
#define I2C_ICSR2_TMOF           (1 << 0)  /* Timeout Detection Flag */

/* Slave Address Register Upper (SARU) *************************************/

#define I2C_SARU_SVA_SHIFT       1         /* Slave Address (upper 2 bits) */
#define I2C_SARU_SVA_MASK        (0x03 << I2C_SARU_SVA_SHIFT)
#define I2C_SARU_FS              (1 << 0)  /* 7-bit/10-bit Address Format Select */

/* I2C Bit Rate Registers **************************************************/

#define I2C_BRR_RESERVED_BITS    0xE0      /* Reserved bits in bit rate registers */

/* Clock source selection values for ICMR1.CKS */
#define I2C_CLOCK_PCLKB_1        0x00      /* PCLKB/1 */
#define I2C_CLOCK_PCLKB_2        0x01      /* PCLKB/2 */
#define I2C_CLOCK_PCLKB_4        0x02      /* PCLKB/4 */
#define I2C_CLOCK_PCLKB_8        0x03      /* PCLKB/8 */
#define I2C_CLOCK_PCLKB_16       0x04      /* PCLKB/16 */
#define I2C_CLOCK_PCLKB_32       0x05      /* PCLKB/32 */
#define I2C_CLOCK_PCLKB_64       0x06      /* PCLKB/64 */
#define I2C_CLOCK_PCLKB_128      0x07      /* PCLKB/128 */

/* Standard I2C frequencies */
#define I2C_FREQ_STANDARD        100000    /* 100 kHz */
#define I2C_FREQ_FAST            400000    /* 400 kHz */
#define I2C_FREQ_FAST_PLUS       1000000   /* 1 MHz */

/* DTC Transfer Settings for I2C */
#define I2C_DTC_ACTIVATION_RXI   0x01      /* DTC activation source: RXI */
#define I2C_DTC_ACTIVATION_TXI   0x02      /* DTC activation source: TXI */

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_I2C_H */
