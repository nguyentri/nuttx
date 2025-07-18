/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_adc.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA_ADC_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC Base Addresses *******************************************************/

#define RA8_ADC0_BASE           0x40300000
#define RA8_ADC1_BASE           0x40301000

/* ADC Register Offsets *****************************************************/

#define RA_ADC_ADCSR_OFFSET     0x0000  /* A/D Control/Status Register */
#define RA_ADC_ADANSE0_OFFSET   0x0004  /* A/D Channel Select Register 0 */
#define RA_ADC_ADANSE1_OFFSET   0x0008  /* A/D Channel Select Register 1 */
#define RA_ADC_ADADS0_OFFSET    0x000C  /* A/D-converted Value Addition/Average Channel Select Register 0 */
#define RA_ADC_ADADS1_OFFSET    0x0010  /* A/D-converted Value Addition/Average Channel Select Register 1 */
#define RA_ADC_ADADC_OFFSET     0x0014  /* A/D-converted Value Addition/Average Count Select Register */
#define RA_ADC_ADCER_OFFSET     0x0018  /* A/D Control Extended Register */
#define RA_ADC_ADSTRGR_OFFSET   0x001C  /* A/D Conversion Start Trigger Register */
#define RA_ADC_ADEXICR_OFFSET   0x0020  /* A/D Conversion Extended Input Control Register */
#define RA_ADC_ADANSB0_OFFSET   0x0024  /* A/D Channel Select Register B0 */
#define RA_ADC_ADANSB1_OFFSET   0x0028  /* A/D Channel Select Register B1 */
#define RA_ADC_ADDBLDR_OFFSET   0x002C  /* A/D Data Duplication Register */
#define RA_ADC_ADTSDR_OFFSET    0x0030  /* A/D Temperature Sensor Data Register */
#define RA_ADC_ADOCDR_OFFSET    0x0034  /* A/D Internal Reference Voltage Data Register */
#define RA_ADC_ADRD_OFFSET      0x0038  /* A/D Self-Diagnosis Data Register */

/* A/D Data Register An (n = 000 to 028) */
#define RA_ADC_ADDR_OFFSET(n)   (0x0100 + ((n) * 2))

/* ADC Channel Select Register Group B */
#define RA_ADC_ADANSC0_OFFSET   0x0200  /* A/D Channel Select Register C0 */
#define RA_ADC_ADANSC1_OFFSET   0x0204  /* A/D Channel Select Register C1 */

/* ADC Window Function Registers */
#define RA_ADC_ADCMPDR0_OFFSET  0x0300  /* A/D Compare Data Register 0 */
#define RA_ADC_ADCMPDR1_OFFSET  0x0304  /* A/D Compare Data Register 1 */
#define RA_ADC_ADCMPSR0_OFFSET  0x0308  /* A/D Compare Status Register 0 */
#define RA_ADC_ADCMPSR1_OFFSET  0x030C  /* A/D Compare Status Register 1 */
#define RA_ADC_ADCMPSER_OFFSET  0x0310  /* A/D Compare Function Enable Register */
#define RA_ADC_ADCMPBNSR_OFFSET 0x0314  /* A/D Compare Window B Channel Select Register */

/* ADC Sample and Hold Registers */
#define RA_ADC_ADSHCR_OFFSET    0x0380  /* A/D Sample and Hold Control Register */
#define RA_ADC_ADSHMSR_OFFSET   0x0384  /* A/D Sample and Hold Mode Select Register */

/* ADC Internal Reference Voltage Monitor Registers */
#define RA_ADC_ADVMONCR_OFFSET  0x03C0  /* A/D Internal Reference Voltage Monitor Control Register */
#define RA_ADC_ADVMSR_OFFSET    0x03C4  /* A/D Internal Reference Voltage Monitor Status Register */

/* ADC Register Addresses ***************************************************/

#define RA8_ADC0_ADCSR          (RA8_ADC0_BASE + RA_ADC_ADCSR_OFFSET)
#define RA8_ADC0_ADANSE0        (RA8_ADC0_BASE + RA_ADC_ADANSE0_OFFSET)
#define RA8_ADC0_ADANSE1        (RA8_ADC0_BASE + RA_ADC_ADANSE1_OFFSET)
#define RA8_ADC0_ADADS0         (RA8_ADC0_BASE + RA_ADC_ADADS0_OFFSET)
#define RA8_ADC0_ADADS1         (RA8_ADC0_BASE + RA_ADC_ADADS1_OFFSET)
#define RA8_ADC0_ADADC          (RA8_ADC0_BASE + RA_ADC_ADADC_OFFSET)
#define RA8_ADC0_ADCER          (RA8_ADC0_BASE + RA_ADC_ADCER_OFFSET)
#define RA8_ADC0_ADSTRGR        (RA8_ADC0_BASE + RA_ADC_ADSTRGR_OFFSET)
#define RA8_ADC0_ADEXICR        (RA8_ADC0_BASE + RA_ADC_ADEXICR_OFFSET)
#define RA8_ADC0_ADANSB0        (RA8_ADC0_BASE + RA_ADC_ADANSB0_OFFSET)
#define RA8_ADC0_ADANSB1        (RA8_ADC0_BASE + RA_ADC_ADANSB1_OFFSET)
#define RA8_ADC0_ADDBLDR        (RA8_ADC0_BASE + RA_ADC_ADDBLDR_OFFSET)
#define RA8_ADC0_ADTSDR         (RA8_ADC0_BASE + RA_ADC_ADTSDR_OFFSET)
#define RA8_ADC0_ADOCDR         (RA8_ADC0_BASE + RA_ADC_ADOCDR_OFFSET)
#define RA8_ADC0_ADRD           (RA8_ADC0_BASE + RA_ADC_ADRD_OFFSET)

#define RA8_ADC0_ADDR(n)        (RA8_ADC0_BASE + RA_ADC_ADDR_OFFSET(n))

#define RA8_ADC1_ADCSR          (RA8_ADC1_BASE + RA_ADC_ADCSR_OFFSET)
#define RA8_ADC1_ADDR(n)        (RA8_ADC1_BASE + RA_ADC_ADDR_OFFSET(n))

/* ADC Control/Status Register (ADCSR) */

#define ADC_ADCSR_DBLANS_SHIFT  (0)
#define ADC_ADCSR_DBLANS_MASK   (0x1f << ADC_ADCSR_DBLANS_SHIFT)
#define ADC_ADCSR_GBADIE        (1 << 6)   /* Group B ADC Scan End Interrupt Enable */
#define ADC_ADCSR_DBLE          (1 << 7)   /* Double-triggered Mode Select */
#define ADC_ADCSR_EXTRG         (1 << 8)   /* External Trigger Enable */
#define ADC_ADCSR_TRGE          (1 << 9)   /* Trigger Start Enable */
#define ADC_ADCSR_ADIE          (1 << 12)  /* A/D Scan End Interrupt Enable */
#define ADC_ADCSR_ADCS_SHIFT    (13)
#define ADC_ADCSR_ADCS_MASK     (0x3 << ADC_ADCSR_ADCS_SHIFT)
#  define ADC_ADCSR_ADCS_SINGLE (0x0 << ADC_ADCSR_ADCS_SHIFT)  /* Single scan mode */
#  define ADC_ADCSR_ADCS_GROUP  (0x1 << ADC_ADCSR_ADCS_SHIFT)  /* Group scan mode */
#  define ADC_ADCSR_ADCS_CONT   (0x2 << ADC_ADCSR_ADCS_SHIFT)  /* Continuous scan mode */
#define ADC_ADCSR_ADST          (1 << 15)  /* A/D Conversion Start */

/* ADC Channel Select Register 0 (ADANSE0) */

#define ADC_ADANSE0_ANS0        (1 << 0)   /* AN000 */
#define ADC_ADANSE0_ANS1        (1 << 1)   /* AN001 */
#define ADC_ADANSE0_ANS2        (1 << 2)   /* AN002 */
#define ADC_ADANSE0_ANS3        (1 << 3)   /* AN003 */
#define ADC_ADANSE0_ANS4        (1 << 4)   /* AN004 */
#define ADC_ADANSE0_ANS5        (1 << 5)   /* AN005 */
#define ADC_ADANSE0_ANS6        (1 << 6)   /* AN006 */
#define ADC_ADANSE0_ANS7        (1 << 7)   /* AN007 */
#define ADC_ADANSE0_ANS8        (1 << 8)   /* AN008 */
#define ADC_ADANSE0_ANS9        (1 << 9)   /* AN009 */
#define ADC_ADANSE0_ANS10       (1 << 10)  /* AN010 */
#define ADC_ADANSE0_ANS11       (1 << 11)  /* AN011 */
#define ADC_ADANSE0_ANS12       (1 << 12)  /* AN012 */
#define ADC_ADANSE0_ANS13       (1 << 13)  /* AN013 */
#define ADC_ADANSE0_ANS14       (1 << 14)  /* AN014 */
#define ADC_ADANSE0_ANS15       (1 << 15)  /* AN015 */

/* ADC Channel Select Register 1 (ADANSE1) */

#define ADC_ADANSE1_ANS16       (1 << 0)   /* AN016 */
#define ADC_ADANSE1_ANS17       (1 << 1)   /* AN017 */
#define ADC_ADANSE1_ANS18       (1 << 2)   /* AN018 */
#define ADC_ADANSE1_ANS19       (1 << 3)   /* AN019 */
#define ADC_ADANSE1_ANS20       (1 << 4)   /* AN020 */
#define ADC_ADANSE1_ANS21       (1 << 5)   /* AN021 */
#define ADC_ADANSE1_ANS22       (1 << 6)   /* AN022 */
#define ADC_ADANSE1_ANS23       (1 << 7)   /* AN023 */
#define ADC_ADANSE1_ANS24       (1 << 8)   /* AN024 */
#define ADC_ADANSE1_ANS25       (1 << 9)   /* AN025 */
#define ADC_ADANSE1_ANS26       (1 << 10)  /* AN026 */
#define ADC_ADANSE1_ANS27       (1 << 11)  /* AN027 */
#define ADC_ADANSE1_ANS28       (1 << 12)  /* AN028 */

/* ADC Control Extended Register (ADCER) */

#define ADC_ADCER_ADPRC_SHIFT   (1)
#define ADC_ADCER_ADPRC_MASK    (0x3 << ADC_ADCER_ADPRC_SHIFT)
#  define ADC_ADCER_ADPRC_AUTO  (0x0 << ADC_ADCER_ADPRC_SHIFT)  /* Automatic clearing */
#  define ADC_ADCER_ADPRC_CLRRD (0x1 << ADC_ADCER_ADPRC_SHIFT)  /* Clear after reading */
#  define ADC_ADCER_ADPRC_NOCLR (0x2 << ADC_ADCER_ADPRC_SHIFT)  /* Automatic clearing disabled */
#define ADC_ADCER_ACE           (1 << 5)   /* A/D Data Register Automatic Clearing Enable */
#define ADC_ADCER_DIAGM         (1 << 7)   /* Self-diagnosis Mode Select */
#define ADC_ADCER_DIAGLD        (1 << 8)   /* Self-diagnosis Conversion Voltage Select */
#define ADC_ADCER_DIAGVAL_SHIFT (9)
#define ADC_ADCER_DIAGVAL_MASK  (0x3 << ADC_ADCER_DIAGVAL_SHIFT)
#define ADC_ADCER_ADRFMT        (1 << 15)  /* A/D Data Register Format Select */

/* ADC Conversion Start Trigger Register (ADSTRGR) */

#define ADC_ADSTRGR_TRSB_SHIFT  (0)
#define ADC_ADSTRGR_TRSB_MASK   (0x3f << ADC_ADSTRGR_TRSB_SHIFT)
#define ADC_ADSTRGR_TRSA_SHIFT  (8)
#define ADC_ADSTRGR_TRSA_MASK   (0x3f << ADC_ADSTRGR_TRSA_SHIFT)

/* Maximum number of ADC channels */

#define RA8_ADC_MAX_CHANNELS    29

/* ADC Channel definitions */

#define RA8_ADC_CHANNEL_AN000   0
#define RA8_ADC_CHANNEL_AN001   1
#define RA8_ADC_CHANNEL_AN002   2
#define RA8_ADC_CHANNEL_AN003   3
#define RA8_ADC_CHANNEL_AN004   4
#define RA8_ADC_CHANNEL_AN005   5
#define RA8_ADC_CHANNEL_AN006   6
#define RA8_ADC_CHANNEL_AN007   7
#define RA8_ADC_CHANNEL_AN008   8
#define RA8_ADC_CHANNEL_AN009   9
#define RA8_ADC_CHANNEL_AN010   10
#define RA8_ADC_CHANNEL_AN011   11
#define RA8_ADC_CHANNEL_AN012   12
#define RA8_ADC_CHANNEL_AN013   13
#define RA8_ADC_CHANNEL_AN014   14
#define RA8_ADC_CHANNEL_AN015   15
#define RA8_ADC_CHANNEL_AN016   16
#define RA8_ADC_CHANNEL_AN017   17
#define RA8_ADC_CHANNEL_AN018   18
#define RA8_ADC_CHANNEL_AN019   19
#define RA8_ADC_CHANNEL_AN020   20
#define RA8_ADC_CHANNEL_AN021   21
#define RA8_ADC_CHANNEL_AN022   22
#define RA8_ADC_CHANNEL_AN023   23
#define RA8_ADC_CHANNEL_AN024   24
#define RA8_ADC_CHANNEL_AN025   25
#define RA8_ADC_CHANNEL_AN026   26
#define RA8_ADC_CHANNEL_AN027   27
#define RA8_ADC_CHANNEL_AN028   28

/* Special ADC channels */

#define RA8_ADC_CHANNEL_TEMPERATURE  (-3)
#define RA8_ADC_CHANNEL_VREF         (-2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* ADC Resolution enumeration */

enum ra8_adc_resolution_e
{
  RA8_ADC_RESOLUTION_12BIT = 0,  /* 12-bit resolution */
  RA8_ADC_RESOLUTION_10BIT = 1,  /* 10-bit resolution */
  RA8_ADC_RESOLUTION_8BIT  = 2,  /* 8-bit resolution */
  RA8_ADC_RESOLUTION_14BIT = 3,  /* 14-bit resolution */
  RA8_ADC_RESOLUTION_16BIT = 4,  /* 16-bit resolution */
};

/* ADC Scan Mode enumeration */

enum ra8_adc_mode_e
{
  RA8_ADC_MODE_SINGLE_SCAN = 0,  /* Single scan mode */
  RA8_ADC_MODE_GROUP_SCAN  = 1,  /* Group scan mode */
  RA8_ADC_MODE_CONTINUOUS  = 2,  /* Continuous scan mode */
};

/* ADC Trigger Source enumeration */

enum ra8_adc_trigger_e
{
  RA8_ADC_TRIGGER_SOFTWARE = 0,    /* Software trigger */
  RA8_ADC_TRIGGER_SYNC_ELC = 2,    /* Synchronous trigger via ELC */
  RA8_ADC_TRIGGER_ASYNC_EXT = 3,   /* External asynchronous trigger */
};

/* ADC Data Alignment enumeration */

enum ra8_adc_alignment_e
{
  RA8_ADC_ALIGNMENT_RIGHT = 0,  /* Right-aligned data */
  RA8_ADC_ALIGNMENT_LEFT  = 1,  /* Left-aligned data */
};

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_ADC_H */
