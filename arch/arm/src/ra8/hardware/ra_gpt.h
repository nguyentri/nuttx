/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_gpt.h
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

#ifndef __ARCH_ARM_SRC_RA_HARDWARE_RA_GPT_H
#define __ARCH_ARM_SRC_RA_HARDWARE_RA_GPT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "hardware/ra_memorymap.h"

#ifndef __ASSEMBLY__
#include "arm_internal.h"  /* For putreg32/getreg32 */
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPT Channel Base Address Calculator *************************************/

#define RA_GPT_CH_STRIDE              0x100u

#define RA_GPT_BASE(ch)               (R_GPT0_BASE + ((uint32_t)(ch) * RA_GPT_CH_STRIDE))

/* Channel classification */
#define RA_GPT_IS_32BIT(ch)           ((unsigned)(ch) <= 5u)
#define RA_GPT_IS_16BIT(ch)           ((unsigned)(ch) >= 10u && (unsigned)(ch) <= 13u)

/* GPT Register Offsets *************************************************/

#define RA_GPT_GTWP_OFFSET     0x0000  /* General PWM Timer Write-Protection Register */
#define RA_GPT_GTSTR_OFFSET    0x0004  /* General PWM Timer Software Start Register */
#define RA_GPT_GTSTP_OFFSET    0x0008  /* General PWM Timer Software Stop Register */
#define RA_GPT_GTCLR_OFFSET    0x000c  /* General PWM Timer Software Clear Register */
#define RA_GPT_GTSSR_OFFSET    0x0010  /* General PWM Timer Start Source Select Register */
#define RA_GPT_GTPSR_OFFSET    0x0014  /* General PWM Timer Stop Source Select Register */
#define RA_GPT_GTCSR_OFFSET    0x0018  /* General PWM Timer Clear Source Select Register */
#define RA_GPT_GTUPSR_OFFSET   0x001c  /* General PWM Timer Up Count Source Select Register */
#define RA_GPT_GTDNSR_OFFSET   0x0020  /* General PWM Timer Down Count Source Select Register */
#define RA_GPT_GTICASR_OFFSET  0x0024  /* General PWM Timer Input Capture Source Select Register A */
#define RA_GPT_GTICBSR_OFFSET  0x0028  /* General PWM Timer Input Capture Source Select Register B */
#define RA_GPT_GTCR_OFFSET     0x002c  /* General PWM Timer Control Register */
#define RA_GPT_GTUDDTYC_OFFSET 0x0030  /* General PWM Timer Count Direction and Duty Setting Register */
#define RA_GPT_GTIOR_OFFSET    0x0034  /* General PWM Timer I/O Control Register */
#define RA_GPT_GTINTAD_OFFSET  0x0038  /* General PWM Timer Interrupt Output Setting Register */
#define RA_GPT_GTST_OFFSET     0x003c  /* General PWM Timer Status Register */
#define RA_GPT_GTBER_OFFSET    0x0040  /* General PWM Timer Buffer Enable Register */
#define RA_GPT_GTITC_OFFSET    0x0044  /* General PWM Timer Interrupt and A/D Converter Start Request Skipping Setting Register */
#define RA_GPT_GTCNT_OFFSET    0x0048  /* General PWM Timer Counter */
#define RA_GPT_GTCCRA_OFFSET   0x004c  /* General PWM Timer Compare Capture Register A */
#define RA_GPT_GTCCRB_OFFSET   0x0050  /* General PWM Timer Compare Capture Register B */
#define RA_GPT_GTCCRC_OFFSET   0x0054  /* General PWM Timer Compare Capture Register C */
#define RA_GPT_GTCCRD_OFFSET   0x0058  /* General PWM Timer Compare Capture Register D */
#define RA_GPT_GTCCRE_OFFSET   0x005c  /* General PWM Timer Compare Capture Register E */
#define RA_GPT_GTCCRF_OFFSET   0x0060  /* General PWM Timer Compare Capture Register F */
#define RA_GPT_GTPR_OFFSET     0x0064  /* General PWM Timer Cycle Setting Register */
#define RA_GPT_GTPBR_OFFSET    0x0068  /* General PWM Timer Cycle Setting Buffer Register */
#define RA_GPT_GTPDBR_OFFSET   0x006c  /* General PWM Timer Cycle Setting Double-Buffer Register */
#define RA_GPT_GTADTRA_OFFSET  0x0070  /* A/D Converter Start Request Timing Register A */
#define RA_GPT_GTADTBRA_OFFSET 0x0074  /* A/D Converter Start Request Timing Buffer Register A */
#define RA_GPT_GTADTDBRA_OFFSET 0x0078 /* A/D Converter Start Request Timing Double-Buffer Register A */
#define RA_GPT_GTADTRB_OFFSET  0x007c  /* A/D Converter Start Request Timing Register B */
#define RA_GPT_GTADTBRB_OFFSET 0x0080  /* A/D Converter Start Request Timing Buffer Register B */
#define RA_GPT_GTADTDBRB_OFFSET 0x0084 /* A/D Converter Start Request Timing Double-Buffer Register B */
#define RA_GPT_GTDTCR_OFFSET   0x0088  /* General PWM Timer Dead Time Control Register */
#define RA_GPT_GTDVU_OFFSET    0x008c  /* General PWM Timer Dead Time Value Register U */
#define RA_GPT_GTDVD_OFFSET    0x0090  /* General PWM Timer Dead Time Value Register D */
#define RA_GPT_GTDBU_OFFSET    0x0094  /* General PWM Timer Dead Time Buffer Register U */
#define RA_GPT_GTDBD_OFFSET    0x0098  /* General PWM Timer Dead Time Buffer Register D */
#define RA_GPT_GTSOS_OFFSET    0x009c  /* General PWM Timer Output Protection Function Status Register */
#define RA_GPT_GTSOTR_OFFSET   0x00a0  /* General PWM Timer Output Protection Function Temporary Release Register */

/* Additional register offsets for timer functionality */
#define RA_GPT_GTUD_OFFSET     0x00b0  /* General PWM Timer Count Direction Register */
#define RA_GPT_GTIER_OFFSET    0x00b4  /* General PWM Timer Interrupt Enable Register */

/* Register Address Calculation Macros *********************************/

#define RA_GPT_REG(ch, off)           (RA_GPT_BASE(ch) + (off))

/* Named register address helpers */
#define RA_GPT_GTWP(ch)               RA_GPT_REG(ch, RA_GPT_GTWP_OFFSET)
#define RA_GPT_GTSTR(ch)              RA_GPT_REG(ch, RA_GPT_GTSTR_OFFSET)
#define RA_GPT_GTSTP(ch)              RA_GPT_REG(ch, RA_GPT_GTSTP_OFFSET)
#define RA_GPT_GTCLR(ch)              RA_GPT_REG(ch, RA_GPT_GTCLR_OFFSET)
#define RA_GPT_GTCR(ch)               RA_GPT_REG(ch, RA_GPT_GTCR_OFFSET)
#define RA_GPT_GTCNT(ch)              RA_GPT_REG(ch, RA_GPT_GTCNT_OFFSET)
#define RA_GPT_GTPR(ch)               RA_GPT_REG(ch, RA_GPT_GTPR_OFFSET)
#define RA_GPT_GTPBR(ch)              RA_GPT_REG(ch, RA_GPT_GTPBR_OFFSET)
#define RA_GPT_GTST_REG(ch)           RA_GPT_REG(ch, RA_GPT_GTST_OFFSET)
#define RA_GPT_GTINTAD(ch)            RA_GPT_REG(ch, RA_GPT_GTINTAD_OFFSET)
#define RA_GPT_GTIOR(ch)              RA_GPT_REG(ch, RA_GPT_GTIOR_OFFSET)
#define RA_GPT_GTCCRA(ch)             RA_GPT_REG(ch, RA_GPT_GTCCRA_OFFSET)
#define RA_GPT_GTCCRB(ch)             RA_GPT_REG(ch, RA_GPT_GTCCRB_OFFSET)
#define RA_GPT_GTCCRC(ch)             RA_GPT_REG(ch, RA_GPT_GTCCRC_OFFSET)
#define RA_GPT_GTCCRE(ch)             RA_GPT_REG(ch, RA_GPT_GTCCRE_OFFSET)
#define RA_GPT_GTUDDTYC(ch)           RA_GPT_REG(ch, RA_GPT_GTUDDTYC_OFFSET)
#define RA_GPT_GTBER(ch)              RA_GPT_REG(ch, RA_GPT_GTBER_OFFSET)

/* GPT Register Definitions *********************************************/

/* GTWP - General PWM Timer Write-Protection Register */

#define GPT_GTWP_WP                     (1 << 0)  /* Register Write Disable */
#define GPT_GTWP_STRWP                  (1 << 1)  /* GTSTR.CSTRT Bit Write Disable */
#define GPT_GTWP_STPWP                  (1 << 2)  /* GTSTP.CSTOP Bit Write Disable */
#define GPT_GTWP_CLRWP                  (1 << 3)  /* GTCLR.CCLR Bit Write Disable */
#define GPT_GTWP_CMNWP                  (1 << 4)  /* Common Register Write Disabled */
#define GPT_GTWP_PRKEY_SHIFT            (8)       /* GTWP Key Code */
#define GPT_GTWP_PRKEY_MASK             (0xff << GPT_GTWP_PRKEY_SHIFT)
#define GPT_GTWP_PRKEY                  (0xa5 << GPT_GTWP_PRKEY_SHIFT)

/* Timer initialization compatibility definitions */
#define RA_GPT_GTWP_WP_KEY              0xa5      /* Write protection key */
#define RA_GPT_GTWP_WP_SHIFT            8         /* Write protection shift */

/* Write protection control macros */
#define RA_GPT_WP_DISABLE(ch)         putreg32(GPT_GTWP_PRKEY | GPT_GTWP_WP, RA_GPT_GTWP(ch))
#define RA_GPT_WP_ENABLE(ch)          putreg32(GPT_GTWP_PRKEY, RA_GPT_GTWP(ch))

/* GTCR - General PWM Timer Control Register */

#define GPT_GTCR_CST                    (1 << 0)  /* Count Start */
#define GPT_GTCR_MD_SHIFT               (16)      /* Mode Select */
#define GPT_GTCR_MD_MASK                (7 << GPT_GTCR_MD_SHIFT)
#define GPT_GTCR_MD_SAW_WAVE_UP         (0 << GPT_GTCR_MD_SHIFT)  /* Saw-wave PWM mode (up-counting) */
#define GPT_GTCR_MD_SAW_WAVE_DOWN       (1 << GPT_GTCR_MD_SHIFT)  /* Saw-wave PWM mode (down-counting) */
#define GPT_GTCR_MD_TRIANGLE_WAVE       (4 << GPT_GTCR_MD_SHIFT)  /* Triangle-wave PWM mode */
#define GPT_GTCR_MD_ONE_SHOT_PULSE      (3 << GPT_GTCR_MD_SHIFT)  /* One-shot pulse mode */
#define GPT_GTCR_TPCS_SHIFT             (24)      /* Timer Prescaler Select */
#define GPT_GTCR_TPCS_MASK              (7 << GPT_GTCR_TPCS_SHIFT)
#define GPT_GTCR_TPCS_PCLKD_1           (0 << GPT_GTCR_TPCS_SHIFT)  /* PCLKD */
#define GPT_GTCR_TPCS_PCLKD_4           (1 << GPT_GTCR_TPCS_SHIFT)  /* PCLKD/4 */
#define GPT_GTCR_TPCS_PCLKD_16          (2 << GPT_GTCR_TPCS_SHIFT)  /* PCLKD/16 */
#define GPT_GTCR_TPCS_PCLKD_64          (3 << GPT_GTCR_TPCS_SHIFT)  /* PCLKD/64 */
#define GPT_GTCR_TPCS_PCLKD_256         (4 << GPT_GTCR_TPCS_SHIFT)  /* PCLKD/256 */
#define GPT_GTCR_TPCS_PCLKD_1024        (5 << GPT_GTCR_TPCS_SHIFT)  /* PCLKD/1024 */

/* GTIOR - General PWM Timer I/O Control Register */

#define GPT_GTIOR_GTIOA_SHIFT           (0)       /* GTIOA Pin Function Select */
#define GPT_GTIOR_GTIOA_MASK            (0x1f << GPT_GTIOR_GTIOA_SHIFT)
#define GPT_GTIOR_GTIOA_INITIAL_LOW     (0x09 << GPT_GTIOR_GTIOA_SHIFT)  /* Initial low, Compare match high */
#define GPT_GTIOR_GTIOA_INITIAL_HIGH    (0x0a << GPT_GTIOR_GTIOA_SHIFT)  /* Initial high, Compare match low */
#define GPT_GTIOR_GTIOA_TOGGLE          (0x03 << GPT_GTIOR_GTIOA_SHIFT)  /* Toggle on compare match */

#define GPT_GTIOR_GTIOB_SHIFT           (16)      /* GTIOB Pin Function Select */
#define GPT_GTIOR_GTIOB_MASK            (0x1f << GPT_GTIOR_GTIOB_SHIFT)
#define GPT_GTIOR_GTIOB_INITIAL_LOW     (0x09 << GPT_GTIOR_GTIOB_SHIFT)  /* Initial low, Compare match high */
#define GPT_GTIOR_GTIOB_INITIAL_HIGH    (0x0a << GPT_GTIOR_GTIOB_SHIFT)  /* Initial high, Compare match low */
#define GPT_GTIOR_GTIOB_TOGGLE          (0x03 << GPT_GTIOR_GTIOB_SHIFT)  /* Toggle on compare match */

/* GTINTAD - General PWM Timer Interrupt Output Setting Register */

#define GPT_GTINTAD_GTINTA              (1 << 0)  /* GTCCRA Compare Match/Input Capture Interrupt Enable */
#define GPT_GTINTAD_GTINTB              (1 << 1)  /* GTCCRB Compare Match/Input Capture Interrupt Enable */
#define GPT_GTINTAD_GTINTC              (1 << 2)  /* GTCCRC Compare Match Interrupt Enable */
#define GPT_GTINTAD_GTINTD              (1 << 3)  /* GTCCRD Compare Match Interrupt Enable */
#define GPT_GTINTAD_GTINTE              (1 << 4)  /* GTCCRE Compare Match Interrupt Enable */
#define GPT_GTINTAD_GTINTF              (1 << 5)  /* GTCCRF Compare Match Interrupt Enable */
#define GPT_GTINTAD_GTINTV              (1 << 6)  /* Overflow Interrupt Enable */
#define GPT_GTINTAD_GTINTU              (1 << 7)  /* Underflow Interrupt Enable */

/* GTST - General PWM Timer Status Register */

#define GPT_GTST_TCFA                   (1 << 0)  /* Input Capture/Compare Match Flag A */
#define GPT_GTST_TCFB                   (1 << 1)  /* Input Capture/Compare Match Flag B */
#define GPT_GTST_TCFC                   (1 << 2)  /* Input Compare Match Flag C */
#define GPT_GTST_TCFD                   (1 << 3)  /* Input Compare Match Flag D */
#define GPT_GTST_TCFE                   (1 << 4)  /* Input Compare Match Flag E */
#define GPT_GTST_TCFF                   (1 << 5)  /* Input Compare Match Flag F */
#define GPT_GTST_TCFPO                  (1 << 6)  /* Overflow Flag */
#define GPT_GTST_TCFPU                  (1 << 7)  /* Underflow Flag */

/* GPT Channel Configuration Structure */

struct ra_gpt_config_s
{
  uint32_t base;                   /* GPT peripheral base address */
  uint32_t pclkd_frequency;        /* PCLKD frequency in Hz */
  uint8_t  channel;                /* GPT channel (0-13) */
  uint8_t  irq;                    /* Interrupt request number */
};

/* GPT Device Structure */

struct ra_gpt_dev_s
{
  const struct ra_gpt_config_s *config;  /* GPT configuration */
  uint32_t frequency;                      /* PWM frequency */
  uint32_t period;                         /* PWM period in timer counts */
  bool started;                            /* PWM started flag */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA_HARDWARE_RA_GPT_H */
