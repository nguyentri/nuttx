/****************************************************************************
 * arch/arm/include/ra8/ra8e1_irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_RA_RA8E1_IRQ_H
#define __ARCH_ARM_INCLUDE_RA_RA8E1_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/ra8/chip.h>
/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* Total number of IRQ numbers */
#define RA_IRQ_IELSR0                           (RA_IRQ_FIRST + 0)   /* 0:  Event selected in the ICU.IELSR0 register */
#define RA_IRQ_IELSR1                           (RA_IRQ_FIRST + 1)   /* 1:  Event selected in the ICU.IELSR1 register */
#define RA_IRQ_IELSR2                           (RA_IRQ_FIRST + 2)   /* 2:  Event selected in the ICU.IELSR2 register */
#define RA_IRQ_IELSR3                           (RA_IRQ_FIRST + 3)   /* 3:  Event selected in the ICU.IELSR3 register */
#define RA_IRQ_IELSR4                           (RA_IRQ_FIRST + 4)   /* 4:  Event selected in the ICU.IELSR4 register */
#define RA_IRQ_IELSR5                           (RA_IRQ_FIRST + 5)   /* 5:  Event selected in the ICU.IELSR5 register */
#define RA_IRQ_IELSR6                           (RA_IRQ_FIRST + 6)   /* 6:  Event selected in the ICU.IELSR6 register */
#define RA_IRQ_IELSR7                           (RA_IRQ_FIRST + 7)   /* 7:  Event selected in the ICU.IELSR7 register */
#define RA_IRQ_IELSR8                           (RA_IRQ_FIRST + 8)   /* 8:  Event selected in the ICU.IELSR8 register */
#define RA_IRQ_IELSR9                           (RA_IRQ_FIRST + 9)   /* 9:  Event selected in the ICU.IELSR9 register */
#define RA_IRQ_IELSR10                          (RA_IRQ_FIRST + 10)  /* 10:  Event selected in the ICU.IELSR10 register */
#define RA_IRQ_IELSR11                          (RA_IRQ_FIRST + 11)  /* 11:  Event selected in the ICU.IELSR11 register */
#define RA_IRQ_IELSR12                          (RA_IRQ_FIRST + 12)  /* 12:  Event selected in the ICU.IELSR12 register */
#define RA_IRQ_IELSR13                          (RA_IRQ_FIRST + 13)  /* 13:  Event selected in the ICU.IELSR13 register */
#define RA_IRQ_IELSR14                          (RA_IRQ_FIRST + 14)  /* 14:  Event selected in the ICU.IELSR14 register */
#define RA_IRQ_IELSR15                          (RA_IRQ_FIRST + 15)  /* 15:  Event selected in the ICU.IELSR15 register */
#define RA_IRQ_IELSR16                          (RA_IRQ_FIRST + 16)  /* 16:  Event selected in the ICU.IELSR16 register */
#define RA_IRQ_IELSR17                          (RA_IRQ_FIRST + 17)  /* 17:  Event selected in the ICU.IELSR17 register */
#define RA_IRQ_IELSR18                          (RA_IRQ_FIRST + 18)  /* 18:  Event selected in the ICU.IELSR18 register */
#define RA_IRQ_IELSR19                          (RA_IRQ_FIRST + 19)  /* 19:  Event selected in the ICU.IELSR19 register */
#define RA_IRQ_IELSR20                          (RA_IRQ_FIRST + 20)  /* 20:  Event selected in the ICU.IELSR20 register */
#define RA_IRQ_IELSR21                          (RA_IRQ_FIRST + 21)  /* 21:  Event selected in the ICU.IELSR21 register */
#define RA_IRQ_IELSR22                          (RA_IRQ_FIRST + 22)  /* 22:  Event selected in the ICU.IELSR22 register */
#define RA_IRQ_IELSR23                          (RA_IRQ_FIRST + 23)  /* 23:  Event selected in the ICU.IELSR23 register */
#define RA_IRQ_IELSR24                          (RA_IRQ_FIRST + 24)  /* 24:  Event selected in the ICU.IELSR24 register */
#define RA_IRQ_IELSR25                          (RA_IRQ_FIRST + 25)  /* 25:  Event selected in the ICU.IELSR25 register */
#define RA_IRQ_IELSR26                          (RA_IRQ_FIRST + 26)  /* 26:  Event selected in the ICU.IELSR26 register */
#define RA_IRQ_IELSR27                          (RA_IRQ_FIRST + 27)  /* 27:  Event selected in the ICU.IELSR27 register */
#define RA_IRQ_IELSR28                          (RA_IRQ_FIRST + 28)  /* 28:  Event selected in the ICU.IELSR28 register */
#define RA_IRQ_IELSR29                          (RA_IRQ_FIRST + 29)  /* 29:  Event selected in the ICU.IELSR29 register */
#define RA_IRQ_IELSR30                          (RA_IRQ_FIRST + 30)  /* 30:  Event selected in the ICU.IELSR30 register */
#define RA_IRQ_IELSR31                          (RA_IRQ_FIRST + 31)  /* 31:  Event selected in the ICU.IELSR31 register */
#define RA_IRQ_IELSR32                          (RA_IRQ_FIRST + 32)  /* 32:  Event selected in the ICU.IELSR32 register */
#define RA_IRQ_IELSR33                          (RA_IRQ_FIRST + 33)  /* 33:  Event selected in the ICU.IELSR33 register */
#define RA_IRQ_IELSR34                          (RA_IRQ_FIRST + 34)  /* 34:  Event selected in the ICU.IELSR34 register */
#define RA_IRQ_IELSR35                          (RA_IRQ_FIRST + 35)  /* 35:  Event selected in the ICU.IELSR35 register */
#define RA_IRQ_IELSR36                          (RA_IRQ_FIRST + 36)  /* 36:  Event selected in the ICU.IELSR36 register */
#define RA_IRQ_IELSR37                          (RA_IRQ_FIRST + 37)  /* 37:  Event selected in the ICU.IELSR37 register */
#define RA_IRQ_IELSR38                          (RA_IRQ_FIRST + 38)  /* 38:  Event selected in the ICU.IELSR38 register */
#define RA_IRQ_IELSR39                          (RA_IRQ_FIRST + 39)  /* 39:  Event selected in the ICU.IELSR39 register */
#define RA_IRQ_IELSR40                          (RA_IRQ_FIRST + 40)  /* 40:  Event selected in the ICU.IELSR40 register */
#define RA_IRQ_IELSR41                          (RA_IRQ_FIRST + 41)  /* 41:  Event selected in the ICU.IELSR41 register */
#define RA_IRQ_IELSR42                          (RA_IRQ_FIRST + 42)  /* 42:  Event selected in the ICU.IELSR42 register */
#define RA_IRQ_IELSR43                          (RA_IRQ_FIRST + 43)  /* 43:  Event selected in the ICU.IELSR43 register */
#define RA_IRQ_IELSR44                          (RA_IRQ_FIRST + 44)  /* 44:  Event selected in the ICU.IELSR44 register */
#define RA_IRQ_IELSR45                          (RA_IRQ_FIRST + 45)  /* 45:  Event selected in the ICU.IELSR45 register */
#define RA_IRQ_IELSR46                          (RA_IRQ_FIRST + 46)  /* 46:  Event selected in the ICU.IELSR46 register */
#define RA_IRQ_IELSR47                          (RA_IRQ_FIRST + 47)  /* 47:  Event selected in the ICU.IELSR47 register */
#define RA_IRQ_IELSR48                          (RA_IRQ_FIRST + 48)  /* 48:  Event selected in the ICU.IELSR48 register */
#define RA_IRQ_IELSR49                          (RA_IRQ_FIRST + 49)  /* 49:  Event selected in the ICU.IELSR49 register */
#define RA_IRQ_IELSR50                          (RA_IRQ_FIRST + 50)  /* 50:  Event selected in the ICU.IELSR50 register */
#define RA_IRQ_IELSR51                          (RA_IRQ_FIRST + 51)  /* 51:  Event selected in the ICU.IELSR51 register */
#define RA_IRQ_IELSR52                          (RA_IRQ_FIRST + 52)  /* 52:  Event selected in the ICU.IELSR52 register */
#define RA_IRQ_IELSR53                          (RA_IRQ_FIRST + 53)  /* 53:  Event selected in the ICU.IELSR53 register */
#define RA_IRQ_IELSR54                          (RA_IRQ_FIRST + 54)  /* 54:  Event selected in the ICU.IELSR54 register */
#define RA_IRQ_IELSR55                          (RA_IRQ_FIRST + 55)  /* 55:  Event selected in the ICU.IELSR55 register */
#define RA_IRQ_IELSR56                          (RA_IRQ_FIRST + 56)  /* 56:  Event selected in the ICU.IELSR56 register */
#define RA_IRQ_IELSR57                          (RA_IRQ_FIRST + 57)  /* 57:  Event selected in the ICU.IELSR57 register */
#define RA_IRQ_IELSR58                          (RA_IRQ_FIRST + 58)  /* 58:  Event selected in the ICU.IELSR58 register */
#define RA_IRQ_IELSR59                          (RA_IRQ_FIRST + 59)  /* 59:  Event selected in the ICU.IELSR59 register */
#define RA_IRQ_IELSR60                          (RA_IRQ_FIRST + 60)  /* 60:  Event selected in the ICU.IELSR60 register */
#define RA_IRQ_IELSR61                          (RA_IRQ_FIRST + 61)  /* 61:  Event selected in the ICU.IELSR61 register */
#define RA_IRQ_IELSR62                          (RA_IRQ_FIRST + 62)  /* 62:  Event selected in the ICU.IELSR62 register */
#define RA_IRQ_IELSR63                          (RA_IRQ_FIRST + 63)  /* 63:  Event selected in the ICU.IELSR63 register */
#define RA_IRQ_IELSR64                          (RA_IRQ_FIRST + 64)  /* 64:  Event selected in the ICU.IELSR64 register */
#define RA_IRQ_IELSR65                          (RA_IRQ_FIRST + 65)  /* 65:  Event selected in the ICU.IELSR65 register */
#define RA_IRQ_IELSR66                          (RA_IRQ_FIRST + 66)  /* 66:  Event selected in the ICU.IELSR66 register */
#define RA_IRQ_IELSR67                          (RA_IRQ_FIRST + 67)  /* 67:  Event selected in the ICU.IELSR67 register */
#define RA_IRQ_IELSR68                          (RA_IRQ_FIRST + 68)  /* 68:  Event selected in the ICU.IELSR68 register */
#define RA_IRQ_IELSR69                          (RA_IRQ_FIRST + 69)  /* 69:  Event selected in the ICU.IELSR69 register */
#define RA_IRQ_IELSR70                          (RA_IRQ_FIRST + 70)  /* 70:  Event selected in the ICU.IELSR70 register */
#define RA_IRQ_IELSR71                          (RA_IRQ_FIRST + 71)  /* 71:  Event selected in the ICU.IELSR71 register */
#define RA_IRQ_IELSR72                          (RA_IRQ_FIRST + 72)  /* 72:  Event selected in the ICU.IELSR72 register */
#define RA_IRQ_IELSR73                          (RA_IRQ_FIRST + 73)  /* 73:  Event selected in the ICU.IELSR73 register */
#define RA_IRQ_IELSR74                          (RA_IRQ_FIRST + 74)  /* 74:  Event selected in the ICU.IELSR74 register */
#define RA_IRQ_IELSR75                          (RA_IRQ_FIRST + 75)  /* 75:  Event selected in the ICU.IELSR75 register */
#define RA_IRQ_IELSR76                          (RA_IRQ_FIRST + 76)  /* 76:  Event selected in the ICU.IELSR76 register */
#define RA_IRQ_IELSR77                          (RA_IRQ_FIRST + 77)  /* 77:  Event selected in the ICU.IELSR77 register */
#define RA_IRQ_IELSR78                          (RA_IRQ_FIRST + 78)  /* 78:  Event selected in the ICU.IELSR78 register */
#define RA_IRQ_IELSR79                          (RA_IRQ_FIRST + 79)  /* 79:  Event selected in the ICU.IELSR79 register */
#define RA_IRQ_IELSR80                          (RA_IRQ_FIRST + 80)  /* 80:  Event selected in the ICU.IELSR80 register */
#define RA_IRQ_IELSR81                          (RA_IRQ_FIRST + 81)  /* 81:  Event selected in the ICU.IELSR81 register */
#define RA_IRQ_IELSR82                          (RA_IRQ_FIRST + 82)  /* 82:  Event selected in the ICU.IELSR82 register */
#define RA_IRQ_IELSR83                          (RA_IRQ_FIRST + 83)  /* 83:  Event selected in the ICU.IELSR83 register */
#define RA_IRQ_IELSR84                          (RA_IRQ_FIRST + 84)  /* 84:  Event selected in the ICU.IELSR84 register */
#define RA_IRQ_IELSR85                          (RA_IRQ_FIRST + 85)  /* 85:  Event selected in the ICU.IELSR85 register */
#define RA_IRQ_IELSR86                          (RA_IRQ_FIRST + 86)  /* 86:  Event selected in the ICU.IELSR86 register */
#define RA_IRQ_IELSR87                          (RA_IRQ_FIRST + 87)  /* 87:  Event selected in the ICU.IELSR87 register */
#define RA_IRQ_IELSR88                          (RA_IRQ_FIRST + 88)  /* 88:  Event selected in the ICU.IELSR88 register */
#define RA_IRQ_IELSR89                          (RA_IRQ_FIRST + 89)  /* 89:  Event selected in the ICU.IELSR89 register */
#define RA_IRQ_IELSR90                          (RA_IRQ_FIRST + 90)  /* 90:  Event selected in the ICU.IELSR90 register */
#define RA_IRQ_IELSR91                          (RA_IRQ_FIRST + 91)  /* 91:  Event selected in the ICU.IELSR91 register */
#define RA_IRQ_IELSR92                          (RA_IRQ_FIRST + 92)  /* 92:  Event selected in the ICU.IELSR92 register */
#define RA_IRQ_IELSR93                          (RA_IRQ_FIRST + 93)  /* 93:  Event selected in the ICU.IELSR93 register */
#define RA_IRQ_IELSR94                          (RA_IRQ_FIRST + 94)  /* 94:  Event selected in the ICU.IELSR94 register */
#define RA_IRQ_IELSR95                          (RA_IRQ_FIRST + 95)  /* 95:  Event selected in the ICU.IELSR95 register */

/* RA8E1 has 96 external interrupt vectors + 16 Cortex-M85 core vectors = 112 total but extend to maximum event */
# define RA_IRQ_NEXTINT                         96

/* Event number definitions based on RA8E1 HWM Event Table 13.4 */
#define  RA_EL_EVENT_ICU_IRQ0                   (0x001)  /* External pin interrupt 0 - Event 0x001 */
#define  RA_EL_EVENT_ICU_IRQ1                   (0x002)  /* External pin interrupt 1 - Event 0x002 */
#define  RA_EL_EVENT_ICU_IRQ2                   (0x003)  /* External pin interrupt 2 - Event 0x003 */
#define  RA_EL_EVENT_ICU_IRQ3                   (0x004)  /* External pin interrupt 3 - Event 0x004 */
#define  RA_EL_EVENT_ICU_IRQ4                   (0x005)  /* External pin interrupt 4 - Event 0x005 */
#define  RA_EL_EVENT_ICU_IRQ5                   (0x006)  /* External pin interrupt 5 - Event 0x006 */
#define  RA_EL_EVENT_ICU_IRQ6                   (0x007)  /* External pin interrupt 6 - Event 0x007 */
#define  RA_EL_EVENT_ICU_IRQ7                   (0x008)  /* External pin interrupt 7 - Event 0x008 */
#define  RA_EL_EVENT_ICU_IRQ8                   (0x009)  /* External pin interrupt 8 - Event 0x009 */
#define  RA_EL_EVENT_ICU_IRQ9                   (0x00A)  /* External pin interrupt 9 - Event 0x00A */
#define  RA_EL_EVENT_ICU_IRQ10                  (0x00B)  /* External pin interrupt 10 - Event 0x00B */
#define  RA_EL_EVENT_ICU_IRQ11                  (0x00C)  /* External pin interrupt 11 - Event 0x00C */
#define  RA_EL_EVENT_ICU_IRQ12                  (0x00D)  /* External pin interrupt 12 - Event 0x00D */
#define  RA_EL_EVENT_ICU_IRQ13                  (0x00E)  /* External pin interrupt 13 - Event 0x00E */
#define  RA_EL_EVENT_ICU_IRQ14                  (0x00F)  /* External pin interrupt 14 - Event 0x00F */
#define  RA_EL_EVENT_ICU_IRQ15                  (0x010)  /* External pin interrupt 15 - Event 0x010 */
#define  RA_EL_EVENT_DMAC0_INT                  (0x011)  /* DMAC0 transfer end - Event 0x011 */
#define  RA_EL_EVENT_DMAC1_INT                  (0x012)  /* DMAC1 transfer end - Event 0x012 */
#define  RA_EL_EVENT_DMAC2_INT                  (0x013)  /* DMAC2 transfer end - Event 0x013 */
#define  RA_EL_EVENT_DMAC3_INT                  (0x014)  /* DMAC3 transfer end - Event 0x014 */
#define  RA_EL_EVENT_DMAC4_INT                  (0x015)  /* DMAC4 transfer end - Event 0x015 */
#define  RA_EL_EVENT_DMAC5_INT                  (0x016)  /* DMAC5 transfer end - Event 0x016 */
#define  RA_EL_EVENT_DMAC6_INT                  (0x017)  /* DMAC6 transfer end - Event 0x017 */
#define  RA_EL_EVENT_DMAC7_INT                  (0x018)  /* DMAC7 transfer end - Event 0x018 */
#define  RA_EL_EVENT_DTC_END                    (0x022)  /* DTC transfer end - Event 0x022 */
#define  RA_EL_EVENT_DTC_COMPLETE               (0x022)  /* DTC transfer complete - Event 0x022 */
#define  RA_EL_EVENT_DMA_TRANSERR               (0x027)  /* DMA/DTC transfer error - Event 0x027 */
#define  RA_EL_EVENT_ICU_SNOOZE_CANCEL          (0x029)  /* Canceling from Snooze mode - DBG_CTIIRQ0 */
#define  RA_EL_EVENT_FCU_FRDYI                  (0x031)  /* Flash ready interrupt - Event 0x031 */
#define  RA_EL_EVENT_LVD_LVD1                   (0x038)  /* Voltage monitor 1 interrupt - Event 0x038 */
#define  RA_EL_EVENT_LVD_LVD2                   (0x039)  /* Voltage monitor 2 interrupt - Event 0x039 */
#define  RA_EL_EVENT_LVD_VBATT                  (0x03D)  /* VBATT low voltage detect - Event 0x03D */
#define  RA_EL_EVENT_CGC_MOSC_STOP              (0x03E)  /* Main Clock oscillation stop - Event 0x03E */
#define  RA_EL_EVENT_LPM_SNOOZE_REQUEST         (0x040)  /* Snooze entry - ULPT0_ULPTI */
#define  RA_EL_EVENT_ULPT0_INT                  (0x040)  /* ULPT0 Underflow - Event 0x040 */
#define  RA_EL_EVENT_ULPT0_COMPARE_A            (0x041)  /* ULPT0 Compare match A - Event 0x041 */
#define  RA_EL_EVENT_ULPT0_COMPARE_B            (0x042)  /* ULPT0 Compare match B - Event 0x042 */
#define  RA_EL_EVENT_ULPT1_INT                  (0x043)  /* ULPT1 Underflow - Event 0x043 */
#define  RA_EL_EVENT_ULPT1_COMPARE_A            (0x044)  /* ULPT1 Compare match A - Event 0x044 */
#define  RA_EL_EVENT_ULPT1_COMPARE_B            (0x045)  /* ULPT1 Compare match B - Event 0x045 */
#define  RA_EL_EVENT_AGT0_INT                   (0x046)  /* AGT interrupt - Event 0x046 */
#define  RA_EL_EVENT_AGT0_COMPARE_A             (0x047)  /* Compare match A - Event 0x047 */
#define  RA_EL_EVENT_AGT0_COMPARE_B             (0x048)  /* Compare match B - Event 0x048 */
#define  RA_EL_EVENT_AGT1_INT                   (0x049)  /* AGT interrupt - Event 0x049 */
#define  RA_EL_EVENT_AGT1_COMPARE_A             (0x04A)  /* Compare match A - Event 0x04A */
#define  RA_EL_EVENT_AGT1_COMPARE_B             (0x04B)  /* Compare match B - Event 0x04B */
#define  RA_EL_EVENT_IWDT_UNDERFLOW             (0x052)  /* IWDT underflow - Event 0x052 */
#define  RA_EL_EVENT_WDT_UNDERFLOW              (0x053)  /* WDT underflow - Event 0x053 */
#define  RA_EL_EVENT_RTC_ALARM                  (0x055)  /* Alarm interrupt - Event 0x055 */
#define  RA_EL_EVENT_RTC_PERIOD                 (0x056)  /* Periodic interrupt - Event 0x056 */
#define  RA_EL_EVENT_RTC_CARRY                  (0x057)  /* Carry interrupt - Event 0x057 */
#define  RA_EL_EVENT_ADC0_SCAN_END              (0x1AE) /* End of A/D scanning operation - Event 0x1AE */
#define  RA_EL_EVENT_ADC0_SCAN_END_B            (0x1AF) /* A/D scan end interrupt for group B - Event 0x1AF */
#define  RA_EL_EVENT_ADC0_WINDOW_A              (0x1B0) /* Window A Compare match interrupt - Event 0x1B0 */
#define  RA_EL_EVENT_ADC0_WINDOW_B              (0x1B1) /* Window B Compare match interrupt - Event 0x1B1 */
#define  RA_EL_EVENT_ADC0_COMPARE_MATCH         (0x1B2) /* Compare match - Event 0x1B2 */
#define  RA_EL_EVENT_ADC0_COMPARE_MISMATCH      (0x1B3) /* Compare mismatch - Event 0x1B3 */
#define  RA_EL_EVENT_ADC1_SCAN_END              (0x1B4) /* End of A/D scanning operation - Event 0x1B4 */
#define  RA_EL_EVENT_ADC1_SCAN_END_B            (0x1B5) /* A/D scan end interrupt for group B - Event 0x1B5 */
#define  RA_EL_EVENT_ADC1_WINDOW_A              (0x1B6) /* Window A Compare match interrupt - Event 0x1B6 */
#define  RA_EL_EVENT_ADC1_WINDOW_B              (0x1B7) /* Window B Compare match interrupt - Event 0x1B7 */
#define  RA_EL_EVENT_ADC1_COMPARE_MATCH         (0x1B8) /* Compare match - Event 0x1B8 */
#define  RA_EL_EVENT_ADC1_COMPARE_MISMATCH      (0x1B9) /* Compare mismatch - Event 0x1B9 */
#define  RA_EL_EVENT_ACMPLP0_INT                (0x07B)  /* Low Power Comparator channel 0 interrupt - ACMP_HS0 */
#define  RA_EL_EVENT_ACMPLP1_INT                (0x07C)  /* Low Power Comparator channel 1 interrupt - ACMP_HS1 */
#define  RA_EL_EVENT_ACMPHS0_INT                (0x07B)  /* High Speed Comparator channel 0 interrupt - Event 0x07B */
#define  RA_EL_EVENT_ACMPHS1_INT                (0x07C)  /* High Speed Comparator channel 1 interrupt - Event 0x07C */
#define  RA_EL_EVENT_USBFS_FIFO_0               (0x058)  /* DMA transfer request 0 - Event 0x058 */
#define  RA_EL_EVENT_USBFS_FIFO_1               (0x059)  /* DMA transfer request 1 - Event 0x059 */
#define  RA_EL_EVENT_USBFS_INT                  (0x05A)  /* USBFS interrupt - Event 0x05A */
#define  RA_EL_EVENT_USBFS_RESUME               (0x05B)  /* USBFS resume interrupt - Event 0x05B */
#define  RA_EL_EVENT_IIC0_RXI                   (0x05C)  /* Receive data full - Event 0x05C */
#define  RA_EL_EVENT_IIC0_TXI                   (0x05D)  /* Transmit data empty - Event 0x05D */
#define  RA_EL_EVENT_IIC0_TEI                   (0x05E)  /* Transmit end - Event 0x05E */
#define  RA_EL_EVENT_IIC0_ERI                   (0x05F)  /* Transfer error - Event 0x05F */
#define  RA_EL_EVENT_IIC0_WUI                   (0x060)  /* Wakeup interrupt - Event 0x060 */
#define  RA_EL_EVENT_IIC1_RXI                   (0x061)  /* Receive data full - Event 0x061 */
#define  RA_EL_EVENT_IIC1_TXI                   (0x062)  /* Transmit data empty - Event 0x062 */
#define  RA_EL_EVENT_IIC1_TEI                   (0x063)  /* Transmit end - Event 0x063 */
#define  RA_EL_EVENT_IIC1_ERI                   (0x064)  /* Transfer error - Event 0x064 */
#define  RA_EL_EVENT_SSI0_TXI                   (0x073)  /* Transmit data empty - Event 0x073 */
#define  RA_EL_EVENT_SSI0_RXI                   (0x074)  /* Receive data full - Event 0x074 */
#define  RA_EL_EVENT_SSI0_INT                   (0x076)  /* Error interrupt - Event 0x076 */
#define  RA_EL_EVENT_SSI1_SSIRT                 (0x079)  /* Transmit/Receive - Event 0x079 */
#define  RA_EL_EVENT_SSI1_INT                   (0x07A)  /* Error interrupt - Event 0x07A */
#define  RA_EL_EVENT_CEU_CEUI                   (0x1DA)  /* Write request interrupt - CEU_CEUI */
#define  RA_EL_EVENT_KEY_INT                    (0x1BC)  /* Key interrupt - RSIP_TADI */
#define  RA_EL_EVENT_DOC_INT                    (0x1BA) /* Data operation circuit interrupt - Event 0x1BA */
#define  RA_EL_EVENT_CAC_FREQUENCY_ERROR        (0x08C)  /* Frequency error interrupt - Event 0x08C */
#define  RA_EL_EVENT_CAC_MEASUREMENT_END        (0x08D)  /* Measurement end interrupt - Event 0x08D */
#define  RA_EL_EVENT_CAC_OVERFLOW               (0x08E)  /* Overflow interrupt - Event 0x08E */
#define  RA_EL_EVENT_CAN0_ERROR                 (0x186) /* Global error - Event 0x186 */
#define  RA_EL_EVENT_CAN0_FIFO_RX               (0x185) /* Global receive FIFO interrupt - Event 0x185 */
#define  RA_EL_EVENT_CAN0_FIFO_TX               (0x18F) /* Transmit interrupt - Event 0x18F */
#define  RA_EL_EVENT_CAN0_MAILBOX_RX            (0x193) /* Receive message buffer interrupt - Event 0x193 */
#define  RA_EL_EVENT_CAN0_MAILBOX_TX            (0x18F) /* Transmit interrupt - CAN0_TX */
#define  RA_EL_EVENT_CAN1_ERROR                 (0x195) /* Channel error - Event 0x195 */
#define  RA_EL_EVENT_CAN1_FIFO_RX               (0x196) /* Common FIFO receive interrupt - Event 0x196 */
#define  RA_EL_EVENT_CAN1_FIFO_TX               (0x194) /* Transmit interrupt - Event 0x194 */
#define  RA_EL_EVENT_CAN1_MAILBOX_RX            (0x198) /* Receive message buffer interrupt - Event 0x198 */
#define  RA_EL_EVENT_CAN1_MAILBOX_TX            (0x194) /* Channel DMA request - CAN1_TX */
#define  RA_EL_EVENT_IOPORT_EVENT_1             (0x088)  /* Port 1 event - Event 0x088 */
#define  RA_EL_EVENT_IOPORT_EVENT_2             (0x089)  /* Port 2 event - Event 0x089 */
#define  RA_EL_EVENT_IOPORT_EVENT_3             (0x08A)  /* Port 3 event - Event 0x08A */
#define  RA_EL_EVENT_IOPORT_EVENT_4             (0x08B)  /* Port 4 event - Event 0x08B */
#define  RA_EL_EVENT_SOFTWARE_EVENT_0           (0x083)  /* Software event 0 - Event 0x083 */
#define  RA_EL_EVENT_SOFTWARE_EVENT_1           (0x084)  /* Software event 1 - Event 0x084 */
#define  RA_EL_EVENT_POEG0_EVENT                (0x08F)  /* Port Output disable 0 interrupt - Event 0x08F */
#define  RA_EL_EVENT_POEG1_EVENT                (0x090)  /* Port Output disable 1 interrupt - Event 0x090 */
#define  RA_EL_EVENT_POEG2_EVENT                (0x091)  /* Port Output disable 2 interrupt - Event 0x091 */
#define  RA_EL_EVENT_POEG3_EVENT                (0x092)  /* Port Output disable 3 interrupt - Event 0x092 */
#define  RA_EL_EVENT_GPT0_CAPTURE_COMPARE_A     (0x0A1)  /* Capture/Compare match A - Event 0x0A1 */
#define  RA_EL_EVENT_GPT0_CAPTURE_COMPARE_B     (0x0A2)  /* Capture/Compare match B - Event 0x0A2 */
#define  RA_EL_EVENT_GPT0_COMPARE_C             (0x0A3)  /* Compare match C - Event 0x0A3 */
#define  RA_EL_EVENT_GPT0_COMPARE_D             (0x0A4)  /* Compare match D - Event 0x0A4 */
#define  RA_EL_EVENT_GPT0_COMPARE_E             (0x0A5)  /* Compare match E - Event 0x0A5 */
#define  RA_EL_EVENT_GPT0_COMPARE_F             (0x0A6)  /* Compare match F - Event 0x0A6 */
#define  RA_EL_EVENT_GPT0_COUNTER_OVERFLOW      (0x0A7)  /* Overflow - Event 0x0A7 */
#define  RA_EL_EVENT_GPT0_COUNTER_UNDERFLOW     (0x0A8)  /* Underflow - Event 0x0A8 */
#define  RA_EL_EVENT_GPT0_PC                    (0x0A9)  /* Period count function finish - Event 0x0A9 */
#define  RA_EL_EVENT_GPT1_CAPTURE_COMPARE_A     (0x0AA)  /* Capture/Compare match A - Event 0x0AA */
#define  RA_EL_EVENT_GPT1_CAPTURE_COMPARE_B     (0x0AB)  /* Capture/Compare match B - Event 0x0AB */
#define  RA_EL_EVENT_GPT1_COMPARE_C             (0x0AC)  /* Compare match C - Event 0x0AC */
#define  RA_EL_EVENT_GPT1_COMPARE_D             (0x0AD)  /* Compare match D - Event 0x0AD */
#define  RA_EL_EVENT_GPT1_COMPARE_E             (0x0AE)  /* Compare match E - Event 0x0AE */
#define  RA_EL_EVENT_GPT1_COMPARE_F             (0x0AF)  /* Compare match F - Event 0x0AF */
#define  RA_EL_EVENT_GPT1_COUNTER_OVERFLOW      (0x0B0)  /* Overflow - Event 0x0B0 */
#define  RA_EL_EVENT_GPT1_COUNTER_UNDERFLOW     (0x0B1)  /* Underflow - Event 0x0B1 */
#define  RA_EL_EVENT_GPT1_PC                    (0x0B2)  /* Period count function finish - Event 0x0B2 */
#define  RA_EL_EVENT_GPT2_CAPTURE_COMPARE_A     (0x0B3)  /* Capture/Compare match A - Event 0x0B3 */
#define  RA_EL_EVENT_GPT2_CAPTURE_COMPARE_B     (0x0B4)  /* Capture/Compare match B - Event 0x0B4 */
#define  RA_EL_EVENT_GPT2_COMPARE_C             (0x0B5)  /* Compare match C - Event 0x0B5 */
#define  RA_EL_EVENT_GPT2_COMPARE_D             (0x0B6)  /* Compare match D - Event 0x0B6 */
#define  RA_EL_EVENT_GPT2_COMPARE_E             (0x0B7)  /* Compare match E - Event 0x0B7 */
#define  RA_EL_EVENT_GPT2_COMPARE_F             (0x0B8)  /* Compare match F - Event 0x0B8 */
#define  RA_EL_EVENT_GPT2_COUNTER_OVERFLOW      (0x0B9)  /* Overflow - Event 0x0B9 */
#define  RA_EL_EVENT_GPT2_COUNTER_UNDERFLOW     (0x0BA)  /* Underflow - Event 0x0BA */
#define  RA_EL_EVENT_GPT2_PC                    (0x0BB)  /* Period count function finish - Event 0x0BB */
#define  RA_EL_EVENT_GPT3_CAPTURE_COMPARE_A     (0x0BC)  /* Capture/Compare match A - Event 0x0BC */
#define  RA_EL_EVENT_GPT3_CAPTURE_COMPARE_B     (0x0BD)  /* Capture/Compare match B - Event 0x0BD */
#define  RA_EL_EVENT_GPT3_COMPARE_C             (0x0BE)  /* Compare match C - Event 0x0BE */
#define  RA_EL_EVENT_GPT3_COMPARE_D             (0x0BF)  /* Compare match D - Event 0x0BF */
#define  RA_EL_EVENT_GPT3_COMPARE_E             (0x0C0)  /* Compare match E - Event 0x0C0 */
#define  RA_EL_EVENT_GPT3_COMPARE_F             (0x0C1)  /* Compare match F - Event 0x0C1 */
#define  RA_EL_EVENT_GPT3_COUNTER_OVERFLOW      (0x0C2)  /* Overflow - Event 0x0C2 */
#define  RA_EL_EVENT_GPT3_COUNTER_UNDERFLOW     (0x0C3)  /* Underflow - Event 0x0C3 */
#define  RA_EL_EVENT_GPT3_PC                    (0x0C4)  /* Period count function finish - Event 0x0C4 */
#define  RA_EL_EVENT_GPT4_CAPTURE_COMPARE_A     (0x0C5)  /* Capture/Compare match A - Event 0x0C5 */
#define  RA_EL_EVENT_GPT4_CAPTURE_COMPARE_B     (0x0C6)  /* Capture/Compare match B - Event 0x0C6 */
#define  RA_EL_EVENT_GPT4_COMPARE_C             (0x0C7)  /* Compare match C - Event 0x0C7 */
#define  RA_EL_EVENT_GPT4_COMPARE_D             (0x0C8)  /* Compare match D - Event 0x0C8 */
#define  RA_EL_EVENT_GPT4_COMPARE_E             (0x0C9)  /* Compare match E - Event 0x0C9 */
#define  RA_EL_EVENT_GPT4_COMPARE_F             (0x0CA)  /* Compare match F - Event 0x0CA */
#define  RA_EL_EVENT_GPT4_COUNTER_OVERFLOW      (0x0CB)  /* Overflow - Event 0x0CB */
#define  RA_EL_EVENT_GPT4_COUNTER_UNDERFLOW     (0x0CC)  /* Underflow - Event 0x0CC */
#define  RA_EL_EVENT_GPT5_CAPTURE_COMPARE_A     (0x0CE)  /* Capture/Compare match A - Event 0x0CE */
#define  RA_EL_EVENT_GPT5_CAPTURE_COMPARE_B     (0x0CF)  /* Capture/Compare match B - Event 0x0CF */
#define  RA_EL_EVENT_GPT5_COMPARE_C             (0x0D0)  /* Compare match C - Event 0x0D0 */
#define  RA_EL_EVENT_GPT5_COMPARE_D             (0x0D1)  /* Compare match D - Event 0x0D1 */
#define  RA_EL_EVENT_GPT5_COMPARE_E             (0x0D2)  /* Compare match E - Event 0x0D2 */
#define  RA_EL_EVENT_GPT5_COMPARE_F             (0x0D3)  /* Compare match F - Event 0x0D3 */
#define  RA_EL_EVENT_GPT5_COUNTER_OVERFLOW      (0x0D4)  /* Overflow - Event 0x0D4 */
#define  RA_EL_EVENT_GPT5_COUNTER_UNDERFLOW     (0x0D5)  /* Underflow - Event 0x0D5 */
#define  RA_EL_EVENT_GPT6_CAPTURE_COMPARE_A     (0x0D6)  /* Capture/Compare match A - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_CAPTURE_COMPARE_B     (0x0D7)  /* Capture/Compare match B - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_COMPARE_C             (0x0D8)  /* Compare match C - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_COMPARE_D             (0x0D9)  /* Compare match D - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_COMPARE_E             (0x0DA)  /* Compare match E - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_COMPARE_F             (0x0DB)  /* Compare match F - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_COUNTER_OVERFLOW      (0x0DC)  /* Overflow - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT6_COUNTER_UNDERFLOW     (0x0DD)  /* Underflow - GPT6 (estimated) */
#define  RA_EL_EVENT_GPT7_CAPTURE_COMPARE_A     (0x0DE)  /* Capture/Compare match A - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_CAPTURE_COMPARE_B     (0x0DF)  /* Capture/Compare match B - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_COMPARE_C             (0x0E0)  /* Compare match C - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_COMPARE_D             (0x0E1)  /* Compare match D - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_COMPARE_E             (0x0E2)  /* Compare match E - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_COMPARE_F             (0x0E3)  /* Compare match F - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_COUNTER_OVERFLOW      (0x0E4)  /* Overflow - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT7_COUNTER_UNDERFLOW     (0x0E5)  /* Underflow - GPT7 (estimated) */
#define  RA_EL_EVENT_GPT10_CAPTURE_COMPARE_A    (0x0FB)  /* Capture/Compare match A - Event 0x0FB */
#define  RA_EL_EVENT_GPT10_CAPTURE_COMPARE_B    (0x0FC)  /* Capture/Compare match B - Event 0x0FC */
#define  RA_EL_EVENT_GPT10_COMPARE_C            (0x0FD)  /* Compare match C - Event 0x0FD */
#define  RA_EL_EVENT_GPT10_COMPARE_D            (0x0FE)  /* Compare match D - Event 0x0FE */
#define  RA_EL_EVENT_GPT10_COMPARE_E            (0x0FF) /* Compare match E - Event 0x0FF */
#define  RA_EL_EVENT_GPT10_COMPARE_F            (0x100) /* Compare match F - Event 0x100 */
#define  RA_EL_EVENT_GPT10_COUNTER_OVERFLOW     (0x101) /* Overflow - Event 0x101 */
#define  RA_EL_EVENT_GPT10_COUNTER_UNDERFLOW    (0x102) /* Underflow - Event 0x102 */
#define  RA_EL_EVENT_GPT10_PC                   (0x103) /* Period count function finish - Event 0x103 */
#define  RA_EL_EVENT_GPT11_CAPTURE_COMPARE_A    (0x104) /* Capture/Compare match A - Event 0x104 */
#define  RA_EL_EVENT_GPT11_CAPTURE_COMPARE_B    (0x105) /* Capture/Compare match B - Event 0x105 */
#define  RA_EL_EVENT_GPT11_COMPARE_C            (0x106) /* Compare match C - Event 0x106 */
#define  RA_EL_EVENT_GPT11_COMPARE_D            (0x107) /* Compare match D - Event 0x107 */
#define  RA_EL_EVENT_GPT11_COMPARE_E            (0x108) /* Compare match E - Event 0x108 */
#define  RA_EL_EVENT_GPT11_COMPARE_F            (0x109) /* Compare match F - Event 0x109 */
#define  RA_EL_EVENT_GPT11_COUNTER_OVERFLOW     (0x10A) /* Overflow - Event 0x10A */
#define  RA_EL_EVENT_GPT11_COUNTER_UNDERFLOW    (0x10B) /* Underflow - Event 0x10B */
#define  RA_EL_EVENT_GPT12_CAPTURE_COMPARE_A    (0x10D) /* Capture/Compare match A - Event 0x10D */
#define  RA_EL_EVENT_GPT12_CAPTURE_COMPARE_B    (0x10E) /* Capture/Compare match B - Event 0x10E */
#define  RA_EL_EVENT_GPT12_COMPARE_C            (0x10F) /* Compare match C - Event 0x10F */
#define  RA_EL_EVENT_GPT12_COMPARE_D            (0x110) /* Compare match D - Event 0x110 */
#define  RA_EL_EVENT_GPT12_COMPARE_E            (0x111) /* Compare match E - Event 0x111 */
#define  RA_EL_EVENT_GPT12_COMPARE_F            (0x112) /* Compare match F - Event 0x112 */
#define  RA_EL_EVENT_GPT12_COUNTER_OVERFLOW     (0x113) /* Overflow - Event 0x113 */
#define  RA_EL_EVENT_GPT12_COUNTER_UNDERFLOW    (0x114) /* Underflow - Event 0x114 */
#define  RA_EL_EVENT_GPT13_CAPTURE_COMPARE_A    (0x116) /* Capture/Compare match A - Event 0x116 */
#define  RA_EL_EVENT_GPT13_CAPTURE_COMPARE_B    (0x117) /* Capture/Compare match B - Event 0x117 */
#define  RA_EL_EVENT_GPT13_COMPARE_C            (0x118) /* Compare match C - Event 0x118 */
#define  RA_EL_EVENT_GPT13_COMPARE_D            (0x119) /* Compare match D - Event 0x119 */
#define  RA_EL_EVENT_GPT13_COMPARE_E            (0x11A) /* Compare match E - Event 0x11A */
#define  RA_EL_EVENT_GPT13_COMPARE_F            (0x11B) /* Compare match F - Event 0x11B */
#define  RA_EL_EVENT_GPT13_COUNTER_OVERFLOW     (0x11C) /* Overflow - Event 0x11C */
#define  RA_EL_EVENT_GPT13_COUNTER_UNDERFLOW    (0x11D) /* Underflow - Event 0x11D */
#define  RA_EL_EVENT_OPS_UVW_EDGE               (0x120)  /* UVW edge event - Event 0x120 */
#define  RA_EL_EVENT_SCI0_RXI                   (0x124) /* Receive data full - Event 0x124 */
#define  RA_EL_EVENT_SCI0_TXI                   (0x125) /* Transmit data empty - Event 0x125 */
#define  RA_EL_EVENT_SCI0_TEI                   (0x126) /* Transmit end - Event 0x126 */
#define  RA_EL_EVENT_SCI0_ERI                   (0x127) /* Receive error - Event 0x127 */
#define  RA_EL_EVENT_SCI0_AM                    (0x12A) /* Address match event - Event 0x12A */
#define  RA_EL_EVENT_SCI0_RXI_OR_ERI            (0x124)  /* Receive data full/Receive error - Same as RXI */
#define  RA_EL_EVENT_SCI1_RXI                   (0x12B) /* Receive data full - Event 0x12B */
#define  RA_EL_EVENT_SCI1_TXI                   (0x12C) /* Transmit data empty - Event 0x12C */
#define  RA_EL_EVENT_SCI1_TEI                   (0x12D) /* Transmit end - Event 0x12D */
#define  RA_EL_EVENT_SCI1_ERI                   (0x12E) /* Receive error - Event 0x12E */
#define  RA_EL_EVENT_SCI1_AM                    (0x131) /* Address match event - Event 0x131 */
#define  RA_EL_EVENT_SCI2_RXI                   (0x132) /* Receive data full - Event 0x132 */
#define  RA_EL_EVENT_SCI2_TXI                   (0x133) /* Transmit data empty - Event 0x133 */
#define  RA_EL_EVENT_SCI2_TEI                   (0x134) /* Transmit end - Event 0x134 */
#define  RA_EL_EVENT_SCI2_ERI                   (0x135) /* Receive error - Event 0x135 */
#define  RA_EL_EVENT_SCI2_AM                    (0x138) /* Address match event - Event 0x138 */
#define  RA_EL_EVENT_SCI3_RXI                   (0x139) /* Receive data full - Event 0x139 */
#define  RA_EL_EVENT_SCI3_TXI                   (0x13A) /* Transmit data empty - Event 0x13A */
#define  RA_EL_EVENT_SCI3_TEI                   (0x13B) /* Transmit end - Event 0x13B */
#define  RA_EL_EVENT_SCI3_ERI                   (0x13C) /* Receive error - Event 0x13C */
#define  RA_EL_EVENT_SCI3_AM                    (0x13F) /* Address match event - Event 0x13F */
#define  RA_EL_EVENT_SCI4_RXI                   (0x140) /* Receive data full - Event 0x140 */
#define  RA_EL_EVENT_SCI4_TXI                   (0x141) /* Transmit data empty - Event 0x141 */
#define  RA_EL_EVENT_SCI4_TEI                   (0x142) /* Transmit end - Event 0x142 */
#define  RA_EL_EVENT_SCI4_ERI                   (0x143) /* Receive error - Event 0x143 */
#define  RA_EL_EVENT_SCI4_AM                    (0x146) /* Address match event - Event 0x146 */
#define  RA_EL_EVENT_SCI9_RXI                   (0x163) /* Receive data full - Event 0x163 */
#define  RA_EL_EVENT_SCI9_TXI                   (0x164) /* Transmit data empty - Event 0x164 */
#define  RA_EL_EVENT_SCI9_TEI                   (0x165) /* Transmit end - Event 0x165 */
#define  RA_EL_EVENT_SCI9_ERI                   (0x166) /* Receive error - Event 0x166 */
#define  RA_EL_EVENT_SCI9_AM                    (0x169) /* Address match event - Event 0x169 */
#define  RA_EL_EVENT_SPI0_RXI                   (0x178) /* Receive buffer full - Event 0x178 */
#define  RA_EL_EVENT_SPI0_TXI                   (0x179) /* Transmit buffer empty - Event 0x179 */
#define  RA_EL_EVENT_SPI0_IDLE                  (0x17A) /* Idle - Event 0x17A */
#define  RA_EL_EVENT_SPI0_ERI                   (0x17B) /* Error - Event 0x17B */
#define  RA_EL_EVENT_SPI0_TEI                   (0x17C) /* Transmission complete event - Event 0x17C */
#define  RA_EL_EVENT_SPI1_RXI                   (0x17D) /* Receive buffer full - Event 0x17D */
#define  RA_EL_EVENT_SPI1_TXI                   (0x17E) /* Transmit buffer empty - Event 0x17E */
#define  RA_EL_EVENT_SPI1_IDLE                  (0x17F) /* Idle - Event 0x17F */
#define  RA_EL_EVENT_SPI1_ERI                   (0x180) /* Error - Event 0x180 */
#define  RA_EL_EVENT_SPI1_TEI                   (0x181) /* Transmission complete event - Event 0x181 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
#endif

#endif /* __ARCH_ARM_INCLUDE_RA_RA_IRQ_H */
