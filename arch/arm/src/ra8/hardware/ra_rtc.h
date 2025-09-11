/****************************************************************************
 * arch/arm/src/ra8/hardware/ra8e1/ra8e1_rtc.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA8E1_RTC_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA8E1_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define RA_RTC_R64CNT_OFFSET     0x0000  /* 64-Hz Counter */
#define RA_RTC_RSECCNT_OFFSET    0x0002  /* Second Counter */
#define RA_RTC_RMINCNT_OFFSET    0x0004  /* Minute Counter */
#define RA_RTC_RHRCNT_OFFSET     0x0006  /* Hour Counter */
#define RA_RTC_RWKCNT_OFFSET     0x0008  /* Day-of-Week Counter */
#define RA_RTC_RDAYCNT_OFFSET    0x000A  /* Day Counter */
#define RA_RTC_RMONCNT_OFFSET    0x000C  /* Month Counter */
#define RA_RTC_RYRCNT_OFFSET     0x000E  /* Year Counter */
#define RA_RTC_RSECAR_OFFSET     0x0010  /* Second Alarm Register */
#define RA_RTC_RMINAR_OFFSET     0x0012  /* Minute Alarm Register */
#define RA_RTC_RHRAR_OFFSET      0x0014  /* Hour Alarm Register */
#define RA_RTC_RWKAR_OFFSET      0x0016  /* Day-of-Week Alarm Register */
#define RA_RTC_RDAYAR_OFFSET     0x0018  /* Date Alarm Register */
#define RA_RTC_RMONAR_OFFSET     0x001A  /* Month Alarm Register */
#define RA_RTC_RYRAR_OFFSET      0x001C  /* Year Alarm Register */
#define RA_RTC_RCR1_OFFSET       0x0022  /* RTC Control Register 1 */
#define RA_RTC_RCR2_OFFSET       0x0024  /* RTC Control Register 2 */
#define RA_RTC_RCR3_OFFSET       0x0026  /* RTC Control Register 3 */
#define RA_RTC_RCR4_OFFSET       0x0028  /* RTC Control Register 4 */
#define RA_RTC_RFRH_OFFSET       0x002A  /* Frequency Register H */
#define RA_RTC_RFRL_OFFSET       0x002C  /* Frequency Register L */
#define RA_RTC_RADJ_OFFSET       0x002E  /* Time Error Adjustment Register */

/* Register Addresses *******************************************************/

#define RA_RTC_R64CNT            (R_RTC_BASE + RA_RTC_R64CNT_OFFSET)
#define RA_RTC_RSECCNT           (R_RTC_BASE + RA_RTC_RSECCNT_OFFSET)
#define RA_RTC_RMINCNT           (R_RTC_BASE + RA_RTC_RMINCNT_OFFSET)
#define RA_RTC_RHRCNT            (R_RTC_BASE + RA_RTC_RHRCNT_OFFSET)
#define RA_RTC_RWKCNT            (R_RTC_BASE + RA_RTC_RWKCNT_OFFSET)
#define RA_RTC_RDAYCNT           (R_RTC_BASE + RA_RTC_RDAYCNT_OFFSET)
#define RA_RTC_RMONCNT           (R_RTC_BASE + RA_RTC_RMONCNT_OFFSET)
#define RA_RTC_RYRCNT            (R_RTC_BASE + RA_RTC_RYRCNT_OFFSET)
#define RA_RTC_RSECAR            (R_RTC_BASE + RA_RTC_RSECAR_OFFSET)
#define RA_RTC_RMINAR            (R_RTC_BASE + RA_RTC_RMINAR_OFFSET)
#define RA_RTC_RHRAR             (R_RTC_BASE + RA_RTC_RHRAR_OFFSET)
#define RA_RTC_RWKAR             (R_RTC_BASE + RA_RTC_RWKAR_OFFSET)
#define RA_RTC_RDAYAR            (R_RTC_BASE + RA_RTC_RDAYAR_OFFSET)
#define RA_RTC_RMONAR            (R_RTC_BASE + RA_RTC_RMONAR_OFFSET)
#define RA_RTC_RYRAR             (R_RTC_BASE + RA_RTC_RYRAR_OFFSET)
#define RA_RTC_RCR1              (R_RTC_BASE + RA_RTC_RCR1_OFFSET)
#define RA_RTC_RCR2              (R_RTC_BASE + RA_RTC_RCR2_OFFSET)
#define RA_RTC_RCR3              (R_RTC_BASE + RA_RTC_RCR3_OFFSET)
#define RA_RTC_RCR4              (R_RTC_BASE + RA_RTC_RCR4_OFFSET)
#define RA_RTC_RFRH              (R_RTC_BASE + RA_RTC_RFRH_OFFSET)
#define RA_RTC_RFRL              (R_RTC_BASE + RA_RTC_RFRL_OFFSET)
#define RA_RTC_RADJ              (R_RTC_BASE + RA_RTC_RADJ_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* RCR1 - RTC Control Register 1 */
#define RA_RTC_RCR1_PES_SHIFT    (4)        /* Periodic Interrupt Select */
#define RA_RTC_RCR1_PES_MASK     (0x0F << RA_RTC_RCR1_PES_SHIFT)
#define RA_RTC_RCR1_RTCOS        (1 << 3)   /* RTCOUT Output Select */
#define RA_RTC_RCR1_PIE          (1 << 2)   /* Periodic Interrupt Enable */
#define RA_RTC_RCR1_CIE          (1 << 1)   /* Carry Interrupt Enable */
#define RA_RTC_RCR1_AIE          (1 << 0)   /* Alarm Interrupt Enable */

/* RCR2 - RTC Control Register 2 */
#define RA_RTC_RCR2_CNTMD        (1 << 7)   /* Count Mode Select */
#define RA_RTC_RCR2_HR24         (1 << 6)   /* Hours Mode (0=12hr, 1=24hr) */
#define RA_RTC_RCR2_AADJP        (1 << 5)   /* Automatic Adjustment Period Select */
#define RA_RTC_RCR2_AADJE        (1 << 4)   /* Automatic Adjustment Enable */
#define RA_RTC_RCR2_RTCOE        (1 << 3)   /* RTCOUT Output Enable */
#define RA_RTC_RCR2_ADJ30        (1 << 2)   /* 30-Second Adjustment */
#define RA_RTC_RCR2_RESET        (1 << 1)   /* RTC Software Reset */
#define RA_RTC_RCR2_START        (1 << 0)   /* Start */

/* RCR3 - RTC Control Register 3 */
#define RA_RTC_RCR3_RTCEN        (1 << 0)   /* Sub-clock oscillator enable */

/* RCR4 - RTC Control Register 4 */
#define RA_RTC_RCR4_RCKSEL       (1 << 0)   /* Count Source Select */

/* Time counter masks */
#define RA_RTC_SEC_MASK          0x7F       /* Second (0-59) */
#define RA_RTC_MIN_MASK          0x7F       /* Minute (0-59) */
#define RA_RTC_HOUR_MASK         0x3F       /* Hour (0-23) */
#define RA_RTC_WDAY_MASK         0x07       /* Day of week (0-6) */
#define RA_RTC_MDAY_MASK         0x3F       /* Day of month (1-31) */
#define RA_RTC_MON_MASK          0x1F       /* Month (1-12) */
#define RA_RTC_YEAR_MASK         0xFF       /* Year (0-99) */

/* BCD conversion macros */
#define RA_RTC_BIN2BCD(bin)      (((bin) / 10) << 4 | ((bin) % 10))
#define RA_RTC_BCD2BIN(bcd)      (((bcd) >> 4) * 10 + ((bcd) & 0x0F))

/* RTC Feature Definitions */
#define RA_RTC_RESET_DELAY_US    200        /* Reset delay in microseconds */

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA8E1_RTC_H */
