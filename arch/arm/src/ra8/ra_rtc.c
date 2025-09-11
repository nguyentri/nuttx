/****************************************************************************
 * arch/arm/src/ra8/ra_rtc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

#include "arm_internal.h"

#include "ra_mstp.h"
#include "hardware/ra_rtc.h"

#ifdef CONFIG_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* External reference to the RTC enabled state variable 
 * (defined in arch_rtc.c when CONFIG_RTC_ARCH is enabled)
 */

extern volatile bool g_rtc_enabled;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_rtc_wait_register
 *
 * Description:
 *   Wait for RTC register operation to complete
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_rtc_wait_register(void)
{
  /* Wait for register update to complete */
  volatile int timeout = 10000;
  while (timeout-- > 0)
    {
      /* Small delay to allow register update */
      up_udelay(10);
    }
}

/****************************************************************************
 * Name: ra_rtc_reset
 *
 * Description:
 *   Reset the RTC hardware
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void ra_rtc_reset(void)
{
  /* Reset RTC */
  putreg8(0, RA_RTC_RCR2);
  ra_rtc_wait_register();

  putreg8(RA_RTC_RCR2_RESET, RA_RTC_RCR2);
  ra_rtc_wait_register();

  /* Wait for reset to complete */
  while (getreg8(RA_RTC_RCR2) & RA_RTC_RCR2_RESET)
    {
      ra_rtc_wait_register();
    }

  /* Disable RTC interrupts */
  putreg8(0, RA_RTC_RCR1);
  ra_rtc_wait_register();
}

/****************************************************************************
 * Name: ra_rtc_bcd_to_bin
 *
 * Description:
 *   Convert BCD to binary
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
static uint8_t ra_rtc_bcd_to_bin(uint8_t bcd)
{
  return ((bcd >> 4) * 10) + (bcd & 0x0f);
}
#endif

/****************************************************************************
 * Name: ra_rtc_bin_to_bcd
 *
 * Description:
 *   Convert binary to BCD
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
static uint8_t ra_rtc_bin_to_bcd(uint8_t bin)
{
  return ((bin / 10) << 4) | (bin % 10);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This
 *   function is called once during the OS initialization sequence.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  rtcinfo("up_rtc_initialize: Initializing RA8 RTC\n");

  /* Enable RTC module clock via MSTP */
  ra_mstp_start(RA_MSTP_RTC);

  /* Configure RTC clock source - use SOSC (sub-clock) */
  putreg8(0, RA_RTC_RCR4);  /* RCKSEL = 0: SOSC */
  ra_rtc_wait_register();

  /* Reset RTC */
  ra_rtc_reset();

  /* Configure RTC for 24-hour mode */
  putreg8(RA_RTC_RCR2_HR24, RA_RTC_RCR2);
  ra_rtc_wait_register();

  /* Start RTC */
  uint8_t regval = getreg8(RA_RTC_RCR2);
  regval |= RA_RTC_RCR2_START;
  putreg8(regval, RA_RTC_RCR2);
  ra_rtc_wait_register();

  g_rtc_enabled = true;

  rtcinfo("up_rtc_initialize: RTC initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS
 *   during initialization to set up the system time when CONFIG_RTC and
 *   CONFIG_RTC_DATETIME are selected (and CONFIG_RTC_HIRES is not).
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint8_t sec, min, hour, mday, mon, wday;
  uint16_t year;

  if (!g_rtc_enabled)
    {
      rtcerr("RTC not initialized\n");
      return -EAGAIN;
    }

  if (!tp)
    {
      return -EINVAL;
    }

  /* Read current time from RTC registers */
  sec  = getreg8(RA_RTC_RSECCNT);
  min  = getreg8(RA_RTC_RMINCNT);
  hour = getreg8(RA_RTC_RHRCNT);
  wday = getreg8(RA_RTC_RWKCNT);
  mday = getreg8(RA_RTC_RDAYCNT);
  mon  = getreg8(RA_RTC_RMONCNT);
  year = getreg16(RA_RTC_RYRCNT);

  /* Convert BCD to binary */
  tp->tm_sec  = ra_rtc_bcd_to_bin(sec & RA_RTC_SEC_MASK);
  tp->tm_min  = ra_rtc_bcd_to_bin(min & RA_RTC_MIN_MASK);
  tp->tm_hour = ra_rtc_bcd_to_bin(hour & RA_RTC_HOUR_MASK);
  tp->tm_mday = ra_rtc_bcd_to_bin(mday & RA_RTC_MDAY_MASK);
  tp->tm_mon  = ra_rtc_bcd_to_bin(mon & RA_RTC_MON_MASK) - 1; /* 0-11 */
  tp->tm_year = ra_rtc_bcd_to_bin(year & RA_RTC_YEAR_MASK) + 100; /* Years since 1900 */
  tp->tm_wday = wday & RA_RTC_WDAY_MASK;
  tp->tm_yday = 0; /* Not used */
  tp->tm_isdst = 0; /* Not used */

  rtcinfo("RTC read: %04d-%02d-%02d %02d:%02d:%02d\n",
          tp->tm_year + 1900, tp->tm_mon + 1, tp->tm_mday,
          tp->tm_hour, tp->tm_min, tp->tm_sec);

  return OK;
}
#endif

/****************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  RTC implementations which provide
 *   up_rtc_getdatetime() (CONFIG_RTC_DATETIME is selected) should provide
 *   this function.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DATETIME
int up_rtc_settime(FAR const struct timespec *tp)
{
  struct tm newtime;
  uint8_t regval;
  time_t timestamp;

  if (!g_rtc_enabled)
    {
      rtcerr("RTC not initialized\n");
      return -EAGAIN;
    }

  if (!tp)
    {
      return -EINVAL;
    }

  /* Convert timestamp to broken-down time */
  timestamp = tp->tv_sec;
  if (gmtime_r(&timestamp, &newtime) == NULL)
    {
      rtcerr("gmtime_r failed\n");
      return -EINVAL;
    }

  rtcinfo("RTC set: %04d-%02d-%02d %02d:%02d:%02d\n",
          newtime.tm_year + 1900, newtime.tm_mon + 1, newtime.tm_mday,
          newtime.tm_hour, newtime.tm_min, newtime.tm_sec);

  /* Stop RTC before setting time */
  regval = getreg8(RA_RTC_RCR2);
  regval &= ~RA_RTC_RCR2_START;
  putreg8(regval, RA_RTC_RCR2);
  ra_rtc_wait_register();

  /* Set time registers in BCD format */
  putreg8(ra_rtc_bin_to_bcd(newtime.tm_sec), RA_RTC_RSECCNT);
  putreg8(ra_rtc_bin_to_bcd(newtime.tm_min), RA_RTC_RMINCNT);
  putreg8(ra_rtc_bin_to_bcd(newtime.tm_hour), RA_RTC_RHRCNT);
  putreg8(newtime.tm_wday, RA_RTC_RWKCNT);
  putreg8(ra_rtc_bin_to_bcd(newtime.tm_mday), RA_RTC_RDAYCNT);
  putreg8(ra_rtc_bin_to_bcd(newtime.tm_mon + 1), RA_RTC_RMONCNT); /* 1-12 */
  putreg16(ra_rtc_bin_to_bcd(newtime.tm_year - 100), RA_RTC_RYRCNT); /* Years since 2000 */

  /* Restart RTC */
  regval = getreg8(RA_RTC_RCR2);
  regval |= RA_RTC_RCR2_START;
  putreg8(regval, RA_RTC_RCR2);
  ra_rtc_wait_register();

  return OK;
}
#endif

#endif /* CONFIG_RTC */
