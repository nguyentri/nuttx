/****************************************************************************
 * arch/arm/src/ra8/ra_rtc_lowerhalf.c
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

#include <sys/types.h>
#include <debug.h>
#include <errno.h>
#include <time.h>

#include <nuttx/timers/rtc.h>

#include "arm_internal.h"

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ra_rtc_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ra_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct rtc_time *rtctime);
static int ra_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct rtc_time *rtctime);
static bool ra_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RA RTC lower half driver operations */

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = ra_rtc_rdtime,
  .settime     = ra_rtc_settime,
  .havesettime = ra_rtc_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = NULL,
  .setrelative = NULL,
  .cancelalarm = NULL,
  .rdalarm     = NULL,
#endif
#ifdef CONFIG_RTC_PERIODIC
  .setperiodic    = NULL,
  .cancelperiodic = NULL,
#endif
#ifdef CONFIG_RTC_IOCTL
  .ioctl       = NULL,
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  .destroy     = NULL,
#endif
};

/* RA RTC device state */

static struct ra_rtc_lowerhalf_s g_rtc_lowerhalf =
{
  .ops = &g_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_rtc_rdtime
 *
 * Description:
 *   Implements the rdtime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rtctime - The location in which to return the current RTC time.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int ra_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                         FAR struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  struct tm tm_time;
  int ret;

  ret = up_rtc_getdatetime(&tm_time);
  if (ret < 0)
    {
      rtcerr("ERROR: up_rtc_getdatetime failed: %d\n", ret);
      return ret;
    }

  /* Copy tm structure to rtc_time */
  rtctime->tm_sec   = tm_time.tm_sec;
  rtctime->tm_min   = tm_time.tm_min;
  rtctime->tm_hour  = tm_time.tm_hour;
  rtctime->tm_mday  = tm_time.tm_mday;
  rtctime->tm_mon   = tm_time.tm_mon;
  rtctime->tm_year  = tm_time.tm_year;
  rtctime->tm_wday  = tm_time.tm_wday;
  rtctime->tm_yday  = tm_time.tm_yday;
  rtctime->tm_isdst = tm_time.tm_isdst;

  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ra_rtc_settime
 *
 * Description:
 *   Implements the settime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *   rtctime - The new time to set
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

static int ra_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                          FAR const struct rtc_time *rtctime)
{
#ifdef CONFIG_RTC_DATETIME
  struct timespec ts;
  struct tm tm_time;

  /* Copy rtc_time to tm structure */
  tm_time.tm_sec   = rtctime->tm_sec;
  tm_time.tm_min   = rtctime->tm_min;
  tm_time.tm_hour  = rtctime->tm_hour;
  tm_time.tm_mday  = rtctime->tm_mday;
  tm_time.tm_mon   = rtctime->tm_mon;
  tm_time.tm_year  = rtctime->tm_year;
  tm_time.tm_wday  = rtctime->tm_wday;
  tm_time.tm_yday  = rtctime->tm_yday;
  tm_time.tm_isdst = rtctime->tm_isdst;

  /* Convert to timespec */
  ts.tv_sec = timegm(&tm_time);
  ts.tv_nsec = 0;

  return up_rtc_settime(&ts);
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: ra_rtc_havesettime
 *
 * Description:
 *   Implements the havesettime() method of the RTC driver interface
 *
 * Input Parameters:
 *   lower   - A reference to RTC lower half driver state structure
 *
 * Returned Value:
 *   Returns true if RTC date-time have been previously set.
 *
 ****************************************************************************/

static bool ra_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  extern volatile bool g_rtc_enabled;
  return g_rtc_enabled;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_rtc_lowerhalf
 *
 * Description:
 *   Instantiate the RTC lower half driver for the RA8. General usage:
 *
 *     #include "ra_rtc_lowerhalf.c"
 *     ...
 *     lower = ra_rtc_lowerhalf();
 *     rtc_initialize(0, lower);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a non-NULL RTC lower interface is returned.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *ra_rtc_lowerhalf(void)
{
  return (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
}

#endif /* CONFIG_RTC_DRIVER */
