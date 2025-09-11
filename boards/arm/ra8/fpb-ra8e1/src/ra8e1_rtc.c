/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra_rtc.c
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

#include <nuttx/timers/rtc.h>
#include <nuttx/arch.h>

#ifdef CONFIG_RTC_ARCH
#include <nuttx/timers/arch_rtc.h>
#endif

#ifdef CONFIG_RTC_DRIVER
#include "ra_rtc_lowerhalf.c"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_rtc_initialize
 *
 * Description:
 *   Initialize the RTC driver for the FPB-RA8E1 board
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
int ra_rtc_initialize(void)
{
  FAR struct rtc_lowerhalf_s *lower;
  int ret;

  /* Get the RTC lowerhalf driver */
  lower = ra_rtc_lowerhalf();
  if (lower == NULL)
    {
      rtcerr("ERROR: Failed to get RTC lowerhalf\n");
      return -ENODEV;
    }

  /* Register the RTC driver */
  ret = rtc_initialize(0, lower);
  if (ret < 0)
    {
      rtcerr("ERROR: Failed to register RTC driver: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_RTC_ARCH
  /* For CONFIG_RTC_HIRES, register the lowerhalf with the generic arch RTC */
  up_rtc_set_lowerhalf(lower, true);
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: board_rtc_initialize
 *
 * Description:
 *   Initialize and register the RTC driver for the board
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

#ifdef CONFIG_RTC_DRIVER
int board_rtc_initialize(void)
{
  return ra_rtc_initialize();
}
#endif
