/****************************************************************************
 * arch/arm/src/common/arm_idle.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/power/pm.h>

#include <debug.h>
#include <time.h>
#include <syslog.h>

#include "arm_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* Local heartbeat every 1000ms */
  static struct timespec last_idle = {0, 0};
  struct timespec current_time;

  /* Get current time */
  if (clock_gettime(CLOCK_MONOTONIC, &current_time) == 0)
    {
      /* Check if 1 second has passed */
      time_t time_diff = current_time.tv_sec - last_idle.tv_sec;
      long nsec_diff = current_time.tv_nsec - last_idle.tv_nsec;

      /* Calculate total difference in milliseconds */
      long total_ms = (time_diff * 1000) + (nsec_diff / 1000000);

      if (total_ms >= 1000)
        {
          last_idle = current_time;

          /* Get current time and format it */
          struct timespec ts;
          struct tm tm_info;
          time_t time_for_format;
          char time_str[32];

          if (clock_gettime(CLOCK_REALTIME, &ts) == 0)
            {
              time_for_format = ts.tv_sec;
              if (gmtime_r(&time_for_format, &tm_info) != NULL)
                {
                  strftime(time_str, sizeof(time_str), "%d/%m/%y %H:%M:%S", &tm_info);
                  syslog(LOG_INFO, "Nuttx IDLE Heart Beat: %s.%03ld\n",
                         time_str, ts.tv_nsec / 1000000);
                }
              else
                {
                  syslog(LOG_INFO, "Nuttx IDLE Heart Beat: %lu.%03ld sec\n",
                         (unsigned long)ts.tv_sec, ts.tv_nsec / 1000000);
                }
            }
          else
            {
              syslog(LOG_INFO, "Nuttx IDLE Heart Beat: %lu.%03ld sec\n",
                     (unsigned long)current_time.tv_sec, current_time.tv_nsec / 1000000);
            }

#ifdef CONFIG_PM
          /* Perform power management idle operations */
          pm_idle(NULL);
#endif
        }
    }

#if 0
  /* Sleep until an interrupt occurs to save power */
  BEGIN_IDLE();
  asm("WFI");
  END_IDLE();
#endif
#endif
}
