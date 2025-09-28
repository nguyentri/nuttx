/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/pwm_status.c
 *
 * Simple status app to query PWM devices and print characteristics.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include <nuttx/timers/pwm.h>

/* Some build-analysis contexts may not expose the full pwm.h symbols; provide
 * a minimal local fallback so this utility can still build for analysis.
 */
#ifndef PWMIOC_GETCHARACTERISTICS
/* If PWMIOC_GETCHARACTERISTICS isn't available in the include context then
 * we cannot query the upper-half via ioctl; print a message instead.
 */
#define PWM_STATUS_IOCTL_UNAVAILABLE 1
#endif

int pwm_status_main(int argc, char *argv[])
{
  const char *devs[] = { "/dev/pwm0", "/dev/pwm1", "/dev/pwm2", "/dev/pwm3", "/dev/pwm4", "/dev/pwm5" };
  int nd = sizeof(devs) / sizeof(devs[0]);

  for (int i = 0; i < nd; i++)
    {
      int fd = open(devs[i], O_RDONLY);
      if (fd < 0)
        {
          syslog(LOG_INFO, "%s: not present (err=%d)", devs[i], errno);
          continue;
        }

#ifndef PWM_STATUS_IOCTL_UNAVAILABLE
      struct pwm_info_s info;
      int ret = ioctl(fd, PWMIOC_GETCHARACTERISTICS, (unsigned long)&info);
      if (ret < 0)
        {
          syslog(LOG_INFO, "%s: PWMIOC_GETCHARACTERISTICS failed: %d", devs[i], ret);
          close(fd);
          continue;
        }

      uint32_t freq = info.frequency;
#ifdef CONFIG_PWM_MULTICHAN
      if (info.count > 1)
        {
          uint32_t duty0 = info.channels[0].duty;
          uint32_t duty1 = info.channels[1].duty;
          uint32_t period_us = (freq > 0) ? (1000000U / freq) : 0;
          uint32_t pulse0_us = (uint32_t)(((uint64_t)duty0 * period_us) >> 16);
          uint32_t pulse1_us = (uint32_t)(((uint64_t)duty1 * period_us) >> 16);
     syslog(LOG_INFO, "%s: freq=%" PRIu32 "Hz period~%" PRIu32 "us ch0 duty=%" PRIu32 " (~%" PRIu32 "us) ch1 duty=%" PRIu32 " (~%" PRIu32 "us)",
       devs[i], freq, period_us, duty0, pulse0_us, duty1, pulse1_us);
        }
      else
#endif
        {
          uint32_t duty = info.duty;
          uint32_t period_us = (freq > 0) ? (1000000U / freq) : 0;
          uint32_t pulse_us = (uint32_t)(((uint64_t)duty * period_us) >> 16);
     syslog(LOG_INFO, "%s: freq=%" PRIu32 "Hz period~%" PRIu32 "us duty=%" PRIu32 " (~%" PRIu32 "us)",
       devs[i], freq, period_us, duty, pulse_us);
        }

      close(fd);
#else
      syslog(LOG_INFO, "%s: PWM ioctl not available in this build; skipping", devs[i]);
      close(fd);
      continue;
#endif
    }

  return 0;
}