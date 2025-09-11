/****************************************************************************
 * apps/examples/cyclic_simple/cyclic_simple.c
 *
 * Simple cyclic logger that can be called directly from bringup
 * (No task creation needed - runs in bringup context)
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <syslog.h>
#include <nuttx/kthread.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void *cyclic_simple_thread(void *arg)
{
  struct timespec start_time;
  uint32_t counter = 0;

  syslog(LOG_INFO, "Cyclic Simple Logger: Starting 1-second interval task\n");

  /* Get start time for reference */
  clock_gettime(CLOCK_REALTIME, &start_time);

  while (1)
    {
      struct timespec current_time;
      clock_gettime(CLOCK_REALTIME, &current_time);

      /* Calculate elapsed time since start */
      long elapsed_sec = current_time.tv_sec - start_time.tv_sec;
      long elapsed_nsec = current_time.tv_nsec - start_time.tv_nsec;

      if (elapsed_nsec < 0)
        {
          elapsed_sec--;
          elapsed_nsec += 1000000000;
        }

      double elapsed_total = elapsed_sec + elapsed_nsec / 1000000000.0;

      /* Print log message with timing information */
      syslog(LOG_INFO, "[CYCLIC] Count: %lu, Elapsed: %.3fs, Tick: %lu",
             (unsigned long)counter,
             elapsed_total,
             (unsigned long)clock());

      counter++;

      /* Sleep for 1 second */
      sleep(1);
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * cyclic_simple_start
 ****************************************************************************/

int cyclic_simple_start(void)
{
  pid_t pid;

  /* Create a kernel thread for the cyclic logger */
  pid = kthread_create("cyclic_simple", 100, 2048,
                       (main_t)cyclic_simple_thread, NULL);

  if (pid < 0)
    {
      syslog(LOG_ERR, "Failed to create cyclic simple thread: %d\n", pid);
      return pid;
    }

  syslog(LOG_INFO, "Cyclic simple logger started (PID: %d)\n", pid);
  return 0;
}
