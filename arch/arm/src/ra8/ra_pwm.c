/****************************************************************************
 * arch/arm/src/ra8/ra_pwm.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "hardware/ra_gpt.h"
#include "hardware/ra_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM debug output */

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwminfo _info
#else
#  define pwminfo(x...)
#endif

#ifdef CONFIG_DEBUG_PWM_ERROR
#  define pwmerr _err
#else
#  define pwmerr(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one PWM timer */

struct ra8_pwm_s
{
  const struct pwm_ops_s *ops;     /* PWM operations */
  uint8_t channel;                 /* PWM channel number */
  bool started;                    /* True: PWM started */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver
 */

static const struct pwm_ops_s g_pwm_ops =
{
  .setup      = pwm_setup,
  .shutdown   = pwm_shutdown,
  .start      = pwm_start,
  .stop       = pwm_stop,
  .ioctl      = pwm_ioctl,
};

/* PWM device configurations */

static struct ra8_pwm_s g_pwm_devs[] =
{
#ifdef CONFIG_RA_PWM0
  {
    .ops     = &g_pwm_ops,
    .channel = 0,
  },
#endif
#ifdef CONFIG_RA_PWM1
  {
    .ops     = &g_pwm_ops,
    .channel = 1,
  },
#endif
#ifdef CONFIG_RA_PWM2
  {
    .ops     = &g_pwm_ops,
    .channel = 2,
  },
#endif
#ifdef CONFIG_RA_PWM3
  {
    .ops     = &g_pwm_ops,
    .channel = 3,
  },
#endif
/* Add more channels as needed */
};

#define NPWM_DEVS (sizeof(g_pwm_devs) / sizeof(struct ra8_pwm_s))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct ra8_pwm_s *priv = (struct ra8_pwm_s *)dev;
  struct pwm_lowerhalf_s *gpt_dev;

  pwminfo("PWM%d setup\n", priv->channel);

  /* Get the underlying GPT device */

  gpt_dev = ra8_gpt_initialize(priv->channel);
  if (gpt_dev == NULL)
    {
      pwmerr("ERROR: Failed to initialize GPT%d\n", priv->channel);
      return -ENODEV;
    }

  /* Setup the GPT timer */

  return gpt_dev->ops->setup(gpt_dev);
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct ra8_pwm_s *priv = (struct ra8_pwm_s *)dev;
  struct pwm_lowerhalf_s *gpt_dev;

  pwminfo("PWM%d shutdown\n", priv->channel);

  /* Get the underlying GPT device */

  gpt_dev = ra8_gpt_initialize(priv->channel);
  if (gpt_dev == NULL)
    {
      return -ENODEV;
    }

  /* Shutdown the GPT timer */

  priv->started = false;
  return gpt_dev->ops->shutdown(gpt_dev);
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct ra8_pwm_s *priv = (struct ra8_pwm_s *)dev;
  struct pwm_lowerhalf_s *gpt_dev;
  int ret;

  pwminfo("PWM%d start: frequency=%lu duty=%08lx\n",
          priv->channel, info->frequency, info->duty);

  DEBUGASSERT(info->frequency > 0);

  /* Get the underlying GPT device */

  gpt_dev = ra8_gpt_initialize(priv->channel);
  if (gpt_dev == NULL)
    {
      return -ENODEV;
    }

  /* Start the GPT timer with PWM configuration */

  ret = gpt_dev->ops->start(gpt_dev, info);
  if (ret == OK)
    {
      priv->started = true;
    }

  return ret;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct ra8_pwm_s *priv = (struct ra8_pwm_s *)dev;
  struct pwm_lowerhalf_s *gpt_dev;

  pwminfo("PWM%d stop\n", priv->channel);

  /* Get the underlying GPT device */

  gpt_dev = ra8_gpt_initialize(priv->channel);
  if (gpt_dev == NULL)
    {
      return -ENODEV;
    }

  /* Stop the GPT timer */

  priv->started = false;
  return gpt_dev->ops->stop(gpt_dev);
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
  struct ra8_pwm_s *priv = (struct ra8_pwm_s *)dev;
  struct pwm_lowerhalf_s *gpt_dev;

  pwminfo("PWM%d ioctl: cmd=%d arg=%08lx\n", priv->channel, cmd, arg);

  /* Get the underlying GPT device */

  gpt_dev = ra8_gpt_initialize(priv->channel);
  if (gpt_dev == NULL)
    {
      return -ENODEV;
    }

  /* Forward ioctl to the GPT timer */

  return gpt_dev->ops->ioctl(gpt_dev, cmd, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8_pwm_initialize
 *
 * Description:
 *   Initialize one PWM channel for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   channel - A number identifying the PWM channel.
 *
 * Returned Value:
 *   On success, a pointer to the RA8 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *ra8_pwm_initialize(int channel)
{
  struct ra8_pwm_s *lower;
  int i;

  pwminfo("PWM%d initialize\n", channel);

  /* Find the matching channel */

  for (i = 0; i < NPWM_DEVS; i++)
    {
      if (g_pwm_devs[i].channel == channel)
        {
          lower = &g_pwm_devs[i];
          break;
        }
    }

  if (i >= NPWM_DEVS)
    {
      pwmerr("ERROR: No such PWM configured: %d\n", channel);
      return NULL;
    }

  return (struct pwm_lowerhalf_s *)lower;
}

/****************************************************************************
 * Name: ra8_pwm_setup
 *
 * Description:
 *   Configure the PWM channel.
 *
 * Input Parameters:
 *   dev  - A reference to the PWM device structure
 *   info - PWM configuration information
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8_pwm_setup(struct pwm_lowerhalf_s *dev, 
                  const struct pwm_info_s *info)
{
  if (dev == NULL || info == NULL)
    {
      return -EINVAL;
    }

  return dev->ops->setup(dev);
}

/****************************************************************************
 * Name: ra8_pwm_shutdown
 *
 * Description:
 *   Disable the PWM channel.
 *
 * Input Parameters:
 *   dev - A reference to the PWM device structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8_pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  return dev->ops->shutdown(dev);
}

/****************************************************************************
 * Name: ra8_pwm_start
 *
 * Description:
 *   Start the PWM output.
 *
 * Input Parameters:
 *   dev  - A reference to the PWM device structure
 *   info - PWM configuration information
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8_pwm_start(struct pwm_lowerhalf_s *dev,
                  const struct pwm_info_s *info)
{
  if (dev == NULL || info == NULL)
    {
      return -EINVAL;
    }

  return dev->ops->start(dev, info);
}

/****************************************************************************
 * Name: ra8_pwm_stop
 *
 * Description:
 *   Stop the PWM output.
 *
 * Input Parameters:
 *   dev - A reference to the PWM device structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8_pwm_stop(struct pwm_lowerhalf_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  return dev->ops->stop(dev);
}

/****************************************************************************
 * Name: ra8_pwm_ioctl
 *
 * Description:
 *   Handle PWM ioctl commands.
 *
 * Input Parameters:
 *   dev - A reference to the PWM device structure
 *   cmd - The ioctl command
 *   arg - The argument of the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8_pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd, unsigned long arg)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  return dev->ops->ioctl(dev, cmd, arg);
}
