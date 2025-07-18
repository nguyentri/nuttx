/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_pwm_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_PWM_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_PWM_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESC PWM Parameters */

#define NUM_ESC_CHANNELS        4
#define ESC_PWM_FREQUENCY       400      /* 400Hz for ESC control */
#define ESC_PWM_MIN_US          1000     /* Minimum pulse width (microseconds) */
#define ESC_PWM_MAX_US          2000     /* Maximum pulse width (microseconds) */
#define ESC_PWM_ARM_US          1000     /* Arming pulse width */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ESC Status Structure */

struct esc_status_s
{
  bool armed;                   /* ESC armed status */
  uint16_t throttle_percent;    /* Current throttle percentage */
  uint32_t pulse_width_us;      /* Current pulse width in microseconds */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ra8e1_pwm_demo_main
 *
 * Description:
 *   Main entry point for PWM ESC control demo
 *
 * Input Parameters:
 *   argc - Number of command line arguments
 *   argv - Array of command line arguments
 *
 * Returned Value:
 *   Zero on success; negative value on failure
 *
 ****************************************************************************/

int ra8e1_pwm_demo_main(int argc, char *argv[]);

/****************************************************************************
 * Name: ra8e1_esc_get_status
 *
 * Description:
 *   Get current status of specified ESC channel
 *
 * Input Parameters:
 *   esc_index - ESC channel index (0-3)
 *   status    - Pointer to status structure to fill
 *
 * Returned Value:
 *   Zero on success; negative value on failure
 *
 ****************************************************************************/

int ra8e1_esc_get_status(int esc_index, struct esc_status_s *status);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_PWM_DEMO_H */
