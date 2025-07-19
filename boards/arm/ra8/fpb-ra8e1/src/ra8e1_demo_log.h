/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_demo_log.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_DEMO_LOG_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_DEMO_LOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Unified logging macros for all FPB-RA8E1 demos
 * These will automatically use the configured NuttX logging system
 * (Segger RTT, UART, or other configured output)
 */

/* Info level logging - for general demo output */
#define demoinfo(format, ...)   _info(format, ##__VA_ARGS__)

/* Error level logging - for errors and failures */
#define demoerr(format, ...)    _err(format, ##__VA_ARGS__)

/* Warning level logging - for warnings */
#define demowarn(format, ...)   _warn(format, ##__VA_ARGS__)

/* Always print logging - for critical messages */
#define demollog(format, ...)   syslog(LOG_INFO, format, ##__VA_ARGS__)

/* Interactive demo output - for user interface and results */
#define demoprintf(format, ...) printf(format, ##__VA_ARGS__)

/* Input functions for interactive demos */
#ifdef CONFIG_SEGGER_RTT
#include "SEGGER_RTT.h"
#define demo_haskey()           SEGGER_RTT_HasKey()
#define demo_getkey()           SEGGER_RTT_GetKey()
#else
/* Use standard input when RTT is not available */
#define demo_haskey()           (false)  /* Non-blocking not available with standard input */
#define demo_getkey()           getchar()
#endif

/****************************************************************************
 * Demo Logging Guidelines
 ****************************************************************************/

/* 
 * Usage guidelines for demo logging:
 *
 * 1. demoinfo() - Use for general demo status, progress updates, and 
 *    informational messages. These may be filtered based on debug level.
 *
 * 2. demoerr() - Use for errors, failures, and critical issues that 
 *    prevent demo operation.
 *
 * 3. demowarn() - Use for warnings and non-critical issues.
 *
 * 4. demollog() - Use for important messages that should always be shown
 *    regardless of debug configuration.
 *
 * 5. demoprintf() - Use for interactive demo output, data displays, and
 *    user interface elements that need to be visible for demo functionality.
 *
 * The NuttX logging system will automatically route these messages to:
 * - Segger RTT (if CONFIG_SYSLOG_RTT=y)
 * - UART/Serial (if configured)
 * - Other configured syslog outputs
 */

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_DEMO_LOG_H */
