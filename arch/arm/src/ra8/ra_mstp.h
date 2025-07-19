/****************************************************************************
 * arch/arm/src/ra8/ra_mstp.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_MSTP_H
#define __ARCH_ARM_SRC_RA8_RA_MSTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module enumeration for MSTP control */

typedef enum
{
  /* MSTPCRB register modules */
  RA_MSTP_SCI0 = 0,
  RA_MSTP_SCI1,
  RA_MSTP_SCI2,
  RA_MSTP_SCI3,
  RA_MSTP_SCI4,
  RA_MSTP_SCI5,
  RA_MSTP_SCI6,
  RA_MSTP_SCI7,
  RA_MSTP_SCI8,
  RA_MSTP_SCI9,
  RA_MSTP_SPI0,
  RA_MSTP_SPI1,
  RA_MSTP_USBFS,
  RA_MSTP_IIC0,
  RA_MSTP_IIC1,
  RA_MSTP_IIC2,
  RA_MSTP_CANFD,

  /* MSTPCRC register modules */
  RA_MSTP_SCE5,
  RA_MSTP_TRNG,
  RA_MSTP_JPEG,
  RA_MSTP_EDMAC0,
  RA_MSTP_ELC,
  RA_MSTP_DOC,
  RA_MSTP_SSIE0,
  RA_MSTP_SLCDC,
  RA_MSTP_CTSU,
  RA_MSTP_CRC,
  RA_MSTP_CAC,

  /* MSTPCRD register modules */
  RA_MSTP_OPAMP,
  RA_MSTP_ACMPLP,
  RA_MSTP_ACMPHS,
  RA_MSTP_ULPT1,
  RA_MSTP_ULPT0,
  RA_MSTP_CEU,
  RA_MSTP_DAC12,
  RA_MSTP_DAC8,
  RA_MSTP_TSN,
  RA_MSTP_ADC1,
  RA_MSTP_ADC0,
  RA_MSTP_POEG,
  RA_MSTP_GPT_1,
  RA_MSTP_GPT_2,
  RA_MSTP_AGT0,
  RA_MSTP_AGT1,

  RA_MSTP_MAX_MODULE
} ra_mstp_module_t;

/* MSTP status structure */

typedef struct
{
  uint32_t mstpcrb;    /* MSTPCRB register value */
  uint32_t mstpcrc;    /* MSTPCRC register value */
  uint32_t mstpcrd;    /* MSTPCRD register value */
} ra_mstp_status_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ra_mstp_initialize
 *
 * Description:
 *   Initialize the MSTP driver
 *
 ****************************************************************************/

void ra_mstp_initialize(void);

/****************************************************************************
 * Name: ra_mstp_start
 *
 * Description:
 *   Start (enable clock for) a peripheral module
 *
 * Input Parameters:
 *   module - The module to start
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 ****************************************************************************/

int ra_mstp_start(ra_mstp_module_t module);

/****************************************************************************
 * Name: ra_mstp_stop
 *
 * Description:
 *   Stop (disable clock for) a peripheral module
 *
 * Input Parameters:
 *   module - The module to stop
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 ****************************************************************************/

int ra_mstp_stop(ra_mstp_module_t module);

/****************************************************************************
 * Name: ra_mstp_is_stopped
 *
 * Description:
 *   Check if a peripheral module is stopped
 *
 * Input Parameters:
 *   module - The module to check
 *
 * Returned Value:
 *   true if module is stopped; false if running
 *
 ****************************************************************************/

bool ra_mstp_is_stopped(ra_mstp_module_t module);

/****************************************************************************
 * Name: ra_mstp_start_multiple
 *
 * Description:
 *   Start multiple modules atomically
 *
 * Input Parameters:
 *   modules - Array of modules to start
 *   count   - Number of modules in array
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 ****************************************************************************/

int ra_mstp_start_multiple(const ra_mstp_module_t *modules, int count);

/****************************************************************************
 * Name: ra_mstp_stop_multiple
 *
 * Description:
 *   Stop multiple modules atomically
 *
 * Input Parameters:
 *   modules - Array of modules to stop
 *   count   - Number of modules in array
 *
 * Returned Value:
 *   OK on success; negated errno on failure
 *
 ****************************************************************************/

int ra_mstp_stop_multiple(const ra_mstp_module_t *modules, int count);

/****************************************************************************
 * Name: ra_mstp_get_status
 *
 * Description:
 *   Get the current MSTP register status
 *
 * Input Parameters:
 *   status - Pointer to status structure to fill
 *
 ****************************************************************************/

void ra_mstp_get_status(ra_mstp_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_MSTP_H */
