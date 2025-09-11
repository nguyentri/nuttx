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
  /* MSTPCRA register modules */
  RA_MSTP_UNNECESSARY = 0,  /* MSTPA0: Unnecessary Circuit */
  RA_MSTP_SRAM1,            /* MSTPA1: SRAM1 */
  RA_MSTP_STANDBY_SRAM,     /* MSTPA15: Standby SRAM */
  RA_MSTP_DMAC_DTC,         /* MSTPA22: DMA Controller and Data Transfer Controller */

  /* MSTPCRB register modules */
  RA_MSTP_SCI0,             /* MSTPB31: Serial Communication Interface 0 */
  RA_MSTP_SCI1,             /* MSTPB30: Serial Communication Interface 1 */
  RA_MSTP_SCI2,             /* MSTPB29: Serial Communication Interface 2 */
  RA_MSTP_SCI3,             /* MSTPB28: Serial Communication Interface 3 */
  RA_MSTP_SCI4,             /* MSTPB27: Serial Communication Interface 4 */
  RA_MSTP_SCI9,             /* MSTPB22: Serial Communication Interface 9 */
  RA_MSTP_SPI0,             /* MSTPB19: Serial Peripheral Interface 0 */
  RA_MSTP_SPI1,             /* MSTPB18: Serial Peripheral Interface 1 */
  RA_MSTP_SCI10,            /* MSTPB16: SCI Communication Interface 10 */
  RA_MSTP_ETHERCAT,         /* MSTPB15: EtherCAT */
  RA_MSTP_USBFS,            /* MSTPB11: Universal Serial Bus 2.0 FS Interface */
  RA_MSTP_IIC0,             /* MSTPB9: I2C Bus Interface 0 */
  RA_MSTP_IIC1,             /* MSTPB8: I2C Bus Interface 1 */

  /* MSTPCRC register modules */
  RA_MSTP_SCE5,             /* MSTPC31: Renesas Secure IP */
  RA_MSTP_CANFD0,           /* MSTPC27: Controller Area Network with Flexible Data-Rate 0 */
  RA_MSTP_CANFD1,           /* MSTPC26: Controller Area Network with Flexible Data-Rate 1 */
  RA_MSTP_CEU,              /* MSTPC16: Capture Engine Unit */
  RA_MSTP_ELC,              /* MSTPC14: Event Link Controller */
  RA_MSTP_DOC,              /* MSTPC13: Data Operation Circuit */
  RA_MSTP_SSIE0,            /* MSTPC8: Serial Sound Interface Enhanced 0 */
  RA_MSTP_SSIE1,            /* MSTPC7: Serial Sound Interface Enhanced 1 */
  RA_MSTP_CRC,              /* MSTPC1: Cyclic Redundancy Check Calculator */
  RA_MSTP_CAC,              /* MSTPC0: Clock Frequency Accuracy Measurement Circuit */

  /* MSTPCRD register modules */
  RA_MSTP_ACMPHS0,          /* MSTPD28: High-Speed Analog Comparator 0 */
  RA_MSTP_ACMPHS1,          /* MSTPD27: High-Speed Analog Comparator 1 */
  RA_MSTP_RTC,              /* MSTPD23: Real Time Clock */
  RA_MSTP_TSN,              /* MSTPD22: Temperature Sensor */
  RA_MSTP_DAC12,            /* MSTPD20: 12-bit D/A Converter */
  RA_MSTP_ADC0,             /* MSTPD16: 12-bit A/D Converter 0 */
  RA_MSTP_ADC1,             /* MSTPD15: 12-bit A/D Converter 1 */
  RA_MSTP_POEG0,            /* MSTPD14: Port Output Enable for GPT Group A */
  RA_MSTP_POEG1,            /* MSTPD13: Port Output Enable for GPT Group B */
  RA_MSTP_POEG2,            /* MSTPD12: Port Output Enable for GPT Group C */
  RA_MSTP_POEG3,            /* MSTPD11: Port Output Enable for GPT Group D */
  RA_MSTP_AGT0,             /* MSTPD5: Low Power Asynchronous General Purpose Timer 0 */
  RA_MSTP_AGT1,             /* MSTPD4: Low Power Asynchronous General Purpose Timer 1 */

  /* MSTPCRE register modules */
  RA_MSTP_GPT0,             /* MSTPE31: General PWM Timer 0 */
  RA_MSTP_GPT1,             /* MSTPE30: General PWM Timer 1 */
  RA_MSTP_GPT2,             /* MSTPE29: General PWM Timer 2 */
  RA_MSTP_GPT3,             /* MSTPE28: General PWM Timer 3 */
  RA_MSTP_GPT4,             /* MSTPE27: General PWM Timer 4 */
  RA_MSTP_GPT5,             /* MSTPE26: General PWM Timer 5 */
  RA_MSTP_GPT10,            /* MSTPE21: General PWM Timer 10 */
  RA_MSTP_GPT11,            /* MSTPE20: General PWM Timer 11 */
  RA_MSTP_GPT12,            /* MSTPE19: General PWM Timer 12 */
  RA_MSTP_GPT13,            /* MSTPE18: General PWM Timer 13 */
  RA_MSTP_ULPT0,            /* MSTPE9: Ultra-Low Power Timer 0 */
  RA_MSTP_ULPT1,            /* MSTPE8: Ultra-Low Power Timer 1 */

  RA_MSTP_MAX_MODULE
} ra_mstp_module_t;

/* Compatibility aliases */
#define RA_MSTP_CANFD             RA_MSTP_CANFD0
#define RA_MSTP_SSI0              RA_MSTP_SSIE0
#define RA_MSTP_SSI1              RA_MSTP_SSIE1
#define RA_MSTP_ACMPHS            RA_MSTP_ACMPHS0
#define RA_MSTP_POEG              RA_MSTP_POEG0
#define RA_MSTP_DAC               RA_MSTP_DAC12

/* MSTP status structure */

typedef struct
{
  uint32_t mstpcra;    /* MSTPCRA register value */
  uint32_t mstpcrb;    /* MSTPCRB register value */
  uint32_t mstpcrc;    /* MSTPCRC register value */
  uint32_t mstpcrd;    /* MSTPCRD register value */
  uint32_t mstpcre;    /* MSTPCRE register value */
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
