/****************************************************************************
 * arch/arm/src/ra8/ra_icu.h
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

#ifndef __ARCH_ARM_SRC_RA_ICU_H
#define __ARCH_ARM_SRC_RA_ICU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/ra_icu.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* ICU detection modes */
#define RA_ICU_IRQ_LEVEL_LOW        0x00  /* Level-triggered (low) */
#define RA_ICU_IRQ_EDGE_FALLING     0x01  /* Edge-triggered (falling) */
#define RA_ICU_IRQ_EDGE_RISING      0x02  /* Edge-triggered (rising) */
#define RA_ICU_IRQ_EDGE_BOTH        0x03  /* Edge-triggered (both) */

/* ICU filter clock selections */
#define RA_ICU_FILTER_PCLK_DIV_1    0x00  /* PCLKB */
#define RA_ICU_FILTER_PCLK_DIV_8    0x01  /* PCLKB/8 */
#define RA_ICU_FILTER_PCLK_DIV_32   0x02  /* PCLKB/32 */
#define RA_ICU_FILTER_PCLK_DIV_64   0x03  /* PCLKB/64 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Legacy functions */
void ra_attach_icu(void);
void ra_clear_ir(int irq);

/* Enhanced ICU driver functions */
void ra_icu_initialize(void);
int ra_icu_attach(int icu_irq, int (*handler)(int, void *, void *), void *arg);
int ra_icu_detach(int icu_irq);
void ra_icu_enable(int icu_irq);
void ra_icu_disable(int icu_irq);
int ra_icu_config(int icu_irq, uint8_t mode, bool filter_enable, 
                  uint8_t filter_clock);
int ra_icu_set_event(int icu_slot, int event);

/* Wakeup control functions */
void ra_icu_enable_wakeup(uint32_t mask);
void ra_icu_disable_wakeup(uint32_t mask);

/* NMI control functions */
void ra_icu_clear_nmi_status(uint16_t mask);
uint16_t ra_icu_get_nmi_status(void);
void ra_icu_enable_nmi(uint16_t mask);
void ra_icu_disable_nmi(uint16_t mask);

#ifdef __cplusplus
}
#endif

#endif
