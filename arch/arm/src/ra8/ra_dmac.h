/****************************************************************************
 * arch/arm/src/ra8/ra_dmac.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_DMAC_H
#define __ARCH_ARM_SRC_RA8_RA_DMAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ra_dmac_initialize
 *
 * Description:
 *   Initialize the DMAC controller
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_dmac_initialize(void);

/****************************************************************************
 * Name: ra_dmac_channel_alloc
 *
 * Description:
 *   Allocate a DMAC channel
 *
 * Returned Value:
 *   Channel number on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_dmac_channel_alloc(void);

/****************************************************************************
 * Name: ra_dmac_channel_free
 *
 * Description:
 *   Free a DMAC channel
 *
 * Input Parameters:
 *   channel - The channel to free
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_dmac_channel_free(int channel);

/****************************************************************************
 * Name: ra_dmac_start
 *
 * Description:
 *   Start DMAC transfer
 *
 * Input Parameters:
 *   channel - DMAC channel number
 *   src     - Source address
 *   dst     - Destination address
 *   len     - Transfer length
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_dmac_start(int channel, uint32_t src, uint32_t dst, uint32_t len);

/****************************************************************************
 * Name: ra_dmac_stop
 *
 * Description:
 *   Stop DMAC transfer
 *
 * Input Parameters:
 *   channel - DMAC channel number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra_dmac_stop(int channel);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_DMAC_H */
