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

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/ra_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAC Events */
#define RA_DMAC_EVENT_COMPLETE   (0)  /* Transfer complete */
#define RA_DMAC_EVENT_ERROR      (1)  /* Transfer error */

/* DMAC Limits */
#define RA_DMAC_MAX_NORMAL_LENGTH     (0x10000)  /* Maximum transfers in normal mode */
#define RA_DMAC_MAX_REPEAT_LENGTH     (0x400)    /* Maximum transfers in repeat/block mode */
#define RA_DMAC_MAX_BLOCK_COUNT       (0x100)    /* Maximum block count */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declaration */
typedef void *ra_dmac_handle_t;

/* DMAC callback function type */
typedef void (*ra_dmac_callback_t)(void *handle, int event, void *user_data);

/* DMAC configuration structure */
typedef struct ra_dmac_config_s
{
  ra_dmac_mode_t       mode;             /* Transfer mode */
  ra_dmac_size_t       size;             /* Transfer data size */
  ra_dmac_addr_mode_t  src_addr_mode;    /* Source address mode */
  ra_dmac_addr_mode_t  dest_addr_mode;   /* Destination address mode */
  ra_dmac_trigger_t    trigger;          /* Trigger mode */

  uint32_t             src_addr;         /* Source address */
  uint32_t             dest_addr;        /* Destination address */
  uint32_t             transfer_count;   /* Number of transfers */
  uint32_t             block_count;      /* Number of blocks (block mode) */

  int                  elc_end;           /* DMA End of transfer event link */
  int                  elc_err;           /* DMA Error event link */
  int                  elc_src;           /* Even link of activation source */

  ra_dmac_callback_t   callback;         /* Transfer callback */
  void                *user_data;        /* User data for callback */
} ra_dmac_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* DMAC module functions */
int ra_dmac_initialize(void);

/* DMAC transfer functions */
int ra_dmac_open(ra_dmac_handle_t *handle, const ra_dmac_config_t *config);
int ra_dmac_close(ra_dmac_handle_t handle);
int ra_dmac_enable(ra_dmac_handle_t handle);
int ra_dmac_disable(ra_dmac_handle_t handle);
int ra_dmac_software_start(ra_dmac_handle_t handle);
int ra_dmac_reset(ra_dmac_handle_t handle, uint32_t src_addr,
                  uint32_t dest_addr, uint32_t transfer_count);

/* DMAC status functions */
uint32_t ra_dmac_get_remaining_count(ra_dmac_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_DMAC_H */