/****************************************************************************
 * arch/arm/src/ra8/ra_dtc.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_DTC_H
#define __ARCH_ARM_SRC_RA8_RA_DTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/ra_dtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DTC Events */
#define RA_DTC_EVENT_COMPLETE    (0)  /* Transfer complete */
#define RA_DTC_EVENT_ERROR       (1)  /* Transfer error */

/* DTC Limits */
#define RA_DTC_MAX_TRANSFER_COUNT     (0x10000)  /* Maximum transfers in normal mode */
#define RA_DTC_MAX_BLOCK_COUNT        (0x100)    /* Maximum block count */
#define RA_DTC_MAX_CONTEXTS           (32)       /* Maximum DTC contexts */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declaration */
typedef void *ra_dtc_handle_t;

/* DTC callback function type */
typedef void (*ra_dtc_callback_t)(void *handle, int event, void *user_data);

/* DTC configuration structure */
typedef struct ra_dtc_config_s
{
  ra_dtc_mode_t        mode;             /* Transfer mode */
  ra_dtc_size_t        size;             /* Transfer data size */
  ra_dtc_addr_mode_t   src_addr_mode;    /* Source address mode */
  ra_dtc_addr_mode_t   dest_addr_mode;   /* Destination address mode */
  bool                 software_trigger; /* Software trigger mode */

  uint32_t             src_addr;         /* Source address */
  uint32_t             dest_addr;        /* Destination address */
  uint32_t             transfer_count;   /* Number of transfers */
  uint32_t             block_count;      /* Number of blocks (block mode) */

  int                  elc_src; /* Hardware trigger source (event link) */

  ra_dtc_callback_t    callback;         /* Transfer callback */
  void                *user_data;        /* User data for callback */
} ra_dtc_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* DTC module functions */
int ra_dtc_initialize(void);

/* DTC transfer functions */
int ra_dtc_open(ra_dtc_handle_t *handle, const ra_dtc_config_t *config);
int ra_dtc_close(ra_dtc_handle_t handle);
int ra_dtc_enable(ra_dtc_handle_t handle);
int ra_dtc_disable(ra_dtc_handle_t handle);
int ra_dtc_software_start(ra_dtc_handle_t handle);
int ra_dtc_reset(ra_dtc_handle_t handle, uint32_t src_addr,
                 uint32_t dest_addr, uint32_t transfer_count);

/* DTC status functions */
uint32_t ra_dtc_get_remaining_count(ra_dtc_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_RA8_RA_DTC_H */
