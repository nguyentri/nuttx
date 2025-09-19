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

  int                  activation_source; /* Hardware trigger source (event link) */

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
/* UART Device Structure */
typedef struct
{
    const ra_dma_config_t *config;  /* UART configuration */
    ra_dma_state_t state;           /* Current state */

    /* DMA Control Blocks */
    void *tx_dma_ctrl;               /* TX DMA control block */
    void *rx_dma_ctrl;               /* RX DMA control block */

    /* Buffer Management */
    uint8_t *tx_buffer;              /* TX DMA buffer */
    uint8_t *rx_buffer;              /* RX DMA buffer */
    uint16_t tx_buffer_size;         /* TX buffer size */
    uint16_t rx_buffer_size;         /* RX buffer size */

    /* Transfer Tracking */
    volatile uint16_t tx_count;      /* Bytes to transmit */
    volatile uint16_t rx_count;      /* Bytes to receive */
    volatile uint32_t events;        /* Event flags */

    /* Callback */
    void (*callback)(struct ra_dma_dev_s *dev, uint32_t event);
    void *callback_context;
} ra_dma_dev_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dma_initialize
 *
 * Description:
 *   Initialize UART driver with DMA support
 *
 * Input Parameters:
 *   dev - UART device structure
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dma_initialize(ra_dma_dev_t *dev);

/****************************************************************************
 * Name: ra_dma_finalize
 *
 * Description:
 *   Finalize UART driver and free resources
 *
 * Input Parameters:
 *   dev - UART device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra_dma_finalize(ra_dma_dev_t *dev);

/****************************************************************************
 * Name: ra_dma_send_dma
 *
 * Description:
 *   Send data using DMA
 *
 * Input Parameters:
 *   dev - UART device structure
 *   buffer - Data buffer to send
 *   length - Number of bytes to send
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dma_send_dma(ra_dma_dev_t *dev, const uint8_t *buffer,
                     uint16_t length);

/****************************************************************************
 * Name: ra_dma_receive_dma
 *
 * Description:
 *   Receive data using DMA
 *
 * Input Parameters:
 *   dev - UART device structure
 *   buffer - Buffer to store received data
 *   length - Number of bytes to receive
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dma_receive_dma(ra_dma_dev_t *dev, uint8_t *buffer,
                        uint16_t length);

/****************************************************************************
 * Name: ra_dma_abort_transfer
 *
 * Description:
 *   Abort ongoing DMA transfer
 *
 * Input Parameters:
 *   dev - UART device structure
 *   tx - true to abort TX, false to abort RX
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dma_abort_transfer(ra_dma_dev_t *dev, bool tx);

/****************************************************************************
 * Name: ra_dma_set_callback
 *
 * Description:
 *   Set callback function for UART events
 *
 * Input Parameters:
 *   dev - UART device structure
 *   callback - Callback function
 *   context - Context passed to callback
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra_dma_set_callback(ra_dma_dev_t *dev,
                          void (*callback)(ra_dma_dev_t *dev, uint32_t event),
                          void *context);

/****************************************************************************
 * Name: ra_dma_config_baudrate
 *
 * Description:
 *   Configure UART baud rate
 *
 * Input Parameters:
 *   dev - UART device structure
 *   baud - Desired baud rate
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dma_config_baudrate(ra_dma_dev_t *dev, uint32_t baud);

/****************************************************************************
 * Name: ra_dma_get_status
 *
 * Description:
 *   Get UART status
 *
 * Input Parameters:
 *   dev - UART device structure
 *
 * Returned Value:
 *   Current status register value
 *
 ****************************************************************************/

uint32_t ra_dma_get_status(ra_dma_dev_t *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8_RA_DMA_H */
