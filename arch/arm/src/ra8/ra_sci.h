/****************************************************************************
 * arch/arm/src/ra8/ra_sci.h
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

#ifndef __ARCH_ARM_SRC_RA8_RA_UART_H
#define __ARCH_ARM_SRC_RA8_RA_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer size macros - map Kconfig to driver constants */
#ifndef CONFIG_RA_UART_TX_BUFFER_SIZE
#  define CONFIG_RA_UART_TX_BUFFER_SIZE 64
#endif

#ifndef CONFIG_RA_UART_RX_BUFFER_SIZE
#  define CONFIG_RA_UART_RX_BUFFER_SIZE 64
#endif

#ifdef CONFIG_UART0_SERIALDRIVER
#  ifdef CONFIG_UART0_RX_BUFFER_SIZE
#    define RA_UART0_RX_BUFFER_SIZE CONFIG_UART0_RX_BUFFER_SIZE
#  else
#    define RA_UART0_RX_BUFFER_SIZE CONFIG_RA_UART_RX_BUFFER_SIZE
#  endif
#  ifdef CONFIG_UART0_TX_BUFFER_SIZE
#    define RA_UART0_TX_BUFFER_SIZE CONFIG_UART0_TX_BUFFER_SIZE
#  else
#    define RA_UART0_TX_BUFFER_SIZE CONFIG_RA_UART_TX_BUFFER_SIZE
#  endif
#endif

#ifdef CONFIG_UART1_SERIALDRIVER
#  ifdef CONFIG_UART1_RX_BUFFER_SIZE
#    define RA_UART1_RX_BUFFER_SIZE CONFIG_UART1_RX_BUFFER_SIZE
#  else
#    define RA_UART1_RX_BUFFER_SIZE CONFIG_RA_UART_RX_BUFFER_SIZE
#  endif
#  ifdef CONFIG_UART1_TX_BUFFER_SIZE
#    define RA_UART1_TX_BUFFER_SIZE CONFIG_UART1_TX_BUFFER_SIZE
#  else
#    define RA_UART1_TX_BUFFER_SIZE CONFIG_RA_UART_TX_BUFFER_SIZE
#  endif
#endif

#ifdef CONFIG_UART2_SERIALDRIVER
#  ifdef CONFIG_UART2_RX_BUFFER_SIZE
#    define RA_UART2_RX_BUFFER_SIZE CONFIG_UART2_RX_BUFFER_SIZE
#  else
#    define RA_UART2_RX_BUFFER_SIZE CONFIG_RA_UART_RX_BUFFER_SIZE
#  endif
#  ifdef CONFIG_UART2_TX_BUFFER_SIZE
#    define RA_UART2_TX_BUFFER_SIZE CONFIG_UART2_TX_BUFFER_SIZE
#  else
#    define RA_UART2_TX_BUFFER_SIZE CONFIG_RA_UART_TX_BUFFER_SIZE
#  endif
#endif

#ifdef CONFIG_UART3_SERIALDRIVER
#  ifdef CONFIG_UART3_RX_BUFFER_SIZE
#    define RA_UART3_RX_BUFFER_SIZE CONFIG_UART3_RX_BUFFER_SIZE
#  else
#    define RA_UART3_RX_BUFFER_SIZE CONFIG_RA_UART_RX_BUFFER_SIZE
#  endif
#  ifdef CONFIG_UART3_TX_BUFFER_SIZE
#    define RA_UART3_TX_BUFFER_SIZE CONFIG_UART3_TX_BUFFER_SIZE
#  else
#    define RA_UART3_TX_BUFFER_SIZE CONFIG_RA_UART_TX_BUFFER_SIZE
#  endif
#endif

/* DMA Configuration */
#define RA_UART_MAX_NORMAL_TRANSFER_LENGTH    (0xFFFF)
#define RA_UART_MAX_REPEAT_TRANSFER_LENGTH    (0x400)
#define RA_UART_MAX_BLOCK_TRANSFER_LENGTH     (0x400)
#define RA_UART_MAX_REPEAT_COUNT              (0x10000)
#define RA_UART_MAX_BLOCK_COUNT               (0x10000)

/* UART Events */
#define RA_UART_EVENT_TX_COMPLETE             (1U << 0)
#define RA_UART_EVENT_RX_COMPLETE             (1U << 1)
#define RA_UART_EVENT_RX_CHAR                 (1U << 2)
#define RA_UART_EVENT_TX_DATA_EMPTY           (1U << 3)
#define RA_UART_EVENT_ERR_PARITY              (1U << 4)
#define RA_UART_EVENT_ERR_FRAMING             (1U << 5)
#define RA_UART_EVENT_ERR_OVERFLOW            (1U << 6)
#define RA_UART_EVENT_BREAK_DETECT            (1U << 7)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* UART Driver State */
typedef enum
{
    RA_UART_STATE_UNINITIALIZED = 0,
    RA_UART_STATE_IDLE,
    RA_UART_STATE_TX_IN_PROGRESS,
    RA_UART_STATE_RX_IN_PROGRESS,
    RA_UART_STATE_ERROR
} ra_sci_state_t;

/* DMA Transfer Configuration */
typedef struct
{
    uint8_t  channel;           /* DMA channel */
    uint32_t irq;               /* DMA interrupt */
    uint8_t  ipl;               /* Interrupt priority level */
    bool     enabled;           /* DMA enabled for this direction */
} ra_sci_dma_config_t;

/* UART Configuration Structure */
typedef struct
{
    uint32_t base;              /* UART base address */
    uint32_t baud;              /* Configured baud rate */
    uint8_t  bits;              /* Number of data bits (5-9) */
    uint8_t  parity;            /* Parity selection */
    uint8_t  stop;              /* Number of stop bits */
    bool     flow_control;      /* Hardware flow control */
    
    /* IRQ Configuration */
    uint32_t rxi_irq;           /* RX interrupt */
    uint32_t txi_irq;           /* TX interrupt */
    uint32_t tei_irq;           /* TX end interrupt */
    uint32_t eri_irq;           /* Error interrupt */
    uint8_t  rxi_ipl;           /* RX interrupt priority */
    uint8_t  txi_ipl;           /* TX interrupt priority */
    uint8_t  tei_ipl;           /* TX end interrupt priority */
    uint8_t  eri_ipl;           /* Error interrupt priority */
    
    /* DMA Configuration */
    ra_sci_dma_config_t tx_dma;
    ra_sci_dma_config_t rx_dma;
    
    /* Pin Configuration */
    uint32_t tx_pin;            /* TX pin */
    uint32_t rx_pin;            /* RX pin */
    uint32_t rts_pin;           /* RTS pin (if flow control) */
    uint32_t cts_pin;           /* CTS pin (if flow control) */
} ra_sci_config_t;

/* UART Device Structure */
typedef struct
{
    const ra_sci_config_t *config;  /* UART configuration */
    ra_sci_state_t state;           /* Current state */
    
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
    void (*callback)(struct ra_sci_dev_s *dev, uint32_t event);
    void *callback_context;
} ra_sci_dev_t;

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
 * Name: ra_sci_initialize
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

int ra_sci_initialize(ra_sci_dev_t *dev);

/****************************************************************************
 * Name: ra_sci_finalize
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

void ra_sci_finalize(ra_sci_dev_t *dev);

/****************************************************************************
 * Name: ra_sci_send_dma
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

int ra_sci_send_dma(ra_sci_dev_t *dev, const uint8_t *buffer, 
                     uint16_t length);

/****************************************************************************
 * Name: ra_sci_receive_dma
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

int ra_sci_receive_dma(ra_sci_dev_t *dev, uint8_t *buffer, 
                        uint16_t length);

/****************************************************************************
 * Name: ra_sci_abort_transfer
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

int ra_sci_abort_transfer(ra_sci_dev_t *dev, bool tx);

/****************************************************************************
 * Name: ra_sci_set_callback
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

void ra_sci_set_callback(ra_sci_dev_t *dev, 
                          void (*callback)(ra_sci_dev_t *dev, uint32_t event),
                          void *context);

/****************************************************************************
 * Name: ra_sci_config_baudrate
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

int ra_sci_config_baudrate(ra_sci_dev_t *dev, uint32_t baud);

/****************************************************************************
 * Name: ra_sci_get_status
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

uint32_t ra_sci_get_status(ra_sci_dev_t *dev);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA8_RA_UART_H */
