/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_uart_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_UART_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_UART_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART Configuration Parameters */

#define DEMO_UART_BAUD_RATE         115200
#define DEMO_BUFFER_SIZE            256
#define DEMO_TEST_MESSAGE           "Hello from RA8E1 UART with DMA!\r\n"

/* UART Hardware Base Addresses */

#define R_SCI0_BASE                 0x40000000UL
#define R_SCI1_BASE                 0x40000100UL
#define R_SCI2_BASE                 0x40000200UL
#define R_SCI3_BASE                 0x40000300UL
#define R_SCI4_BASE                 0x40000400UL
#define R_SCI9_BASE                 0x40000900UL

/* DMA Base Addresses */

#define R_DMAC0_BASE                0x40001000UL
#define R_DMAC1_BASE                0x40001800UL
#define R_DMAC2_BASE                0x40002000UL

/* IRQ Numbers */

#define IRQ_SCI0_RXI                32
#define IRQ_SCI0_TXI                33
#define IRQ_SCI0_TEI                34
#define IRQ_SCI0_ERI                35
#define IRQ_SCI1_RXI                36
#define IRQ_SCI1_TXI                37
#define IRQ_SCI1_TEI                38
#define IRQ_SCI1_ERI                39
#define IRQ_DMAC0_INT               40
#define IRQ_DMAC1_INT               41
#define IRQ_DMAC2_INT               42

/* GPIO Pin Definitions (RA8E1 FPB specific) */

#define GPIO_SCI0_TXD               0x0310      /* P3_10 */
#define GPIO_SCI0_RXD               0x0309      /* P3_09 */
#define GPIO_SCI1_TXD               0x0102      /* P1_02 (J-Link VCOM) */
#define GPIO_SCI1_RXD               0x0101      /* P1_01 (J-Link VCOM) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* UART Event Types */

typedef enum
{
    UART_EVENT_TX_COMPLETE      = (1U << 0),
    UART_EVENT_RX_COMPLETE      = (1U << 1),
    UART_EVENT_RX_CHAR          = (1U << 2),
    UART_EVENT_TX_DATA_EMPTY    = (1U << 3),
    UART_EVENT_ERR_PARITY       = (1U << 4),
    UART_EVENT_ERR_FRAMING      = (1U << 5),
    UART_EVENT_ERR_OVERFLOW     = (1U << 6),
    UART_EVENT_BREAK_DETECT     = (1U << 7)
} uart_event_t;

/* UART State */

typedef enum
{
    UART_STATE_UNINITIALIZED = 0,
    UART_STATE_IDLE,
    UART_STATE_TX_IN_PROGRESS,
    UART_STATE_RX_IN_PROGRESS,
    UART_STATE_ERROR
} uart_state_t;

/* DMA Configuration */

struct dma_config_s
{
    uint8_t  channel;           /* DMA channel */
    uint32_t irq;               /* DMA interrupt */
    uint8_t  priority;          /* Interrupt priority */
    bool     enabled;           /* DMA enabled */
};

/* UART Configuration */

struct uart_config_s
{
    uint32_t base;              /* UART base address */
    uint32_t baud;              /* Baud rate */
    uint8_t  data_bits;         /* Data bits (7-9) */
    uint8_t  parity;            /* Parity (0=none, 1=odd, 2=even) */
    uint8_t  stop_bits;         /* Stop bits (1-2) */
    bool     flow_control;      /* Hardware flow control */
    
    /* IRQ Configuration */
    uint32_t rxi_irq;           /* RX interrupt */
    uint32_t txi_irq;           /* TX interrupt */
    uint32_t tei_irq;           /* TX end interrupt */
    uint32_t eri_irq;           /* Error interrupt */
    
    /* DMA Configuration */
    struct dma_config_s tx_dma;
    struct dma_config_s rx_dma;
    
    /* Pin Configuration */
    uint32_t tx_pin;            /* TX pin */
    uint32_t rx_pin;            /* RX pin */
};

/* Forward declaration */
struct uart_dev_s;

/* UART Device */

struct uart_dev_s
{
    const struct uart_config_s *config;    /* Configuration */
    uart_state_t state;                    /* Current state */
    
    /* DMA Control */
    void *tx_dma_ctrl;                     /* TX DMA handle */
    void *rx_dma_ctrl;                     /* RX DMA handle */
    
    /* Buffer Management */
    uint8_t *tx_buffer;                    /* TX buffer */
    uint8_t *rx_buffer;                    /* RX buffer */
    uint16_t tx_buffer_size;               /* TX buffer size */
    uint16_t rx_buffer_size;               /* RX buffer size */
    
    /* Transfer Status */
    volatile uint16_t tx_count;            /* TX bytes remaining */
    volatile uint16_t rx_count;            /* RX bytes remaining */
    volatile uint32_t events;              /* Event flags */
    
    /* Callback */
    void (*callback)(struct uart_dev_s *dev, uart_event_t event);
    void *callback_context;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

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
 * Name: ra8e1_uart_demo_main
 *
 * Description:
 *   UART with DMA demo main function
 *
 ****************************************************************************/

int ra8e1_uart_demo_main(void);

/****************************************************************************
 * Name: ra8e1_uart_demo_show_config
 *
 * Description:
 *   Display UART configuration details
 *
 ****************************************************************************/

void ra8e1_uart_demo_show_config(void);

/****************************************************************************
 * Name: ra8e1_uart_demo_init
 *
 * Description:
 *   Initialize UART device with configuration
 *
 ****************************************************************************/

int ra8e1_uart_demo_init(struct uart_dev_s *dev, 
                         const struct uart_config_s *config);

/****************************************************************************
 * Name: ra8e1_uart_demo_send_dma
 *
 * Description:
 *   Send data using DMA
 *
 ****************************************************************************/

int ra8e1_uart_demo_send_dma(struct uart_dev_s *dev, 
                              const uint8_t *buffer, 
                              uint16_t length);

/****************************************************************************
 * Name: ra8e1_uart_demo_receive_dma
 *
 * Description:
 *   Receive data using DMA
 *
 ****************************************************************************/

int ra8e1_uart_demo_receive_dma(struct uart_dev_s *dev, 
                                 uint8_t *buffer, 
                                 uint16_t length);

/****************************************************************************
 * Name: ra8e1_uart_demo_get_state
 *
 * Description:
 *   Get current UART state
 *
 ****************************************************************************/

uart_state_t ra8e1_uart_demo_get_state(const struct uart_dev_s *dev);

/****************************************************************************
 * Name: ra8e1_uart_demo_set_callback
 *
 * Description:
 *   Set event callback function
 *
 ****************************************************************************/

void ra8e1_uart_demo_set_callback(struct uart_dev_s *dev,
                                   void (*callback)(struct uart_dev_s *dev, 
                                                    uart_event_t event),
                                   void *context);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_UART_DEMO_H */
