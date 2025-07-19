/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra_uart_demo.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "ra8e1_uart.h"

/* Conditional RTT support */
#ifdef CONFIG_SEGGER_RTT
#include "SEGGER_RTT.h"
#define RTT_PRINTF(...)     SEGGER_RTT_printf(0, __VA_ARGS__)
#define RTT_HASKEY()        SEGGER_RTT_HasKey()
#define RTT_GETKEY()        SEGGER_RTT_GetKey()
#else
#define RTT_PRINTF(...)     printf(__VA_ARGS__)
#define RTT_HASKEY()        (false)
#define RTT_GETKEY()        (0)
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Additional configuration that's not in header */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Type aliases for compatibility */
typedef struct dma_config_s dma_config_t;
typedef struct uart_config_s uart_config_t;
typedef struct uart_dev_s uart_dev_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART0 Configuration for RA8E1 FPB Board */
static const uart_config_t g_uart0_config =
{
    .base = R_SCI0_BASE,
    .baud = DEMO_UART_BAUD_RATE,
    .data_bits = 8,
    .parity = 0,                    /* No parity */
    .stop_bits = 1,
    .flow_control = false,
    
    /* IRQ Configuration */
    .rxi_irq = IRQ_SCI0_RXI,
    .txi_irq = IRQ_SCI0_TXI,
    .tei_irq = IRQ_SCI0_TEI,
    .eri_irq = IRQ_SCI0_ERI,
    
    /* DMA Configuration */
    .tx_dma = 
    {
        .channel = 0,
        .irq = IRQ_DMAC0_INT,
        .priority = 3,
        .enabled = true,
    },
    .rx_dma = 
    {
        .channel = 1,
        .irq = IRQ_DMAC1_INT,
        .priority = 3,
        .enabled = true,
    },
    
    /* Pin Configuration */
    .tx_pin = GPIO_SCI0_TXD,
    .rx_pin = GPIO_SCI0_RXD,
};

/* UART1 Configuration for J-Link VCOM */
static const uart_config_t g_uart1_config =
{
    .base = R_SCI1_BASE,
    .baud = DEMO_UART_BAUD_RATE,
    .data_bits = 8,
    .parity = 0,                    /* No parity */
    .stop_bits = 1,
    .flow_control = false,
    
    /* IRQ Configuration */
    .rxi_irq = IRQ_SCI1_RXI,
    .txi_irq = IRQ_SCI1_TXI,
    .tei_irq = IRQ_SCI1_TEI,
    .eri_irq = IRQ_SCI1_ERI,
    
    /* DMA Configuration */
    .tx_dma = 
    {
        .channel = 2,
        .irq = IRQ_DMAC2_INT,
        .priority = 3,
        .enabled = true,
    },
    .rx_dma = 
    {
        .channel = 0,               /* Reuse channel 0 for RX */
        .irq = IRQ_DMAC0_INT,
        .priority = 3,
        .enabled = false,           /* Disable RX DMA for demo */
    },
    
    /* Pin Configuration */
    .tx_pin = GPIO_SCI1_TXD,        /* J-Link VCOM TX */
    .rx_pin = GPIO_SCI1_RXD,        /* J-Link VCOM RX */
};

/* Device instances */
static uart_dev_t g_uart0_device;
static uart_dev_t g_uart1_device;

/* Test buffers */
static uint8_t g_tx_test_buffer[DEMO_BUFFER_SIZE];
static uint8_t g_rx_test_buffer[DEMO_BUFFER_SIZE];

/* Status flags */
static volatile bool g_tx_completed = false;
static volatile bool g_rx_completed = false;
static volatile uart_event_t g_last_event = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Function declarations */
static void demo_uart_event_callback(uart_dev_t *dev, uart_event_t event);
static int demo_uart_simple_init(uart_dev_t *dev, const uart_config_t *config);
static int demo_uart_send_dma(uart_dev_t *dev, const uint8_t *buffer, uint16_t length);
static int demo_uart_receive_dma(uart_dev_t *dev, uint8_t *buffer, uint16_t length);
static void demo_uart_simulate_completion(uart_dev_t *dev, bool is_tx);

/* Public function declarations */
int ra8e1_uart_main(void);
void ra8e1_uart_show_config(void);

/****************************************************************************
 * Name: demo_uart_event_callback
 *
 * Description:
 *   UART event callback function
 *
 ****************************************************************************/

static void demo_uart_event_callback(uart_dev_t *dev, uart_event_t event)
{
    g_last_event = event;
    
    switch (event)
    {
        case UART_EVENT_TX_COMPLETE:
            g_tx_completed = true;
            RTT_PRINTF("UART TX DMA completed\n");
            break;
            
        case UART_EVENT_RX_COMPLETE:
            g_rx_completed = true;
            RTT_PRINTF("UART RX DMA completed\n");
            break;
            
        case UART_EVENT_ERR_PARITY:
            RTT_PRINTF("UART parity error detected\n");
            break;
            
        case UART_EVENT_ERR_FRAMING:
            RTT_PRINTF("UART framing error detected\n");
            break;
            
        case UART_EVENT_ERR_OVERFLOW:
            RTT_PRINTF("UART overflow error detected\n");
            break;
            
        default:
            RTT_PRINTF("UART event: 0x%08X\n", (unsigned int)event);
            break;
    }
}

/****************************************************************************
 * Name: demo_uart_simple_init
 *
 * Description:
 *   Simple UART initialization (mock implementation)
 *
 ****************************************************************************/

static int demo_uart_simple_init(uart_dev_t *dev, const uart_config_t *config)
{
    if (dev == NULL || config == NULL)
    {
        return -1;  /* EINVAL */
    }
    
    /* Initialize device structure */
    dev->config = config;
    dev->state = UART_STATE_IDLE;
    dev->tx_dma_ctrl = NULL;
    dev->rx_dma_ctrl = NULL;
    dev->tx_buffer = g_tx_test_buffer;
    dev->rx_buffer = g_rx_test_buffer;
    dev->tx_buffer_size = DEMO_BUFFER_SIZE;
    dev->rx_buffer_size = DEMO_BUFFER_SIZE;
    dev->tx_count = 0;
    dev->rx_count = 0;
    dev->events = 0;
    dev->callback = demo_uart_event_callback;
    dev->callback_context = NULL;
    
    RTT_PRINTF("UART initialized: Base=0x%08lX, Baud=%lu\n", 
           (unsigned long)config->base, (unsigned long)config->baud);
    RTT_PRINTF("  TX DMA: Channel=%d, IRQ=%lu, Enabled=%s\n",
           config->tx_dma.channel, (unsigned long)config->tx_dma.irq,
           config->tx_dma.enabled ? "Yes" : "No");
    RTT_PRINTF("  RX DMA: Channel=%d, IRQ=%lu, Enabled=%s\n",
           config->rx_dma.channel, (unsigned long)config->rx_dma.irq,
           config->rx_dma.enabled ? "Yes" : "No");
    
    return 0;  /* OK */
}

/****************************************************************************
 * Name: demo_uart_send_dma
 *
 * Description:
 *   Mock DMA send function
 *
 ****************************************************************************/

static int demo_uart_send_dma(uart_dev_t *dev, const uint8_t *buffer, uint16_t length)
{
    if (dev == NULL || buffer == NULL || length == 0)
    {
        return -1;  /* EINVAL */
    }
    
    if (dev->state != UART_STATE_IDLE)
    {
        return -2;  /* EBUSY */
    }
    
    if (!dev->config->tx_dma.enabled)
    {
        return -3;  /* ENOTSUP */
    }
    
    RTT_PRINTF("Starting TX DMA transfer: %d bytes\n", length);
    RTT_PRINTF("TX Data: \"");
    for (int i = 0; i < length && i < 32; i++)
    {
        if (buffer[i] >= 32 && buffer[i] < 127)
        {
            RTT_PRINTF("%c", buffer[i]);
        }
        else
        {
            RTT_PRINTF("\\x%02X", buffer[i]);
        }
    }
    if (length > 32) RTT_PRINTF("...");
    RTT_PRINTF("\"\n");
    
    /* Update state */
    dev->state = UART_STATE_TX_IN_PROGRESS;
    dev->tx_count = length;
    
    /* Simulate DMA completion (in real implementation, this would be interrupt-driven) */
    g_tx_completed = false;
    
    /* Simulate completion after a short delay */
    RTT_PRINTF("TX DMA transfer simulated - would normally complete via interrupt\n");
    
    return 0;  /* OK */
}

/****************************************************************************
 * Name: demo_uart_receive_dma
 *
 * Description:
 *   Mock DMA receive function
 *
 ****************************************************************************/

static int demo_uart_receive_dma(uart_dev_t *dev, uint8_t *buffer, uint16_t length)
{
    if (dev == NULL || buffer == NULL || length == 0)
    {
        return -1;  /* EINVAL */
    }
    
    if (dev->state != UART_STATE_IDLE)
    {
        return -2;  /* EBUSY */
    }
    
    if (!dev->config->rx_dma.enabled)
    {
        return -3;  /* ENOTSUP */
    }
    
    RTT_PRINTF("Starting RX DMA transfer: %d bytes\n", length);
    
    /* Update state */
    dev->state = UART_STATE_RX_IN_PROGRESS;
    dev->rx_count = length;
    
    /* Simulate DMA setup */
    g_rx_completed = false;
    
    RTT_PRINTF("RX DMA transfer setup - waiting for data\n");
    
    return 0;  /* OK */
}

/****************************************************************************
 * Name: demo_uart_simulate_completion
 *
 * Description:
 *   Simulate DMA completion (for demo purposes)
 *
 ****************************************************************************/

static void demo_uart_simulate_completion(uart_dev_t *dev, bool is_tx)
{
    if (dev == NULL) return;
    
    if (is_tx && dev->state == UART_STATE_TX_IN_PROGRESS)
    {
        dev->state = UART_STATE_IDLE;
        dev->tx_count = 0;
        if (dev->callback)
        {
            dev->callback(dev, UART_EVENT_TX_COMPLETE);
        }
    }
    else if (!is_tx && dev->state == UART_STATE_RX_IN_PROGRESS)
    {
        dev->state = UART_STATE_IDLE;
        dev->rx_count = 0;
        
        /* Simulate received data */
        strcpy((char *)dev->rx_buffer, "Simulated RX data");
        
        if (dev->callback)
        {
            dev->callback(dev, UART_EVENT_RX_COMPLETE);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_uart_main
 *
 * Description:
 *   Main demo function
 *
 ****************************************************************************/

int ra8e1_uart_main(void)
{
    int ret;
    
    RTT_PRINTF("\n=== RA8E1 UART with DMA Demo ===\n\n");
    
    /* Initialize UART0 */
    RTT_PRINTF("1. Initializing UART0...\n");
    ret = demo_uart_simple_init(&g_uart0_device, &g_uart0_config);
    if (ret < 0)
    {
        RTT_PRINTF("Failed to initialize UART0: %d\n", ret);
        return ret;
    }
    
    /* Initialize UART1 */
    RTT_PRINTF("\n2. Initializing UART1 (J-Link VCOM)...\n");
    ret = demo_uart_simple_init(&g_uart1_device, &g_uart1_config);
    if (ret < 0)
    {
        RTT_PRINTF("Failed to initialize UART1: %d\n", ret);
        return ret;
    }
    
    /* Test TX DMA on UART0 */
    RTT_PRINTF("\n3. Testing TX DMA on UART0...\n");
    strcpy((char *)g_tx_test_buffer, DEMO_TEST_MESSAGE);
    ret = demo_uart_send_dma(&g_uart0_device, g_tx_test_buffer, strlen(DEMO_TEST_MESSAGE));
    if (ret < 0)
    {
        RTT_PRINTF("Failed to start TX DMA: %d\n", ret);
    }
    else
    {
        /* Simulate completion */
        demo_uart_simulate_completion(&g_uart0_device, true);
    }
    
    /* Test TX DMA on UART1 */
    RTT_PRINTF("\n4. Testing TX DMA on UART1...\n");
    strcpy((char *)g_tx_test_buffer, "Hello from UART1 via DMA!\r\n");
    ret = demo_uart_send_dma(&g_uart1_device, g_tx_test_buffer, strlen((char *)g_tx_test_buffer));
    if (ret < 0)
    {
        RTT_PRINTF("Failed to start TX DMA: %d\n", ret);
    }
    else
    {
        /* Simulate completion */
        demo_uart_simulate_completion(&g_uart1_device, true);
    }
    
    /* Test RX DMA on UART0 */
    RTT_PRINTF("\n5. Testing RX DMA on UART0...\n");
    ret = demo_uart_receive_dma(&g_uart0_device, g_rx_test_buffer, 32);
    if (ret < 0)
    {
        RTT_PRINTF("Failed to start RX DMA: %d\n", ret);
    }
    else
    {
        /* Simulate completion */
        demo_uart_simulate_completion(&g_uart0_device, false);
        RTT_PRINTF("Received data: \"%s\"\n", g_rx_test_buffer);
    }
    
    /* Display final status */
    RTT_PRINTF("\n6. Final Status:\n");
    RTT_PRINTF("  UART0 State: %d\n", g_uart0_device.state);
    RTT_PRINTF("  UART1 State: %d\n", g_uart1_device.state);
    RTT_PRINTF("  Last Event: 0x%08X\n", (unsigned int)g_last_event);
    
    RTT_PRINTF("\n=== Demo Completed Successfully ===\n");
    RTT_PRINTF("\nIn a real implementation:\n");
    RTT_PRINTF("- GPIO pins would be configured for UART functionality\n");
    RTT_PRINTF("- UART registers would be programmed with baud rate and format\n");
    RTT_PRINTF("- DMA channels would be allocated and configured\n");
    RTT_PRINTF("- Interrupt handlers would be installed\n");
    RTT_PRINTF("- Data transfer would happen asynchronously via DMA\n");
    RTT_PRINTF("- Callbacks would be invoked upon completion or errors\n");
    
    RTT_PRINTF("\n=== RTT Interactive Commands ===\n");
    RTT_PRINTF("Available commands (enter via RTT):\n");
    RTT_PRINTF("  'c' - Show configuration\n");
    RTT_PRINTF("  's' - Show status\n");
    RTT_PRINTF("  't' - Test TX DMA\n");
    RTT_PRINTF("  'r' - Test RX DMA\n");
    RTT_PRINTF("  'q' - Quit demo\n");
    RTT_PRINTF("\nEnter command: ");
    
    /* Simple command loop for RTT interaction */
    char cmd;
    bool running = true;
    while (running && RTT_HASKEY())
    {
        cmd = RTT_GETKEY();
        RTT_PRINTF("\nCommand: %c\n", cmd);
        
        switch (cmd)
        {
            case 'c':
            case 'C':
                ra8e1_uart_show_config();
                break;
                
            case 's':
            case 'S':
                RTT_PRINTF("UART Status:\n");
                RTT_PRINTF("  UART0 State: %d\n", g_uart0_device.state);
                RTT_PRINTF("  UART1 State: %d\n", g_uart1_device.state);
                RTT_PRINTF("  TX Completed: %s\n", g_tx_completed ? "Yes" : "No");
                RTT_PRINTF("  RX Completed: %s\n", g_rx_completed ? "Yes" : "No");
                RTT_PRINTF("  Last Event: 0x%08X\n", (unsigned int)g_last_event);
                break;
                
            case 't':
            case 'T':
                RTT_PRINTF("Testing TX DMA on UART0...\n");
                strcpy((char *)g_tx_test_buffer, "RTT Test Message\r\n");
                ret = demo_uart_send_dma(&g_uart0_device, g_tx_test_buffer, strlen((char *)g_tx_test_buffer));
                if (ret == 0)
                {
                    demo_uart_simulate_completion(&g_uart0_device, true);
                }
                break;
                
            case 'r':
            case 'R':
                RTT_PRINTF("Testing RX DMA on UART0...\n");
                ret = demo_uart_receive_dma(&g_uart0_device, g_rx_test_buffer, 16);
                if (ret == 0)
                {
                    demo_uart_simulate_completion(&g_uart0_device, false);
                }
                break;
                
            case 'q':
            case 'Q':
                RTT_PRINTF("Exiting demo...\n");
                running = false;
                break;
                
            default:
                RTT_PRINTF("Unknown command. Available: c, s, t, r, q\n");
                break;
        }
        
        if (running)
        {
            RTT_PRINTF("\nEnter command: ");
        }
    }
    
    return 0;
}

/****************************************************************************
 * Name: ra8e1_uart_show_config
 *
 * Description:
 *   Display configuration details
 *
 ****************************************************************************/

void ra8e1_uart_show_config(void)
{
    RTT_PRINTF("\n=== UART Configuration Details ===\n");
    
    RTT_PRINTF("\nUART0 Configuration:\n");
    RTT_PRINTF("  Base Address: 0x%08lX\n", (unsigned long)g_uart0_config.base);
    RTT_PRINTF("  Baud Rate: %lu\n", (unsigned long)g_uart0_config.baud);
    RTT_PRINTF("  Data Bits: %d\n", g_uart0_config.data_bits);
    RTT_PRINTF("  Parity: %d (0=None, 1=Odd, 2=Even)\n", g_uart0_config.parity);
    RTT_PRINTF("  Stop Bits: %d\n", g_uart0_config.stop_bits);
    RTT_PRINTF("  Flow Control: %s\n", g_uart0_config.flow_control ? "Enabled" : "Disabled");
    RTT_PRINTF("  TX Pin: 0x%04lX\n", (unsigned long)g_uart0_config.tx_pin);
    RTT_PRINTF("  RX Pin: 0x%04lX\n", (unsigned long)g_uart0_config.rx_pin);
    
    RTT_PRINTF("\nUART1 Configuration:\n");
    RTT_PRINTF("  Base Address: 0x%08lX\n", (unsigned long)g_uart1_config.base);
    RTT_PRINTF("  Baud Rate: %lu\n", (unsigned long)g_uart1_config.baud);
    RTT_PRINTF("  Data Bits: %d\n", g_uart1_config.data_bits);
    RTT_PRINTF("  Parity: %d (0=None, 1=Odd, 2=Even)\n", g_uart1_config.parity);
    RTT_PRINTF("  Stop Bits: %d\n", g_uart1_config.stop_bits);
    RTT_PRINTF("  Flow Control: %s\n", g_uart1_config.flow_control ? "Enabled" : "Disabled");
    RTT_PRINTF("  TX Pin: 0x%04lX\n", (unsigned long)g_uart1_config.tx_pin);
    RTT_PRINTF("  RX Pin: 0x%04lX\n", (unsigned long)g_uart1_config.rx_pin);
    
    RTT_PRINTF("\n=== DMA Configuration ===\n");
    RTT_PRINTF("UART0 TX DMA: Channel %d, IRQ %lu, Priority %d\n",
           g_uart0_config.tx_dma.channel, 
           (unsigned long)g_uart0_config.tx_dma.irq,
           g_uart0_config.tx_dma.priority);
    RTT_PRINTF("UART0 RX DMA: Channel %d, IRQ %lu, Priority %d\n",
           g_uart0_config.rx_dma.channel,
           (unsigned long)g_uart0_config.rx_dma.irq,
           g_uart0_config.rx_dma.priority);
    RTT_PRINTF("UART1 TX DMA: Channel %d, IRQ %lu, Priority %d\n",
           g_uart1_config.tx_dma.channel,
           (unsigned long)g_uart1_config.tx_dma.irq,
           g_uart1_config.tx_dma.priority);
}

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Main entry point for the RA8E1 UART DMA demonstration
 *
 ****************************************************************************/

int main(int argc, char *argv[])
{
    int ret;
    
    RTT_PRINTF("RA8E1 UART with DMA Driver Demo\n");
    RTT_PRINTF("================================\n\n");
    
    if (argc > 1 && strcmp(argv[1], "config") == 0)
    {
        /* Show configuration details */
        ra8e1_uart_show_config();
        return 0;
    }
    
    /* Run the main demo */
    ret = ra8e1_uart_main();
    
    if (ret == 0)
    {
        RTT_PRINTF("\nDemo completed successfully!\n");
        RTT_PRINTF("\nTo see configuration details, run: %s config\n", argv[0]);
    }
    else
    {
        RTT_PRINTF("\nDemo failed with error code: %d\n", ret);
    }
    
    return ret;
}

#ifdef STANDALONE_DEMO
/****************************************************************************
 * Name: Alternative main for standalone compilation
 ****************************************************************************/

#include <string.h>

int standalone_main(void)
{
    RTT_PRINTF("RA8E1 UART with DMA - Standalone Demo\n");
    RTT_PRINTF("=====================================\n\n");
    
    /* Show configuration */
    ra8e1_uart_show_config();
    
    /* Run demo */
    return ra8e1_uart_main();
}
#endif
