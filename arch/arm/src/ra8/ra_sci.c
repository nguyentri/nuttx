/****************************************************************************
 * arch/arm/src/ra8/ra_sci.c
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
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ra_sci.h"
#include "hardware/ra_sci.h"
#include "hardware/ra_memorymap.h"
#include "ra_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART Register Bit Definitions */

/* Serial Mode Register (SMR) */
#define RA_SCI_SMR_CKS_SHIFT      (0)        /* Clock Select */
#define RA_SCI_SMR_CKS_MASK       (3 << RA_SCI_SMR_CKS_SHIFT)
#define RA_SCI_SMR_MP             (1 << 2)   /* Multi-processor mode */
#define RA_SCI_SMR_STOP           (1 << 3)   /* Stop bit length */
#define RA_SCI_SMR_PM             (1 << 4)   /* Parity mode */
#define RA_SCI_SMR_PE             (1 << 5)   /* Parity enable */
#define RA_SCI_SMR_CHR            (1 << 6)   /* Character length */
#define RA_SCI_SMR_CM             (1 << 7)   /* Communication mode */

/* Serial Control Register (SCR) */
#define RA_SCI_SCR_CKE_SHIFT      (0)        /* Clock Enable */
#define RA_SCI_SCR_CKE_MASK       (3 << RA_SCI_SCR_CKE_SHIFT)
#define RA_SCI_SCR_TEIE           (1 << 2)   /* Transmit End Interrupt Enable */
#define RA_SCI_SCR_MPIE           (1 << 3)   /* Multi-processor Interrupt Enable */
#define RA_SCI_SCR_RE             (1 << 4)   /* Receive Enable */
#define RA_SCI_SCR_TE             (1 << 5)   /* Transmit Enable */
#define RA_SCI_SCR_RIE            (1 << 6)   /* Receive Interrupt Enable */
#define RA_SCI_SCR_TIE            (1 << 7)   /* Transmit Interrupt Enable */

/* Serial Status Register (SSR) */
#define RA_SCI_SSR_MPBT           (1 << 0)   /* Multi-processor Bit Transfer */
#define RA_SCI_SSR_MPB            (1 << 1)   /* Multi-processor Bit */
#define RA_SCI_SSR_TEND           (1 << 2)   /* Transmit End Flag */
#define RA_SCI_SSR_PER            (1 << 3)   /* Parity Error Flag */
#define RA_SCI_SSR_FER            (1 << 4)   /* Framing Error Flag */
#define RA_SCI_SSR_ORER           (1 << 5)   /* Overrun Error Flag */
#define RA_SCI_SSR_RDRF           (1 << 6)   /* Receive Data Ready Flag */
#define RA_SCI_SSR_TDRE           (1 << 7)   /* Transmit Data Empty Flag */

/* Serial Extended Mode Register (SEMR) */
#define RA_SCI_SEMR_ACS0          (1 << 0)   /* Asynchronous Mode Clock Source Select */
#define RA_SCI_SEMR_ABCS          (1 << 4)   /* Asynchronous Mode Base Clock Select */
#define RA_SCI_SEMR_NFEN          (1 << 5)   /* Digital Noise Filter Function Enable */
#define RA_SCI_SEMR_BGDM          (1 << 6)   /* Baud Rate Generator Double-Speed Mode Select */
#define RA_SCI_SEMR_RXDESEL       (1 << 7)   /* Asynchronous Start Bit Edge Detection Select */

/* FIFO Control Register (FCR) */
#define RA_SCI_FCR_RFRST          (1 << 1)   /* Receive FIFO Data Register Reset */
#define RA_SCI_FCR_TFRST          (1 << 2)   /* Transmit FIFO Data Register Reset */
#define RA_SCI_FCR_DRES           (1 << 3)   /* Receive Data Ready Error Select */
#define RA_SCI_FCR_TTRG_SHIFT     (4)        /* Transmit FIFO Data Trigger Number */
#define RA_SCI_FCR_TTRG_MASK      (15 << RA_SCI_FCR_TTRG_SHIFT)
#define RA_SCI_FCR_RTRG_SHIFT     (8)        /* Receive FIFO Data Trigger Number */
#define RA_SCI_FCR_RTRG_MASK      (15 << RA_SCI_FCR_RTRG_SHIFT)
#define RA_SCI_FCR_RSTRG_SHIFT    (12)       /* RTS Output Active Trigger Select */
#define RA_SCI_FCR_RSTRG_MASK     (15 << RA_SCI_FCR_RSTRG_SHIFT)

/* DMA Register Base Addresses */
#define RA_DMAC0_BASE             0x40001000
#define RA_DMAC1_BASE             0x40001800
#define RA_DMAC2_BASE             0x40002000

/* DMA Register Offsets */
#define RA_DMAC_DMSAR_OFFSET      0x00       /* DMA Source Address Register */
#define RA_DMAC_DMDAR_OFFSET      0x04       /* DMA Destination Address Register */
#define RA_DMAC_DMCRA_OFFSET      0x08       /* DMA Transfer Count Register A */
#define RA_DMAC_DMCRB_OFFSET      0x0C       /* DMA Transfer Count Register B */
#define RA_DMAC_DMTMD_OFFSET      0x10       /* DMA Transfer Mode Register */
#define RA_DMAC_DMINT_OFFSET      0x14       /* DMA Interrupt Setting Register */
#define RA_DMAC_DMAMD_OFFSET      0x18       /* DMA Address Mode Register */
#define RA_DMAC_DMOFR_OFFSET      0x1C       /* DMA Offset Register */
#define RA_DMAC_DMCNT_OFFSET      0x20       /* DMA Control Register */
#define RA_DMAC_DMREQ_OFFSET      0x24       /* DMA Software Start Register */
#define RA_DMAC_DMSTS_OFFSET      0x28       /* DMA Status Register */
#define RA_DMAC_DMACT_OFFSET      0x2C       /* DMA Activation Source Register */

/* DMA Control Register Bits */
#define RA_DMAC_DMCNT_DTE         (1 << 0)   /* DMA Transfer Enable */
#define RA_DMAC_DMCNT_RLD         (1 << 1)   /* DMA Repeat Load */
#define RA_DMAC_DMCNT_TM          (1 << 2)   /* DMA Transfer Mode */
#define RA_DMAC_DMCNT_SZ_SHIFT    (3)        /* DMA Data Size */
#define RA_DMAC_DMCNT_SZ_MASK     (3 << RA_DMAC_DMCNT_SZ_SHIFT)

/* DMA Transfer Mode Register Bits */
#define RA_DMAC_DMTMD_DCTG_SHIFT  (0)        /* DMA Transfer Request Source Select */
#define RA_DMAC_DMTMD_DCTG_MASK   (3 << RA_DMAC_DMTMD_DCTG_SHIFT)
#define RA_DMAC_DMTMD_SZ_SHIFT    (8)        /* DMA Data Size */
#define RA_DMAC_DMTMD_SZ_MASK     (3 << RA_DMAC_DMTMD_SZ_SHIFT)
#define RA_DMAC_DMTMD_DTS_SHIFT   (12)       /* DMA Transfer Source */
#define RA_DMAC_DMTMD_DTS_MASK    (3 << RA_DMAC_DMTMD_DTS_SHIFT)
#define RA_DMAC_DMTMD_MD_SHIFT    (14)       /* DMA Transfer Mode */
#define RA_DMAC_DMTMD_MD_MASK     (3 << RA_DMAC_DMTMD_MD_SHIFT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA Control Block */
typedef struct
{
    uint32_t base;              /* DMA channel base address */
    uint8_t  channel;           /* DMA channel number */
    uint32_t irq;               /* DMA interrupt number */
    bool     in_use;            /* Channel in use flag */
} ra_dma_channel_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA channel pool */
static ra_dma_channel_t g_dma_channels[] =
{
    { RA_DMAC0_BASE, 0, RA_IRQ_IELSR0, false },
    { RA_DMAC1_BASE, 1, RA_IRQ_IELSR1, false },
    { RA_DMAC2_BASE, 2, RA_IRQ_IELSR2, false },
};

#define RA_NUM_DMA_CHANNELS (sizeof(g_dma_channels) / sizeof(g_dma_channels[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_sci_dma_allocate_channel
 *
 * Description:
 *   Allocate a DMA channel
 *
 ****************************************************************************/

static ra_dma_channel_t *ra_sci_dma_allocate_channel(void)
{
    irqstate_t flags;
    int i;

    flags = enter_critical_section();

    for (i = 0; i < RA_NUM_DMA_CHANNELS; i++)
    {
        if (!g_dma_channels[i].in_use)
        {
            g_dma_channels[i].in_use = true;
            leave_critical_section(flags);
            return &g_dma_channels[i];
        }
    }

    leave_critical_section(flags);
    return NULL;
}

/****************************************************************************
 * Name: ra_sci_dma_free_channel
 *
 * Description:
 *   Free a DMA channel
 *
 ****************************************************************************/

static void ra_sci_dma_free_channel(ra_dma_channel_t *channel)
{
    irqstate_t flags;

    if (channel != NULL)
    {
        flags = enter_critical_section();
        channel->in_use = false;
        leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: ra_sci_dma_setup_transfer
 *
 * Description:
 *   Setup DMA transfer
 *
 ****************************************************************************/

static int ra_sci_dma_setup_transfer(ra_dma_channel_t *dma,
                                       uint32_t src_addr, uint32_t dst_addr,
                                       uint16_t count, bool is_tx)
{
    uint32_t tmdr = 0;
    uint32_t amd = 0;

    if (dma == NULL)
    {
        return -EINVAL;
    }

    /* Disable DMA channel */
    putreg32(0, dma->base + RA_DMAC_DMCNT_OFFSET);

    /* Set source and destination addresses */
    putreg32(src_addr, dma->base + RA_DMAC_DMSAR_OFFSET);
    putreg32(dst_addr, dma->base + RA_DMAC_DMDAR_OFFSET);

    /* Set transfer count */
    putreg32(count, dma->base + RA_DMAC_DMCRA_OFFSET);

    /* Configure transfer mode */
    tmdr = (0 << RA_DMAC_DMTMD_DCTG_SHIFT) |    /* Software trigger */
           (0 << RA_DMAC_DMTMD_SZ_SHIFT) |      /* Byte transfer */
           (0 << RA_DMAC_DMTMD_DTS_SHIFT) |     /* Read-after-write */
           (0 << RA_DMAC_DMTMD_MD_SHIFT);       /* Normal mode */

    putreg32(tmdr, dma->base + RA_DMAC_DMTMD_OFFSET);

    /* Configure address mode */
    if (is_tx)
    {
        /* TX: Source incremented, destination fixed */
        amd = (1 << 14) | (0 << 12);
    }
    else
    {
        /* RX: Source fixed, destination incremented */
        amd = (0 << 14) | (1 << 12);
    }

    putreg32(amd, dma->base + RA_DMAC_DMAMD_OFFSET);

    /* Enable transfer end interrupt */
    putreg32(1, dma->base + RA_DMAC_DMINT_OFFSET);

    return OK;
}

/****************************************************************************
 * Name: ra_sci_dma_start_transfer
 *
 * Description:
 *   Start DMA transfer
 *
 ****************************************************************************/

static int ra_sci_dma_start_transfer(ra_dma_channel_t *dma)
{
    if (dma == NULL)
    {
        return -EINVAL;
    }

    /* Enable DMA transfer */
    putreg32(RA_DMAC_DMCNT_DTE, dma->base + RA_DMAC_DMCNT_OFFSET);

    /* Start transfer */
    putreg32(1, dma->base + RA_DMAC_DMREQ_OFFSET);

    return OK;
}

/****************************************************************************
 * Name: ra_sci_dma_stop_transfer
 *
 * Description:
 *   Stop DMA transfer
 *
 ****************************************************************************/

static void ra_sci_dma_stop_transfer(ra_dma_channel_t *dma)
{
    if (dma != NULL)
    {
        /* Disable DMA transfer */
        putreg32(0, dma->base + RA_DMAC_DMCNT_OFFSET);
    }
}

/****************************************************************************
 * Name: ra_sci_configure_pins
 *
 * Description:
 *   Configure UART pins
 *
 ****************************************************************************/

static int ra_sci_configure_pins(const ra_sci_config_t *config)
{
    int ret = OK;

    /* Configure TX pin */
    if (config->tx_pin != 0)
    {
        ret = ra_gpio_config(config->tx_pin);
        if (ret < 0)
        {
            return ret;
        }
    }

    /* Configure RX pin */
    if (config->rx_pin != 0)
    {
        ret = ra_gpio_config(config->rx_pin);
        if (ret < 0)
        {
            return ret;
        }
    }

    /* Configure flow control pins if enabled */
    if (config->flow_control)
    {
        if (config->rts_pin != 0)
        {
            ret = ra_gpio_config(config->rts_pin);
            if (ret < 0)
            {
                return ret;
            }
        }

        if (config->cts_pin != 0)
        {
            ret = ra_gpio_config(config->cts_pin);
            if (ret < 0)
            {
                return ret;
            }
        }
    }

    return ret;
}

/****************************************************************************
 * Name: ra_sci_configure_registers
 *
 * Description:
 *   Configure UART registers
 *
 ****************************************************************************/

static int ra_sci_configure_registers(ra_sci_dev_t *dev)
{
    const ra_sci_config_t *config = dev->config;
    uint32_t base = config->base;
    uint8_t smr = 0;
    uint8_t scr = 0;
    uint8_t semr = 0;
    uint32_t brr = 0;

    /* Disable transmit and receive */
    putreg8(0, base + R_SCI_SCR_OFFSET);

    /* Configure Serial Mode Register (SMR) */
    switch (config->bits)
    {
        case 7:
            smr |= RA_SCI_SMR_CHR;
            break;
        case 8:
            /* Default - 8 bits */
            break;
        default:
            return -EINVAL;
    }

    switch (config->parity)
    {
        case 0: /* No parity */
            break;
        case 1: /* Odd parity */
            smr |= RA_SCI_SMR_PE;
            break;
        case 2: /* Even parity */
            smr |= RA_SCI_SMR_PE | RA_SCI_SMR_PM;
            break;
        default:
            return -EINVAL;
    }

    if (config->stop == 2)
    {
        smr |= RA_SCI_SMR_STOP;
    }

    putreg8(smr, base + R_SCI_SMR_OFFSET);

    /* Configure baud rate */
    /* Simplified baud rate calculation - should be enhanced for production */
    brr = (CONFIG_RA_ICLK_FREQUENCY / (32 * config->baud)) - 1;
    if (brr > 255)
    {
        return -EINVAL;
    }

    putreg8(brr, base + R_SCI_BRR_OFFSET);

    /* Configure extended mode register */
    semr |= RA_SCI_SEMR_ABCS | RA_SCI_SEMR_BGDM;
    putreg8(semr, base + R_SCI_SEMR_OFFSET);

    /* Configure control register */
    scr = RA_SCI_SCR_TE | RA_SCI_SCR_RE | RA_SCI_SCR_TIE | RA_SCI_SCR_RIE;
    if (config->flow_control)
    {
        /* Enable transmit end interrupt for flow control */
        scr |= RA_SCI_SCR_TEIE;
    }

    putreg8(scr, base + R_SCI_SCR_OFFSET);

    return OK;
}

/****************************************************************************
 * Name: ra_sci_tx_dma_callback
 *
 * Description:
 *   TX DMA completion callback
 *
 ****************************************************************************/

static int ra_sci_tx_dma_callback(int irq, void *context, void *arg)
{
    ra_sci_dev_t *dev = (ra_sci_dev_t *)arg;

    if (dev != NULL)
    {
        dev->state = RA_UART_STATE_IDLE;
        dev->events |= RA_UART_EVENT_TX_COMPLETE;

        if (dev->callback != NULL)
        {
            dev->callback(dev, RA_UART_EVENT_TX_COMPLETE);
        }
    }

    return OK;
}

/****************************************************************************
 * Name: ra_sci_rx_dma_callback
 *
 * Description:
 *   RX DMA completion callback
 *
 ****************************************************************************/

static int ra_sci_rx_dma_callback(int irq, void *context, void *arg)
{
    ra_sci_dev_t *dev = (ra_sci_dev_t *)arg;

    if (dev != NULL)
    {
        dev->state = RA_UART_STATE_IDLE;
        dev->events |= RA_UART_EVENT_RX_COMPLETE;

        if (dev->callback != NULL)
        {
            dev->callback(dev, RA_UART_EVENT_RX_COMPLETE);
        }
    }

    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_sci_initialize
 *
 * Description:
 *   Initialize UART driver with DMA support
 *
 ****************************************************************************/

int ra_sci_initialize(ra_sci_dev_t *dev)
{
    int ret;

    if (dev == NULL || dev->config == NULL)
    {
        return -EINVAL;
    }

    /* Initialize device state */
    dev->state = RA_UART_STATE_UNINITIALIZED;
    dev->tx_dma_ctrl = NULL;
    dev->rx_dma_ctrl = NULL;
    dev->events = 0;

    /* Configure pins */
    ret = ra_sci_configure_pins(dev->config);
    if (ret < 0)
    {
        return ret;
    }

    /* Allocate DMA channels if enabled */
    if (dev->config->tx_dma.enabled)
    {
        dev->tx_dma_ctrl = ra_sci_dma_allocate_channel();
        if (dev->tx_dma_ctrl == NULL)
        {
            syslog(LOG_ERR, "Failed to allocate TX DMA channel\n");
            return -ENOMEM;
        }

        /* Attach TX DMA interrupt */
        ret = irq_attach(((ra_dma_channel_t *)dev->tx_dma_ctrl)->irq,
                         ra_sci_tx_dma_callback, dev);
        if (ret < 0)
        {
            ra_sci_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
            return ret;
        }

        up_enable_irq(((ra_dma_channel_t *)dev->tx_dma_ctrl)->irq);
    }

    if (dev->config->rx_dma.enabled)
    {
        dev->rx_dma_ctrl = ra_sci_dma_allocate_channel();
        if (dev->rx_dma_ctrl == NULL)
        {
            if (dev->tx_dma_ctrl != NULL)
            {
                ra_sci_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
            }
            syslog(LOG_ERR, "Failed to allocate RX DMA channel\n");
            return -ENOMEM;
        }

        /* Attach RX DMA interrupt */
        ret = irq_attach(((ra_dma_channel_t *)dev->rx_dma_ctrl)->irq,
                         ra_sci_rx_dma_callback, dev);
        if (ret < 0)
        {
            ra_sci_dma_free_channel((ra_dma_channel_t *)dev->rx_dma_ctrl);
            if (dev->tx_dma_ctrl != NULL)
            {
                ra_sci_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
            }
            return ret;
        }

        up_enable_irq(((ra_dma_channel_t *)dev->rx_dma_ctrl)->irq);
    }

    /* Configure UART registers */
    ret = ra_sci_configure_registers(dev);
    if (ret < 0)
    {
        ra_sci_finalize(dev);
        return ret;
    }

    dev->state = RA_UART_STATE_IDLE;

    return OK;
}

/****************************************************************************
 * Name: ra_sci_finalize
 *
 * Description:
 *   Finalize UART driver and free resources
 *
 ****************************************************************************/

void ra_sci_finalize(ra_sci_dev_t *dev)
{
    if (dev == NULL)
    {
        return;
    }

    /* Disable UART */
    if (dev->config != NULL)
    {
        putreg8(0, dev->config->base + R_SCI_SCR_OFFSET);
    }

    /* Free DMA channels */
    if (dev->tx_dma_ctrl != NULL)
    {
        ra_sci_dma_stop_transfer((ra_dma_channel_t *)dev->tx_dma_ctrl);
        up_disable_irq(((ra_dma_channel_t *)dev->tx_dma_ctrl)->irq);
        irq_detach(((ra_dma_channel_t *)dev->tx_dma_ctrl)->irq);
        ra_sci_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
        dev->tx_dma_ctrl = NULL;
    }

    if (dev->rx_dma_ctrl != NULL)
    {
        ra_sci_dma_stop_transfer((ra_dma_channel_t *)dev->rx_dma_ctrl);
        up_disable_irq(((ra_dma_channel_t *)dev->rx_dma_ctrl)->irq);
        irq_detach(((ra_dma_channel_t *)dev->rx_dma_ctrl)->irq);
        ra_sci_dma_free_channel((ra_dma_channel_t *)dev->rx_dma_ctrl);
        dev->rx_dma_ctrl = NULL;
    }

    /* Free buffers */
    if (dev->tx_buffer != NULL)
    {
        kmm_free(dev->tx_buffer);
        dev->tx_buffer = NULL;
    }

    if (dev->rx_buffer != NULL)
    {
        kmm_free(dev->rx_buffer);
        dev->rx_buffer = NULL;
    }

    dev->state = RA_UART_STATE_UNINITIALIZED;
}

/****************************************************************************
 * Name: ra_sci_send_dma
 *
 * Description:
 *   Send data using DMA
 *
 ****************************************************************************/

int ra_sci_send_dma(ra_sci_dev_t *dev, const uint8_t *buffer,
                     uint16_t length)
{
    ra_dma_channel_t *dma;
    int ret;

    if (dev == NULL || buffer == NULL || length == 0)
    {
        return -EINVAL;
    }

    if (dev->state != RA_UART_STATE_IDLE)
    {
        return -EBUSY;
    }

    if (!dev->config->tx_dma.enabled || dev->tx_dma_ctrl == NULL)
    {
        return -ENOTSUP;
    }

    dma = (ra_dma_channel_t *)dev->tx_dma_ctrl;

    /* Setup DMA transfer */
    ret = ra_sci_dma_setup_transfer(dma,
                                     (uint32_t)buffer,
                                     dev->config->base + R_SCI_TDR_OFFSET,
                                     length, true);
    if (ret < 0)
    {
        return ret;
    }

    /* Update state */
    dev->state = RA_UART_STATE_TX_IN_PROGRESS;
    dev->tx_count = length;

    /* Start DMA transfer */
    ret = ra_sci_dma_start_transfer(dma);
    if (ret < 0)
    {
        dev->state = RA_UART_STATE_IDLE;
        return ret;
    }

    return OK;
}

/****************************************************************************
 * Name: ra_sci_receive_dma
 *
 * Description:
 *   Receive data using DMA
 *
 ****************************************************************************/

int ra_sci_receive_dma(ra_sci_dev_t *dev, uint8_t *buffer,
                        uint16_t length)
{
    ra_dma_channel_t *dma;
    int ret;

    if (dev == NULL || buffer == NULL || length == 0)
    {
        return -EINVAL;
    }

    if (dev->state != RA_UART_STATE_IDLE)
    {
        return -EBUSY;
    }

    if (!dev->config->rx_dma.enabled || dev->rx_dma_ctrl == NULL)
    {
        return -ENOTSUP;
    }

    dma = (ra_dma_channel_t *)dev->rx_dma_ctrl;

    /* Setup DMA transfer */
    ret = ra_sci_dma_setup_transfer(dma,
                                     dev->config->base + R_SCI_RDR_OFFSET,
                                     (uint32_t)buffer,
                                     length, false);
    if (ret < 0)
    {
        return ret;
    }

    /* Update state */
    dev->state = RA_UART_STATE_RX_IN_PROGRESS;
    dev->rx_count = length;

    /* Start DMA transfer */
    ret = ra_sci_dma_start_transfer(dma);
    if (ret < 0)
    {
        dev->state = RA_UART_STATE_IDLE;
        return ret;
    }

    return OK;
}

/****************************************************************************
 * Name: ra_sci_abort_transfer
 *
 * Description:
 *   Abort ongoing DMA transfer
 *
 ****************************************************************************/

int ra_sci_abort_transfer(ra_sci_dev_t *dev, bool tx)
{
    ra_dma_channel_t *dma;

    if (dev == NULL)
    {
        return -EINVAL;
    }

    if (tx)
    {
        if (dev->state != RA_UART_STATE_TX_IN_PROGRESS)
        {
            return -EINVAL;
        }

        dma = (ra_dma_channel_t *)dev->tx_dma_ctrl;
    }
    else
    {
        if (dev->state != RA_UART_STATE_RX_IN_PROGRESS)
        {
            return -EINVAL;
        }

        dma = (ra_dma_channel_t *)dev->rx_dma_ctrl;
    }

    if (dma != NULL)
    {
        ra_sci_dma_stop_transfer(dma);
    }

    dev->state = RA_UART_STATE_IDLE;

    return OK;
}

/****************************************************************************
 * Name: ra_sci_set_callback
 *
 * Description:
 *   Set callback function for UART events
 *
 ****************************************************************************/

void ra_sci_set_callback(ra_sci_dev_t *dev,
                          void (*callback)(ra_sci_dev_t *dev, uint32_t event),
                          void *context)
{
    if (dev != NULL)
    {
        dev->callback = callback;
        dev->callback_context = context;
    }
}

/****************************************************************************
 * Name: ra_sci_config_baudrate
 *
 * Description:
 *   Configure UART baud rate
 *
 ****************************************************************************/

int ra_sci_config_baudrate(ra_sci_dev_t *dev, uint32_t baud)
{
    uint32_t brr;

    if (dev == NULL || dev->config == NULL)
    {
        return -EINVAL;
    }

    if (dev->state != RA_UART_STATE_IDLE)
    {
        return -EBUSY;
    }

    /* Calculate and set baud rate */
    brr = (CONFIG_RA_ICLK_FREQUENCY / (32 * baud)) - 1;
    if (brr > 255)
    {
        return -EINVAL;
    }

    putreg8(brr, dev->config->base + R_SCI_BRR_OFFSET);

    return OK;
}

/****************************************************************************
 * Name: ra_sci_get_status
 *
 * Description:
 *   Get UART status
 *
 ****************************************************************************/

uint32_t ra_sci_get_status(ra_sci_dev_t *dev)
{
    if (dev == NULL || dev->config == NULL)
    {
        return 0;
    }

    return getreg8(dev->config->base + R_SCI_SSR_OFFSET);
}
