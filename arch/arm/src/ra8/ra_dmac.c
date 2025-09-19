/****************************************************************************
 * arch/arm/src/ra8/ra_dmac.c
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
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ra_dmac.h"
#include "ra_dmac.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMAC_OPEN_ID             (0x444D4143)  /* "DMAC" in ASCII */
#define DMAC_ALIGNMENT_CHECK(addr, size) \
  ((addr) & ((1U << (size)) - 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMAC channel control structure */
typedef struct ra_dmac_ctrl_s
{
  uint32_t             open_id;        /* Open identifier */
  uint8_t              channel;        /* Channel number */
  bool                 enabled;        /* Channel enabled flag */
  bool                 in_use;         /* Channel in use flag */
  ra_dmac_config_t    *config;         /* Configuration */
  int                  irq;            /* IRQ number */
} ra_dmac_ctrl_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMAC channel control blocks */
static ra_dmac_ctrl_t g_dmac_channels[RA_DMAC_NUM_CHANNELS];

/* DMAC module initialized flag */
static bool g_dmac_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dmac_validate_config
 *
 * Description:
 *   Validate DMAC configuration parameters
 *
 ****************************************************************************/

static int ra_dmac_validate_config(const ra_dmac_config_t *config)
{
  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Check transfer length based on mode */
  switch (config->mode)
    {
      case RA_DMAC_MODE_NORMAL:
        if (config->transfer_count == 0 ||
            config->transfer_count > RA_DMAC_MAX_NORMAL_LENGTH)
          {
            return -EINVAL;
          }
        break;

      case RA_DMAC_MODE_REPEAT:
      case RA_DMAC_MODE_BLOCK:
        if (config->transfer_count == 0 ||
            config->transfer_count > RA_DMAC_MAX_REPEAT_LENGTH)
          {
            return -EINVAL;
          }
        break;

      default:
        return -EINVAL;
    }

  /* Check address alignment */
  if (DMAC_ALIGNMENT_CHECK(config->src_addr, config->size) ||
      DMAC_ALIGNMENT_CHECK(config->dest_addr, config->size))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dmac_setup_channel
 *
 * Description:
 *   Setup DMAC channel registers
 *
 ****************************************************************************/

static int ra_dmac_setup_channel(ra_dmac_ctrl_t *ctrl)
{
  ra_dmac_config_t *config = ctrl->config;
  uint8_t channel = ctrl->channel;
  uint32_t dmtmd = 0;
  uint32_t dmamd = 0;
  uint32_t dmint = 0;

  /* Setup Transfer Mode Register (DMTMD) */
  dmtmd |= (config->trigger << RA_DMAC_DMTMD_DCTG_SHIFT) & RA_DMAC_DMTMD_DCTG_MASK;
  dmtmd |= (config->size << RA_DMAC_DMTMD_SZ_SHIFT) & RA_DMAC_DMTMD_SZ_MASK;
  dmtmd |= (config->mode << RA_DMAC_DMTMD_DTS_SHIFT) & RA_DMAC_DMTMD_DTS_MASK;
  dmtmd |= (config->mode << RA_DMAC_DMTMD_MD_SHIFT) & RA_DMAC_DMTMD_MD_MASK;

  /* Setup Address Mode Register (DMAMD) */
  dmamd |= (config->dest_addr_mode << RA_DMAC_DMAMD_DM_SHIFT) & RA_DMAC_DMAMD_DM_MASK;
  dmamd |= (config->src_addr_mode << RA_DMAC_DMAMD_SM_SHIFT) & RA_DMAC_DMAMD_SM_MASK;

  /* Setup Interrupt Setting Register (DMINT) */
  if (config->callback != NULL)
    {
      dmint |= RA_DMAC_DMINT_DTIE;  /* Enable transfer complete interrupt */
    }

  /* Write registers */
  putreg32(config->src_addr, RA_DMAC_DMSAR(channel));
  putreg32(config->dest_addr, RA_DMAC_DMDAR(channel));
  putreg32(config->transfer_count, RA_DMAC_DMCRA(channel));
  putreg32(config->block_count, RA_DMAC_DMCRB(channel));
  putreg32(dmtmd, RA_DMAC_DMTMD(channel));
  putreg32(dmamd, RA_DMAC_DMAMD(channel));
  putreg8(dmint, RA_DMAC_DMINT(channel));

  return OK;
}

/****************************************************************************
 * Name: ra_dmac_interrupt_handler
 *
 * Description:
 *   DMAC interrupt handler
 *
 ****************************************************************************/

static int ra_dmac_interrupt_handler(int irq, void *context, void *arg)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)arg;
  uint32_t status;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return OK;
    }

  /* Read and clear status */
  status = getreg32(RA_DMAC_DMSTS(ctrl->channel));

  if (status & RA_DMAC_DMSTS_DTIF)
    {
      /* Clear interrupt flag */
      putreg32(status | RA_DMAC_DMSTS_DTIF, RA_DMAC_DMSTS(ctrl->channel));

      /* Call user callback */
      if (ctrl->config && ctrl->config->callback)
        {
          ctrl->config->callback(ctrl, RA_DMAC_EVENT_COMPLETE,
                                 ctrl->config->user_data);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dmac_find_free_channel
 *
 * Description:
 *   Find a free DMAC channel
 *
 ****************************************************************************/

static int ra_dmac_find_free_channel(void)
{
  int i;

  for (i = 0; i < RA_DMAC_NUM_CHANNELS; i++)
    {
      if (!g_dmac_channels[i].in_use)
        {
          return i;
        }
    }

  return -ENOMEM;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dmac_initialize
 *
 * Description:
 *   Initialize the DMAC module
 *
 ****************************************************************************/

int ra_dmac_initialize(void)
{
  int i;

  if (g_dmac_initialized)
    {
      return OK;
    }

  /* Initialize channel control blocks */
  for (i = 0; i < RA_DMAC_NUM_CHANNELS; i++)
    {
      memset(&g_dmac_channels[i], 0, sizeof(ra_dmac_ctrl_t));
      g_dmac_channels[i].channel = i;
    }

  g_dmac_initialized = true;

  dminfo("DMAC initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_open
 *
 * Description:
 *   Open and configure a DMAC transfer
 *
 ****************************************************************************/

int ra_dmac_open(ra_dmac_handle_t *handle, const ra_dmac_config_t *config)
{
  ra_dmac_ctrl_t *ctrl;
  int channel;
  int ret;

  if (handle == NULL || config == NULL)
    {
      return -EINVAL;
    }

  /* Validate configuration */
  ret = ra_dmac_validate_config(config);
  if (ret < 0)
    {
      return ret;
    }

  /* Find free channel */
  channel = ra_dmac_find_free_channel();
  if (channel < 0)
    {
      return channel;
    }

  ctrl = &g_dmac_channels[channel];

  /* Allocate and copy configuration */
  ctrl->config = kmm_zalloc(sizeof(ra_dmac_config_t));
  if (ctrl->config == NULL)
    {
      return -ENOMEM;
    }

  memcpy(ctrl->config, config, sizeof(ra_dmac_config_t));

  /* Setup channel registers */
  ret = ra_dmac_setup_channel(ctrl);
  if (ret < 0)
    {
      kmm_free(ctrl->config);
      return ret;
    }

  /* Attach interrupt if callback is provided */
  if (config->callback != NULL)
    {
      ret = ra_icu_attach(config->activation_source,
                          ra_dmac_interrupt_handler, ctrl);
      if (ret < 0)
        {
          kmm_free(ctrl->config);
          return ret;
        }
      ctrl->irq = ret;  /* Store the assigned IRQ slot number */
    }

  ctrl->open_id = DMAC_OPEN_ID;
  ctrl->in_use = true;
  *handle = ctrl;

  dminfo("DMAC channel %d opened successfully\n", channel);
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_enable
 *
 * Description:
 *   Enable DMAC transfer
 *
 ****************************************************************************/

int ra_dmac_enable(ra_dmac_handle_t handle)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Enable the channel */
  putreg32(1 << ctrl->channel, RA_DMAC_DMAST);

  /* Enable interrupt if assigned */
  if (ctrl->irq >= 0)
    {
      up_enable_irq(ctrl->irq);  /* Enable the assigned IRQ slot */
    }

  ctrl->enabled = true;

  dminfo("DMAC channel %d enabled\n", ctrl->channel);
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_disable
 *
 * Description:
 *   Disable DMAC transfer
 *
 ****************************************************************************/

int ra_dmac_disable(ra_dmac_handle_t handle)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Disable interrupt if assigned */
  if (ctrl->irq >= 0)
    {
      up_disable_irq(ctrl->irq);  /* Disable the assigned IRQ slot */
    }

  /* Disable the channel */
  putreg32(1 << ctrl->channel, RA_DMAC_DMAST);

  ctrl->enabled = false;

  dminfo("DMAC channel %d disabled\n", ctrl->channel);
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_software_start
 *
 * Description:
 *   Start DMAC transfer by software trigger
 *
 ****************************************************************************/

int ra_dmac_software_start(ra_dmac_handle_t handle)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Software start request */
  putreg32(RA_DMAC_DMREQ_SWREQ, RA_DMAC_DMREQ(ctrl->channel));

  dminfo("DMAC channel %d software start\n", ctrl->channel);
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_reset
 *
 * Description:
 *   Reset DMAC transfer addresses and count
 *
 ****************************************************************************/

int ra_dmac_reset(ra_dmac_handle_t handle, uint32_t src_addr,
                  uint32_t dest_addr, uint32_t transfer_count)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Check address alignment */
  if (DMAC_ALIGNMENT_CHECK(src_addr, ctrl->config->size) ||
      DMAC_ALIGNMENT_CHECK(dest_addr, ctrl->config->size))
    {
      return -EINVAL;
    }

  /* Update registers */
  putreg32(src_addr, RA_DMAC_DMSAR(ctrl->channel));
  putreg32(dest_addr, RA_DMAC_DMDAR(ctrl->channel));
  putreg32(transfer_count, RA_DMAC_DMCRA(ctrl->channel));

  dminfo("DMAC channel %d reset: src=0x%08lx, dest=0x%08lx, count=%ld\n",
         ctrl->channel, src_addr, dest_addr, transfer_count);
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_close
 *
 * Description:
 *   Close DMAC transfer and free resources
 *
 ****************************************************************************/

int ra_dmac_close(ra_dmac_handle_t handle)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Disable transfer first */
  ra_dmac_disable(handle);

  /* Detach interrupt */
  if (ctrl->irq >= 0)
    {
      ra_icu_detach(ctrl->irq);  /* Use the assigned IRQ slot number */
    }

  /* Free allocated memory */
  if (ctrl->config != NULL)
    {
      kmm_free(ctrl->config);
      ctrl->config = NULL;
    }

  /* Clear control structure */
  ctrl->open_id = 0;
  ctrl->enabled = false;
  ctrl->in_use = false;
  ctrl->irq = -1;

  dminfo("DMAC channel %d closed\n", ctrl->channel);
  return OK;
}

/****************************************************************************
 * Name: ra_dmac_get_remaining_count
 *
 * Description:
 *   Get remaining transfer count
 *
 ****************************************************************************/

uint32_t ra_dmac_get_remaining_count(ra_dmac_handle_t handle)
{
  ra_dmac_ctrl_t *ctrl = (ra_dmac_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DMAC_OPEN_ID)
    {
      return 0;
    }

  return getreg32(RA_DMAC_DMCRA(ctrl->channel));
}

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
#include "hardware/ra_dmac.h"
#include "hardware/ra_memorymap.h"
#include "ra_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA Control Block */
typedef struct
{
    uint32_t base;              /* DMA channel base address */
    uint8_t  channel;           /* DMA channel number */
    int      irq;               /* DMA interrupt slot number */
    int      el;                /* DMA interrupt even link number */
    bool     in_use;            /* Channel in use flag */
} ra_dma_channel_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA channel pool */
static ra_dma_channel_t g_dma_channels[] =
{
    { R_DMAC0_BASE, 0, -1, RA_EL_DMAC0_INT, false },
    { R_DMAC1_BASE, 1, -1, RA_EL_DMAC1_INT, false },
    { R_DMAC2_BASE, 2, -1, RA_EL_DMAC2_INT, false },
};

#define RA_NUM_DMA_CHANNELS (sizeof(g_dma_channels) / sizeof(g_dma_channels[0]))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dma_allocate_channel
 *
 * Description:
 *   Allocate a DMA channel
 *
 ****************************************************************************/

static ra_dma_channel_t *ra_dma_allocate_channel(void)
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
 * Name: ra_dma_free_channel
 *
 * Description:
 *   Free a DMA channel
 *
 ****************************************************************************/

static void ra_dma_free_channel(ra_dma_channel_t *channel)
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
 * Name: ra_dma_setup_transfer
 *
 * Description:
 *   Setup DMA transfer
 *
 ****************************************************************************/

static int ra_dma_setup_transfer(ra_dma_channel_t *dma,
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
 * Name: ra_dma_start_transfer
 *
 * Description:
 *   Start DMA transfer
 *
 ****************************************************************************/

static int ra_dma_start_transfer(ra_dma_channel_t *dma)
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
 * Name: ra_dma_stop_transfer
 *
 * Description:
 *   Stop DMA transfer
 *
 ****************************************************************************/

static void ra_dma_stop_transfer(ra_dma_channel_t *dma)
{
    if (dma != NULL)
    {
        /* Disable DMA transfer */
        putreg32(0, dma->base + RA_DMAC_DMCNT_OFFSET);
    }
}

/****************************************************************************
 * Name: ra_dma_configure_registers
 *
 * Description:
 *   Configure UART registers
 *
 ****************************************************************************/

static int ra_dma_configure_registers(ra_dma_dev_t *dev)
{
    const ra_dma_config_t *config = dev->config;
    uint32_t base = config->base;
    return OK;
}

/****************************************************************************
 * Name: ra_dma_tx_dma_callback
 *
 * Description:
 *   TX DMA completion callback
 *
 ****************************************************************************/

static int ra_dma_tx_dma_callback(int irq, void *context, void *arg)
{
    ra_dma_dev_t *dev = (ra_dma_dev_t *)arg;

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
 * Name: ra_dma_rx_dma_callback
 *
 * Description:
 *   RX DMA completion callback
 *
 ****************************************************************************/

static int ra_dma_rx_dma_callback(int irq, void *context, void *arg)
{
    ra_dma_dev_t *dev = (ra_dma_dev_t *)arg;

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
 * Name: ra_dma_initialize
 *
 * Description:
 *   Initialize UART driver with DMA support
 *
 ****************************************************************************/

int ra_dma_initialize(ra_dma_dev_t *dev)
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
    ret = ra_dma_configure_pins(dev->config);
    if (ret < 0)
    {
        return ret;
    }

    /* Allocate DMA channels if enabled */
    if (dev->config->tx_dma.enabled)
    {
        dev->tx_dma_ctrl = ra_dma_allocate_channel();
        if (dev->tx_dma_ctrl == NULL)
        {
            syslog(LOG_ERR, "Failed to allocate TX DMA channel\n");
            return -ENOMEM;
        }

        /* Attach TX DMA interrupt */
        ret = ra_icu_attach(((ra_dma_channel_t *)dev->tx_dma_ctrl)->el,
                         ra_dma_tx_dma_callback, dev);
        if (ret < 0)
        {
            ra_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
            return ret;
        }

        up_enable_irq(((ra_dma_channel_t *)dev->tx_dma_ctrl)->el);
    }

    if (dev->config->rx_dma.enabled)
    {
        dev->rx_dma_ctrl = ra_dma_allocate_channel();
        if (dev->rx_dma_ctrl == NULL)
        {
            if (dev->tx_dma_ctrl != NULL)
            {
                ra_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
            }
            syslog(LOG_ERR, "Failed to allocate RX DMA channel\n");
            return -ENOMEM;
        }

        /* Attach RX DMA interrupt */
        ret = irq_attach(((ra_dma_channel_t *)dev->rx_dma_ctrl)->el,
                         ra_dma_rx_dma_callback, dev);
        if (ret < 0)
        {
            ra_dma_free_channel((ra_dma_channel_t *)dev->rx_dma_ctrl);
            if (dev->tx_dma_ctrl != NULL)
            {
                ra_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
            }
            return ret;
        }

        up_enable_irq(((ra_dma_channel_t *)dev->rx_dma_ctrl)->el);
    }

    /* Configure UART registers */
    ret = ra_dma_configure_registers(dev);
    if (ret < 0)
    {
        ra_dma_finalize(dev);
        return ret;
    }

    dev->state = RA_UART_STATE_IDLE;

    return OK;
}

/****************************************************************************
 * Name: ra_dma_finalize
 *
 * Description:
 *   Finalize UART driver and free resources
 *
 ****************************************************************************/

void ra_dma_finalize(ra_dma_dev_t *dev)
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
        ra_dma_stop_transfer((ra_dma_channel_t *)dev->tx_dma_ctrl);
        ra_icu_detach(((ra_dma_channel_t *)dev->tx_dma_ctrl)->el);
        ra_dma_free_channel((ra_dma_channel_t *)dev->tx_dma_ctrl);
        dev->tx_dma_ctrl = NULL;
    }

    if (dev->rx_dma_ctrl != NULL)
    {
        ra_dma_stop_transfer((ra_dma_channel_t *)dev->rx_dma_ctrl);
        ra_icu_detach(((ra_dma_channel_t *)dev->rx_dma_ctrl)->irq);
        ra_dma_free_channel((ra_dma_channel_t *)dev->rx_dma_ctrl);
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
 * Name: ra_dma_send_dma
 *
 * Description:
 *   Send data using DMA
 *
 ****************************************************************************/

int ra_dma_send_dma(ra_dma_dev_t *dev, const uint8_t *buffer,
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
    ret = ra_dma_setup_transfer(dma,
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
    ret = ra_dma_start_transfer(dma);
    if (ret < 0)
    {
        dev->state = RA_UART_STATE_IDLE;
        return ret;
    }

    return OK;
}

/****************************************************************************
 * Name: ra_dma_receive_dma
 *
 * Description:
 *   Receive data using DMA
 *
 ****************************************************************************/

int ra_dma_receive_dma(ra_dma_dev_t *dev, uint8_t *buffer,
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
    ret = ra_dma_setup_transfer(dma,
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
    ret = ra_dma_start_transfer(dma);
    if (ret < 0)
    {
        dev->state = RA_UART_STATE_IDLE;
        return ret;
    }

    return OK;
}

/****************************************************************************
 * Name: ra_dma_abort_transfer
 *
 * Description:
 *   Abort ongoing DMA transfer
 *
 ****************************************************************************/

int ra_dma_abort_transfer(ra_dma_dev_t *dev, bool tx)
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
        ra_dma_stop_transfer(dma);
    }

    dev->state = RA_UART_STATE_IDLE;

    return OK;
}

/****************************************************************************
 * Name: ra_dma_set_callback
 *
 * Description:
 *   Set callback function for UART events
 *
 ****************************************************************************/

void ra_dma_set_callback(ra_dma_dev_t *dev,
                          void (*callback)(ra_dma_dev_t *dev, uint32_t event),
                          void *context)
{
    if (dev != NULL)
    {
        dev->callback = callback;
        dev->callback_context = context;
    }
}

/****************************************************************************
 * Name: ra_dma_get_status
 *
 * Description:
 *   Get UART status
 *
 ****************************************************************************/

uint32_t ra_dma_get_status(ra_dma_dev_t *dev)
{
    if (dev == NULL || dev->config == NULL)
    {
        return 0;
    }

    return getreg8(dev->config->base + R_SCI_SSR_OFFSET);
}
