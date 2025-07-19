/****************************************************************************
 * arch/arm/src/ra8/ra_adc.c
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/analog/adc.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ra_adc.h"
#include "ra_gpio.h"

#ifdef CONFIG_RA_ADC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_MAX_CHANNELS         29
#define ADC_FIFO_DEPTH           8

/* ADC channel configuration macros */

#define ADC_CHANNEL_MASK(ch)     (1 << (ch))

/* Timeout values for ADC operations */

#define ADC_TIMEOUT_US           10000  /* 10ms timeout */

/* DTC Transfer settings for ADC */

#define ADC_DTC_TRANSFER_SIZE    TRANSFER_SIZE_2_BYTE
#define ADC_DTC_MODE_NORMAL      0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* ADC channel configuration structure */

struct ra8_adc_chan_s
{
  uint8_t channel;                    /* ADC channel number */
  uint32_t pinmux;                    /* Pin multiplexing configuration */
};

/* RA8 ADC private data structure */

struct ra8_adc_priv_s
{
  const struct adc_callback_s *cb;    /* Upper half callback */
  uint32_t base;                      /* ADC base address */
  uint8_t intf;                       /* ADC interface number */
  int irq;                            /* ADC interrupt number */
  sem_t sem_excl;                     /* Mutual exclusion semaphore */
  uint32_t chanlist;                  /* Configured channel list */
  uint8_t nchannels;                  /* Number of configured channels */
  
#ifdef CONFIG_RA_ADC_DTC
  /* DTC related fields */
  bool dtc_enable;                    /* DTC transfer enabled */
  uint32_t *dma_buffer;               /* DMA buffer for multi-channel data */
  uint8_t *channel_buffer;            /* Channel ID buffer for DTC */
  size_t buffer_size;                 /* Buffer size */
#endif

  /* Configuration */
  enum ra8_adc_resolution_e resolution; /* ADC resolution */
  enum ra8_adc_mode_e mode;             /* Scan mode */
  enum ra8_adc_trigger_e trigger;       /* Trigger source */
  enum ra8_adc_alignment_e alignment;   /* Data alignment */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC operations */

static int  ra8_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback);
static void ra8_adc_reset(FAR struct adc_dev_s *dev);
static int  ra8_adc_setup(FAR struct adc_dev_s *dev);
static void ra8_adc_shutdown(FAR struct adc_dev_s *dev);
static void ra8_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  ra8_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg);

/* Interrupt handling */

static int  ra8_adc_interrupt(int irq, FAR void *context, FAR void *arg);

/* Utility functions */

static void ra8_adc_putreg(FAR struct ra8_adc_priv_s *priv,
                           int offset, uint32_t value);
static uint32_t ra8_adc_getreg(FAR struct ra8_adc_priv_s *priv,
                               int offset);
static void ra8_adc_modifyreg(FAR struct ra8_adc_priv_s *priv,
                              int offset, uint32_t clearbits,
                              uint32_t setbits);

/* Configuration functions */

static int  ra8_adc_configure(FAR struct ra8_adc_priv_s *priv);
static void ra8_adc_enable_channels(FAR struct ra8_adc_priv_s *priv);

#ifdef CONFIG_RA_ADC_DTC
/* DTC functions */

static int  ra8_adc_setup_dtc(FAR struct ra8_adc_priv_s *priv);
static void ra8_adc_cleanup_dtc(FAR struct ra8_adc_priv_s *priv);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = ra8_adc_bind,
  .ao_reset       = ra8_adc_reset,
  .ao_setup       = ra8_adc_setup,
  .ao_shutdown    = ra8_adc_shutdown,
  .ao_rxint       = ra8_adc_rxint,
  .ao_ioctl       = ra8_adc_ioctl,
};

/* ADC0 channel configuration */

static const struct ra8_adc_chan_s g_adc0_channels[] =
{
  {
    .channel = RA8_ADC_CHANNEL_AN000,    /* Battery voltage (P004) */
    .pinmux  = GPIO_P004_AN000,
  },
  {
    .channel = RA8_ADC_CHANNEL_AN001,    /* Reserved AN001 (P003) - Battery current AN104 mapped differently */
    .pinmux  = GPIO_P003_AN001,
  },
  /* Add more channels as needed */
};

#ifdef CONFIG_RA_ADC1
/* ADC1 channel configuration */

static const struct ra8_adc_chan_s g_adc1_channels[] =
{
  {
    .channel = RA8_ADC_CHANNEL_AN104,    /* Battery current (P003) mapped to ADC1 */
    .pinmux  = GPIO_P003_AN104,
  },
  /* Add more ADC1 channels as needed */
};
#endif

/* ADC0 private data */

static struct ra8_adc_priv_s g_adc0_priv =
{
  .base       = RA8_ADC0_BASE,
  .intf       = 0,
  .irq        = RA8E1_IRQ_ADC0_SCAN_END,
  .resolution = RA8_ADC_RESOLUTION_12BIT,
  .mode       = RA8_ADC_MODE_SINGLE_SCAN,
  .trigger    = RA8_ADC_TRIGGER_SOFTWARE,
  .alignment  = RA8_ADC_ALIGNMENT_RIGHT,
#ifdef CONFIG_RA_ADC_DTC
  .dtc_enable = true,
#endif
};

/* ADC0 device structure */

static struct adc_dev_s g_adc0_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adc0_priv,
};

#ifdef CONFIG_RA_ADC1
/* ADC1 private data */

static struct ra8_adc_priv_s g_adc1_priv =
{
  .base       = RA8_ADC1_BASE,
  .intf       = 1,
  .irq        = RA8E1_IRQ_ADC1_SCAN_END,
  .resolution = RA8_ADC_RESOLUTION_12BIT,
  .mode       = RA8_ADC_MODE_SINGLE_SCAN,
  .trigger    = RA8_ADC_TRIGGER_SOFTWARE,
  .alignment  = RA8_ADC_ALIGNMENT_RIGHT,
#ifdef CONFIG_RA_ADC_DTC
  .dtc_enable = true,
#endif
};

/* ADC1 device structure */

static struct adc_dev_s g_adc1_dev =
{
  .ad_ops  = &g_adcops,
  .ad_priv = &g_adc1_priv,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8_adc_putreg
 *
 * Description:
 *   Write a value to an ADC register
 *
 ****************************************************************************/

static void ra8_adc_putreg(FAR struct ra8_adc_priv_s *priv,
                           int offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: ra8_adc_getreg
 *
 * Description:
 *   Read a value from an ADC register
 *
 ****************************************************************************/

static uint32_t ra8_adc_getreg(FAR struct ra8_adc_priv_s *priv,
                               int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: ra8_adc_modifyreg
 *
 * Description:
 *   Modify an ADC register
 *
 ****************************************************************************/

static void ra8_adc_modifyreg(FAR struct ra8_adc_priv_s *priv,
                              int offset, uint32_t clearbits,
                              uint32_t setbits)
{
  modifyreg32(priv->base + offset, clearbits, setbits);
}

/****************************************************************************
 * Name: ra8_adc_enable_channels
 *
 * Description:
 *   Enable the configured ADC channels
 *
 ****************************************************************************/

static void ra8_adc_enable_channels(FAR struct ra8_adc_priv_s *priv)
{
  uint32_t adanse0 = 0;
  uint32_t adanse1 = 0;
  uint32_t chanlist = priv->chanlist;
  int i;

  /* Configure channel selection registers based on enabled channels */

  for (i = 0; i < 32; i++)
    {
      if (chanlist & (1 << i))
        {
          if (i < 16)
            {
              adanse0 |= (1 << i);
            }
          else if (i < 29)
            {
              adanse1 |= (1 << (i - 16));
            }
        }
    }

  /* Set channel selection registers */

  ra8_adc_putreg(priv, RA_ADC_ADANSE0_OFFSET, adanse0);
  ra8_adc_putreg(priv, RA_ADC_ADANSE1_OFFSET, adanse1);

  ainfo("ADC%d: Channel configuration - ADANSE0: 0x%08lx, ADANSE1: 0x%08lx\n",
        priv->intf, adanse0, adanse1);
}

/****************************************************************************
 * Name: ra8_adc_configure
 *
 * Description:
 *   Configure the ADC peripheral
 *
 ****************************************************************************/

static int ra8_adc_configure(FAR struct ra8_adc_priv_s *priv)
{
  uint32_t regval;

  /* Configure ADC control/status register (ADCSR) */

  regval = 0;
  regval |= (priv->mode << ADC_ADCSR_ADCS_SHIFT) & ADC_ADCSR_ADCS_MASK;

  if (priv->trigger != RA8_ADC_TRIGGER_SOFTWARE)
    {
      regval |= ADC_ADCSR_TRGE;
      if (priv->trigger == RA8_ADC_TRIGGER_ASYNC_EXT)
        {
          regval |= ADC_ADCSR_EXTRG;
        }
    }

  ra8_adc_putreg(priv, RA_ADC_ADCSR_OFFSET, regval);

  /* Configure ADC control extended register (ADCER) */

  regval = 0;
  if (priv->alignment == RA8_ADC_ALIGNMENT_LEFT)
    {
      regval |= ADC_ADCER_ADRFMT;
    }

  /* Enable automatic clearing of data registers */

  regval |= ADC_ADCER_ACE;

  ra8_adc_putreg(priv, RA_ADC_ADCER_OFFSET, regval);

  /* Configure channels */

  ra8_adc_enable_channels(priv);

  ainfo("ADC%d: Configuration complete\n", priv->intf);
  return OK;
}

#ifdef CONFIG_RA_ADC_DTC
/****************************************************************************
 * Name: ra8_adc_setup_dtc
 *
 * Description:
 *   Setup DTC (Data Transfer Controller) for ADC data transfer
 *
 ****************************************************************************/

static int ra8_adc_setup_dtc(FAR struct ra8_adc_priv_s *priv)
{
  size_t buffer_size;

  if (!priv->dtc_enable || priv->nchannels == 0)
    {
      return OK;
    }

  /* Calculate buffer size based on number of channels */

  buffer_size = priv->nchannels * sizeof(uint32_t);

  /* Allocate DMA buffer for ADC data */

  priv->dma_buffer = kmm_malloc(buffer_size);
  if (priv->dma_buffer == NULL)
    {
      aerr("ERROR: Failed to allocate DMA buffer\n");
      return -ENOMEM;
    }

  /* Allocate channel buffer for channel identification */

  priv->channel_buffer = kmm_malloc(priv->nchannels);
  if (priv->channel_buffer == NULL)
    {
      kmm_free(priv->dma_buffer);
      priv->dma_buffer = NULL;
      aerr("ERROR: Failed to allocate channel buffer\n");
      return -ENOMEM;
    }

  priv->buffer_size = buffer_size;

  /* Initialize channel buffer with channel numbers */

  int chan_idx = 0;
  for (int i = 0; i < 32; i++)
    {
      if (priv->chanlist & (1 << i))
        {
          priv->channel_buffer[chan_idx++] = i;
        }
    }

  ainfo("ADC%d: DTC setup complete - %d channels, buffer size: %zu\n",
        priv->intf, priv->nchannels, buffer_size);

  return OK;
}

/****************************************************************************
 * Name: ra8_adc_cleanup_dtc
 *
 * Description:
 *   Cleanup DTC resources
 *
 ****************************************************************************/

static void ra8_adc_cleanup_dtc(FAR struct ra8_adc_priv_s *priv)
{
  if (priv->dma_buffer != NULL)
    {
      kmm_free(priv->dma_buffer);
      priv->dma_buffer = NULL;
    }

  if (priv->channel_buffer != NULL)
    {
      kmm_free(priv->channel_buffer);
      priv->channel_buffer = NULL;
    }

  priv->buffer_size = 0;
}
#endif /* CONFIG_RA_ADC_DTC */

/****************************************************************************
 * Name: ra8_adc_interrupt
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int ra8_adc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct adc_dev_s *dev = (FAR struct adc_dev_s *)arg;
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;
  uint32_t regval;
  int32_t data;
  int i;

  /* Check if this is a scan end interrupt */

  regval = ra8_adc_getreg(priv, RA_ADC_ADCSR_OFFSET);
  if (!(regval & ADC_ADCSR_ADST))
    {
      /* Scan has completed */

#ifdef CONFIG_RA_ADC_DTC
      if (priv->dtc_enable && priv->cb->au_receive_batch != NULL)
        {
          /* Handle batch transfer with DTC */

          /* Read all channel data into DMA buffer */

          for (i = 0; i < priv->nchannels; i++)
            {
              int channel = priv->channel_buffer[i];
              data = ra8_adc_getreg(priv, RA_ADC_ADDR_OFFSET(channel)) & 0xFFFF;
              priv->dma_buffer[i] = data;
            }

          /* Call batch receive callback */

          priv->cb->au_receive_batch(dev, priv->channel_buffer,
                                     priv->dma_buffer, priv->nchannels);
        }
      else
#endif
        {
          /* Handle individual channel transfers */

          for (i = 0; i < 32; i++)
            {
              if (priv->chanlist & (1 << i))
                {
                  data = ra8_adc_getreg(priv, RA_ADC_ADDR_OFFSET(i)) & 0xFFFF;
                  
                  /* Call receive callback for each channel */

                  if (priv->cb && priv->cb->au_receive)
                    {
                      priv->cb->au_receive(dev, i, data);
                    }
                }
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ra8_adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *
 ****************************************************************************/

static int ra8_adc_bind(FAR struct adc_dev_s *dev,
                        FAR const struct adc_callback_s *callback)
{
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: ra8_adc_reset
 *
 * Description:
 *   Reset the ADC device. Called early to initialize the hardware.
 *
 ****************************************************************************/

static void ra8_adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;
  irqstate_t flags;

  ainfo("ADC%d: Reset\n", priv->intf);

  flags = enter_critical_section();

  /* Reset ADC control registers */

  ra8_adc_putreg(priv, RA_ADC_ADCSR_OFFSET, 0);
  ra8_adc_putreg(priv, RA_ADC_ADANSE0_OFFSET, 0);
  ra8_adc_putreg(priv, RA_ADC_ADANSE1_OFFSET, 0);
  ra8_adc_putreg(priv, RA_ADC_ADCER_OFFSET, 0);

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ra8_adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.
 *
 ****************************************************************************/

static int ra8_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;
  int ret;

  ainfo("ADC%d: Setup\n", priv->intf);

  /* Initialize semaphore */

  nxsem_init(&priv->sem_excl, 0, 1);

  /* Configure ADC */

  ret = ra8_adc_configure(priv);
  if (ret < 0)
    {
      aerr("ERROR: Failed to configure ADC%d: %d\n", priv->intf, ret);
      return ret;
    }

#ifdef CONFIG_RA_ADC_DTC
  /* Setup DTC if enabled */

  ret = ra8_adc_setup_dtc(priv);
  if (ret < 0)
    {
      aerr("ERROR: Failed to setup DTC for ADC%d: %d\n", priv->intf, ret);
      return ret;
    }
#endif

  /* Attach interrupt */

  ret = irq_attach(priv->irq, ra8_adc_interrupt, dev);
  if (ret < 0)
    {
      aerr("ERROR: Failed to attach interrupt for ADC%d: %d\n", priv->intf, ret);
#ifdef CONFIG_RA_ADC_DTC
      ra8_adc_cleanup_dtc(priv);
#endif
      return ret;
    }

  up_enable_irq(priv->irq);

  ainfo("ADC%d: Setup complete\n", priv->intf);
  return OK;
}

/****************************************************************************
 * Name: ra8_adc_shutdown
 *
 * Description:
 *   Disable the ADC. This method is called when the ADC device is closed.
 *
 ****************************************************************************/

static void ra8_adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;

  ainfo("ADC%d: Shutdown\n", priv->intf);

  /* Disable interrupt */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Stop any ongoing conversion */

  ra8_adc_putreg(priv, RA_ADC_ADCSR_OFFSET, 0);

#ifdef CONFIG_RA_ADC_DTC
  /* Cleanup DTC resources */

  ra8_adc_cleanup_dtc(priv);
#endif

  /* Destroy semaphore */

  nxsem_destroy(&priv->sem_excl);
}

/****************************************************************************
 * Name: ra8_adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void ra8_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;

  ainfo("ADC%d: RX interrupt %s\n", priv->intf, enable ? "enable" : "disable");

  if (enable)
    {
      /* Enable ADC scan end interrupt */

      ra8_adc_modifyreg(priv, RA_ADC_ADCSR_OFFSET, 0, ADC_ADCSR_ADIE);
    }
  else
    {
      /* Disable ADC scan end interrupt */

      ra8_adc_modifyreg(priv, RA_ADC_ADCSR_OFFSET, ADC_ADCSR_ADIE, 0);
    }
}

/****************************************************************************
 * Name: ra8_adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int ra8_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct ra8_adc_priv_s *priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;
  int ret = OK;

  ainfo("ADC%d: ioctl cmd=0x%02x arg=%lu\n", priv->intf, cmd, arg);

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Trigger a conversion */

          ra8_adc_modifyreg(priv, RA_ADC_ADCSR_OFFSET, 0, ADC_ADCSR_ADST);
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->nchannels;
        }
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8_adc_initialize
 *
 * Description:
 *   Initialize the ADC subsystem
 *
 * Input Parameters:
 *   intf - ADC interface number (0 or 1)
 *   chanlist - Bit mask of channels to enable
 *   nchannels - Number of channels enabled
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; NULL on failure
 *
 ****************************************************************************/

FAR struct adc_dev_s *ra8_adc_initialize(int intf, uint32_t chanlist,
                                         int nchannels)
{
  FAR struct adc_dev_s *dev;
  FAR struct ra8_adc_priv_s *priv;

  ainfo("Initializing ADC%d with chanlist=0x%08lx, nchannels=%d\n",
        intf, chanlist, nchannels);

  /* Select the correct ADC interface */

  switch (intf)
    {
      case 0:
        dev = &g_adc0_dev;
        break;

#ifdef CONFIG_RA_ADC1
      case 1:
        dev = &g_adc1_dev;
        break;
#endif

      default:
        aerr("ERROR: Invalid ADC interface: %d\n", intf);
        return NULL;
    }

  priv = (FAR struct ra8_adc_priv_s *)dev->ad_priv;

  /* Configure channel list */

  priv->chanlist = chanlist;
  priv->nchannels = nchannels;

  /* Configure GPIO pins for ADC channels */

  const struct ra8_adc_chan_s *channels;
  int num_channels;

  if (intf == 0)
    {
      channels = g_adc0_channels;
      num_channels = sizeof(g_adc0_channels) / sizeof(g_adc0_channels[0]);
    }
#ifdef CONFIG_RA_ADC1
  else
    {
      channels = g_adc1_channels;
      num_channels = sizeof(g_adc1_channels) / sizeof(g_adc1_channels[0]);
    }
#endif

  /* Configure pins for enabled channels */

  for (int i = 0; i < num_channels; i++)
    {
      if (chanlist & ADC_CHANNEL_MASK(channels[i].channel))
        {
          ra_gpio_config(channels[i].pinmux);
          ainfo("ADC%d: Configured pin for channel %d\n", 
                intf, channels[i].channel);
        }
    }

  ainfo("ADC%d initialization complete\n", intf);
  return dev;
}

#endif /* CONFIG_RA_ADC */
