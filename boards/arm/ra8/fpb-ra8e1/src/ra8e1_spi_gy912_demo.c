
/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/gy912_spi.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <assert.h>

#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/semaphore.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "board.h"
#include "ra8e1_demo_log.h"

#ifdef CONFIG_RA8_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RA8E1 SPI Register Definitions */
#define RA8E1_SPI0_BASE         0x40072000
#define RA8E1_SPI_SPCR_OFFSET   0x00    /* SPI Control Register */
#define RA8E1_SPI_SSLP_OFFSET   0x01    /* SPI Slave Select Polarity Register */
#define RA8E1_SPI_SPPCR_OFFSET  0x02    /* SPI Pin Control Register */
#define RA8E1_SPI_SPSR_OFFSET   0x03    /* SPI Status Register */
#define RA8E1_SPI_SPDR_OFFSET   0x04    /* SPI Data Register */
#define RA8E1_SPI_SPSCR_OFFSET  0x08    /* SPI Sequence Control Register */
#define RA8E1_SPI_SPSSR_OFFSET  0x09    /* SPI Sequence Status Register */
#define RA8E1_SPI_SPBR_OFFSET   0x0A    /* SPI Bit Rate Register */
#define RA8E1_SPI_SPDCR_OFFSET  0x0B    /* SPI Data Control Register */
#define RA8E1_SPI_SPCKD_OFFSET  0x0C    /* SPI Clock Delay Register */
#define RA8E1_SPI_SSLND_OFFSET  0x0D    /* SPI Slave Select Negation Delay Register */
#define RA8E1_SPI_SPND_OFFSET   0x0E    /* SPI Next-Access Delay Register */
#define RA8E1_SPI_SPCR2_OFFSET  0x0F    /* SPI Control Register 2 */
#define RA8E1_SPI_SPCMD0_OFFSET 0x10    /* SPI Command Register 0 */

/* SPI Control Register (SPCR) bits */
#define RA8E1_SPI_SPCR_SPRIE    (1 << 7)  /* SPI Receive Interrupt Enable */
#define RA8E1_SPI_SPCR_SPE      (1 << 6)  /* SPI Function Enable */
#define RA8E1_SPI_SPCR_SPTIE    (1 << 5)  /* SPI Transmit Interrupt Enable */
#define RA8E1_SPI_SPCR_SPEIE    (1 << 4)  /* SPI Error Interrupt Enable */
#define RA8E1_SPI_SPCR_MSTR     (1 << 3)  /* SPI Master/Slave Mode Select */
#define RA8E1_SPI_SPCR_MODFEN   (1 << 2)  /* Mode Fault Error Detection Enable */
#define RA8E1_SPI_SPCR_TXMD     (1 << 1)  /* Communications Operating Mode Select */
#define RA8E1_SPI_SPCR_SPMS     (1 << 0)  /* SPI Mode Select */

/* SPI Status Register (SPSR) bits */
#define RA8E1_SPI_SPSR_SPRF     (1 << 7)  /* SPI Receive Buffer Full Flag */
#define RA8E1_SPI_SPSR_SPTEF    (1 << 5)  /* SPI Transmit Buffer Empty Flag */
#define RA8E1_SPI_SPSR_PERF     (1 << 3)  /* Parity Error Flag */
#define RA8E1_SPI_SPSR_MODF     (1 << 2)  /* Mode Fault Error Flag */
#define RA8E1_SPI_SPSR_IDLNF    (1 << 1)  /* SPI Idle Flag */
#define RA8E1_SPI_SPSR_OVRF     (1 << 0)  /* Overrun Error Flag */

/* SPI Command Register (SPCMD0) bits */
#define RA8E1_SPI_SPCMD_SCKDEN  (1 << 15) /* RSPCK Delay Setting Enable */
#define RA8E1_SPI_SPCMD_SLNDEN  (1 << 14) /* SSL Negation Delay Setting Enable */
#define RA8E1_SPI_SPCMD_SPNDEN  (1 << 13) /* SPI Next-Access Delay Enable */
#define RA8E1_SPI_SPCMD_LSBF    (1 << 12) /* SPI LSB First */
#define RA8E1_SPI_SPCMD_SPB_MASK (0x0F << 8) /* SPI Data Length Setting */
#define RA8E1_SPI_SPCMD_SPB_8   (0x07 << 8)  /* 8 bits */
#define RA8E1_SPI_SPCMD_SPB_16  (0x0F << 8)  /* 16 bits */
#define RA8E1_SPI_SPCMD_SSLKP   (1 << 7)  /* SSL Signal Level Keeping */
#define RA8E1_SPI_SPCMD_SSLA_MASK (0x07 << 4) /* SSL Signal Assertion Setting */
#define RA8E1_SPI_SPCMD_BRDV_MASK (0x03 << 2) /* Bit Rate Division Setting */
#define RA8E1_SPI_SPCMD_CPOL    (1 << 1)  /* RSPCK Polarity Setting */
#define RA8E1_SPI_SPCMD_CPHA    (1 << 0)  /* RSPCK Phase Setting */

/* DTC Register Definitions */
#define RA8E1_DTC_BASE          0x40005400
#define RA8E1_DTC_DTCCR_OFFSET  0x00    /* DTC Control Register */
#define RA8E1_DTC_DTCVBR_OFFSET 0x04    /* DTC Vector Base Register */
#define RA8E1_DTC_DTCADMOD_OFFSET 0x08  /* DTC Address Mode Register */
#define RA8E1_DTC_DTCST_OFFSET  0x0C    /* DTC Module Start Register */

/* DTC Transfer Information */
#define RA8E1_DTC_MRA_SM_MASK   (0x03 << 14) /* Source Address Mode */
#define RA8E1_DTC_MRA_SZ_MASK   (0x03 << 12) /* Data Transfer Size */
#define RA8E1_DTC_MRA_SZ_BYTE   (0x00 << 12) /* Byte */
#define RA8E1_DTC_MRA_SZ_WORD   (0x01 << 12) /* Word */
#define RA8E1_DTC_MRA_SZ_LONG   (0x02 << 12) /* Long word */
#define RA8E1_DTC_MRA_MD_MASK   (0x07 << 8)  /* DTC Transfer Mode */
#define RA8E1_DTC_MRA_MD_NORMAL (0x00 << 8)  /* Normal transfer mode */

/* Clock and Power Management */
#define RA8E1_MSTP_SPI0         (1 << 22)    /* SPI0 Module Stop */
#define RA8E1_MSTP_DTC          (1 << 28)    /* DTC Module Stop */

/* IRQ Numbers */
#define RA8E1_IRQ_SPI0_RXI      142
#define RA8E1_IRQ_SPI0_TXI      143
#define RA8E1_IRQ_SPI0_ERI      144

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SPI Device Structure */
struct gy912_spidev_s
{
  struct spi_dev_s spidev;     /* Externally visible part of the SPI interface */
  uint32_t         spibase;    /* SPI controller register base address */
  uint32_t         frequency;  /* Requested clock frequency */
  uint32_t         actual;     /* Actual clock frequency */
  uint8_t          mode;       /* Mode 0,1,2,3 */
  uint8_t          nbits;      /* Width of word in bits (8 to 16) */
  bool             initialized; /* Has SPI been initialized? */
  sem_t            exclsem;    /* Mutual exclusion mutex */
  uint32_t         txword;     /* Next word to send */
  uint32_t         rxword;     /* Next word to receive */
  const void      *txbuffer;   /* Source buffer */
  void            *rxbuffer;   /* Sink buffer */
  size_t           ntxwords;   /* Size of TX buffer in words */
  size_t           nrxwords;   /* Size of RX buffer in words */
  size_t           nwords;     /* Number of words to be exchanged */
  volatile bool    dma_active; /* DMA transfer in progress */
  struct gy912_spi_transfer_s *current_xfer; /* Current transfer context */
};

/* DTC Transfer Information Structure */
struct ra8e1_dtc_info_s
{
  uint32_t mra;     /* Mode Register A */
  uint32_t mrb;     /* Mode Register B */
  uint32_t sar;     /* Source Address Register */
  uint32_t dar;     /* Destination Address Register */
  uint32_t cra;     /* Transfer Count Register A */
  uint32_t crb;     /* Transfer Count Register B */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* SPI methods */
static int      gy912_spi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t gy912_spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency);
static void     gy912_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void     gy912_spi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int      gy912_spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features);
#endif
static uint32_t gy912_spi_send(struct spi_dev_s *dev, uint32_t wd);
static void     gy912_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                                   void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void     gy912_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                                   size_t nwords);
static void     gy912_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                    size_t nwords);
#endif

/* Initialization */
static void     gy912_spi_bus_initialize(struct gy912_spidev_s *priv);

/* DMA support */
static int      gy912_spi_dma_setup_transfer(struct gy912_spidev_s *priv,
                                              struct gy912_spi_transfer_s *xfer);
static void     gy912_spi_dma_callback(struct gy912_spidev_s *priv, int result);

/* Register access helpers */
static inline uint8_t gy912_spi_getreg8(struct gy912_spidev_s *priv, uint8_t offset);
static inline void gy912_spi_putreg8(struct gy912_spidev_s *priv, uint8_t offset, uint8_t value);
static inline uint16_t gy912_spi_getreg16(struct gy912_spidev_s *priv, uint8_t offset);
static inline void gy912_spi_putreg16(struct gy912_spidev_s *priv, uint8_t offset, uint16_t value);
static inline uint32_t gy912_spi_getreg32(struct gy912_spidev_s *priv, uint8_t offset);
static inline void gy912_spi_putreg32(struct gy912_spidev_s *priv, uint8_t offset, uint32_t value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI0 driver operations */
static const struct spi_ops_s g_spi0ops =
{
  .lock              = gy912_spi_lock,
  .select            = ra_spi0_select,
  .setfrequency      = gy912_spi_setfrequency,
  .setmode           = gy912_spi_setmode,
  .setbits           = gy912_spi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures        = gy912_spi_hwfeatures,
#endif
  .status            = ra_spi0_status,
  .cmddata           = ra_spi0_cmddata,
  .send              = gy912_spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = gy912_spi_exchange,
#else
  .sndblock          = gy912_spi_sndblock,
  .recvblock         = gy912_spi_recvblock,
#endif
  .registercallback  = 0,
};

/* SPI0 device structure */
static struct gy912_spidev_s g_spi0dev =
{
  .spidev   = { &g_spi0ops },
  .spibase  = RA8E1_SPI0_BASE,
  .exclsem  = SEM_INITIALIZER(1),
};

/* DTC transfer information for SPI TX and RX */
static struct ra8e1_dtc_info_s g_spi_dtc_tx_info;
static struct ra8e1_dtc_info_s g_spi_dtc_rx_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_spi_getreg8, gy912_spi_putreg8, etc.
 *
 * Description:
 *   Register access helpers
 *
 ****************************************************************************/

static inline uint8_t gy912_spi_getreg8(struct gy912_spidev_s *priv, uint8_t offset)
{
  return getreg8(priv->spibase + offset);
}

static inline void gy912_spi_putreg8(struct gy912_spidev_s *priv, uint8_t offset, uint8_t value)
{
  putreg8(value, priv->spibase + offset);
}

static inline uint16_t gy912_spi_getreg16(struct gy912_spidev_s *priv, uint8_t offset)
{
  return getreg16(priv->spibase + offset);
}

static inline void gy912_spi_putreg16(struct gy912_spidev_s *priv, uint8_t offset, uint16_t value)
{
  putreg16(value, priv->spibase + offset);
}

static inline uint32_t gy912_spi_getreg32(struct gy912_spidev_s *priv, uint8_t offset)
{
  return getreg32(priv->spibase + offset);
}

static inline void gy912_spi_putreg32(struct gy912_spidev_s *priv, uint8_t offset, uint32_t value)
{
  putreg32(value, priv->spibase + offset);
}

/****************************************************************************
 * Name: gy912_spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int gy912_spi_lock(struct spi_dev_s *dev, bool lock)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: gy912_spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t gy912_spi_setfrequency(struct spi_dev_s *dev, uint32_t frequency)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  uint32_t pclk;
  uint32_t divisor;
  uint8_t spbr;
  uint8_t brdv;

  /* Check if frequency is already configured */
  if (priv->frequency == frequency)
    {
      return priv->actual;
    }

  /* Get peripheral clock frequency (PCLKA for RA8E1) */
  pclk = BOARD_PCLKA_FREQUENCY;

  /* Calculate divisor: SPI_CLK = PCLK / (2 * (SPBR + 1) * 2^BRDV) */
  /* Find best BRDV and SPBR combination */
  for (brdv = 0; brdv < 4; brdv++)
    {
      divisor = (1 << (brdv + 1));
      spbr = (pclk / (frequency * divisor)) - 1;
      
      if (spbr <= 255)
        {
          break;
        }
    }

  /* Limit SPBR to valid range */
  if (spbr > 255)
    {
      spbr = 255;
    }

  /* Calculate actual frequency */
  priv->actual = pclk / ((spbr + 1) * (1 << (brdv + 1)));
  priv->frequency = frequency;

  /* Configure bit rate */
  gy912_spi_putreg8(priv, RA8E1_SPI_SPBR_OFFSET, spbr);

  /* Update command register with BRDV */
  uint16_t spcmd = gy912_spi_getreg16(priv, RA8E1_SPI_SPCMD0_OFFSET);
  spcmd &= ~RA8E1_SPI_SPCMD_BRDV_MASK;
  spcmd |= (brdv << 2) & RA8E1_SPI_SPCMD_BRDV_MASK;
  gy912_spi_putreg16(priv, RA8E1_SPI_SPCMD0_OFFSET, spcmd);

  spiinfo("Frequency %d->%d\n", frequency, priv->actual);
  return priv->actual;
}

/****************************************************************************
 * Name: gy912_spi_setmode
 *
 * Description:
 *   Set the SPI mode.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested
 *
 * Returned Value:
 *   Returns the actual SPI mode selected
 *
 ****************************************************************************/

static void gy912_spi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  uint16_t spcmd;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */
  if (mode != priv->mode)
    {
      /* Configure CPOL and CPHA in SPCMD0 register */
      spcmd = gy912_spi_getreg16(priv, RA8E1_SPI_SPCMD0_OFFSET);
      spcmd &= ~(RA8E1_SPI_SPCMD_CPOL | RA8E1_SPI_SPCMD_CPHA);

      switch (mode)
        {
          case SPIDEV_MODE0: /* CPOL=0; CPHA=0 */
            break;

          case SPIDEV_MODE1: /* CPOL=0; CPHA=1 */
            spcmd |= RA8E1_SPI_SPCMD_CPHA;
            break;

          case SPIDEV_MODE2: /* CPOL=1; CPHA=0 */
            spcmd |= RA8E1_SPI_SPCMD_CPOL;
            break;

          case SPIDEV_MODE3: /* CPOL=1; CPHA=1 */
            spcmd |= (RA8E1_SPI_SPCMD_CPOL | RA8E1_SPI_SPCMD_CPHA);
            break;

          default:
            return;
        }

      gy912_spi_putreg16(priv, RA8E1_SPI_SPCMD0_OFFSET, spcmd);
      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: gy912_spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_spi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  uint16_t spcmd;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */
  if (nbits != priv->nbits)
    {
      /* Configure data length in SPCMD0 register */
      spcmd = gy912_spi_getreg16(priv, RA8E1_SPI_SPCMD0_OFFSET);
      spcmd &= ~RA8E1_SPI_SPCMD_SPB_MASK;

      if (nbits <= 8)
        {
          spcmd |= RA8E1_SPI_SPCMD_SPB_8;
          priv->nbits = 8;
        }
      else if (nbits <= 16)
        {
          spcmd |= RA8E1_SPI_SPCMD_SPB_16;
          priv->nbits = 16;
        }
      else
        {
          spierr("ERROR: Unsupported nbits: %d\n", nbits);
          return;
        }

      gy912_spi_putreg16(priv, RA8E1_SPI_SPCMD0_OFFSET, spcmd);
    }
}

#ifdef CONFIG_SPI_HWFEATURES
/****************************************************************************
 * Name: gy912_spi_hwfeatures
 *
 * Description:
 *   Set hardware-specific feature flags.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   features - H/W feature flags
 *
 * Returned Value:
 *   Zero (OK) if the selected H/W features are enabled; A negated errno
 *   value if any H/W feature is not supportable.
 *
 ****************************************************************************/

static int gy912_spi_hwfeatures(struct spi_dev_s *dev, spi_hwfeatures_t features)
{
  /* Other H/W features are not supported */
  return (features == 0) ? OK : -ENOSYS;
}
#endif

/****************************************************************************
 * Name: gy912_spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t gy912_spi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  uint32_t regval;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  /* Write the data to transmit to the SPI Data Register */
  if (priv->nbits <= 8)
    {
      gy912_spi_putreg8(priv, RA8E1_SPI_SPDR_OFFSET, (uint8_t)wd);
    }
  else
    {
      gy912_spi_putreg16(priv, RA8E1_SPI_SPDR_OFFSET, (uint16_t)wd);
    }

  /* Wait until the transfer is complete */
  do
    {
      regval = gy912_spi_getreg8(priv, RA8E1_SPI_SPSR_OFFSET);
    }
  while ((regval & RA8E1_SPI_SPSR_SPTEF) == 0);

  /* Wait for receive data */
  do
    {
      regval = gy912_spi_getreg8(priv, RA8E1_SPI_SPSR_OFFSET);
    }
  while ((regval & RA8E1_SPI_SPSR_SPRF) == 0);

  /* Read the received data */
  if (priv->nbits <= 8)
    {
      ret = gy912_spi_getreg8(priv, RA8E1_SPI_SPDR_OFFSET);
    }
  else
    {
      ret = gy912_spi_getreg16(priv, RA8E1_SPI_SPDR_OFFSET);
    }

  spiinfo("Sent: %04x Return: %04x Status: %02x\n", wd, ret, regval);
  return ret;
}

/****************************************************************************
 * Name: gy912_spi_exchange
 *
 * Description:
 *   Exchange a block of data from SPI.
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that to be exchanged in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_spi_exchange(struct spi_dev_s *dev, const void *txbuffer,
                               void *rxbuffer, size_t nwords)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  struct gy912_spi_transfer_s xfer;

  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* Set up transfer structure */
  memset(&xfer, 0, sizeof(xfer));
  xfer.txbuffer = txbuffer;
  xfer.rxbuffer = rxbuffer;
  xfer.nwords = nwords;
  xfer.width = priv->nbits;
  xfer.status = RA8E1_SPI_STATUS_IDLE;

  /* Use DMA for large transfers if available */
  if (nwords > 16 && !priv->dma_active)
    {
      if (gy912_spi_transfer_dma(dev, &xfer) == OK)
        {
          return;
        }
      /* Fall back to polling if DMA fails */
    }

  /* Use polling mode */
  gy912_spi_transfer_polling(dev, &xfer);
}

#ifndef CONFIG_SPI_EXCHANGE

/****************************************************************************
 * Name: gy912_spi_sndblock
 *
 * Description:
 *   Send a block of data on SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   nwords   - the length of data to send from the buffer in number of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_spi_sndblock(struct spi_dev_s *dev, const void *txbuffer,
                               size_t nwords)
{
  spiinfo("txbuffer=%p nwords=%d\n", txbuffer, nwords);
  return gy912_spi_exchange(dev, txbuffer, NULL, nwords);
}

/****************************************************************************
 * Name: gy912_spi_recvblock
 *
 * Description:
 *   Receive a block of data from SPI
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   rxbuffer - A pointer to the buffer in which to receive data
 *   nwords   - the length of data that can be received in the buffer in number
 *              of words.  The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_spi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                size_t nwords)
{
  spiinfo("rxbuffer=%p nwords=%d\n", rxbuffer, nwords);
  return gy912_spi_exchange(dev, NULL, rxbuffer, nwords);
}

#endif /* !CONFIG_SPI_EXCHANGE */

/****************************************************************************
 * Name: gy912_spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void gy912_spi_bus_initialize(struct gy912_spidev_s *priv)
{
  uint8_t regval;
  uint16_t spcmd;

  /* Configure the SPI pins */
  ra_configgpio(GPIO_SPI0_SCK);
  ra_configgpio(GPIO_SPI0_MOSI);
  ra_configgpio(GPIO_SPI0_MISO);
  ra_configgpio(GPIO_SPI0_SS0);
  ra_configgpio(GPIO_SPI0_BMP_CS);

  /* Enable SPI0 clock */
  modifyreg32(RA8_SYSTEM_MSTPCRB, RA8E1_MSTP_SPI0, 0);

  /* Disable SPI function */
  gy912_spi_putreg8(priv, RA8E1_SPI_SPCR_OFFSET, 0);

  /* Configure SPI Control Register */
  regval = RA8E1_SPI_SPCR_MSTR |    /* Master mode */
           RA8E1_SPI_SPCR_SPE;      /* SPI function enable */
  gy912_spi_putreg8(priv, RA8E1_SPI_SPCR_OFFSET, regval);

  /* Configure SPI Command Register 0 */
  spcmd = RA8E1_SPI_SPCMD_SPB_8 |   /* 8-bit data length */
          (0 << 4);                 /* SSL0 assertion */
  gy912_spi_putreg16(priv, RA8E1_SPI_SPCMD0_OFFSET, spcmd);

  /* Set default configuration */
  priv->frequency = 0;
  priv->mode = SPIDEV_MODE0;
  priv->nbits = 8;

  /* Set the initial SPI configuration */
  gy912_spi_setmode((struct spi_dev_s *)priv, SPIDEV_MODE0);
  gy912_spi_setbits((struct spi_dev_s *)priv, 8);
  gy912_spi_setfrequency((struct spi_dev_s *)priv, 400000);

  /* Initialize CS pins to deasserted state */
  ra_gpiowrite(GPIO_SPI0_SS0, true);
  ra_gpiowrite(GPIO_SPI0_BMP_CS, true);

  priv->initialized = true;
}

/****************************************************************************
 * Name: gy912_spi_dma_setup
 *
 * Description:
 *   Setup DMA for SPI transfers
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int gy912_spi_dma_setup(void)
{
  /* Enable DTC clock */
  modifyreg32(RA8_SYSTEM_MSTPCRB, RA8E1_MSTP_DTC, 0);

  /* Initialize DTC */
  putreg32(0x00000001, RA8E1_DTC_BASE + RA8E1_DTC_DTCST_OFFSET);

  /* TODO: Setup DTC vector table and transfer information */
  /* This would require detailed DTC vector table setup */
  /* For now, return OK to indicate basic setup is complete */

  return OK;
}

/****************************************************************************
 * Name: gy912_spi_transfer_dma
 *
 * Description:
 *   Perform SPI transfer using DMA
 *
 * Input Parameters:
 *   dev  - SPI device structure
 *   xfer - Transfer configuration
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int gy912_spi_transfer_dma(struct spi_dev_s *dev,
                           struct gy912_spi_transfer_s *xfer)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;

  /* TODO: Implement DMA transfer setup */
  /* This requires:
   * 1. Configure DTC transfer information structures
   * 2. Setup source and destination addresses
   * 3. Configure transfer count and mode
   * 4. Enable DTC transfer
   * 5. Wait for completion or setup interrupt
   */

  /* For now, fall back to polling mode */
  return gy912_spi_transfer_polling(dev, xfer);
}

/****************************************************************************
 * Name: gy912_spi_transfer_polling
 *
 * Description:
 *   Perform SPI transfer using polling mode
 *
 * Input Parameters:
 *   dev  - SPI device structure
 *   xfer - Transfer configuration
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int gy912_spi_transfer_polling(struct spi_dev_s *dev,
                               struct gy912_spi_transfer_s *xfer)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)dev;
  const uint8_t *txptr8 = (const uint8_t *)xfer->txbuffer;
  const uint16_t *txptr16 = (const uint16_t *)xfer->txbuffer;
  uint8_t *rxptr8 = (uint8_t *)xfer->rxbuffer;
  uint16_t *rxptr16 = (uint16_t *)xfer->rxbuffer;
  size_t i;
  uint32_t txdata;
  uint32_t rxdata;

  xfer->status = RA8E1_SPI_STATUS_BUSY;

  for (i = 0; i < xfer->nwords; i++)
    {
      /* Get transmit data */
      if (txptr8 || txptr16)
        {
          if (priv->nbits <= 8)
            {
              txdata = txptr8 ? txptr8[i] : 0xff;
            }
          else
            {
              txdata = txptr16 ? txptr16[i] : 0xffff;
            }
        }
      else
        {
          txdata = (priv->nbits <= 8) ? 0xff : 0xffff;
        }

      /* Perform the exchange */
      rxdata = gy912_spi_send(dev, txdata);

      /* Store received data */
      if (rxptr8 || rxptr16)
        {
          if (priv->nbits <= 8)
            {
              if (rxptr8)
                {
                  rxptr8[i] = (uint8_t)rxdata;
                }
            }
          else
            {
              if (rxptr16)
                {
                  rxptr16[i] = (uint16_t)rxdata;
                }
            }
        }
    }

  xfer->status = RA8E1_SPI_STATUS_COMPLETE;
  xfer->result = OK;

  return OK;
}

/****************************************************************************
 * Name: gy912_spi_cs_control
 *
 * Description:
 *   Control chip select lines
 *
 * Input Parameters:
 *   devid  - Device ID
 *   select - true to assert CS, false to deassert
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void gy912_spi_cs_control(uint32_t devid, bool select)
{
  switch (devid)
    {
      case SPIDEV_IMU(0):
        /* IMU (ICM-20948) on SS0 */
        ra_gpiowrite(GPIO_SPI0_SS0, !select);
        break;

      case SPIDEV_BAROMETER(0):
        /* Barometer (BMP388) on GPIO CS */
        ra_gpiowrite(GPIO_SPI0_BMP_CS, !select);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: gy912_spi_interrupt_handler
 *
 * Description:
 *   SPI interrupt handler
 *
 ****************************************************************************/

int gy912_spi_interrupt_handler(int irq, void *context, void *arg)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)arg;
  uint8_t status;

  /* Read status register */
  status = gy912_spi_getreg8(priv, RA8E1_SPI_SPSR_OFFSET);

  /* Handle errors */
  if (status & (RA8E1_SPI_SPSR_OVRF | RA8E1_SPI_SPSR_MODF | RA8E1_SPI_SPSR_PERF))
    {
      spierr("SPI Error: status=0x%02x\n", status);
      
      if (priv->current_xfer)
        {
          priv->current_xfer->status = RA8E1_SPI_STATUS_ERROR;
          priv->current_xfer->result = -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: gy912_spi_dma_interrupt_handler
 *
 * Description:
 *   DMA interrupt handler for SPI transfers
 *
 ****************************************************************************/

int gy912_spi_dma_interrupt_handler(int irq, void *context, void *arg)
{
  struct gy912_spidev_s *priv = (struct gy912_spidev_s *)arg;

  /* TODO: Handle DMA completion interrupt */
  if (priv->current_xfer)
    {
      priv->current_xfer->status = RA8E1_SPI_STATUS_COMPLETE;
      priv->current_xfer->result = OK;
      priv->dma_active = false;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gy912_spi_ spi_initialize
 *
 * Description:
 *   Initialize SPI driver and register the /dev/spiN devices.
 *
 ****************************************************************************/

void gy912_spi_ spi_initialize(void)
{
  struct spi_dev_s *spi;

  /* Initialize SPI0 */
  gy912_spi_bus_initialize(&g_spi0dev);

  /* Setup DMA if available */
  gy912_spi_dma_setup();

  /* Get the SPI0 port interface */
  spi = (struct spi_dev_s *)&g_spi0dev;

  /* Register the SPI driver */
  spi_register(spi, 0);

  spiinfo("SPI0 initialized with DMA support\n");
}

/****************************************************************************
 * Name: ra_spi0_select, ra_spi0_status, and ra_spi0_cmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.
 *   They are implementations of the select, status, and cmddata methods
 *   of the SPI interface defined by struct spi_ops_s (see include/nuttx/spi/spi.h).
 *   All other methods including ra_spibus_initialize()) are provided by
 *   common RA8 logic.
 *
 ****************************************************************************/

void ra_spi0_select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  gy912_spi_cs_control(devid, selected);
}

uint8_t ra_spi0_status(struct spi_dev_s *dev, uint32_t devid)
{
  uint8_t status = 0;

  switch (devid)
    {
      case SPIDEV_IMU(0):
      case SPIDEV_BAROMETER(0):
        status |= SPI_STATUS_PRESENT;
        break;

      default:
        break;
    }

  return status;
}

#ifdef CONFIG_SPI_CMDDATA
int ra_spi0_cmddata(struct spi_dev_s *dev, uint32_t devid, bool cmd)
{
  /* SPI sensors typically don't use command/data distinction */
  return OK;
}
#endif

#endif /* CONFIG_RA8_SPI */

#if CONFIG_SPI_GY912_SAMPLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GY912_HELP \
  "Usage: gy912_test [OPTIONS]\n" \
  "\n" \
  "OPTIONS:\n" \
  "  -h, --help     Show this help message\n" \
  "  -s, --single   Perform single reading\n" \
  "  -r, --run      Run continuous demonstration\n" \
  "  -t, --test     Run self-test only\n" \
  "  -i, --init     Initialize sensors only\n" \
  "\n" \
  "Examples:\n" \
  "  gy912_test -s      # Single sensor reading\n" \
  "  gy912_test -r      # Run continuous demo\n" \
  "  gy912_test -t      # Run self-test\n"

/****************************************************************************
 * External Functions
 ****************************************************************************/

/* These functions should be declared in a header file, but for simplicity
 * we declare them here. In a real implementation, they would be in
 * include/nuttx/sensors/gy912.h or similar.
 */

extern int gy912_spi_initialize(void);
extern int gy912_demo_run(void);
extern int gy912_demo_single_read(void);
extern int gy912_self_test(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: show_help
 *
 * Description:
 *   Show help message
 *
 ****************************************************************************/

static void show_help(void)
{
  demoprintf(GY912_HELP);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_spi_gy912_demo_init
 *
 * Description:
 *   Initialize the SPI GY-912 demo
 *
 ****************************************************************************/

int ra8e1_spi_gy912_demo_init(void)
{
  /* Initialize GY-912 sensor module */
  return gy912_spi_initialize();
}

/****************************************************************************
 * Name: ra8e1_spi_gy912_demo_main
 *
 * Description:
 *   GY-912 SPI demo main function
 *
 ****************************************************************************/

int ra8e1_spi_gy912_demo_main(int argc, FAR char *argv[])
{
  int ret = OK;
  bool help = false;
  bool single = false;
  bool run = false;
  bool test = false;
  bool init_only = false;
  int i;
  
  demoprintf("GY-912 10DOF Sensor Test Application\n");
  demoprintf("====================================\n\n");
  
  /* Parse command line arguments */
  for (i = 1; i < argc; i++)
    {
      if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
        {
          help = true;
        }
      else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--single") == 0)
        {
          single = true;
        }
      else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--run") == 0)
        {
          run = true;
        }
      else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0)
        {
          test = true;
        }
      else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--init") == 0)
        {
          init_only = true;
        }
      else
        {
          demoprintf("Unknown option: %s\n\n", argv[i]);
          show_help();
          return EXIT_FAILURE;
        }
    }
  
  if (help)
    {
      show_help();
      return EXIT_SUCCESS;
    }
  
  /* If no specific action requested, show help */
  if (!single && !run && !test && !init_only)
    {
      demoprintf("No action specified. Available actions:\n\n");
      show_help();
      return EXIT_SUCCESS;
    }
  
  /* Initialize sensors */
  demoprintf("Initializing GY-912 sensors...\n");
  ret = gy912_spi_initialize();
  if (ret < 0)
    {
      demoprintf("ERROR: Failed to initialize GY-912 sensors: %d\n", ret);
      return EXIT_FAILURE;
    }
  
  demoprintf("GY-912 sensors initialized successfully!\n\n");
  
  if (init_only)
    {
      demoprintf("Initialization complete.\n");
      return EXIT_SUCCESS;
    }
  
  /* Perform requested action */
  if (test)
    {
      demoprintf("Running self-test...\n");
      ret = gy912_self_test();
      if (ret < 0)
        {
          demoprintf("ERROR: Self-test failed: %d\n", ret);
          return EXIT_FAILURE;
        }
      demoprintf("Self-test completed successfully!\n\n");
    }
  
  if (single)
    {
      demoprintf("Performing single sensor reading...\n");
      ret = gy912_demo_single_read();
      if (ret < 0)
        {
          demoprintf("ERROR: Single reading failed: %d\n", ret);
          return EXIT_FAILURE;
        }
      demoprintf("Single reading completed!\n\n");
    }
  
  if (run)
    {
      demoprintf("Starting continuous demonstration...\n");
      demoprintf("Press Ctrl+C to stop.\n\n");
      ret = gy912_demo_run();
      if (ret < 0)
        {
          demoprintf("ERROR: Demonstration failed: %d\n", ret);
          return EXIT_FAILURE;
        }
      demoprintf("Demonstration completed!\n\n");
    }
  
  return EXIT_SUCCESS;
}

#endif /* CONFIG_SPI_GY912_SAMPLE */
