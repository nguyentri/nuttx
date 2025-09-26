/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_spi.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_RA_SPI_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_RA_SPI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RA8E1 SPI Register Offsets */
#define RA_SPI_SPDR_OFFSET       0x00    /* SPI Data Register */
#define RA_SPI_SPDECR_OFFSET     0x04    /* SPI Delay Control Register */
#define RA_SPI_SPCR_OFFSET       0x08    /* SPI Control Register */
#define RA_SPI_SPCR2_OFFSET      0x0C    /* SPI Control Register 2 */
#define RA_SPI_SPCR3_OFFSET      0x10    /* SPI Control Register 3 */
#define RA_SPI_SPCMD0_OFFSET     0x14    /* SPI Command Register 0 */
#define RA_SPI_SPCMD1_OFFSET     0x18    /* SPI Command Register 1 */
#define RA_SPI_SPCMD2_OFFSET     0x1C    /* SPI Command Register 2 */
#define RA_SPI_SPCMD3_OFFSET     0x20    /* SPI Command Register 3 */
#define RA_SPI_SPCMD4_OFFSET     0x24    /* SPI Command Register 4 */
#define RA_SPI_SPCMD5_OFFSET     0x28    /* SPI Command Register 5 */
#define RA_SPI_SPCMD6_OFFSET     0x2C    /* SPI Command Register 6 */
#define RA_SPI_SPCMD7_OFFSET     0x30    /* SPI Command Register 7 */
#define RA_SPI_SPDCR_OFFSET      0x40    /* SPI Data Control Register */
#define RA_SPI_SPDCR2_OFFSET     0x44    /* SPI Data Control Register 2 */
#define RA_SPI_SPSR_OFFSET       0x50    /* SPI Status Register */
#define RA_SPI_SPTFSR_OFFSET     0x58    /* SPI Transfer FIFO Status Register */
#define RA_SPI_SPRFSR_OFFSET     0x5C    /* SPI Receive FIFO Status Register */
#define RA_SPI_SPPSR_OFFSET      0x60    /* SPI Polling Register */
#define RA_SPI_SPSRC_OFFSET      0x68    /* SPI Status Clear Register */
/* SPI FIFO Clear Register (SPFCR) - RSPI FIFO Clear Register */
#define RA_SPI_SPFCR_OFFSET      0x6c    /* SPI FIFO Clear Register */

/* Legacy register offset aliases for backward compatibility */
#define RA_SPI_SSLP_OFFSET       RA_SPI_SPCR3_OFFSET  /* SSL polarity is in SPCR3 */
#define RA_SPI_SPSCR_OFFSET      RA_SPI_SPCR3_OFFSET  /* Sequence control is in SPCR3 */
#define RA_SPI_SPBR_OFFSET       RA_SPI_SPCR3_OFFSET  /* Bit rate is in SPCR3 */
#define RA_SPI_SPCKD_OFFSET      RA_SPI_SPDECR_OFFSET /* Clock delay is in SPDECR */
#define RA_SPI_SSLND_OFFSET      RA_SPI_SPDECR_OFFSET /* SSL negation delay is in SPDECR */
#define RA_SPI_SPND_OFFSET       RA_SPI_SPDECR_OFFSET /* Next access delay is in SPDECR */

/* SPI Delay Control Register (SPDECR) bit fields */
#define RA_SPI_SPDECR_SCKDL_SHIFT  0       /* RSPCK Delay Setting */
#define RA_SPI_SPDECR_SCKDL_MASK   (0x07 << RA_SPI_SPDECR_SCKDL_SHIFT)
#define RA_SPI_SPDECR_SLNDL_SHIFT  8       /* SSL Negation Delay Setting */
#define RA_SPI_SPDECR_SLNDL_MASK   (0x07 << RA_SPI_SPDECR_SLNDL_SHIFT)
#define RA_SPI_SPDECR_SPNDL_SHIFT  16      /* SPI Next-Access Delay Setting */
#define RA_SPI_SPDECR_SPNDL_MASK   (0x07 << RA_SPI_SPDECR_SPNDL_SHIFT)

/* SPI Control Register 3 (SPCR3) bit fields */
#define RA_SPI_SPCR3_SSL0P         (1 << 0)  /* SSL0 Signal Polarity */
#define RA_SPI_SPCR3_SSL1P         (1 << 1)  /* SSL1 Signal Polarity */
#define RA_SPI_SPCR3_SSL2P         (1 << 2)  /* SSL2 Signal Polarity */
#define RA_SPI_SPCR3_SSL3P         (1 << 3)  /* SSL3 Signal Polarity */
#define RA_SPI_SPCR3_SPBR_SHIFT    8         /* SPI Bit Rate */
#define RA_SPI_SPCR3_SPBR_MASK     (0xFF << RA_SPI_SPCR3_SPBR_SHIFT)
#define RA_SPI_SPCR3_SPSLN_SHIFT   24        /* SPI Sequence Length */
#define RA_SPI_SPCR3_SPSLN_MASK    (0x07 << RA_SPI_SPCR3_SPSLN_SHIFT)

/* SPI Control Register (SPCR) */
#define RA_SPI_SPCR_SPE          (1 << 0)  /* SPI Function Enable */
#define RA_SPI_SPCR_SPPE         (1 << 8)  /* Parity Enable */
#define RA_SPI_SPCR_SPOE         (1 << 9)  /* Parity Mode */
#define RA_SPI_SPCR_PTE          (1 << 11) /* Parity Self-Diagnosis Enable */
#define RA_SPI_SPCR_SCKASE       (1 << 12) /* RSPCK Auto-Stop Function Enable */
#define RA_SPI_SPCR_BFDS         (1 << 13) /* Between Burst Transfer Frames Delay Select */
#define RA_SPI_SPCR_MODFEN       (1 << 14) /* Mode Fault Error Detection Enable */
#define RA_SPI_SPCR_SPEIE        (1 << 16) /* SPI Error Interrupt Enable */
#define RA_SPI_SPCR_SPRIE        (1 << 17) /* SPI Receive Buffer Full Interrupt Enable */
#define RA_SPI_SPCR_SPIIE        (1 << 18) /* SPI Idle Interrupt Enable */
#define RA_SPI_SPCR_SPDRES       (1 << 19) /* SPI receive data ready error select */
#define RA_SPI_SPCR_SPTIE        (1 << 20) /* SPI Transmit Buffer Empty Interrupt Enable */
#define RA_SPI_SPCR_CENDIE       (1 << 21) /* SPI Communication End Interrupt Enable */
#define RA_SPI_SPCR_SPMS         (1UL << 24) /* SPI Mode Select */
#define RA_SPI_SPCR_SPFRF        (1UL << 25) /* SPI Frame Format Select */
#define RA_SPI_SPCR_TXMD_SHIFT   28        /* Communication Mode Select bit shift */
#define RA_SPI_SPCR_TXMD_MASK    (3UL << RA_SPI_SPCR_TXMD_SHIFT)
#define RA_SPI_SPCR_TXMD_TX_RX   (0UL << RA_SPI_SPCR_TXMD_SHIFT)
#define RA_SPI_SPCR_TXMD_TX_ONLY (1UL << RA_SPI_SPCR_TXMD_SHIFT)
#define RA_SPI_SPCR_TXMD_RX_ONLY (2UL << RA_SPI_SPCR_TXMD_SHIFT)
#define RA_SPI_SPCR_MSTR         (1UL << 30) /* SPI Master/Slave Mode Select */
#define RA_SPI_SPCR_BPEN         (1UL << 31) /* Synchronization Circuit Bypass Enable */

/* SPI Slave Select Polarity Register (SSLP) */
#define RA_SPI_SSLP_SSL0P        (1 << 0)  /* SSL0 Signal Polarity Setting */
#define RA_SPI_SSLP_SSL1P        (1 << 1)  /* SSL1 Signal Polarity Setting */
#define RA_SPI_SSLP_SSL2P        (1 << 2)  /* SSL2 Signal Polarity Setting */
#define RA_SPI_SSLP_SSL3P        (1 << 3)  /* SSL3 Signal Polarity Setting */

/* SPI Status Register (SPSR) */
#define RA_SPI_SPSR_SPCP_SHIFT   8         /* SPI Command Pointer */
#define RA_SPI_SPSR_SPCP_MASK    (0x7 << RA_SPI_SPSR_SPCP_SHIFT)
#define RA_SPI_SPSR_SPECM_SHIFT  12        /* SPI Error Command */
#define RA_SPI_SPSR_SPECM_MASK   (0x7 << RA_SPI_SPSR_SPECM_SHIFT)
#define RA_SPI_SPSR_SPDRF        (1 << 23) /* SPI Receive Data Ready Flag */
#define RA_SPI_SPSR_OVRF         (1 << 24) /* Overrun Error Flag */
#define RA_SPI_SPSR_IDLNF        (1 << 25) /* SPI Idle Flag */
#define RA_SPI_SPSR_MODF         (1 << 26) /* Mode Fault Error Flag */
#define RA_SPI_SPSR_PERF         (1 << 27) /* Parity Error Flag */
#define RA_SPI_SPSR_UDRF         (1 << 28) /* Underrun Error Flag */
#define RA_SPI_SPSR_SPTEF        (1 << 29) /* SPI Transmit Buffer Empty Flag */
#define RA_SPI_SPSR_CENDF        (1 << 30) /* Communication End Flag */
#define RA_SPI_SPSR_SPRF         (1 << 31) /* SPI Receive Buffer Full Flag */

/* SPI Status Clear Register (SPSRC) - Write 1 to clear corresponding SPSR flag */
#define RA_SPI_SPSRC_SPDRFC      (1 << 23) /* Clear SPI Receive Data Ready Flag */
#define RA_SPI_SPSRC_OVRFC       (1 << 24) /* Clear Overrun Error Flag */
#define RA_SPI_SPSRC_MODFC       (1 << 26) /* Clear Mode Fault Error Flag */
#define RA_SPI_SPSRC_PERFC       (1 << 27) /* Clear Parity Error Flag */
#define RA_SPI_SPSRC_UDRFC       (1 << 28) /* Clear Underrun Error Flag */
#define RA_SPI_SPSRC_SPTEFC      (1 << 29) /* Clear SPI Transmit Buffer Empty Flag */
#define RA_SPI_SPSRC_CENDFC      (1 << 30) /* Clear Communication End Flag */
#define RA_SPI_SPSRC_SPRFC       (1 << 31) /* Clear SPI Receive Buffer Full Flag */
#define RA_SPI_SPSRC_ALL_CLEAR   (0xFD800000) /* All flags that can be cleared */

/* SPI Data Control Register (SPDCR) */
#define RA_SPI_SPDCR_BYSW        (1 << 0)  /* Byte Swap Operating Mode Select */
#define RA_SPI_SPDCR_SPRDTD      (1 << 3)  /* SPI Receive/Transmit Data Select */
#define RA_SPI_SPDCR_SINV        (1 << 4)  /* Serial data invert bit */
#define RA_SPI_SPDCR_SPFC_SHIFT  (8)       /* Frame Count */
#define RA_SPI_SPDCR_SPFC_MASK   (0x03 << RA_SPI_SPDCR_SPFC_SHIFT)
#define RA_SPI_SPDCR_SPFC_1      (0x00 << RA_SPI_SPDCR_SPFC_SHIFT)
#define RA_SPI_SPDCR_SPFC_2      (0x01 << RA_SPI_SPDCR_SPFC_SHIFT)
#define RA_SPI_SPDCR_SPFC_3      (0x02 << RA_SPI_SPDCR_SPFC_SHIFT)
#define RA_SPI_SPDCR_SPFC_4      (0x03 << RA_SPI_SPDCR_SPFC_SHIFT)

/* SPI Control Register 2 (SPCR2) - pin control and master-receive-only bits */
#define RA_SPI_SPCR2_RMFM_SHIFT   (0)       /* Frame processing count (RMFM) LSB */
#define RA_SPI_SPCR2_RMFM_MASK    (0x1FUL << RA_SPI_SPCR2_RMFM_SHIFT)
#define RA_SPI_SPCR2_RMEDTG       (1 << 6)  /* End Trigger in Master Receive only (RMEDTG) */
#define RA_SPI_SPCR2_RMSTTG       (1 << 7)  /* Start Trigger in Master Receive only (RMSTTG) */
#define RA_SPI_SPCR2_SPDRC_SHIFT  (8)       /* SPI received data ready detect adjustment */
#define RA_SPI_SPCR2_SPDRC_MASK   (0xFF << RA_SPI_SPCR2_SPDRC_SHIFT)
#define RA_SPI_SPCR2_SPLP         (1 << 16) /* SPI Loopback */
#define RA_SPI_SPCR2_SPLP2        (1 << 17) /* SPI Loopback 2 */
#define RA_SPI_SPCR2_MOIFV        (1 << 20) /* MOSI Idle Fixed Value */
#define RA_SPI_SPCR2_MOIFE        (1 << 21) /* MOSI Idle Value Fixing Enable */

/* SPI Data Control Register 2 (SPDCR2) - FIFO threshold settings */
#define RA_SPI_SPDCR2_RTRG_SHIFT  (0)       /* Receive FIFO threshold setting */
#define RA_SPI_SPDCR2_RTRG_MASK   (0x03 << RA_SPI_SPDCR2_RTRG_SHIFT)
#define RA_SPI_SPDCR2_TTRG_SHIFT  (8)       /* Transmission FIFO threshold setting */
#define RA_SPI_SPDCR2_TTRG_MASK   (0x03 << RA_SPI_SPDCR2_TTRG_SHIFT)

/* SPI Control Register 3 (SPCR3) - SSL polarity, bit rate, sequence length */
#define RA_SPI_SPCR3_SSL0P       (1 << 0)  /* SSL0 Signal Polarity */
#define RA_SPI_SPCR3_SSL1P       (1 << 1)  /* SSL1 Signal Polarity */
#define RA_SPI_SPCR3_SSL2P       (1 << 2)  /* SSL2 Signal Polarity */
#define RA_SPI_SPCR3_SSL3P       (1 << 3)  /* SSL3 Signal Polarity */

/* SPI Command Register (SPCMD0-7) */
#define RA_SPI_SPCMD_CPHA        (1 << 0)  /* RSPCK Phase Setting */
#define RA_SPI_SPCMD_CPOL        (1 << 1)  /* RSPCK Polarity Setting */
#define RA_SPI_SPCMD_BRDV_SHIFT  (2)       /* Bit Rate Division Setting */
#define RA_SPI_SPCMD_BRDV_MASK   (0x03 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_1      (0x00 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_2      (0x01 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_4      (0x02 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_8      (0x03 << RA_SPI_SPCMD_BRDV_SHIFT)

#define RA_SPI_SPCMD_SSLKP       (1 << 7)  /* SSL Signal Level Keeping */

#define RA_SPI_SPCMD_LSBF        (1 << 12) /* SPI LSB First */
#define RA_SPI_SPCMD_SPNDEN      (1 << 13) /* SPI Next-Access Delay Enable */
#define RA_SPI_SPCMD_SLNDEN      (1 << 14) /* SSL Negation Delay Setting Enable */
#define RA_SPI_SPCMD_SCKDEN      (1 << 15) /* RSPCK Delay Setting Enable */

#define RA_SPI_SPCMD_SPB_SHIFT   (16)      /* SPI Data Length Setting */
#define RA_SPI_SPCMD_SPB_MASK    (0x1FUL << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_4       (0x03UL << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_8       (0x07UL << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_16      (0x0FUL << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_20      (0x13UL << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_24      (0x17UL << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_32      (0x1FUL << RA_SPI_SPCMD_SPB_SHIFT)

#define RA_SPI_SPCMD_SSLA_SHIFT  (24)      /* SSL Signal Assertion Setting */
#define RA_SPI_SPCMD_SSLA_MASK   (0x07UL << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_0      (0x00UL << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_1      (0x01UL << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_2      (0x02UL << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_3      (0x03UL << RA_SPI_SPCMD_SSLA_SHIFT)

/* Register Addresses */
#define RA_SPI_SPDR(n)           (RA_SPI_BASE(n) + RA_SPI_SPDR_OFFSET)
#define RA_SPI_SPDECR(n)         (RA_SPI_BASE(n) + RA_SPI_SPDECR_OFFSET)
#define RA_SPI_SPCR(n)           (RA_SPI_BASE(n) + RA_SPI_SPCR_OFFSET)
#define RA_SPI_SPCR2(n)          (RA_SPI_BASE(n) + RA_SPI_SPCR2_OFFSET)
#define RA_SPI_SPCR3(n)          (RA_SPI_BASE(n) + RA_SPI_SPCR3_OFFSET)
#define RA_SPI_SPCMD0(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD0_OFFSET)
#define RA_SPI_SPCMD1(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD1_OFFSET)
#define RA_SPI_SPCMD2(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD2_OFFSET)
#define RA_SPI_SPCMD3(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD3_OFFSET)
#define RA_SPI_SPCMD4(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD4_OFFSET)
#define RA_SPI_SPCMD5(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD5_OFFSET)
#define RA_SPI_SPCMD6(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD6_OFFSET)
#define RA_SPI_SPCMD7(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD7_OFFSET)
#define RA_SPI_SPDCR(n)          (RA_SPI_BASE(n) + RA_SPI_SPDCR_OFFSET)
#define RA_SPI_SPDCR2(n)         (RA_SPI_BASE(n) + RA_SPI_SPDCR2_OFFSET)
#define RA_SPI_SPSR(n)           (RA_SPI_BASE(n) + RA_SPI_SPSR_OFFSET)
#define RA_SPI_SPTFSR(n)         (RA_SPI_BASE(n) + RA_SPI_SPTFSR_OFFSET)
#define RA_SPI_SPRFSR(n)         (RA_SPI_BASE(n) + RA_SPI_SPRFSR_OFFSET)
#define RA_SPI_SPPSR(n)          (RA_SPI_BASE(n) + RA_SPI_SPPSR_OFFSET)
#define RA_SPI_SPSRC(n)          (RA_SPI_BASE(n) + RA_SPI_SPSRC_OFFSET)
/* SPFCR register address macro and bit */
#define RA_SPI_SPFCR(n)          (RA_SPI_BASE(n) + RA_SPI_SPFCR_OFFSET)
#define RA_SPI_SPFCR_SPFRST      (1 << 0)  /* FIFO reset bit */

/* SPI Base Addresses */
#ifdef CONFIG_RA_SPI0
#define RA_SPI0_BASE             R_SPI0_BASE
#endif
#ifdef CONFIG_RA_SPI1
#define RA_SPI1_BASE             R_SPI1_BASE
#endif

/* Helper macros */
#define RA_SPI_BASE(n)           ((n == 0) ? R_SPI0_BASE : R_SPI1_BASE)

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_RA_SPI_H */
