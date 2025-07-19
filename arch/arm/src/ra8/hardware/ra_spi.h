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
#define RA_SPI_SPCR_OFFSET       0x00    /* SPI Control Register */
#define RA_SPI_SSLP_OFFSET       0x01    /* SPI Slave Select Polarity Register */
#define RA_SPI_SPPCR_OFFSET      0x02    /* SPI Pin Control Register */
#define RA_SPI_SPSR_OFFSET       0x03    /* SPI Status Register */
#define RA_SPI_SPDR_OFFSET       0x04    /* SPI Data Register (32-bit) */
#define RA_SPI_SPSCR_OFFSET      0x08    /* SPI Sequence Control Register */
#define RA_SPI_SPSSR_OFFSET      0x09    /* SPI Sequence Status Register */
#define RA_SPI_SPBR_OFFSET       0x0A    /* SPI Bit Rate Register */
#define RA_SPI_SPDCR_OFFSET      0x0B    /* SPI Data Control Register */
#define RA_SPI_SPCKD_OFFSET      0x0C    /* SPI Clock Delay Register */
#define RA_SPI_SSLND_OFFSET      0x0D    /* SPI Slave Select Negation Delay Register */
#define RA_SPI_SPND_OFFSET       0x0E    /* SPI Next-Access Delay Register */
#define RA_SPI_SPCR2_OFFSET      0x0F    /* SPI Control Register 2 */
#define RA_SPI_SPCMD0_OFFSET     0x10    /* SPI Command Register 0 */
#define RA_SPI_SPCMD1_OFFSET     0x12    /* SPI Command Register 1 */
#define RA_SPI_SPCMD2_OFFSET     0x14    /* SPI Command Register 2 */
#define RA_SPI_SPCMD3_OFFSET     0x16    /* SPI Command Register 3 */
#define RA_SPI_SPCMD4_OFFSET     0x18    /* SPI Command Register 4 */
#define RA_SPI_SPCMD5_OFFSET     0x1A    /* SPI Command Register 5 */
#define RA_SPI_SPCMD6_OFFSET     0x1C    /* SPI Command Register 6 */
#define RA_SPI_SPCMD7_OFFSET     0x1E    /* SPI Command Register 7 */
#define RA_SPI_SPDCR2_OFFSET     0x20    /* SPI Data Control Register 2 */

/* SPI Control Register (SPCR) */
#define RA_SPI_SPCR_SPRIE        (1 << 7)  /* SPI Receive Interrupt Enable */
#define RA_SPI_SPCR_SPE          (1 << 6)  /* SPI Function Enable */
#define RA_SPI_SPCR_SPTIE        (1 << 5)  /* SPI Transmit Interrupt Enable */
#define RA_SPI_SPCR_SPEIE        (1 << 4)  /* SPI Error Interrupt Enable */
#define RA_SPI_SPCR_MSTR         (1 << 3)  /* SPI Master/Slave Mode Select */
#define RA_SPI_SPCR_MODFEN       (1 << 2)  /* Mode Fault Error Detection Enable */
#define RA_SPI_SPCR_TXMD         (1 << 1)  /* Communications Operating Mode Select */
#define RA_SPI_SPCR_SPMS         (1 << 0)  /* SPI Mode Select */

/* SPI Slave Select Polarity Register (SSLP) */
#define RA_SPI_SSLP_SSL0P        (1 << 0)  /* SSL0 Signal Polarity Setting */
#define RA_SPI_SSLP_SSL1P        (1 << 1)  /* SSL1 Signal Polarity Setting */
#define RA_SPI_SSLP_SSL2P        (1 << 2)  /* SSL2 Signal Polarity Setting */
#define RA_SPI_SSLP_SSL3P        (1 << 3)  /* SSL3 Signal Polarity Setting */

/* SPI Pin Control Register (SPPCR) */
#define RA_SPI_SPPCR_SPLP        (1 << 0)  /* SPI Loopback */
#define RA_SPI_SPPCR_SPLP2       (1 << 1)  /* SPI Loopback 2 */
#define RA_SPI_SPPCR_MOIFV       (1 << 4)  /* MOSI Idle Fixed Value */
#define RA_SPI_SPPCR_MOIFE       (1 << 5)  /* MOSI Idle Value Fixing Enable */

/* SPI Status Register (SPSR) */
#define RA_SPI_SPSR_OVRF         (1 << 0)  /* Overrun Error Flag */
#define RA_SPI_SPSR_IDLNF        (1 << 1)  /* SPI Idle Flag */
#define RA_SPI_SPSR_MODF         (1 << 2)  /* Mode Fault Error Flag */
#define RA_SPI_SPSR_PERF         (1 << 3)  /* Parity Error Flag */
#define RA_SPI_SPSR_UDRF         (1 << 4)  /* Underrun Error Flag */
#define RA_SPI_SPSR_SPTEF        (1 << 5)  /* SPI Transmit Buffer Empty Flag */
#define RA_SPI_SPSR_CENDF        (1 << 6)  /* Communication End Flag */
#define RA_SPI_SPSR_SPRF         (1 << 7)  /* SPI Receive Buffer Full Flag */

/* SPI Sequence Control Register (SPSCR) */
#define RA_SPI_SPSCR_SPSLN_SHIFT (0)       /* SPI Sequence Length Setting */
#define RA_SPI_SPSCR_SPSLN_MASK  (0x07 << RA_SPI_SPSCR_SPSLN_SHIFT)

/* SPI Bit Rate Register (SPBR) */
#define RA_SPI_SPBR_MASK         (0xFF)    /* Bit Rate Setting */

/* SPI Data Control Register (SPDCR) */
#define RA_SPI_SPDCR_SPLW        (1 << 5)  /* SPI Word Access/Halfword Access Specification */
#define RA_SPI_SPDCR_SPRDTD      (1 << 4)  /* SPI Receive/Transmit Data Select */
#define RA_SPI_SPDCR_SPLW_SHIFT  (5)
#define RA_SPI_SPDCR_SPLW_MASK   (1 << RA_SPI_SPDCR_SPLW_SHIFT)

/* SPI Clock Delay Register (SPCKD) */
#define RA_SPI_SPCKD_SCKDL_SHIFT (0)       /* RSPCK Delay Setting */
#define RA_SPI_SPCKD_SCKDL_MASK  (0x07 << RA_SPI_SPCKD_SCKDL_SHIFT)

/* SPI Control Register 2 (SPCR2) */
#define RA_SPI_SPCR2_SPPE        (1 << 0)  /* Parity Enable */
#define RA_SPI_SPCR2_SPOE        (1 << 1)  /* Odd Parity */
#define RA_SPI_SPCR2_SPIIE       (1 << 2)  /* SPI Idle Interrupt Enable */
#define RA_SPI_SPCR2_PTE         (1 << 3)  /* Parity Self-Testing */
#define RA_SPI_SPCR2_SCKASE      (1 << 4)  /* RSPCK Auto-Stop Function Enable */

/* SPI Command Register (SPCMD0-7) */
#define RA_SPI_SPCMD_CPHA        (1 << 0)  /* RSPCK Phase Setting */
#define RA_SPI_SPCMD_CPOL        (1 << 1)  /* RSPCK Polarity Setting */
#define RA_SPI_SPCMD_BRDV_SHIFT  (2)       /* Bit Rate Division Setting */
#define RA_SPI_SPCMD_BRDV_MASK   (0x03 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_1      (0x00 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_2      (0x01 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_4      (0x02 << RA_SPI_SPCMD_BRDV_SHIFT)
#define RA_SPI_SPCMD_BRDV_8      (0x03 << RA_SPI_SPCMD_BRDV_SHIFT)

#define RA_SPI_SPCMD_SSLA_SHIFT  (4)       /* SSL Signal Assertion Setting */
#define RA_SPI_SPCMD_SSLA_MASK   (0x07 << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_0      (0x00 << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_1      (0x01 << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_2      (0x02 << RA_SPI_SPCMD_SSLA_SHIFT)
#define RA_SPI_SPCMD_SSLA_3      (0x03 << RA_SPI_SPCMD_SSLA_SHIFT)

#define RA_SPI_SPCMD_SSLKP       (1 << 7)  /* SSL Signal Level Keeping */

#define RA_SPI_SPCMD_SPB_SHIFT   (8)       /* SPI Data Length Setting */
#define RA_SPI_SPCMD_SPB_MASK    (0x0F << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_4       (0x03 << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_8       (0x07 << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_16      (0x0F << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_20      (0x00 << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_24      (0x01 << RA_SPI_SPCMD_SPB_SHIFT)
#define RA_SPI_SPCMD_SPB_32      (0x02 << RA_SPI_SPCMD_SPB_SHIFT)

#define RA_SPI_SPCMD_LSBF        (1 << 12) /* SPI LSB First */
#define RA_SPI_SPCMD_SPNDEN      (1 << 13) /* SPI Next-Access Delay Enable */
#define RA_SPI_SPCMD_SLNDEN      (1 << 14) /* SSL Negation Delay Setting Enable */
#define RA_SPI_SPCMD_SCKDEN      (1 << 15) /* RSPCK Delay Setting Enable */

/* Register Addresses */
#define RA_SPI_SPCR(n)           (RA_SPI_BASE(n) + RA_SPI_SPCR_OFFSET)
#define RA_SPI_SSLP(n)           (RA_SPI_BASE(n) + RA_SPI_SSLP_OFFSET)
#define RA_SPI_SPPCR(n)          (RA_SPI_BASE(n) + RA_SPI_SPPCR_OFFSET)
#define RA_SPI_SPSR(n)           (RA_SPI_BASE(n) + RA_SPI_SPSR_OFFSET)
#define RA_SPI_SPDR(n)           (RA_SPI_BASE(n) + RA_SPI_SPDR_OFFSET)
#define RA_SPI_SPSCR(n)          (RA_SPI_BASE(n) + RA_SPI_SPSCR_OFFSET)
#define RA_SPI_SPSSR(n)          (RA_SPI_BASE(n) + RA_SPI_SPSSR_OFFSET)
#define RA_SPI_SPBR(n)           (RA_SPI_BASE(n) + RA_SPI_SPBR_OFFSET)
#define RA_SPI_SPDCR(n)          (RA_SPI_BASE(n) + RA_SPI_SPDCR_OFFSET)
#define RA_SPI_SPCKD(n)          (RA_SPI_BASE(n) + RA_SPI_SPCKD_OFFSET)
#define RA_SPI_SSLND(n)          (RA_SPI_BASE(n) + RA_SPI_SSLND_OFFSET)
#define RA_SPI_SPND(n)           (RA_SPI_BASE(n) + RA_SPI_SPND_OFFSET)
#define RA_SPI_SPCR2(n)          (RA_SPI_BASE(n) + RA_SPI_SPCR2_OFFSET)
#define RA_SPI_SPCMD0(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD0_OFFSET)
#define RA_SPI_SPCMD1(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD1_OFFSET)
#define RA_SPI_SPCMD2(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD2_OFFSET)
#define RA_SPI_SPCMD3(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD3_OFFSET)
#define RA_SPI_SPCMD4(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD4_OFFSET)
#define RA_SPI_SPCMD5(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD5_OFFSET)
#define RA_SPI_SPCMD6(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD6_OFFSET)
#define RA_SPI_SPCMD7(n)         (RA_SPI_BASE(n) + RA_SPI_SPCMD7_OFFSET)
#define RA_SPI_SPDCR2(n)         (RA_SPI_BASE(n) + RA_SPI_SPDCR2_OFFSET)

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
