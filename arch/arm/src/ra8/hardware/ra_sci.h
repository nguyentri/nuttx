/****************************************************************************
 * arch/arm/src/ra8/hardware/ra_sci.h
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

#ifndef __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8_SCI_H
#define __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8_SCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "ra_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* Legacy SCI registers */
#define R_SCI_SMR_OFFSET          0x0000  /* Status register (32-bits) */
#define R_SCI_BRR_OFFSET          0x0001  /* Bit Rate Register (8-bits) */
#define R_SCI_SCR_OFFSET          0x0002  /* Serial Control Register (8-bits) */
#define R_SCI_TDR_OFFSET          0x0003  /* Transmit Data Register (8-bits) */
#define R_SCI_SSR_OFFSET          0x0004  /* Serial Status Register (8-bits) */
#define R_SCI_RDR_OFFSET          0x0005  /* Receive Data Register (8-bits) */

/* SCI_B (version 2) registers - used by RA8E1 */
#define R_SCI_B_RDR_OFFSET        0x0000  /* Receive Data Register (32-bits) */
#define R_SCI_B_RDR_BY_OFFSET     0x0000  /* Receive Data Register Byte (8-bits) */
#define R_SCI_B_TDR_OFFSET        0x0004  /* Transmit Data Register (32-bits) */
#define R_SCI_B_TDR_BY_OFFSET     0x0004  /* Transmit Data Register Byte (8-bits) */
#define R_SCI_B_CCR0_OFFSET       0x0008  /* Common Control Register 0 (32-bits) */
#define R_SCI_B_CCR1_OFFSET       0x000C  /* Common Control Register 1 (32-bits) */
#define R_SCI_B_CCR2_OFFSET       0x0010  /* Common Control Register 2 (32-bits) */
#define R_SCI_B_CCR3_OFFSET       0x0014  /* Common Control Register 3 (32-bits) */
#define R_SCI_B_CCR4_OFFSET       0x0018  /* Common Control Register 4 (32-bits) */
#define R_SCI_B_CESR_OFFSET       0x001C  /* Communication Enable Status Register (8-bits) */
#define R_SCI_B_CSR_OFFSET        0x0048  /* Communication Status Register (32-bits) */
#define R_SCI_B_CFCLR_OFFSET      0x0068  /* Communication Flag Clear Register (32-bits) */
#define R_SCI_B_FFCLR_OFFSET      0x0070  /* FIFO Flag Clear Register (32-bits) */
#define R_SCI_SCMR_OFFSET         0x0006  /* Smart Card Mode Register (8-bits) */
#define R_SCI_SEMR_OFFSET         0x0007  /* Serial Extended Mode Register (8-bits) */
#define R_SCI_SNFR_OFFSET         0x0008  /* Noise Filter Setting Register (8-bits) */
#define R_SCI_SIMR1_OFFSET        0x0009  /* I2C Mode Register 1 (8-bits) */
#define R_SCI_SIMR2_OFFSET        0x000a  /* I2C Mode Register 2 (8-bits) */
#define R_SCI_SIMR3_OFFSET        0x000b  /* I2C Mode Register 3 (8-bits) */
#define R_SCI_SISR_OFFSET         0x000c  /* I2C Status Register (8-bits) */
#define R_SCI_SPMR_OFFSET         0x000d  /* SPI Mode Register (8-bits) */
#define R_SCI_TDRHL_OFFSET        0x000e  /* Transmit 9-Bit Data Register (16-bits) */
#define R_SCI_FTDRHL_OFFSET       0x000e  /* Transmit FIFO Data Register HL (16-bits) */
#define R_SCI_RDRHL_OFFSET        0x0010  /* Receive 9-bit Data Register  (16-bits) */
#define R_SCI_FRDRHL_OFFSET       0x0010  /* Receive FIFO Data Register HL (16-bits) */
#define R_SCI_MDDR_OFFSET         0x0012  /* Modulation Duty Register  (8-bits) */
#define R_SCI_DCCR_OFFSET         0x0013  /* Data Compare Match Control Register (8-bits) */
#define R_SCI_FCR_OFFSET          0x0014  /* FIFO Control Register (16-bits) */
#define R_SCI_FDR_OFFSET          0x0016  /* FIFO Data Count Register (16-bits) */
#define R_SCI_LSR_OFFSET          0x0018  /* Line Status Register (16-bits) */
#define R_SCI_CDR_OFFSET          0x001a  /* Compare Match Data Register (16-bits) */
#define R_SCI_SPTR_OFFSET         0x001c  /* Serial Port Register(8-bits) */

/* Register Addresses *******************************************************/

/* SCI0  Registers */

#  define R_SCI0_SMR              (R_SCI0_BASE + R_SCI_SMR_OFFSET)
#  define R_SCI0_BRR              (R_SCI0_BASE + R_SCI_BRR_OFFSET)
#  define R_SCI0_SCR              (R_SCI0_BASE + R_SCI_SCR_OFFSET)
#  define R_SCI0_TDR              (R_SCI0_BASE + R_SCI_TDR_OFFSET)
#  define R_SCI0_SSR              (R_SCI0_BASE + R_SCI_SSR_OFFSET)
#  define R_SCI0_RDR              (R_SCI0_BASE + R_SCI_RDR_OFFSET)
#  define R_SCI0_SCMR             (R_SCI0_BASE + R_SCI_SCMR_OFFSET)
#  define R_SCI0_SEMR             (R_SCI0_BASE + R_SCI_SEMR_OFFSET)
#  define R_SCI0_SNFR             (R_SCI0_BASE + R_SCI_SNFR_OFFSET)
#  define R_SCI0_SIMR1            (R_SCI0_BASE + R_SCI_SIMR1_OFFSET)
#  define R_SCI0_SIMR2            (R_SCI0_BASE + R_SCI_SIMR2_OFFSET)
#  define R_SCI0_SIMR3            (R_SCI0_BASE + R_SCI_SIMR3_OFFSET)
#  define R_SCI0_SISR             (R_SCI0_BASE + R_SCI_SISR_OFFSET)
#  define R_SCI0_SPMR             (R_SCI0_BASE + R_SCI_SPMR_OFFSET)
#  define R_SCI0_TDRHL            (R_SCI0_BASE + R_SCI_TDRHL_OFFSET)
#  define R_SCI0_FTDRHL           (R_SCI0_BASE + R_SCI_FTDRHL_OFFSET)
#  define R_SCI0_RDRHL            (R_SCI0_BASE + R_SCI_RDRHL_OFFSET)
#  define R_SCI0_FRDRHL           (R_SCI0_BASE + R_SCI_FRDRHL_OFFSET)
#  define R_SCI0_MDDR             (R_SCI0_BASE + R_SCI_MDDR_OFFSET)
#  define R_SCI0_DCCR             (R_SCI0_BASE + R_SCI_DCCR_OFFSET)
#  define R_SCI0_FCR              (R_SCI0_BASE + R_SCI_FCR_OFFSET)
#  define R_SCI0_FDR              (R_SCI0_BASE + R_SCI_FDR_OFFSET)
#  define R_SCI0_LSR              (R_SCI0_BASE + R_SCI_LSR_OFFSET)
#  define R_SCI0_CDR              (R_SCI0_BASE + R_SCI_CDR_OFFSET)
#  define R_SCI0_SPTR             (R_SCI0_BASE + R_SCI_SPTR_OFFSET)

/* SCI1  Registers */

#  define R_SCI1_SMR              (R_SCI1_BASE + R_SCI_SMR_OFFSET)
#  define R_SCI1_BRR              (R_SCI1_BASE + R_SCI_BRR_OFFSET)
#  define R_SCI1_SCR              (R_SCI1_BASE + R_SCI_SCR_OFFSET)
#  define R_SCI1_TDR              (R_SCI1_BASE + R_SCI_TDR_OFFSET)
#  define R_SCI1_SSR              (R_SCI1_BASE + R_SCI_SSR_OFFSET)
#  define R_SCI1_RDR              (R_SCI1_BASE + R_SCI_RDR_OFFSET)
#  define R_SCI1_SCMR             (R_SCI1_BASE + R_SCI_SCMR_OFFSET)
#  define R_SCI1_SEMR             (R_SCI1_BASE + R_SCI_SEMR_OFFSET)
#  define R_SCI1_SNFR             (R_SCI1_BASE + R_SCI_SNFR_OFFSET)
#  define R_SCI1_SIMR1            (R_SCI1_BASE + R_SCI_SIMR1_OFFSET)
#  define R_SCI1_SIMR2            (R_SCI1_BASE + R_SCI_SIMR2_OFFSET)
#  define R_SCI1_SIMR3            (R_SCI1_BASE + R_SCI_SIMR3_OFFSET)
#  define R_SCI1_SISR             (R_SCI1_BASE + R_SCI_SISR_OFFSET)
#  define R_SCI1_SPMR             (R_SCI1_BASE + R_SCI_SPMR_OFFSET)
#  define R_SCI1_TDRHL            (R_SCI1_BASE + R_SCI_TDRHL_OFFSET)
#  define R_SCI1_FTDRHL           (R_SCI1_BASE + R_SCI_FTDRHL_OFFSET)
#  define R_SCI1_RDRHL            (R_SCI1_BASE + R_SCI_RDRHL_OFFSET)
#  define R_SCI1_FRDRHL           (R_SCI1_BASE + R_SCI_FRDRHL_OFFSET)
#  define R_SCI1_MDDR             (R_SCI1_BASE + R_SCI_MDDR_OFFSET)
#  define R_SCI1_DCCR             (R_SCI1_BASE + R_SCI_DCCR_OFFSET)
#  define R_SCI1_FCR              (R_SCI1_BASE + R_SCI_FCR_OFFSET)
#  define R_SCI1_FDR              (R_SCI1_BASE + R_SCI_FDR_OFFSET)
#  define R_SCI1_LSR              (R_SCI1_BASE + R_SCI_LSR_OFFSET)
#  define R_SCI1_CDR              (R_SCI1_BASE + R_SCI_CDR_OFFSET)
#  define R_SCI1_SPTR             (R_SCI1_BASE + R_SCI_SPTR_OFFSET)

/* SCI2  Registers (No FIFO) */

#  define R_SCI2_SMR              (R_SCI2_BASE + R_SCI_SMR_OFFSET)
#  define R_SCI2_BRR              (R_SCI2_BASE + R_SCI_BRR_OFFSET)
#  define R_SCI2_SCR              (R_SCI2_BASE + R_SCI_SCR_OFFSET)
#  define R_SCI2_TDR              (R_SCI2_BASE + R_SCI_TDR_OFFSET)
#  define R_SCI2_SSR              (R_SCI2_BASE + R_SCI_SSR_OFFSET)
#  define R_SCI2_RDR              (R_SCI2_BASE + R_SCI_RDR_OFFSET)
#  define R_SCI2_SCMR             (R_SCI2_BASE + R_SCI_SCMR_OFFSET)
#  define R_SCI2_SEMR             (R_SCI2_BASE + R_SCI_SEMR_OFFSET)
#  define R_SCI2_SNFR             (R_SCI2_BASE + R_SCI_SNFR_OFFSET)
#  define R_SCI2_SIMR1            (R_SCI2_BASE + R_SCI_SIMR1_OFFSET)
#  define R_SCI2_SIMR2            (R_SCI2_BASE + R_SCI_SIMR2_OFFSET)
#  define R_SCI2_SIMR3            (R_SCI2_BASE + R_SCI_SIMR3_OFFSET)
#  define R_SCI2_SISR             (R_SCI2_BASE + R_SCI_SISR_OFFSET)
#  define R_SCI2_SPMR             (R_SCI2_BASE + R_SCI_SPMR_OFFSET)
#  define R_SCI2_TDRHL            (R_SCI2_BASE + R_SCI_TDRHL_OFFSET)
#  define R_SCI2_RDRHL            (R_SCI2_BASE + R_SCI_RDRHL_OFFSET)
#  define R_SCI2_MDDR             (R_SCI2_BASE + R_SCI_MDDR_OFFSET)
#  define R_SCI2_DCCR             (R_SCI2_BASE + R_SCI_DCCR_OFFSET)
#  define R_SCI2_CDR              (R_SCI2_BASE + R_SCI_CDR_OFFSET)
#  define R_SCI2_SPTR             (R_SCI2_BASE + R_SCI_SPTR_OFFSET)

/* SCI9  Registers (No FIFO) */

#  define R_SCI9_SMR              (R_SCI9_BASE + R_SCI_SMR_OFFSET)
#  define R_SCI9_BRR              (R_SCI9_BASE + R_SCI_BRR_OFFSET)
#  define R_SCI9_SCR              (R_SCI9_BASE + R_SCI_SCR_OFFSET)
#  define R_SCI9_TDR              (R_SCI9_BASE + R_SCI_TDR_OFFSET)
#  define R_SCI9_SSR              (R_SCI9_BASE + R_SCI_SSR_OFFSET)
#  define R_SCI9_RDR              (R_SCI9_BASE + R_SCI_RDR_OFFSET)
#  define R_SCI9_SCMR             (R_SCI9_BASE + R_SCI_SCMR_OFFSET)
#  define R_SCI9_SEMR             (R_SCI9_BASE + R_SCI_SEMR_OFFSET)
#  define R_SCI9_SNFR             (R_SCI9_BASE + R_SCI_SNFR_OFFSET)
#  define R_SCI9_SIMR1            (R_SCI9_BASE + R_SCI_SIMR1_OFFSET)
#  define R_SCI9_SIMR2            (R_SCI9_BASE + R_SCI_SIMR2_OFFSET)
#  define R_SCI9_SIMR3            (R_SCI9_BASE + R_SCI_SIMR3_OFFSET)
#  define R_SCI9_SISR             (R_SCI9_BASE + R_SCI_SISR_OFFSET)
#  define R_SCI9_SPMR             (R_SCI9_BASE + R_SCI_SPMR_OFFSET)
#  define R_SCI9_TDRHL            (R_SCI9_BASE + R_SCI_TDRHL_OFFSET)
#  define R_SCI9_RDRHL            (R_SCI9_BASE + R_SCI_RDRHL_OFFSET)
#  define R_SCI9_MDDR             (R_SCI9_BASE + R_SCI_MDDR_OFFSET)
#  define R_SCI9_DCCR             (R_SCI9_BASE + R_SCI_DCCR_OFFSET)
#  define R_SCI9_CDR              (R_SCI9_BASE + R_SCI_CDR_OFFSET)
#  define R_SCI9_SPTR             (R_SCI9_BASE + R_SCI_SPTR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Status register */

#define R_SCI_SMR_CM                     (1 <<  7)                     /* Bit 7: Communication Mode */
#define R_SCI_SMR_CHR                    (1 <<  6)                     /* Bit 6: Character Length (Valid only in asynchronous mode) */
#define R_SCI_SMR_PE                     (1 <<  5)                     /* Bit 5: Parity Enable (Valid only in asynchronous mode) */
#define R_SCI_SMR_PM                     (1 <<  4)                     /* Bit 4: Parity Mode (Valid only when the PE bit is 1) */
#define R_SCI_SMR_STOP                   (1 <<  3)                     /* Bit 3: Stop Bit Length (Valid only in asynchronous mode) */
#define R_SCI_SMR_MP                     (1 <<  2)                     /* Bit 2: Multi-Processor Mode (Valid only in asynchronous mode) */
#define R_SCI_SMR_CKS_SHIFT              (0)                           /* Bits 1-0:  Clock Select */
#define R_SCI_SMR_CKS_MASK               (3 << R_SCI_SMR_CKS_SHIFT)
#define R_SCI_SMR_CKS_DIV_0            (0 << R_SCI_SMR_CKS_SHIFT)    /* PCLKA clock (n = 0) */
#define R_SCI_SMR_CKS_DIV_4            (1 << R_SCI_SMR_CKS_SHIFT)    /* PCLKA/4 clock (n = 0) */
#define R_SCI_SMR_CKS_DIV_16           (2 << R_SCI_SMR_CKS_SHIFT)    /* PCLKA/16 clock (n = 0) */
#define R_SCI_SMR_CKS_DIV_64           (3 << R_SCI_SMR_CKS_SHIFT)    /* PCLKA/64 clock (n = 0) */

/* Bit Rate Register  */

#define R_SCI_BRR_SHIFT                 (0) /* Bits 7-0: BRR is an 8-bit register that adjusts the bit rate. */
#define R_SCI_BRR_MASK                  (0xff)

/* Serial Control Register   */

#define R_SCI_SCR_TIE                    (1 <<  7)                     /* Bit 7: Transmit Interrupt Enable */
#define R_SCI_SCR_RIE                    (1 <<  6)                     /* Bit 6: Receive Interrupt Enable */
#define R_SCI_SCR_TE                     (1 <<  5)                     /* Bit 5: Transmit Enable */
#define R_SCI_SCR_RE                     (1 <<  4)                     /* Bit 4: Receive Enable */
#define R_SCI_SCR_MPIE                   (1 <<  3)                     /* Bit 3: Multi-Processor Interrupt Enable (Valid in asynchronous mode when SMR.MP = 1) */
#define R_SCI_SCR_TEIE                   (1 <<  2)                     /* Bit 2: Transmit End Interrupt Enable */
#define R_SCI_SCR_CKE_SHIFT              (0)                           /* Bits 1-0:  Clock Enable */
#define R_SCI_SCR_CKE_MASK               (3 << R_SCI_SCR_CKE_SHIFT)
#  define R_SCI_SCR_CKE_MOD0             (0 << R_SCI_SCR_CKE_SHIFT)    /* Asynchronous mode: On-chip baud rate generator and the SCKn pin is available for use as an I/O port, Clock synchronous mode:  Internal clock */
#  define R_SCI_SCR_CKE_MOD1             (1 << R_SCI_SCR_CKE_SHIFT)    /* Asynchronous mode: On-chip baud rate generator and a same bit rate clock outputted on the SCKn pin, Clock synchronous mode:  Internal clock */
#  define R_SCI_SCR_CKE_MOD2             (2 << R_SCI_SCR_CKE_SHIFT)    /* External clock*/

/* Transmit Data Register  */

#define R_SCI_TDR_SHIFT                 (0) /* Bits 7-0: TDR is an 8-bit register that stores transmit data. */
#define R_SCI_TDR_MASK                  (0xff)

/* Serial Status Register */

#define R_SCI_SSR_TDRE                   (1 <<  7) /* Bit 7: Transmit Data Empty Flag */
#define R_SCI_SSR_RDRF                   (1 <<  6) /* Bit 6: Receive Data Full Flag */
#define R_SCI_SSR_ORER                   (1 <<  5) /* Bit 5: Overrun Error Flag */
#define R_SCI_SSR_FER                    (1 <<  4) /* Bit 4: Framing Error Flag */
#define R_SCI_SSR_PER                    (1 <<  3) /* Bit 3: Parity Error Flag */
#define R_SCI_SSR_TEND                   (1 <<  2) /* Bit 2: Transmit End Flag */
#define R_SCI_SSR_MPB                    (1 <<  1) /* Bit 1: Multi-Processor */
#define R_SCI_SSR_MPBT                   (1 <<  0) /* Bit 0: Multi-Processor Bit Transfer */

/* Receive Data Register  */

#define R_SCI_RDR_SHIFT                 (0) /* Bits 7-0: RDR is an 8-bit register that stores receive data. */
#define R_SCI_RDR_MASK                  (0xff)

/* Smart Card Mode Register  */

#define R_SCI_SCMR_BCP2                  (1 <<  7) /* Bit 7: Base Clock Pulse 2 Selects the number of base clock cycles in combination with the SMR.BCP[1:0] bits */
#define R_SCI_SCMR_CHR1                  (1 <<  4) /* Bit 4: Character Length 1 (Only valid in asynchronous mode) */
#define R_SCI_SCMR_SDIR                  (1 <<  3) /* Bit 3: Transmitted/Received Data Transfer Direction NOTE: The setting is invalid and a fixed data length of 8 bits is used in modes other than asynchronous mode. Set this bit to 1 if operation is to be in simple I2C mode. */
#define R_SCI_SCMR_SINV                  (1 <<  2) /* Bit 2: Transmitted/Received Data Invert Set this bit to 0 if operation is to be in simple I2C mode. */
#define R_SCI_SCMR_SMIF                  (1 <<  0) /* Bit 0: Smart Card Interface Mode Select */

/* Serial Extended Mode Register */

#define R_SCI_SEMR_RXDESEL               (1 <<  7) /* Bit 7: Asynchronous Start Bit Edge Detection Select (Valid only in asynchronous mode) */
#define R_SCI_SEMR_BGDM                  (1 <<  6) /* Bit 6: Baud Rate Generator Double-Speed Mode Select (Only valid the CKE[1] bit in SCR is 0 in asynchronous mode). */
#define R_SCI_SEMR_NFEN                  (1 <<  5) /* Bit 5: Digital Noise Filter Function Enable (The NFEN bit should be 0 without simple I2C mode and asynchronous mode.) In asynchronous mode, for RXDn input only. In simple I2C mode, for RXDn/TxDn input. */
#define R_SCI_SEMR_ABCS                  (1 <<  4) /* Bit 4: Asynchronous Mode Base Clock Select (Valid only in asynchronous mode) */
#define R_SCI_SEMR_ABCSE                 (1 <<  3) /* Bit 3: Asynchronous Mode Extended Base Clock Select 1 (Valid only in asynchronous mode and SCR.CKE[1]=0) */
#define R_SCI_SEMR_BRME                  (1 <<  2) /* Bit 2: Bit Rate Modulation Enable */

/* Noise Filter Setting Register  */

#define R_SCI_SNFR_NFCS_SHIFT                 (0)                          /* Bits 3-0:  Noise Filter Clock Select */
#define R_SCI_SNFR_NFCS_MASK                  (7)
#  define R_SCI_SNFR_NFCS_DIV1_ASYNC          (0 << R_SCI_SNFR_NFCS_SHIFT) /* Asynchronous mode: : The clock signal divided by 1 is used with the noise filter */
#  define R_SCI_SNFR_NFCS_DIV1_IIC            (1 << R_SCI_SNFR_NFCS_SHIFT) /* simple IIC mode:  The clock signal divided by 1 is used with the noise filter */
#  define R_SCI_SNFR_NFCS_DIV2_IIC            (2 << R_SCI_SNFR_NFCS_SHIFT) /* simple IIC mode:  The clock signal divided by 2 is used with the noise filter */
#  define R_SCI_SNFR_NFCS_DIV4_IIC            (3 << R_SCI_SNFR_NFCS_SHIFT) /* simple IIC mode:  The clock signal divided by 4 is used with the noise filter */
#  define R_SCI_SNFR_NFCS_DIV8_IIC            (4 << R_SCI_SNFR_NFCS_SHIFT) /* simple IIC mode:  The clock signal divided by 8 is used with the noise filter */

/* I2C Mode Register 1 */

#define R_SCI_SIMR1_IICDL_SHIFT           (3)       /* Bits 7-3:  SDA Delay Output Select Cycles below are of the clock signal from the on-chip baud rate generator. */
#define R_SCI_SIMR1_IICDL_MASK            (0x1f)
#define R_SCI_SIMR1_IICM                  (1 <<  0) /* Bit 0: Simple I2C Mode Select */

/* I2C Mode Register 2 */

#define R_SCI_SIMR2_IICACKT              (1 <<  5) /* Bit 5: ACK Transmission Data */
#define R_SCI_SIMR2_IICCSC               (1 <<  1) /* Bit 1: Clock Synchronization */
#define R_SCI_SIMR2_IICINTM              (1 <<  0) /* Bit 0: I2C Interrupt Mode Select */

/* I2C Mode Register 3 */
#define R_SCI_SIMR3_IICSCLS_SHIFT        (6)                                 /* Bit 7-6: SCL Output Select */
#define R_SCI_SIMR3_IICSCLS_MASK         (3)
#  define R_SCI_SIMR3_IICSCLS_SCL_OUT_0  (0 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SCL Output Select : Serial data output*/
#  define R_SCI_SIMR3_IICSCLS_SCL_OUT_1  (1 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SCL Output Select : Generate a start, restart, or stop condition*/
#  define R_SCI_SIMR3_IICSCLS_SCL_OUT_2  (2 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SCL Output Select : Output low level on the SCLn pin*/
#  define R_SCI_SIMR3_IICSCLS_SCL_OUT_3  (3 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SCL Output Select : Drive SCLn pin to high-impedance state*/
#define R_SCI_SIMR3_IICSDAS_SHIFT        (4)                                 /* Bit 5-4: SDA Output Select */
#define R_SCI_SIMR3_IICSDAS_MASK         (3)
#  define R_SCI_SIMR3_IICSDAS_SDA_OUT_0  (0 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SDA Output Select : Serial data output*/
#  define R_SCI_SIMR3_IICSDAS_SDA_OUT_1  (1 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SDA Output Select : Generate a start, restart, or stop condition*/
#  define R_SCI_SIMR3_IICSDAS_SDA_OUT_2  (2 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SDA Output Select : Output low level on the SDAn pin*/
#  define R_SCI_SIMR3_IICSDAS_SDA_OUT_3  (3 << R_SCI_SIMR3_IICSCLS_SHIFT)    /* SDA Output Select : Drive SDAn pin to high-impedance state*/
#define R_SCI_SIMR3_IICSTIF              (1 <<  3)                           /* Bit 3: Issuing of Start, Restart, or Stop Condition Completed Flag (When 0 is written to IICSTIF, it is cleared to 0.) */
#define R_SCI_SIMR3_IICSTPREQ            (1 <<  2)                           /* Bit 2: Stop Condition Generation */
#define R_SCI_SIMR3_IICRSTAREQ           (1 <<  1)                           /* Bit 1: Restart Condition Generation */
#define R_SCI_SIMR3_IICSTAREQ            (1 <<  0)                           /* Bit 0: Start Condition Generation */

/* I2C Status Register */

#define R_SCI_SISR_IICACKR               (1 <<  0) /* Bit 0: ACK Reception Data Flag */

/* SPI Mode Register  */

#define R_SCI_SPMR_CKPH                  (1 <<  7) /* Bit 7: Clock Phase Select */
#define R_SCI_SPMR_CKPOL                 (1 <<  6) /* Bit 6: Clock Polarity Select */
#define R_SCI_SPMR_MFF                   (1 <<  4) /* Bit 4: Mode Fault Flag */
#define R_SCI_SPMR_MSS                   (1 <<  2) /* Bit 2: Master Slave Select */
#define R_SCI_SPMR_CTSE                  (1 <<  1) /* Bit 1: CTS Enable */
#define R_SCI_SPMR_SSE                   (1 <<  0) /* Bit 0: SSn Pin Function Enable */

/* Transmit 9-Bit Data Register */

#define R_SCI_TDRHL_SHIFT                (0) /* Bit 15-0: TDRHL is a 16-bit register that stores transmit data. */
#define R_SCI_TDRHL_MASK                 (0xffff)

/* Transmit FIFO Data Register HL */

#define R_SCI_FTDRHL_MPBT                (1 <<  9) /* Bit 9: Multi-processor transfer bit flag (Valid only in asynchronous mode and SMR.MP=1 and FIFO selected) */
#define R_SCI_FTDRHL_TDAT_SHIFT          (0)       /* Bit 8-0: Serial transmit data (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode, and FIFO selected) */
#define R_SCI_FTDRHL_TDAT_MASK           (0x1ff)

/* Receive 9-bit Data Register  */

#define R_SCI_RDRHL_SHIFT                (0) /* Bit 15-0:: TDRHL is a 16-bit register that stores receive data. */
#define R_SCI_RDRHL_MASK                 (0xffff)

/* Transmit FIFO Data Register HL */

#define R_SCI_FRDRHL_RDF                 (1 << 14) /* Bit 14: Receive FIFO data full flag (It is same as SSR.RDF) */
#define R_SCI_FRDRHL_ORER                (1 << 13) /* Bit 13: Overrun error flag (It is same as SSR.ORER) */
#define R_SCI_FRDRHL_FER                 (1 << 12) /* Bit 12: Framing error flag */
#define R_SCI_FRDRHL_PER                 (1 << 11) /* Bit 11: Parity error flag */
#define R_SCI_FRDRHL_DR                  (1 << 10) /* Bit 10: Receive data ready flag (It is same as SSR.DR) */
#define R_SCI_FRDRHL_MPB                 (1 <<  9) /* Bit 9: Multi-processor bit flag (Valid only in asynchronous mode with SMR.MP=1 and FIFO selected) It can read multi-processor bit corresponded to serial receive data(RDATA[8:0]) */
#define R_SCI_FRDRHL_RDAT_SHIFT          (0)       /* Bit 8-0: Serial receive data (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode, and FIFO selected) */
#define R_SCI_FRDRHL_RDAT_MASK           (0x1ff)

/* Data Compare Match Control Register */

#define R_SCI_DCCR_DCME                  (1 <<  7) /* Bit 7: Data Compare Match Enable (Valid only in asynchronous mode(including multi-processor) */
#define R_SCI_DCCR_IDSEL                 (1 <<  6) /* Bit 6: ID frame select (Valid only in asynchronous mode(including multi-processor) */
#define R_SCI_DCCR_DFER                  (1 <<  4) /* Bit 4: Data Compare Match Framing Error Flag */
#define R_SCI_DCCR_DPER                  (1 <<  3) /* Bit 3: Data Compare Match Parity Error Flag */
#define R_SCI_DCCR_DCMF                  (1 <<  0) /* Bit 0: Data Compare Match Flag */

/* FIFO Control Register  */

#define R_SCI_FCR_RSTRG_SHIFT            (12)         /* Bit 15-12: RTS Output Active Trigger Number Select (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode) */
#define R_SCI_FCR_RSTRG_MASK             (0x0f)
#define R_SCI_FCR_RTRG_SHIFT             (8)          /* Bit 11-8: Receive FIFO data trigger number (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode) */
#define R_SCI_FCR_RTRG_MASK              (0x0f)
#define R_SCI_FCR_TTRG_SHIFT             (4)          /* Bit 7-4: Transmit FIFO data trigger number (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode) */
#define R_SCI_FCR_TTRG_MASK              (0x0f)
#define R_SCI_FCR_DRES                   (1 <<  3)    /* Bit 3: Receive data ready error select bit (When detecting a reception data ready, the interrupt request is selected.) */
#define R_SCI_FCR_TFRST                  (1 <<  2)    /* Bit 2: Transmit FIFO Data Register Reset (Valid only in FCR.FM=1) */
#define R_SCI_FCR_RFRST                  (1 <<  1)    /* Bit 1: Receive FIFO Data Register Reset (Valid only in FCR.FM=1) */
#define R_SCI_FCR_FM                     (1 <<  0)    /* Bit 0: FIFO Mode Select (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode) */

/* FIFO Data Count Register  */

#define R_SCI_FDR_T_SHIFT                (8) /* Bit 12-8: Transmit FIFO Data Count Indicate the quantity of non-transmit data stored in FTDRH and FTDRL (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode, while FCR.FM=1) */
#define R_SCI_FDR_T_MASK                 (0x1f)
#define R_SCI_FDR_R_SHIFT                (0) /* Bit 4-0: Receive FIFO Data Count Indicate the quantity of receive data stored in FRDRH and FRDRL (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode, while FCR.FM=1) */
#define R_SCI_FDR_R_MASK                 (0x1f)

/* Line Status Register */

#define R_SCI_LSR_PNUM_SHIFT             (8)        /* Bit 12-8: Parity Error Count Indicates the quantity of data with a parity error among the receive data stored in the receive FIFO data register (FRDRH and FRDRL). */
#define R_SCI_LSR_PNUM_MASK              (0x1f)
#define R_SCI_LSR_FNUM_SHIFT             (2)        /* Bit 6-2: Framing Error Count Indicates the quantity of data with a framing error among the receive data stored in the receive FIFO data register (FRDRH and FRDRL). */
#define R_SCI_LSR_FNUM_MASK              (0x1f)
#define R_SCI_LSR_ORER                   (1 <<  0)  /* Bit 0: Overrun Error Flag (Valid only in asynchronous mode(including multi-processor) or clock synchronous mode, and FIFO selected) */

/* Compare Match Data Register  */

#define R_SCI_CDR_CMPD_SHIFT              (0) /* Bit 8-0: Compare Match Data Compare data pattern for address match wake-up function */
#define R_SCI_CDR_CMPD_MASK               (0x1ff)

/* Serial Port Register */

#define R_SCI_SPTR_SPB2IO                (1 <<  2) /* 04: Serial port break I/O bit (It's selected whether the value of SPB2DT is output to TxD terminal.) */
#define R_SCI_SPTR_SPB2DT                (1 <<  1) /* 02: Serial port break data select bit (The output level of TxD terminal is selected when.) */
#define R_SCI_SPTR_RXDMON                (1 <<  0) /* 01: Serial input data monitor bit (The state of the RXD terminal is shown.) */

/* SCI_B (version 2) Register Bitfield Definitions */

/* Communication Status Register (CSR) */
#define R_SCI_B_CSR_RDRF                 (1 << 31) /* Bit 31: Receive Data Register Full */
#define R_SCI_B_CSR_TEND                 (1 << 30) /* Bit 30: Transmit End Flag */
#define R_SCI_B_CSR_TDRE                 (1 << 29) /* Bit 29: Transmit Data Register Empty */
#define R_SCI_B_CSR_FER                  (1 << 28) /* Bit 28: Framing Error Flag */
#define R_SCI_B_CSR_PER                  (1 << 27) /* Bit 27: Parity Error Flag */
#define R_SCI_B_CSR_MFF                  (1 << 26) /* Bit 26: Mode Fault Flag */
#define R_SCI_B_CSR_ORER                 (1 << 24) /* Bit 24: Overrun Error Flag */

/* Communication Enable Status Register (CESR) */
#define R_SCI_B_CESR_TIST                (1 <<  4) /* Bit 4: TE Internal Status */
#define R_SCI_B_CESR_RIST                (1 <<  0) /* Bit 0: RE Internal Status */

/* Common Control Register 0 (CCR0) */
#define R_SCI_B_CCR0_SSE                 (1 << 24) /* Bit 24: SSn Pin Function Enable */
#define R_SCI_B_CCR0_TEIE                (1 << 21) /* Bit 21: Transmit End Interrupt Enable */
#define R_SCI_B_CCR0_TIE                 (1 << 20) /* Bit 20: Transmit Interrupt Enable */
#define R_SCI_B_CCR0_RIE                 (1 << 16) /* Bit 16: Receive Interrupt Enable */
#define R_SCI_B_CCR0_IDSEL               (1 << 10) /* Bit 10: ID frame select */
#define R_SCI_B_CCR0_DCME                (1 <<  9) /* Bit 9: Data Compare Match Enable */
#define R_SCI_B_CCR0_MPIE                (1 <<  8) /* Bit 8: Multi-Processor Interrupt Enable */
#define R_SCI_B_CCR0_TE                  (1 <<  4) /* Bit 4: Transmit Enable */
#define R_SCI_B_CCR0_RE                  (1 <<  0) /* Bit 0: Receive Enable */

/* Common Control Register 1 (CCR1) */
#define R_SCI_B_CCR1_NFEN                (1 << 28) /* Bit 28: Digital Noise Filter Function Enable */
#define R_SCI_B_CCR1_NFCS_SHIFT          (24)      /* Bits 26-24: Noise Filter Clock Select */
#define R_SCI_B_CCR1_NFCS_MASK           (7 << R_SCI_B_CCR1_NFCS_SHIFT)
#define R_SCI_B_CCR1_SHARPS              (1 << 20) /* Bit 20: Half-duplex communication select */
#define R_SCI_B_CCR1_SPLP                (1 << 16) /* Bit 16: Loopback Control */
#define R_SCI_B_CCR1_RINV                (1 << 13) /* Bit 13: RXD invert */
#define R_SCI_B_CCR1_TINV                (1 << 12) /* Bit 12: TXD invert */
#define R_SCI_B_CCR1_PM                  (1 <<  9) /* Bit 9: Parity Mode */
#define R_SCI_B_CCR1_PE                  (1 <<  8) /* Bit 8: Parity Enable */
#define R_SCI_B_CCR1_SPB2IO              (1 <<  5) /* Bit 5: Serial port break I/O */
#define R_SCI_B_CCR1_SPB2DT              (1 <<  4) /* Bit 4: Serial port break data select */
#define R_SCI_B_CCR1_CTSPEN              (1 <<  1) /* Bit 1: CTS external pin Enable */
#define R_SCI_B_CCR1_CTSE                (1 <<  0) /* Bit 0: CTS Enable */

/* Common Control Register 2 (CCR2) */
#define R_SCI_B_CCR2_MDDR_SHIFT          (24)      /* Bits 31-24: Modulation Duty Setting */
#define R_SCI_B_CCR2_MDDR_MASK           (0xFF << R_SCI_B_CCR2_MDDR_SHIFT)
#define R_SCI_B_CCR2_CKS_SHIFT           (20)      /* Bits 21-20: Clock Select */
#define R_SCI_B_CCR2_CKS_MASK            (3 << R_SCI_B_CCR2_CKS_SHIFT)
#define R_SCI_B_CCR2_BRME                (1 << 16) /* Bit 16: Bit Modulation Enable */
#define R_SCI_B_CCR2_BRR_SHIFT           (8)       /* Bits 15-8: Bit rate setting */
#define R_SCI_B_CCR2_BRR_MASK            (0xFF << R_SCI_B_CCR2_BRR_SHIFT)
#define R_SCI_B_CCR2_ABCSE               (1 <<  6) /* Bit 6: Asynchronous Mode Extended Base Clock Select */
#define R_SCI_B_CCR2_ABCS                (1 <<  5) /* Bit 5: Asynchronous Mode Base Clock Select */
#define R_SCI_B_CCR2_BGDM                (1 <<  4) /* Bit 4: Baud Rate Generator Double-Speed Mode Select */
#define R_SCI_B_CCR2_BCP_SHIFT           (0)       /* Bits 2-0: Base Clock Pulse */
#define R_SCI_B_CCR2_BCP_MASK            (7 << R_SCI_B_CCR2_BCP_SHIFT)

/* Common Control Register 3 (CCR3) */
#define R_SCI_B_CCR3_BLK                 (1 << 29) /* Bit 29: Block Transfer Mode */
#define R_SCI_B_CCR3_GM                  (1 << 28) /* Bit 28: GSM Mode */
#define R_SCI_B_CCR3_CKE_SHIFT           (24)      /* Bits 25-24: Clock enable */
#define R_SCI_B_CCR3_CKE_MASK            (3 << R_SCI_B_CCR3_CKE_SHIFT)
#define R_SCI_B_CCR3_DEN                 (1 << 21) /* Bit 21: Driver enable */
#define R_SCI_B_CCR3_FM                  (1 << 20) /* Bit 20: FIFO Mode select */
#define R_SCI_B_CCR3_MP                  (1 << 19) /* Bit 19: Multi-Processor Mode */
#define R_SCI_B_CCR3_MOD_SHIFT           (16)      /* Bits 18-16: Communication mode select */
#define R_SCI_B_CCR3_MOD_MASK            (7 << R_SCI_B_CCR3_MOD_SHIFT)
#define R_SCI_B_CCR3_RXDESEL             (1 << 15) /* Bit 15: Asynchronous Start Bit Edge Detection Select */
#define R_SCI_B_CCR3_STP                 (1 << 14) /* Bit 14: Stop Bit Length */
#define R_SCI_B_CCR3_SINV                (1 << 13) /* Bit 13: Transmitted/Received Data Invert */
#define R_SCI_B_CCR3_LSBF                (1 << 12) /* Bit 12: LSB First select */
#define R_SCI_B_CCR3_CHR_SHIFT           (8)       /* Bits 9-8: Character Length */
#define R_SCI_B_CCR3_CHR_MASK            (3 << R_SCI_B_CCR3_CHR_SHIFT)
#define R_SCI_B_CCR3_CHR                 (1 <<  8) /* Bit 8: Character Length (7-bit when set) */
#define R_SCI_B_CCR3_PE                  (1 <<  5) /* Bit 5: Parity Enable */
#define R_SCI_B_CCR3_PM                  (1 <<  4) /* Bit 4: Parity Mode (0=even, 1=odd) */
#define R_SCI_B_CCR3_BPEN                (1 <<  7) /* Bit 7: Synchronizer bypass enable */
#define R_SCI_B_CCR3_CPOL                (1 <<  1) /* Bit 1: Clock Polarity Select */
#define R_SCI_B_CCR3_CPHA                (1 <<  0) /* Bit 0: Clock Phase Select */

/* Communication Flag Clear Register (CFCLR) values */
#define R_SCI_B_CFCLR_RDRFC              (1 << 31) /* Clear RDRF flag */
#define R_SCI_B_CFCLR_TDREC              (1 << 29) /* Clear TDRE flag */
#define R_SCI_B_CFCLR_FERC               (1 << 28) /* Clear FER flag */
#define R_SCI_B_CFCLR_PERC               (1 << 27) /* Clear PER flag */
#define R_SCI_B_CFCLR_MFFC               (1 << 26) /* Clear MFF flag */
#define R_SCI_B_CFCLR_ORERC              (1 << 24) /* Clear ORER flag */
#define R_SCI_B_CFCLR_DFERC              (1 << 18) /* Clear DFER flag */
#define R_SCI_B_CFCLR_DPERC              (1 << 17) /* Clear DPER flag */
#define R_SCI_B_CFCLR_DCMFC              (1 << 16) /* Clear DCMF flag */
#define R_SCI_B_CFCLR_ERSC               (1 <<  4) /* Clear ERS flag */

/* FIFO Flag Clear Register (FFCLR) values */
#define R_SCI_B_FFCLR_DRC                (1 <<  0) /* Clear DR flag */

/* SCI_B Configuration Constants */
#define SCI_B_UART_MDDR_MIN              (128U)    /* Minimum MDDR value */
#define SCI_B_UART_MDDR_MAX              (256UL)   /* Maximum MDDR value (disables modulation) */
#define SCI_B_UART_BRR_115200_120MHZ     (47U)     /* BRR value for 115200 baud at 120MHz with BGDM=1 */

#if defined (CONFIG_RA_SCI0_UART)
#define CONFIG_SCI0_RXI    47  /* Receive data full */
#define CONFIG_SCI0_TXI    48  /* Transmit data empty */
#define CONFIG_SCI0_TEI    49  /* Transmit end */
#define CONFIG_SCI0_ERI    50  /* Receive error */
#endif

#if defined  (CONFIG_RA_SCI1_UART)
#define CONFIG_SCI1_RXI    51  /* Receive data full */
#define CONFIG_SCI1_TXI    52  /* Transmit data empty */
#define CONFIG_SCI1_TEI    53  /* Transmit end */
#define CONFIG_SCI1_ERI    54  /* Receive error */
#endif

#if defined (CONFIG_RA_SCI2_UART)
#define CONFIG_SCI2_RXI    55  /* Receive data full */
#define CONFIG_SCI2_TXI    56  /* Transmit data empty */
#define CONFIG_SCI2_TEI    57  /* Transmit end */
#define CONFIG_SCI2_ERI    58  /* Receive error */
#endif

#if defined (CONFIG_RA_SCI9_UART)
#define CONFIG_SCI9_RXI    67  /* Receive data full */
#define CONFIG_SCI9_TXI    68  /* Transmit data empty */
#define CONFIG_SCI9_TEI    69  /* Transmit end */
#define CONFIG_SCI9_ERI    70  /* Receive error */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_RA8E1_HARDWARE_RA8_SCI_H */
