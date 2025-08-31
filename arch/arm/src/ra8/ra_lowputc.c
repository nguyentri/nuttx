/****************************************************************************
 * arch/arm/src/ra8/ra_lowputc.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "ra_lowputc.h"
#include "ra_gpio.h"
#include "hardware/ra_sci.h"
#include "hardware/ra_mstp.h"
#include "hardware/ra_system.h"

/* The board.h file may redefine pin configurations defined in ra_pinmap.h */

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check for RTT console */

#ifdef CONFIG_SERIAL_RTT_CONSOLE
#  define HAVE_RTT_CONSOLE 1
#endif

/* Is there a serial console?  It could be on SCI0-1 or USART0-3 */

#if defined(CONFIG_SCI0_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI0_UART)
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI1_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI2_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI3_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI4_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI9_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE) && defined(CONFIG_RA_SCI9_UART)
#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#define HAVE_CONSOLE    1
#else
#if !defined(HAVE_RTT_CONSOLE) && !defined(HAVE_CONSOLE)
#warning "No valid console configuration"
#endif

#undef CONFIG_SCI0_SERIAL_CONSOLE
#undef CONFIG_SCI1_SERIAL_CONSOLE
#undef CONFIG_SCI2_SERIAL_CONSOLE
#undef CONFIG_SCI3_SERIAL_CONSOLE
#undef CONFIG_SCI4_SERIAL_CONSOLE
#undef HAVE_CONSOLE
#endif

#if defined(HAVE_CONSOLE) && !defined(HAVE_RTT_CONSOLE)

/* Select USART parameters for the selected console */

#  if defined(CONFIG_SCI0_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI0_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_SCI0
#    define RA_CONSOLE_BAUD     CONFIG_SCI0_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI0_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI0_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI0_2STOP
#  elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI1_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_SCI1
#    define RA_CONSOLE_BAUD     CONFIG_SCI1_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI1_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI1_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI1_2STOP
#  elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI2_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_SCI2
#    define RA_CONSOLE_BAUD     CONFIG_SCI2_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI2_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI2_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI2_2STOP
#  elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI3_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_SCI3
#    define RA_CONSOLE_BAUD     CONFIG_SCI3_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI3_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI3_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI3_2STOP
#  elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI4_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_SCI4
#    define RA_CONSOLE_BAUD     CONFIG_SCI4_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI4_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI4_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI4_2STOP
#  elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
#    define RA_CONSOLE_BASE     R_SCI9_BASE
#    define RA_CONSOLE_MTSP     R_MSTP_MSTPCRB_SCI9
#    define RA_CONSOLE_BAUD     CONFIG_SCI9_BAUD
#    define RA_CONSOLE_BITS     CONFIG_SCI9_BITS
#    define RA_CONSOLE_PARITY   CONFIG_SCI9_PARITY
#    define RA_CONSOLE_2STOP    CONFIG_SCI9_2STOP
#  else
#    error "No CONFIG_UARTn_SERIAL_CONSOLE Setting"
#  endif
#endif

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
#ifdef HAVE_CONSOLE
static spinlock_t g_ra_lowputc_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
#ifdef HAVE_CONSOLE
  irqstate_t flags;

  /* RA8E1 uses SCI_B (version 2) registers, not legacy SCI registers
   * For SCI_B:
   * - Use CSR register for status instead of SSR
   * - Use TDR_BY register for byte transmission instead of TDR
   * - TDRE flag is at bit position 29 in CSR, not bit 7 in SSR
   */

  /* Wait for Transmit Data Register Empty (TDRE) flag in CSR register */
  while ((getreg32(RA_CONSOLE_BASE + R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TDRE) == 0)
    {
    }

  /* Disable interrupts so that the test and the transmission are atomic */
  flags = spin_lock_irqsave(&g_ra_lowputc_lock);

  /* Double-check TDRE is still set */
  if ((getreg32(RA_CONSOLE_BASE + R_SCI_B_CSR_OFFSET) & R_SCI_B_CSR_TDRE) != 0)
    {
      /* Send the character to TDR_BY register (byte access) */
      putreg8((uint32_t)ch, RA_CONSOLE_BASE + R_SCI_B_TDR_BY_OFFSET);

      /* Clear TDRE flag by writing to CFCLR register */
      putreg32(R_SCI_B_CFCLR_TDREC, RA_CONSOLE_BASE + R_SCI_B_CFCLR_OFFSET);
    }

  spin_unlock_irqrestore(&g_ra_lowputc_lock, flags);
#endif
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

void up_putc(int ch)
{
#ifdef HAVE_CONSOLE
  arm_lowputc(ch);
#endif
}

/****************************************************************************
 * Name: ra_lowsetup
 *
 * Description:
 *   This performs basic initialization of the USART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void ra_lowsetup(void)
{
#ifdef HAVE_CONSOLE
  uint32_t regval;
#endif

#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI0_RX);
  ra_configgpio(GPIO_SCI0_TX);
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI1_RX);
  ra_configgpio(GPIO_SCI1_TX);
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI2_RX);
  ra_configgpio(GPIO_SCI2_TX);
#elif defined(CONFIG_SCI3_SERIAL_CONSOLE)
  /* TODO: Add proper GPIO pin configuration for SCI3 when pins are determined */
  /* ra_configgpio(GPIO_SCI3_RX); */
  /* ra_configgpio(GPIO_SCI3_TX); */
#elif defined(CONFIG_SCI4_SERIAL_CONSOLE)
  /* TODO: Add proper GPIO pin configuration for SCI4 when pins are determined */
  /* ra_configgpio(GPIO_SCI4_RX); */
  /* ra_configgpio(GPIO_SCI4_TX); */
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
  ra_configgpio(GPIO_SCI9_RX);
  ra_configgpio(GPIO_SCI9_TX);
#endif

#if defined(HAVE_CONSOLE)
  /* RA8E1 SCI_B initialization sequence based on FSP reference
   * Follow the exact sequence from R_SCI_B_UART_Open() in FSP
   */

  /* 1. Enable module stop control for the SCI channel (FSP: R_BSP_MODULE_START)
   * For SCI2: MSTPCRB bit (31-2) = bit 29
   * Formula: BSP_MSTP_BIT_FSP_IP_SCI(channel) = (1U << (31U - channel))
   */
  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);

  /* Clear the module stop bit for the SCI channel */
#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI0, 0);  /* Clear SCI0 module stop bit */
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI1, 0);  /* Clear SCI1 module stop bit */
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI2, 0);  /* Clear SCI2 module stop bit */
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI9, 0);  /* Clear SCI9 module stop bit */
#endif

  /* Read back to ensure write completed and add delay (FSP: FSP_REGISTER_READ) */
  (void)getreg32(R_MSTP_MSTPCRB);

  /* Add a small delay to ensure the module is powered up before register access */
  for (volatile int i = 0; i < 1000; i++)
    {
      /* Wait for module power-up */
    }

  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);

  /* 2. Initialize registers as per FSP sequence */
  /* First set CCR0 with IDSEL (FSP does this first) */
  regval = R_SCI_B_CCR0_IDSEL;  /* IDSEL=1 for ID frame select */
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR0_OFFSET);

  /* 3. Set the UART configuration (FSP: r_sci_b_uart_config_set) */

  /* Configure CCR2 for baud rate (based on FSP: brr=47, bgdm=1, cks=0)
   * For 115200 baud at 120MHz PCLKA with double-speed mode:
   * CCR2 = (MDDR=256 << 24) | (CKS=0 << 20) | (BRR=47 << 8) | (BGDM=1 << 4)
   * Note: Use 32-bit constant (256UL) to avoid shift overflow warning
   */
  regval = (SCI_B_UART_MDDR_MAX << R_SCI_B_CCR2_MDDR_SHIFT) |
           (0 << R_SCI_B_CCR2_CKS_SHIFT) |
           (SCI_B_UART_BRR_115200_120MHZ << R_SCI_B_CCR2_BRR_SHIFT) |
           R_SCI_B_CCR2_BGDM;
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR2_OFFSET);

  /* Configure CCR3 for data format (8-bit, 1 stop, async mode)
   * CCR3 = LSBF | CHR(8-bit=0) | STP(1 stop=0) | MOD(async=0)
   */
  regval = R_SCI_B_CCR3_LSBF |
           (0 << R_SCI_B_CCR3_CHR_SHIFT) |
           (0 & R_SCI_B_CCR3_STP) |
           (0 << R_SCI_B_CCR3_MOD_SHIFT); /* LSBF=1, 8-bit, 1 stop, async */
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR3_OFFSET);

  /* Configure CCR1 for parity and flow control
   * CCR1 = SPB2DT(1) for proper TXD pin level when TE=0
   */
  regval = R_SCI_B_CCR1_SPB2DT; /* SPB2DT=1, no parity, no flow control */
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR1_OFFSET);

  /* Clear CCR4 (additional features) */
  putreg32(0, RA_CONSOLE_BASE + R_SCI_B_CCR4_OFFSET);

  /* 4. Clear all status flags in CFCLR (FSP does this) */
  putreg32(R_SCI_B_CFCLR_RDRFC | R_SCI_B_CFCLR_TDREC | R_SCI_B_CFCLR_FERC |
           R_SCI_B_CFCLR_PERC | R_SCI_B_CFCLR_MFFC | R_SCI_B_CFCLR_ORERC |
           R_SCI_B_CFCLR_DFERC | R_SCI_B_CFCLR_DPERC | R_SCI_B_CFCLR_DCMFC |
           R_SCI_B_CFCLR_ERSC, RA_CONSOLE_BASE + R_SCI_B_CFCLR_OFFSET);

  /* 5. Clear FIFO flags if FIFO is present (FSP does this) */
  putreg32(R_SCI_B_FFCLR_DRC, RA_CONSOLE_BASE + R_SCI_B_FFCLR_OFFSET);

  /* 6. Enable transmitter and receiver in CCR0 (FSP sequence)
   * CCR0 = IDSEL | RE | TE
   */
  regval = R_SCI_B_CCR0_IDSEL | R_SCI_B_CCR0_RE | R_SCI_B_CCR0_TE;
  putreg32(regval, RA_CONSOLE_BASE + R_SCI_B_CCR0_OFFSET);

  /* 7. Wait for receiver internal state = 1 (FSP: CESR.RIST=1)
   * CESR.RIST bit: 0
   */
  while ((getreg8(RA_CONSOLE_BASE + R_SCI_B_CESR_OFFSET) & R_SCI_B_CESR_RIST) == 0)
    {
      /* Wait for receiver internal state = 1 (RIST bit) */
    }
#endif
}
