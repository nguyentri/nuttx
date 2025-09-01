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
  /* Only GPIO configuration and module power-up
   * Full SCI_B configuration is done later in arm_earlyserialinit()
   * This avoids duplication between lowputc and serial driver initialization
   */

  /* Configure GPIO pins for console UART only */
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
  /* Enable module stop control for the console SCI channel only
   * This is the minimal setup required for early debug output
   * Full SCI_B register configuration will be done in arm_earlyserialinit()
   */
  putreg16((R_SYSTEM_PRCR_PRKEY_VALUE | R_SYSTEM_PRCR_PRC1), R_SYSTEM_PRCR);

  /* Clear the module stop bit for the console SCI channel */
#if defined(CONFIG_SCI0_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI0, 0);
#elif defined(CONFIG_SCI1_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI1, 0);
#elif defined(CONFIG_SCI2_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI2, 0);
#elif defined(CONFIG_SCI9_SERIAL_CONSOLE)
  modifyreg32(R_MSTP_MSTPCRB, R_MSTP_MSTPCRB_SCI9, 0);
#endif

  /* Read back to ensure write completed and add delay */
  (void)getreg32(R_MSTP_MSTPCRB);

  /* Add a small delay to ensure the module is powered up */
  for (volatile int i = 0; i < 1000; i++)
    {
      /* Wait for module power-up */
    }

  putreg16(R_SYSTEM_PRCR_PRKEY_VALUE, R_SYSTEM_PRCR);
#endif
}
