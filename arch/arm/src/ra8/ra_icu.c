/****************************************************************************
 * arch/arm/src/ra8/ra_icu.c
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

#ifdef CONFIG_SERIAL_TERMIOS
#include <termios.h>
#endif

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/ra_sci.h"
#include "hardware/ra_mstp.h"
#include "hardware/ra_system.h"
#include "hardware/ra_icu.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_icu_handlers[NR_IRQS];
static void *g_icu_args[NR_IRQS];

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_icu_interrupt
 *
 * Description:
 *   Common ICU interrupt handler
 *
 ****************************************************************************/

static int ra_icu_interrupt(int irq, void *context, void *arg)
{
  int icu_irq = (int)arg;

  /* Clear the interrupt flag */
  ra_icu_clear_irq(irq);

  /* Call the registered handler if available */
  if (g_icu_handlers[icu_irq] != NULL)
    {
      return ((int (*)(int, void *, void *))g_icu_handlers[icu_irq])
               (irq, context, g_icu_args[icu_irq]);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_icu_initialize
 *
 * Description:
 *   Initialize the ICU driver
 *
 ****************************************************************************/

void ra_icu_initialize(void)
{
  int i;

  /* Clear all handlers */
  for (i = 0; i < NR_IRQS; i++)
    {
      g_icu_handlers[i] = NULL;
      g_icu_args[i] = NULL;
    }

  /* Clear all interrupt flags */
  putreg16(0xFFFF, R_ICU_NMICLR);
}

/****************************************************************************
 * Name: ra_icu_attach_irq
 *
 * Description:
 *   Attach an ICU interrupt handler
 *
 ****************************************************************************/

int ra_icu_attach_irq(int icu_irq, int (*handler)(int, void *, void *), void *arg)
{
  if (icu_irq < 0 || icu_irq >= NR_IRQS)
    {
      return -EINVAL;
    }

  g_icu_handlers[icu_irq] = handler;
  g_icu_args[icu_irq] = arg;

  /* Attach the common interrupt handler using IELSR IRQ numbers */
  irq_attach(RA_IRQ_FIRST + icu_irq, ra_icu_interrupt, (void *)(uintptr_t)icu_irq);

  return OK;
}

/****************************************************************************
 * Name: ra_icu_detach
 *
 * Description:
 *   Detach an ICU interrupt handler
 *
 ****************************************************************************/

int ra_icu_detach(int icu_irq)
{
  if (icu_irq < 0 || icu_irq >= NR_IRQS)
    {
      return -EINVAL;
    }

  g_icu_handlers[icu_irq] = NULL;
  g_icu_args[icu_irq] = NULL;

  /* Detach the interrupt handler */
  irq_detach(icu_irq);

  return OK;
}

/****************************************************************************
 * Name: ra_icu_enable
 *
 * Description:
 *   Enable an ICU interrupt
 *
 ****************************************************************************/

void ra_icu_enable(int icu_irq)
{
  if (icu_irq >= 0 && icu_irq < NR_IRQS)
    {
      up_enable_irq(icu_irq);
    }
}

/****************************************************************************
 * Name: ra_icu_disable
 *
 * Description:
 *   Disable an ICU interrupt
 *
 ****************************************************************************/

void ra_icu_disable(int icu_irq)
{
  if (icu_irq >= 0 && icu_irq < NR_IRQS)
    {
      up_disable_irq(icu_irq);
    }
}

/****************************************************************************
 * Name: ra_icu_config
 *
 * Description:
 *   Configure an ICU interrupt
 *
 ****************************************************************************/

int ra_icu_config(int icu_irq, uint8_t mode, bool filter_enable,
                  uint8_t filter_clock)
{
  uint32_t regaddr;
  uint8_t regval;

  if (icu_irq < 0 || icu_irq >= 16)  /* Only IRQ0-15 have configuration */
    {
      return -EINVAL;
    }

  regaddr = R_ICU_IRQCR(icu_irq);
  regval = 0;

  /* Set detection mode */
  regval |= (mode & R_ICU_IRQCR_IRQMD_MASK) << R_ICU_IRQCR_IRQMD;

  /* Set filter configuration */
  if (filter_enable)
    {
      regval |= R_ICU_IRQCR_FLTEN;
      regval |= (filter_clock & R_ICU_IRQCR_FCLKSEL_MASK) << R_ICU_IRQCR_FCLKSEL_SHIFT;
    }

  putreg8(regval, regaddr);

  return OK;
}

/****************************************************************************
 * Name: ra_icu_set_event
 *
 * Description:
 *   Set ICU event link
 *
 ****************************************************************************/

int ra_icu_set_event(int icu_slot, int event)
{
  uint32_t regaddr;
  uint32_t regval;

  uint32_t icu_slot_ielsr = icu_slot - RA_IRQ_FIRST;

  if (icu_slot_ielsr < 0 || icu_slot_ielsr >= RA_IRQ_IELSR_SIZE)
    {
      return -EINVAL;
    }

  regaddr = R_ICU_IELSR(icu_slot_ielsr);
  regval = getreg32(regaddr);

  regval &= ~(R_ICU_IELSR_IELS_MASK << R_ICU_IELSR_IELS_SHIFT);
  regval |= (event & R_ICU_IELSR_IELS_MASK) << R_ICU_IELSR_IELS_SHIFT;

  putreg32(regval, regaddr);

  return OK;
}

/****************************************************************************
 * Name: ra_icu_enable_wakeup
 *
 * Description:
 *   Enable wakeup for specific ICU interrupts
 *
 ****************************************************************************/

void ra_icu_enable_wakeup(uint32_t mask)
{
  modifyreg32(R_ICU_WUPEN, 0, mask);
}

/****************************************************************************
 * Name: ra_icu_disable_wakeup
 *
 * Description:
 *   Disable wakeup for specific ICU interrupts
 *
 ****************************************************************************/

void ra_icu_disable_wakeup(uint32_t mask)
{
  modifyreg32(R_ICU_WUPEN, mask, 0);
}

/****************************************************************************
 * Name: ra_icu_clear_nmi_status
 *
 * Description:
 *   Clear NMI status flags
 *
 ****************************************************************************/

void ra_icu_clear_nmi_status(uint16_t mask)
{
  putreg16(mask, R_ICU_NMICLR);
}

/****************************************************************************
 * Name: ra_icu_get_nmi_status
 *
 * Description:
 *   Get NMI status flags
 *
 ****************************************************************************/

uint16_t ra_icu_get_nmi_status(void)
{
  return getreg16(R_ICU_NMISR);
}

/****************************************************************************
 * Name: ra_icu_enable_nmi
 *
 * Description:
 *   Enable NMI interrupts
 *
 ****************************************************************************/

void ra_icu_enable_nmi(uint16_t mask)
{
  modifyreg16(R_ICU_NMIER, 0, mask);
}

/****************************************************************************
 * Name: ra_icu_disable_nmi
 *
 * Description:
 *   Disable NMI interrupts
 *
 ****************************************************************************/

void ra_icu_disable_nmi(uint16_t mask)
{
  modifyreg16(R_ICU_NMIER, mask, 0);
}


/****************************************************************************
 * Name: ra_icu_clear_irq
 *
 * Description:
 *   Clear interrupt request status
 *
 ****************************************************************************/

void ra_icu_clear_irq(int irq)
{
  uint32_t regaddr;
  regaddr = irq - RA_IRQ_FIRST;
  modifyreg32(R_ICU_IELSR(regaddr), R_ICU_IELSR_IR, 0);
  getreg32(R_ICU_IELSR(regaddr));
}

/****************************************************************************
 * Name: ra_icu_attach_all
 *
 * Description:
 *   Attach all ICU events to the IRQ vector table
 *
 ****************************************************************************/
void ra_icu_attach_all(void)
{
  /* Attach all ICU events to the IRQ vector table - currently not used */
}
