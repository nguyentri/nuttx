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

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include <arch/board/board.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/ra_icu.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef struct {
  int el[RA_IRQ_IELSR_SIZE]; /* Event Link number - use hardcoded size for now */
  xcpt_t handler[RA_IRQ_IELSR_SIZE]; /* Handler function */
  void *arg[RA_IRQ_IELSR_SIZE]; /* Argument for handler */
} ra_icu_handler_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Forward declarations for configuration-time interrupts */
int button_handler_isr(int irq, void *context, void *arg);
int ra_systick_isr(int irq, void *context, void *arg);

/* Global ICU handler structure - supports both configuration and runtime registration */
static ra_icu_handler_t g_icu_handlers;
static uint32_t g_icu_slot = 0; /* next available slot */

/****************************************************************************
 * Configuration-time interrupt definitions
 * These are registered during system initialization
 ****************************************************************************/

typedef struct
{
  int event;
  xcpt_t handler;
  void *arg;
} ra_config_irq_t;

static const ra_config_irq_t g_config_irqs[] =
{
#ifdef CONFIG_RA_SYSTICK_GPT
  { RA_EL_GPT0_COUNTER_OVERFLOW, ra_systick_isr, NULL },
#endif
#ifdef CONFIG_ARCH_BUTTONS
  { RA_EL_ICU_IRQ13, button_handler_isr, NULL },
#endif
  { -1, NULL, NULL } /* Sentinel */
};

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
  int icu_slot = (int)(uintptr_t)arg;

  /* Call the registered handler if available */
  if (g_icu_handlers.handler[icu_slot] != NULL)
    {
      return g_icu_handlers.handler[icu_slot](irq, context, g_icu_handlers.arg[icu_slot]);
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
 *   Initialize the ICU driver and register configuration-time interrupts
 *
 ****************************************************************************/

void ra_icu_initialize(void)
{
  int i;

  /* Initialize the handlers structure */
  for (i = 0; i < RA_IRQ_IELSR_SIZE; i++)
    {
      g_icu_handlers.el[i] = -1;
      g_icu_handlers.handler[i] = NULL;
      g_icu_handlers.arg[i] = NULL;
    }

  /* Reset slot counter */
  g_icu_slot = 0;

  /* Register configuration-time interrupts using the unified API */
  for (i = 0; g_config_irqs[i].event != -1; i++)
    {
      int ret = ra_icu_attach(g_config_irqs[i].event,
                              g_config_irqs[i].handler,
                              g_config_irqs[i].arg);
      if (ret < 0)
        {
          /* Note: Cannot use _err() here as syslog is not ready during early IRQ init.
           * Any error will be evident by system not working properly.
           * For debugging, enable early showprogress in ra_start.c instead.
           */
           arm_lowputc('E'); // Indicate error with 'E'
        }
    }
}

/****************************************************************************
 * Name: ra_icu_attach
 *
 * Description:
 *   Attach an ICU interrupt handler (unified API for both config-time and runtime)
 *   This function handles both event linking and IRQ enabling
 *
 ****************************************************************************/

int ra_icu_attach(int event, xcpt_t handler, void *arg)
{
  int slot;

  /* Find next available slot */
  if (g_icu_slot >= RA_IRQ_IELSR_SIZE)
    {
      return -ENOMEM;
    }

  slot = g_icu_slot++;

  /* Set up the ICU event link */
  ra_icu_set_event(slot, event);

  /* Store the handler information */
  g_icu_handlers.el[slot] = event;
  g_icu_handlers.handler[slot] = handler;
  g_icu_handlers.arg[slot] = arg;

  /* Attach the common interrupt handler */
  irq_attach(RA_IRQ_FIRST + slot, ra_icu_interrupt, (void *)(uintptr_t)slot);

  /* Enable the interrupt */
  up_enable_irq(RA_IRQ_FIRST + slot);

  return RA_IRQ_FIRST + slot;
}

/****************************************************************************
 * Name: ra_icu_detach
 *
 * Description:
 *   Detach an ICU interrupt handler (unified API)
 *   This function handles both IRQ disabling and slot deallocation
 *
 ****************************************************************************/

int ra_icu_detach(int icu_irq)
{
  int slot;
  int i;

  /* Validate IRQ range */
  if (icu_irq < RA_IRQ_FIRST || icu_irq >= 112)
    {
      return -EINVAL;
    }

  slot = icu_irq - RA_IRQ_FIRST;

  /* Validate slot range */
  if (slot < 0 || slot >= RA_IRQ_IELSR_SIZE)
    {
      return -EINVAL;
    }

  /* Check if slot is actually in use */
  if (g_icu_handlers.handler[slot] == NULL)
    {
      return -ENOENT;
    }

  /* Disable the interrupt */
  up_disable_irq(icu_irq);

  /* Detach the interrupt handler */
  irq_detach(icu_irq);

  /* Clear the handler information */
  g_icu_handlers.el[slot] = -1;
  g_icu_handlers.handler[slot] = NULL;
  g_icu_handlers.arg[slot] = NULL;

  /* Clear the ICU event link */
  ra_icu_set_event(slot, 0);

  /* Compact the slot allocation if this was the last allocated slot */
  if (slot == (g_icu_slot - 1))
    {
      /* Find the highest used slot */
      int highest_used = -1;
      for (i = 0; i < g_icu_slot; i++)
        {
          if (g_icu_handlers.handler[i] != NULL)
            {
              highest_used = i;
            }
        }

      /* Update g_icu_slot to the next available slot after highest used */
      g_icu_slot = highest_used + 1;
    }

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

  if (icu_slot < 0 || icu_slot >= RA_IRQ_IELSR_SIZE)
    {
      return -EINVAL;
    }

  regaddr = R_ICU_IELSR(icu_slot);
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
 *   Clear interrupt request status. For most RA8 peripherals, interrupt
 *   clearing is handled by the peripheral itself (e.g., reading data from
 *   UART, clearing GPT status flags). The ICU IELSR.IR bit is automatically
 *   cleared when the interrupt is acknowledged.
 *
 ****************************************************************************/

void ra_icu_clear_irq(int irq)
{
  /* For most RA8 interrupts, clearing is handled automatically by:
   * 1. The peripheral itself (e.g., UART read/write, GPT status clear)
   * 2. The ICU hardware when the interrupt is acknowledged
   *
   * The IR (Interrupt Request) bit in IELSR is automatically cleared
   * by hardware when the interrupt is acknowledged by the CPU.
   *
   * Some specific cases that might need manual clearing:
   * - External IRQ pins (IRQ0-15) - cleared by writing to IRQCR
   * - Software interrupts
   *
   * For now, we implement a generic approach that works for most cases.
   */

  /* The IR bit is automatically cleared by hardware for most events.
   * If specific peripherals need manual clearing, they should handle
   * it in their own interrupt handlers.
   */
}
