/****************************************************************************
 * arch/arm/src/ra8/ra_irq.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/armv8-m/nvicpri.h>

#include "nvic.h"
#include "arm_internal.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define DEFPRIORITY32                  \
  (NVIC_SYSH_PRIORITY_DEFAULT << 24 |  \
    NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
    NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
    NVIC_SYSH_PRIORITY_DEFAULT)

#define NVIC_ENA_OFFSET     (0)
#define NVIC_CLRENA_OFFSET  (NVIC_IRQ0_31_CLEAR - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dumpnvic
 *
 * Description:
 *   Dump some interesting NVIC registers
 *
 ****************************************************************************/

/****************************************************************************
 * Name: ra_prioritize_syscall
 *
 * Description:
 *   Set the priority of an exception.  This function may be needed
 *   internally even if support for prioritized interrupts is not enabled.
 *
 ****************************************************************************/

static inline void ra_prioritize_syscall(int priority)
{
  uint32_t regval;

  /* SVCALL is system handler 11 */

  regval    = getreg32(NVIC_SYSH8_11_PRIORITY);
  regval    &= ~NVIC_SYSH_PRIORITY_PR11_MASK;
  regval    |= (priority << NVIC_SYSH_PRIORITY_PR11_SHIFT);
  putreg32(regval, NVIC_SYSH8_11_PRIORITY);
}

#if defined(CONFIG_DEBUG_IRQ_INFO)
static void ra_dumpnvic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = enter_critical_section();

  irqinfo("NVIC (%s, irq=%d):\n", msg, irq);
  irqinfo("  INTCTRL:    %08x VECTAB:  %08x\n", getreg32(
            NVIC_INTCTRL), getreg32(NVIC_VECTAB));
  irqinfo("  IRQ ENABLE: %08x %08x %08x\n", getreg32(
            NVIC_IRQ0_31_ENABLE), getreg32(NVIC_IRQ32_63_ENABLE),
          getreg32(NVIC_IRQ64_95_ENABLE));
  irqinfo("  SYSH_PRIO:  %08x %08x %08x\n", getreg32(
            NVIC_SYSH4_7_PRIORITY), getreg32(NVIC_SYSH8_11_PRIORITY),
          getreg32(NVIC_SYSH12_15_PRIORITY));
  irqinfo("  IRQ PRIO:   %08x %08x %08x %08x\n", getreg32(
            NVIC_IRQ0_3_PRIORITY), getreg32(NVIC_IRQ4_7_PRIORITY),
          getreg32(NVIC_IRQ8_11_PRIORITY), getreg32(NVIC_IRQ12_15_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ16_19_PRIORITY), getreg32(
            NVIC_IRQ20_23_PRIORITY), getreg32(
            NVIC_IRQ24_27_PRIORITY), getreg32(NVIC_IRQ28_31_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ32_35_PRIORITY), getreg32(
            NVIC_IRQ36_39_PRIORITY), getreg32(
            NVIC_IRQ40_43_PRIORITY), getreg32(NVIC_IRQ44_47_PRIORITY));
  irqinfo("              %08x %08x %08x %08x\n",
          getreg32(NVIC_IRQ48_51_PRIORITY), getreg32(
            NVIC_IRQ52_55_PRIORITY), getreg32(
            NVIC_IRQ56_59_PRIORITY), getreg32(NVIC_IRQ60_63_PRIORITY));
  irqinfo("              %08x\n", getreg32(NVIC_IRQ64_67_PRIORITY));

  leave_critical_section(flags);
}

#else
#  define ra_dumpnvic(msg, irq)
#endif

/****************************************************************************
 * Name: ra_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int ra_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                      uintptr_t offset)
{
  int n;

  DEBUGASSERT(irq >= RA_IRQ_NMI && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= RA_IRQ_FIRST)
    {
      n         = irq - RA_IRQ_FIRST;
      *regaddr  = NVIC_IRQ_ENABLE(n) + offset;
      *bit      = (uint32_t)1 << (n & 0x1f);
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == RA_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == RA_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == RA_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == RA_IRQ_SYSTICK)
        {
          *regaddr  = NVIC_SYSTICK_CTRL;
          *bit      = NVIC_SYSTICK_CTRL_ENABLE;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t  regaddr;

  int       num_priority_registers;
  int       i;

  /* Disable all interrupts */

  for (i = 0; i < NR_IRQS - RA_IRQ_FIRST; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(DEFPRIORITY32, NVIC_SYSH4_7_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH8_11_PRIORITY);
  putreg32(DEFPRIORITY32, NVIC_SYSH12_15_PRIORITY);

  /* The NVIC ICTR register (bits 0-4) holds the number of interrupt
   * lines that the NVIC supports:
   *
   *  0 -> 32 interrupt lines,  8 priority registers
   *  1 -> 64 "       " "   ", 16 priority registers
   *  2 -> 96 "       " "   ", 32 priority registers
   *  ...
   */

  num_priority_registers = (getreg32(NVIC_ICTR) + 1) * 8;

  /* Now set all of the interrupt lines to the default priority */

  regaddr = NVIC_IRQ0_3_PRIORITY;
  while (num_priority_registers--)
    {
      putreg32(DEFPRIORITY32, regaddr);
      regaddr += 4;
    }

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(RA_IRQ_SVCALL, arm_svcall, NULL);
  irq_attach(RA_IRQ_HARDFAULT, arm_hardfault, NULL);

  /* Attach the ICU events to the IRQ vector table */
  ra_icu_attach_all();

  ra_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);

  ra_dumpnvic("initial", RA_IRQ_FIRST + 32);

  /* And finally, enable interrupts */

  up_irq_enable();
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t  regval;
  uint32_t  bit;

  if (ra_irqinfo(irq, &regaddr, &bit, NVIC_CLRENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= RA_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval    = getreg32(regaddr);
          regval    &= ~bit;
          putreg32(regval, regaddr);
        }
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  uintptr_t regaddr;
  uint32_t  regval;
  uint32_t  bit;

  if (ra_irqinfo(irq, &regaddr, &bit, NVIC_ENA_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= RA_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          regval    = getreg32(regaddr);
          regval    |= bit;
          putreg32(regval, regaddr);
        }
    }
}

/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  uint32_t  regaddr;
  uint32_t  regval;
  int       shift;

  DEBUGASSERT(
    irq >= RA_IRQ_MEMFAULT && irq < NR_IRQS &&
    (unsigned)priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < RA_IRQ_FIRST)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr   = NVIC_SYSH_PRIORITY(irq);
      irq       -= 4;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq       -= RA_IRQ_FIRST;
      regaddr   = NVIC_IRQ_PRIORITY(irq);
    }

  regval    = getreg32(regaddr);
  shift     = ((irq & 3) << 3);
  regval    &= ~(0xff << shift);
  regval    |= (priority << shift);
  putreg32(regval, regaddr);

  ra_dumpnvic("prioritize", irq);
  return OK;
}

#endif
