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

#define RA_ICU_MAX_IRQ_NUM    32    /* Maximum number of ICU IRQ lines */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static void *g_icu_handlers[RA_ICU_MAX_IRQ_NUM];
static void *g_icu_args[RA_ICU_MAX_IRQ_NUM];

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
  for (i = 0; i < RA_ICU_MAX_IRQ_NUM; i++)
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
  if (icu_irq < 0 || icu_irq >= RA_ICU_MAX_IRQ_NUM)
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
  if (icu_irq < 0 || icu_irq >= RA_ICU_MAX_IRQ_NUM)
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
  if (icu_irq >= 0 && icu_irq < RA_ICU_MAX_IRQ_NUM)
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
  if (icu_irq >= 0 && icu_irq < RA_ICU_MAX_IRQ_NUM)
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

  if (icu_slot < 0 || icu_slot >= R_ICU_IELSR_SIZE)
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

void ra_icu_attach_all(void)
{
  /* ADC Interrupt Configuration */
#ifdef CONFIG_RA_ADC
  /* ADC0 interrupts */
  putreg32(EVENT_ADC0_SCAN_END, R_ICU_IELSR(0));      /* ADC0 Scan End */
  putreg32(EVENT_ADC0_SCAN_END_B, R_ICU_IELSR(1));    /* ADC0 Scan End B */
  putreg32(EVENT_ADC0_WINDOW_A, R_ICU_IELSR(2));      /* ADC0 Window A */
  putreg32(EVENT_ADC0_WINDOW_B, R_ICU_IELSR(3));      /* ADC0 Window B */

  /* ADC1 interrupts */
  putreg32(EVENT_ADC1_SCAN_END, R_ICU_IELSR(4));      /* ADC1 Scan End */
  putreg32(EVENT_ADC1_SCAN_END_B, R_ICU_IELSR(5));    /* ADC1 Scan End B */
  putreg32(EVENT_ADC1_WINDOW_A, R_ICU_IELSR(6));      /* ADC1 Window A */
  putreg32(EVENT_ADC1_WINDOW_B, R_ICU_IELSR(7));      /* ADC1 Window B */
#endif

  /* GPT Timer/PWM Interrupt Configuration */
#ifdef CONFIG_RA_GPT
  /* GPT0 interrupts - commonly used for PWM and timing */
  /* putreg32(EVENT_GPT0_CAPTURE_COMPARE_A, R_ICU_IELSR(8)); */     /* GPT0 Compare A */
  /* putreg32(EVENT_GPT0_CAPTURE_COMPARE_B, R_ICU_IELSR(9)); */     /* GPT0 Compare B */
  /* putreg32(EVENT_GPT0_COUNTER_OVERFLOW, R_ICU_IELSR(10)); */     /* GPT0 Overflow */
  /* putreg32(EVENT_GPT0_COUNTER_UNDERFLOW, R_ICU_IELSR(11)); */    /* GPT0 Underflow */

  /* GPT3 interrupts - commonly used in examples */
  /* putreg32(EVENT_GPT3_CAPTURE_COMPARE_A, R_ICU_IELSR(12)); */    /* GPT3 Compare A */
  /* putreg32(EVENT_GPT3_CAPTURE_COMPARE_B, R_ICU_IELSR(13)); */    /* GPT3 Compare B */
  /* putreg32(EVENT_GPT3_COUNTER_OVERFLOW, R_ICU_IELSR(14)); */     /* GPT3 Overflow */
  /* putreg32(EVENT_GPT3_COUNTER_UNDERFLOW, R_ICU_IELSR(15)); */    /* GPT3 Underflow */
#endif

#ifdef CONFIG_RA_PWM
  /* Additional GPT channels for PWM operations */
  putreg32(EVENT_GPT1_CAPTURE_COMPARE_A, R_ICU_IELSR(16));    /* GPT1 Compare A */
  putreg32(EVENT_GPT1_COUNTER_OVERFLOW, R_ICU_IELSR(17));     /* GPT1 Overflow */
  putreg32(EVENT_GPT2_CAPTURE_COMPARE_A, R_ICU_IELSR(18));    /* GPT2 Compare A */
  putreg32(EVENT_GPT2_COUNTER_OVERFLOW, R_ICU_IELSR(19));     /* GPT2 Overflow */
#endif

  /* SPI Interrupt Configuration */
#ifdef CONFIG_RA_SPI
  /* SPI0 interrupts */
  putreg32(EVENT_SPI0_RXI, R_ICU_IELSR(20));          /* SPI0 RX */
  putreg32(EVENT_SPI0_TXI, R_ICU_IELSR(21));          /* SPI0 TX */
  putreg32(EVENT_SPI0_TEI, R_ICU_IELSR(22));          /* SPI0 Transfer End */
  putreg32(EVENT_SPI0_ERI, R_ICU_IELSR(23));          /* SPI0 Error */

  /* SPI1 interrupts */
  putreg32(EVENT_SPI1_RXI, R_ICU_IELSR(24));          /* SPI1 RX */
  putreg32(EVENT_SPI1_TXI, R_ICU_IELSR(25));          /* SPI1 TX */
  putreg32(EVENT_SPI1_TEI, R_ICU_IELSR(26));          /* SPI1 Transfer End */
  putreg32(EVENT_SPI1_ERI, R_ICU_IELSR(27));          /* SPI1 Error */
#endif

  /* I2C (IIC) Interrupt Configuration */
#ifdef CONFIG_RA_I2C
  /* IIC0 interrupts */
  putreg32(EVENT_IIC0_RXI, R_ICU_IELSR(28));          /* IIC0 RX */
  putreg32(EVENT_IIC0_TXI, R_ICU_IELSR(29));          /* IIC0 TX */
  putreg32(EVENT_IIC0_TEI, R_ICU_IELSR(30));          /* IIC0 Transfer End */
  putreg32(EVENT_IIC0_ERI, R_ICU_IELSR(31));          /* IIC0 Error */

  /* IIC1 interrupts */
  putreg32(EVENT_IIC1_RXI, R_ICU_IELSR(32));          /* IIC1 RX */
  putreg32(EVENT_IIC1_TXI, R_ICU_IELSR(33));          /* IIC1 TX */
  putreg32(EVENT_IIC1_TEI, R_ICU_IELSR(34));          /* IIC1 Transfer End */
  putreg32(EVENT_IIC1_ERI, R_ICU_IELSR(35));          /* IIC1 Error */
#endif

  /* DMAC Interrupt Configuration */
#ifdef CONFIG_RA_DMAC
  putreg32(EVENT_DMAC0_INT, R_ICU_IELSR(36));         /* DMAC0 Transfer End */
  putreg32(EVENT_DMAC1_INT, R_ICU_IELSR(37));         /* DMAC1 Transfer End */
  putreg32(EVENT_DMAC2_INT, R_ICU_IELSR(38));         /* DMAC2 Transfer End */
  putreg32(EVENT_DMAC3_INT, R_ICU_IELSR(39));         /* DMAC3 Transfer End */
  putreg32(EVENT_DMAC4_INT, R_ICU_IELSR(40));         /* DMAC4 Transfer End */
  putreg32(EVENT_DMAC5_INT, R_ICU_IELSR(41));         /* DMAC5 Transfer End */
  putreg32(EVENT_DMAC6_INT, R_ICU_IELSR(42));         /* DMAC6 Transfer End */
  putreg32(EVENT_DMAC7_INT, R_ICU_IELSR(43));         /* DMAC7 Transfer End */
  putreg32(EVENT_DMA_TRANSERR, R_ICU_IELSR(44));      /* DMA Transfer Error */
#endif

  /* DTC Interrupt Configuration */
#ifdef CONFIG_RA_DTC
  putreg32(EVENT_DTC_END, R_ICU_IELSR(45));           /* DTC Transfer End */
  putreg32(EVENT_DTC_COMPLETE, R_ICU_IELSR(46));      /* DTC Transfer Complete */
#endif

  /* UART (SCI) Interrupt Configuration */
#ifdef CONFIG_RA_SCI0_UART
  putreg32(EVENT_SCI0_RXI, R_ICU_IELSR(47));          /* SCI0 RX */
  putreg32(EVENT_SCI0_TXI, R_ICU_IELSR(48));          /* SCI0 TX */
  putreg32(EVENT_SCI0_TEI, R_ICU_IELSR(49));          /* SCI0 Transfer End */
  putreg32(EVENT_SCI0_ERI, R_ICU_IELSR(50));          /* SCI0 Error */
#endif

#ifdef CONFIG_RA_SCI1_UART
  putreg32(EVENT_SCI1_RXI, R_ICU_IELSR(51));          /* SCI1 RX */
  putreg32(EVENT_SCI1_TXI, R_ICU_IELSR(52));          /* SCI1 TX */
  putreg32(EVENT_SCI1_TEI, R_ICU_IELSR(53));          /* SCI1 Transfer End */
  putreg32(EVENT_SCI1_ERI, R_ICU_IELSR(54));          /* SCI1 Error */
#endif

#ifdef CONFIG_RA_SCI2_UART
  putreg32(EVENT_SCI2_RXI, R_ICU_IELSR(55));          /* SCI2 RX */
  putreg32(EVENT_SCI2_TXI, R_ICU_IELSR(56));          /* SCI2 TX */
  putreg32(EVENT_SCI2_TEI, R_ICU_IELSR(57));          /* SCI2 Transfer End */
  putreg32(EVENT_SCI2_ERI, R_ICU_IELSR(58));          /* SCI2 Error */
#endif

#ifdef CONFIG_RA_SCI3_UART
  putreg32(EVENT_SCI3_RXI, R_ICU_IELSR(59));          /* SCI3 RX */
  putreg32(EVENT_SCI3_TXI, R_ICU_IELSR(60));          /* SCI3 TX */
  putreg32(EVENT_SCI3_TEI, R_ICU_IELSR(61));          /* SCI3 Transfer End */
  putreg32(EVENT_SCI3_ERI, R_ICU_IELSR(62));          /* SCI3 Error */
#endif

#ifdef CONFIG_RA_SCI4_UART
  putreg32(EVENT_SCI4_RXI, R_ICU_IELSR(63));          /* SCI4 RX */
  putreg32(EVENT_SCI4_TXI, R_ICU_IELSR(64));          /* SCI4 TX */
  putreg32(EVENT_SCI4_TEI, R_ICU_IELSR(65));          /* SCI4 Transfer End */
  putreg32(EVENT_SCI4_ERI, R_ICU_IELSR(66));          /* SCI4 Error */
#endif

#ifdef CONFIG_RA_SCI9_UART
  putreg32(EVENT_SCI9_RXI, R_ICU_IELSR(67));          /* SCI9 RX */
  putreg32(EVENT_SCI9_TXI, R_ICU_IELSR(68));          /* SCI9 TX */
  putreg32(EVENT_SCI9_TEI, R_ICU_IELSR(69));          /* SCI9 Transfer End */
  putreg32(EVENT_SCI9_ERI, R_ICU_IELSR(70));          /* SCI9 Error */
#endif

  /* External Pin Interrupts for Button/GPIO */
#ifdef CONFIG_RA8E1_SWITCH
  putreg32(EVENT_ICU_IRQ13, R_ICU_IELSR(71));         /* SW1 button IRQ13 (P009) */
#endif

#ifdef CONFIG_RA_ICU
  /* Additional external interrupts can be configured here */
  putreg32(EVENT_ICU_IRQ0, R_ICU_IELSR(72));          /* External IRQ0 */
  putreg32(EVENT_ICU_IRQ1, R_ICU_IELSR(73));          /* External IRQ1 */
  putreg32(EVENT_ICU_IRQ2, R_ICU_IELSR(74));          /* External IRQ2 */
  putreg32(EVENT_ICU_IRQ3, R_ICU_IELSR(75));          /* External IRQ3 */
#endif

  /* Additional Timer Interrupts */
#ifdef CONFIG_RA_AGT
  putreg32(EVENT_AGT0_INT, R_ICU_IELSR(76));          /* AGT0 Interrupt */
  putreg32(EVENT_AGT0_COMPARE_A, R_ICU_IELSR(77));    /* AGT0 Compare A */
  putreg32(EVENT_AGT0_COMPARE_B, R_ICU_IELSR(78));    /* AGT0 Compare B */
  putreg32(EVENT_AGT1_INT, R_ICU_IELSR(79));          /* AGT1 Interrupt */
  putreg32(EVENT_AGT1_COMPARE_A, R_ICU_IELSR(80));    /* AGT1 Compare A */
  putreg32(EVENT_AGT1_COMPARE_B, R_ICU_IELSR(81));    /* AGT1 Compare B */
#endif

#ifdef CONFIG_RA_ULPT
  putreg32(EVENT_ULPT0_INT, R_ICU_IELSR(82));         /* ULPT0 Underflow */
  putreg32(EVENT_ULPT0_COMPARE_A, R_ICU_IELSR(83));   /* ULPT0 Compare A */
  putreg32(EVENT_ULPT0_COMPARE_B, R_ICU_IELSR(84));   /* ULPT0 Compare B */
#endif

  /* Flash Memory Interrupts */
#ifdef CONFIG_RA_FLASH
  putreg32(EVENT_FCU_FRDYI, R_ICU_IELSR(85));         /* Flash Ready Interrupt */
#endif

  /* Watchdog Timer Interrupts */
#ifdef CONFIG_RA_WDT
  putreg32(EVENT_WDT_UNDERFLOW, R_ICU_IELSR(86));     /* WDT Underflow */
  putreg32(EVENT_IWDT_UNDERFLOW, R_ICU_IELSR(87));    /* IWDT Underflow */
#endif

  /* RTC Interrupts */
#ifdef CONFIG_RA_RTC
  putreg32(EVENT_RTC_ALARM, R_ICU_IELSR(88));         /* RTC Alarm */
  putreg32(EVENT_RTC_PERIOD, R_ICU_IELSR(89));        /* RTC Periodic */
  putreg32(EVENT_RTC_CARRY, R_ICU_IELSR(90));         /* RTC Carry */
#endif

  /* USB Interrupts */
#ifdef CONFIG_RA_USB
  putreg32(EVENT_USBFS_INT, R_ICU_IELSR(91));         /* USB FS Interrupt */
  putreg32(EVENT_USBFS_RESUME, R_ICU_IELSR(92));      /* USB FS Resume */
  putreg32(EVENT_USBFS_FIFO_0, R_ICU_IELSR(93));      /* USB FS DMA 0 */
  putreg32(EVENT_USBFS_FIFO_1, R_ICU_IELSR(94));      /* USB FS DMA 1 */
#endif

  /* Comparator Interrupts */
#ifdef CONFIG_RA_ACMP
  putreg32(EVENT_ACMPLP0_INT, R_ICU_IELSR(95));       /* Low Power Comparator 0 */
  putreg32(EVENT_ACMPLP1_INT, R_ICU_IELSR(96));       /* Low Power Comparator 1 */
  putreg32(EVENT_ACMPHS0_INT, R_ICU_IELSR(97));       /* High Speed Comparator 0 */
  putreg32(EVENT_ACMPHS1_INT, R_ICU_IELSR(98));       /* High Speed Comparator 1 */
#endif

  /* CAN Interrupts */
#ifdef CONFIG_RA_CAN
  putreg32(EVENT_CAN0_ERROR, R_ICU_IELSR(99));        /* CAN0 Error */
  putreg32(EVENT_CAN0_FIFO_RX, R_ICU_IELSR(100));     /* CAN0 RX FIFO */
#endif

  /* GPT Timer Interrupts */
#ifdef CONFIG_RA_SYSTICK_GPT
  /* Map GPT3 overflow event for system timer tick */
  putreg32(EVENT_GPT3_COUNTER_OVERFLOW, R_ICU_IELSR(0));
#endif
}
#ifdef CONFIG_RA_SCI_UART
#  ifdef CONFIG_RA_SCI_UART_DMA_ENABLE
  /* Ensure DMAC channels 0/1 mapped for SCI2 TX/RX if needed */
  /* These events (DMAC0/1) already assigned above when CONFIG_RA_DMAC; keep slot numbers consistent */
#  endif
#endif

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
