/****************************************************************************
 * arch/arm/src/ra8/ra_clock.h
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

#ifndef __ARCH_ARM_SRC_RA_CLOCKCONFIG_H
#define __ARCH_ARM_SRC_RA_CLOCKCONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FSP-Based Clock Configuration Defaults */

#ifndef CONFIG_RA_HOCO_FREQUENCY
#  define CONFIG_RA_HOCO_FREQUENCY    20000000  /* 20MHz HOCO (default for RA8E1) */
#endif

#ifndef CONFIG_RA_MOCO_FREQUENCY
#  define CONFIG_RA_MOCO_FREQUENCY    8000000   /* 8MHz MOCO */
#endif

#ifndef CONFIG_RA_LOCO_FREQUENCY
#  define CONFIG_RA_LOCO_FREQUENCY    32768     /* 32.768kHz LOCO */
#endif

#ifndef CONFIG_RA_XTAL_FREQUENCY
#  define CONFIG_RA_XTAL_FREQUENCY    20000000  /* 20MHz External Crystal */
#endif

#ifndef CONFIG_RA_MAIN_OSC_FREQUENCY
#  define CONFIG_RA_MAIN_OSC_FREQUENCY CONFIG_RA_XTAL_FREQUENCY
#endif

/* FSP-Based System Clock Selection */

/**
 * X=Integer portion of the multiplier.
 * Y=Fractional portion of the multiplier.
 */
 #define RA_CLOCKS_PLL_MUL(X, Y)    ((((X) -1U) << 2UL) | ((Y) == 50U ? 3U : ((Y) / 33UL)))

#ifndef RA_CKSEL
#  ifdef CONFIG_RA_CLOCK_PLL1P
#    define RA_CKSEL                   5         /* PLL1P */
#  elif defined(CONFIG_RA_CLOCK_PLL)
#    define RA_CKSEL                   5         /* PLL */
#  elif defined(CONFIG_RA_CLOCK_HOCO)
#    define RA_CKSEL                   0         /* HOCO */
#  elif defined(CONFIG_RA_CLOCK_MOCO)
#    define RA_CKSEL                   1         /* MOCO */
#  elif defined(CONFIG_RA_CLOCK_MAIN_OSC)
#    define RA_CKSEL                   3         /* Main OSC */
#  else
#    define RA_CKSEL                   0         /* Default to HOCO */
#  endif
#endif

/* Clock Source Definitions (FSP Compatible) */
#define RA_CLOCKS_SOURCE_CLOCK_HOCO      0     /* HOCO */
#define RA_CLOCKS_SOURCE_CLOCK_MOCO      1     /* MOCO */
#define RA_CLOCKS_SOURCE_CLOCK_LOCO      2     /* LOCO */
#define RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC  3     /* Main OSC */
#define RA_CLOCKS_SOURCE_CLOCK_SUBCLOCK  4     /* Sub-clock */
#define RA_CLOCKS_SOURCE_CLOCK_PLL       5     /* PLL */
#define RA_CLOCKS_SOURCE_CLOCK_PLL1P     5     /* PLL1P (alias for PLL) */
#define RA_CLOCKS_SOURCE_CLOCK_PLL1Q     7     /* PLL1Q */
#define RA_CLOCKS_SOURCE_CLOCK_PLL1R     8     /* PLL1R */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2      6     /* PLL2 */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2P     6     /* PLL2P */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2Q     9     /* PLL2Q */
#define RA_CLOCKS_SOURCE_CLOCK_PLL2R    10     /* PLL2R */
#define RA_CLOCKS_CLOCK_DISABLED        15     /* Clock disabled */

/* System clock divider options. */
#define RA_CLOCKS_SYS_CLOCK_DIV_1                   (0)  // System clock divided by 1.
#define RA_CLOCKS_SYS_CLOCK_DIV_2                   (1)  // System clock divided by 2.
#define RA_CLOCKS_SYS_CLOCK_DIV_4                   (2)  // System clock divided by 4.
#define RA_CLOCKS_SYS_CLOCK_DIV_8                   (3)  // System clock divided by 8.
#define RA_CLOCKS_SYS_CLOCK_DIV_16                  (4)  // System clock divided by 16.
#define RA_CLOCKS_SYS_CLOCK_DIV_32                  (5)  // System clock divided by 32.
#define RA_CLOCKS_SYS_CLOCK_DIV_64                  (6)  // System clock divided by 64.
#define RA_CLOCKS_SYS_CLOCK_DIV_128                 (7)  // System clock divided by 128 (available for CLKOUT only).
#define RA_CLOCKS_SYS_CLOCK_DIV_3                   (8)  // System clock divided by 3.
#define RA_CLOCKS_SYS_CLOCK_DIV_6                   (9)  // System clock divided by 6.
#define RA_CLOCKS_SYS_CLOCK_DIV_12                  (10) // System clock divided by 12.
#define RA_CLOCKS_SYS_CLOCK_DIV_24                  (11) // System clock divided by 24.

/* USB clock divider options. */
#define RA_CLOCKS_USB_CLOCK_DIV_1                   (0)  // Divide USB source clock by 1
#define RA_CLOCKS_USB_CLOCK_DIV_2                   (1)  // Divide USB source clock by 2
#define RA_CLOCKS_USB_CLOCK_DIV_3                   (2)  // Divide USB source clock by 3
#define RA_CLOCKS_USB_CLOCK_DIV_4                   (3)  // Divide USB source clock by 4
#define RA_CLOCKS_USB_CLOCK_DIV_5                   (4)  // Divide USB source clock by 5
#define RA_CLOCKS_USB_CLOCK_DIV_6                   (5)  // Divide USB source clock by 6
#define RA_CLOCKS_USB_CLOCK_DIV_8                   (7)  // Divide USB source clock by 8
#define RA_CLOCKS_USB_CLOCK_DIV_10                  (9)  // Divide USB source clock by 10
#define RA_CLOCKS_USB_CLOCK_DIV_16                  (15) // Divide USB source clock by 16
#define RA_CLOCKS_USB_CLOCK_DIV_32                  (9)  // Divide USB source clock by 32

/* USB60 clock divider options. */
#define RA_CLOCKS_USB60_CLOCK_DIV_1                 (0)  // Divide USB60 source clock by 1
#define RA_CLOCKS_USB60_CLOCK_DIV_2                 (1)  // Divide USB60 source clock by 2
#define RA_CLOCKS_USB60_CLOCK_DIV_3                 (5)  // Divide USB60 source clock by 3
#define RA_CLOCKS_USB60_CLOCK_DIV_4                 (2)  // Divide USB60 source clock by 4
#define RA_CLOCKS_USB60_CLOCK_DIV_5                 (6)  // Divide USB60 source clock by 5
#define RA_CLOCKS_USB60_CLOCK_DIV_6                 (3)  // Divide USB66 source clock by 6
#define RA_CLOCKS_USB60_CLOCK_DIV_8                 (4)  // Divide USB60 source clock by 8
#define RA_CLOCKS_USB60_CLOCK_DIV_10                (7)  // Divide USB60 source clock by 10
#define RA_CLOCKS_USB60_CLOCK_DIV_16                (8)  // Divide USB60 source clock by 16
#define RA_CLOCKS_USB60_CLOCK_DIV_32                (9)  // Divide USB60 source clock by 32

/* GLCD clock divider options. */
#define RA_CLOCKS_LCD_CLOCK_DIV_1                   (0)  // Divide LCD source clock by 1
#define RA_CLOCKS_LCD_CLOCK_DIV_2                   (1)  // Divide LCD source clock by 2
#define RA_CLOCKS_LCD_CLOCK_DIV_3                   (5)  // Divide LCD source clock by 3
#define RA_CLOCKS_LCD_CLOCK_DIV_4                   (2)  // Divide LCD source clock by 4
#define RA_CLOCKS_LCD_CLOCK_DIV_5                   (6)  // Divide LCD source clock by 5
#define RA_CLOCKS_LCD_CLOCK_DIV_6                   (3)  // Divide LCD source clock by 6
#define RA_CLOCKS_LCD_CLOCK_DIV_8                   (4)  // Divide LCD source clock by 8
#define RA_CLOCKS_LCD_CLOCK_DIV_10                  (7)  // Divide LCD source clock by 10
#define RA_CLOCKS_LCD_CLOCK_DIV_16                  (8)  // Divide LCD source clock by 16
#define RA_CLOCKS_LCD_CLOCK_DIV_32                  (9)  // Divide LCD source clock by 32

/* OCTA clock divider options. */
#define RA_CLOCKS_OCTA_CLOCK_DIV_1                  (0)  // Divide OCTA source clock by 1
#define RA_CLOCKS_OCTA_CLOCK_DIV_2                  (1)  // Divide OCTA source clock by 2
#define RA_CLOCKS_OCTA_CLOCK_DIV_3                  (5)  // Divide OCTA source clock by 3
#define RA_CLOCKS_OCTA_CLOCK_DIV_4                  (2)  // Divide OCTA source clock by 4
#define RA_CLOCKS_OCTA_CLOCK_DIV_5                  (6)  // Divide OCTA source clock by 5
#define RA_CLOCKS_OCTA_CLOCK_DIV_6                  (3)  // Divide OCTA source clock by 6
#define RA_CLOCKS_OCTA_CLOCK_DIV_8                  (4)  // Divide OCTA source clock by 8
#define RA_CLOCKS_OCTA_CLOCK_DIV_10                 (7)  // Divide OCTA source clock by 10
#define RA_CLOCKS_OCTA_CLOCK_DIV_16                 (8)  // Divide OCTA source clock by 16
#define RA_CLOCKS_OCTA_CLOCK_DIV_32                 (9)  // Divide OCTA source clock by 32

/* CANFD clock divider options. */
#define RA_CLOCKS_CANFD_CLOCK_DIV_1                 (0)  // Divide CANFD source clock by 1
#define RA_CLOCKS_CANFD_CLOCK_DIV_2                 (1)  // Divide CANFD source clock by 2
#define RA_CLOCKS_CANFD_CLOCK_DIV_3                 (5)  // Divide CANFD source clock by 3
#define RA_CLOCKS_CANFD_CLOCK_DIV_4                 (2)  // Divide CANFD source clock by 4
#define RA_CLOCKS_CANFD_CLOCK_DIV_5                 (6)  // Divide CANFD source clock by 5
#define RA_CLOCKS_CANFD_CLOCK_DIV_6                 (3)  // Divide CANFD source clock by 6
#define RA_CLOCKS_CANFD_CLOCK_DIV_8                 (4)  // Divide CANFD source clock by 8
#define RA_CLOCKS_CANFD_CLOCK_DIV_10                (7)  // Divide CANFD source clock by 10
#define RA_CLOCKS_CANFD_CLOCK_DIV_16                (8)  // Divide CANFD source clock by 16
#define RA_CLOCKS_CANFD_CLOCK_DIV_32                (9)  // Divide CANFD source clock by 32

/* SCI clock divider options. */
#define RA_CLOCKS_SCI_CLOCK_DIV_1                   (0)  // Divide SCI source clock by 1
#define RA_CLOCKS_SCI_CLOCK_DIV_2                   (1)  // Divide SCI source clock by 2
#define RA_CLOCKS_SCI_CLOCK_DIV_3                   (5)  // Divide SCI source clock by 3
#define RA_CLOCKS_SCI_CLOCK_DIV_4                   (2)  // Divide SCI source clock by 4
#define RA_CLOCKS_SCI_CLOCK_DIV_5                   (6)  // Divide SCI source clock by 5
#define RA_CLOCKS_SCI_CLOCK_DIV_6                   (3)  // Divide SCI source clock by 6
#define RA_CLOCKS_SCI_CLOCK_DIV_8                   (4)  // Divide SCI source clock by 8
#define RA_CLOCKS_SCI_CLOCK_DIV_10                  (7)  // Divide SCI source clock by 10
#define RA_CLOCKS_SCI_CLOCK_DIV_16                  (8)  // Divide SCI source clock by 16
#define RA_CLOCKS_SCI_CLOCK_DIV_32                  (9)  // Divide SCI source clock by 32

/* SPI clock divider options. */
#define RA_CLOCKS_SPI_CLOCK_DIV_1                   (0)  // Divide SPI source clock by 1
#define RA_CLOCKS_SPI_CLOCK_DIV_2                   (1)  // Divide SPI source clock by 2
#define RA_CLOCKS_SPI_CLOCK_DIV_3                   (5)  // Divide SPI source clock by 3
#define RA_CLOCKS_SPI_CLOCK_DIV_4                   (2)  // Divide SPI source clock by 4
#define RA_CLOCKS_SPI_CLOCK_DIV_5                   (6)  // Divide SPI source clock by 5
#define RA_CLOCKS_SPI_CLOCK_DIV_6                   (3)  // Divide SPI source clock by 6
#define RA_CLOCKS_SPI_CLOCK_DIV_8                   (4)  // Divide SPI source clock by 8
#define RA_CLOCKS_SPI_CLOCK_DIV_10                  (7)  // Divide SPI source clock by 10
#define RA_CLOCKS_SPI_CLOCK_DIV_16                  (8)  // Divide SPI source clock by 16
#define RA_CLOCKS_SPI_CLOCK_DIV_32                  (9)  // Divide SPI source clock by 32

/* SCISPI clock divider options. */
#define RA_CLOCKS_SCISPI_CLOCK_DIV_1                (0)  // Divide SCISPI source clock by 1
#define RA_CLOCKS_SCISPI_CLOCK_DIV_2                (1)  // Divide SCISPI source clock by 2
#define RA_CLOCKS_SCISPI_CLOCK_DIV_4                (2)  // Divide SCISPI source clock by 4
#define RA_CLOCKS_SCISPI_CLOCK_DIV_6                (3)  // Divide SCISPI source clock by 6
#define RA_CLOCKS_SCISPI_CLOCK_DIV_8                (4)  // Divide SCISPI source clock by 8

/* GPT clock divider options. */
#define RA_CLOCKS_GPT_CLOCK_DIV_1                   (0)  // Divide GPT source clock by 1
#define RA_CLOCKS_GPT_CLOCK_DIV_2                   (1)  // Divide GPT source clock by 2
#define RA_CLOCKS_GPT_CLOCK_DIV_3                   (5)  // Divide GPT source clock by 3
#define RA_CLOCKS_GPT_CLOCK_DIV_4                   (2)  // Divide GPT source clock by 4
#define RA_CLOCKS_GPT_CLOCK_DIV_5                   (6)  // Divide GPT source clock by 5
#define RA_CLOCKS_GPT_CLOCK_DIV_6                   (3)  // Divide GPT source clock by 6
#define RA_CLOCKS_GPT_CLOCK_DIV_8                   (4)  // Divide GPT source clock by 8
#define RA_CLOCKS_GPT_CLOCK_DIV_10                  (7)  // Divide GPT source clock by 10
#define RA_CLOCKS_GPT_CLOCK_DIV_16                  (8)  // Divide GPT source clock by 16
#define RA_CLOCKS_GPT_CLOCK_DIV_32                  (9)  // Divide GPT source clock by 32

/* IIC clock divider options. */
#define RA_CLOCKS_IIC_CLOCK_DIV_1                   (0)  // Divide IIC source clock by 1
#define RA_CLOCKS_IIC_CLOCK_DIV_2                   (1)  // Divide IIC source clock by 2
#define RA_CLOCKS_IIC_CLOCK_DIV_4                   (2)  // Divide IIC source clock by 4
#define RA_CLOCKS_IIC_CLOCK_DIV_6                   (3)  // Divide IIC source clock by 6
#define RA_CLOCKS_IIC_CLOCK_DIV_8                   (4)  // Divide IIC source clock by 8

/* CEC clock divider options. */
#define RA_CLOCKS_CEC_CLOCK_DIV_1                   (0)  // Divide CEC source clock by 1
#define RA_CLOCKS_CEC_CLOCK_DIV_2                   (1)  // Divide CEC source clock by 2

/* I3C clock divider options. */
#define RA_CLOCKS_I3C_CLOCK_DIV_1                   (0)  // Divide I3C source clock by 1
#define RA_CLOCKS_I3C_CLOCK_DIV_2                   (1)  // Divide I3C source clock by 2
#define RA_CLOCKS_I3C_CLOCK_DIV_3                   (5)  // Divide I3C source clock by 3
#define RA_CLOCKS_I3C_CLOCK_DIV_4                   (2)  // Divide I3C source clock by 4
#define RA_CLOCKS_I3C_CLOCK_DIV_5                   (6)  // Divide I3C source clock by 5
#define RA_CLOCKS_I3C_CLOCK_DIV_6                   (3)  // Divide I3C source clock by 6
#define RA_CLOCKS_I3C_CLOCK_DIV_8                   (4)  // Divide I3C source clock by 8
#define RA_CLOCKS_I3C_CLOCK_DIV_10                  (7)  // Divide I3C source clock by 10
#define RA_CLOCKS_I3C_CLOCK_DIV_16                  (8)  // Divide I3C source clock by 16
#define RA_CLOCKS_I3C_CLOCK_DIV_32                  (9)  // Divide I3C source clock by 32

/* ADC clock divider options. */
#define RA_CLOCKS_ADC_CLOCK_DIV_1                   (0)  // Divide ADC source clock by 1
#define RA_CLOCKS_ADC_CLOCK_DIV_2                   (1)  // Divide ADC source clock by 2
#define RA_CLOCKS_ADC_CLOCK_DIV_3                   (5)  // Divide ADC source clock by 3
#define RA_CLOCKS_ADC_CLOCK_DIV_4                   (2)  // Divide ADC source clock by 4
#define RA_CLOCKS_ADC_CLOCK_DIV_5                   (6)  // Divide ADC source clock by 5
#define RA_CLOCKS_ADC_CLOCK_DIV_6                   (3)  // Divide ADC source clock by 6
#define RA_CLOCKS_ADC_CLOCK_DIV_8                   (4)  // Divide ADC source clock by 8
#define RA_CLOCKS_ADC_CLOCK_DIV_10                  (7)  // Divide ADC source clock by 10
#define RA_CLOCKS_ADC_CLOCK_DIV_16                  (8)  // Divide ADC source clock by 16
#define RA_CLOCKS_ADC_CLOCK_DIV_32                  (9)  // Divide ADC source clock by 32

/* ESW clock divider options. */
#define RA_CLOCKS_ESW_CLOCK_DIV_1                   (0)  // Divide ESW source clock by 1
#define RA_CLOCKS_ESW_CLOCK_DIV_2                   (1)  // Divide ESW source clock by 2
#define RA_CLOCKS_ESW_CLOCK_DIV_3                   (5)  // Divide ESW source clock by 3
#define RA_CLOCKS_ESW_CLOCK_DIV_4                   (2)  // Divide ESW source clock by 4
#define RA_CLOCKS_ESW_CLOCK_DIV_5                   (6)  // Divide ESW source clock by 5
#define RA_CLOCKS_ESW_CLOCK_DIV_6                   (3)  // Divide ESW source clock by 6
#define RA_CLOCKS_ESW_CLOCK_DIV_8                   (4)  // Divide ESW source clock by 8
#define RA_CLOCKS_ESW_CLOCK_DIV_10                  (7)  // Divide ESW source clock by 10
#define RA_CLOCKS_ESW_CLOCK_DIV_16                  (8)  // Divide ESW source clock by 16
#define RA_CLOCKS_ESW_CLOCK_DIV_32                  (9)  // Divide ESW source clock by 32

/* ESWPHY clock divider options. */
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_1                (0)  // Divide ESWPHY source clock by 1
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_2                (1)  // Divide ESWPHY source clock by 2
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_3                (5)  // Divide ESWPHY source clock by 3
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_4                (2)  // Divide ESWPHY source clock by 4
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_5                (6)  // Divide ESWPHY source clock by 5
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_6                (3)  // Divide ESWPHY source clock by 6
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_8                (4)  // Divide ESWPHY source clock by 8
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_10               (7)  // Divide ESWPHY source clock by 10
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_16               (8)  // Divide ESWPHY source clock by 16
#define RA_CLOCKS_ESWPHY_CLOCK_DIV_32               (9)  // Divide ESWPHY source clock by 32

/* ETHPHY clock divider options. */
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_1                (0)  // Divide ETHPHY source clock by 1
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_2                (1)  // Divide ETHPHY source clock by 2
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_3                (5)  // Divide ETHPHY source clock by 3
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_4                (2)  // Divide ETHPHY source clock by 4
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_5                (6)  // Divide ETHPHY source clock by 5
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_6                (3)  // Divide ETHPHY source clock by 6
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_8                (4)  // Divide ETHPHY source clock by 8
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_10               (7)  // Divide ETHPHY source clock by 10
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_16               (8)  // Divide ETHPHY source clock by 16
#define RA_CLOCKS_ETHPHY_CLOCK_DIV_32               (9)  // Divide ETHPHY source clock by 32

/* BCLKA clock divider options. */
#define RA_CLOCKS_BCLKA_CLOCK_DIV_1                 (0)  // Divide BCLKA source clock by 1
#define RA_CLOCKS_BCLKA_CLOCK_DIV_2                 (1)  // Divide BCLKA source clock by 2
#define RA_CLOCKS_BCLKA_CLOCK_DIV_3                 (5)  // Divide BCLKA source clock by 3
#define RA_CLOCKS_BCLKA_CLOCK_DIV_4                 (2)  // Divide BCLKA source clock by 4
#define RA_CLOCKS_BCLKA_CLOCK_DIV_5                 (6)  // Divide BCLKA source clock by 5
#define RA_CLOCKS_BCLKA_CLOCK_DIV_6                 (3)  // Divide BCLKA source clock by 6
#define RA_CLOCKS_BCLKA_CLOCK_DIV_8                 (4)  // Divide BCLKA source clock by 8
#define RA_CLOCKS_BCLKA_CLOCK_DIV_10                (7)  // Divide BCLKA source clock by 10
#define RA_CLOCKS_BCLKA_CLOCK_DIV_16                (8)  // Divide BCLKA source clock by 16
#define RA_CLOCKS_BCLKA_CLOCK_DIV_32                (9)  // Divide BCLKA source clock by 32

/* SAU clock divider options. */
#define RA_CLOCKS_SAU_CLOCK_DIV_1                   (0)  // Divide SAU source clock by 1
#define RA_CLOCKS_SAU_CLOCK_DIV_2                   (1)  // Divide SAU source clock by 2
#define RA_CLOCKS_SAU_CLOCK_DIV_4                   (2)  // Divide SAU source clock by 4
#define RA_CLOCKS_SAU_CLOCK_DIV_8                   (3)  // Divide SAU source clock by 8
#define RA_CLOCKS_SAU_CLOCK_DIV_16                  (4)  // Divide SAU source clock by 16
#define RA_CLOCKS_SAU_CLOCK_DIV_32                  (5)  // Divide SAU source clock by 32
#define RA_CLOCKS_SAU_CLOCK_DIV_64                  (6)  // Divide SAU source clock by 64
#define RA_CLOCKS_SAU_CLOCK_DIV_128                 (7)  // Divide SAU source clock by 128
#define RA_CLOCKS_SAU_CLOCK_DIV_256                 (8)  // Divide SAU source clock by 256
#define RA_CLOCKS_SAU_CLOCK_DIV_512                 (9)  // Divide SAU source clock by 512
#define RA_CLOCKS_SAU_CLOCK_DIV_1024                (10) // Divide SAU source clock by 1024
#define RA_CLOCKS_SAU_CLOCK_DIV_2048                (11) // Divide SAU source clock by 2048
#define RA_CLOCKS_SAU_CLOCK_DIV_4096                (12) // Divide SAU source clock by 4096
#define RA_CLOCKS_SAU_CLOCK_DIV_8192                (13) // Divide SAU source clock by 8192
#define RA_CLOCKS_SAU_CLOCK_DIV_16384               (14) // Divide SAU source clock by 16384
#define RA_CLOCKS_SAU_CLOCK_DIV_32768               (15) // Divide SAU source clock by 32768

/* Extra peripheral 0 clock divider options. */
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_1     (0)  // Divide extra peripheral 0 source clock by 1
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_2     (1)  // Divide extra peripheral 0 source clock by 2
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_3     (5)  // Divide extra peripheral 0 source clock by 3
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_4     (2)  // Divide extra peripheral 0 source clock by 4
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_5     (6)  // Divide extra peripheral 0 source clock by 5
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_6     (3)  // Divide extra peripheral 0 source clock by 6
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_8     (4)  // Divide extra peripheral 0 source clock by 8
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_10    (7)  // Divide extra peripheral 0 source clock by 10
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_16    (8)  // Divide extra peripheral 0 source clock by 16
#define RA_CLOCKS_EXTRA_PERIPHERAL0_CLOCK_DIV_32    (9)  // Divide extra peripheral 0 source clock by 32

/* Extra peripheral 1 clock divider options. */
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_1     (0)  // Divide extra peripheral 1 source clock by 1
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_2     (1)  // Divide extra peripheral 1 source clock by 2
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_3     (5)  // Divide extra peripheral 1 source clock by 3
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_4     (2)  // Divide extra peripheral 1 source clock by 4
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_5     (6)  // Divide extra peripheral 1 source clock by 5
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_6     (3)  // Divide extra peripheral 1 source clock by 6
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_8     (4)  // Divide extra peripheral 1 source clock by 8
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_10    (7)  // Divide extra peripheral 1 source clock by 10
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_16    (8)  // Divide extra peripheral 1 source clock by 16
#define RA_CLOCKS_EXTRA_PERIPHERAL1_CLOCK_DIV_32    (9)  // Divide extra peripheral 1 source clock by 32

/* PLL divider options. */
#define RA_CLOCKS_PLL_DIV_1                         (0)
#define RA_CLOCKS_PLL_DIV_2                         (1)
#define RA_CLOCKS_PLL_DIV_3                         (2)
#define RA_CLOCKS_PLL_DIV_4                         (3)
#define RA_CLOCKS_PLL_DIV_5                         (4)
#define RA_CLOCKS_PLL_DIV_6                         (5)
#define RA_CLOCKS_PLL_DIV_8                         (7)
#define RA_CLOCKS_PLL_DIV_9                         (8)
#define RA_CLOCKS_PLL_DIV_1_5                       (9)
#define RA_CLOCKS_PLL_DIV_16                        (15)

/* PLL configuration macros (FSP compatible) */
#ifndef CONFIG_RA_PLL_SOURCE
#  define CONFIG_RA_PLL_SOURCE        RA_CLOCKS_SOURCE_CLOCK_HOCO  /* HOCO as PLL source */
#endif

#ifndef CONFIG_RA_PLL_DIV
#  define CONFIG_RA_PLL_DIV           1         /* PLL input divider /2 (FSP value 1 = /2) */
#endif

#ifndef CONFIG_RA_PLL_MUL
#  define CONFIG_RA_PLL_MUL           72        /* PLL multiplier x72 for 720MHz */
#endif

/* PLL frequency calculation */
#if CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_HOCO
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_HOCO_FREQUENCY
#elif CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_MAIN_OSC_FREQUENCY
#else
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_HOCO_FREQUENCY  /* Default */
#endif

#define RA_PLL_VCO_FREQUENCY              (RA_PLL_SOURCE_FREQ * CONFIG_RA_PLL_MUL / (CONFIG_RA_PLL_DIV + 1))

/* PLL output frequencies (based on BSP config) */
#define RA_CFG_PLL1P_FREQUENCY_HZ         (360000000)  /* PLL1P 360MHz */
#define RA_CFG_PLL1Q_FREQUENCY_HZ         (360000000)  /* PLL1Q 360MHz */
#define RA_CFG_PLL1R_FREQUENCY_HZ         (360000000)  /* PLL1R 360MHz */

/* Clock source selection */
#ifndef RA_CFG_CLOCK_SOURCE
#  ifdef CONFIG_RA_CLOCK_PLL1P
#    define RA_CFG_CLOCK_SOURCE            5         /* PLL1P */
#  elif defined(CONFIG_RA_CLOCK_PLL)
#    define RA_CFG_CLOCK_SOURCE            5         /* PLL */
#  elif defined(CONFIG_RA_CLOCK_HOCO)
#    define RA_CFG_CLOCK_SOURCE            0         /* HOCO */
#  elif defined(CONFIG_RA_CLOCK_MOCO)
#    define RA_CFG_CLOCK_SOURCE            1         /* MOCO */
#  elif defined(CONFIG_RA_CLOCK_MAIN_OSC)
#    define RA_CFG_CLOCK_SOURCE            3         /* Main OSC */
#  else
#    define RA_CFG_CLOCK_SOURCE            5         /* Default to PLL1P for RA8E1 */
#  endif
#endif

/* Clock settling delay */
#ifndef RA_CFG_CLOCK_SETTLING_DELAY_US
#  define RA_CFG_CLOCK_SETTLING_DELAY_US  150U
#endif

#ifndef CONFIG_RA_PLL_MUL
#  define CONFIG_RA_PLL_MUL           72        /* PLL multiplier x72 for 720MHz */
#endif

/* PLL Output Dividers (RA8E1 specific) */
#ifndef CONFIG_RA_PLL1P_DIV
#  define CONFIG_RA_PLL1P_DIV         1         /* PLL1P divider /2 (FSP value 1 = /2) */
#endif

#ifndef CONFIG_RA_PLL1Q_DIV
#  define CONFIG_RA_PLL1Q_DIV         1         /* PLL1Q divider /2 */
#endif

#ifndef CONFIG_RA_PLL1R_DIV
#  define CONFIG_RA_PLL1R_DIV         1         /* PLL1R divider /2 */
#endif

/* PLL2 Configuration */
#ifndef CONFIG_RA_PLL2_SOURCE
#  define CONFIG_RA_PLL2_SOURCE       RA_CLOCKS_CLOCK_DISABLED  /* PLL2 disabled by default */
#endif

#ifndef CONFIG_RA_PLL2_DIV
#  define CONFIG_RA_PLL2_DIV          1         /* PLL2 input divider */
#endif

#ifndef CONFIG_RA_PLL2_MUL
#  define CONFIG_RA_PLL2_MUL          96        /* PLL2 multiplier */
#endif

#ifndef CONFIG_RA_PLODIVR
#  define CONFIG_RA_PLODIVR (RA_CLOCKS_PLL_DIV_2) /* PLL1R Div /2 */
#endif

#ifndef CONFIG_RA_PLODIVP
#  define CONFIG_RA_PLODIVP (RA_CLOCKS_PLL_DIV_2) /* PLL1P Div /2 */
#endif

#ifndef CONFIG_RA_PLODIVQ
#  define CONFIG_RA_PLODIVQ (RA_CLOCKS_PLL_DIV_2) /* PLL1Q Div /2 */
#endif

/* PLL Frequency Calculations */
#if CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_HOCO
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_HOCO_FREQUENCY
#elif CONFIG_RA_PLL_SOURCE == RA_CLOCKS_SOURCE_CLOCK_MAIN_OSC
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_MAIN_OSC_FREQUENCY
#else
#  define RA_PLL_SOURCE_FREQ          CONFIG_RA_HOCO_FREQUENCY  /* Default */
#endif

#define RA_PLL_FREQUENCY              (RA_PLL_SOURCE_FREQ * CONFIG_RA_PLL_MUL / (CONFIG_RA_PLL_DIV + 1))
#define RA_PLL1P_FREQUENCY            (RA_PLL_FREQUENCY / (CONFIG_RA_PLL1P_DIV + 1))
#define RA_PLL1Q_FREQUENCY            (RA_PLL_FREQUENCY / (CONFIG_RA_PLL1Q_DIV + 1))
#define RA_PLL1R_FREQUENCY            (RA_PLL_FREQUENCY / (CONFIG_RA_PLL1R_DIV + 1))

/* FSP-Based Clock Frequencies */

#if defined(CONFIG_RA_CLOCK_PLL1P) || defined(CONFIG_RA_CLOCK_PLL)
#  define RA_SYSTEM_CLOCK_FREQUENCY    RA_PLL1P_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_HOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_HOCO_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_MOCO)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_MOCO_FREQUENCY
#elif defined(CONFIG_RA_CLOCK_MAIN_OSC)
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_MAIN_OSC_FREQUENCY
#else
#  define RA_SYSTEM_CLOCK_FREQUENCY    CONFIG_RA_HOCO_FREQUENCY
#endif

#ifndef CONFIG_RA_CPUCLK_DIV
#  define CONFIG_RA_CPUCLK_DIV        RA_CLOCKS_SYS_CLOCK_DIV_1   /* CPU clock = System clock / 1 */
#endif

#ifndef CONFIG_RA_ICK_DIV
#  define CONFIG_RA_ICK_DIV           RA_CLOCKS_SYS_CLOCK_DIV_3   /* ICLK = System clock / 3 */
#endif

#ifndef CONFIG_RA_PCKA_DIV
#  define CONFIG_RA_PCKA_DIV          RA_CLOCKS_SYS_CLOCK_DIV_3   /* PCKA = System clock / 3 */
#endif

#ifndef CONFIG_RA_PCKB_DIV
#  define CONFIG_RA_PCKB_DIV          RA_CLOCKS_SYS_CLOCK_DIV_6   /* PCKB = System clock / 6 */
#endif

#ifndef CONFIG_RA_PCKC_DIV
#  define CONFIG_RA_PCKC_DIV          RA_CLOCKS_SYS_CLOCK_DIV_6   /* PCKC = System clock / 6 */
#endif

#ifndef CONFIG_RA_PCKD_DIV
#  define CONFIG_RA_PCKD_DIV          RA_CLOCKS_SYS_CLOCK_DIV_3   /* PCKD = System clock / 3 */
#endif

#ifndef CONFIG_RA_PCKЕ_DIV
#  define CONFIG_RA_PCKЕ_DIV          RA_CLOCKS_SYS_CLOCK_DIV_3   /* PCKE = System clock / 3 */
#endif

#ifndef CONFIG_RA_BCLK_DIV
#  define CONFIG_RA_BCLK_DIV          RA_CLOCKS_SYS_CLOCK_DIV_12  /* BCLK = System clock / 12 */
#endif

#ifndef CONFIG_RA_FCLK_DIV
#  define CONFIG_RA_FCLK_DIV          RA_CLOCKS_SYS_CLOCK_DIV_6   /* FCLK = System clock / 6 */
#endif

/* Peripheral Clock Sources and Dividers */
#ifndef CONFIG_RA_SCICLK_SOURCE
#  define CONFIG_RA_SCICLK_SOURCE     RA_CLOCKS_SOURCE_CLOCK_PLL1P /* SCI clock source */
#endif

#ifndef CONFIG_RA_SCICLK_DIV
#  define CONFIG_RA_SCICLK_DIV        RA_CLOCKS_SCI_CLOCK_DIV_4   /* SCI clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_SPICLK_SOURCE
#  define CONFIG_RA_SPICLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* SPI clock disabled */
#endif

#ifndef CONFIG_RA_CANFDCLK_SOURCE
#  define CONFIG_RA_CANFDCLK_SOURCE   RA_CLOCKS_CLOCK_DISABLED    /* CANFD clock disabled */
#endif

#ifndef CONFIG_RA_USBCLK_SOURCE
#  define CONFIG_RA_USBCLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* USB clock disabled */
#endif

#ifndef CONFIG_RA_OCTACLK_SOURCE
#  define CONFIG_RA_OCTACLK_SOURCE    RA_CLOCKS_CLOCK_DISABLED    /* OCTA clock disabled */
#endif

/* Peripheral Clock Divider Configurations */
#ifndef CONFIG_RA_SPICLK_DIV
#  define CONFIG_RA_SPICLK_DIV        RA_CLOCKS_SPI_CLOCK_DIV_4                           /* SPI clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_CANFDCLK_DIV
#  define CONFIG_RA_CANFDCLK_DIV      RA_CLOCKS_CANFD_CLOCK_DIV_8                           /* CANFD clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_OCTACLK_DIV
#  define CONFIG_RA_OCTACLK_DIV       RA_CLOCKS_OCTA_CLOCK_DIV_4                           /* OCTA clock div /2 (FSP val 1 = /2) */
#endif

#ifndef CONFIG_RA_USBCLK_DIV
#  define CONFIG_RA_USBCLK_DIV        RA_CLOCKS_USB_CLOCK_DIV_5                           /* USB clock div /4 (FSP val 3 = /4) */
#endif

#ifndef CONFIG_RA_SCISPICLK_SOURCE
#  define CONFIG_RA_SCISPICLK_SOURCE  RA_CLOCKS_CLOCK_DISABLED    /* SCISPI clock disabled */
#endif

#ifndef CONFIG_RA_SCISPICLK_DIV
#  define CONFIG_RA_SCISPICLK_DIV     2                           /* SCISPI clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_GPTCLK_SOURCE
#  define CONFIG_RA_GPTCLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* GPT clock disabled */
#endif

#ifndef CONFIG_RA_GPTCLK_DIV
#  define CONFIG_RA_GPTCLK_DIV        2                           /* GPT clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_IICCLK_SOURCE
#  define CONFIG_RA_IICCLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* IIC clock disabled */
#endif

#ifndef CONFIG_RA_IICCLK_DIV
#  define CONFIG_RA_IICCLK_DIV        2                           /* IIC clock div /4 (FSP val 2 = /4) */
#endif

#ifndef CONFIG_RA_ADCCLK_SOURCE
#  define CONFIG_RA_ADCCLK_SOURCE     RA_CLOCKS_CLOCK_DISABLED    /* ADC clock disabled */
#endif

#ifndef CONFIG_RA_ADCCLK_DIV
#  define CONFIG_RA_ADCCLK_DIV        2                           /* ADC clock div /4 (FSP val 2 = /4) */
#endif

/* Clock Divider Shift Positions for Register Settings */
#define RA_ICK_DIV                     (CONFIG_RA_ICK_DIV << 24)
#define RA_FCK_DIV                     (CONFIG_RA_FCLK_DIV << 28)
#define RA_PCKA_DIV                    (CONFIG_RA_PCKA_DIV << 12)
#define RA_PCKB_DIV                    (CONFIG_RA_PCKB_DIV << 8)
#define RA_PCKC_DIV                    (CONFIG_RA_PCKC_DIV << 4)
#define RA_PCKD_DIV                    (CONFIG_RA_PCKD_DIV << 0)

/* Helper macro to convert divider value to actual divisor */
#define RA_DIV_TO_DIVISOR(div) \
  (((div) >= 8) ? (((div) == 8) ? 3 : \
                   ((div) == 9) ? 6 : \
                   ((div) == 10) ? 12 : \
                   ((div) == 11) ? 24 : (1 << (div))) : (1 << (div)))

/* FSP-Based Derived Frequencies */

#define RA_CPUCLK_FREQUENCY            (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_CPUCLK_DIV))
#define RA_ICLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_ICK_DIV))
#define RA_PCLKA_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKA_DIV))
#define RA_PCLKB_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKB_DIV))
#define RA_PCLKC_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKC_DIV))
#define RA_PCLKD_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKD_DIV))
#define RA_PCLKE_FREQUENCY             (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_PCKЕ_DIV))
#define RA_BCLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_BCLK_DIV))
#define RA_FCLK_FREQUENCY              (RA_SYSTEM_CLOCK_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_FCLK_DIV))

/* Peripheral Clock Frequencies */
#if CONFIG_RA_SCICLK_SOURCE == RA_CLOCKS_SOURCE_CLOCK_PLL1P
#  define RA_SCICLK_FREQUENCY          (RA_PLL1P_FREQUENCY / RA_DIV_TO_DIVISOR(CONFIG_RA_SCICLK_DIV))
#elif CONFIG_RA_SCICLK_SOURCE == RA_CLOCKS_CLOCK_DISABLED
#  define RA_SCICLK_FREQUENCY          0
#else
#  define RA_SCICLK_FREQUENCY          RA_PCLKB_FREQUENCY  /* Default to PCLKB */
#endif

/* FSP-Based Option Function Select Register Settings */

/* HOCO Frequency Options for RA8E1 */
#ifndef RA_HOCO_FREQUENCY
#  if CONFIG_RA_HOCO_FREQUENCY == 16000000
#    define RA_HOCO_FREQUENCY          0
#  elif CONFIG_RA_HOCO_FREQUENCY == 18000000
#    define RA_HOCO_FREQUENCY          1
#  elif CONFIG_RA_HOCO_FREQUENCY == 20000000
#    define RA_HOCO_FREQUENCY          2
#  elif CONFIG_RA_HOCO_FREQUENCY == 24000000
#    define RA_HOCO_FREQUENCY          4
#  elif CONFIG_RA_HOCO_FREQUENCY == 32000000
#    define RA_HOCO_FREQUENCY          5
#  elif CONFIG_RA_HOCO_FREQUENCY == 48000000
#    define RA_HOCO_FREQUENCY          7
#  elif CONFIG_RA_HOCO_FREQUENCY == 64000000
#    define RA_HOCO_FREQUENCY          8
#  else
#    define RA_HOCO_FREQUENCY          2         /* Default 20MHz for RA8E1 */
#  endif
#endif

#ifndef RA_HOCOEN
#  ifdef CONFIG_RA_HOCO_ENABLE
#    define RA_HOCOEN                  0         /* HOCO enabled */
#  else
#    define RA_HOCOEN                  1         /* HOCO disabled */
#  endif
#endif

/* Key code for writing PRCR register. */
#define RA_PRCR_KEY            (0xA500U)
#define RA_PRCR_PRC1_UNLOCK    ((RA_PRCR_KEY) | 0x2U)
#define RA_PRCR_UNLOCK         ((RA_PRCR_KEY) | 0x3U)
#define RA_PRCR_LOCK           ((RA_PRCR_KEY) | 0x0U)

/* FSP-Based ID Code Configuration */
#ifndef IDCODE1
#  define IDCODE1                      0xFFFFFFFF
#endif

#ifndef IDCODE2
#  define IDCODE2                      0xFFFFFFFF
#endif

#ifndef IDCODE3
#  define IDCODE3                      0xFFFFFFFF
#endif

#ifndef IDCODE4
#  define IDCODE4                      0xFFFFFFFF
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* FSP-Compatible Clock Configuration Structure */

typedef struct
{
  uint32_t system_clock_freq;
  uint32_t cpu_clock_freq;
  uint32_t iclk_freq;
  uint32_t pclka_freq;
  uint32_t pclkb_freq;
  uint32_t pclkc_freq;
  uint32_t pclkd_freq;
  uint32_t pclke_freq;
  uint32_t bclk_freq;
  uint32_t fclk_freq;
  uint32_t sciclk_freq;
  uint32_t hoco_frequency;
  uint8_t clock_source;
  bool hoco_enabled;
  bool moco_enabled;
  bool pll_enabled;
} ra_clock_config_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* FSP-compatible system core clock variable */
extern uint32_t g_sys_core_clock;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: ra_clock
 *
 * Description:
 *   Called to initialize the RA clocks. This does whatever setup is needed
 *   to put the SoC in a usable state using FSP-compatible clock
 *   initialization sequence.
 *
 ****************************************************************************/

void ra_clock(void);

/****************************************************************************
 * Name: ra_sys_core_clock_update
 *
 * Description:
 *   Update the system core clock frequency based on current clock settings.
 *   FSP-compatible implementation.
 *
 ****************************************************************************/

void ra_sys_core_clock_update(void);

/****************************************************************************
 * Name: ra_get_clock_config
 *
 * Description:
 *   Get current FSP-compatible clock configuration
 *
 ****************************************************************************/

void ra_get_clock_config(ra_clock_config_t *config);

/****************************************************************************
 * Name: ra_print_clock_info
 *
 * Description:
 *   Print FSP-compatible clock information for debugging
 *
 ****************************************************************************/

void ra_print_clock_info(void);

/****************************************************************************
 * Name: ra_get_peripheral_clock
 *
 * Description:
 *   Get peripheral clock frequency for FSP compatibility
 *
 ****************************************************************************/

uint32_t ra_get_peripheral_clock(int peripheral_id);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RA_CLOCKCONFIG_H */
