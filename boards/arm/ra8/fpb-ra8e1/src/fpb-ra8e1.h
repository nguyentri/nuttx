/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/fpb-ra8e1.h
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

#ifndef __BOARDS_ARM_RA8_FPBA8E1_SRC_H
#define __BOARDS_ARM_RA8_FPBA8E1_SRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int ra8e1_bringup(void);

/****************************************************************************
 * Name: ra_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_RA8_GPIO
int ra_gpio_initialize(void);
#endif

#ifdef CONFIG_RA8_SPI
int fpb_ra8e1_spi_initialize(void);
#endif

#ifdef CONFIG_RA8_SPI_LOOPBACK_DEMO
int ra8e1_spi_loopback_demo_init(void);
int ra8e1_spi_loopback_demo_main(int argc, char *argv[]);
#endif

#ifdef CONFIG_RA8_ADC_BATTERY_MONITOR
int ra8e1_adc_bms_demo_init(void);
int ra8e1_adc_bms_demo_main(int argc, FAR char *argv[]);
#endif

#ifdef CONFIG_RA8_CODE_FLASH
int ra8e1_code_flash_demo_init(void);
int ra8e1_code_flash_demo_main(int argc, char *argv[]);
#endif

#ifdef CONFIG_RA8_DATA_FLASH
int ra8e1_data_flash_demo_init(void);
int ra8e1_data_flash_demo_main(int argc, char *argv[]);
#endif

#ifdef CONFIG_RA8_PWM_ESCS
int ra8e1_escs_demo_init(void);
int ra8e1_escs_demo_main(int argc, char *argv[]);
#endif

#ifdef CONFIG_RA8_SCI_UART
int ra8e1_gps_demo_init(void);
int ra8e1_gps_demo_main(int argc, char *argv[]);
int ra8e1_sbus_demo_init(void);
int ra8e1_sbus_demo_main(int argc, char *argv[]);
#endif

#ifdef CONFIG_RA8_I2C
int ra8e1_i2c_gy912_demo_init(void);
int ra8e1_i2c_gy912_demo_main(int argc, char *argv[]);
int ra8e1_i2c_acc_demo_init(void);
int ra8e1_i2c_acc_demo_main(int argc, char *argv[]);
int ra8e1_i2c_simple_demo_init(void);
int ra8e1_i2c_simple_demo_main(int argc, char *argv[]);
#endif

#ifdef CONFIG_RA8_SPI_GY912
int ra8e1_spi_gy912_demo_init(void);
int ra8e1_spi_gy912_demo_main(int argc, FAR char *argv[]);
#endif

#ifdef CONFIG_RA8_SW_BUTTON
void board_button_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_H */
