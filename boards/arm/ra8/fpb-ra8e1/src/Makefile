############################################################################
# boards/arm/ra8/fpb-ra8e1/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = ra8e1_boot.c ra8e1_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += ra8e1_auto_leds.c
else
CSRCS += ra8e1_user_leds.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += ra8e1_app_init.c
endif

ifeq ($(CONFIG_RA_ADC),y)
CSRCS += ra8e1_adc_bms_demo.c
endif

ifeq ($(CONFIG_RA8E1_ADC_BMS_DEMO),y)
CSRCS += ra8e1_adc_bms_demo.c
endif

ifeq ($(CONFIG_RA_CODE_FLASH),y)
CSRCS += ra8e1_code_flash_demo.c
endif

ifeq ($(CONFIG_RA_DATA_FLASH),y)
CSRCS += ra8e1_data_flash_demo.c
endif

ifeq ($(CONFIG_RA_PWM),y)
CSRCS += ra8e1_escs_demo.c
endif

ifeq ($(CONFIG_RA_SCI_UART),y)
CSRCS += ra8e1_uart.c
endif

ifeq ($(CONFIG_RA8E1_GPS_DEMO),y)
CSRCS += ra8e1_gps_demo.c
endif

ifeq ($(CONFIG_RA8E1_SBUS_DEMO),y)
CSRCS += ra8e1_sbus_demo.c
endif

# I2C Demo Support
ifeq ($(CONFIG_RA8E1_I2C_GY912_DEMO),y)
CSRCS += ra8e1_i2c_gy912_demo.c
endif

ifeq ($(CONFIG_RA8E1_I2C_SIMPLE_DEMO),y)
CSRCS += ra8e1_i2c_simple_demo.c
endif

ifeq ($(CONFIG_RA8E1_I2C_ACCEL_DEMO),y)
CSRCS += ra8e1_i2c_acc_demo.c
endif

# PWM Demo Support
ifeq ($(CONFIG_RA8E1_PWM_DEMO),y)
CSRCS += ra8e1_escs_demo.c
endif

# Data Flash Demo Support
ifeq ($(CONFIG_RA8E1_DATA_FLASH_DEMO),y)
CSRCS += ra8e1_data_flash_demo.c
endif

# Code Flash Demo Support
ifeq ($(CONFIG_RA8E1_CODE_FLASH_DEMO),y)
CSRCS += ra8e1_code_flash_demo.c
endif

# SPI support
ifeq ($(CONFIG_RA_SPI),y)
CSRCS += ra8e1_spi.c
endif

# SPI Loopback Demo
ifeq ($(CONFIG_RA8E1_SPI_LOOPBACK_DEMO),y)
CSRCS += ra8e1_spi_loopback_demo.c
endif

# GY-912 sensor support  
ifeq ($(CONFIG_RA8E1_SPI_GY912_DEMO),y)
CSRCS += ra8e1_spi_gy912_demo.c
endif

# Button support
ifeq ($(CONFIG_RA8E1_SW_BUTTON),y)
CSRCS += ra8e1_buttons.c
endif

# ICU Input Capture support
ifeq ($(CONFIG_ICU_INPUT_CAPTURE),y)
CSRCS += ra8e1_user_icu.c
endif

include $(TOPDIR)/boards/Board.mk
