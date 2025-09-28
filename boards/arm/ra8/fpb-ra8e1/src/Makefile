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

# Core board initialization files
CSRCS = ra8e1_boot.c ra8e1_bringup.c
CSRCS += ra8e1_app_examples.c

############################################################################
# Board-specific initialization
############################################################################

# Button/Switch support
ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += ra8e1_buttons.c
endif

# LED support
ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += ra8e1_auto_leds.c
else
CSRCS += ra8e1_user_leds.c
endif

############################################################################
# Driver support
############################################################################

# RTC driver support
ifeq ($(CONFIG_RTC_DRIVER),y)
CSRCS += ra8e1_rtc.c
endif

# SPI driver support
ifeq ($(CONFIG_RA_SPI),y)
# SPI board functions are now implemented directly in application files
# that override the weak functions from the ra_spi.c driver
endif

############################################################################
# Application specific support CONFIG_BOARDCTL
############################################################################
ifeq ($(CONFIG_RA8E1_ADC_BMS_EXAMPLE),y)
CSRCS += ra8e1_adc_bms.c
endif

ifeq ($(CONFIG_RA8E1_CODE_FLASH_EXAMPLE),y)
CSRCS += ra8e1_code_flash.c
endif

ifeq ($(CONFIG_RA8E1_DATA_FLASH_EXAMPLE),y)
CSRCS += ra8e1_data_flash.c
endif

ifeq ($(CONFIG_RA8E1_GPS_EXAMPLE),y)
CSRCS += ra8e1_uart_gps.c
endif

ifeq ($(CONFIG_RA8E1_SBUS_EXAMPLE),y)
CSRCS += ra8e1_uart_sbus.c
endif

ifeq ($(CONFIG_RA8E1_I2C_GY912_EXAMPLE),y)
CSRCS += ra8e1_i2c_gy912.c
endif

ifeq ($(CONFIG_RA8E1_I2C_ACCEL_EXAMPLE),y)
CSRCS += ra8e1_i2c_acc.c
endif

ifeq ($(CONFIG_RA8E1_GPT_PWM_EXAMPLE),y)
CSRCS += pwm_status.c
CSRCS += ra8e1_gpt_pwm.c
endif

ifeq ($(CONFIG_RA8E1_PWM_ESCS_EXAMPLE),y)
CSRCS += pwm_status.c
CSRCS += ra8e1_gpt_escs.c
endif

# SPI Loopback Demo
ifeq ($(CONFIG_RA8E1_SPI_LOOPBACK_EXAMPLE),y)
CSRCS += ra8e1_spi_loopback.c
endif

# SPI Loopback Demo
ifeq ($(CONFIG_RA8E1_SPI_MASTERSLAVE_EXAMPLE),y)
CSRCS += ra8e1_spi_masterslave.c
endif

# GY-912 sensor support
ifeq ($(CONFIG_RA8E1_SPI_GY912_EXAMPLE),y)
CSRCS += ra8e1_spi_gy912.c
endif

# Rust Demo Support
ifeq ($(CONFIG_RA8E1_RUST_EXAMPLE),y)
CSRCS += ra8e1_rust.c
# Rust library integration
RUST_LIB_DIR = $(TOPDIR)/boards/arm/ra8/fpb-ra8e1/rust_lib
RUST_TARGET_DIR = $(RUST_LIB_DIR)/target
RUST_TARGET = thumbv8m.main-none-eabihf
RUST_LIB_PATH = $(RUST_TARGET_DIR)/$(RUST_TARGET)/release/libra8e1_rust_lib.a

# Check if cargo is available
CARGO := $(shell which cargo 2>/dev/null)
ifneq ($(CARGO),)
  # Add Rust library to user libraries (gets linked at final stage)
  USERLIBS += $(RUST_LIB_PATH)

  # Build Rust library before C code (like add_dependencies in Zephyr)
  context:: rust_lib

  rust_lib: $(RUST_LIB_PATH)

  # Build Rust library with proper dependencies (like add_custom_command in Zephyr)
  $(RUST_LIB_PATH): $(RUST_LIB_DIR)/src/lib.rs $(RUST_LIB_DIR)/Cargo.toml
	@echo "Building board-level Rust library for $(RUST_TARGET)..."
	@cd $(RUST_LIB_DIR) && \
		CARGO_TARGET_DIR=$(RUST_TARGET_DIR) \
		$(CARGO) build --release --target $(RUST_TARGET)
	@echo "Rust library built: $(RUST_LIB_PATH)"
	@echo "Adding Rust library to USERLIBS for final linking"

  clean::
	@echo "Cleaning board-level Rust build artifacts..."
	@cd $(RUST_LIB_DIR) && $(CARGO) clean 2>/dev/null || true

  .PHONY: rust_lib
else
  $(warning "Cargo (Rust) not found. Rust integration disabled.")
endif
endif

include $(TOPDIR)/boards/Board.mk
