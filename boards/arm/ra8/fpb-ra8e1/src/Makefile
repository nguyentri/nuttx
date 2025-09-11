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
CSRCS = ra8e1_boot.c ra8e1_bringup.c ra8e1_app_init.c
CSRCS += fpb_ra8e1_boardinitialize.c

# Application initialization
ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += ra8e1_appinit.c
endif

# LED support
ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += ra8e1_auto_leds.c
else
CSRCS += ra8e1_user_leds.c
endif

# Button/Switch support
ifeq ($(CONFIG_RA8E1_SWITCH),y)
CSRCS += ra8e1_buttons.c
endif

############################################################################
# Application specific support CONFIG_BOARDCTL
############################################################################
ifeq ($(CONFIG_RA8E1_ADC_BMS_DEMO),y)
CSRCS += ra8e1_adc_bms_demo.c
endif

ifeq ($(CONFIG_RA8E1_CODE_FLASH_DEMO),y)
CSRCS += ra8e1_code_flash_demo.c
endif

ifeq ($(CONFIG_RA8E1_DATA_FLASH_DEMO),y)
CSRCS += ra8e1_data_flash_demo.c
endif

ifeq ($(CONFIG_RA8E1_GPS_DEMO),y)
CSRCS += ra8e1_gps_demo.c
endif

ifeq ($(CONFIG_RA8E1_SBUS_DEMO),y)
CSRCS += ra8e1_sbus_demo.c
endif

ifeq ($(CONFIG_RA8E1_I2C_GY912_DEMO),y)
CSRCS += ra8e1_i2c_gy912_demo.c
endif

ifeq ($(CONFIG_RA8E1_I2C_TEST),y)
CSRCS += ra8e1_i2c_test.c
endif

ifeq ($(CONFIG_RA8E1_I2C_ACCEL_DEMO),y)
CSRCS += ra8e1_i2c_acc_demo.c
endif

ifeq ($(CONFIG_RA8E1_PWM_ESCS_DEMO),y)
CSRCS += ra8e1_pwm_escs_demo.c
endif

# Data Flash Demo Support
ifeq ($(CONFIG_RA8E1_DATA_FLASH_DEMO),y)
CSRCS += ra8e1_data_flash_demo.c
endif

# Code Flash Demo Support
ifeq ($(CONFIG_RA8E1_CODE_FLASH_DEMO),y)
CSRCS += ra8e1_code_flash_demo.c
endif


# SPI Loopback Demo
ifeq ($(CONFIG_RA8E1_SPI_LOOPBACK_DEMO),y)
CSRCS += ra8e1_spi_loopback_demo.c
endif

# GY-912 sensor support
ifeq ($(CONFIG_RA8E1_SPI_GY912_DEMO),y)
CSRCS += ra8e1_spi_gy912_demo.c
endif

# RTC driver support
ifeq ($(CONFIG_RTC_DRIVER),y)
CSRCS += ra8e1_rtc.c
endif

# Rust library integration (Zephyr-style approach)
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

include $(TOPDIR)/boards/Board.mk
