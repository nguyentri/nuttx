# ##############################################################################
# boards/arm/ra8/fpb-ra8e1/src/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

set(SRCS ra8e1_boot.c ra8e1_bringup.c ra8e1_app_init.c)

if(CONFIG_ARCH_LEDS)
  list(APPEND SRCS ra8e1_auto_leds.c)
else()
  list(APPEND SRCS ra8e1_user_leds.c)
endif()

if(CONFIG_RA8E1_SWITCH)
  list(APPEND SRCS ra8e1_buttons.c)
endif()

############################################################################
# Peripheral initialization, User Driver interfaces on FPB-RA8E1
############################################################################
if(CONFIG_RA_SPI)
  list(APPEND SRCS ra8e1_spi.c)
endif()

############################################################################
# Application specific support CONFIG_BOARDCTL
############################################################################
if(CONFIG_RA8E1_ADC_BMS_DEMO)
  list(APPEND SRCS ra8e1_adc_bms_demo.c)
endif()

if(CONFIG_RA8E1_CODE_FLASH_DEMO)
  list(APPEND SRCS ra8e1_code_flash_demo.c)
endif()

if(CONFIG_RA8E1_DATA_FLASH_DEMO)
  list(APPEND SRCS ra8e1_data_flash_demo.c)
endif()

if(CONFIG_RA8E1_GPS_DEMO)
  list(APPEND SRCS ra8e1_gps_demo.c)
endif()

if(CONFIG_RA8E1_SBUS_DEMO)
  list(APPEND SRCS ra8e1_sbus_demo.c)
endif()

if(CONFIG_RA8E1_I2C_GY912_DEMO)
  list(APPEND SRCS ra8e1_i2c_gy912_demo.c)
endif()

if(CONFIG_RA8E1_I2C_TEST)
  list(APPEND SRCS ra8e1_i2c_test.c)
endif()

if(CONFIG_RA8E1_I2C_ACCEL_DEMO)
  list(APPEND SRCS ra8e1_i2c_acc_demo.c)
endif()

if(CONFIG_RA8E1_PWM_ESCS_DEMO)
  list(APPEND SRCS ra8e1_pwm_escs_demo.c)
endif()

if(CONFIG_RA8E1_SPI_LOOPBACK_DEMO)
  list(APPEND SRCS ra8e1_spi_loopback_demo.c)
endif()

if(CONFIG_RA8E1_SPI_GY912_DEMO)
  list(APPEND SRCS ra8e1_spi_gy912_demo.c)
endif()


target_sources(board PRIVATE ${SRCS})

set_property(GLOBAL PROPERTY LD_SCRIPT
                             "${NUTTX_BOARD_DIR}/scripts/fpb-ra8e1.ld")
