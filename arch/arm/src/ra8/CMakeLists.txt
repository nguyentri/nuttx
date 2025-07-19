# ##############################################################################
# arch/arm/src/ra8/CMakeLists.txt
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

# Required RA8 core files
set(SRCS
    ra_start.c
    ra_timerisr.c
    ra_clockconfig.c
    ra_irq.c
    ra_lowputc.c
    ra_icu.c
    ra_gpio.c
    ra_allocateheap.c
    ra_serial.c)

# Module Stop Control
if(CONFIG_RA_MSTP)
  list(APPEND SRCS ra_mstp.c)
endif()

# Pin mapping support
if(CONFIG_RA_PINMAP)
  list(APPEND SRCS ra_pinmap.c)
endif()

# ADC driver
if(CONFIG_RA_ADC)
  list(APPEND SRCS ra_adc.c)
endif()

# I2C driver
if(CONFIG_RA_I2C)
  list(APPEND SRCS ra_i2c.c)
endif()

# I2C slave driver
if(CONFIG_RA_I2C_SLAVE)
  list(APPEND SRCS ra_i2c_slave.c)
endif()

# DMAC driver
if(CONFIG_RA_DMAC)
  list(APPEND SRCS ra_dmac.c)
endif()

# DMA driver
if(CONFIG_RA_DMA)
  list(APPEND SRCS ra_dmac.c)
endif()

# SPI driver
if(CONFIG_RA_SPI)
  list(APPEND SRCS ra_spi.c)
endif()

# GPT timer driver
if(CONFIG_RA_GPT)
  list(APPEND SRCS ra_gpt.c)
endif()

# PWM driver
if(CONFIG_RA_PWM)
  list(APPEND SRCS ra_pwm.c)
endif()

# Enhanced UART+DMA driver
if(CONFIG_RA_SCI_UART)
  list(APPEND SRCS ra_uart.c)
endif()

# Flash memory driver
if(CONFIG_RA_FLASH)
  list(APPEND SRCS ra_flash.c)
endif()

target_sources(arch PRIVATE ${SRCS})