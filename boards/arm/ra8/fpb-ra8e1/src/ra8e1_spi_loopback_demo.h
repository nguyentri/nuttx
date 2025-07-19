/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_spi_loopback_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_SPI_LOOPBACK_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_SPI_LOOPBACK_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#include <stdint.h>
#include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI Configuration */
#define SPI_LOOPBACK_BUFFER_SIZE    32
#define SPI_LOOPBACK_FREQUENCY      1000000  /* 1 MHz */
#define SPI_LOOPBACK_MODE           SPIDEV_MODE0

/* Test Commands */
#define SPI_TEST_WRITE_AND_READ     1
#define SPI_TEST_WRITE_READ         2
#define SPI_TEST_EXIT               3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_spi_loopback_demo_init
 *
 * Description:
 *   Initialize SPI loopback demo with two SPI units
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_init(void);

/****************************************************************************
 * Name: ra8e1_spi_loopback_demo_test
 *
 * Description:
 *   Run SPI loopback tests between master and slave units
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_test(void);

/****************************************************************************
 * Name: ra8e1_spi_loopback_demo_main
 *
 * Description:
 *   Main entry point for SPI loopback demonstration
 *
 * Input Parameters:
 *   argc - Number of arguments
 *   argv - Argument array
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_main(int argc, char *argv[]);

/****************************************************************************
 * Name: ra8e1_spi_loopback_demo_cleanup
 *
 * Description:
 *   Clean up SPI loopback demo resources
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ra8e1_spi_loopback_demo_cleanup(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_SPI_LOOPBACK_DEMO_H */
