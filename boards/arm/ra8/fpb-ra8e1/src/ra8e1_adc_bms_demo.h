/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/include/ra8e1_adc_bms_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_INCLUDE_FPB_RA8E1_ADC_BMS_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_INCLUDE_FPB_RA8E1_ADC_BMS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
# include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC Channels *************************************************************/

/* Battery monitoring pins on FPB-RA8E1:
 * - P004 (AN000): Battery voltage through 5.7:1 voltage divider
 * - P003 (AN104): Battery current through ACS712-05B current sensor
 */

#define GPIO_ADC_BATT_VOLTAGE   (PORT_0 | PIN_4)   /* P004/AN000 - Battery voltage */
#define GPIO_ADC_BATT_CURRENT   (PORT_0 | PIN_3)   /* P003/AN104 - Battery current */

/* ADC Channel Numbers */

#define ADC_BATTERY_VOLTAGE_CHANNEL    0    /* AN000 */
#define ADC_BATTERY_CURRENT_CHANNEL    104  /* AN104 */

/* Board ADC Configuration **************************************************/

#define BOARD_ADC_VREF              3300    /* ADC reference voltage in mV */
#define BOARD_ADC_RESOLUTION        4096    /* 12-bit ADC resolution */

/* Battery Monitoring Configuration *****************************************/

/* Voltage divider ratio for battery voltage sensing (5.7:1) */

#define BATTERY_VOLTAGE_DIVIDER_RATIO    57   /* 5.7:1 ratio * 10 for integer math */

/* ACS712-05B current sensor specifications:
 * - Sensitivity: 185 mV/A
 * - Zero current output: Vcc/2
 * - Operating voltage: 5V (but can work with 3.3V)
 */

#define CURRENT_SENSOR_SENSITIVITY   185    /* mV per Amp */
#define CURRENT_SENSOR_ZERO_OFFSET   1650   /* mV at zero current (Vcc/2) */

/* Battery Voltage Thresholds (in millivolts) */

#define BATTERY_VOLTAGE_FULL         4200   /* 100% charge */
#define BATTERY_VOLTAGE_HIGH         4000   /* 75% charge */
#define BATTERY_VOLTAGE_MEDIUM       3800   /* 50% charge */
#define BATTERY_VOLTAGE_LOW          3600   /* 25% charge */
#define BATTERY_VOLTAGE_CRITICAL     3400   /* 5% charge */
#define BATTERY_VOLTAGE_EMPTY        3200   /* 0% charge */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Battery status structure */

struct battery_status_s
{
  uint32_t voltage_mv;      /* Battery voltage in millivolts */
  int32_t  current_ma;      /* Battery current in milliamps (positive = charging) */
  uint32_t power_mw;        /* Power consumption in milliwatts */
  uint8_t  percentage;      /* Battery charge percentage (0-100) */
  bool     is_charging;     /* True if battery is charging */
  bool     is_valid;        /* True if readings are valid */
};

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
 * Name: fpb_ra8e1_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int fpb_ra8e1_bringup(void);

/****************************************************************************
 * ADC Functions
 ****************************************************************************/

#ifdef CONFIG_RA8_ADC

/****************************************************************************
 * Name: ra8e1_adc_initialize
 *
 * Description:
 *   Initialize ADC subsystem for battery monitoring on FPB-RA8E1
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void ra8e1_adc_initialize(void);

/****************************************************************************
 * Name: ra8e1_read_battery_voltage
 *
 * Description:
 *   Read battery voltage in millivolts
 *
 * Input Parameters:
 *   voltage_mv - Pointer to store voltage value
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_read_battery_voltage(uint32_t *voltage_mv);

/****************************************************************************
 * Name: ra8e1_read_battery_current
 *
 * Description:
 *   Read battery current in milliamps
 *
 * Input Parameters:
 *   current_ma - Pointer to store current value
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_read_battery_current(int32_t *current_ma);

/****************************************************************************
 * Name: ra8e1_get_battery_status
 *
 * Description:
 *   Get complete battery status information
 *
 * Input Parameters:
 *   status - Pointer to store battery status
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_get_battery_status(struct battery_status_s *status);

/****************************************************************************
 * Name: ra8e1_adc_sample_demo
 *
 * Description:
 *   Demonstration function showing ADC usage for battery monitoring
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra8e1_adc_sample_demo(void);

#endif /* CONFIG_RA8_ADC */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_INCLUDE_FPB_RA8E1_ADC_BMS_H */
