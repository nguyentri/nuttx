/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_gps_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_GPS_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_GPS_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPS NMEA Parameters */

#define GPS_BAUDRATE            38400    /* GPS baud rate */
#define GPS_UART_NUM            3        /* UART3 for GPS */
#define GPS_RX_BUFFER_SIZE      128      /* RX buffer size */
#define GPS_TX_BUFFER_SIZE      64       /* TX buffer size */

/* NMEA Sentence Types */

#define NMEA_MAX_SENTENCE_LEN   82       /* Maximum NMEA sentence length */
#define NMEA_MAX_FIELDS         20       /* Maximum fields in NMEA sentence */

/* GPS Fix Types */

#define GPS_FIX_INVALID         0        /* Invalid fix */
#define GPS_FIX_GPS             1        /* GPS fix */
#define GPS_FIX_DGPS            2        /* DGPS fix */
#define GPS_FIX_PPS             3        /* PPS fix */
#define GPS_FIX_RTK             4        /* RTK fix */
#define GPS_FIX_FLOAT_RTK       5        /* Float RTK fix */
#define GPS_FIX_ESTIMATED       6        /* Estimated fix */
#define GPS_FIX_MANUAL          7        /* Manual input mode */
#define GPS_FIX_SIMULATION      8        /* Simulation mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPS Data Structure */

struct gps_data_s
{
  /* Position data */
  double latitude;                      /* Latitude in degrees */
  double longitude;                     /* Longitude in degrees */
  float altitude;                       /* Altitude in meters */
  
  /* Time data */
  uint8_t hour;                         /* UTC hour */
  uint8_t minute;                       /* UTC minute */
  uint8_t second;                       /* UTC second */
  uint16_t millisecond;                 /* UTC millisecond */
  
  uint8_t day;                          /* UTC day */
  uint8_t month;                        /* UTC month */
  uint16_t year;                        /* UTC year */
  
  /* Quality data */
  uint8_t fix_type;                     /* Fix type */
  uint8_t satellites_used;              /* Number of satellites used */
  float hdop;                           /* Horizontal dilution of precision */
  float vdop;                           /* Vertical dilution of precision */
  float pdop;                           /* Position dilution of precision */
  
  /* Motion data */
  float speed_kmh;                      /* Speed in km/h */
  float course;                         /* Course over ground in degrees */
  
  /* Status */
  bool fix_valid;                       /* Fix validity */
  char talker_id[3];                    /* Talker ID (GP, GL, etc.) */
  uint32_t timestamp;                   /* System timestamp */
};

/* NMEA Parser State */

struct nmea_parser_s
{
  char sentence[NMEA_MAX_SENTENCE_LEN]; /* Current sentence buffer */
  uint8_t sentence_pos;                 /* Current position in sentence */
  bool sentence_ready;                  /* Sentence ready for parsing */
  uint8_t checksum;                     /* Calculated checksum */
  bool checksum_mode;                   /* In checksum mode */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
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
 * Name: ra8e1_gps_demo_main
 *
 * Description:
 *   GPS demo main function
 *
 ****************************************************************************/

int ra8e1_gps_demo_main(int argc, char *argv[]);

/****************************************************************************
 * Name: ra8e1_gps_parse
 *
 * Description:
 *   Parse NMEA sentence
 *
 ****************************************************************************/

int ra8e1_gps_parse(const char *nmea_sentence, struct gps_data_s *gps_data);

/****************************************************************************
 * Name: ra8e1_nmea_parse_gga
 *
 * Description:
 *   Parse GGA (Global Positioning System Fix Data) sentence
 *
 ****************************************************************************/

int ra8e1_nmea_parse_gga(const char *sentence, struct gps_data_s *gps_data);

/****************************************************************************
 * Name: ra8e1_nmea_parse_rmc
 *
 * Description:
 *   Parse RMC (Recommended Minimum) sentence
 *
 ****************************************************************************/

int ra8e1_nmea_parse_rmc(const char *sentence, struct gps_data_s *gps_data);

/****************************************************************************
 * Name: ra8e1_nmea_parse_gsa
 *
 * Description:
 *   Parse GSA (GPS DOP and active satellites) sentence
 *
 ****************************************************************************/

int ra8e1_nmea_parse_gsa(const char *sentence, struct gps_data_s *gps_data);

/****************************************************************************
 * Name: ra8e1_nmea_checksum_valid
 *
 * Description:
 *   Validate NMEA sentence checksum
 *
 ****************************************************************************/

bool ra8e1_nmea_checksum_valid(const char *sentence);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_GPS_DEMO_H */
