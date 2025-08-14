/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_gps_demo.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/serial/serial.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "ra_gpio.h"
#include "ra_sci.h"
#include "ra8e1_demo_log.h"

/* GPS NMEA Parameters */

#define GPS_BAUDRATE            38400
#define GPS_UART_NUM            3
#define GPS_RX_BUFFER_SIZE      128
#define GPS_TX_BUFFER_SIZE      64

/* NMEA Sentence Types */

#define NMEA_MAX_SENTENCE_LEN   82
#define NMEA_MAX_FIELDS         20

/* GPS Fix Types */

#define GPS_FIX_INVALID         0
#define GPS_FIX_GPS             1
#define GPS_FIX_DGPS            2
#define GPS_FIX_PPS             3
#define GPS_FIX_RTK             4
#define GPS_FIX_FLOAT_RTK       5
#define GPS_FIX_ESTIMATED       6
#define GPS_FIX_MANUAL          7
#define GPS_FIX_SIMULATION      8

#ifdef CONFIG_RA8E1_GPS_DEMO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NMEA Parser State */
struct nmea_parser_s
{
  char sentence[NMEA_MAX_SENTENCE_LEN]; /* Current sentence buffer */
  uint8_t sentence_pos;                 /* Current position in sentence */
  bool sentence_ready;                  /* Sentence ready for parsing */
  uint8_t checksum;                     /* Calculated checksum */
  bool checksum_mode;                   /* In checksum mode */
  uint32_t sentence_count;              /* Total sentence count */
  uint32_t parse_errors;                /* Parse error count */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NMEA parser state */
static struct nmea_parser_s g_nmea_parser;

/* UART device for GPS */
static ra_sci_dev_t g_gps_uart;

/* GPS data */
static struct gps_data_s g_gps_data;

/* DMA buffers */
static uint8_t g_gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static uint8_t g_gps_tx_buffer[GPS_TX_BUFFER_SIZE];

/* Demo running flag */
static volatile bool g_demo_running = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int gps_uart_initialize(void);
static void gps_uart_callback(ra_sci_dev_t *dev, uint32_t event);
static void nmea_process_byte(uint8_t byte);
static bool nmea_parse_sentence(const char *sentence);
static bool nmea_checksum_valid(const char *sentence);
static int nmea_parse_gga(const char *sentence);
static int nmea_parse_rmc(const char *sentence);
static int nmea_parse_gsa(const char *sentence);
static double nmea_parse_coordinate(const char *field, const char *dir_field);
static float nmea_parse_float(const char *field);
static int nmea_parse_int(const char *field);
static void nmea_split_fields(const char *sentence, char fields[][16], int *field_count);
static void gps_print_position(void);
static void gps_print_status(void);
static void process_rtt_command(const char *command);
static void print_menu(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gps_uart_initialize
 *
 * Description:
 *   Initialize UART for GPS communication
 *
 ****************************************************************************/

static int gps_uart_initialize(void)
{
  ra_sci_config_t config;
  int ret;

  /* Configure UART for GPS */
  memset(&config, 0, sizeof(config));
  config.base = R_SCI3_BASE;
  config.baud = GPS_BAUDRATE;
  config.bits = 8;
  config.parity = 0; /* No parity */
  config.stop = 1;   /* 1 stop bit */
  
  /* Pin configuration for UART3 */
  config.tx_pin = RA_GPIO_PIN(RA_GPIO_PORT3, 10); /* P310 - TXD3 */
  config.rx_pin = RA_GPIO_PIN(RA_GPIO_PORT3, 9);  /* P309 - RXD3 */
  
  /* DMA configuration */
  config.rx_dma.enabled = true;
  config.rx_dma.channel = 2;
  config.tx_dma.enabled = true;
  config.tx_dma.channel = 3;
  
  /* Set up device structure */
  g_gps_uart.config = &config;
  g_gps_uart.rx_buffer = g_gps_rx_buffer;
  g_gps_uart.rx_buffer_size = GPS_RX_BUFFER_SIZE;
  g_gps_uart.tx_buffer = g_gps_tx_buffer;
  g_gps_uart.tx_buffer_size = GPS_TX_BUFFER_SIZE;
  g_gps_uart.callback = gps_uart_callback;
  
  /* Initialize UART with DMA */
  ret = ra_sci_initialize(&g_gps_uart);
  if (ret < 0)
    {
      demoerr("GPS: Failed to initialize UART: %d\n", ret);
      return ret;
    }
  
  demoinfo("GPS: UART3 initialized for 38400 baud, 8N1\n");
  return OK;
}

/****************************************************************************
 * Name: gps_uart_callback
 *
 * Description:
 *   UART callback for GPS data reception
 *
 ****************************************************************************/

static void gps_uart_callback(ra_sci_dev_t *dev, uint32_t event)
{
  int i;
  
  if (event & RA_UART_EVENT_RX_CHAR)
    {
      /* Process received bytes */
      for (i = 0; i < dev->rx_count; i++)
        {
          nmea_process_byte(dev->rx_buffer[i]);
        }
    }
  
  if (event & (RA_UART_EVENT_ERR_PARITY | RA_UART_EVENT_ERR_FRAMING | RA_UART_EVENT_ERR_OVERFLOW))
    {
      g_nmea_parser.parse_errors++;
      demowarn("GPS: UART error event: 0x%08x\n", event);
    }
}

/****************************************************************************
 * Name: nmea_process_byte
 *
 * Description:
 *   Process a single byte from GPS NMEA stream
 *
 ****************************************************************************/

static void nmea_process_byte(uint8_t byte)
{
  /* NMEA sentence parsing state machine */
  if (byte == NMEA_START_CHAR)
    {
      /* Start of new sentence */
      g_nmea_parser.sentence_pos = 0;
      g_nmea_parser.checksum = 0;
      g_nmea_parser.checksum_mode = false;
      g_nmea_parser.sentence_ready = false;
      g_nmea_parser.sentence[g_nmea_parser.sentence_pos++] = byte;
    }
  else if (g_nmea_parser.sentence_pos > 0)
    {
      /* Continue building sentence */
      if (byte == NMEA_CHECKSUM_CHAR)
        {
          g_nmea_parser.checksum_mode = true;
          g_nmea_parser.sentence[g_nmea_parser.sentence_pos++] = byte;
        }
      else if (byte == '\r' || byte == '\n')
        {
          /* End of sentence */
          g_nmea_parser.sentence[g_nmea_parser.sentence_pos] = '\0';
          g_nmea_parser.sentence_ready = true;
          
          /* Process complete sentence */
          if (nmea_parse_sentence(g_nmea_parser.sentence))
            {
              g_nmea_parser.sentence_count++;
            }
          else
            {
              g_nmea_parser.parse_errors++;
            }
          
          g_nmea_parser.sentence_pos = 0;
        }
      else if (g_nmea_parser.sentence_pos < NMEA_MAX_SENTENCE_LEN - 1)
        {
          g_nmea_parser.sentence[g_nmea_parser.sentence_pos++] = byte;
          
          /* Calculate checksum (exclude $ and * characters) */
          if (!g_nmea_parser.checksum_mode && byte != NMEA_START_CHAR)
            {
              g_nmea_parser.checksum ^= byte;
            }
        }
      else
        {
          /* Sentence too long, reset */
          g_nmea_parser.sentence_pos = 0;
          g_nmea_parser.parse_errors++;
        }
    }
}

/****************************************************************************
 * Name: nmea_parse_sentence
 *
 * Description:
 *   Parse complete NMEA sentence
 *
 ****************************************************************************/

static bool nmea_parse_sentence(const char *sentence)
{
  if (!sentence || strlen(sentence) < 6)
    {
      return false;
    }
  
  /* Validate checksum */
  if (!nmea_checksum_valid(sentence))
    {
      return false;
    }
  
  /* Parse specific sentence types */
  if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0)
    {
      return nmea_parse_gga(sentence) == OK;
    }
  else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0)
    {
      return nmea_parse_rmc(sentence) == OK;
    }
  else if (strncmp(sentence, "$GPGSA", 6) == 0 || strncmp(sentence, "$GNGSA", 6) == 0)
    {
      return nmea_parse_gsa(sentence) == OK;
    }
  
  /* Unknown sentence type, but valid checksum */
  return true;
}

/****************************************************************************
 * Name: nmea_checksum_valid
 *
 * Description:
 *   Validate NMEA sentence checksum
 *
 ****************************************************************************/

static bool nmea_checksum_valid(const char *sentence)
{
  const char *checksum_ptr;
  uint8_t calculated_checksum = 0;
  uint8_t received_checksum;
  int i;
  
  /* Find checksum delimiter */
  checksum_ptr = strchr(sentence, '*');
  if (!checksum_ptr || strlen(checksum_ptr) < 3)
    {
      return false;
    }
  
  /* Calculate checksum */
  for (i = 1; i < (checksum_ptr - sentence); i++)
    {
      calculated_checksum ^= sentence[i];
    }
  
  /* Parse received checksum */
  received_checksum = (uint8_t)strtol(checksum_ptr + 1, NULL, 16);
  
  return calculated_checksum == received_checksum;
}

/****************************************************************************
 * Name: nmea_split_fields
 *
 * Description:
 *   Split NMEA sentence into fields
 *
 ****************************************************************************/

static void nmea_split_fields(const char *sentence, char fields[][16], int *field_count)
{
  int len = strlen(sentence);
  int field_idx = 0;
  int char_idx = 0;
  int i;
  
  *field_count = 0;
  
  for (i = 0; i < len && field_idx < NMEA_MAX_FIELDS; i++)
    {
      if (sentence[i] == ',' || sentence[i] == '*')
        {
          fields[field_idx][char_idx] = '\0';
          field_idx++;
          char_idx = 0;
          (*field_count)++;
        }
      else if (char_idx < 15)
        {
          fields[field_idx][char_idx++] = sentence[i];
        }
    }
  
  if (field_idx < NMEA_MAX_FIELDS && char_idx > 0)
    {
      fields[field_idx][char_idx] = '\0';
      (*field_count)++;
    }
}

/****************************************************************************
 * Name: nmea_parse_gga
 *
 * Description:
 *   Parse GGA (Global Positioning System Fix Data) sentence
 *
 ****************************************************************************/

static int nmea_parse_gga(const char *sentence)
{
  char fields[NMEA_MAX_FIELDS][16];
  int field_count;
  
  nmea_split_fields(sentence, fields, &field_count);
  
  if (field_count < 15)
    {
      return -EINVAL;
    }
  
  /* Parse time (field 1) */
  if (strlen(fields[1]) >= 6)
    {
      char time_str[3];
      time_str[2] = '\0';
      
      strncpy(time_str, fields[1], 2);
      g_gps_data.hour = atoi(time_str);
      
      strncpy(time_str, fields[1] + 2, 2);
      g_gps_data.minute = atoi(time_str);
      
      strncpy(time_str, fields[1] + 4, 2);
      g_gps_data.second = atoi(time_str);
    }
  
  /* Parse latitude (fields 2, 3) */
  g_gps_data.latitude = nmea_parse_coordinate(fields[2], fields[3]);
  
  /* Parse longitude (fields 4, 5) */
  g_gps_data.longitude = nmea_parse_coordinate(fields[4], fields[5]);
  
  /* Parse fix quality (field 6) */
  g_gps_data.fix_type = nmea_parse_int(fields[6]);
  g_gps_data.fix_valid = g_gps_data.fix_type > 0;
  
  /* Parse satellites used (field 7) */
  g_gps_data.satellites_used = nmea_parse_int(fields[7]);
  
  /* Parse HDOP (field 8) */
  g_gps_data.hdop = nmea_parse_float(fields[8]);
  
  /* Parse altitude (field 9) */
  g_gps_data.altitude = nmea_parse_float(fields[9]);
  
  /* Update timestamp */
  g_gps_data.timestamp = up_systime();
  
  return OK;
}

/****************************************************************************
 * Name: nmea_parse_rmc
 *
 * Description:
 *   Parse RMC (Recommended Minimum) sentence
 *
 ****************************************************************************/

static int nmea_parse_rmc(const char *sentence)
{
  char fields[NMEA_MAX_FIELDS][16];
  int field_count;
  
  nmea_split_fields(sentence, fields, &field_count);
  
  if (field_count < 12)
    {
      return -EINVAL;
    }
  
  /* Parse status (field 2) */
  g_gps_data.fix_valid = (fields[2][0] == 'A');
  
  /* Parse speed (field 7) - convert from knots to km/h */
  g_gps_data.speed_kmh = nmea_parse_float(fields[7]) * 1.852f;
  
  /* Parse course (field 8) */
  g_gps_data.course = nmea_parse_float(fields[8]);
  
  /* Parse date (field 9) */
  if (strlen(fields[9]) >= 6)
    {
      char date_str[3];
      date_str[2] = '\0';
      
      strncpy(date_str, fields[9], 2);
      g_gps_data.day = atoi(date_str);
      
      strncpy(date_str, fields[9] + 2, 2);
      g_gps_data.month = atoi(date_str);
      
      strncpy(date_str, fields[9] + 4, 2);
      g_gps_data.year = 2000 + atoi(date_str);
    }
  
  return OK;
}

/****************************************************************************
 * Name: nmea_parse_gsa
 *
 * Description:
 *   Parse GSA (GPS DOP and active satellites) sentence
 *
 ****************************************************************************/

static int nmea_parse_gsa(const char *sentence)
{
  char fields[NMEA_MAX_FIELDS][16];
  int field_count;
  
  nmea_split_fields(sentence, fields, &field_count);
  
  if (field_count < 18)
    {
      return -EINVAL;
    }
  
  /* Parse PDOP (field 15) */
  g_gps_data.pdop = nmea_parse_float(fields[15]);
  
  /* Parse HDOP (field 16) */
  g_gps_data.hdop = nmea_parse_float(fields[16]);
  
  /* Parse VDOP (field 17) */
  g_gps_data.vdop = nmea_parse_float(fields[17]);
  
  return OK;
}

/****************************************************************************
 * Name: nmea_parse_coordinate
 *
 * Description:
 *   Parse NMEA coordinate (DDMM.MMMMM format)
 *
 ****************************************************************************/

static double nmea_parse_coordinate(const char *field, const char *dir_field)
{
  double coord = 0.0;
  double degrees, minutes;
  
  if (strlen(field) < 4)
    {
      return 0.0;
    }
  
  /* Parse degrees and minutes */
  if (strlen(field) > 4)
    {
      /* Latitude: DDMM.MMMMM */
      degrees = (field[0] - '0') * 10 + (field[1] - '0');
      minutes = atof(field + 2);
    }
  else
    {
      /* Longitude: DDDMM.MMMMM */
      degrees = (field[0] - '0') * 100 + (field[1] - '0') * 10 + (field[2] - '0');
      minutes = atof(field + 3);
    }
  
  coord = degrees + minutes / 60.0;
  
  /* Apply direction */
  if (dir_field[0] == 'S' || dir_field[0] == 'W')
    {
      coord = -coord;
    }
  
  return coord;
}

/****************************************************************************
 * Name: nmea_parse_float
 *
 * Description:
 *   Parse float from NMEA field
 *
 ****************************************************************************/

static float nmea_parse_float(const char *field)
{
  if (!field || strlen(field) == 0)
    {
      return 0.0f;
    }
  
  return (float)atof(field);
}

/****************************************************************************
 * Name: nmea_parse_int
 *
 * Description:
 *   Parse integer from NMEA field
 *
 ****************************************************************************/

static int nmea_parse_int(const char *field)
{
  if (!field || strlen(field) == 0)
    {
      return 0;
    }
  
  return atoi(field);
}

/****************************************************************************
 * Name: gps_print_position
 *
 * Description:
 *   Print GPS position information
 *
 ****************************************************************************/

static void gps_print_position(void)
{
  demoprintf("\nGPS Position:\n");
  demoprintf("  Fix: %s (Type: %d)\n", 
             g_gps_data.fix_valid ? "Valid" : "Invalid", g_gps_data.fix_type);
  demoprintf("  Latitude:  %.6f°\n", g_gps_data.latitude);
  demoprintf("  Longitude: %.6f°\n", g_gps_data.longitude);
  demoprintf("  Altitude:  %.1f m\n", g_gps_data.altitude);
  demoprintf("  Speed:     %.1f km/h\n", g_gps_data.speed_kmh);
  demoprintf("  Course:    %.1f°\n", g_gps_data.course);
  demoprintf("  Time:      %02d:%02d:%02d UTC\n", 
             g_gps_data.hour, g_gps_data.minute, g_gps_data.second);
  demoprintf("  Date:      %02d/%02d/%04d\n", 
             g_gps_data.day, g_gps_data.month, g_gps_data.year);
  demoprintf("  Satellites: %d\n", g_gps_data.satellites_used);
  demoprintf("  HDOP:      %.1f\n", g_gps_data.hdop);
  demoprintf("  VDOP:      %.1f\n", g_gps_data.vdop);
  demoprintf("  PDOP:      %.1f\n", g_gps_data.pdop);
}

/****************************************************************************
 * Name: gps_print_status
 *
 * Description:
 *   Print GPS reception status
 *
 ****************************************************************************/

static void gps_print_status(void)
{
  uint32_t current_time = up_systime();
  uint32_t time_since_last = current_time - g_gps_data.timestamp;
  
  demoprintf("\nGPS Status:\n");
  demoprintf("  Sentences received: %u\n", g_nmea_parser.sentence_count);
  demoprintf("  Parse errors: %u\n", g_nmea_parser.parse_errors);
  demoprintf("  Last update: %u ms ago\n", time_since_last);
  demoprintf("  Message rate: %.1f Hz\n", 
             g_nmea_parser.sentence_count > 0 ? (float)g_nmea_parser.sentence_count * 1000.0f / current_time : 0.0f);
}

/****************************************************************************
 * Name: print_menu
 *
 * Description:
 *   Print RTT command menu
 *
 ****************************************************************************/

static void print_menu(void)
{
  demoprintf("\nGPS Demo Commands:\n");
  demoprintf("  p - Show position\n");
  demoprintf("  s - Show status\n");
  demoprintf("  r - Reset counters\n");
  demoprintf("  h - Show this menu\n");
  demoprintf("  q - Quit demo\n");
}

/****************************************************************************
 * Name: process_rtt_command
 *
 * Description:
 *   Process RTT commands
 *
 ****************************************************************************/

static void process_rtt_command(const char *command)
{
  if (!command || strlen(command) == 0)
    {
      return;
    }
  
  switch (command[0])
    {
      case 'p':
      case 'P':
        gps_print_position();
        break;
        
      case 's':
      case 'S':
        gps_print_status();
        break;
        
      case 'r':
      case 'R':
        g_nmea_parser.sentence_count = 0;
        g_nmea_parser.parse_errors = 0;
        demoprintf("Counters reset\n");
        break;
        
      case 'h':
      case 'H':
        print_menu();
        break;
        
      case 'q':
      case 'Q':
        g_demo_running = false;
        demoinfo("Exiting GPS demo\n");
        break;
        
      default:
        demoprintf("Unknown command: %c\n", command[0]);
        print_menu();
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra8e1_gps_demo_init
 *
 * Description:
 *   Initialize the GPS demo
 *
 ****************************************************************************/

int ra8e1_gps_demo_init(void)
{
  /* GPS demo initialization is done within main function */
  return 0;
}

/****************************************************************************
 * Name: ra8e1_gps_demo_main
 *
 * Description:
 *   GPS demo main function
 *
 ****************************************************************************/

int ra8e1_gps_demo_main(int argc, char *argv[])
{
  int ret;
  int key;
  uint8_t cmd_pos = 0;
  
  demoprintf("\nRA8E1 GPS Demo Starting...\n");
  demoprintf("GPS TX: P310 (TXD3) -> GPS RX\n");
  demoprintf("GPS RX: P309 (RXD3) <- GPS TX\n");
  demoprintf("Configuration: 38400 baud, 8N1\n\n");
  
  /* Initialize NMEA parser */
  memset(&g_nmea_parser, 0, sizeof(g_nmea_parser));
  memset(&g_gps_data, 0, sizeof(g_gps_data));
  
  /* Initialize UART for GPS */
  ret = gps_uart_initialize();
  if (ret < 0)
    {
      demoerr("GPS: Failed to initialize UART: %d\n", ret);
      return ret;
    }
  
  print_menu();
  g_demo_running = true;
  
  demoinfo("Demo initialized - ready for commands\n");

  /* Main demo loop */
  while (g_demo_running)
    {
      /* Check for input */
      if (demo_haskey())
        {
          key = demo_getkey();
          if (key >= 0)
            {
              if (key == '\r' || key == '\n')
                {
                  if (cmd_pos > 0)
                    {
                      g_rtt_buffer[cmd_pos] = '\0';
                      process_rtt_command(g_rtt_buffer);
                      cmd_pos = 0;
                    }
                }
              else if (key == '\b' || key == 127) /* Backspace */
                {
                  if (cmd_pos > 0)
                    {
                      cmd_pos--;
                    }
                }
              else if (cmd_pos < RTT_BUFFER_SIZE - 1)
                {
                  g_rtt_buffer[cmd_pos++] = key;
                }
            }
        }
      
      /* Small delay to prevent overwhelming the system */
      usleep(10000); /* 10ms */
    }
  
  /* Cleanup */
  ra_sci_finalize(&g_gps_uart);
  demoinfo("GPS demo finished\n");
  
  return OK;
}

/****************************************************************************
 * Name: ra8e1_gps_parse
 *
 * Description:
 *   Parse GPS NMEA sentence (public interface)
 *
 ****************************************************************************/

int ra8e1_gps_parse(const char *nmea_sentence, struct gps_data_s *gps_data)
{
  if (!nmea_sentence || !gps_data)
    {
      return -EINVAL;
    }
  
  /* This is a simplified interface - in practice, you would 
   * maintain state and parse multiple sentence types */
  
  if (strncmp(nmea_sentence, "$GPGGA", 6) == 0 || strncmp(nmea_sentence, "$GNGGA", 6) == 0)
    {
      /* Copy current data and parse GGA */
      memcpy(gps_data, &g_gps_data, sizeof(struct gps_data_s));
      return nmea_parse_gga(nmea_sentence);
    }
  
  return -ENOTSUP;
}

#endif /* CONFIG_RA8E1_GPS_DEMO */
