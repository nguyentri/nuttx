/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_sbus_demo.c
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
#include "ra_uart.h"
#include "ra8e1_demo_log.h"

/* SBUS Protocol Parameters */

#define SBUS_FRAME_SIZE         25
#define SBUS_NUM_CHANNELS       16
#define SBUS_BAUDRATE           100000
#define SBUS_HEADER_BYTE        0x0F
#define SBUS_FOOTER_BYTE        0x00

/* SBUS Channel Values */

#define SBUS_CHANNEL_MIN        172
#define SBUS_CHANNEL_MID        992
#define SBUS_CHANNEL_MAX        1811

/* SBUS Flags */

#define SBUS_FLAG_CH17          (1 << 0)
#define SBUS_FLAG_CH18          (1 << 1)
#define SBUS_FLAG_FRAME_LOST    (1 << 2)
#define SBUS_FLAG_FAILSAFE      (1 << 3)

/* UART Configuration */

#define SBUS_UART_NUM           2
#define SBUS_RX_BUFFER_SIZE     64

#ifdef CONFIG_RA8E1_SBUS_DEMO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SBUS Frame Structure */
#define SBUS_FRAME_SIZE         25       /* SBUS frame size in bytes */
#define SBUS_HEADER_BYTE        0x0F     /* SBUS header byte */
#define SBUS_FOOTER_BYTE        0x00     /* SBUS footer byte */
#define SBUS_NUM_CHANNELS       16       /* Number of SBUS channels */

/* SBUS Channel Values */
#define SBUS_CHANNEL_MIN        172      /* Minimum channel value */
#define SBUS_CHANNEL_MID        992      /* Middle channel value */
#define SBUS_CHANNEL_MAX        1811     /* Maximum channel value */

/* SBUS Flags */
#define SBUS_FLAG_CH17          (1 << 0) /* Digital channel 17 */
#define SBUS_FLAG_CH18          (1 << 1) /* Digital channel 18 */
#define SBUS_FLAG_FRAME_LOST    (1 << 2) /* Frame lost flag */
#define SBUS_FLAG_FAILSAFE      (1 << 3) /* Failsafe flag */

/* UART Configuration for SBUS */
#define SBUS_UART_NUM           2        /* UART2 for SBUS */
#define SBUS_BAUDRATE           100000   /* 100 kbps */
#define SBUS_DATABITS           8        /* 8 data bits */
#define SBUS_PARITY             2        /* Even parity */
#define SBUS_STOPBITS           2        /* 2 stop bits */
#define SBUS_INVERTED           true     /* Inverted signal */

/* Buffer Configuration */
#define SBUS_RX_BUFFER_SIZE     64       /* RX buffer size */
#define RTT_BUFFER_SIZE         64       /* RTT command buffer size */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* SBUS Frame Parser State */
struct sbus_parser_s
{
  uint8_t frame_buffer[SBUS_FRAME_SIZE]; /* Frame buffer */
  uint8_t frame_pos;                     /* Current position in frame */
  bool frame_sync;                       /* Frame synchronization */
  uint32_t last_frame_time;              /* Last frame timestamp */
  uint32_t frame_count;                  /* Total frame count */
  uint32_t error_count;                  /* Error count */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SBUS parser state */
static struct sbus_parser_s g_sbus_parser;

/* UART device for SBUS */
static ra_uart_dev_t g_sbus_uart;

/* SBUS data */
static struct sbus_data_s g_sbus_data;

/* DMA buffers */
static uint8_t g_sbus_rx_buffer[SBUS_RX_BUFFER_SIZE];

/* RTT command buffer */
static char g_rtt_buffer[RTT_BUFFER_SIZE];
static volatile bool g_demo_running = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sbus_uart_initialize(void);
static void sbus_uart_callback(ra_uart_dev_t *dev, uint32_t event);
static int sbus_parse_frame(const uint8_t *frame, struct sbus_data_s *data);
static bool sbus_validate_frame(const uint8_t *frame);
static void sbus_print_channels(const struct sbus_data_s *data);
static void sbus_print_status(void);
static void process_rtt_command(const char *command);
static void print_menu(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sbus_uart_initialize
 *
 * Description:
 *   Initialize UART for SBUS reception
 *
 ****************************************************************************/

static int sbus_uart_initialize(void)
{
  ra_uart_config_t config;
  int ret;

  /* Configure UART for SBUS */
  memset(&config, 0, sizeof(config));
  config.base = R_SCI2_BASE;
  config.baud = SBUS_BAUDRATE;
  config.bits = SBUS_DATABITS;
  config.parity = SBUS_PARITY;
  config.stop = SBUS_STOPBITS;
  
  /* Pin configuration for UART2 */
  config.rx_pin = RA_GPIO_PIN(RA_GPIO_PORT8, 2); /* P802 - RXD2 */
  config.tx_pin = 0; /* TX not used for SBUS reception */
  
  /* DMA configuration */
  config.rx_dma.enabled = true;
  config.rx_dma.channel = 0;
  
  /* Set up device structure */
  g_sbus_uart.config = &config;
  g_sbus_uart.rx_buffer = g_sbus_rx_buffer;
  g_sbus_uart.rx_buffer_size = SBUS_RX_BUFFER_SIZE;
  g_sbus_uart.callback = sbus_uart_callback;
  
  /* Initialize UART with DMA */
  ret = ra_uart_initialize(&g_sbus_uart);
  if (ret < 0)
    {
      demoprintf("SBUS: Failed to initialize UART: %d\n", ret);
      return ret;
    }
  
  demoprintf("SBUS: UART2 initialized for 100kbps, even parity, 2 stop bits\n");
  return OK;
}

/****************************************************************************
 * Name: sbus_uart_callback
 *
 * Description:
 *   UART callback for SBUS data reception
 *
 ****************************************************************************/

static void sbus_uart_callback(ra_uart_dev_t *dev, uint32_t event)
{
  uint8_t byte;
  int i;
  
  if (event & RA_UART_EVENT_RX_CHAR)
    {
      /* Process received bytes */
      for (i = 0; i < dev->rx_count; i++)
        {
          byte = dev->rx_buffer[i];
          
          /* SBUS frame parsing state machine */
          if (!g_sbus_parser.frame_sync)
            {
              /* Look for header byte */
              if (byte == SBUS_HEADER_BYTE)
                {
                  g_sbus_parser.frame_buffer[0] = byte;
                  g_sbus_parser.frame_pos = 1;
                  g_sbus_parser.frame_sync = true;
                }
            }
          else
            {
              /* Collect frame bytes */
              if (g_sbus_parser.frame_pos < SBUS_FRAME_SIZE)
                {
                  g_sbus_parser.frame_buffer[g_sbus_parser.frame_pos++] = byte;
                  
                  /* Check if frame is complete */
                  if (g_sbus_parser.frame_pos == SBUS_FRAME_SIZE)
                    {
                      /* Validate and parse frame */
                      if (sbus_validate_frame(g_sbus_parser.frame_buffer))
                        {
                          if (sbus_parse_frame(g_sbus_parser.frame_buffer, &g_sbus_data) == OK)
                            {
                              g_sbus_parser.frame_count++;
                              g_sbus_data.timestamp = up_systime();
                            }
                        }
                      else
                        {
                          g_sbus_parser.error_count++;
                        }
                      
                      /* Reset for next frame */
                      g_sbus_parser.frame_sync = false;
                      g_sbus_parser.frame_pos = 0;
                    }
                }
              else
                {
                  /* Frame too long, reset */
                  g_sbus_parser.frame_sync = false;
                  g_sbus_parser.frame_pos = 0;
                  g_sbus_parser.error_count++;
                }
            }
        }
    }
  
  if (event & (RA_UART_EVENT_ERR_PARITY | RA_UART_EVENT_ERR_FRAMING | RA_UART_EVENT_ERR_OVERFLOW))
    {
      g_sbus_parser.error_count++;
      demoprintf("SBUS: UART error event: 0x%08x\n", event);
    }
}

/****************************************************************************
 * Name: sbus_validate_frame
 *
 * Description:
 *   Validate SBUS frame
 *
 ****************************************************************************/

static bool sbus_validate_frame(const uint8_t *frame)
{
  /* Check header and footer */
  if (frame[0] != SBUS_HEADER_BYTE || frame[24] != SBUS_FOOTER_BYTE)
    {
      return false;
    }
  
  return true;
}

/****************************************************************************
 * Name: sbus_parse_frame
 *
 * Description:
 *   Parse SBUS frame into channel data
 *
 ****************************************************************************/

static int sbus_parse_frame(const uint8_t *frame, struct sbus_data_s *data)
{
  uint8_t flags;
  
  if (!frame || !data)
    {
      return -EINVAL;
    }
  
  /* Extract 16 channels from 11-bit packed data */
  data->channels[0]  = ((frame[1]    ) | (frame[2] << 8))                 & 0x07FF;
  data->channels[1]  = ((frame[2]>>3 ) | (frame[3] << 5))                 & 0x07FF;
  data->channels[2]  = ((frame[3]>>6 ) | (frame[4] << 2) | (frame[5]<<10)) & 0x07FF;
  data->channels[3]  = ((frame[5]>>1 ) | (frame[6] << 7))                 & 0x07FF;
  data->channels[4]  = ((frame[6]>>4 ) | (frame[7] << 4))                 & 0x07FF;
  data->channels[5]  = ((frame[7]>>7 ) | (frame[8] << 1) | (frame[9]<<9))  & 0x07FF;
  data->channels[6]  = ((frame[9]>>2 ) | (frame[10]<< 6))                 & 0x07FF;
  data->channels[7]  = ((frame[10]>>5) | (frame[11]<< 3))                 & 0x07FF;
  data->channels[8]  = ((frame[12]   ) | (frame[13]<< 8))                 & 0x07FF;
  data->channels[9]  = ((frame[13]>>3) | (frame[14]<< 5))                 & 0x07FF;
  data->channels[10] = ((frame[14]>>6) | (frame[15]<< 2) | (frame[16]<<10)) & 0x07FF;
  data->channels[11] = ((frame[16]>>1) | (frame[17]<< 7))                 & 0x07FF;
  data->channels[12] = ((frame[17]>>4) | (frame[18]<< 4))                 & 0x07FF;
  data->channels[13] = ((frame[18]>>7) | (frame[19]<< 1) | (frame[20]<<9))  & 0x07FF;
  data->channels[14] = ((frame[20]>>2) | (frame[21]<< 6))                 & 0x07FF;
  data->channels[15] = ((frame[21]>>5) | (frame[22]<< 3))                 & 0x07FF;
  
  /* Extract flags */
  flags = frame[23];
  data->ch17 = (flags & SBUS_FLAG_CH17) != 0;
  data->ch18 = (flags & SBUS_FLAG_CH18) != 0;
  data->frame_lost = (flags & SBUS_FLAG_FRAME_LOST) != 0;
  data->failsafe = (flags & SBUS_FLAG_FAILSAFE) != 0;
  data->flags = flags;
  
  return OK;
}

/****************************************************************************
 * Name: sbus_print_channels
 *
 * Description:
 *   Print SBUS channel values
 *
 ****************************************************************************/

static void sbus_print_channels(const struct sbus_data_s *data)
{
  int i;
  
  demoprintf("SBUS Channels:\n");
  for (i = 0; i < SBUS_NUM_CHANNELS; i += 4)
    {
      demoprintf("  CH%02d:%4d  CH%02d:%4d  CH%02d:%4d  CH%02d:%4d\n",
                 i+1, data->channels[i],
                 i+2, (i+1 < SBUS_NUM_CHANNELS) ? data->channels[i+1] : 0,
                 i+3, (i+2 < SBUS_NUM_CHANNELS) ? data->channels[i+2] : 0,
                 i+4, (i+3 < SBUS_NUM_CHANNELS) ? data->channels[i+3] : 0);
    }
  
  demoprintf("Digital: CH17=%d CH18=%d  Status: FrameLost=%d Failsafe=%d\n",
             data->ch17, data->ch18, data->frame_lost, data->failsafe);
}

/****************************************************************************
 * Name: sbus_print_status
 *
 * Description:
 *   Print SBUS reception status
 *
 ****************************************************************************/

static void sbus_print_status(void)
{
  uint32_t current_time = up_systime();
  uint32_t time_since_last = current_time - g_sbus_data.timestamp;
  
  demoprintf("\nSBUS Status:\n");
  demoprintf("  Frames received: %u\n", g_sbus_parser.frame_count);
  demoprintf("  Errors: %u\n", g_sbus_parser.error_count);
  demoprintf("  Last frame: %u ms ago\n", time_since_last);
  demoprintf("  Frame rate: %.1f Hz\n", 
             g_sbus_parser.frame_count > 0 ? (float)g_sbus_parser.frame_count * 1000.0f / current_time : 0.0f);
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
  demoprintf("\nSBUS Demo Commands:\n");
  demoprintf("  c - Show channels\n");
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
      case 'c':
      case 'C':
        sbus_print_channels(&g_sbus_data);
        break;
        
      case 's':
      case 'S':
        sbus_print_status();
        break;
        
      case 'r':
      case 'R':
        g_sbus_parser.frame_count = 0;
        g_sbus_parser.error_count = 0;
        demoprintf("Counters reset\n");
        break;
        
      case 'h':
      case 'H':
        print_menu();
        break;
        
      case 'q':
      case 'Q':
        g_demo_running = false;
        demoprintf("Exiting SBUS demo\n");
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
 * Name: ra8e1_sbus_demo_init
 *
 * Description:
 *   Initialize the SBUS demo
 *
 ****************************************************************************/

int ra8e1_sbus_demo_init(void)
{
  /* SBUS demo initialization is done within main function */
  return 0;
}

/****************************************************************************
 * Name: ra8e1_sbus_demo_main
 *
 * Description:
 *   SBUS demo main function
 *
 ****************************************************************************/

int ra8e1_sbus_demo_main(int argc, char *argv[])
{
  int ret;
  int key;
  uint8_t cmd_pos = 0;
  
  demoprintf("\nRA8E1 SBUS Demo Starting...\n");
  demoprintf("SBUS: P802 (RXD2) <- RC Receiver SBUS output\n");
  demoprintf("Configuration: 100kbps, 8E2, inverted\n\n");
  
  /* Initialize SBUS parser */
  memset(&g_sbus_parser, 0, sizeof(g_sbus_parser));
  memset(&g_sbus_data, 0, sizeof(g_sbus_data));
  
  /* Initialize UART for SBUS */
  ret = sbus_uart_initialize();
  if (ret < 0)
    {
      demoprintf("SBUS: Failed to initialize UART: %d\n", ret);
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
  ra_uart_finalize(&g_sbus_uart);
  demoprintf("SBUS demo finished\n");
  
  return OK;
}

/****************************************************************************
 * Name: ra8e1_sbus_decode
 *
 * Description:
 *   Decode SBUS frame (public interface)
 *
 ****************************************************************************/

int ra8e1_sbus_decode(const uint8_t *sbus_data, struct sbus_data_s *decoded)
{
  if (!sbus_data || !decoded)
    {
      return -EINVAL;
    }
  
  if (!sbus_validate_frame(sbus_data))
    {
      return -EINVAL;
    }
  
  return sbus_parse_frame(sbus_data, decoded);
}

/****************************************************************************
 * Name: ra8e1_sbus_get_channel
 *
 * Description:
 *   Get specific channel value
 *
 ****************************************************************************/

uint16_t ra8e1_sbus_get_channel(const struct sbus_data_s *sbus_data, 
                                 uint8_t channel)
{
  if (!sbus_data || channel >= SBUS_NUM_CHANNELS)
    {
      return 0;
    }
  
  return sbus_data->channels[channel];
}

/****************************************************************************
 * Name: ra8e1_sbus_is_valid_frame
 *
 * Description:
 *   Check if SBUS frame is valid
 *
 ****************************************************************************/

bool ra8e1_sbus_is_valid_frame(const uint8_t *frame)
{
  return sbus_validate_frame(frame);
}

#endif /* CONFIG_RA8E1_SBUS_DEMO */
