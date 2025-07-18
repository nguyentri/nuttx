/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/src/ra8e1_sbus_demo.h
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

#ifndef __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_SBUS_DEMO_H
#define __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_SBUS_DEMO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SBUS Protocol Parameters */

#define SBUS_FRAME_SIZE         25       /* SBUS frame size in bytes */
#define SBUS_NUM_CHANNELS       16       /* Number of SBUS channels */
#define SBUS_BAUDRATE           100000   /* 100 kbps */
#define SBUS_HEADER_BYTE        0x0F     /* SBUS header byte */
#define SBUS_FOOTER_BYTE        0x00     /* SBUS footer byte */

/* SBUS Channel Values */

#define SBUS_CHANNEL_MIN        172      /* Minimum channel value */
#define SBUS_CHANNEL_MID        992      /* Middle channel value */
#define SBUS_CHANNEL_MAX        1811     /* Maximum channel value */

/* SBUS Flags */

#define SBUS_FLAG_CH17          (1 << 0) /* Digital channel 17 */
#define SBUS_FLAG_CH18          (1 << 1) /* Digital channel 18 */
#define SBUS_FLAG_FRAME_LOST    (1 << 2) /* Frame lost flag */
#define SBUS_FLAG_FAILSAFE      (1 << 3) /* Failsafe flag */

/* UART Configuration */

#define SBUS_UART_NUM           2        /* UART2 for SBUS */
#define SBUS_RX_BUFFER_SIZE     64       /* RX buffer size */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* SBUS Data Structure */

struct sbus_data_s
{
  uint16_t channels[SBUS_NUM_CHANNELS]; /* Channel values */
  uint8_t flags;                        /* Status flags */
  bool frame_lost;                      /* Frame lost indicator */
  bool failsafe;                        /* Failsafe indicator */
  bool ch17;                            /* Digital channel 17 */
  bool ch18;                            /* Digital channel 18 */
  uint32_t timestamp;                   /* Frame timestamp */
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
 * Name: ra8e1_sbus_demo_main
 *
 * Description:
 *   SBUS demo main function
 *
 ****************************************************************************/

int ra8e1_sbus_demo_main(int argc, char *argv[]);

/****************************************************************************
 * Name: ra8e1_sbus_decode
 *
 * Description:
 *   Decode SBUS frame data
 *
 ****************************************************************************/

int ra8e1_sbus_decode(const uint8_t *sbus_data, struct sbus_data_s *decoded);

/****************************************************************************
 * Name: ra8e1_sbus_get_channel
 *
 * Description:
 *   Get specific channel value
 *
 ****************************************************************************/

uint16_t ra8e1_sbus_get_channel(const struct sbus_data_s *sbus_data, 
                                 uint8_t channel);

/****************************************************************************
 * Name: ra8e1_sbus_is_valid_frame
 *
 * Description:
 *   Check if SBUS frame is valid
 *
 ****************************************************************************/

bool ra8e1_sbus_is_valid_frame(const uint8_t *frame);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __BOARDS_ARM_RA8_FPB_RA8E1_SRC_RA8E1_SBUS_DEMO_H */
