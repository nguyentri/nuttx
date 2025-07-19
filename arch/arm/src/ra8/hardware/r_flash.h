/****************************************************************************
 * arch/arm/src/ra8/hardware/r_flash.h
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

#ifndef __ARCH_ARM_SRC_RA8_HARDWARE_R_FLASH_H
#define __ARCH_ARM_SRC_RA8_HARDWARE_R_FLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Flash Control Unit (FCU) Base Address */
#define RA8_FCU_BASE                    0x407E0000

/* Flash registers */
#define RA8_FLASH_FASTAT                (RA8_FCU_BASE + 0x0010)
#define RA8_FLASH_FAEINT                (RA8_FCU_BASE + 0x0014)
#define RA8_FLASH_FRDYIE                (RA8_FCU_BASE + 0x0018)
#define RA8_FLASH_FSADDR                (RA8_FCU_BASE + 0x0030)
#define RA8_FLASH_FEADDR                (RA8_FCU_BASE + 0x0034)
#define RA8_FLASH_FWEPROR               (RA8_FCU_BASE + 0x0040)
#define RA8_FLASH_FCURAME               (RA8_FCU_BASE + 0x0044)
#define RA8_FLASH_FSTATR                (RA8_FCU_BASE + 0x0080)
#define RA8_FLASH_FENTRYR               (RA8_FCU_BASE + 0x0084)
#define RA8_FLASH_FPROTR                (RA8_FCU_BASE + 0x0088)
#define RA8_FLASH_FRESETR               (RA8_FCU_BASE + 0x008C)
#define RA8_FLASH_FCMDR                 (RA8_FCU_BASE + 0x00A0)
#define RA8_FLASH_FPESTAT               (RA8_FCU_BASE + 0x00C0)
#define RA8_FLASH_FBCCNT                (RA8_FCU_BASE + 0x00D0)
#define RA8_FLASH_FBCSTAT               (RA8_FCU_BASE + 0x00D4)
#define RA8_FLASH_FPSADDR               (RA8_FCU_BASE + 0x00D8)
#define RA8_FLASH_FAWMON                (RA8_FCU_BASE + 0x00DC)
#define RA8_FLASH_FCPSR                 (RA8_FCU_BASE + 0x00E0)
#define RA8_FLASH_FPCKAR                (RA8_FCU_BASE + 0x00E4)
#define RA8_FLASH_FSUACR                (RA8_FCU_BASE + 0x00E8)

/* Flash memory regions */
#define RA8_CODE_FLASH_START            0x00000000
#define RA8_CODE_FLASH_SIZE             0x00100000  /* 1MB */
#define RA8_DATA_FLASH_START            0x08000000
#define RA8_DATA_FLASH_SIZE             0x00003000  /* 12KB */

/* Flash block sizes */
#define RA8_CODE_FLASH_BLOCK_SIZE       0x00002000  /* 8KB */
#define RA8_DATA_FLASH_BLOCK_SIZE       0x00000040  /* 64 bytes */

/* Flash command codes */
#define FLASH_CMD_PROGRAM               0xE8
#define FLASH_CMD_BLOCK_ERASE          0x20
#define FLASH_CMD_PE_SUSPEND           0xB0
#define FLASH_CMD_PE_RESUME            0xD0
#define FLASH_CMD_STATUS_CLEAR         0x50
#define FLASH_CMD_FORCED_STOP          0xB3
#define FLASH_CMD_BLANK_CHECK          0x71
#define FLASH_CMD_CONFIG_SET_1         0x40
#define FLASH_CMD_CONFIG_SET_2         0x08
#define FLASH_CMD_LOCK_BIT_PGM         0x77
#define FLASH_CMD_LOCK_BIT_READ        0x76

/* Status register bits */
#define FLASH_FSTATR_FRDY              (1 << 15)   /* Flash Ready */
#define FLASH_FSTATR_ILGLERR           (1 << 14)   /* Illegal Command Error */
#define FLASH_FSTATR_ERSERR            (1 << 13)   /* Erase Error */
#define FLASH_FSTATR_PRGERR            (1 << 12)   /* Program Error */
#define FLASH_FSTATR_SUSRDY            (1 << 11)   /* Suspend Ready */
#define FLASH_FSTATR_DBFULL            (1 << 10)   /* Data Buffer Full */
#define FLASH_FSTATR_ERSSPD            (1 << 9)    /* Erase Suspend */
#define FLASH_FSTATR_PRGSPD            (1 << 8)    /* Program Suspend */
#define FLASH_FSTATR_FCUERR            (1 << 4)    /* FCU Error */
#define FLASH_FSTATR_FLWEERR           (1 << 3)    /* Flash Write/Erase Error */

/* Entry register bits */
#define FLASH_FENTRYR_FENTRYC          (1 << 7)    /* Code Flash P/E Mode Entry */
#define FLASH_FENTRYR_FENTRYD          (1 << 0)    /* Data Flash P/E Mode Entry */

/* Protection register bits */
#define FLASH_FPROTR_FPROTCN           (1 << 0)    /* Code Flash Protection */

/* Interrupt enable bits */
#define FLASH_FRDYIE_FRDYIE            (1 << 0)    /* Flash Ready Interrupt Enable */

/* Error interrupt bits */
#define FLASH_FAEINT_CMDLKIE           (1 << 4)    /* Command Lock Error Interrupt Enable */
#define FLASH_FAEINT_CFAERIE           (1 << 3)    /* Code Flash Access Error Interrupt Enable */
#define FLASH_FAEINT_DFAERIE           (1 << 2)    /* Data Flash Access Error Interrupt Enable */

#endif /* __ARCH_ARM_SRC_RA8_HARDWARE_R_FLASH_H */