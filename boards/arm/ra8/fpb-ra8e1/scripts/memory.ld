/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/scripts/memory.ld
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

/* The RA8E1 FPB Board has:
 * - 1MB Code Flash beginning at address 0x02000000
 * - 512KB SRAM beginning at address 0x22060000
 * - 16KB ITCM beginning at address 0x00000000
 * - 16KB DTCM beginning at address 0x20000000
 */

/* Specify the memory areas */

MEMORY
{
  /* Code Flash */
  kflash (rx)   : ORIGIN = 0x02000000, LENGTH = 1024K
  
  /* Main SRAM for kernel and applications */
  ksram (rwx)   : ORIGIN = 0x22060000, LENGTH = 256K
  usram (rwx)   : ORIGIN = 0x220A0000, LENGTH = 256K
  
  /* Tightly Coupled Memory */
  itcm (rwx)    : ORIGIN = 0x00000000, LENGTH = 16K
  dtcm (rwx)    : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Provide total memory size calculations for stack and heap management */
_flashsize = LENGTH(kflash);
_sramsize = LENGTH(ksram) + LENGTH(usram);
_itcmsize = LENGTH(itcm);
_dtcmsize = LENGTH(dtcm);
