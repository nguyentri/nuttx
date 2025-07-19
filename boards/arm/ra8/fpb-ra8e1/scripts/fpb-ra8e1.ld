/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/scripts/fpb-ra8e1.ld
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

/* The RA8E1 has 1MB of FLASH beginning at address 0x02000000 and
 * 512KB RAM beginning at address 0x22060000, plus TCM memories.
 * Updated to match FSP memory layout.
 */

MEMORY
{
  /* Main Flash - 1MB */
  flash (rx)   : ORIGIN = 0x02000000, LENGTH = 0x00100000
  
  /* Main RAM - 512KB */
  sram (rwx)   : ORIGIN = 0x22060000, LENGTH = 0x00080000
  
  /* Data Flash - 12KB */
  data_flash (rx) : ORIGIN = 0x27000000, LENGTH = 0x00003000
  
  /* Instruction Tightly Coupled Memory - 16KB */
  itcm (rwx)   : ORIGIN = 0x00000000, LENGTH = 0x00004000
  
  /* Data Tightly Coupled Memory - 16KB */
  dtcm (rwx)   : ORIGIN = 0x20000000, LENGTH = 0x00004000
  
  /* OSPI Flash regions (external) */
  ospi0_cs0 (rwx) : ORIGIN = 0x80000000, LENGTH = 0x10000000
  ospi0_cs1 (rx)  : ORIGIN = 0x90000000, LENGTH = 0x10000000
  
  /* Option Setting regions */
  option_ofs0 (r)    : ORIGIN = 0x0300a100, LENGTH = 0x00000004
  option_ofs2 (r)    : ORIGIN = 0x0300a104, LENGTH = 0x00000004
  option_dualsel (r) : ORIGIN = 0x0300a110, LENGTH = 0x00000004
  
  /* ID Code region */
  idcode (rx)  : ORIGIN = 0x01010018, LENGTH = 0x20
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .text :
    {
        _stext = ABSOLUTE(.);
        *(.vectors)
        . = _stext + 0x400;
        KEEP(*(.rom_registers*))
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > flash

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array .ctors))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

    .ARM.exidx :
    {
        __exidx_start = ABSOLUTE(.);
        *(.ARM.exidx*)
        __exidx_end = ABSOLUTE(.);
    } > flash

    /* Option setting sections */
    .option_setting_ofs0 :
    {
        KEEP(*(.option_setting_ofs0))
    } > option_ofs0

    .option_setting_ofs2 :
    {
        KEEP(*(.option_setting_ofs2))
    } > option_ofs2

    .option_setting_dualsel :
    {
        KEEP(*(.option_setting_dualsel))
    } > option_dualsel

    .id_code :
    {
        __ID_Code_Start = .;
        KEEP(*(.id_code*))
        __ID_Code_End = .;
    } > idcode

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    _eronly = LOADADDR(.data);

    .ramfunc ALIGN(4):
    {
        _sramfuncs = ABSOLUTE(.);
        *(.ramfunc  .ramfunc.*)
        _eramfuncs = ABSOLUTE(.);
    } > sram AT > flash

    _framfuncs = LOADADDR(.ramfunc);

    /* ITCM sections */
    .itcm_data :
    {
        _sitcm = ABSOLUTE(.);
        *(.itcm .itcm.*)
        *(.itcm_code .itcm_code.*)
        _eitcm = ABSOLUTE(.);
    } > itcm AT > flash

    _fitcm = LOADADDR(.itcm_data);

    /* DTCM sections */
    .dtcm_data :
    {
        _sdtcm = ABSOLUTE(.);
        *(.dtcm .dtcm.*)
        *(.dtcm_code .dtcm_code.*)
        _edtcm = ABSOLUTE(.);
    } > dtcm AT > flash

    _fdtcm = LOADADDR(.dtcm_data);

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = ABSOLUTE(.);
    } > sram

    /* ITCM BSS */
    .itcm_bss (NOLOAD) :
    {
        _sitcm_bss = ABSOLUTE(.);
        *(.itcm_bss .itcm_bss.*)
        _eitcm_bss = ABSOLUTE(.);
    } > itcm

    /* DTCM BSS */
    .dtcm_bss (NOLOAD) :
    {
        _sdtcm_bss = ABSOLUTE(.);
        *(.dtcm_bss .dtcm_bss.*)
        _edtcm_bss = ABSOLUTE(.);
    } > dtcm

    /* Stabs debugging sections. */
    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
