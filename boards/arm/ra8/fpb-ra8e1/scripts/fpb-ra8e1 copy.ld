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
 */
RAM_START = 0x22060000;
RAM_LENGTH = 0x00080000;
FLASH_START = 0x02000000;
FLASH_LENGTH = 0x00100000;
DATA_FLASH_START = 0x27000000;
DATA_FLASH_LENGTH = 0x00003000;
OSPI0_CS0_START = 0x80000000;
OSPI0_CS0_LENGTH = 0x10000000;
OSPI0_CS1_START = 0x90000000;
OSPI0_CS1_LENGTH = 0x10000000;
OPTION_SETTING_OFS0_START = 0x0300a100;
OPTION_SETTING_OFS0_LENGTH = 0x00000004;
OPTION_SETTING_OFS2_START = 0x0300a104;
OPTION_SETTING_OFS2_LENGTH = 0x00000004;
OPTION_SETTING_DUALSEL_START = 0x0300a110;
OPTION_SETTING_DUALSEL_LENGTH = 0x00000004;
OPTION_SETTING_OFS1_START = 0x1300a180;
OPTION_SETTING_OFS1_LENGTH = 0x00000004;
OPTION_SETTING_BANKSEL_START = 0x1300a190;
OPTION_SETTING_BANKSEL_LENGTH = 0x00000004;
OPTION_SETTING_BPS_START = 0x1300a1c0;
OPTION_SETTING_BPS_LENGTH = 0x00000010;
OPTION_SETTING_PBPS_START = 0x1300a1e0;
OPTION_SETTING_PBPS_LENGTH = 0x00000010;
OPTION_SETTING_OFS1_SEC_START = 0x0300a200;
OPTION_SETTING_OFS1_SEC_LENGTH = 0x00000004;
OPTION_SETTING_BANKSEL_SEC_START = 0x0300a210;
OPTION_SETTING_BANKSEL_SEC_LENGTH = 0x00000004;
OPTION_SETTING_BPS_SEC_START = 0x0300a240;
OPTION_SETTING_BPS_SEC_LENGTH = 0x00000010;
OPTION_SETTING_PBPS_SEC_START = 0x0300a260;
OPTION_SETTING_PBPS_SEC_LENGTH = 0x00000010;
OPTION_SETTING_OFS1_SEL_START = 0x0300a280;
OPTION_SETTING_OFS1_SEL_LENGTH = 0x00000004;
OPTION_SETTING_BANKSEL_SEL_START = 0x0300a290;
OPTION_SETTING_BANKSEL_SEL_LENGTH = 0x00000004;
OPTION_SETTING_BPS_SEL_START = 0x0300a2c0;
OPTION_SETTING_BPS_SEL_LENGTH = 0x00000010;
ITCM_START = 0x00000000;
ITCM_LENGTH = 0x00004000;
DTCM_START = 0x20000000;
DTCM_LENGTH = 0x00004000;
MEMORY
{
  /* Main Flash - 1MB */
  flash (rx)   : ORIGIN = FLASH_START, LENGTH = FLASH_LENGTH

  /* Main RAM - 512KB */
  sram (rwx)   : ORIGIN = RAM_START, LENGTH = RAM_LENGTH

  /* Data Flash - 12KB */
  data_flash (rx) : ORIGIN = DATA_FLASH_START, LENGTH = DATA_FLASH_LENGTH

  /* Instruction Tightly Coupled Memory - 16KB */
  itcm (rwx)   : ORIGIN = ITCM_START, LENGTH = ITCM_LENGTH

  /* Data Tightly Coupled Memory - 16KB */
  dtcm (rwx)   : ORIGIN = DTCM_START, LENGTH = DTCM_LENGTH

  /* OSPI Flash regions (external) */
  ospi0_cs0 (rwx) : ORIGIN = OSPI0_CS0_START, LENGTH = OSPI0_CS0_LENGTH
  ospi0_cs1 (rx)  : ORIGIN = OSPI0_CS1_START, LENGTH = OSPI0_CS1_LENGTH

  /* Option Setting regions */
  option_ofs0 (r)    : ORIGIN = OPTION_SETTING_OFS0_START, LENGTH = OPTION_SETTING_OFS0_LENGTH
  option_ofs2 (r)    : ORIGIN = OPTION_SETTING_OFS2_START, LENGTH = OPTION_SETTING_OFS2_LENGTH
  option_dualsel (r) : ORIGIN = OPTION_SETTING_DUALSEL_START, LENGTH = OPTION_SETTING_DUALSEL_LENGTH
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
    } > data_flash

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
