/****************************************************************************
 * boards/arm/ra8/fpb-ra8e1/scripts/flash.ld
 * Flat build, run from internal flash
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

/* Memory map for RA8E1 */
FLASH_ORIGIN  = 0x02000000; /* Code flash 1MB */
FLASH_LENGTH  = 0x00100000;
RAM_ORIGIN    = 0x22060000; /* SRAM 512KB */
RAM_LENGTH    = 0x00080000;
ITCM_ORIGIN   = 0x00000000; /* 16KB ITCM (size may adjust) */
ITCM_LENGTH   = 0x00004000;
DTCM_ORIGIN   = 0x20000000; /* 16KB DTCM */
DTCM_LENGTH   = 0x00004000;

MEMORY
{
  flash (rx)  : ORIGIN = FLASH_ORIGIN, LENGTH = FLASH_LENGTH
  ram   (rwx) : ORIGIN = RAM_ORIGIN,   LENGTH = RAM_LENGTH
  itcm  (rwx) : ORIGIN = ITCM_ORIGIN,  LENGTH = ITCM_LENGTH
  dtcm  (rwx) : ORIGIN = DTCM_ORIGIN,  LENGTH = DTCM_LENGTH
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
  /* Vector table at beginning of flash */
  .text :
  {
    _stext = ABSOLUTE(.);
    KEEP(*(.vectors))
    *(.text .text.*)
    *(.fixup)
    *(.gnu.warning)
    *(.rodata .rodata.*)
    *(.gnu.linkonce.t.*)
    *(.glue_7) *(.glue_7t)
    *(.got) *(.gcc_except_table)
    *(.gnu.linkonce.r.*)
    _etext = ABSOLUTE(.);
  } > flash

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(.init_array .init_array.*))
    _einit = ABSOLUTE(.);
  } > flash

  .ARM.extab : { *(.ARM.extab*) } > flash
  __exidx_start = ABSOLUTE(.);
  .ARM.exidx : { *(.ARM.exidx*) } > flash
  __exidx_end = ABSOLUTE(.);
  _eronly = ABSOLUTE(.);

  /* ITCM optional hot code section */
  .itcm_text :
  {
    _sitcm = ABSOLUTE(.);
    *(.itcm .itcm.*)
    _eitcm = ABSOLUTE(.);
  } > itcm AT > flash
  _itcm_load = LOADADDR(.itcm_text);

  /* DATA */
  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > ram AT > flash

  /* Functions forced into RAM */
  .ramfunc ALIGN(4):
  {
    _sramfuncs = ABSOLUTE(.);
    *(.ramfunc .ramfunc.*)
    _eramfuncs = ABSOLUTE(.);
  } > ram AT > flash
  _ramfunc_load = LOADADDR(.ramfunc);

  /* Zero init / BSS */
  .bss :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = ABSOLUTE(.);
  } > ram

  /* Idle thread stack placeholder (matched by RA start code symbol names) */
  __ram_thread_stack$$ ALIGN(8) (NOLOAD):
  {
    __ram_thread_stack$$Base = .;
    . += 0x400; /* reserve 1KB base area; idle top computed in C */
    __ram_thread_stack$$Limit = .;
  } > ram

/* Debug sections (DWARF v2–v5). Place at VMA 0 so they don't occupy flash. */
  .debug_info        0 : { *(.debug_info .debug_info.* .gnu.linkonce.wi.*) }
  .zdebug_info       0 : { *(.zdebug_info .zdebug_info.*) }
  .debug_abbrev      0 : { *(.debug_abbrev .debug_abbrev.*) }
  .zdebug_abbrev     0 : { *(.zdebug_abbrev .zdebug_abbrev.*) }
  .debug_line        0 : { *(.debug_line .debug_line.*) }
  .zdebug_line       0 : { *(.zdebug_line .zdebug_line.*) }
  .debug_line_str    0 : { *(.debug_line_str .debug_line_str.*) }
  .debug_str         0 : { *(.debug_str .debug_str.*) }
  .zdebug_str        0 : { *(.zdebug_str .zdebug_str.*) }
  .debug_str_offsets 0 : { *(.debug_str_offsets .debug_str_offsets.*) }
  .debug_frame       0 : { *(.debug_frame .debug_frame.*) }
  .debug_aranges     0 : { *(.debug_aranges .debug_aranges.*) }
  .debug_loc         0 : { *(.debug_loc .debug_loc.*) }
  .debug_loclists    0 : { *(.debug_loclists .debug_loclists.*) }
  .debug_rnglists    0 : { *(.debug_rnglists .debug_rnglists.*) }
  .debug_macro       0 : { *(.debug_macro .debug_macro.*) }
  .debug_macinfo     0 : { *(.debug_macinfo .debug_macinfo.*) }
  .debug_pubnames    0 : { *(.debug_pubnames .debug_pubnames.*) }
  .debug_pubtypes    0 : { *(.debug_pubtypes .debug_pubtypes.*) }
  .debug_types       0 : { *(.debug_types .debug_types.*) }
  .debug_names       0 : { *(.debug_names .debug_names.*) }
  .debug_sup         0 : { *(.debug_sup .debug_sup.*) }

  /* RA option setting fixed addresses (placeholders) */
  .ra_option_settings :
  {
    KEEP(*(.option_setting_ofs0))
    KEEP(*(.option_setting_ofs2))
    KEEP(*(.option_setting_dualsel))
    KEEP(*(.option_setting_ofs1))
    KEEP(*(.option_setting_banksel))
    KEEP(*(.option_setting_bps))
    KEEP(*(.option_setting_pbps))
    KEEP(*(.id_code))
  } > flash
}

PROVIDE(_edata = _edata);
PROVIDE(_ebss = _ebss);
