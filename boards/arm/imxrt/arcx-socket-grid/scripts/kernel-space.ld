/****************************************************************************
 * boards/arm/imxrt/arcx-socket-grid/scripts/kernel-space.ld
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* NOTE:  This depends on the memory.ld script having been included prior to
 * this script.
 */

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
  .text :
  {
    _stext = ABSOLUTE(.);
    *(.vectors)
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
  } > kflash

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
    _einit = ABSOLUTE(.);
  } > kflash

  .ARM.extab :
  {
    *(.ARM.extab*)
  } > kflash

  __exidx_start = ABSOLUTE(.);
  .ARM.exidx :
  {
    *(.ARM.exidx*)
  } > kflash

  __exidx_end = ABSOLUTE(.);

  _eronly = ABSOLUTE(.);

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > kocram AT > kflash

  .bss :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = ABSOLUTE(.);
  } > kocram

  /* Stabs debugging sections */

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
