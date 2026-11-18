/****************************************************************************
 * boards/arm/am263x/lp-am263/scripts/ocram.ld
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

/* NuttX on R5FSS0-0, loaded into RAM by the TI SBL.
 *
 * The SBL copies every loadable segment and then resets the core, which
 * starts at the reset vector at 0x0 in ATCM.  So only the vector table
 * lives in ATCM; its entries load their targets from a literal pool, so
 * they reach code anywhere.  Everything else goes into OCRAM above
 * 0x70040000: the SBL itself runs from 0x70000000-0x7003ffff while it
 * loads the image, so nothing may be placed there.
 *
 * The ELF entry point is not used by the SBL (it always starts at 0x0);
 * it is set to __start for debuggers.
 */

MEMORY
{
  atcm  (rwx) : ORIGIN = 0x00000000, LENGTH = 32K
  ocram (rwx) : ORIGIN = 0x70040000, LENGTH = 0x1c0000
}

OUTPUT_ARCH(arm)
EXTERN(_vector_start)
ENTRY(__start)

SECTIONS
{
  .vectors :
  {
    _svectors = ABSOLUTE(.);
    KEEP(*(.vectors))
    _evectors = ABSOLUTE(.);
  } > atcm

  .text :
  {
    _stext = ABSOLUTE(.);
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
  } > ocram

  .init_section :
  {
    _sinit = ABSOLUTE(.);
    KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
    _einit = ABSOLUTE(.);
  } > ocram

  .ARM.extab :
  {
    *(.ARM.extab*)
  } > ocram

  .ARM.exidx :
  {
    __exidx_start = ABSOLUTE(.);
    *(.ARM.exidx*)
    __exidx_end = ABSOLUTE(.);
  } > ocram

  _eronly = ABSOLUTE(.);

  .data :
  {
    _sdata = ABSOLUTE(.);
    *(.data .data.*)
    *(.gnu.linkonce.d.*)
    CONSTRUCTORS
    . = ALIGN(4);
    _edata = ABSOLUTE(.);
  } > ocram

  .noinit (NOLOAD) :
  {
    _snoinit = ABSOLUTE(.);
    *(.noinit*)
    _enoinit = ABSOLUTE(.);
  } > ocram

  /* The SBL copies only file-backed bytes; arm_head.S zeroes .bss.  The
   * idle stack and then the heap follow _ebss up to the end of OCRAM.
   */

  .bss (NOLOAD) :
  {
    _sbss = ABSOLUTE(.);
    *(.bss .bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(8);
    _ebss = ABSOLUTE(.);
  } > ocram

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
