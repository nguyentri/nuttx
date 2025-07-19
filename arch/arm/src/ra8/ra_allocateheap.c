/****************************************************************************
 * arch/arm/src/ra8/ra_allocateheap.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "mpu.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   The following memory map is assumed for the flat build:
 *
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region.  Size determined at link time.
 *     Kernel .bss  region  Size determined at link time.
 *     Kernel IDLE thread stack.  Size determined by
 *                           CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* FSP-based memory map validation and heap setup - Compatible with linker script */
  
  board_autoled_on(LED_HEAPALLOCATE);

#ifdef CONFIG_RA8_DTCM_HEAP
  /* Use DTCM for heap if configured - FSP memory optimization */
  /* Linker script: dtcm (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00004000 */
  *heap_start = (void *)CONFIG_RA8_DTCM_BASE;
  *heap_size  = CONFIG_RA8_DTCM_SIZE;
  
#ifdef CONFIG_RA8_HEAP_DEBUG
  syslog(LOG_INFO, "Using DTCM heap: start=0x%08lx size=%zu bytes\n", 
         (unsigned long)*heap_start, *heap_size);
#endif

#elif defined(CONFIG_RA8_ITCM_HEAP)
  /* Use ITCM for heap if configured - Alternative fast memory */
  /* Linker script: itcm (rwx) : ORIGIN = 0x00000000, LENGTH = 0x00004000 */
  *heap_start = (void *)CONFIG_RA8_ITCM_BASE;
  *heap_size  = CONFIG_RA8_ITCM_SIZE;
  
#ifdef CONFIG_RA8_HEAP_DEBUG
  syslog(LOG_INFO, "Using ITCM heap: start=0x%08lx size=%zu bytes\n", 
         (unsigned long)*heap_start, *heap_size);
#endif

#elif defined(CONFIG_RA8_EXTERNAL_RAM_HEAP)
  /* Use external RAM if available - FSP external memory support */
  /* Linker script: ospi0_cs0 (rwx) : ORIGIN = 0x80000000, LENGTH = 0x10000000 */
  *heap_start = (void *)CONFIG_RA8_EXTERNAL_RAM_BASE;
  *heap_size  = CONFIG_RA8_EXTERNAL_RAM_SIZE;
  
#ifdef CONFIG_RA8_HEAP_DEBUG
  syslog(LOG_INFO, "Using external RAM heap: start=0x%08lx size=%zu bytes\n", 
         (unsigned long)*heap_start, *heap_size);
#endif

#else
  /* Standard internal SRAM heap configuration */
  /* Linker script: sram (rwx) : ORIGIN = 0x22060000, LENGTH = 0x00080000 */
  /* Validate memory boundaries using FSP memory map */
  uintptr_t heap_end = CONFIG_RAM_END;  /* 0x22060000 + 0x80000 = 0x220E0000 */
  
#ifdef CONFIG_RA8_STACK_GUARD
  /* Reserve stack guard region - FSP security feature */
  heap_end -= CONFIG_RA8_STACK_GUARD_SIZE;
#endif

#ifdef CONFIG_RA8_HEAP_ALIGNMENT
  /* Apply FSP-style heap alignment requirements */
  uintptr_t heap_start_aligned = (g_idle_topstack + CONFIG_RA8_HEAP_ALIGNMENT - 1) &
                                 ~(CONFIG_RA8_HEAP_ALIGNMENT - 1);
  *heap_start = (void *)heap_start_aligned;
  *heap_size  = heap_end - heap_start_aligned;
#else
  /* Default heap configuration */
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = heap_end - g_idle_topstack;
#endif

#ifdef CONFIG_RA8_HEAP_DEBUG
  syslog(LOG_INFO, "Using main SRAM heap: start=0x%08lx size=%zu bytes\n", 
         (unsigned long)*heap_start, *heap_size);
  syslog(LOG_INFO, "SRAM region: 0x%08lx-0x%08lx, idle_topstack=0x%08lx\n",
         CONFIG_RA8_SRAM_BASE, CONFIG_RAM_END, (unsigned long)g_idle_topstack);
#endif
#endif

  /* Validate heap configuration against linker script */
  if (*heap_start == NULL || *heap_size == 0)
    {
      syslog(LOG_ERR, "Invalid heap configuration: start=%p size=%zu\n", 
             *heap_start, *heap_size);
      board_autoled_on(LED_PANIC);
      PANIC();
    }

  /* Validate heap size meets minimum requirements */
  if (*heap_size < CONFIG_IDLETHREAD_STACKSIZE)
    {
      /* Insufficient heap space - FSP error handling */
      syslog(LOG_ERR, "Insufficient heap space: %zu < %d bytes\n", 
             *heap_size, CONFIG_IDLETHREAD_STACKSIZE);
      board_autoled_on(LED_PANIC);
      PANIC();
    }

  /* Validate heap boundaries against linker script memory regions */
  uintptr_t heap_start_addr = (uintptr_t)*heap_start;
  uintptr_t heap_end_addr = heap_start_addr + *heap_size;

#ifdef CONFIG_RA8_DTCM_HEAP
  if (heap_start_addr < CONFIG_RA8_DTCM_BASE || 
      heap_end_addr > (CONFIG_RA8_DTCM_BASE + CONFIG_RA8_DTCM_SIZE))
    {
      syslog(LOG_ERR, "DTCM heap exceeds linker boundaries\n");
      PANIC();
    }
#elif defined(CONFIG_RA8_ITCM_HEAP)
  if (heap_start_addr < CONFIG_RA8_ITCM_BASE || 
      heap_end_addr > (CONFIG_RA8_ITCM_BASE + CONFIG_RA8_ITCM_SIZE))
    {
      syslog(LOG_ERR, "ITCM heap exceeds linker boundaries\n");
      PANIC();
    }
#elif defined(CONFIG_RA8_EXTERNAL_RAM_HEAP)
  if (heap_start_addr < CONFIG_RA8_EXTERNAL_RAM_BASE || 
      heap_end_addr > (CONFIG_RA8_EXTERNAL_RAM_BASE + CONFIG_RA8_EXTERNAL_RAM_SIZE))
    {
      syslog(LOG_ERR, "External RAM heap exceeds linker boundaries\n");
      PANIC();
    }
#else
  if (heap_start_addr < CONFIG_RA8_SRAM_BASE || 
      heap_end_addr > CONFIG_RAM_END)
    {
      syslog(LOG_ERR, "SRAM heap exceeds linker boundaries\n");
      PANIC();
    }
#endif
}
