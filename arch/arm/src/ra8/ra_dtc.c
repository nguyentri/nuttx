/****************************************************************************
 * arch/arm/src/ra8/ra_dtc.c
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
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ra_dtc.h"
#include "hardware/ra8e1/ra8e1_memorymap.h"
#include "ra_dtc.h"
#include "ra_icu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DTC_OPEN_ID             (0x44544300)  /* "DTC" in ASCII */
#define DTC_ALIGNMENT_CHECK(addr, size) \
  (((uintptr_t)(addr) & ((size) - 1)) == 0)

/* DTC Control Register values */
#define DTC_DTCCR_RRS_ENABLE    (0x18)
#define DTC_DTCCR_RRS_DISABLE   (0x08)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DTC context control structure */
typedef struct ra_dtc_ctrl_s
{
  uint32_t             open_id;        /* Open identifier */
  bool                 in_use;         /* Context in use flag */
  ra_dtc_config_t      config;         /* Configuration */
  ra_dtc_info_t        info;           /* Transfer information */
  int                  irq;            /* IRQ number */
  bool                 slot_allocated; /* True if we allocated an ICU slot for trigger */
  int                  trigger_irq;    /* IRQ number allocated for elc_src -> slot mapping */
} ra_dtc_ctrl_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DTC context control blocks */
ra_dtc_ctrl_t g_dtc_contexts[RA_DTC_MAX_CONTEXTS];

/* DTC vector table (aligned to 1024 bytes) */
/* Place the DTC vector table into the linker section that the linker
 * script names __ram_dtc_vector$$ (section name: .fsp_dtc_vector_table)
 */
ra_dtc_info_t *g_dtc_vector_table[RA_DTC_VECTOR_TABLE_ENTRIES]
  __attribute__((section(".fsp_dtc_vector_table")))
  __attribute__((aligned(RA_DTC_VECTOR_TABLE_ALIGN)));

/* DTC module initialized flag */
static bool g_dtc_initialized = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dtc_validate_config
 *
 * Description:
 *   Validate DTC configuration parameters
 *
 * Input Parameters:
 *   config - DTC configuration
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int ra_dtc_validate_config(const ra_dtc_config_t *config)
{
  /* Check configuration pointer */
  if (config == NULL)
    {
      return -EINVAL;
    }

  /* Check transfer mode */
  if (config->mode > RA_DTC_MODE_BLOCK)
    {
      return -EINVAL;
    }

  /* Check transfer size */
  if (config->size > RA_DTC_SIZE_LONG)
    {
      return -EINVAL;
    }

  /* Check address modes */
  if (config->src_addr_mode > RA_DTC_ADDR_DECR ||
      config->dest_addr_mode > RA_DTC_ADDR_DECR)
    {
      return -EINVAL;
    }

  /* Check addresses are not null */
  if (config->src_addr == 0 || config->dest_addr == 0)
    {
      return -EINVAL;
    }

  /* Check alignment based on transfer size */
  uint32_t align_mask = (1 << config->size) - 1;
  if ((config->src_addr & align_mask) || (config->dest_addr & align_mask))
    {
      return -EINVAL;
    }

  /* Check transfer count limits */
  switch (config->mode)
    {
      case RA_DTC_MODE_NORMAL:
        if (config->transfer_count > RA_DTC_MAX_NORMAL_LENGTH)
          {
            return -EINVAL;
          }
        break;

      case RA_DTC_MODE_REPEAT:
        if (config->transfer_count > RA_DTC_MAX_REPEAT_LENGTH ||
            config->block_count > RA_DTC_MAX_BLOCK_LENGTH)
          {
            return -EINVAL;
          }
        break;

      case RA_DTC_MODE_BLOCK:
        if (config->transfer_count > RA_DTC_MAX_BLOCK_LENGTH ||
            config->block_count > RA_DTC_MAX_NORMAL_LENGTH)
          {
            return -EINVAL;
          }
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_setup_transfer_info
 *
 * Description:
 *   Setup DTC transfer information structure
 *
 * Input Parameters:
 *   ctrl - DTC control block
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

static int ra_dtc_setup_transfer_info(ra_dtc_ctrl_t *ctrl)
{
  ra_dtc_info_t *info = &ctrl->info;
  const ra_dtc_config_t *config = &ctrl->config;

  /* Clear transfer information */
  memset(info, 0, sizeof(ra_dtc_info_t));

  /* Setup Mode Register A - source addressing only */
  info->mra = (config->mode << RA_DTC_MRA_MD_SHIFT) |
              (config->size << RA_DTC_MRA_SZ_SHIFT) |
              (config->src_addr_mode << RA_DTC_MRA_SM_SHIFT);

  /* Setup Mode Register B - destination addressing */
  info->mrb = (config->dest_addr_mode << RA_DTC_MRB_DM_SHIFT);

  /* Setup addresses */
  info->sar = config->src_addr;
  info->dar = config->dest_addr;

  /* Setup transfer counts */
  switch (config->mode)
    {
      case RA_DTC_MODE_NORMAL:
        info->cra = config->transfer_count;
        info->crb = 0;
        break;

      case RA_DTC_MODE_REPEAT:
        info->cra = config->transfer_count;
        info->crb = config->block_count;
        break;

      case RA_DTC_MODE_BLOCK:
        info->cra = config->transfer_count;
        info->crb = config->block_count;
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_interrupt_handler
 *
 * Description:
 *   DTC interrupt handler
 *
 * Input Parameters:
 *   irq - IRQ number
 *   context - Interrupt context
 *   arg - Argument passed to handler
 *
 * Returned Value:
 *   OK on success
 *
 ****************************************************************************/

static int ra_dtc_interrupt_handler(int irq, void *context, void *arg)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)arg;

  if (ctrl && ctrl->config.callback)
    {
      /* For DTC completion interrupt (RA_ELC_DTC_COMPLETE), the specific
       * transfer that completed is indicated in DTCSTS register.
       * However, since we're using individual control blocks per transfer,
       * we can directly call the callback for this specific context.
       */

      /* Check if DTC is still active for this specific transfer */
      uint32_t dtc_status = getreg32(RA_DTC_DTCSTS);

      /* If DTC is no longer active, the transfer completed */
      if (!(dtc_status & RA_DTC_DTCSTS_ACT))
        {
          /* Transfer completed, call user callback */
          ctrl->config.callback(ctrl, RA_DTC_EVENT_COMPLETE, ctrl->config.user_data);
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_find_free_context
 *
 * Description:
 *   Find a free DTC context
 *
 * Returned Value:
 *   Pointer to free context or NULL if none available
 *
 ****************************************************************************/

static ra_dtc_ctrl_t *ra_dtc_find_free_context(void)
{
  for (int i = 0; i < RA_DTC_MAX_CONTEXTS; i++)
    {
      if (!g_dtc_contexts[i].in_use)
        {
          return &g_dtc_contexts[i];
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: ra_dtc_wait_for_completion
 *
 * Description:
 *   Wait for DTC transfer to complete
 *
 * Input Parameters:
 *   ctrl - DTC control block
 *
 ****************************************************************************/

static void ra_dtc_wait_for_completion(ra_dtc_ctrl_t *ctrl)
{
  /* Wait for DTC to become inactive */
  while (getreg32(RA_DTC_DTCSTS) & RA_DTC_DTCSTS_ACT)
    {
      /* Small delay to prevent busy waiting */
      up_udelay(1);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_dtc_initialize
 *
 * Description:
 *   Initialize DTC module
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_initialize(void)
{
  if (g_dtc_initialized)
    {
      return OK;
    }

  /* Clear all contexts */
  memset(g_dtc_contexts, 0, sizeof(g_dtc_contexts));

  /* Clear vector table */
  memset(g_dtc_vector_table, 0, sizeof(g_dtc_vector_table));

  /* Set vector table base address */
  putreg32((uint32_t)g_dtc_vector_table, RA_DTC_DTCVBR_SEC);

  /* Disable read skip initially */
  putreg32(0, RA_DTC_DTCCR_SEC);

  /* Enable read skip for performance */
  putreg32(RA_DTC_DTCCR_RRSS, RA_DTC_DTCCR_SEC);

  /* Start the DTC module by setting DTCST.DTCST = 1 */
  putreg8(1, RA_DTC_DTCST);

  _info("DTC module started with DTCST=1\n");

  g_dtc_initialized = true;
  return OK;
}

/****************************************************************************
 * Name: ra_dtc_open
 *
 * Description:
 *   Open DTC transfer channel
 *
 * Input Parameters:
 *   handle - Pointer to store handle
 *   config - DTC configuration
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_open(ra_dtc_handle_t *handle, const ra_dtc_config_t *config)
{
  ra_dtc_ctrl_t *ctrl;
  int ret;

  if (handle == NULL || config == NULL)
    {
      return -EINVAL;
    }

  /* Initialize module if needed */
  ret = ra_dtc_initialize();
  if (ret < 0)
    {
      return ret;
    }

  /* Validate configuration */
  ret = ra_dtc_validate_config(config);
  if (ret < 0)
    {
      return ret;
    }

  /* Find free context */
  ctrl = ra_dtc_find_free_context();
  if (ctrl == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize control defaults for this context */
  ctrl->slot_allocated = false;
  ctrl->trigger_irq = -1;
  ctrl->irq = -1;

  /* Copy configuration */
  memcpy(&ctrl->config, config, sizeof(ra_dtc_config_t));

  /* Setup transfer information */
  ret = ra_dtc_setup_transfer_info(ctrl);
  if (ret < 0)
    {
      return ret;
    }

  /* Setup hardware trigger if used: prefer irq_src (IRQ number), fall back to elc_src */
  if (!config->software_trigger && (config->irq_src >= 0 || config->elc_src >= 0))
    {
      /* For event link triggered transfers, DTC hardware uses the event link
       * number as index into vector table, NOT the ICU IRQ slot number.
       * The event link itself should NOT have an interrupt handler attached
       * since DTC handles the transfer automatically on the event.
       */

      /* Decide which index to use in the vector table:
       * - If irq_src is provided, it contains the full IRQ number (RA_IRQ_FIRST + slot).
       *   Convert to slot by subtracting RA_IRQ_FIRST and use that as the vector index.
       * - Otherwise use the elc_src (event link) index directly.
       */
      int vec_index = -1;
      if (config->irq_src >= 0)
        {
          int slot = config->irq_src - RA_IRQ_FIRST;
          if (slot < 0 || slot >= RA_DTC_VECTOR_TABLE_ENTRIES)
            {
              return -EINVAL;
            }
          /* Use provided IRQ as the trigger IRQ for indexing/cleanup */
          ctrl->trigger_irq = config->irq_src;
          ctrl->slot_allocated = false; /* caller provided slot, we didn't allocate it */
          vec_index = slot;
        }
      else
        {
          /* Attach an ICU slot for this elc_src so we have a stable ICU slot
           * index to place into the dtc vector table. ra_icu_attach returns
           * the IRQ number (RA_IRQ_FIRST + slot) on success. Store it in
           * ctrl->trigger_irq so we don't modify the const config pointer.
           */
          /* Attach the ICU slot for the event without a handler (we only need
           * the slot so the DTC can be mapped). ra_icu_attach returns the IRQ
           * number (RA_IRQ_FIRST + slot) on success.
           */
          ret = ra_icu_attach(config->elc_src, NULL, NULL, false);
          if (ret < 0)
            {
              return ret;
            }
          ctrl->trigger_irq = ret;
          /* Enable DTC trigger on this ICU slot so the DTC hardware will be
           * started by the peripheral event.
           */
          ra_icu_enable_dtc(ctrl->trigger_irq);
          /* Use the slot (IRQ - RA_IRQ_FIRST) as the vector index */
          vec_index = ctrl->trigger_irq - RA_IRQ_FIRST;
          ctrl->slot_allocated = true;
        }

      /* Update vector table safely: disable read-skip before update and re-enable */
      putreg32(0, RA_DTC_DTCCR_SEC);
      g_dtc_vector_table[vec_index] = &ctrl->info;
      putreg32(RA_DTC_DTCCR_RRSS, RA_DTC_DTCCR_SEC);

      /* If completion callback is needed, attach DTC completion interrupt */
      if (config->callback)
        {
          /* Use DTC completion interrupt for notification */
          ret = ra_icu_attach(RA_ELC_DTC_COMPLETE, ra_dtc_interrupt_handler, ctrl, false);
          if (ret < 0)
            {
              g_dtc_vector_table[vec_index] = NULL;
              return ret;
            }
          ctrl->irq = ret;  /* Store the assigned IRQ slot for completion */
        }
      else
        {
          ctrl->irq = -1;  /* No completion interrupt needed */
        }
    }

  /* Mark context as in use */
  ctrl->in_use = true;
  ctrl->open_id = DTC_OPEN_ID;

  *handle = ctrl;
  return OK;
}

/****************************************************************************
 * Name: ra_dtc_close
 *
 * Description:
 *   Close DTC transfer channel
 *
 * Input Parameters:
 *   handle - DTC handle
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_close(ra_dtc_handle_t handle)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DTC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Wait for any ongoing transfer to complete */
  ra_dtc_wait_for_completion(ctrl);

  /* Disable transfer */
  ra_dtc_disable(handle);

  /* Detach interrupt handler if used */
  if (ctrl->irq >= 0)
    {
      ra_icu_detach(ctrl->irq);  /* Use the assigned IRQ slot number */
    }

  /* Clear vector table entry if hardware trigger was used */
  if (!ctrl->config.software_trigger && (ctrl->config.irq_src >= 0 || ctrl->config.elc_src >= 0 || ctrl->slot_allocated))
    {
      int vec_index = -1;
      if (ctrl->slot_allocated)
        {
          vec_index = ctrl->trigger_irq - RA_IRQ_FIRST;
        }
      else if (ctrl->config.irq_src >= 0)
        {
          vec_index = ctrl->config.irq_src - RA_IRQ_FIRST;
        }
      else
        {
          vec_index = ctrl->config.elc_src;
        }

      if (vec_index >= 0 && vec_index < RA_DTC_VECTOR_TABLE_ENTRIES)
        {
          /* Protect with read-skip disable/enable in case DTC is active */
          putreg32(0, RA_DTC_DTCCR_SEC);
          g_dtc_vector_table[vec_index] = NULL;
          putreg32(RA_DTC_DTCCR_RRSS, RA_DTC_DTCCR_SEC);
        }

      /* If we allocated the slot for this context, detach it now */
      if (ctrl->slot_allocated)
        {
          /* Disable DTC trigger first, then detach the allocated ICU slot */
          ra_icu_disable_dtc(ctrl->trigger_irq);
          ra_icu_detach(ctrl->trigger_irq);
          ctrl->slot_allocated = false;
          ctrl->trigger_irq = -1;
        }
    }

  /* No dynamic transfer info to free because it's embedded in the context */

  /* Clear control block */
  memset(ctrl, 0, sizeof(ra_dtc_ctrl_t));

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_enable
 *
 * Description:
 *   Enable DTC transfer
 *
 * Input Parameters:
 *   handle - DTC handle
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_enable(ra_dtc_handle_t handle)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DTC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Enable interrupt if hardware trigger is used */
  if (!ctrl->config.software_trigger && ctrl->irq >= 0)
    {
      up_enable_irq(ctrl->irq);  /* Enable the assigned IRQ slot */
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_disable
 *
 * Description:
 *   Disable DTC transfer
 *
 * Input Parameters:
 *   handle - DTC handle
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_disable(ra_dtc_handle_t handle)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DTC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Disable interrupt if hardware trigger is used */
  if (!ctrl->config.software_trigger && ctrl->irq >= 0)
    {
      up_disable_irq(ctrl->irq);  /* Disable the assigned IRQ slot */
    }

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_software_start
 *
 * Description:
 *   Start DTC transfer by software trigger
 *
 * Input Parameters:
 *   handle - DTC handle
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_software_start(ra_dtc_handle_t handle)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DTC_OPEN_ID)
    {
      return -EINVAL;
    }

  if (!ctrl->config.software_trigger)
    {
      return -ENOTSUP;
    }

  /* Software start is implemented by writing to DTCST register */
  /* For now, this is a placeholder as software start requires */
  /* specific implementation based on the exact trigger mechanism */

  return -ENOTSUP;  /* Not yet implemented */
}

/****************************************************************************
 * Name: ra_dtc_reset
 *
 * Description:
 *   Reset DTC transfer parameters
 *
 * Input Parameters:
 *   handle - DTC handle
 *   src_addr - New source address
 *   dest_addr - New destination address
 *   transfer_count - New transfer count
 *
 * Returned Value:
 *   OK on success; a negated errno on failure
 *
 ****************************************************************************/

int ra_dtc_reset(ra_dtc_handle_t handle, uint32_t src_addr,
                 uint32_t dest_addr, uint32_t transfer_count)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DTC_OPEN_ID)
    {
      return -EINVAL;
    }

  /* Wait for current transfer to complete */
  ra_dtc_wait_for_completion(ctrl);

  /* Disable read skip for register updates */
  putreg32(0, RA_DTC_DTCCR_SEC);

  /* Update transfer information */
  ctrl->info.sar = src_addr;
  ctrl->info.dar = dest_addr;

  switch (ctrl->config.mode)
    {
      case RA_DTC_MODE_NORMAL:
  ctrl->info.cra = transfer_count;
        break;

      case RA_DTC_MODE_REPEAT:
      case RA_DTC_MODE_BLOCK:
  ctrl->info.cra = transfer_count;
        /* Keep existing block count */
        break;
    }

  /* Update configuration */
  ctrl->config.src_addr = src_addr;
  ctrl->config.dest_addr = dest_addr;
  ctrl->config.transfer_count = transfer_count;

  /* Re-enable read skip */
  putreg32(RA_DTC_DTCCR_RRSS, RA_DTC_DTCCR_SEC);

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_get_remaining_count
 *
 * Description:
 *   Get remaining transfer count
 *
 * Input Parameters:
 *   handle - DTC handle
 *
 * Returned Value:
 *   Remaining transfer count or 0 on error
 *
 ****************************************************************************/

uint32_t ra_dtc_get_remaining_count(ra_dtc_handle_t handle)
{
  ra_dtc_ctrl_t *ctrl = (ra_dtc_ctrl_t *)handle;

  if (ctrl == NULL || ctrl->open_id != DTC_OPEN_ID)
    {
      return 0;
    }

  /* Return current transfer count from CRA register */
  return ctrl->info.cra;
}

/****************************************************************************
 * Name: ra_dtc_set_vector
 *
 * Description:
 *   Set DTC vector table entry for the specified ICU slot
 *
 * Input Parameters:
 *   icu_slot - ICU slot number assigned by ra_icu_attach
 *   transfer_info - Pointer to transfer information structure
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_dtc_set_vector(int icu_slot, ra_dtc_info_t *transfer_info)
{
  if (icu_slot < 0 || icu_slot >= RA_DTC_VECTOR_TABLE_ENTRIES)
    {
      return -EINVAL;
    }

  if (transfer_info == NULL)
    {
      return -EINVAL;
    }

  /* Set vector table entry */
  g_dtc_vector_table[icu_slot] = transfer_info;

  return OK;
}

/****************************************************************************
 * Name: ra_dtc_clear_vector
 *
 * Description:
 *   Clear DTC vector table entry for the specified ICU slot
 *
 * Input Parameters:
 *   icu_slot - ICU slot number assigned by ra_icu_attach
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure
 *
 ****************************************************************************/

int ra_dtc_clear_vector(int icu_slot)
{
  if (icu_slot < 0 || icu_slot >= RA_DTC_VECTOR_TABLE_ENTRIES)
    {
      return -EINVAL;
    }

  /* Clear vector table entry */
  g_dtc_vector_table[icu_slot] = NULL;

  return OK;
}