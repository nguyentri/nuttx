/****************************************************************************
 * arch/arm/src/ra8/ra_mstp.c
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
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ra_mstp.h"
#include "hardware/ra_memorymap.h"
#include "ra_mstp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Module stop control register definitions */
#define RA_MSTP_REG_B     0
#define RA_MSTP_REG_C     1
#define RA_MSTP_REG_D     2

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_mstp_get_regaddr
 *
 * Description:
 *   Get MSTP register address from module ID
 *
 ****************************************************************************/

static uint32_t ra_mstp_get_regaddr(ra_mstp_module_t module)
{
  if (module >= RA_MSTP_SCI0 && module <= RA_MSTP_CANFD)
    {
      return R_MSTP_MSTPCRB;
    }
  else if (module >= RA_MSTP_SCE5 && module <= RA_MSTP_CAC)
    {
      return R_MSTP_MSTPCRC;
    }
  else if (module >= RA_MSTP_OPAMP && module <= RA_MSTP_AGT1)
    {
      return R_MSTP_MSTPCRD;
    }
  else
    {
      return 0; /* Invalid module */
    }
}

/****************************************************************************
 * Name: ra_mstp_get_bitmask
 *
 * Description:
 *   Get bit mask for specific module
 *
 ****************************************************************************/

static uint32_t ra_mstp_get_bitmask(ra_mstp_module_t module)
{
  switch (module)
    {
      /* MSTPCRB register modules */
      case RA_MSTP_SCI0:   return R_MSTP_MSTPCRB_SCI0;
      case RA_MSTP_SCI1:   return R_MSTP_MSTPCRB_SCI1;
      case RA_MSTP_SCI2:   return R_MSTP_MSTPCRB_SCI2;
      case RA_MSTP_SCI3:   return R_MSTP_MSTPCRB_SCI3;
      case RA_MSTP_SCI4:   return R_MSTP_MSTPCRB_SCI4;
      case RA_MSTP_SCI5:   return R_MSTP_MSTPCRB_SCI5;
      case RA_MSTP_SCI6:   return R_MSTP_MSTPCRB_SCI6;
      case RA_MSTP_SCI7:   return R_MSTP_MSTPCRB_SCI7;
      case RA_MSTP_SCI8:   return R_MSTP_MSTPCRB_SCI8;
      case RA_MSTP_SCI9:   return R_MSTP_MSTPCRB_SCI9;
      case RA_MSTP_SPI0:   return R_MSTP_MSTPCRB_SPI0;
      case RA_MSTP_SPI1:   return R_MSTP_MSTPCRB_SPI1;
      case RA_MSTP_USBFS:  return R_MSTP_MSTPCRB_USBFS;
      case RA_MSTP_IIC0:   return R_MSTP_MSTPCRB_IIC0;
      case RA_MSTP_IIC1:   return R_MSTP_MSTPCRB_IIC1;
      case RA_MSTP_IIC2:   return R_MSTP_MSTPCRB_IIC2;
      case RA_MSTP_CANFD:  return R_MSTP_MSTPCRB_CANFD;

      /* MSTPCRC register modules */
      case RA_MSTP_SCE5:   return R_MSTP_MSTPCRC_SCE5;
      case RA_MSTP_TRNG:   return R_MSTP_MSTPCRC_TRNG;
      case RA_MSTP_JPEG:   return R_MSTP_MSTPCRC_JPEG;
      case RA_MSTP_EDMAC0: return R_MSTP_MSTPCRC_EDMAC0;
      case RA_MSTP_ELC:    return R_MSTP_MSTPCRC_ELC;
      case RA_MSTP_DOC:    return R_MSTP_MSTPCRC_DOC;
      case RA_MSTP_SSIE0:  return R_MSTP_MSTPCRC_SSIE0;
      case RA_MSTP_SLCDC:  return R_MSTP_MSTPCRC_SLCDC;
      case RA_MSTP_CTSU:   return R_MSTP_MSTPCRC_CTSU;
      case RA_MSTP_CRC:    return R_MSTP_MSTPCRC_CRC;
      case RA_MSTP_CAC:    return R_MSTP_MSTPCRC_CAC;

      /* MSTPCRD register modules */
      case RA_MSTP_OPAMP:  return R_MSTP_MSTPCRD_OPAMP;
      case RA_MSTP_ACMPLP: return R_MSTP_MSTPCRD_ACMPLP;
      case RA_MSTP_ACMPHS: return R_MSTP_MSTPCRD_ACMPHS;
      case RA_MSTP_ULPT1:  return R_MSTP_MSTPCRD_ULPT1;
      case RA_MSTP_ULPT0:  return R_MSTP_MSTPCRD_ULPT0;
      case RA_MSTP_CEU:    return R_MSTP_MSTPCRD_CEU;
      case RA_MSTP_DAC12:  return R_MSTP_MSTPCRD_DAC12;
      case RA_MSTP_DAC8:   return R_MSTP_MSTPCRD_DAC8;
      case RA_MSTP_TSN:    return R_MSTP_MSTPCRD_TSN;
      case RA_MSTP_ADC1:   return R_MSTP_MSTPCRD_ADC1;
      case RA_MSTP_ADC0:   return R_MSTP_MSTPCRD_ADC0;
      case RA_MSTP_POEG:   return R_MSTP_MSTPCRD_POEG;
      case RA_MSTP_GPT_1:  return R_MSTP_MSTPCRD_GPT_1;
      case RA_MSTP_GPT_2:  return R_MSTP_MSTPCRD_GPT_2;
      case RA_MSTP_AGT0:   return R_MSTP_MSTPCRD_AGT0;
      case RA_MSTP_AGT1:   return R_MSTP_MSTPCRD_AGT1;

      default:
        return 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra_mstp_initialize
 *
 * Description:
 *   Initialize the MSTP driver
 *
 ****************************************************************************/

void ra_mstp_initialize(void)
{
  /* All modules start in stopped state by default in hardware */
  /* No additional initialization required */
}

/****************************************************************************
 * Name: ra_mstp_start
 *
 * Description:
 *   Start (enable clock for) a peripheral module
 *
 ****************************************************************************/

int ra_mstp_start(ra_mstp_module_t module)
{
  uint32_t regaddr;
  uint32_t bitmask;

  regaddr = ra_mstp_get_regaddr(module);
  if (regaddr == 0)
    {
      return -EINVAL;
    }

  bitmask = ra_mstp_get_bitmask(module);
  if (bitmask == 0)
    {
      return -EINVAL;
    }

  /* Clear the stop bit to start the module */
  modifyreg32(regaddr, bitmask, 0);

  return OK;
}

/****************************************************************************
 * Name: ra_mstp_stop
 *
 * Description:
 *   Stop (disable clock for) a peripheral module
 *
 ****************************************************************************/

int ra_mstp_stop(ra_mstp_module_t module)
{
  uint32_t regaddr;
  uint32_t bitmask;

  regaddr = ra_mstp_get_regaddr(module);
  if (regaddr == 0)
    {
      return -EINVAL;
    }

  bitmask = ra_mstp_get_bitmask(module);
  if (bitmask == 0)
    {
      return -EINVAL;
    }

  /* Set the stop bit to stop the module */
  modifyreg32(regaddr, 0, bitmask);

  return OK;
}

/****************************************************************************
 * Name: ra_mstp_is_stopped
 *
 * Description:
 *   Check if a peripheral module is stopped
 *
 ****************************************************************************/

bool ra_mstp_is_stopped(ra_mstp_module_t module)
{
  uint32_t regaddr;
  uint32_t bitmask;
  uint32_t regval;

  regaddr = ra_mstp_get_regaddr(module);
  if (regaddr == 0)
    {
      return true; /* Invalid module is considered stopped */
    }

  bitmask = ra_mstp_get_bitmask(module);
  if (bitmask == 0)
    {
      return true; /* Invalid module is considered stopped */
    }

  regval = getreg32(regaddr);
  return (regval & bitmask) != 0;
}

/****************************************************************************
 * Name: ra_mstp_start_multiple
 *
 * Description:
 *   Start multiple modules atomically
 *
 ****************************************************************************/

int ra_mstp_start_multiple(const ra_mstp_module_t *modules, int count)
{
  uint32_t regb_mask = 0;
  uint32_t regc_mask = 0;
  uint32_t regd_mask = 0;
  int i;

  if (modules == NULL || count <= 0)
    {
      return -EINVAL;
    }

  /* Build masks for each register */
  for (i = 0; i < count; i++)
    {
      uint32_t regaddr = ra_mstp_get_regaddr(modules[i]);
      uint32_t bitmask = ra_mstp_get_bitmask(modules[i]);

      if (regaddr == 0 || bitmask == 0)
        {
          return -EINVAL;
        }

      if (regaddr == R_MSTP_MSTPCRB)
        {
          regb_mask |= bitmask;
        }
      else if (regaddr == R_MSTP_MSTPCRC)
        {
          regc_mask |= bitmask;
        }
      else if (regaddr == R_MSTP_MSTPCRD)
        {
          regd_mask |= bitmask;
        }
    }

  /* Apply masks atomically */
  if (regb_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRB, regb_mask, 0);
    }

  if (regc_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRC, regc_mask, 0);
    }

  if (regd_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRD, regd_mask, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: ra_mstp_stop_multiple
 *
 * Description:
 *   Stop multiple modules atomically
 *
 ****************************************************************************/

int ra_mstp_stop_multiple(const ra_mstp_module_t *modules, int count)
{
  uint32_t regb_mask = 0;
  uint32_t regc_mask = 0;
  uint32_t regd_mask = 0;
  int i;

  if (modules == NULL || count <= 0)
    {
      return -EINVAL;
    }

  /* Build masks for each register */
  for (i = 0; i < count; i++)
    {
      uint32_t regaddr = ra_mstp_get_regaddr(modules[i]);
      uint32_t bitmask = ra_mstp_get_bitmask(modules[i]);

      if (regaddr == 0 || bitmask == 0)
        {
          return -EINVAL;
        }

      if (regaddr == R_MSTP_MSTPCRB)
        {
          regb_mask |= bitmask;
        }
      else if (regaddr == R_MSTP_MSTPCRC)
        {
          regc_mask |= bitmask;
        }
      else if (regaddr == R_MSTP_MSTPCRD)
        {
          regd_mask |= bitmask;
        }
    }

  /* Apply masks atomically */
  if (regb_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRB, 0, regb_mask);
    }

  if (regc_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRC, 0, regc_mask);
    }

  if (regd_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRD, 0, regd_mask);
    }

  return OK;
}

/****************************************************************************
 * Name: ra_mstp_get_status
 *
 * Description:
 *   Get the current MSTP register status
 *
 ****************************************************************************/

void ra_mstp_get_status(ra_mstp_status_t *status)
{
  if (status != NULL)
    {
      status->mstpcrb = getreg32(R_MSTP_MSTPCRB);
      status->mstpcrc = getreg32(R_MSTP_MSTPCRC);
      status->mstpcrd = getreg32(R_MSTP_MSTPCRD);
    }
}
