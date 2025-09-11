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
  if (module >= RA_MSTP_UNNECESSARY && module <= RA_MSTP_DMAC_DTC)
    {
      return R_MSTP_MSTPCRA;
    }
  else if (module >= RA_MSTP_SCI0 && module <= RA_MSTP_IIC1)
    {
      return R_MSTP_MSTPCRB;
    }
  else if (module >= RA_MSTP_SCE5 && module <= RA_MSTP_CAC)
    {
      return R_MSTP_MSTPCRC;
    }
  else if (module >= RA_MSTP_ACMPHS0 && module <= RA_MSTP_AGT1)
    {
      return R_MSTP_MSTPCRD;
    }
  else if (module >= RA_MSTP_GPT0 && module <= RA_MSTP_ULPT1)
    {
      return R_MSTP_MSTPCRE;
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
      /* MSTPCRA register modules */
      case RA_MSTP_UNNECESSARY:   return (1 <<  0);  /* MSTPA0 */
      case RA_MSTP_SRAM1:         return (1 <<  1);  /* MSTPA1 */
      case RA_MSTP_STANDBY_SRAM:  return (1 << 15);  /* MSTPA15 */
      case RA_MSTP_DMAC_DTC:      return (1 << 22);  /* MSTPA22 */

      /* MSTPCRB register modules */
      case RA_MSTP_SCI0:          return (1 << 31);  /* MSTPB31 */
      case RA_MSTP_SCI1:          return (1 << 30);  /* MSTPB30 */
      case RA_MSTP_SCI2:          return (1 << 29);  /* MSTPB29 */
      case RA_MSTP_SCI3:          return (1 << 28);  /* MSTPB28 */
      case RA_MSTP_SCI4:          return (1 << 27);  /* MSTPB27 */
      case RA_MSTP_SCI9:          return (1 << 22);  /* MSTPB22 */
      case RA_MSTP_SPI0:          return (1 << 19);  /* MSTPB19 */
      case RA_MSTP_SPI1:          return (1 << 18);  /* MSTPB18 */
      case RA_MSTP_SCI10:         return (1 << 16);  /* MSTPB16 */
      case RA_MSTP_ETHERCAT:      return (1 << 15);  /* MSTPB15 */
      case RA_MSTP_USBFS:         return (1 << 11);  /* MSTPB11 */
      case RA_MSTP_IIC0:          return (1 <<  9);  /* MSTPB9 */
      case RA_MSTP_IIC1:          return (1 <<  8);  /* MSTPB8 */

      /* MSTPCRC register modules */
      case RA_MSTP_SCE5:          return (1 << 31);  /* MSTPC31 */
      case RA_MSTP_CANFD0:        return (1 << 27);  /* MSTPC27 */
      case RA_MSTP_CANFD1:        return (1 << 26);  /* MSTPC26 */
      case RA_MSTP_CEU:           return (1 << 16);  /* MSTPC16 */
      case RA_MSTP_ELC:           return (1 << 14);  /* MSTPC14 */
      case RA_MSTP_DOC:           return (1 << 13);  /* MSTPC13 */
      case RA_MSTP_SSIE0:         return (1 <<  8);  /* MSTPC8 */
      case RA_MSTP_SSIE1:         return (1 <<  7);  /* MSTPC7 */
      case RA_MSTP_CRC:           return (1 <<  1);  /* MSTPC1 */
      case RA_MSTP_CAC:           return (1 <<  0);  /* MSTPC0 */

      /* MSTPCRD register modules */
      case RA_MSTP_ACMPHS0:       return (1 << 28);  /* MSTPD28 */
      case RA_MSTP_ACMPHS1:       return (1 << 27);  /* MSTPD27 */
      case RA_MSTP_RTC:           return (1 << 23);  /* MSTPD23 */
      case RA_MSTP_TSN:           return (1 << 22);  /* MSTPD22 */
      case RA_MSTP_DAC12:         return (1 << 20);  /* MSTPD20 */
      case RA_MSTP_ADC0:          return (1 << 16);  /* MSTPD16 */
      case RA_MSTP_ADC1:          return (1 << 15);  /* MSTPD15 */
      case RA_MSTP_POEG0:         return (1 << 14);  /* MSTPD14 */
      case RA_MSTP_POEG1:         return (1 << 13);  /* MSTPD13 */
      case RA_MSTP_POEG2:         return (1 << 12);  /* MSTPD12 */
      case RA_MSTP_POEG3:         return (1 << 11);  /* MSTPD11 */
      case RA_MSTP_AGT0:          return (1 <<  5);  /* MSTPD5 */
      case RA_MSTP_AGT1:          return (1 <<  4);  /* MSTPD4 */

      /* MSTPCRE register modules */
      case RA_MSTP_GPT0:          return (1 << 31);  /* MSTPE31 */
      case RA_MSTP_GPT1:          return (1 << 30);  /* MSTPE30 */
      case RA_MSTP_GPT2:          return (1 << 29);  /* MSTPE29 */
      case RA_MSTP_GPT3:          return (1 << 28);  /* MSTPE28 */
      case RA_MSTP_GPT4:          return (1 << 27);  /* MSTPE27 */
      case RA_MSTP_GPT5:          return (1 << 26);  /* MSTPE26 */
      case RA_MSTP_GPT10:         return (1 << 21);  /* MSTPE21 */
      case RA_MSTP_GPT11:         return (1 << 20);  /* MSTPE20 */
      case RA_MSTP_GPT12:         return (1 << 19);  /* MSTPE19 */
      case RA_MSTP_GPT13:         return (1 << 18);  /* MSTPE18 */
      case RA_MSTP_ULPT0:         return (1 <<  9);  /* MSTPE9 */
      case RA_MSTP_ULPT1:         return (1 <<  8);  /* MSTPE8 */

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
  uint32_t rega_mask = 0;
  uint32_t regb_mask = 0;
  uint32_t regc_mask = 0;
  uint32_t regd_mask = 0;
  uint32_t rege_mask = 0;
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

      if (regaddr == R_MSTP_MSTPCRA)
        {
          rega_mask |= bitmask;
        }
      else if (regaddr == R_MSTP_MSTPCRB)
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
      else if (regaddr == R_MSTP_MSTPCRE)
        {
          rege_mask |= bitmask;
        }
    }

  /* Apply masks atomically */
  if (rega_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRA, rega_mask, 0);
    }

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

  if (rege_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRE, rege_mask, 0);
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
  uint32_t rega_mask = 0;
  uint32_t regb_mask = 0;
  uint32_t regc_mask = 0;
  uint32_t regd_mask = 0;
  uint32_t rege_mask = 0;
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

      if (regaddr == R_MSTP_MSTPCRA)
        {
          rega_mask |= bitmask;
        }
      else if (regaddr == R_MSTP_MSTPCRB)
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
      else if (regaddr == R_MSTP_MSTPCRE)
        {
          rege_mask |= bitmask;
        }
    }

  /* Apply masks atomically */
  if (rega_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRA, 0, rega_mask);
    }

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

  if (rege_mask != 0)
    {
      modifyreg32(R_MSTP_MSTPCRE, 0, rege_mask);
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
      status->mstpcra = getreg32(R_MSTP_MSTPCRA);
      status->mstpcrb = getreg32(R_MSTP_MSTPCRB);
      status->mstpcrc = getreg32(R_MSTP_MSTPCRC);
      status->mstpcrd = getreg32(R_MSTP_MSTPCRD);
      status->mstpcre = getreg32(R_MSTP_MSTPCRE);
    }
}
