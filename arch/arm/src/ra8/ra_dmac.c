/****************************************************************************
 * arch/arm/src/ra8/ra_dmac.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef CONFIG_RA_DMAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
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
 * Name: ra_dmac_initialize
 *
 * Description:
 *   Initialize the DMAC controller
 *
 ****************************************************************************/

int ra_dmac_initialize(void)
{
  /* TODO: Implement DMAC initialization */

  return 0;  /* OK */
}

/****************************************************************************
 * Name: ra_dmac_channel_alloc
 *
 * Description:
 *   Allocate a DMAC channel
 *
 ****************************************************************************/

int ra_dmac_channel_alloc(void)
{
  /* TODO: Implement DMAC channel allocation */

  return -1;  /* ENOSYS */
}

/****************************************************************************
 * Name: ra_dmac_channel_free
 *
 * Description:
 *   Free a DMAC channel
 *
 ****************************************************************************/

int ra_dmac_channel_free(int channel)
{
  /* TODO: Implement DMAC channel free */

  (void)channel;  /* Suppress unused parameter warning */
  return 0;  /* OK */
}

/****************************************************************************
 * Name: ra_dmac_start
 *
 * Description:
 *   Start DMAC transfer
 *
 ****************************************************************************/

int ra_dmac_start(int channel, uint32_t src, uint32_t dst, uint32_t len)
{
  /* TODO: Implement DMAC transfer start */

  (void)channel;  /* Suppress unused parameter warnings */
  (void)src;
  (void)dst;
  (void)len;

  return -1;  /* ENOSYS */
}

/****************************************************************************
 * Name: ra_dmac_stop
 *
 * Description:
 *   Stop DMAC transfer
 *
 ****************************************************************************/

int ra_dmac_stop(int channel)
{
  /* TODO: Implement DMAC transfer stop */

  (void)channel;  /* Suppress unused parameter warning */
  return 0;  /* OK */
}

#endif /* CONFIG_RA_DMAC */
