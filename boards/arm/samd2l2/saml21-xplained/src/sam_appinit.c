/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/src/sam_appinit.c
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

#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>

#include "sam_config.h"
#include "saml21-xplained.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Some configuration checks */

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE_EXT1
#  ifndef SAMD2L2_HAVE_SPI0
#    error I/O1 module on EXT1 requires SERCOM SPI0
#    undef CONFIG_SAML21_XPLAINED_IOMODULE
#  endif
#  define SPI_PORTNO 0
#endif

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE_EXT2
#  ifndef SAMD2L2_HAVE_SPI1
#    error I/O1 module on EXT2 requires SERCOM SPI1
#    undef CONFIG_SAML21_XPLAINED_IOMODULE
#  endif
#  define SPI_PORTNO 1
#endif

#ifdef CONFIG_SAML21_XPLAINED_IOMODULE
/* Support for the SD card slot on the I/O1 module */

/* Verify NSH PORT and SLOT settings */

#  define SAMD2L2_MMCSDSLOTNO  0 /* There is only one slot */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != SAMD2L2_MMCSDSLOTNO
#    error Only one MMC/SD slot:  Slot 0 (CONFIG_NSH_MMCSDSLOTNO)
#  endif

#  if defined(CONFIG_NSH_MMCSDSPIPORTNO) && CONFIG_NSH_MMCSDSPIPORTNO != SPI_PORTNO
#    error CONFIG_NSH_MMCSDSPIPORTNO must have the same value as SPI_PORTNO
#  endif

/* Default MMC/SD minor number */

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#if defined(SAMD2L2_HAVE_SPI0) && defined(CONFIG_SAML21_XPLAINED_IOMODULE)
  /* Initialize the SPI-based MMC/SD slot */

  int ret = sam_sdinitialize(SPI_PORTNO, CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot: %d\n",
             ret);
      return ret;
    }
#endif

  return OK;
}
