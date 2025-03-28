/****************************************************************************
 * arch/arm/src/efm32/hardware/efm32_burtc.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2014 Silicon Laboratories, Inc.
 * SPDX-FileCopyrightText: 2014 Pierre-noel Bouteville . All rights reserved.
 * SPDX-FileCopyrightText: 2014 Gregory Nutt. All rights reserved.
 * SPDX-FileContributor: Pierre-noel Bouteville <pnb990@gmail.com>
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of
 * any kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties
 * against infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_BURTC_H
#define __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_BURTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_memorymap.h"

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning This is the EFM32GG header file; Review/modification needed for this architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EFM32_BURTC_NREGS                     128 /* Number of backup retention registers */

/* BURTC Register Offsets ***************************************************/

#define EFM32_BURTC_CTRL_OFFSET               0x0000  /* Control Register */
#define EFM32_BURTC_LPMODE_OFFSET             0x0004  /* Low power mode configuration */
#define EFM32_BURTC_CNT_OFFSET                0x0008  /* Counter Value Register */
#define EFM32_BURTC_COMP0_OFFSET              0x000c  /* Counter Compare Value */
#define EFM32_BURTC_TIMESTAMP_OFFSET          0x0010  /* Backup mode timestamp */
#define EFM32_BURTC_LFXOFDET_OFFSET           0x0014  /* LFXO */
#define EFM32_BURTC_STATUS_OFFSET             0x0018  /* Backup domain status */
#define EFM32_BURTC_CMD_OFFSET                0x001c  /* Command Register */
#define EFM32_BURTC_POWERDOWN_OFFSET          0x0020  /* Retention RAM power-down register */
#define EFM32_BURTC_LOCK_OFFSET               0x0024  /* Configuration Lock Register */
#define EFM32_BURTC_IF_OFFSET                 0x0028  /* Interrupt Flag Register */
#define EFM32_BURTC_IFS_OFFSET                0x002c  /* Interrupt Flag Set Register */
#define EFM32_BURTC_IFC_OFFSET                0x0030  /* Interrupt Flag Clear Register */
#define EFM32_BURTC_IEN_OFFSET                0x0034  /* Interrupt Enable Register */
#define EFM32_BURTC_FREEZE_OFFSET             0x0038  /* Freeze Register */
#define EFM32_BURTC_SYNCBUSY_OFFSET           0x003c  /* Synchronization Busy Register */

/* Backup retention register */

#define EFM32_BURTC_RET_REG_OFFSET(n)         (0x0100 + ((n) << 2))

/* BURTC Register Addresses *************************************************/

#define EFM32_BURTC_CTRL                      (EFM32_BCKRTC_BASE+EFM32_BURTC_CTRL_OFFSET)
#define EFM32_BURTC_LPMODE                    (EFM32_BCKRTC_BASE+EFM32_BURTC_LPMODE_OFFSET)
#define EFM32_BURTC_CNT                       (EFM32_BCKRTC_BASE+EFM32_BURTC_CNT_OFFSET)
#define EFM32_BURTC_COMP0                     (EFM32_BCKRTC_BASE+EFM32_BURTC_COMP0_OFFSET)
#define EFM32_BURTC_TIMESTAMP                 (EFM32_BCKRTC_BASE+EFM32_BURTC_TIMESTAMP_OFFSET)
#define EFM32_BURTC_LFXOFDET                  (EFM32_BCKRTC_BASE+EFM32_BURTC_LFXOFDET_OFFSET)
#define EFM32_BURTC_STATUS                    (EFM32_BCKRTC_BASE+EFM32_BURTC_STATUS_OFFSET)
#define EFM32_BURTC_CMD                       (EFM32_BCKRTC_BASE+EFM32_BURTC_CMD_OFFSET)
#define EFM32_BURTC_POWERDOWN                 (EFM32_BCKRTC_BASE+EFM32_BURTC_POWERDOWN_OFFSET)
#define EFM32_BURTC_LOCK                      (EFM32_BCKRTC_BASE+EFM32_BURTC_LOCK_OFFSET)
#define EFM32_BURTC_IF                        (EFM32_BCKRTC_BASE+EFM32_BURTC_IF_OFFSET)
#define EFM32_BURTC_IFS                       (EFM32_BCKRTC_BASE+EFM32_BURTC_IFS_OFFSET)
#define EFM32_BURTC_IFC                       (EFM32_BCKRTC_BASE+EFM32_BURTC_IFC_OFFSET)
#define EFM32_BURTC_IEN                       (EFM32_BCKRTC_BASE+EFM32_BURTC_IEN_OFFSET)
#define EFM32_BURTC_FREEZE                    (EFM32_BCKRTC_BASE+EFM32_BURTC_FREEZE_OFFSET)
#define EFM32_BURTC_SYNCBUSY                  (EFM32_BCKRTC_BASE+EFM32_BURTC_SYNCBUSY_OFFSET)

/* Backup retention register */

#define EFM32_BURTC_RET_REG(n)                (EFM32_BCKRTC_BASE+EFM32_BURTC_RET_REG_OFFSET(n))

/* BURTC Register Bit Field Definitions *************************************/

/* Bit fields for BURTC CTRL */

#define _BURTC_CTRL_RESETVALUE                0x00000008UL                           /* Default value for BURTC_CTRL */
#define _BURTC_CTRL_MASK                      0x000077FFUL                           /* Mask for BURTC_CTRL */

#define _BURTC_CTRL_MODE_SHIFT                0                                      /* Shift value for BURTC_MODE */
#define _BURTC_CTRL_MODE_MASK                 0x3UL                                  /* Bit mask for BURTC_MODE */
#define _BURTC_CTRL_MODE_DEFAULT              0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define _BURTC_CTRL_MODE_DISABLE              0x00000000UL                           /* Mode DISABLE for BURTC_CTRL */
#define _BURTC_CTRL_MODE_EM2EN                0x00000001UL                           /* Mode EM2EN for BURTC_CTRL */
#define _BURTC_CTRL_MODE_EM3EN                0x00000002UL                           /* Mode EM3EN for BURTC_CTRL */
#define _BURTC_CTRL_MODE_EM4EN                0x00000003UL                           /* Mode EM4EN for BURTC_CTRL */
#define BURTC_CTRL_MODE_DEFAULT               (_BURTC_CTRL_MODE_DEFAULT << 0)        /* Shifted mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_MODE_DISABLE               (_BURTC_CTRL_MODE_DISABLE << 0)        /* Shifted mode DISABLE for BURTC_CTRL */
#define BURTC_CTRL_MODE_EM2EN                 (_BURTC_CTRL_MODE_EM2EN << 0)          /* Shifted mode EM2EN for BURTC_CTRL */
#define BURTC_CTRL_MODE_EM3EN                 (_BURTC_CTRL_MODE_EM3EN << 0)          /* Shifted mode EM3EN for BURTC_CTRL */
#define BURTC_CTRL_MODE_EM4EN                 (_BURTC_CTRL_MODE_EM4EN << 0)          /* Shifted mode EM4EN for BURTC_CTRL */
#define BURTC_CTRL_DEBUGRUN                   (0x1UL << 2)                           /* Debug Mode Run Enable */
#define _BURTC_CTRL_DEBUGRUN_SHIFT            2                                      /* Shift value for BURTC_DEBUGRUN */
#define _BURTC_CTRL_DEBUGRUN_MASK             0x4UL                                  /* Bit mask for BURTC_DEBUGRUN */
#define _BURTC_CTRL_DEBUGRUN_DEFAULT          0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_DEBUGRUN_DEFAULT           (_BURTC_CTRL_DEBUGRUN_DEFAULT << 2)    /* Shifted mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_RSTEN                      (0x1UL << 3)                           /* Enable BURTC reset */
#define _BURTC_CTRL_RSTEN_SHIFT               3                                      /* Shift value for BURTC_RSTEN */
#define _BURTC_CTRL_RSTEN_MASK                0x8UL                                  /* Bit mask for BURTC_RSTEN */
#define _BURTC_CTRL_RSTEN_DEFAULT             0x00000001UL                           /* Mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_RSTEN_DEFAULT              (_BURTC_CTRL_RSTEN_DEFAULT << 3)       /* Shifted mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_COMP0TOP                   (0x1UL << 4)                           /* Compare clear enable */
#define _BURTC_CTRL_COMP0TOP_SHIFT            4                                      /* Shift value for BURTC_COMP0TOP */
#define _BURTC_CTRL_COMP0TOP_MASK             0x10UL                                 /* Bit mask for BURTC_COMP0TOP */
#define _BURTC_CTRL_COMP0TOP_DEFAULT          0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_COMP0TOP_DEFAULT           (_BURTC_CTRL_COMP0TOP_DEFAULT << 4)    /* Shifted mode DEFAULT for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_SHIFT              5                                      /* Shift value for BURTC_LPCOMP */
#define _BURTC_CTRL_LPCOMP_MASK               0xE0UL                                 /* Bit mask for BURTC_LPCOMP */
#define _BURTC_CTRL_LPCOMP_DEFAULT            0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN0LSB            0x00000000UL                           /* Mode IGN0LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN1LSB            0x00000001UL                           /* Mode IGN1LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN2LSB            0x00000002UL                           /* Mode IGN2LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN3LSB            0x00000003UL                           /* Mode IGN3LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN4LSB            0x00000004UL                           /* Mode IGN4LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN5LSB            0x00000005UL                           /* Mode IGN5LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN6LSB            0x00000006UL                           /* Mode IGN6LSB for BURTC_CTRL */
#define _BURTC_CTRL_LPCOMP_IGN7LSB            0x00000007UL                           /* Mode IGN7LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_DEFAULT             (_BURTC_CTRL_LPCOMP_DEFAULT << 5)      /* Shifted mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN0LSB             (_BURTC_CTRL_LPCOMP_IGN0LSB << 5)      /* Shifted mode IGN0LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN1LSB             (_BURTC_CTRL_LPCOMP_IGN1LSB << 5)      /* Shifted mode IGN1LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN2LSB             (_BURTC_CTRL_LPCOMP_IGN2LSB << 5)      /* Shifted mode IGN2LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN3LSB             (_BURTC_CTRL_LPCOMP_IGN3LSB << 5)      /* Shifted mode IGN3LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN4LSB             (_BURTC_CTRL_LPCOMP_IGN4LSB << 5)      /* Shifted mode IGN4LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN5LSB             (_BURTC_CTRL_LPCOMP_IGN5LSB << 5)      /* Shifted mode IGN5LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN6LSB             (_BURTC_CTRL_LPCOMP_IGN6LSB << 5)      /* Shifted mode IGN6LSB for BURTC_CTRL */
#define BURTC_CTRL_LPCOMP_IGN7LSB             (_BURTC_CTRL_LPCOMP_IGN7LSB << 5)      /* Shifted mode IGN7LSB for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_SHIFT               8                                      /* Shift value for BURTC_PRESC */
#define _BURTC_CTRL_PRESC_MASK                0x700UL                                /* Bit mask for BURTC_PRESC */
#define _BURTC_CTRL_PRESC_DEFAULT             0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV1                0x00000000UL                           /* Mode DIV1 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV2                0x00000001UL                           /* Mode DIV2 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV4                0x00000002UL                           /* Mode DIV4 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV8                0x00000003UL                           /* Mode DIV8 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV16               0x00000004UL                           /* Mode DIV16 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV32               0x00000005UL                           /* Mode DIV32 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV64               0x00000006UL                           /* Mode DIV64 for BURTC_CTRL */
#define _BURTC_CTRL_PRESC_DIV128              0x00000007UL                           /* Mode DIV128 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DEFAULT              (_BURTC_CTRL_PRESC_DEFAULT << 8)       /* Shifted mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV1                 (_BURTC_CTRL_PRESC_DIV1 << 8)          /* Shifted mode DIV1 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV2                 (_BURTC_CTRL_PRESC_DIV2 << 8)          /* Shifted mode DIV2 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV4                 (_BURTC_CTRL_PRESC_DIV4 << 8)          /* Shifted mode DIV4 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV8                 (_BURTC_CTRL_PRESC_DIV8 << 8)          /* Shifted mode DIV8 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV16                (_BURTC_CTRL_PRESC_DIV16 << 8)         /* Shifted mode DIV16 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV32                (_BURTC_CTRL_PRESC_DIV32 << 8)         /* Shifted mode DIV32 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV64                (_BURTC_CTRL_PRESC_DIV64 << 8)         /* Shifted mode DIV64 for BURTC_CTRL */
#define BURTC_CTRL_PRESC_DIV128               (_BURTC_CTRL_PRESC_DIV128 << 8)        /* Shifted mode DIV128 for BURTC_CTRL */
#define _BURTC_CTRL_CLKSEL_SHIFT              12                                     /* Shift value for BURTC_CLKSEL */
#define _BURTC_CTRL_CLKSEL_MASK               0x3000UL                               /* Bit mask for BURTC_CLKSEL */
#define _BURTC_CTRL_CLKSEL_DEFAULT            0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define _BURTC_CTRL_CLKSEL_NONE               0x00000000UL                           /* Mode NONE for BURTC_CTRL */
#define _BURTC_CTRL_CLKSEL_LFRCO              0x00000001UL                           /* Mode LFRCO for BURTC_CTRL */
#define _BURTC_CTRL_CLKSEL_LFXO               0x00000002UL                           /* Mode LFXO for BURTC_CTRL */
#define _BURTC_CTRL_CLKSEL_ULFRCO             0x00000003UL                           /* Mode ULFRCO for BURTC_CTRL */
#define BURTC_CTRL_CLKSEL_DEFAULT             (_BURTC_CTRL_CLKSEL_DEFAULT << 12)     /* Shifted mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_CLKSEL_NONE                (_BURTC_CTRL_CLKSEL_NONE << 12)        /* Shifted mode NONE for BURTC_CTRL */
#define BURTC_CTRL_CLKSEL_LFRCO               (_BURTC_CTRL_CLKSEL_LFRCO << 12)       /* Shifted mode LFRCO for BURTC_CTRL */
#define BURTC_CTRL_CLKSEL_LFXO                (_BURTC_CTRL_CLKSEL_LFXO << 12)        /* Shifted mode LFXO for BURTC_CTRL */
#define BURTC_CTRL_CLKSEL_ULFRCO              (_BURTC_CTRL_CLKSEL_ULFRCO << 12)      /* Shifted mode ULFRCO for BURTC_CTRL */
#define BURTC_CTRL_BUMODETSEN                 (0x1UL << 14)                          /* Backup mode timestamp enable */
#define _BURTC_CTRL_BUMODETSEN_SHIFT          14                                     /* Shift value for BURTC_BUMODETSEN */
#define _BURTC_CTRL_BUMODETSEN_MASK           0x4000UL                               /* Bit mask for BURTC_BUMODETSEN */
#define _BURTC_CTRL_BUMODETSEN_DEFAULT        0x00000000UL                           /* Mode DEFAULT for BURTC_CTRL */
#define BURTC_CTRL_BUMODETSEN_DEFAULT         (_BURTC_CTRL_BUMODETSEN_DEFAULT << 14) /* Shifted mode DEFAULT for BURTC_CTRL */

/* Bit fields for BURTC LPMODE */

#define _BURTC_LPMODE_RESETVALUE              0x00000000UL                        /* Default value for BURTC_LPMODE */
#define _BURTC_LPMODE_MASK                    0x00000003UL                        /* Mask for BURTC_LPMODE */

#define _BURTC_LPMODE_LPMODE_SHIFT            0                                   /* Shift value for BURTC_LPMODE */
#define _BURTC_LPMODE_LPMODE_MASK             0x3UL                               /* Bit mask for BURTC_LPMODE */
#define _BURTC_LPMODE_LPMODE_DEFAULT          0x00000000UL                        /* Mode DEFAULT for BURTC_LPMODE */
#define _BURTC_LPMODE_LPMODE_DISABLE          0x00000000UL                        /* Mode DISABLE for BURTC_LPMODE */
#define _BURTC_LPMODE_LPMODE_ENABLE           0x00000001UL                        /* Mode ENABLE for BURTC_LPMODE */
#define _BURTC_LPMODE_LPMODE_BUEN             0x00000002UL                        /* Mode BUEN for BURTC_LPMODE */
#define BURTC_LPMODE_LPMODE_DEFAULT           (_BURTC_LPMODE_LPMODE_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_LPMODE */
#define BURTC_LPMODE_LPMODE_DISABLE           (_BURTC_LPMODE_LPMODE_DISABLE << 0) /* Shifted mode DISABLE for BURTC_LPMODE */
#define BURTC_LPMODE_LPMODE_ENABLE            (_BURTC_LPMODE_LPMODE_ENABLE << 0)  /* Shifted mode ENABLE for BURTC_LPMODE */
#define BURTC_LPMODE_LPMODE_BUEN              (_BURTC_LPMODE_LPMODE_BUEN << 0)    /* Shifted mode BUEN for BURTC_LPMODE */

/* Bit fields for BURTC CNT */

#define _BURTC_CNT_RESETVALUE                 0x00000000UL                  /* Default value for BURTC_CNT */
#define _BURTC_CNT_MASK                       0xFFFFFFFFUL                  /* Mask for BURTC_CNT */

#define _BURTC_CNT_CNT_SHIFT                  0                             /* Shift value for BURTC_CNT */
#define _BURTC_CNT_CNT_MASK                   0xFFFFFFFFUL                  /* Bit mask for BURTC_CNT */
#define _BURTC_CNT_CNT_DEFAULT                0x00000000UL                  /* Mode DEFAULT for BURTC_CNT */
#define BURTC_CNT_CNT_DEFAULT                 (_BURTC_CNT_CNT_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_CNT */

/* Bit fields for BURTC COMP0 */

#define _BURTC_COMP0_RESETVALUE               0x00000000UL                      /* Default value for BURTC_COMP0 */
#define _BURTC_COMP0_MASK                     0xFFFFFFFFUL                      /* Mask for BURTC_COMP0 */

#define _BURTC_COMP0_COMP0_SHIFT              0                                 /* Shift value for BURTC_COMP0 */
#define _BURTC_COMP0_COMP0_MASK               0xFFFFFFFFUL                      /* Bit mask for BURTC_COMP0 */
#define _BURTC_COMP0_COMP0_DEFAULT            0x00000000UL                      /* Mode DEFAULT for BURTC_COMP0 */
#define BURTC_COMP0_COMP0_DEFAULT             (_BURTC_COMP0_COMP0_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_COMP0 */

/* Bit fields for BURTC TIMESTAMP */

#define _BURTC_TIMESTAMP_RESETVALUE           0x00000000UL                              /* Default value for BURTC_TIMESTAMP */
#define _BURTC_TIMESTAMP_MASK                 0xFFFFFFFFUL                              /* Mask for BURTC_TIMESTAMP */

#define _BURTC_TIMESTAMP_TIMESTAMP_SHIFT      0                                         /* Shift value for BURTC_TIMESTAMP */
#define _BURTC_TIMESTAMP_TIMESTAMP_MASK       0xFFFFFFFFUL                              /* Bit mask for BURTC_TIMESTAMP */
#define _BURTC_TIMESTAMP_TIMESTAMP_DEFAULT    0x00000000UL                              /* Mode DEFAULT for BURTC_TIMESTAMP */
#define BURTC_TIMESTAMP_TIMESTAMP_DEFAULT     (_BURTC_TIMESTAMP_TIMESTAMP_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_TIMESTAMP */

/* Bit fields for BURTC LFXOFDET */

#define _BURTC_LFXOFDET_RESETVALUE            0x00000000UL                       /* Default value for BURTC_LFXOFDET */
#define _BURTC_LFXOFDET_MASK                  0x000001F3UL                       /* Mask for BURTC_LFXOFDET */

#define _BURTC_LFXOFDET_OSC_SHIFT             0                                  /* Shift value for BURTC_OSC */
#define _BURTC_LFXOFDET_OSC_MASK              0x3UL                              /* Bit mask for BURTC_OSC */
#define _BURTC_LFXOFDET_OSC_DEFAULT           0x00000000UL                       /* Mode DEFAULT for BURTC_LFXOFDET */
#define _BURTC_LFXOFDET_OSC_DISABLE           0x00000000UL                       /* Mode DISABLE for BURTC_LFXOFDET */
#define _BURTC_LFXOFDET_OSC_LFRCO             0x00000001UL                       /* Mode LFRCO for BURTC_LFXOFDET */
#define _BURTC_LFXOFDET_OSC_ULFRCO            0x00000002UL                       /* Mode ULFRCO for BURTC_LFXOFDET */
#define BURTC_LFXOFDET_OSC_DEFAULT            (_BURTC_LFXOFDET_OSC_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_LFXOFDET */
#define BURTC_LFXOFDET_OSC_DISABLE            (_BURTC_LFXOFDET_OSC_DISABLE << 0) /* Shifted mode DISABLE for BURTC_LFXOFDET */
#define BURTC_LFXOFDET_OSC_LFRCO              (_BURTC_LFXOFDET_OSC_LFRCO << 0)   /* Shifted mode LFRCO for BURTC_LFXOFDET */
#define BURTC_LFXOFDET_OSC_ULFRCO             (_BURTC_LFXOFDET_OSC_ULFRCO << 0)  /* Shifted mode ULFRCO for BURTC_LFXOFDET */
#define _BURTC_LFXOFDET_TOP_SHIFT             4                                  /* Shift value for BURTC_TOP */
#define _BURTC_LFXOFDET_TOP_MASK              0x1F0UL                            /* Bit mask for BURTC_TOP */
#define _BURTC_LFXOFDET_TOP_DEFAULT           0x00000000UL                       /* Mode DEFAULT for BURTC_LFXOFDET */
#define BURTC_LFXOFDET_TOP_DEFAULT            (_BURTC_LFXOFDET_TOP_DEFAULT << 4) /* Shifted mode DEFAULT for BURTC_LFXOFDET */

/* Bit fields for BURTC STATUS */

#define _BURTC_STATUS_RESETVALUE              0x00000000UL                           /* Default value for BURTC_STATUS */
#define _BURTC_STATUS_MASK                    0x00000007UL                           /* Mask for BURTC_STATUS */

#define BURTC_STATUS_LPMODEACT                (0x1UL << 0)                           /* Low power mode active */
#define _BURTC_STATUS_LPMODEACT_SHIFT         0                                      /* Shift value for BURTC_LPMODEACT */
#define _BURTC_STATUS_LPMODEACT_MASK          0x1UL                                  /* Bit mask for BURTC_LPMODEACT */
#define _BURTC_STATUS_LPMODEACT_DEFAULT       0x00000000UL                           /* Mode DEFAULT for BURTC_STATUS */
#define BURTC_STATUS_LPMODEACT_DEFAULT        (_BURTC_STATUS_LPMODEACT_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_STATUS */
#define BURTC_STATUS_BUMODETS                 (0x1UL << 1)                           /* Timestamp for backup mode entry stored. */
#define _BURTC_STATUS_BUMODETS_SHIFT          1                                      /* Shift value for BURTC_BUMODETS */
#define _BURTC_STATUS_BUMODETS_MASK           0x2UL                                  /* Bit mask for BURTC_BUMODETS */
#define _BURTC_STATUS_BUMODETS_DEFAULT        0x00000000UL                           /* Mode DEFAULT for BURTC_STATUS */
#define BURTC_STATUS_BUMODETS_DEFAULT         (_BURTC_STATUS_BUMODETS_DEFAULT << 1)  /* Shifted mode DEFAULT for BURTC_STATUS */
#define BURTC_STATUS_RAMWERR                  (0x1UL << 2)                           /* RAM write error. */
#define _BURTC_STATUS_RAMWERR_SHIFT           2                                      /* Shift value for BURTC_RAMWERR */
#define _BURTC_STATUS_RAMWERR_MASK            0x4UL                                  /* Bit mask for BURTC_RAMWERR */
#define _BURTC_STATUS_RAMWERR_DEFAULT         0x00000000UL                           /* Mode DEFAULT for BURTC_STATUS */
#define BURTC_STATUS_RAMWERR_DEFAULT          (_BURTC_STATUS_RAMWERR_DEFAULT << 2)   /* Shifted mode DEFAULT for BURTC_STATUS */

/* Bit fields for BURTC CMD */

#define _BURTC_CMD_RESETVALUE                 0x00000000UL                        /* Default value for BURTC_CMD */
#define _BURTC_CMD_MASK                       0x00000001UL                        /* Mask for BURTC_CMD */

#define BURTC_CMD_CLRSTATUS                   (0x1UL << 0)                        /* Clear BURTC_STATUS register. */
#define _BURTC_CMD_CLRSTATUS_SHIFT            0                                   /* Shift value for BURTC_CLRSTATUS */
#define _BURTC_CMD_CLRSTATUS_MASK             0x1UL                               /* Bit mask for BURTC_CLRSTATUS */
#define _BURTC_CMD_CLRSTATUS_DEFAULT          0x00000000UL                        /* Mode DEFAULT for BURTC_CMD */
#define BURTC_CMD_CLRSTATUS_DEFAULT           (_BURTC_CMD_CLRSTATUS_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_CMD */

/* Bit fields for BURTC POWERDOWN */

#define _BURTC_POWERDOWN_RESETVALUE           0x00000000UL                        /* Default value for BURTC_POWERDOWN */
#define _BURTC_POWERDOWN_MASK                 0x00000001UL                        /* Mask for BURTC_POWERDOWN */

#define BURTC_POWERDOWN_RAM                   (0x1UL << 0)                        /* Retention RAM power-down */
#define _BURTC_POWERDOWN_RAM_SHIFT            0                                   /* Shift value for BURTC_RAM */
#define _BURTC_POWERDOWN_RAM_MASK             0x1UL                               /* Bit mask for BURTC_RAM */
#define _BURTC_POWERDOWN_RAM_DEFAULT          0x00000000UL                        /* Mode DEFAULT for BURTC_POWERDOWN */
#define BURTC_POWERDOWN_RAM_DEFAULT           (_BURTC_POWERDOWN_RAM_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_POWERDOWN */

/* Bit fields for BURTC LOCK */

#define _BURTC_LOCK_RESETVALUE                0x00000000UL                        /* Default value for BURTC_LOCK */
#define _BURTC_LOCK_MASK                      0x0000FFFFUL                        /* Mask for BURTC_LOCK */

#define _BURTC_LOCK_LOCKKEY_SHIFT             0                                   /* Shift value for BURTC_LOCKKEY */
#define _BURTC_LOCK_LOCKKEY_MASK              0xFFFFUL                            /* Bit mask for BURTC_LOCKKEY */
#define _BURTC_LOCK_LOCKKEY_DEFAULT           0x00000000UL                        /* Mode DEFAULT for BURTC_LOCK */
#define _BURTC_LOCK_LOCKKEY_LOCK              0x00000000UL                        /* Mode LOCK for BURTC_LOCK */
#define _BURTC_LOCK_LOCKKEY_UNLOCKED          0x00000000UL                        /* Mode UNLOCKED for BURTC_LOCK */
#define _BURTC_LOCK_LOCKKEY_LOCKED            0x00000001UL                        /* Mode LOCKED for BURTC_LOCK */
#define _BURTC_LOCK_LOCKKEY_UNLOCK            0x0000AEE8UL                        /* Mode UNLOCK for BURTC_LOCK */
#define BURTC_LOCK_LOCKKEY_DEFAULT            (_BURTC_LOCK_LOCKKEY_DEFAULT << 0)  /* Shifted mode DEFAULT for BURTC_LOCK */
#define BURTC_LOCK_LOCKKEY_LOCK               (_BURTC_LOCK_LOCKKEY_LOCK << 0)     /* Shifted mode LOCK for BURTC_LOCK */
#define BURTC_LOCK_LOCKKEY_UNLOCKED           (_BURTC_LOCK_LOCKKEY_UNLOCKED << 0) /* Shifted mode UNLOCKED for BURTC_LOCK */
#define BURTC_LOCK_LOCKKEY_LOCKED             (_BURTC_LOCK_LOCKKEY_LOCKED << 0)   /* Shifted mode LOCKED for BURTC_LOCK */
#define BURTC_LOCK_LOCKKEY_UNLOCK             (_BURTC_LOCK_LOCKKEY_UNLOCK << 0)   /* Shifted mode UNLOCK for BURTC_LOCK */

/* Bit fields for BURTC IF */

#define _BURTC_IF_RESETVALUE                  0x00000000UL                      /* Default value for BURTC_IF */
#define _BURTC_IF_MASK                        0x00000007UL                      /* Mask for BURTC_IF */

#define BURTC_IF_OF                           (0x1UL << 0)                      /* Overflow Interrupt Flag */
#define _BURTC_IF_OF_SHIFT                    0                                 /* Shift value for BURTC_OF */
#define _BURTC_IF_OF_MASK                     0x1UL                             /* Bit mask for BURTC_OF */
#define _BURTC_IF_OF_DEFAULT                  0x00000000UL                      /* Mode DEFAULT for BURTC_IF */
#define BURTC_IF_OF_DEFAULT                   (_BURTC_IF_OF_DEFAULT << 0)       /* Shifted mode DEFAULT for BURTC_IF */
#define BURTC_IF_COMP0                        (0x1UL << 1)                      /* Compare match Interrupt Flag */
#define _BURTC_IF_COMP0_SHIFT                 1                                 /* Shift value for BURTC_COMP0 */
#define _BURTC_IF_COMP0_MASK                  0x2UL                             /* Bit mask for BURTC_COMP0 */
#define _BURTC_IF_COMP0_DEFAULT               0x00000000UL                      /* Mode DEFAULT for BURTC_IF */
#define BURTC_IF_COMP0_DEFAULT                (_BURTC_IF_COMP0_DEFAULT << 1)    /* Shifted mode DEFAULT for BURTC_IF */
#define BURTC_IF_LFXOFAIL                     (0x1UL << 2)                      /* LFXO failure Interrupt Flag */
#define _BURTC_IF_LFXOFAIL_SHIFT              2                                 /* Shift value for BURTC_LFXOFAIL */
#define _BURTC_IF_LFXOFAIL_MASK               0x4UL                             /* Bit mask for BURTC_LFXOFAIL */
#define _BURTC_IF_LFXOFAIL_DEFAULT            0x00000000UL                      /* Mode DEFAULT for BURTC_IF */
#define BURTC_IF_LFXOFAIL_DEFAULT             (_BURTC_IF_LFXOFAIL_DEFAULT << 2) /* Shifted mode DEFAULT for BURTC_IF */

/* Bit fields for BURTC IFS */

#define _BURTC_IFS_RESETVALUE                 0x00000000UL                       /* Default value for BURTC_IFS */
#define _BURTC_IFS_MASK                       0x00000007UL                       /* Mask for BURTC_IFS */

#define BURTC_IFS_OF                          (0x1UL << 0)                       /* Set Overflow Interrupt Flag */
#define _BURTC_IFS_OF_SHIFT                   0                                  /* Shift value for BURTC_OF */
#define _BURTC_IFS_OF_MASK                    0x1UL                              /* Bit mask for BURTC_OF */
#define _BURTC_IFS_OF_DEFAULT                 0x00000000UL                       /* Mode DEFAULT for BURTC_IFS */
#define BURTC_IFS_OF_DEFAULT                  (_BURTC_IFS_OF_DEFAULT << 0)       /* Shifted mode DEFAULT for BURTC_IFS */
#define BURTC_IFS_COMP0                       (0x1UL << 1)                       /* Set compare match Interrupt Flag */
#define _BURTC_IFS_COMP0_SHIFT                1                                  /* Shift value for BURTC_COMP0 */
#define _BURTC_IFS_COMP0_MASK                 0x2UL                              /* Bit mask for BURTC_COMP0 */
#define _BURTC_IFS_COMP0_DEFAULT              0x00000000UL                       /* Mode DEFAULT for BURTC_IFS */
#define BURTC_IFS_COMP0_DEFAULT               (_BURTC_IFS_COMP0_DEFAULT << 1)    /* Shifted mode DEFAULT for BURTC_IFS */
#define BURTC_IFS_LFXOFAIL                    (0x1UL << 2)                       /* Set LFXO fail Interrupt Flag */
#define _BURTC_IFS_LFXOFAIL_SHIFT             2                                  /* Shift value for BURTC_LFXOFAIL */
#define _BURTC_IFS_LFXOFAIL_MASK              0x4UL                              /* Bit mask for BURTC_LFXOFAIL */
#define _BURTC_IFS_LFXOFAIL_DEFAULT           0x00000000UL                       /* Mode DEFAULT for BURTC_IFS */
#define BURTC_IFS_LFXOFAIL_DEFAULT            (_BURTC_IFS_LFXOFAIL_DEFAULT << 2) /* Shifted mode DEFAULT for BURTC_IFS */

/* Bit fields for BURTC IFC */

#define _BURTC_IFC_RESETVALUE                 0x00000000UL                       /* Default value for BURTC_IFC */
#define _BURTC_IFC_MASK                       0x00000007UL                       /* Mask for BURTC_IFC */

#define BURTC_IFC_OF                          (0x1UL << 0)                       /* Clear Overflow Interrupt Flag */
#define _BURTC_IFC_OF_SHIFT                   0                                  /* Shift value for BURTC_OF */
#define _BURTC_IFC_OF_MASK                    0x1UL                              /* Bit mask for BURTC_OF */
#define _BURTC_IFC_OF_DEFAULT                 0x00000000UL                       /* Mode DEFAULT for BURTC_IFC */
#define BURTC_IFC_OF_DEFAULT                  (_BURTC_IFC_OF_DEFAULT << 0)       /* Shifted mode DEFAULT for BURTC_IFC */
#define BURTC_IFC_COMP0                       (0x1UL << 1)                       /* Clear compare match Interrupt Flag */
#define _BURTC_IFC_COMP0_SHIFT                1                                  /* Shift value for BURTC_COMP0 */
#define _BURTC_IFC_COMP0_MASK                 0x2UL                              /* Bit mask for BURTC_COMP0 */
#define _BURTC_IFC_COMP0_DEFAULT              0x00000000UL                       /* Mode DEFAULT for BURTC_IFC */
#define BURTC_IFC_COMP0_DEFAULT               (_BURTC_IFC_COMP0_DEFAULT << 1)    /* Shifted mode DEFAULT for BURTC_IFC */
#define BURTC_IFC_LFXOFAIL                    (0x1UL << 2)                       /* Clear LFXO failure Interrupt Flag */
#define _BURTC_IFC_LFXOFAIL_SHIFT             2                                  /* Shift value for BURTC_LFXOFAIL */
#define _BURTC_IFC_LFXOFAIL_MASK              0x4UL                              /* Bit mask for BURTC_LFXOFAIL */
#define _BURTC_IFC_LFXOFAIL_DEFAULT           0x00000000UL                       /* Mode DEFAULT for BURTC_IFC */
#define BURTC_IFC_LFXOFAIL_DEFAULT            (_BURTC_IFC_LFXOFAIL_DEFAULT << 2) /* Shifted mode DEFAULT for BURTC_IFC */

/* Bit fields for BURTC IEN */

#define _BURTC_IEN_RESETVALUE                 0x00000000UL                       /* Default value for BURTC_IEN */
#define _BURTC_IEN_MASK                       0x00000007UL                       /* Mask for BURTC_IEN */

#define BURTC_IEN_OF                          (0x1UL << 0)                       /* Overflow Interrupt Enable */
#define _BURTC_IEN_OF_SHIFT                   0                                  /* Shift value for BURTC_OF */
#define _BURTC_IEN_OF_MASK                    0x1UL                              /* Bit mask for BURTC_OF */
#define _BURTC_IEN_OF_DEFAULT                 0x00000000UL                       /* Mode DEFAULT for BURTC_IEN */
#define BURTC_IEN_OF_DEFAULT                  (_BURTC_IEN_OF_DEFAULT << 0)       /* Shifted mode DEFAULT for BURTC_IEN */
#define BURTC_IEN_COMP0                       (0x1UL << 1)                       /* Compare match Interrupt Enable */
#define _BURTC_IEN_COMP0_SHIFT                1                                  /* Shift value for BURTC_COMP0 */
#define _BURTC_IEN_COMP0_MASK                 0x2UL                              /* Bit mask for BURTC_COMP0 */
#define _BURTC_IEN_COMP0_DEFAULT              0x00000000UL                       /* Mode DEFAULT for BURTC_IEN */
#define BURTC_IEN_COMP0_DEFAULT               (_BURTC_IEN_COMP0_DEFAULT << 1)    /* Shifted mode DEFAULT for BURTC_IEN */
#define BURTC_IEN_LFXOFAIL                    (0x1UL << 2)                       /* LFXO failure Interrupt Enable */
#define _BURTC_IEN_LFXOFAIL_SHIFT             2                                  /* Shift value for BURTC_LFXOFAIL */
#define _BURTC_IEN_LFXOFAIL_MASK              0x4UL                              /* Bit mask for BURTC_LFXOFAIL */
#define _BURTC_IEN_LFXOFAIL_DEFAULT           0x00000000UL                       /* Mode DEFAULT for BURTC_IEN */
#define BURTC_IEN_LFXOFAIL_DEFAULT            (_BURTC_IEN_LFXOFAIL_DEFAULT << 2) /* Shifted mode DEFAULT for BURTC_IEN */

/* Bit fields for BURTC FREEZE */

#define _BURTC_FREEZE_RESETVALUE              0x00000000UL                           /* Default value for BURTC_FREEZE */
#define _BURTC_FREEZE_MASK                    0x00000001UL                           /* Mask for BURTC_FREEZE */

#define BURTC_FREEZE_REGFREEZE                (0x1UL << 0)                           /* Register Update Freeze */
#define _BURTC_FREEZE_REGFREEZE_SHIFT         0                                      /* Shift value for BURTC_REGFREEZE */
#define _BURTC_FREEZE_REGFREEZE_MASK          0x1UL                                  /* Bit mask for BURTC_REGFREEZE */
#define _BURTC_FREEZE_REGFREEZE_DEFAULT       0x00000000UL                           /* Mode DEFAULT for BURTC_FREEZE */
#define _BURTC_FREEZE_REGFREEZE_UPDATE        0x00000000UL                           /* Mode UPDATE for BURTC_FREEZE */
#define _BURTC_FREEZE_REGFREEZE_FREEZE        0x00000001UL                           /* Mode FREEZE for BURTC_FREEZE */
#define BURTC_FREEZE_REGFREEZE_DEFAULT        (_BURTC_FREEZE_REGFREEZE_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_FREEZE */
#define BURTC_FREEZE_REGFREEZE_UPDATE         (_BURTC_FREEZE_REGFREEZE_UPDATE << 0)  /* Shifted mode UPDATE for BURTC_FREEZE */
#define BURTC_FREEZE_REGFREEZE_FREEZE         (_BURTC_FREEZE_REGFREEZE_FREEZE << 0)  /* Shifted mode FREEZE for BURTC_FREEZE */

/* Bit fields for BURTC SYNCBUSY */

#define _BURTC_SYNCBUSY_RESETVALUE            0x00000000UL                          /* Default value for BURTC_SYNCBUSY */
#define _BURTC_SYNCBUSY_MASK                  0x00000003UL                          /* Mask for BURTC_SYNCBUSY */

#define BURTC_SYNCBUSY_LPMODE                 (0x1UL << 0)                          /* LPMODE Register Busy */
#define _BURTC_SYNCBUSY_LPMODE_SHIFT          0                                     /* Shift value for BURTC_LPMODE */
#define _BURTC_SYNCBUSY_LPMODE_MASK           0x1UL                                 /* Bit mask for BURTC_LPMODE */
#define _BURTC_SYNCBUSY_LPMODE_DEFAULT        0x00000000UL                          /* Mode DEFAULT for BURTC_SYNCBUSY */
#define BURTC_SYNCBUSY_LPMODE_DEFAULT         (_BURTC_SYNCBUSY_LPMODE_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_SYNCBUSY */
#define BURTC_SYNCBUSY_COMP0                  (0x1UL << 1)                          /* COMP0 Register Busy */
#define _BURTC_SYNCBUSY_COMP0_SHIFT           1                                     /* Shift value for BURTC_COMP0 */
#define _BURTC_SYNCBUSY_COMP0_MASK            0x2UL                                 /* Bit mask for BURTC_COMP0 */
#define _BURTC_SYNCBUSY_COMP0_DEFAULT         0x00000000UL                          /* Mode DEFAULT for BURTC_SYNCBUSY */
#define BURTC_SYNCBUSY_COMP0_DEFAULT          (_BURTC_SYNCBUSY_COMP0_DEFAULT << 1)  /* Shifted mode DEFAULT for BURTC_SYNCBUSY */

/* Bit fields for BURTC RET_REG */

#define _BURTC_RET_REG_RESETVALUE             0x00000000UL                      /* Default value for BURTC_RET_REG */
#define _BURTC_RET_REG_MASK                   0xFFFFFFFFUL                      /* Mask for BURTC_RET_REG */

#define _BURTC_RET_REG_REG_SHIFT              0                                 /* Shift value for REG */
#define _BURTC_RET_REG_REG_MASK               0xFFFFFFFFUL                      /* Bit mask for REG */
#define _BURTC_RET_REG_REG_DEFAULT            0x00000000UL                      /* Mode DEFAULT for BURTC_RET_REG */
#define BURTC_RET_REG_REG_DEFAULT             (_BURTC_RET_REG_REG_DEFAULT << 0) /* Shifted mode DEFAULT for BURTC_RET_REG */

#endif /* __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_BURTC_H */
