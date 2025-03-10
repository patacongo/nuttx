/****************************************************************************
 * arch/arm64/src/common/arm64_registerdump.c
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
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "sched/sched.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_FPU
#include "arm64_fpu.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint64_t *regs = dumpregs ? dumpregs : running_regs();

  _alert("stack = %p\n", regs);
  _alert("x0:   0x%-16"PRIx64"  x1:   0x%"PRIx64"\n",
    regs[REG_X0], regs[REG_X1]);
  _alert("x2:   0x%-16"PRIx64"  x3:   0x%"PRIx64"\n",
    regs[REG_X2], regs[REG_X3]);
  _alert("x4:   0x%-16"PRIx64"  x5:   0x%"PRIx64"\n",
    regs[REG_X4], regs[REG_X5]);
  _alert("x6:   0x%-16"PRIx64"  x7:   0x%"PRIx64"\n",
    regs[REG_X6], regs[REG_X7]);
  _alert("x8:   0x%-16"PRIx64"  x9:   0x%"PRIx64"\n",
    regs[REG_X8], regs[REG_X9]);
  _alert("x10:  0x%-16"PRIx64"  x11:  0x%"PRIx64"\n",
    regs[REG_X10], regs[REG_X11]);
  _alert("x12:  0x%-16"PRIx64"  x13:  0x%"PRIx64"\n",
    regs[REG_X12], regs[REG_X13]);
  _alert("x14:  0x%-16"PRIx64"  x15:  0x%"PRIx64"\n",
    regs[REG_X14], regs[REG_X15]);
  _alert("x16:  0x%-16"PRIx64"  x17:  0x%"PRIx64"\n",
    regs[REG_X16], regs[REG_X17]);
  _alert("x18:  0x%-16"PRIx64"  x19:  0x%"PRIx64"\n",
    regs[REG_X18], regs[REG_X19]);
  _alert("x20:  0x%-16"PRIx64"  x21:  0x%"PRIx64"\n",
    regs[REG_X20], regs[REG_X21]);
  _alert("x22:  0x%-16"PRIx64"  x23:  0x%"PRIx64"\n",
    regs[REG_X22], regs[REG_X23]);
  _alert("x24:  0x%-16"PRIx64"  x25:  0x%"PRIx64"\n",
    regs[REG_X24], regs[REG_X25]);
  _alert("x26:  0x%-16"PRIx64"  x27:  0x%"PRIx64"\n",
    regs[REG_X26], regs[REG_X27]);
  _alert("x28:  0x%-16"PRIx64"  x29:  0x%"PRIx64"\n",
    regs[REG_X28], regs[REG_X29]);
  _alert("x30:  0x%-16"PRIx64"\n", regs[REG_X30]);

  _alert("\n");
  _alert("STATUS Registers:\n");
  _alert("SPSR:      0x%-16"PRIx64"\n", regs[REG_SPSR]);
  _alert("ELR:       0x%-16"PRIx64"\n", regs[REG_ELR]);
  _alert("SP_EL0:    0x%-16"PRIx64"\n", regs[REG_SP_EL0]);
  _alert("SP_ELX:    0x%-16"PRIx64"\n", regs[REG_SP_ELX]);
  _alert("EXE_DEPTH: 0x%-16"PRIx64"\n", regs[REG_EXE_DEPTH]);
  _alert("SCTLR_EL1: 0x%-16"PRIx64"\n", regs[REG_SCTLR_EL1]);
}
