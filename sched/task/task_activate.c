/****************************************************************************
 * sched/task/task_activate.c
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

#include <sched.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_activate
 *
 * Description:
 *   This function activates tasks initialized by nxtask_setup_scheduler().
 *   Without activation, a task is ineligible for execution by the
 *   scheduler.
 *
 * Input Parameters:
 *   tcb - The TCB for the task for the task (same as the nxtask_init
 *         argument).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxtask_activate(FAR struct tcb_s *tcb)
{
  irqstate_t flags = enter_critical_section();
  FAR struct tcb_s *rtcb = this_task();

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Check if this is really a re-start */

  if (tcb->task_state != TSTATE_TASK_INACTIVE)
    {
      /* Inform the instrumentation layer that the task
       * has stopped
       */

      sched_note_stop(tcb);
    }

  /* Inform the instrumentation layer that the task
   * has started
   */

  sched_note_start(tcb);
#endif

  /* Remove the task from waiting list */

  nxsched_remove_blocked(tcb);

  sinfo("%s pid=%d,TCB=%p\n", get_task_name(tcb),
        tcb->pid, tcb);

  /* Add the task to ready-to-run task list, and
   * perform the context switch if one is needed
   */

  if (nxsched_add_readytorun(tcb))
    {
      up_switch_context(tcb, rtcb);
    }

  leave_critical_section(flags);
}
