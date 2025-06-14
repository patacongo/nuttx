/****************************************************************************
 * sched/sched/sched_cpuload.c
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

#include <errno.h>
#include <assert.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/wdog.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Are we using the system timer, or an external clock?  Get the rate
 * of the sampling in ticks per second for the selected timer.
 */

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
#  ifndef CONFIG_SCHED_CPULOAD_TICKSPERSEC
#    error CONFIG_SCHED_CPULOAD_TICKSPERSEC is not defined
#  endif
#  define CPULOAD_TICKSPERSEC CONFIG_SCHED_CPULOAD_TICKSPERSEC
#else
#  define CPULOAD_TICKSPERSEC CLOCKS_PER_SEC
#endif

/* When g_cpuload_total exceeds the following time constant, the load and
 * the counts will be scaled back by two.  In the CONFIG_SMP, g_cpuload_total
 * will be incremented multiple times per tick.
 */

#define CPULOAD_TIMECONSTANT \
     (CONFIG_SMP_NCPUS * \
      CONFIG_SCHED_CPULOAD_TIMECONSTANT * \
      CPULOAD_TICKSPERSEC)

/* The sampling period in system timer ticks */

#define CPULOAD_SAMPLING_PERIOD \
     (TICK_PER_SEC / CONFIG_SCHED_CPULOAD_TICKSPERSEC)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the total number of clock tick counts.  Essentially the
 * 'denominator' for all CPU load calculations.
 *
 * For a single CPU, this value is increment once per sample interval.  So,
 * for example, if nothing is running but the IDLE thread, that IDLE thread
 * will get 100% of the load.
 *
 * But for the case of multiple CPUs (with CONFIG_SMP=y), this value is
 * incremented for each CPU on each sample interval. So, as an example, if
 * there are four CPUs and is nothing is running but the IDLE threads, then
 * each would have a load of 25% of the total.
 */

volatile clock_t g_cpuload_total;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_CPULOAD_SYSCLK
static struct wdog_s g_cpuload_wdog;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cpuload_callback
 *
 * Description:
 *   This is the callback function that will be invoked when the watchdog
 *   timer expires.
 *
 * Input Parameters:
 *   argc - the argument passed with the timer when the timer was started.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CPULOAD_SYSCLK
static void cpuload_callback(wdparm_t arg)
{
  FAR struct wdog_s *wdog = (FAR struct wdog_s *)arg;
  nxsched_process_cpuload_ticks(CPULOAD_SAMPLING_PERIOD);
  wd_start_next(wdog, CPULOAD_SAMPLING_PERIOD, cpuload_callback, arg);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_process_taskload_ticks
 *
 * Description:
 *   Collect data that can be used for task load measurements.
 *
 * Input Parameters:
 *   tcb   - The task that we are performing the load operations on.
 *   ticks - The ticks that we process in this cpuload.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxsched_process_taskload_ticks(FAR struct tcb_s *tcb, clock_t ticks)
{
  tcb->ticks += ticks;
  g_cpuload_total += ticks;

  if (g_cpuload_total > CPULOAD_TIMECONSTANT)
    {
      uint32_t total = 0;
      int i;

      /* Divide the tick count for every task by two and recalculate the
       * total.
       */

      for (i = 0; i < g_npidhash; i++)
        {
          if (g_pidhash[i])
            {
              g_pidhash[i]->ticks >>= 1;
              total += g_pidhash[i]->ticks;
            }
        }

      /* Save the new total. */

      g_cpuload_total = total;
    }
}

/****************************************************************************
 * Name: nxsched_process_cpuload_ticks
 *
 * Description:
 *   Collect data that can be used for CPU load measurements.  When
 *   CONFIG_SCHED_CPULOAD_EXTCLK is defined, this is an exported interface,
 *   use the the external clock logic.  Otherwise, it is an OS Internal
 *   interface.
 *
 * Input Parameters:
 *   ticks - The ticks that we increment in this cpuload
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   This function is called from a timer interrupt handler with all
 *   interrupts disabled.
 *
 ****************************************************************************/

void nxsched_process_cpuload_ticks(clock_t ticks)
{
  int i;

  /* Perform scheduler operations on all CPUs. */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      FAR struct tcb_s *rtcb = current_task(i);
      nxsched_process_taskload_ticks(rtcb, ticks);
    }
}

/****************************************************************************
 * Name:  clock_cpuload
 *
 * Description:
 *   Return load measurement data for the select PID.
 *
 * Input Parameters:
 *   pid - The task ID of the thread of interest.  pid == 0 is the IDLE
 *         thread.
 *   cpuload - The location to return the CPU load
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.  The only reason
 *   that this function can fail is if 'pid' no longer refers to a valid
 *   thread.
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_cpuload(int pid, FAR struct cpuload_s *cpuload)
{
  irqstate_t flags;
  int hash_index;
  int ret = -ESRCH;

  DEBUGASSERT(cpuload);

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
  /* Update critmon in case of the target thread busyloop */

  nxsched_update_critmon(nxsched_get_tcb(pid));
#endif

  /* Momentarily disable interrupts.  We need (1) the task to stay valid
   * while we are doing these operations and (2) the tick counts to be
   * synchronized when read.
   */

  flags = enter_critical_section();
  hash_index = PIDHASH(pid);

  /* Make sure that the entry is valid (TCB field is not NULL) and matches
   * the requested PID.  The first check is needed if the thread has exited.
   * The second check is needed for the case where the task associated with
   * the requested PID has exited and the slot has been taken by another
   * thread with a different PID.
   *
   * NOTE also that CPU load measurement data is retained in the g_pidhash
   * table vs. in the TCB which would seem to be the more logic place.  It
   * is place in the hash table, instead, to facilitate CPU load adjustments
   * on all threads during timer interrupt handling. nxsched_foreach() could
   * do this too, but this would require a little more overhead.
   */

  if (g_pidhash[hash_index] && g_pidhash[hash_index]->pid == pid)
    {
      cpuload->total  = g_cpuload_total;
      cpuload->active = g_pidhash[hash_index]->ticks;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: cpuload_init
 *
 * Description:
 *   Initialize the CPU load measurement logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CPULOAD_SYSCLK
void cpuload_init(void)
{
  wd_start(&g_cpuload_wdog, CPULOAD_SAMPLING_PERIOD, cpuload_callback,
           (wdparm_t)&g_cpuload_wdog);
}
#endif
