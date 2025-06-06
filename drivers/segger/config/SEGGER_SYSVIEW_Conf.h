/****************************************************************************
 * drivers/segger/config/SEGGER_SYSVIEW_Conf.h
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

#ifndef __DRIVERS_SEGGER_CONFIG_SEGGER_SYSVIEW_CONF_H
#define __DRIVERS_SEGGER_CONFIG_SEGGER_SYSVIEW_CONF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Function macro to retrieve the Id of the currently active interrupt.
 * Call user-supplied function SEGGER_SYSVIEW_X_GetInterruptId().
 */

#define SEGGER_SYSVIEW_GET_INTERRUPT_ID    note_sysview_get_interrupt_id

/* Function macro to retrieve a system timestamp for SYSVIEW events.
 * Call user-supplied function SEGGER_SYSVIEW_X_GetTimestamp().
 */

#define SEGGER_SYSVIEW_GET_TIMESTAMP       note_sysview_get_timestamp

#define SEGGER_SYSVIEW_TIMESTAMP_FREQ      note_sysview_timestamp_freq

/* The RTT channel that SystemView will use. */

#define SEGGER_SYSVIEW_RTT_CHANNEL         CONFIG_SEGGER_SYSVIEW_RTT_CHANNEL

/* Number of bytes that SystemView uses for the RTT buffer. */

#define SEGGER_SYSVIEW_RTT_BUFFER_SIZE     CONFIG_SEGGER_SYSVIEW_RTT_BUFFER_SIZE

/* Largest cache line size (in bytes) in the target system. */

#define SEGGER_SYSVIEW_CPU_CACHE_LINE_SIZE CONFIG_SEGGER_RTT_CPU_CACHE_LINE_SIZE

/****************************************************************************
 * Name: note_sysview_get_timestamp
 *
 * Description:
 *   Retrieve a system timestamp for SYSVIEW events.
 *
 ****************************************************************************/

#define note_sysview_get_timestamp() perf_gettime()
#define note_sysview_timestamp_freq() perf_getfreq()

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: note_sysview_get_interrupt_id
 *
 * Description:
 *   Retrieve the Id of the currently active interrupt.
 *
 ****************************************************************************/

unsigned int note_sysview_get_interrupt_id(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __DRIVERS_SEGGER_CONFIG_SEGGER_SYSVIEW_CONF_H */
