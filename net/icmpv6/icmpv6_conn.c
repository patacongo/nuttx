/****************************************************************************
 * net/icmpv6/icmpv6_conn.c
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "icmpv6/icmpv6.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_ICMPv6_SOCKET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_ICMPv6_MAX_CONNS
#  define CONFIG_NET_ICMPv6_MAX_CONNS 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The array containing all IPPROTO_ICMP socket connections */

NET_BUFPOOL_DECLARE(g_icmpv6_connections, sizeof(struct icmpv6_conn_s),
                    CONFIG_NET_ICMPv6_PREALLOC_CONNS,
                    CONFIG_NET_ICMPv6_ALLOC_CONNS,
                    CONFIG_NET_ICMPv6_MAX_CONNS);
static mutex_t g_free_lock = NXMUTEX_INITIALIZER;

/* A list of all allocated IPPROTO_ICMP socket connections */

static dq_queue_t g_active_icmpv6_connections;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_alloc
 *
 * Description:
 *   Allocate a new, uninitialized IPPROTO_ICMP socket connection structure.
 *   This is normally something done by the implementation of the socket()
 *   interface.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_alloc(void)
{
  FAR struct icmpv6_conn_s *conn = NULL;
  int ret;

  /* The free list is protected by a mutex. */

  ret = nxmutex_lock(&g_free_lock);
  if (ret >= 0)
    {
      conn = NET_BUFPOOL_TRYALLOC(g_icmpv6_connections);
      if (conn != NULL)
        {
          /* Enqueue the connection into the active list */

          dq_addlast(&conn->sconn.node, &g_active_icmpv6_connections);
        }

      nxmutex_unlock(&g_free_lock);
    }

  return conn;
}

/****************************************************************************
 * Name: icmpv6_free
 *
 * Description:
 *   Free a IPPROTO_ICMP socket connection structure that is no longer in
 *   use.  This should be done by the implementation of close().
 *
 ****************************************************************************/

void icmpv6_free(FAR struct icmpv6_conn_s *conn)
{
  /* The free list is protected by a mutex. */

  DEBUGASSERT(conn->crefs == 0);

  /* Take the mutex (perhaps waiting) */

  nxmutex_lock(&g_free_lock);

  /* Remove the connection from the active list */

  dq_rem(&conn->sconn.node, &g_active_icmpv6_connections);

  /* Free the connection. */

  NET_BUFPOOL_FREE(g_icmpv6_connections, conn);

  nxmutex_unlock(&g_free_lock);
}

/****************************************************************************
 * Name: icmpv6_active()
 *
 * Description:
 *   Find a connection structure that is the appropriate connection to be
 *   used with the provided ECHO request ID.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_active(uint16_t id)
{
  FAR struct icmpv6_conn_s *conn =
    (FAR struct icmpv6_conn_s *)g_active_icmpv6_connections.head;

  while (conn != NULL)
    {
      /* FIXME lmac in conn should have been set by icmpv6_bind() */

      if (id == conn->id)
        {
          /* Matching connection found.. return a reference to it */

          break;
        }

      /* Look at the next active connection */

      conn = (FAR struct icmpv6_conn_s *)conn->sconn.node.flink;
    }

  return conn;
}

/****************************************************************************
 * Name: icmpv6_nextconn
 *
 * Description:
 *   Traverse the list of allocated packet connections
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

FAR struct icmpv6_conn_s *icmpv6_nextconn(FAR struct icmpv6_conn_s *conn)
{
  if (conn == NULL)
    {
      return (FAR struct icmpv6_conn_s *)g_active_icmpv6_connections.head;
    }
  else
    {
      return (FAR struct icmpv6_conn_s *)conn->sconn.node.flink;
    }
}

/****************************************************************************
 * Name: icmpv6_foreach
 *
 * Description:
 *   Enumerate each ICMPv6 connection structure. This function will terminate
 *   when either (1) all connection have been enumerated or (2) when a
 *   callback returns any non-zero value.
 *
 * Assumptions:
 *   This function is called from network logic at with the network locked.
 *
 ****************************************************************************/

int icmpv6_foreach(icmpv6_callback_t callback, FAR void *arg)
{
  FAR struct icmpv6_conn_s *conn;
  int ret = 0;

  if (callback != NULL)
    {
      for (conn = icmpv6_nextconn(NULL); conn != NULL;
           conn = icmpv6_nextconn(conn))
        {
          ret = callback(conn, arg);
          if (ret != 0)
            {
              break;
            }
        }
    }

  return ret;
}
#endif /* CONFIG_NET_ICMP */
