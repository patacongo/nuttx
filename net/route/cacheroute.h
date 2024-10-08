/****************************************************************************
 * net/route/cacheroute.h
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

#ifndef __NET_ROUTE_CACHEROUTE_H
#define __NET_ROUTE_CACHEROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_CACHEROUTE) || defined(CONFIG_ROUTE_IPv6_CACHEROUTE)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_init_cacheroute
 *
 * Description:
 *   Initialize the in-memory, routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in initialization so that no special protection is needed.
 *
 ****************************************************************************/

void net_init_cacheroute(void);

/****************************************************************************
 * Name: net_addcache_ipv4 and net_addcache_ipv6
 *
 * Description:
 *   Add one route to the routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
int net_addcache_ipv4(FAR struct net_route_ipv4_s *route);
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
int net_addcache_ipv6(FAR struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Name: net_foreachcache_ipv4/net_foreachcache_ipv6
 *
 * Description:
 *   Traverse the routing table cache
 *
 * Input Parameters:
 *   handler - Will be called for each route in the routing table cache.
 *   arg     - An arbitrary value that will be passed to the handler.
 *
 * Returned Value:
 *   Zero (OK) returned if the entire table was searched.  A negated errno
 *   value will be returned in the event of a failure.  Handlers may also
 *   terminate the search early with any non-zero value.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
int net_foreachcache_ipv4(route_handler_ipv4_t handler, FAR void *arg);
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
int net_foreachcache_ipv6(route_handler_ipv6_t handler, FAR void *arg);
#endif

/****************************************************************************
 * Name: net_flushcache_ipv4 and net_flushcache_ipv6
 *
 * Description:
 *   Flush the content of the routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
void net_flushcache_ipv4(void);
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
void net_flushcache_ipv6(void);
#endif

#endif /* CONFIG_ROUTE_IPv4_CACHEROUTE || CONFIG_ROUTE_IPv6_CACHEROUTE */
#endif /* __NET_ROUTE_CACHEROUTE_H */
