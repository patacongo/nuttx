/****************************************************************************
 * net/arp/arp.h
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

#ifndef __NET_ARP_ARP_H
#define __NET_ARP_ARP_H

/* The Address Resolution Protocol ARP is used for mapping between IP
 * addresses and link level addresses such as the Ethernet MAC
 * addresses. ARP uses broadcast queries to ask for the link level
 * address of a known IP address and the host which is configured with
 * the IP address for which the query was meant, will respond with its
 * link level address.
 *
 * Note: This ARP implementation only supports Ethernet.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <netinet/arp.h>
#include <netinet/in.h>

#include <nuttx/net/netdev.h>
#include <nuttx/semaphore.h>

#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG_FEATURES
#  undef CONFIG_NET_ARP_DUMP
#endif

#ifndef CONFIG_ARP_SEND_MAXTRIES
#  define CONFIG_ARP_SEND_MAXTRIES 5
#endif

#ifndef CONFIG_ARP_SEND_DELAYMSEC
#  define CONFIG_ARP_SEND_DELAYMSEC 20
#endif

/* ARP Definitions **********************************************************/

#define ARP_REQUEST    1
#define ARP_REPLY      2

#define ARP_HWTYPE_ETH 1

#define RASIZE         4  /* Size of ROUTER ALERT */

/* Allocate a new ARP data callback */

#define arp_callback_alloc(dev)   devif_callback_alloc(dev, \
                                                       &(dev)->d_conncb, \
                                                       &(dev)->d_conncb_tail)
#define arp_callback_free(dev,cb) devif_dev_callback_free(dev, cb)

/* This is a helper pointer for accessing the contents of the IP header */

#define ARPBUF   ((FAR struct arp_hdr_s *)IPBUF(0))
#define ARPIPBUF ((FAR struct arp_iphdr_s *)IPBUF(0))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ARP header -- Size 28 bytes */

struct arp_hdr_s
{
  uint16_t ah_hwtype;        /* 16-bit Hardware type (Ethernet=0x001) */
  uint16_t ah_protocol;      /* 16-bit Protocol type (IP=0x0800) */
  uint8_t  ah_hwlen;         /*  8-bit Hardware address size (6) */
  uint8_t  ah_protolen;      /*  8-bit Protocol address size (4) */
  uint16_t ah_opcode;        /* 16-bit Operation */
  uint8_t  ah_shwaddr[6];    /* 48-bit Sender hardware address */
  uint16_t ah_sipaddr[2];    /* 32-bit Sender IP address */
  uint8_t  ah_dhwaddr[6];    /* 48-bit Target hardware address */
  uint16_t ah_dipaddr[2];    /* 32-bit Target IP address */
};

/* IP header -- Size 20 or 24 bytes */

struct arp_iphdr_s
{
  uint8_t  eh_vhl;           /*  8-bit Version (4) and header length (5 or 6) */
  uint8_t  eh_tos;           /*  8-bit Type of service (e.g., 6=TCP) */
  uint8_t  eh_len[2];        /* 16-bit Total length */
  uint8_t  eh_ipid[2];       /* 16-bit Identification */
  uint8_t  eh_ipoffset[2];   /* 16-bit IP flags + fragment offset */
  uint8_t  eh_ttl;           /*  8-bit Time to Live */
  uint8_t  eh_proto;         /*  8-bit Protocol */
  uint16_t eh_ipchksum;      /* 16-bit Header checksum */
  uint16_t eh_srcipaddr[2];  /* 32-bit Source IP address */
  uint16_t eh_destipaddr[2]; /* 32-bit Destination IP address */
  uint16_t eh_ipoption[2];   /* (optional) */
};

#ifdef CONFIG_NET_ARP_SEND
/* This structure holds the state of the send operation until it can be
 * operated upon from the network driver poll.
 */

typedef CODE void (*arp_send_finish_cb_t)(FAR struct net_driver_s *dev,
                                          int result);
struct arp_send_s
{
  FAR struct devif_callback_s *snd_cb; /* Reference to callback instance */
  FAR arp_send_finish_cb_t finish_cb;  /* Reference to send finish callback */
  sem_t     snd_sem;                   /* Used to wake up the waiting thread */
  uint8_t   snd_retries;               /* Retry count */
  volatile bool snd_sent;              /* True: if request sent */
  uint8_t   snd_ifname[IFNAMSIZ];      /* Interface name */
  int16_t   snd_result;                /* The result of the send operation */
  in_addr_t snd_ipaddr;                /* The IP address to be queried */
};
#endif

#ifdef CONFIG_NET_ARP_SEND
/* Used to notify a thread waiting for a particular ARP response */

struct arp_notify_s
{
  FAR struct arp_notify_s *nt_flink;   /* Supports singly linked list */
  in_addr_t nt_ipaddr;                 /* Waited for IP address in the mapping */
  sem_t     nt_sem;                    /* Will wake up the waiter */
  int       nt_result;                 /* The result of the wait */
};
#endif

/* One entry in the ARP table (volatile!) */

struct arp_entry_s
{
  in_addr_t                at_ipaddr;   /* IP address */
  struct ether_addr        at_ethaddr;  /* Hardware address */
  clock_t                  at_time;     /* Time of last usage */
  FAR struct net_driver_s *at_dev;      /* The device driver structure */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_ARP
/****************************************************************************
 * Name: arp_format
 *
 * Description:
 *   Format an ARP packet.
 *
 * Input Parameters:
 *   dev    - Device structure
 *   ipaddr - Target IP address (32-bit)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

struct net_driver_s; /* Forward reference */
void arp_format(FAR struct net_driver_s *dev, in_addr_t ipaddr);

/****************************************************************************
 * Name: arp_ipin
 *
 * Description:
 *   The arp_ipin() function should be called by Ethernet device drivers
 *   whenever an IP packet arrives from the network.  The function will
 *   check if the address is in the ARP cache, and if so the ARP cache entry
 *   will be refreshed.
 *   If no ARP cache entry was found, a new one is created.
 *
 *   This function expects that an IP packet with an Ethernet header is
 *   present in the d_buf buffer and that the length of the packet is in the
 *   d_len field.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_IPIN
void arp_ipin(FAR struct net_driver_s *dev);
#else
#  define arp_ipin(dev)
#endif

/****************************************************************************
 * Name: arp_out
 *
 * Description:
 *   This function should be called before sending out an IPv4 packet. The
 *   function checks the destination IPv4 address of the IPv4 packet to see
 *   what Ethernet MAC address that should be used as a destination MAC
 *   address on the Ethernet.
 *
 *   If the destination IPv4 address is in the local network (determined
 *   by logical ANDing of netmask and our IPv4 address), the function
 *   checks the ARP cache to see if an entry for the destination IPv4
 *   address is found.  If so, an Ethernet header is prepended at the
 *   beginning of the packet and the function returns.
 *
 *   If no ARP cache entry is found for the destination IIPv4P address, the
 *   packet in the d_buf is replaced by an ARP request packet for the
 *   IPv4 address. The IPv4 packet is dropped and it is assumed that the
 *   higher level protocols (e.g., TCP) eventually will retransmit the
 *   dropped packet.
 *
 *   Upon return in either the case, a packet to be sent is present in the
 *   d_buf buffer and the d_len field holds the length of the Ethernet
 *   frame that should be transmitted.
 *
 ****************************************************************************/

void arp_out(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: arp_send
 *
 * Description:
 *   The arp_send() call may be to send an ARP request to resolve an IPv4
 *   address.  This function first checks if the IPv4 address is already in
 *   the ARP table.  If so, then it returns success immediately.
 *
 *   If the requested IPv4 address in not in the ARP table, then this
 *   function will send an ARP request, delay, then check if the IP address
 *   is now in the ARP table.  It will repeat this sequence until either (1)
 *   the IP address mapping is now in the ARP table, or (2) a configurable
 *   number of timeouts occur without receiving the ARP replay.
 *
 * Input Parameters:
 *   ipaddr   The IP address to be queried.
 *
 * Returned Value:
 *   Zero (OK) is returned on success and the IP address mapping can now be
 *   found in the ARP table.  On error a negated errno value is returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_send(in_addr_t ipaddr);
#else
#  define arp_send(i) (0)
#endif

/****************************************************************************
 * Name: arp_send_async
 *
 * Description:
 *   The arp_send_async() call may be to send an ARP request asyncly to
 *   resolve an IPv4 address.
 *
 * Input Parameters:
 *   ipaddr   The IP address to be queried.
 *   cb       The callback when ARP send is finished, should not be NULL.
 *
 * Returned Value:
 *   Zero (OK) is returned on success the arp been sent to the driver.
 *   On error a negated errno value is returned:
 *
 *     -ETIMEDOUT:    The number or retry counts has been exceed.
 *     -EHOSTUNREACH: Could not find a route to the host
 *
 * Assumptions:
 *   This function is called from the normal tasking context.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_send_async(in_addr_t ipaddr, arp_send_finish_cb_t cb);
#endif

/****************************************************************************
 * Name: arp_poll
 *
 * Description:
 *   Poll all pending transfer for ARP requests to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll().  The network must be locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_poll(FAR struct net_driver_s *dev, devif_poll_callback_t callback);
#else
#  define arp_poll(d,c) (0)
#endif

/****************************************************************************
 * Name: arp_wait_setup
 *
 * Description:
 *   Called BEFORE an ARP request is sent.  This function sets up the ARP
 *   response timeout before the ARP request is sent so that there is
 *   no race condition when arp_wait() is called.
 *
 * Assumptions:
 *   This function is called from ARP send and executes in the normal
 *   tasking environment.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
void arp_wait_setup(in_addr_t ipaddr, FAR struct arp_notify_s *notify);
#else
#  define arp_wait_setup(i,n)
#endif

/****************************************************************************
 * Name: arp_wait_cancel
 *
 * Description:
 *   Cancel any wait set after arp_wait_setup is called but before arm_wait()
 *   is called (arp_wait() will automatically cancel the wait).
 *
 * Assumptions:
 *   This function may execute in the interrupt context when called from
 *   arp_wait().
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_wait_cancel(FAR struct arp_notify_s *notify);
#else
#  define arp_wait_cancel(n) (0)
#endif

/****************************************************************************
 * Name: arp_wait
 *
 * Description:
 *   Called each time that a ARP request is sent.  This function will sleep
 *   until either: (1) the matching ARP response is received, or (2) a
 *   timeout occurs.
 *
 * Assumptions:
 *   This function is called from ARP send and must execute with the network
 *   un-locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
int arp_wait(FAR struct arp_notify_s *notify, unsigned int timeout);
#else
#  define arp_wait(n,t) (0)
#endif

/****************************************************************************
 * Name: arp_notify
 *
 * Description:
 *   Called each time that a ARP response is received in order to wake-up
 *   any threads that may be waiting for this particular ARP response.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   arp_input() and will execute with the network locked.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_SEND
void arp_notify(in_addr_t ipaddr);
#else
#  define arp_notify(i)
#endif

/****************************************************************************
 * Name: arp_find
 *
 * Description:
 *   Find the ARP entry corresponding to this IP address which may or may
 *   not be in the ARP table (it may, instead, be a local network device).
 *
 * Input Parameters:
 *   ipaddr  - Refers to an IP address in network order
 *   ethaddr - Location to return the corresponding Ethernet MAN address.
 *             This address may be NULL.  In that case, this function may be
 *             used simply to determine if the Ethernet MAC address is
 *             available.
 *   dev     - Device structure
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table.
 *
 ****************************************************************************/

struct ether_addr;  /* Forward reference */
int arp_find(in_addr_t ipaddr, FAR uint8_t *ethaddr,
             FAR struct net_driver_s *dev, bool check_expiry);

/****************************************************************************
 * Name: arp_delete
 *
 * Description:
 *   Remove an IP association from the ARP table
 *
 * Input Parameters:
 *   ipaddr - Refers to an IP address in network order
 *   dev    - Device structure
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table.
 *
 ****************************************************************************/

int arp_delete(in_addr_t ipaddr, FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: arp_cleanup
 *
 * Description:
 *   Clear the ARP table on the network device
 *
 * Input Parameters:
 *   dev  - The device driver structure
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table.
 *
 ****************************************************************************/

void arp_cleanup(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: arp_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input Parameters:
 *   dev     - The device driver structure
 *   ipaddr  - The IP address as an inaddr_t
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Returned Value:
 *   Zero (OK) if the ARP table entry was successfully modified.  A negated
 *   errno value is returned on any error.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table
 *
 ****************************************************************************/

int arp_update(FAR struct net_driver_s *dev, in_addr_t ipaddr,
               FAR const uint8_t *ethaddr);

/****************************************************************************
 * Name: arp_hdr_update
 *
 * Description:
 *   Add the IP/HW address mapping to the ARP table -OR- change the IP
 *   address of an existing association.
 *
 * Input Parameters:
 *   dev     - The device driver structure
 *   pipaddr - Refers to an IP address uint16_t[2] in network order
 *   ethaddr - Refers to a HW address uint8_t[IFHWADDRLEN]
 *
 * Returned Value:
 *   Zero (OK) if the ARP table entry was successfully modified.  A negated
 *   errno value is returned on any error.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table
 *
 ****************************************************************************/

void arp_hdr_update(FAR struct net_driver_s *dev, FAR uint16_t *pipaddr,
                    FAR const uint8_t *ethaddr);

/****************************************************************************
 * Name: arp_snapshot
 *
 * Description:
 *   Take a snapshot of the current state of the ARP table.
 *
 * Input Parameters:
 *   snapshot  - Location to return the ARP table copy
 *   nentries  - The size of the user provided 'dest' in entries, each of
 *               size sizeof(struct arp_entry_s)
 *
 * Returned Value:
 *   On success, the number of entries actually copied is returned.  Unused
 *   entries are not returned.
 *
 * Assumptions
 *   The network is locked to assure exclusive access to the ARP table
 *
 ****************************************************************************/

#ifdef CONFIG_NETLINK_ROUTE
unsigned int arp_snapshot(FAR struct arpreq *snapshot,
                          unsigned int nentries);
#else
#  define arp_snapshot(s,n) (0)
#endif

/****************************************************************************
 * Name: arp_dump
 *
 * Description:
 *   Dump the contents of an ARP packet to the SYSLOG device
 *
 * Input Parameters:
 *   arp - A reference to the ARP header to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_DUMP
void arp_dump(FAR struct arp_hdr_s *arp);
#else
#  define arp_dump(arp)
#endif

#ifdef CONFIG_NET_ARP_ACD

/****************************************************************************
 * Name: arp_acd_update
 *
 * Description:
 *   interface of ARP Address Conflict Detection monitor
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void arp_acd_update(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: arp_acd_set_addr
 *
 * Description:
 *   setting address interface of ARP Address Conflict Detection
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void arp_acd_set_addr(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: arp_acd_setup
 *
 * Description:
 *   set up interface of ARP Address Conflict Detection
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *
 * Returned Value:
 *   none
 *
 ****************************************************************************/

void arp_acd_setup(FAR struct net_driver_s *dev);

#endif /* CONFIG_NET_ARP_ACD */

#else /* CONFIG_NET_ARP */

/* If ARP is disabled, stub out all ARP interfaces */

#  define arp_format(d,i);
#  define arp_ipin(dev)
#  define arp_out(dev)
#  define arp_send(i) (0)
#  define arp_poll(d,c) (0)
#  define arp_wait_setup(i,n)
#  define arp_wait_cancel(n) (0)
#  define arp_wait(n,t) (0)
#  define arp_notify(i)
#  define arp_find(i,e,d,u) (-ENOSYS)
#  define arp_delete(i,d) (-ENOSYS)
#  define arp_cleanup(d)
#  define arp_update(d,i,m);
#  define arp_hdr_update(d,i,m);
#  define arp_snapshot(s,n) (0)
#  define arp_dump(arp)

#endif /* CONFIG_NET_ARP */
#endif /* __NET_ARP_ARP_H */
