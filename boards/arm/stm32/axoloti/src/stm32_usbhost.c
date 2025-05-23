/****************************************************************************
 * boards/arm/stm32/axoloti/src/stm32_usbhost.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <sched.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbhost.h>
#include <nuttx/usb/usbdev_trace.h>

#include "arm_internal.h"
#include "stm32.h"
#include "stm32_otghs.h"
#include "axoloti.h"

#ifdef CONFIG_STM32_OTGHS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_USBDEV) || defined(CONFIG_USBHOST)
#define HAVE_USB 1
#else
#warning "CONFIG_STM32_OTGHS is enabled but neither CONFIG_USBDEV nor CONFIG_USBHOST"
#undef HAVE_USB
#endif

#ifndef CONFIG_AXOLOTI_USBHOST_PRIO
#define CONFIG_AXOLOTI_USBHOST_PRIO 100
#endif

#ifndef CONFIG_AXOLOTI_USBHOST_STACKSIZE
#define CONFIG_AXOLOTI_USBHOST_STACKSIZE 1024
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static struct usbhost_connection_s *g_usbconn;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbhost_waiter
 *
 * Description:
 *   Wait for USB devices to be connected.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
static int usbhost_waiter(int argc, char *argv[])
{
  struct usbhost_hubport_s *hport;
  uinfo("Running\n");

  for (; ; )
    {
      /* Wait for the device to change state */

      DEBUGVERIFY(CONN_WAIT(g_usbconn, &hport));
      uinfo("%s\n", hport->connected ? "connected" : "disconnected");

      /* Did we just become connected? */

      if (hport->connected)
        {
          /* Yes.. enumerate the newly connected device */

         CONN_ENUMERATE(g_usbconn, hport);
        }
    }

  /* Keep the compiler from complaining */

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in initialization to setup
 *   USB-related GPIO pins for the Axoloti board.
 *
 ****************************************************************************/

void stm32_usbinitialize(void)
{
#ifdef CONFIG_STM32_OTGHS
  stm32_configgpio(GPIO_OTGHS_PWRON);
  stm32_configgpio(GPIO_OTGHS_OVER);
#endif
}

/****************************************************************************
 * Name: stm32_usbhost_vbusdrive
 *
 * Description:
 *   Enable/disable driving of VBUS 5V output. This function must be provided
 *   be each platform that implements the STM32 OTG HS host interface
 *
 * Input Parameters:
 *   iface  - For future growth to handle multiple USB host interface. Should
 *            be zero.
 *   enable - true: enable VBUS power; false: disable VBUS power
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
void stm32_usbhost_vbusdrive(int iface, bool enable)
{
  DEBUGASSERT(iface == 0);
  if (enable)
    {
      /* Enable the Power Switch by driving the enable pin low */

      stm32_gpiowrite(GPIO_OTGHS_PWRON, false);
    }
  else
    {
      /* Disable the Power Switch by driving the enable pin high */

      stm32_gpiowrite(GPIO_OTGHS_PWRON, true);
    }
}
#endif

/****************************************************************************
 * Name: stm32_setup_overcurrent
 *
 * Description:
 *   Setup to receive an interrupt-level callback if an overcurrent condition
 *   is detected.
 *
 * Input Parameters:
 *   handler - New overcurrent interrupt handler
 *   arg     - The argument provided for the interrupt handler
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_setup_overcurrent(xcpt_t handler, void *arg)
{
  return stm32_gpiosetevent(GPIO_OTGHS_OVER, true, true, true, handler, arg);
}
#endif

/****************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host
 *   functionality. This function will start a thread that will monitor for
 *   device connection/disconnection events.
 *
 ****************************************************************************/

#ifdef CONFIG_USBHOST
int stm32_usbhost_initialize(void)
{
  int ret;

  /* First, register all of the class drivers needed to support the drivers
   * that we care about:
   */

  uinfo("Register class drivers\n");

#ifdef CONFIG_USBHOST_MSC
  /* Register the USB mass storage class class */

  ret = usbhost_msc_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the mass storage class: %d\n", ret);
    }
#endif

#ifdef CONFIG_USBHOST_HIDKBD
  /* Initialize the HID keyboard class */

  ret = usbhost_kbdinit();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID keyboard class\n");
    }
#endif

#ifdef CONFIG_USBHOST_HIDMOUSE
  /* Initialize the HID mouse class */

  ret = usbhost_mouse_init();
  if (ret != OK)
    {
      uerr("ERROR: Failed to register the HID mouse class\n");
    }
#endif

  /* Then get an instance of the USB host interface */

  uinfo("Initialize USB host\n");
  g_usbconn = stm32_otghshost_initialize(0);
  if (g_usbconn)
    {
      /* Start a thread to handle device connection. */

      uinfo("Start usbhost_waiter\n");
      ret = kthread_create("usbhost", CONFIG_AXOLOTI_USBHOST_PRIO,
                           CONFIG_AXOLOTI_USBHOST_STACKSIZE,
                           usbhost_waiter, NULL);
      return ret < 0 ? -ENOEXEC : OK;
    }

  return -ENODEV;
}
#endif

#endif /* CONFIG_STM32_OTGHS */
