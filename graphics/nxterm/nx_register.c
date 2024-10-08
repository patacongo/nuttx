/****************************************************************************
 * graphics/nxterm/nx_register.c
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

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxterm.h>

#include "nxterm.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int nxterm_fill(FAR struct nxterm_state_s *priv,
                       FAR const struct nxgl_rect_s *rect,
                       nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]);
#ifndef CONFIG_NX_WRITEONLY
static int nxterm_move(FAR struct nxterm_state_s *priv,
                       FAR const struct nxgl_rect_s *rect,
                       FAR const struct nxgl_point_s *offset);
#endif
static int nxterm_bitmap(FAR struct nxterm_state_s *priv,
                         FAR const struct nxgl_rect_s *dest,
                         FAR const void *src[CONFIG_NX_NPLANES],
                         FAR const struct nxgl_point_s *origin,
                         unsigned int stride);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct nxterm_operations_s g_nxops =
{
  nxterm_fill,
#ifndef CONFIG_NX_WRITEONLY
  nxterm_move,
#endif
  nxterm_bitmap
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxterm_fill
 *
 * Description:
 *  Fill the specified rectangle in the window with the specified color
 *
 * Input Parameters:
 *   priv   - The driver state structure.
 *   rect  - The location to be filled
 *   color - The color to use in the fill
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

static int nxterm_fill(FAR struct nxterm_state_s *priv,
                       FAR const struct nxgl_rect_s *rect,
                       nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES])
{
  return nx_fill((NXWINDOW)priv->handle, rect, wcolor);
}

/****************************************************************************
 * Name: nxterm_move
 *
 * Description:
 *   Move a rectangular region within the window
 *
 * Input Parameters:
 *   priv   - The driver state structure.
 *   rect   - Describes the rectangular region to move
 *   offset - The offset to move the region.  The  rectangular region will be
 *            moved so that the origin is translated by this amount.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

#ifndef CONFIG_NX_WRITEONLY
static int nxterm_move(FAR struct nxterm_state_s *priv,
                       FAR const struct nxgl_rect_s *rect,
                       FAR const struct nxgl_point_s *offset)
{
  return nx_move((NXWINDOW)priv->handle, rect, offset);
}
#endif

/****************************************************************************
 * Name: nxterm_bitmap
 *
 * Description:
 *   Copy a rectangular region of a larger image into the rectangle in the
 *   specified window.
 *
 * Input Parameters:
 *   priv   - The driver state structure.
 *   dest   - Describes the rectangular region on the display that will
 *            receive the bit map.
 *   src    - The start of the source image.  This is an array source
 *            images of size CONFIG_NX_NPLANES.
 *   origin - The origin of the upper, left-most corner of the full bitmap.
 *            Both dest and origin are in window coordinates, however, origin
 *            may lie outside of the display.
 *   stride - The width of the full source image in bytes.
 *
 * Returned Value:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

static int nxterm_bitmap(FAR struct nxterm_state_s *priv,
                         FAR const struct nxgl_rect_s *dest,
                         FAR const void *src[CONFIG_NX_NPLANES],
                         FAR const struct nxgl_point_s *origin,
                         unsigned int stride)
{
  return nx_bitmap((NXWINDOW)priv->handle, dest, src, origin, stride);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_register
 *
 * Description:
 *   Register a console device on a raw NX window.  The device will be
 *   registered at /dev/nxtermN where N is the provided minor number.
 *
 * Input Parameters:
 *   hwnd - A handle that will be used to access the window.  The window must
 *     persist and this handle must be valid for the life of the NX console.
 *   wndo - Describes the window and font to be used
 *   minor - The device minor number
 *
 * Returned Value:
 *   A non-NULL handle is returned on success.
 *
 ****************************************************************************/

NXTERM nx_register(NXWINDOW hwnd,
                   FAR struct nxterm_window_s *wndo,
                   int minor)
{
  return nxterm_register((NXTERM)hwnd, wndo, &g_nxops, minor);
}
