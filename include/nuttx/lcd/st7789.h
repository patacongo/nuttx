/****************************************************************************
 * include/nuttx/lcd/st7789.h
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

#ifndef __INCLUDE_NUTTX_LCD_ST7789_H
#define __INCLUDE_NUTTX_LCD_ST7789_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_PORTRAIT   0
#define LCD_LANDSCAPE  1
#define LCD_RPORTRAIT  2
#define LCD_RLANDSCAPE 3

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  st7789_initialize
 *
 * Description:
 *   Initialize the ST7789 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_DYN_ORIENTATION
FAR struct lcd_dev_s *st7789_lcdinitialize(FAR struct spi_dev_s *spi,
                                           uint8_t orientation,
                                           uint16_t xoff, uint16_t yoff);
#else
FAR struct lcd_dev_s *st7789_lcdinitialize(FAR struct spi_dev_s *spi);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_ST7789_H */
