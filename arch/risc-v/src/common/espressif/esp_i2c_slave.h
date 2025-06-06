/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_i2c_slave.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_I2C_SLAVE_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_I2C_SLAVE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/i2c/i2c_slave.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_ESPRESSIF_I2C0_SLAVE_MODE
#  define ESPRESSIF_I2C0_SLAVE 0
#endif

#ifdef CONFIG_ESPRESSIF_I2C1_SLAVE_MODE
#  define ESPRESSIF_I2C1_SLAVE 1
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE
/****************************************************************************
 * Name: esp_i2cbus_slave_initialize
 *
 * Description:
 *   Initialize the selected I2C port. And return a unique instance of struct
 *   struct i2c_slave_s.  This function may be called to obtain multiple
 *   instances of the interface, each of which may be set up with a
 *   different frequency.
 *
 * Input Parameters:
 *   port - Port number (for hardware that has multiple I2C interfaces)
 *   addr - Address of the slave device
 *
 * Returned Value:
 *   Valid I2C device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct i2c_slave_s *esp_i2cbus_slave_initialize(int port, int addr);

/****************************************************************************
 * Name: esp_i2cbus_uninitialize
 *
 * Description:
 *   De-initialize the selected I2C port, and power down the device.
 *
 * Input Parameters:
 *   dev - Device structure as returned by the esp_i2cbus_slave_initialize()
 *
 * Returned Value:
 *   OK on success, ERROR when internal reference count mismatch or dev
 *   points to invalid hardware device.
 *
 ****************************************************************************/

int esp_i2cbus_slave_uninitialize(struct i2c_slave_s *dev);
#endif /* CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE */

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_I2C_SLAVE_H */
