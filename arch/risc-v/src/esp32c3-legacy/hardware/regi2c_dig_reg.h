/****************************************************************************
 * arch/risc-v/src/esp32c3-legacy/hardware/regi2c_dig_reg.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_REGI2C_DIG_REG_H
#define __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_REGI2C_DIG_REG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register definitions for digital to get rtc voltage & digital voltage
 * by setting rtc_dbias_Wak & dig_dbias_wak or by analog self-calibration.
 */

#define I2C_DIG_REG 0x6D
#define I2C_DIG_REG_HOSTID 0

#define I2C_DIG_REG_EXT_RTC_DREG    4
#define I2C_DIG_REG_EXT_RTC_DREG_MSB    4
#define I2C_DIG_REG_EXT_RTC_DREG_LSB    0

#define I2C_DIG_REG_EXT_RTC_DREG_SLEEP    5
#define I2C_DIG_REG_EXT_RTC_DREG_SLEEP_MSB    4
#define I2C_DIG_REG_EXT_RTC_DREG_SLEEP_LSB    0

#define I2C_DIG_REG_EXT_DIG_DREG    6
#define I2C_DIG_REG_EXT_DIG_DREG_MSB    4
#define I2C_DIG_REG_EXT_DIG_DREG_LSB    0

#define I2C_DIG_REG_EXT_DIG_DREG_SLEEP    7
#define I2C_DIG_REG_EXT_DIG_DREG_SLEEP_MSB    4
#define I2C_DIG_REG_EXT_DIG_DREG_SLEEP_LSB    0

#define I2C_DIG_REG_XPD_RTC_REG 13
#define I2C_DIG_REG_XPD_RTC_REG_MSB 2
#define I2C_DIG_REG_XPD_RTC_REG_LSB 2

#define I2C_DIG_REG_XPD_DIG_REG 13
#define I2C_DIG_REG_XPD_DIG_REG_MSB 3
#define I2C_DIG_REG_XPD_DIG_REG_LSB 3

#endif /* __ARCH_RISCV_SRC_ESP32C3_LEGACY_HARDWARE_REGI2C_DIG_REG_H */
