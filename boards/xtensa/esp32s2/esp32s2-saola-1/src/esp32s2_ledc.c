/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-saola-1/src/esp32s2_ledc.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "espressif/esp_ledc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_pwm_setup
 *
 * Description:
 *   Initialize LEDC PWM and register the PWM device.
 *
 ****************************************************************************/

int esp32s2_pwm_setup(void)
{
  int ret;
  struct pwm_lowerhalf_s *pwm;

#if defined(CONFIG_ESP32S2_LEDC_TIM0) || defined(CONFIG_ESPRESSIF_LEDC_TIMER0)
  pwm = esp_ledc_init(0);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to get the LEDC PWM 0 lower half\n");
      return -ENODEV;
    }

  /* Register the PWM driver at "/dev/pwm0" */

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_ESP32S2_LEDC_TIM1) || defined(CONFIG_ESPRESSIF_LEDC_TIMER1)
  pwm = esp_ledc_init(1);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to get the LEDC PWM 1 lower half\n");
      return -ENODEV;
    }

  /* Register the PWM driver at "/dev/pwm1" */

  ret = pwm_register("/dev/pwm1", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_ESP32S2_LEDC_TIM2) || defined(CONFIG_ESPRESSIF_LEDC_TIMER2)
  pwm = esp_ledc_init(2);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to get the LEDC PWM 2 lower half\n");
      return -ENODEV;
    }

  /* Register the PWM driver at "/dev/pwm2" */

  ret = pwm_register("/dev/pwm2", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_ESP32S2_LEDC_TIM3) || defined(CONFIG_ESPRESSIF_LEDC_TIMER3)
  pwm = esp_ledc_init(3);
  if (!pwm)
    {
      syslog(LOG_ERR, "ERROR: Failed to get the LEDC PWM 3 lower half\n");
      return -ENODEV;
    }

  /* Register the PWM driver at "/dev/pwm3" */

  ret = pwm_register("/dev/pwm3", pwm);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

  /* May not be used if none LEDC TIMx is enabled */

  UNUSED(ret);
  UNUSED(pwm);

  return OK;
}

