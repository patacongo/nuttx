/****************************************************************************
 * boards/arm/stm32/stm32f401rc-rs485/include/board.h
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

#ifndef __BOARDS_ARM_STM32F401RC_RS485_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F401RC_RS485_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The STM32F401RC-RS485 uses an external 32kHz crystal (X2) to enable HSE
 * clock.
 *
 *   System Clock source           : PLL (HSI)
 *   SYSCLK(Hz)                    : 84000000     Determined by PLL
 *                                                configuration
 *   HCLK(Hz)                      : 84000000     (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 2            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 1            (STM32_RCC_CFGR_PPRE2)
 *   HSI Frequency(Hz)             : 16000000     (nominal)
 *   PLLM                          : 16           (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 4            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PPQ)
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - not installed
 * LSE - not installed
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_BOARD_USEHSI      1

/* Main PLL Configuration.
 *
 * Formulae:
 *
 *   VCO input frequency        = PLL input clock frequency / PLLM,
 *                                2 <= PLLM <= 63
 *   VCO output frequency       = VCO input frequency × PLLN,
 *                                192 <= PLLN <= 432
 *   PLL output clock frequency = VCO frequency / PLLP,
 *                                PLLP = 2, 4, 6, or 8
 *   USB OTG FS clock frequency = VCO frequency / PLLQ,
 *                                2 <= PLLQ <= 15
 *
 * We would like to have SYSYCLK=84MHz and we must have the USB clock= 48MHz.
 * Some possible solutions include:
 *
 *   PLLN=210 PLLM=5  PLLP=8 PLLQ=14 SYSCLK=84000000 OTGFS=48000000
 *   PLLN=210 PLLM=10 PLLP=4 PLLQ=7  SYSCLK=84000000 OTGFS=48000000
 *   PLLN=336 PLLM=8  PLLP=8 PLLQ=14 SYSCLK=84000000 OTGFS=48000000
 *   PLLN=336 PLLM=16 PLLP=4 PLLQ=7  SYSCLK=84000000 OTGFS=48000000
 *   PLLN=420 PLLM=10 PLLP=8 PLLQ=14 SYSCLK=84000000 OTGFS=48000000
 *   PLLN=420 PLLM=20 PLLP=4 PLLQ=7  SYSCLK=84000000 OTGFS=48000000
 *
 * We will configure like this
 *
 *   PLL source is HSI
 *   PLL_VCO = (STM32_HSI_FREQUENCY / PLLM) * PLLN
 *           = (16,000,000 / 16) * 336
 *           = 336,000,000
 *   SYSCLK  = PLL_VCO / PLLP
 *           = 336,000,000 / 4 = 84,000,000
 *   USB OTG FS and SDIO Clock
 *           = PLL_VCO / PLLQ
 *           = 336,000,000 / 7 = 48,000,000
 *
 * REVISIT: Trimming of the HSI is not yet supported.
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(16)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  84000000ul

/* AHB clock (HCLK) is SYSCLK (84MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 */

/* REVISIT */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/1)

/* Timers driven from APB2 will be twice PCLK2 */

/* REVISIT */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

/* REVISIT */

#define BOARD_TIM1_FREQUENCY    (2*STM32_PCLK2_FREQUENCY)
#define BOARD_TIM2_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (2*STM32_PCLK2_FREQUENCY)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 */

/* REVISIT */

#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

/* REVISIT */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

/* REVISIT */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in
 * the future is we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3 <- may later be used by SPI DMA
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

/* Need to VERIFY fwb */

#define DMACHAN_SPI1_RX DMAMAP_SPI1_RX_1
#define DMACHAN_SPI1_TX DMAMAP_SPI1_TX_1
#define DMACHAN_SPI2_RX DMAMAP_SPI2_RX
#define DMACHAN_SPI2_TX DMAMAP_SPI2_TX

/* Alternate function pin selections ****************************************/

/* USART2:
 *   RXD: PA3   CN4 pin 20
 *   TXD: PA2   CN4 pin 18
 */

#ifdef CONFIG_USART2_RS485
  /* Lets use for RS485 */

#  define GPIO_USART2_TX        GPIO_USART2_TX_1 /* PA2 */
#  define GPIO_USART2_RX        GPIO_USART2_RX_1 /* PA3 */

  /* RS485 DIR pin: PA1 */

#  define GPIO_USART2_RS485_DIR (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz |\
                                 GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN1)

#endif

/* USART6:
 *  RXD: PC7    CN2 pin 15
 *  TXD: PC6    CN2 pin 17
 */

#define GPIO_USART6_RX   GPIO_USART6_RX_1    /* PC7 */
#define GPIO_USART6_TX   GPIO_USART6_TX_1    /* PC6 */

/* PWM
 *
 * The STM32F401RC-RS485 has no real on-board PWM devices, but the board
 * can be configured to output a pulse train using TIM3 CH1 on PA6.
 */

#define GPIO_TIM3_CH1OUT  GPIO_TIM3_CH1OUT_1

/* Quadrature Encoder
 *
 * Use Timer 3 (TIM3) on channels 1 and 2 for QEncoder, using PB4 and PA7.
 */

#define  GPIO_TIM3_CH1IN GPIO_TIM3_CH1IN_2
#define  GPIO_TIM3_CH2IN GPIO_TIM3_CH2IN_1

/* HCSR04 driver */

/* Pins config to use with HC-SR04 sensor */

#define GPIO_HCSR04_INT   (GPIO_INPUT |GPIO_FLOAT |GPIO_EXTI | GPIO_PORTB | GPIO_PIN1)
#define GPIO_HCSR04_TRIG  (GPIO_OUTPUT_CLEAR | GPIO_OUTPUT | GPIO_SPEED_50MHz | GPIO_PORTB | GPIO_PIN0)

#define BOARD_HCSR04_GPIO_INT  GPIO_HCSR04_INT
#define BOARD_HCSR04_GPIO_TRIG GPIO_HCSR04_TRIG
#define BOARD_HCSR04_FRTIMER   1    /* TIM1 as free running timer */

/* I2C
 *
 * The optional _GPIO configurations allow the I2C driver to manually
 * reset the bus to clear stuck slaves.  They match the pin configuration,
 * but are normally-high GPIOs.
 */

#define GPIO_I2C1_SCL    GPIO_I2C1_SCL_2
#define GPIO_I2C1_SDA    GPIO_I2C1_SDA_1

#define GPIO_I2C2_SCL    GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA    GPIO_I2C2_SDA_2

/* SPI
 *
 * There are sensors on SPI1, and SPI2 is connected to the FRAM.
 */

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2

/* MAX7219 */

#define STM32_LCD_CS (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)

/* MFRC522 */

#define GPIO_RFID_CS      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)

/* MAX31855 */

#define GPIO_MAX31855_CS   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN4)

/* MAX6675 */

#define GPIO_MAX6675_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN5)
/* LEDs
 *
 * The STM32F401RC-RS485 boards provide 4 blue user LEDs. LD1, LD2, LD3
 * and LD4 that are connected to MCU I/O pins PC0, PC1, PC2 and PC3.
 *   - When the I/O is HIGH value, the LED is on.
 *   - When the I/O is LOW, the LED is off.
 */

/* LED index values for use with board_userled() */

#define BOARD_LD1         0
#define BOARD_LD2         1
#define BOARD_LD3         2
#define BOARD_LD4         3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LD1)
#define BOARD_LED2_BIT    (1 << BOARD_LD2)
#define BOARD_LED3_BIT    (1 << BOARD_LD3)
#define BOARD_LED4_BIT    (1 << BOARD_LD4)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows when the red LED (PE24) is available:
 *
 *   SYMBOL                Meaning                   LD2
 *   -------------------  -----------------------  -----------
 *   LED_STARTED          NuttX has been started     OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF
 *   LED_STACKCREATED     Idle stack created         ON
 *   LED_INIRQ            In an interrupt            No change
 *   LED_SIGNAL           In a signal handler        No change
 *   LED_ASSERTION        An assertion failed        No change
 *   LED_PANIC            The system has crashed     Blinking
 *   LED_IDLE             MCU is is sleep mode       Not used
 *
 * Thus if LD2, NuttX has successfully booted and is, apparently, running
 * normally.  If LD2 is flashing at approximately 2Hz, then a fatal error
 * has been detected and the system has halted.
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Buttons
 *   The STM32F401RC-RS485 has 3 user buttons: SW3, SW4, and SW5.
 *   They are connected to PB13, PB14, and PB15 respectively.
 */

#define BUTTON_SW3         0
#define BUTTON_SW4         1
#define BUTTON_SW5         2
#define NUM_BUTTONS        3

#define BUTTON_SW3_BIT     (1 << BUTTON_SW3)
#define BUTTON_SW4_BIT     (1 << BUTTON_SW4)
#define BUTTON_SW5_BIT     (1 << BUTTON_SW5)

#define GPIO_TIM2_CH1IN (GPIO_TIM2_CH1IN_1 | GPIO_PULLUP)
#define GPIO_TIM2_CH2IN (GPIO_TIM2_CH2IN_1 | GPIO_PULLUP)

/* Stepper Motor - DRV8266 */

#define GPIO_DIR     (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN7)
#define GPIO_STEP    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN4)
#define GPIO_SLEEP   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)

#define GPIO_M1      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN0)
#define GPIO_M2      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN1)
#define GPIO_M3      (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)

#define GPIO_RESET   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN10)

#endif /* __BOARDS_ARM_STM32F401RC_RS485_INCLUDE_BOARD_H */
