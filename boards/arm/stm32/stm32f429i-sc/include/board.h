/************************************************************************************
 * boards/arm/stm32/stm32f429i-disco/include/board.h
 *
 *   Copyright (C) 2012, 2015-2016, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM32F429I_SC_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM32F429I_SC_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32.h"
/* DO NOT include STM32 internal header files here */

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* The STM32F4 Discovery board features a single 8MHz crystal.  Space is provided
 * for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 180000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 180000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 12000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 12            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 12MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        12000000ul
#define STM32F_BOARD_XTAL_MHZ      ( STM32_BOARD_XTAL / 1000000 )
#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * Restrictions:
 * 2   <= PLLQ <= 15
 * PLLP = 2,4,6 or 8
 * 192 <= PLLN <= 432
 * 2   <= PLLM <= 63
 *
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (12,000,000 / 12) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */
#define STM32_F_PERIPH_FREQ      48000000ul
#define STM32_F_PERIPH_FREQ_MHZ ( STM32_F_PERIPH_FREQ / 1000000 )
#define STM32_SYSCLK_FREQUENCY  168000000ul
#define STM32_SYSCLK_FREQUENCY_MHZ ( STM32_SYSCLK_FREQUENCY / 1000000 )

#define STM32_PLLM              STM32_SYSCLK_FREQUENCY_MHZ
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(STM32_PLLM) //12

#if STM32_SYSCLK_FREQUENCY_MHZ < 96
    #define STM32_PLLP              4
    #define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#else
    #define STM32_PLLP              2
    #define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#endif

#define STM32_PLLN              ( STM32_SYSCLK_FREQUENCY_MHZ * STM32_PLLP )          //336
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(STM32_PLLN)

#define FVCO                    ( (STM32F_BOARD_XTAL_MHZ * STM32_PLLN) / STM32_PLLM) // 12 * 336 / 12

#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(FVCO / STM32_F_PERIPH_FREQ_MHZ )     //7


/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY/2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=84MHz, SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118 + 2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (0x76 << SDIO_CLKCR_CLKDIV_SHIFT) //118
#define DMAMAP_SDIO             DMAMAP_SDIO_2

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1 + 2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2 + 2)=12 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1 + 2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2 + 2)=12 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions ******************************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

#define BOARD_LED_GREEN   BOARD_LED1


/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)


/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on board the
 * stm32f429i-disco.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED3 */
#define LED_INIRQ         4  /* LED1 + LED3 */
#define LED_SIGNAL        5  /* LED2 + LED3 */
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 */
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions ***************************************************************/

/* The STM32F4 Discovery supports one button: */

#define BUTTON_USER        0

#define NUM_BUTTONS        1

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ************************************************/

/* USART3:
 *
 * The STM32F4 Discovery has no on-board serial devices, but the console is
 * brought out to PA9 (TX) and PA10 (RX) for connection to an external serial device.
 * (See the README.txt file for other options)
 */

#define GPIO_USART2_RX GPIO_USART2_RX_1
#define GPIO_USART2_TX GPIO_USART2_TX_1


#define GPIO_USART3_RX GPIO_USART3_RX_1
#define GPIO_USART3_TX GPIO_USART3_TX_1
#define GPIO_USART3_RS485_DIR GPIO_USART3_RTS_1 /*PB14*/

#define GPIO_USART6_RX GPIO_USART6_RX_1
#define GPIO_USART6_TX GPIO_USART6_TX_1


/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM8 CH3N on PB15.
 */

#define GPIO_TIM3_CH4OUT  GPIO_TIM3_CH4OUT_1    /* PB1 */
#define GPIO_TIM8_CH3NOUT GPIO_TIM8_CH3N_2

/* I2C - There is a STMPE811 TouchPanel on I2C3 using these pins: */

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_3
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_3


/* Touch screen (ADS7843) IRQ pin:  PB.6 */
#define GPIO_TS_IRQ  (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN8)


/* SPI - touch screen controller (ADS7843) connected on SPI4: */

#define GPIO_SPI4_MISO GPIO_SPI4_MISO_1
#define GPIO_SPI4_MOSI GPIO_SPI4_MOSI_1
#define GPIO_SPI4_SCK  GPIO_SPI4_SCK_1

/* FMC - SDRAM */

#define GPIO_FMC_SDCKE1 GPIO_FMC_SDCKE1_1
#define GPIO_FMC_SDNE1  GPIO_FMC_SDNE1_1
#define GPIO_FMC_SDNWE  GPIO_FMC_SDNWE_1

/* Timer Inputs/Outputs (see the README.txt file for options) */

#define GPIO_TIM2_CH1IN  GPIO_TIM2_CH1IN_2
#define GPIO_TIM2_CH2IN  GPIO_TIM2_CH2IN_1

#define GPIO_TIM8_CH1IN  GPIO_TIM8_CH1IN_1
#define GPIO_TIM8_CH2IN  GPIO_TIM8_CH2IN_1

#ifdef CONFIG_STM32_LTDC

/* LCD
 *
 * The STM32F429I-DISCO board contains an onboard TFT LCD connected to the
 * LTDC interface of the uC.  The LCD is 240x320 pixels. Define the parameters
 * of the LCD and the interface here.
 */

/* Panel configuration
 *
 * LCD Panel (800RGBx480)
 *
 * PLLSAI settings
 * PLLSAIN                : 192
 * PLLSAIR                : 4
 * PLLSAIQ                : 7
 * PLLSAIDIVR             : 2
 *
 * Timings
 * Horizontal Front Porch : 10  (STM32_LTDC_HFP)
 * Horizontal Back Porch  : 10  (STM32_LTDC_HBP)
 * Vertical Front Porch   :  80  (STM32_LTDC_VFP)
 * Vertical Back Porch    :  8  (STM32_LTDC_VBP)
 *
 * Horizontal Sync        : 10  (STM32_LTDC_HSYNC)
 * Vertical Sync          : 10  (STM32_LTDC_VSYNC)
 *
 * Active Width           : 240 (STM32_LTDC_ACTIVEW)
 * Active Height          : 320 (STM32_LTDC_ACTIVEH)
 */

/* LTDC PLL configuration
 *
 * PLLSAI_VCO = STM32_HSE_FREQUENCY / PLLM
 *            = 8000000ul / 8
 *            = 1,000,000
 *
 * PLL LCD clock output
 *            = PLLSAI_VCO * PLLSAIN / PLLSAIR / PLLSAIDIVR
 *            = 1,000,000 * 192 / 4 /8
 *            = 6,000,000
 */

/* Defined panel settings */

/* Custom LCD display configuration */

# define BOARD_LTDC_WIDTH               800
# define BOARD_LTDC_HEIGHT              480

#define BOARD_LTDC_OUTPUT_BPP           16

#define BOARD_LTDC_HFP                  100
#define BOARD_LTDC_HBP                  10
#define BOARD_LTDC_VFP                  80
#define BOARD_LTDC_VBP                  8
#define BOARD_LTDC_HSYNC                10
#define BOARD_LTDC_VSYNC                10

#define BOARD_LTDC_PLLSAIN              192
#define BOARD_LTDC_PLLSAIR              4
#define BOARD_LTDC_PLLSAIQ              7

/* Division factor for LCD clock */

#define STM32_RCC_DCKCFGR_PLLSAIDIVR    RCC_DCKCFGR_PLLSAIDIVR_DIV2

/* Pixel Clock Polarity */

#define BOARD_LTDC_GCR_PCPOL            LTDC_GCR_PCPOL_IPC

/* Data Enable Polarity */

#define BOARD_LTDC_GCR_DEPOL            LTDC_GCR_DEPOL_AL

/* Vertical Sync Polarity */

#define BOARD_LTDC_GCR_VSPOL            LTDC_GCR_VSPOL_AL

/* Horizontal Sync Polarity */

#define BOARD_LTDC_GCR_HSPOL            LTDC_GCR_HSPOL_AL

/* GPIO pinset */

#define GPIO_LTDC_PINS                  18

#define GPIO_LTDC_R2                    GPIO_LTDC_R2_1
#define GPIO_LTDC_R3                    GPIO_LTDC_R3_1
#define GPIO_LTDC_R4                    GPIO_LTDC_R4_1
#define GPIO_LTDC_R5                    GPIO_LTDC_R5_1
#define GPIO_LTDC_R6                    GPIO_LTDC_R6_1
#define GPIO_LTDC_R7                    GPIO_LTDC_R7_1

#define GPIO_LTDC_G2                    GPIO_LTDC_G2_1
#define GPIO_LTDC_G3                    GPIO_LTDC_G3_1
#define GPIO_LTDC_G4                    GPIO_LTDC_G4_1
#define GPIO_LTDC_G5                    GPIO_LTDC_G5_1
#define GPIO_LTDC_G6                    GPIO_LTDC_G6_1
#define GPIO_LTDC_G7                    GPIO_LTDC_G7_1

#define GPIO_LTDC_B2                    GPIO_LTDC_B2_1
#define GPIO_LTDC_B3                    GPIO_LTDC_B3_1
#define GPIO_LTDC_B4                    GPIO_LTDC_B4_1
#define GPIO_LTDC_B5                    GPIO_LTDC_B5_1
#define GPIO_LTDC_B6                    GPIO_LTDC_B6_1
#define GPIO_LTDC_B7                    GPIO_LTDC_B7_1

#define GPIO_LTDC_VSYNC                 GPIO_LTDC_VSYNC_1
#define GPIO_LTDC_HSYNC                 GPIO_LTDC_HSYNC_1
#define GPIO_LTDC_DE                    GPIO_LTDC_DE_1
#define GPIO_LTDC_CLK                   GPIO_LTDC_CLK_1

/* Custom LCD display */

/* Configure PLLSAI */

#define STM32_RCC_PLLSAICFGR_PLLSAIN    RCC_PLLSAICFGR_PLLSAIN(BOARD_LTDC_PLLSAIN)
#define STM32_RCC_PLLSAICFGR_PLLSAIR    RCC_PLLSAICFGR_PLLSAIR(BOARD_LTDC_PLLSAIR)
#define STM32_RCC_PLLSAICFGR_PLLSAIQ    RCC_PLLSAICFGR_PLLSAIQ(BOARD_LTDC_PLLSAIQ)

#endif /* CONFIG_STM32_LTDC */

#ifdef CONFIG_STM32_OTGFS
#define GPIO_OTGFS_VBUS  GPIO_OTGHS_VBUS   /*PA9*/
#define GPIO_OTGFS_PWRON GPIO_OTGHS_PWRON  /*PA8*/
#define GPIO_OTGFS_OVER  GPIO_OTGHS_OVER   /*PI3*/
#endif


/* DMA ******************************************************************************/

#define ADC1_DMA_CHAN DMAMAP_ADC1_1

/* SOUND ****************************************************************************/
#define BOARD_TONE_PWM_TIM ( 3 )
#define BOARD_TONE_ONESHOT_TIM ( 1 )
#define BOARD_TONE_ONESHOT_TIM_RES ( 1000 )

#endif /* __BOARDS_ARM_STM32_STM32F429I_SC_INCLUDE_BOARD_H */
