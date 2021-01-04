/****************************************************************************
 * boards/arm/stm32/stm32f429i-sc/src/stm32_extmem.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"

#include "stm32.h"
#include "stm32f429i-sc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FMC
#warning "FMC is not enabled"
#endif

#if STM32_NGPIO_PORTS < 6
#error "Required GPIO ports not enabled"
#endif

#define STM32_SDRAM_CLKEN     FMC_SDCMR_CMD_CLK_ENABLE | FMC_SDCMR_BANK_2

#define STM32_SDRAM_PALL      FMC_SDCMR_CMD_PALL | FMC_SDCMR_BANK_2

#define STM32_SDRAM_REFRESH   FMC_SDCMR_CMD_AUTO_REFRESH | FMC_SDCMR_BANK_2 |\
                                FMC_SDCMR_NRFS(4)

#define STM32_SDRAM_MODEREG   FMC_SDCMR_CMD_LOAD_MODE | FMC_SDCMR_BANK_2 |\
                                FMC_SDCMR_MDR_BURST_LENGTH_2 | \
                                FMC_SDCMR_MDR_BURST_TYPE_SEQUENTIAL |\
                                FMC_SDCMR_MDR_CAS_LATENCY_3 |\
                                FMC_SDCMR_MDR_WBL_SINGLE

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO configurations common to most external memories */

static const uint32_t g_sdram_config[] = {
  /* 16 data lines */
  GPIO_FMC_D0, GPIO_FMC_D1, GPIO_FMC_D2, GPIO_FMC_D3,
  GPIO_FMC_D4, GPIO_FMC_D5, GPIO_FMC_D6, GPIO_FMC_D7,
  GPIO_FMC_D8, GPIO_FMC_D9, GPIO_FMC_D10, GPIO_FMC_D11,
  GPIO_FMC_D12, GPIO_FMC_D13, GPIO_FMC_D14, GPIO_FMC_D15,

  /* 13 address lines */
  GPIO_FMC_A0, GPIO_FMC_A1, GPIO_FMC_A2, GPIO_FMC_A3,
  GPIO_FMC_A4, GPIO_FMC_A5, GPIO_FMC_A6, GPIO_FMC_A7,
  GPIO_FMC_A8, GPIO_FMC_A9, GPIO_FMC_A10, GPIO_FMC_A11,
  GPIO_FMC_A12,

  /* control lines */
  GPIO_FMC_SDCKE1,  // ^SDCKE PB5
  GPIO_FMC_SDNE1,   // ^SDCE  PB6
  GPIO_FMC_SDNWE,   // ^SDNWE PC0
  GPIO_FMC_SDNRAS,  // ^RAS   PF11
  GPIO_FMC_BA0,     //  BA0   PG4
  GPIO_FMC_BA1,     //  BA1   PG5
  GPIO_FMC_SDCLK,   // SDCLK  PG8
  GPIO_FMC_SDNCAS,  // ^CAS   PG15
  GPIO_FMC_NBL0,    // DQML   PE0
  GPIO_FMC_NBL1,    // DQMH   PE1

};

#define NUM_SDRAM_GPIOS (sizeof(g_sdram_config) / sizeof(uint32_t))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdram_initialize
 *
 * Description:
 *   Called from stm32_bringup to initialize external SDRAM access.
 *
 ****************************************************************************/

void stm32_sdram_initialize(void)
{
  uint32_t val;
  int i;
  volatile int count;

  /* Enable GPIOs as FMC / memory pins */
  for (i = 0; i < NUM_SDRAM_GPIOS; i++)
    {
      stm32_configgpio(g_sdram_config[i]);
    }

  /* Enable AHB clocking to the FMC */
  stm32_fmc_enable();

  /* Configure and enable the SDRAM bank1
   *
   *   FMC clock = 168MHz/2 = 90MHz
   *   90MHz = 11,11 ns
   *   All timings from the datasheet for Speedgrade -7 (=7ns)
   */
  val =
    FMC_SDCR_RPIPE_0       |    // rpipe = 0 hclk         !
	FMC_SDCR_READBURST     |    // ReadBurst              !
    FMC_SDCR_SDCLK_2X      |    // sdclk = 2 hclk         !
	                            // Write Protection
    FMC_SDCR_CAS_LATENCY_2 |    // cas latency = 2 cycles !
    FMC_SDCR_NBANKS_4      |    // 4 internal banks       !
    FMC_SDCR_WIDTH_16      |    // width = 16 bits        !
    FMC_SDCR_ROWS_13       |    // numrows = 13           !
    FMC_SDCR_COLS_9;            // numcols = 9 bits       !
  stm32_fmc_sdram_set_control(1, val);
  stm32_fmc_sdram_set_control(2, val);
/*
  val =
    FMC_SDTR_TRCD(3)     |      // tRCD min = 15ns RCDelay
    FMC_SDTR_TRP(3)      |      // tRP  min = 15ns RPDelay
    FMC_SDTR_TWR(3)      |      // tWR      = 2CLK WriteRecoveryTime
    FMC_SDTR_TRC(8)      |      // tRC  min = 63ns RowCycleDelay
    FMC_SDTR_TRAS(5)     |      // tRAS min = 42ns SelfRefreshTime
    FMC_SDTR_TXSR(8)     |      // tXSR min = 70ns ExitSelfRefreshDelay
    FMC_SDTR_TMRD(3);           // tMRD     = 2CLK LoadToActivityDelay
*/
  val =
    FMC_SDTR_TRCD(2)     |      // tRCD min = 15ns RCDelay
    FMC_SDTR_TRP(2)      |      // tRP  min = 15ns RPDelay
    FMC_SDTR_TWR(3)      |      // tWR      = 2CLK WriteRecoveryTime
    FMC_SDTR_TRC(7)      |      // tRC  min = 63ns RowCycleDelay
    FMC_SDTR_TRAS(4)     |      // tRAS min = 42ns SelfRefreshTime
    FMC_SDTR_TXSR(7)     |      // tXSR min = 70ns ExitSelfRefreshDelay
    FMC_SDTR_TMRD(2);           // tMRD     = 2CLK LoadToActivityDelay
  stm32_fmc_sdram_set_timing(2, val);

  /* SDRAM Initialization sequence */
  stm32_fmc_sdram_command(STM32_SDRAM_CLKEN);   /* Clock enable command */
  for (count = 0; count < 10000; count++);      /* Delay */
  stm32_fmc_sdram_command(STM32_SDRAM_PALL);    /* Precharge ALL command */
  stm32_fmc_sdram_command(STM32_SDRAM_REFRESH); /* Auto refresh command */
  stm32_fmc_sdram_command(STM32_SDRAM_MODEREG); /* Mode Register program */

  /* Set refresh count
   *
   * FMC_CLK = 90MHz
   * Refresh_Rate = 7.81us
   * Counter = (FMC_CLK * Refresh_Rate) - 20
   */
  stm32_fmc_sdram_set_refresh_rate(683);

  /* Disable write protection */
  // stm32_fmc_sdram_write_protect(2, false);
}
