/**
 * \file
 *
 * \brief System configuration file for ATSAME70J21
 *
 * Copyright (c) 2019 Microchip Technology Inc.
 *
 * \license_start
 *
 * \page License
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \license_stop
 *
 */

#include "same70j21.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Clock Settings (600MHz PLL VDDIO 3.3V and VDDCORE 1.2V) */
/* Clock Settings (300MHz HCLK, 150MHz MCK)=> PRESC = 2, MDIV = 2 */
#define SYS_BOARD_OSCOUNT   (CKGR_MOR_MOSCXTST(0x8U))
#define SYS_BOARD_PLLAR     (CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0x31U) | \
                            CKGR_PLLAR_PLLACOUNT(0x3fU) | CKGR_PLLAR_DIVA(0x1U))
#define SYS_BOARD_MCKR      (PMC_MCKR_PRES_CLK_2 | PMC_MCKR_CSS_PLLA_CLK | (1<<8))

uint32_t SystemCoreClock = CHIP_FREQ_CPU_MAX;  /*!< System Clock Frequency (Core Clock)*/

/**
 * Initialize the system
 *
 * \brief  Setup the microcontroller system.
 *         Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit(void)
{
    /* Set FWS according to SYS_BOARD_MCKR configuration */
	EFC->EEFC_FMR = EEFC_FMR_FWS(6);

	/* Initialize main oscillator */
	if (!(PMC->CKGR_MOR & CKGR_MOR_MOSCSEL))
	{
		PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN;

		while (!(PMC->PMC_SR & PMC_SR_MOSCXTS)) {}
	}

	/* Switch to 3-20MHz Xtal oscillator */
	PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | SYS_BOARD_OSCOUNT | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCSEL;

	while (!(PMC->PMC_SR & PMC_SR_MOSCSELS)) {}

	PMC->PMC_MCKR = (PMC->PMC_MCKR & ~(uint32_t)PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;

	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {}

	/* Initialize PLLA */
	PMC->CKGR_PLLAR = SYS_BOARD_PLLAR;
	while (!(PMC->PMC_SR & PMC_SR_LOCKA)) {}

	/* Switch to main clock */
	PMC->PMC_MCKR = (SYS_BOARD_MCKR & ~PMC_MCKR_CSS_Msk) | PMC_MCKR_CSS_MAIN_CLK;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {}

	/* Switch to PLLA */
	PMC->PMC_MCKR = SYS_BOARD_MCKR;
	while (!(PMC->PMC_SR & PMC_SR_MCKRDY)) {}

	SystemCoreClock = CHIP_FREQ_CPU_MAX;
}

/**
 * Update SystemCoreClock variable
 *
 * \brief  Updates the SystemCoreClock with current core Clock
 *         retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void)
{
    /* Determine clock frequency according to clock register values */
	switch (PMC->PMC_MCKR & (uint32_t) PMC_MCKR_CSS_Msk)
	{
		case PMC_MCKR_CSS_SLOW_CLK: /* Slow clock */
			if (SUPC->SUPC_SR & SUPC_SR_OSCSEL)
				SystemCoreClock = CHIP_FREQ_XTAL_32K;
			else
				SystemCoreClock = CHIP_FREQ_SLCK_RC;
		break;

		case PMC_MCKR_CSS_MAIN_CLK: /* Main clock */
		if (PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)
			SystemCoreClock = CHIP_FREQ_XTAL_12M;
		else
		{
			SystemCoreClock = CHIP_FREQ_MAINCK_RC_4MHZ;

			switch (PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk)
			{
			case CKGR_MOR_MOSCRCF_4_MHz:
			break;

			case CKGR_MOR_MOSCRCF_8_MHz:
				SystemCoreClock *= 2U;
			break;

			case CKGR_MOR_MOSCRCF_12_MHz:
				SystemCoreClock *= 3U;
			break;

			default:
			break;
			}
		}
		break;

		case PMC_MCKR_CSS_PLLA_CLK:	/* PLLA clock */
		if (PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)
			SystemCoreClock = CHIP_FREQ_XTAL_12M;
		else
		{
			SystemCoreClock = CHIP_FREQ_MAINCK_RC_4MHZ;

			switch (PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk)
			{
			case CKGR_MOR_MOSCRCF_4_MHz:
			break;

			case CKGR_MOR_MOSCRCF_8_MHz:
				SystemCoreClock *= 2U;
			break;

			case CKGR_MOR_MOSCRCF_12_MHz:
				SystemCoreClock *= 3U;
			break;

			default:
			break;
			}
		}

		if ((uint32_t)(PMC->PMC_MCKR & (uint32_t)PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK)
		{
			SystemCoreClock *= ((((PMC->CKGR_PLLAR) & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos) + 1U);
			SystemCoreClock /= ((((PMC->CKGR_PLLAR) & CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos));
		}
		break;

		default:
		break;
	}

	if ((PMC->PMC_MCKR & PMC_MCKR_PRES_Msk) == PMC_MCKR_PRES_CLK_3)
		SystemCoreClock /= 3U;
	else
		SystemCoreClock >>= ((PMC->PMC_MCKR & PMC_MCKR_PRES_Msk) >> PMC_MCKR_PRES_Pos);
}

/**
 * Initialize flash.
 */
void system_init_flash(uint32_t ul_clk)
{
	/* Set FWS for embedded Flash access according to operating frequency */
	if (ul_clk < CHIP_FREQ_FWS_0)
	{
		EFC->EEFC_FMR = EEFC_FMR_FWS(0) | EEFC_FMR_CLOE;
	}
	else
	{
		if (ul_clk < CHIP_FREQ_FWS_1)
		{
			EFC->EEFC_FMR = EEFC_FMR_FWS(1) | EEFC_FMR_CLOE;
		}
		else
		{
			if (ul_clk < CHIP_FREQ_FWS_2)
			{
				EFC->EEFC_FMR = EEFC_FMR_FWS(2) | EEFC_FMR_CLOE;
			}
			else
			{
				if (ul_clk < CHIP_FREQ_FWS_3)
				{
					EFC->EEFC_FMR = EEFC_FMR_FWS(3) | EEFC_FMR_CLOE;
				}
				else
				{
					if (ul_clk < CHIP_FREQ_FWS_4)
					{
						EFC->EEFC_FMR = EEFC_FMR_FWS(4) | EEFC_FMR_CLOE;
					}
					else
					{
                        if (ul_clk < CHIP_FREQ_FWS_5)
                        {
                            EFC->EEFC_FMR = EEFC_FMR_FWS(5) | EEFC_FMR_CLOE;
                        }
                        else
                        {
						    EFC->EEFC_FMR = EEFC_FMR_FWS(6) | EEFC_FMR_CLOE;
                        }
					}
				}
			}
		}
	}
}

/** \cond 0 */
/* *INDENT-OFF* */
#ifdef __cplusplus
}
#endif
/* *INDENT-ON* */
/** \endcond */
