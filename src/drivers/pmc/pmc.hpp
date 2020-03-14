#pragma once

#include <cstdint>
#include <sam.h>

/* Bit mask for peripheral clocks (PCER0) */
#define PMC_MASK_STATUS0	(0xFFFFFFFC)

/* Bit mask for peripheral clocks (PCER1) */
#define PMC_MASK_STATUS1	(0xFFFFFFFF)

/* Loop counter timeout value */
#define PMC_TIMEOUT			(4096)

/* Key to unlock CKGR_MOR register */
#ifndef CKGR_MOR_KEY_PASSWD
#define CKGR_MOR_KEY_PASSWD    CKGR_MOR_KEY(0x37U)
#endif

/* Key used to write SUPC registers */
#ifndef SUPC_CR_KEY_PASSWD
#define SUPC_CR_KEY_PASSWD    SUPC_CR_KEY(0xA5U)
#endif

#ifndef SUPC_MR_KEY_PASSWD
#define SUPC_MR_KEY_PASSWD    SUPC_MR_KEY(0xA5U)
#endif

/* Mask to access fast startup input */
#define PMC_FAST_STARTUP_Msk    (0x7FFFFu)

/* PMC_WPMR Write Protect KEY, unlock it */
#ifndef PMC_WPMR_WPKEY_PASSWD
#define PMC_WPMR_WPKEY_PASSWD    PMC_WPMR_WPKEY((uint32_t) 0x504D43)
#endif

/* Using external oscillator */
#define PMC_OSC_XTAL            0

/* Oscillator in bypass mode */
#define PMC_OSC_BYPASS          1

#define PMC_PCK_0               0 /* PCK0 ID */
#define PMC_PCK_1               1 /* PCK1 ID */
#define PMC_PCK_2               2 /* PCK2 ID */
#define PMC_PCK_3               3 /* PCK3 ID */
#define PMC_PCK_4               4 /* PCK4 ID */
#define PMC_PCK_5               5 /* PCK5 ID */
#define PMC_PCK_6               6 /* PCK6 ID */

/* Flash state in Wait Mode */
#define PMC_WAIT_MODE_FLASH_STANDBY         PMC_FSMR_FLPM_FLASH_STANDBY
#define PMC_WAIT_MODE_FLASH_DEEP_POWERDOWN  PMC_FSMR_FLPM_FLASH_DEEP_POWERDOWN
#define PMC_WAIT_MODE_FLASH_IDLE            PMC_FSMR_FLPM_FLASH_IDLE

/* Convert startup time from us to MOSCXTST */
#define pmc_us_to_moscxtst(startup_us, slowck_freq)      \
	((startup_us * slowck_freq / 8 / 1000000) < 0x100 ?  \
		(startup_us * slowck_freq / 8 / 1000000) : 0xFF)

/*
 * Master clock (MCK) Source and Prescaler configuration
 *
 * The following functions may be used to select the clock source and
 * prescaler for the master clock.
 */
void pmc_mck_set_prescaler(uint32_t ul_pres);
void pmc_mck_set_division(uint32_t ul_div);
void pmc_mck_set_source(uint32_t ul_source);
uint32_t pmc_switch_mck_to_sclk(uint32_t ul_pres);
uint32_t pmc_switch_mck_to_mainck(uint32_t ul_pres);
uint32_t pmc_switch_mck_to_pllack(uint32_t ul_pres);
uint32_t pmc_switch_mck_to_upllck(uint32_t ul_pres);
void pmc_set_flash_in_wait_mode(uint32_t ul_flash_state);

/*
 * Slow clock (SLCK) oscillator and configuration
 */
void pmc_switch_sclk_to_32kxtal(uint32_t ul_bypass);
uint32_t pmc_osc_is_ready_32kxtal();

/*
 * Main Clock (MAINCK) oscillator and configuration
 */
void pmc_switch_mainck_to_fastrc(uint32_t ul_moscrcf);
void pmc_osc_enable_fastrc(uint32_t ul_rc);
void pmc_osc_disable_fastrc();
uint32_t pmc_osc_is_ready_fastrc();
void pmc_osc_enable_main_xtal(uint32_t ul_xtal_startup_time);
void pmc_osc_bypass_main_xtal();
void pmc_osc_disable_main_xtal();
uint32_t pmc_osc_is_bypassed_main_xtal();
uint32_t pmc_osc_is_ready_main_xtal();
void pmc_switch_mainck_to_xtal(uint32_t ul_bypass, uint32_t ul_xtal_startup_time);
void pmc_osc_disable_xtal(uint32_t ul_bypass);
uint32_t pmc_osc_is_ready_mainck();
void pmc_mainck_osc_select(uint32_t ul_xtal_rc);

/*
 * PLL oscillator and configuration
 */
void pmc_enable_pllack(uint32_t mula, uint32_t pllacount, uint32_t diva);
void pmc_disable_pllack();
uint32_t pmc_is_locked_pllack();
void pmc_enable_upll_clock();
void pmc_disable_upll_clock();
uint32_t pmc_is_locked_upll();

/*
 * Peripherals clock configuration
 */
uint32_t pmc_enable_periph_clk(uint32_t ul_id);
uint32_t pmc_disable_periph_clk(uint32_t ul_id);
void pmc_enable_all_periph_clk();
void pmc_disable_all_periph_clk();
uint32_t pmc_is_periph_clk_enabled(uint32_t ul_id);

/*
 * Programmable clock Source and Prescaler configuration
 *
 * The following functions may be used to select the clock source and
 * prescaler for the specified programmable clock.
 */
void pmc_pck_set_prescaler(uint32_t ul_id, uint32_t ul_pres);
void pmc_pck_set_source(uint32_t ul_id, uint32_t ul_source);
uint32_t pmc_switch_pck_to_sclk(uint32_t ul_id, uint32_t ul_pres);
uint32_t pmc_switch_pck_to_mainck(uint32_t ul_id, uint32_t ul_pres);
uint32_t pmc_switch_pck_to_pllack(uint32_t ul_id, uint32_t ul_pres);
uint32_t pmc_switch_pck_to_upllck(uint32_t ul_id, uint32_t ul_pres);
uint32_t pmc_switch_pck_to_mck(uint32_t ul_id, uint32_t ul_pres);
void pmc_enable_pck(uint32_t ul_id);
void pmc_disable_pck(uint32_t ul_id);
void pmc_enable_all_pck();
void pmc_disable_all_pck();
uint32_t pmc_is_pck_enabled(uint32_t ul_id);

/*
 * USB clock configuration
 */
void pmc_switch_udpck_to_pllack(uint32_t ul_usbdiv);
void pmc_switch_udpck_to_upllck(uint32_t ul_usbdiv);
void pmc_enable_udpck();
void pmc_disable_udpck();

/*
 * Interrupt and status management
 */
void pmc_enable_interrupt(uint32_t ul_sources);
void pmc_disable_interrupt(uint32_t ul_sources);
uint32_t pmc_get_interrupt_mask();
uint32_t pmc_get_status();

/*
 * Power management
 *
 * The following functions are used to configure sleep mode and additional
 * wake up inputs.
 */

void pmc_set_fast_startup_input(uint32_t ul_inputs);
void pmc_clr_fast_startup_input(uint32_t ul_inputs);
void pmc_enable_sleepmode(uint8_t uc_type);
void pmc_enable_waitmode();

/*
 * Failure detector
 */
void pmc_enable_clock_failure_detector();
void pmc_disable_clock_failure_detector();

/*
 * Slow Crystal Oscillator Frequency Monitoring
 */
void pmc_enable_sclk_osc_freq_monitor();
void pmc_disable_sclk_osc_freq_monitor();

/*
 * Write protection
 */
void pmc_set_writeprotect(uint32_t ul_enable);
uint32_t pmc_get_writeprotect_status();

/*
 * Sleepwalking configuration
 */
uint32_t pmc_enable_sleepwalking(uint32_t ul_id);
uint32_t pmc_disable_sleepwalking(uint32_t ul_id);
uint32_t pmc_get_sleepwalking_status0();
uint32_t pmc_get_active_status0();
uint32_t pmc_get_sleepwalking_status1();
uint32_t pmc_get_active_status1();