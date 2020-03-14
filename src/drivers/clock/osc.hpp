 
#pragma once

#include <board_same70.hpp>
#include <conf_clock.h>
#include "../pmc/pmc.hpp"

#if !defined(BOARD_FREQ_SLCK_XTAL)
#  warning The board slow clock xtal frequency has not been defined.
#  define BOARD_FREQ_SLCK_XTAL      (32768UL)
#endif

#if !defined(BOARD_FREQ_SLCK_BYPASS)
#  warning The board slow clock bypass frequency has not been defined.
#  define BOARD_FREQ_SLCK_BYPASS    (32768UL)
#endif

#if !defined(BOARD_FREQ_MAINCK_XTAL)
#  warning The board main clock xtal frequency has not been defined.
#  define BOARD_FREQ_MAINCK_XTAL    (12000000UL)
#endif

#if !defined(BOARD_FREQ_MAINCK_BYPASS)
#  warning The board main clock bypass frequency has not been defined.
#  define BOARD_FREQ_MAINCK_BYPASS  (12000000UL)
#endif

#if !defined(BOARD_OSC_STARTUP_US)
#  warning The board main clock xtal startup time has not been defined.
#  define BOARD_OSC_STARTUP_US      (15625UL)
#endif

// Oscillator identifiers
#define OSC_SLCK_32K_RC      0    //!< Internal 32kHz RC oscillator.
#define OSC_SLCK_32K_XTAL    1    //!< External 32kHz crystal oscillator.
#define OSC_SLCK_32K_BYPASS  2    //!< External 32kHz bypass oscillator.
#define OSC_MAINCK_4M_RC     3    //!< Internal 4MHz RC oscillator.
#define OSC_MAINCK_8M_RC     4    //!< Internal 8MHz RC oscillator.
#define OSC_MAINCK_12M_RC    5    //!< Internal 12MHz RC oscillator.
#define OSC_MAINCK_XTAL      6    //!< External crystal oscillator.
#define OSC_MAINCK_BYPASS    7    //!< External bypass oscillator.

// Oscillator clock speed in hertz
#define OSC_SLCK_32K_RC_HZ      CHIP_FREQ_SLCK_RC         //!< Internal 32kHz RC oscillator.
#define OSC_SLCK_32K_XTAL_HZ    BOARD_FREQ_SLCK_XTAL      //!< External 32kHz crystal oscillator.
#define OSC_SLCK_32K_BYPASS_HZ  BOARD_FREQ_SLCK_BYPASS    //!< External 32kHz bypass oscillator.
#define OSC_MAINCK_4M_RC_HZ     CHIP_FREQ_MAINCK_RC_4MHZ  //!< Internal 4MHz RC oscillator.
#define OSC_MAINCK_8M_RC_HZ     CHIP_FREQ_MAINCK_RC_8MHZ  //!< Internal 8MHz RC oscillator.
#define OSC_MAINCK_12M_RC_HZ    CHIP_FREQ_MAINCK_RC_12MHZ //!< Internal 12MHz RC oscillator.
#define OSC_MAINCK_XTAL_HZ      BOARD_FREQ_MAINCK_XTAL    //!< External crystal oscillator.
#define OSC_MAINCK_BYPASS_HZ    BOARD_FREQ_MAINCK_BYPASS  //!< External bypass oscillator.

static inline void osc_enable(uint32_t ul_id)
{
    switch (ul_id) {
    case OSC_SLCK_32K_RC:
        break;

    case OSC_SLCK_32K_XTAL:
        pmc_switch_sclk_to_32kxtal(PMC_OSC_XTAL);
        break;

    case OSC_SLCK_32K_BYPASS:
        pmc_switch_sclk_to_32kxtal(PMC_OSC_BYPASS);
        break;


    case OSC_MAINCK_4M_RC:
        pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
        break;

    case OSC_MAINCK_8M_RC:
        pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_8_MHz);
        break;

    case OSC_MAINCK_12M_RC:
        pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_12_MHz);
        break;


    case OSC_MAINCK_XTAL:
        pmc_switch_mainck_to_xtal(PMC_OSC_XTAL,
            pmc_us_to_moscxtst(BOARD_OSC_STARTUP_US,
                OSC_SLCK_32K_RC_HZ));
        break;

    case OSC_MAINCK_BYPASS:
        pmc_switch_mainck_to_xtal(PMC_OSC_BYPASS,
            pmc_us_to_moscxtst(BOARD_OSC_STARTUP_US,
                OSC_SLCK_32K_RC_HZ));
        break;
    }
}

static inline void osc_disable(uint32_t ul_id)
{
    switch (ul_id) {
    case OSC_SLCK_32K_RC:
    case OSC_SLCK_32K_XTAL:
    case OSC_SLCK_32K_BYPASS:
        break;

    case OSC_MAINCK_4M_RC:
    case OSC_MAINCK_8M_RC:
    case OSC_MAINCK_12M_RC:
        pmc_osc_disable_fastrc();
        break;

    case OSC_MAINCK_XTAL:
        pmc_osc_disable_xtal(PMC_OSC_XTAL);
        break;

    case OSC_MAINCK_BYPASS:
        pmc_osc_disable_xtal(PMC_OSC_BYPASS);
        break;
    }
}

static inline bool osc_is_ready(uint32_t ul_id)
{
    switch (ul_id) {
    case OSC_SLCK_32K_RC:
        return 1;

    case OSC_SLCK_32K_XTAL:
    case OSC_SLCK_32K_BYPASS:
        return pmc_osc_is_ready_32kxtal();

    case OSC_MAINCK_4M_RC:
    case OSC_MAINCK_8M_RC:
    case OSC_MAINCK_12M_RC:
    case OSC_MAINCK_XTAL:
    case OSC_MAINCK_BYPASS:
        return pmc_osc_is_ready_mainck();
    }

    return 0;
}

static inline uint32_t osc_get_rate(uint32_t ul_id)
{
    switch (ul_id) {
    case OSC_SLCK_32K_RC:
        return OSC_SLCK_32K_RC_HZ;

    case OSC_SLCK_32K_XTAL:
        return BOARD_FREQ_SLCK_XTAL;

    case OSC_SLCK_32K_BYPASS:
        return BOARD_FREQ_SLCK_BYPASS;

    case OSC_MAINCK_4M_RC:
        return OSC_MAINCK_4M_RC_HZ;

    case OSC_MAINCK_8M_RC:
        return OSC_MAINCK_8M_RC_HZ;

    case OSC_MAINCK_12M_RC:
        return OSC_MAINCK_12M_RC_HZ;

    case OSC_MAINCK_XTAL:
        return BOARD_FREQ_MAINCK_XTAL;

    case OSC_MAINCK_BYPASS:
        return BOARD_FREQ_MAINCK_BYPASS;
    }

    return 0;
}

static inline void osc_wait_ready(uint8_t id)
{
    while (!osc_is_ready(id)) {
        /* Do nothing */
    }
}