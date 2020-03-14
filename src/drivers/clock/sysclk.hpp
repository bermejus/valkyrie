#pragma once

#include <cstdint>
#include <conf_clock.h>
#include "osc.hpp"
#include "pll.hpp"

#ifndef CONFIG_SYSCLK_SOURCE
# define CONFIG_SYSCLK_SOURCE   SYSCLK_SRC_MAINCK_4M_RC
#endif

#ifndef CONFIG_SYSCLK_PRES
# define CONFIG_SYSCLK_PRES  0
#endif

// Master Clock Sources (MCK)
#define SYSCLK_SRC_SLCK_RC         0 //!< Internal 32kHz RC oscillator as master source clock
#define SYSCLK_SRC_SLCK_XTAL       1 //!< External 32kHz crystal oscillator as master source clock
#define SYSCLK_SRC_SLCK_BYPASS     2 //!< External 32kHz bypass oscillator as master source clock
#define SYSCLK_SRC_MAINCK_4M_RC    3 //!< Internal 4MHz RC oscillator as master source clock
#define SYSCLK_SRC_MAINCK_8M_RC    4 //!< Internal 8MHz RC oscillator as master source clock
#define SYSCLK_SRC_MAINCK_12M_RC   5 //!< Internal 12MHz RC oscillator as master source clock
#define SYSCLK_SRC_MAINCK_XTAL     6 //!< External crystal oscillator as master source clock
#define SYSCLK_SRC_MAINCK_BYPASS   7 //!< External bypass oscillator as master source clock
#define SYSCLK_SRC_PLLACK          8 //!< Use PLLACK as master source clock
#define SYSCLK_SRC_UPLLCK          9       //!< Use UPLLCK as master source clock

// Master Clock Prescalers (MCK)
#define SYSCLK_PRES_1   PMC_MCKR_PRES_CLK_1  //!< Set master clock prescaler to 1
#define SYSCLK_PRES_2   PMC_MCKR_PRES_CLK_2  //!< Set master clock prescaler to 2
#define SYSCLK_PRES_4   PMC_MCKR_PRES_CLK_4  //!< Set master clock prescaler to 4
#define SYSCLK_PRES_8   PMC_MCKR_PRES_CLK_8  //!< Set master clock prescaler to 8
#define SYSCLK_PRES_16  PMC_MCKR_PRES_CLK_16 //!< Set master clock prescaler to 16
#define SYSCLK_PRES_32  PMC_MCKR_PRES_CLK_32 //!< Set master clock prescaler to 32
#define SYSCLK_PRES_64  PMC_MCKR_PRES_CLK_64 //!< Set master clock prescaler to 64
#define SYSCLK_PRES_3   PMC_MCKR_PRES_CLK_3  //!< Set master clock prescaler to 3

// Master Clock Division (MCK)
#define SYSCLK_DIV_1   PMC_MCKR_MDIV_EQ_PCK  //!< Set master clock division to 1
#define SYSCLK_DIV_2   PMC_MCKR_MDIV_PCK_DIV2  //!< Set master clock division to 2
#define SYSCLK_DIV_4   PMC_MCKR_MDIV_PCK_DIV4  //!< Set master clock division to 4
#define SYSCLK_DIV_3   PMC_MCKR_MDIV_PCK_DIV3  //!< Set master clock division to 3

// USB Clock Sources
#define USBCLK_SRC_PLL0       0     //!< Use PLLA
#define USBCLK_SRC_UPLL       1     //!< Use UPLL

/**
 * \brief Return the current rate in Hz of the main system clock
 *
 * \todo This function assumes that the main clock source never changes
 * once it's been set up, and that PLL0 always runs at the compile-time
 * configured default rate. While this is probably the most common
 * configuration, which we want to support as a special case for
 * performance reasons, we will at some point need to support more
 * dynamic setups as well.
 */
#if (defined CONFIG_SYSCLK_DEFAULT_RETURNS_SLOW_OSC)
extern uint32_t sysclk_initialized;
#endif
static inline uint32_t sysclk_get_main_hz(void)
{
#if (defined CONFIG_SYSCLK_DEFAULT_RETURNS_SLOW_OSC)
    if (!sysclk_initialized) {
        return OSC_MAINCK_4M_RC_HZ;
    }
#endif

    /* Config system clock setting */
    if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_SLCK_RC) {
        return OSC_SLCK_32K_RC_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_SLCK_XTAL) {
        return OSC_SLCK_32K_XTAL_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_SLCK_BYPASS) {
        return OSC_SLCK_32K_BYPASS_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_MAINCK_4M_RC) {
        return OSC_MAINCK_4M_RC_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_MAINCK_8M_RC) {
        return OSC_MAINCK_8M_RC_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_MAINCK_12M_RC) {
        return OSC_MAINCK_12M_RC_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_MAINCK_XTAL) {
        return OSC_MAINCK_XTAL_HZ;
    } else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_MAINCK_BYPASS) {
        return OSC_MAINCK_BYPASS_HZ;
    }
#ifdef CONFIG_PLL0_SOURCE
    else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_PLLACK) {
        return pll_get_default_rate(0);
    }
#endif

#ifdef CONFIG_PLL1_SOURCE
    else if (CONFIG_SYSCLK_SOURCE == SYSCLK_SRC_UPLLCK) {
        return PLL_UPLL_HZ;
    }
#endif
    else {
        /* unhandled_case(CONFIG_SYSCLK_SOURCE); */
        return 0;
    }
}

/**
 * \brief Return the current rate in Hz of the CPU clock
 *
 * \todo This function assumes that the CPU always runs at the system
 * clock frequency. We want to support at least two more scenarios:
 * Fixed CPU/bus clock dividers (config symbols) and dynamic CPU/bus
 * clock dividers (which may change at run time). Ditto for all the bus
 * clocks.
 *
 * \return Frequency of the CPU clock, in Hz.
 */
static inline uint32_t sysclk_get_cpu_hz()
{
    /* CONFIG_SYSCLK_PRES is the register value for setting the expected */
    /* prescaler, not an immediate value. */
    return sysclk_get_main_hz() /
        ((CONFIG_SYSCLK_PRES == SYSCLK_PRES_3) ? 3 :
            (1 << (CONFIG_SYSCLK_PRES >> PMC_MCKR_PRES_Pos)));
}

/**
 * \brief Retrieves the current rate in Hz of the peripheral clocks.
 *
 * \return Frequency of the peripheral clocks, in Hz.
 */
static inline uint32_t sysclk_get_peripheral_hz()
{
    /* CONFIG_SYSCLK_PRES is the register value for setting the expected */
    /* prescaler, not an immediate value. */
    return sysclk_get_main_hz() /
        (((CONFIG_SYSCLK_PRES == SYSCLK_PRES_3) ? 3 : (1 << (CONFIG_SYSCLK_PRES >> PMC_MCKR_PRES_Pos))) * CONFIG_SYSCLK_DIV);
}

/**
 * \brief Retrieves the current rate in Hz of the Peripheral Bus clock attached
 *        to the specified peripheral.
 *
 * \param module Pointer to the module's base address.
 *
 * \return Frequency of the bus attached to the specified peripheral, in Hz.
 */
static inline uint32_t sysclk_get_peripheral_bus_hz(const volatile void *module)
{
    (void)(module);
    return sysclk_get_peripheral_hz();
}
//@}

//! \name Enabling and disabling synchronous clocks
//@{

/**
 * \brief Enable a peripheral's clock.
 *
 * \param ul_id Id (number) of the peripheral clock.
 */
static inline void sysclk_enable_peripheral_clock(uint32_t ul_id)
{
    pmc_enable_periph_clk(ul_id);
}

/**
 * \brief Disable a peripheral's clock.
 *
 * \param ul_id Id (number) of the peripheral clock.
 */
static inline void sysclk_disable_peripheral_clock(uint32_t ul_id)
{
    pmc_disable_periph_clk(ul_id);
}

//@}

//! \name System Clock Source and Prescaler configuration
//@{

extern void sysclk_set_prescalers(uint32_t ul_pres);
extern void sysclk_set_source(uint32_t ul_src);

//@}

extern void sysclk_enable_usb(void);
extern void sysclk_disable_usb(void);

extern void sysclk_init(void);