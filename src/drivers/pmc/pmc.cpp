#include "pmc.hpp"

#include <sam.h>
#include <system_same70.h>

# define MAX_PERIPH_ID    63

/* brief Set the prescaler of the MCK. */
void pmc_mck_set_prescaler(uint32_t ul_pres)
{
    PMC->PMC_MCKR =
            (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}

/* Set the division of the MCK. */
void pmc_mck_set_division(uint32_t ul_div)
{
    switch (ul_div) {
        case 1:
            ul_div = PMC_MCKR_MDIV_EQ_PCK;
            break;
        case 2:
            ul_div = PMC_MCKR_MDIV_PCK_DIV2;
            break;
        case 3:
            ul_div = PMC_MCKR_MDIV_PCK_DIV3;
            break;
        case 4:
            ul_div = PMC_MCKR_MDIV_PCK_DIV4;
            break;
        default:
            ul_div = PMC_MCKR_MDIV_EQ_PCK;
            break;
    }
    PMC->PMC_MCKR =
            (PMC->PMC_MCKR & (~PMC_MCKR_MDIV_Msk)) | ul_div;
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}

/* Set the source of the MCK. */
void pmc_mck_set_source(uint32_t ul_source)
{
    PMC->PMC_MCKR =
            (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) | ul_source;
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));
}

/* Switch master clock source selection to slow clock. */
uint32_t pmc_switch_mck_to_sclk(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
            PMC_MCKR_CSS_SLOW_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch master clock source selection to main clock. */
uint32_t pmc_switch_mck_to_mainck(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
            PMC_MCKR_CSS_MAIN_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch master clock source selection to PLLA clock. */
uint32_t pmc_switch_mck_to_pllack(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
            PMC_MCKR_CSS_PLLA_CLK;

    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch master clock source selection to UPLL clock. */
uint32_t pmc_switch_mck_to_upllck(uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_PRES_Msk)) | ul_pres;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    PMC->PMC_MCKR = (PMC->PMC_MCKR & (~PMC_MCKR_CSS_Msk)) |
            PMC_MCKR_CSS_UPLL_CLK;
    for (ul_timeout = PMC_TIMEOUT; !(PMC->PMC_SR & PMC_SR_MCKRDY);
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch slow clock source selection to external 32k (Xtal or Bypass). */
void pmc_switch_sclk_to_32kxtal(uint32_t ul_bypass)
{
    /* Set Bypass mode if required */
    if (ul_bypass == 1) {
        SUPC->SUPC_MR |= SUPC_MR_KEY_PASSWD |
            SUPC_MR_OSCBYPASS;
    }

    SUPC->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_XTALSEL;
}

/* Check if the external 32k Xtal is ready. */
uint32_t pmc_osc_is_ready_32kxtal()
{
    return ((SUPC->SUPC_SR & SUPC_SR_OSCSEL)
            && (PMC->PMC_SR & PMC_SR_OSCSELS));
}

/* Switch main clock source selection to internal fast RC. */
void pmc_switch_mainck_to_fastrc(uint32_t ul_moscrcf)
{
    /* Enable Fast RC oscillator but DO NOT switch to RC now */
    PMC->CKGR_MOR |= (CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCRCEN);

    /* Wait the Fast RC to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));

    /* Change Fast RC oscillator frequency */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCF_Msk) |
            CKGR_MOR_KEY_PASSWD | ul_moscrcf;

    /* Wait the Fast RC to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));

    /* Switch to Fast RC */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCSEL) |
            CKGR_MOR_KEY_PASSWD;
}

/* Enable fast RC oscillator. */
void pmc_osc_enable_fastrc(uint32_t ul_rc)
{
    /* Enable Fast RC oscillator but DO NOT switch to RC */
    PMC->CKGR_MOR |= (CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCRCEN);
    /* Wait the Fast RC to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));

    /* Change Fast RC oscillator frequency */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCF_Msk) |
            CKGR_MOR_KEY_PASSWD | ul_rc;
    /* Wait the Fast RC to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCRCS));
}

/* Disable the internal fast RC. */
void pmc_osc_disable_fastrc()
{
    /* Disable Fast RC oscillator */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCEN &
                    ~CKGR_MOR_MOSCRCF_Msk)
                | CKGR_MOR_KEY_PASSWD;
}

/* Check if the main fastrc is ready. */
uint32_t pmc_osc_is_ready_fastrc()
{
    return (PMC->PMC_SR & PMC_SR_MOSCRCS);
}

/* Enable main XTAL oscillator. */
void pmc_osc_enable_main_xtal(uint32_t ul_xtal_startup_time)
{
    uint32_t mor = PMC->CKGR_MOR;
    mor &= ~(CKGR_MOR_MOSCXTBY|CKGR_MOR_MOSCXTEN);
    mor |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN |
            CKGR_MOR_MOSCXTST(ul_xtal_startup_time);
    PMC->CKGR_MOR = mor;
    /* Wait the main Xtal to stabilize */
    while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));
}

/* Bypass main XTAL. */
void pmc_osc_bypass_main_xtal()
{
    uint32_t mor = PMC->CKGR_MOR;
    mor &= ~(CKGR_MOR_MOSCXTBY|CKGR_MOR_MOSCXTEN);
    mor |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTBY;
    /* Enable Crystal oscillator but DO NOT switch now. Keep MOSCSEL to 0 */
    PMC->CKGR_MOR = mor;
    /* The MOSCXTS in PMC_SR is automatically set */
}

/* Disable the main Xtal. */
void pmc_osc_disable_main_xtal()
{
    uint32_t mor = PMC->CKGR_MOR;
    mor &= ~(CKGR_MOR_MOSCXTBY|CKGR_MOR_MOSCXTEN);
    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | mor;
}

/* Check if the main crystal is bypassed. */
uint32_t pmc_osc_is_bypassed_main_xtal()
{
    return (PMC->CKGR_MOR & CKGR_MOR_MOSCXTBY);
}

/* Check if the main crystal is ready. */
uint32_t pmc_osc_is_ready_main_xtal()
{
    return (PMC->PMC_SR & PMC_SR_MOSCXTS);
}

/* Switch main clock source selection to external Xtal/Bypass. */
void pmc_switch_mainck_to_xtal(uint32_t ul_bypass,
        uint32_t ul_xtal_startup_time)
{
    /* Enable Main Xtal oscillator */
    if (ul_bypass) {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
                CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTBY |
                CKGR_MOR_MOSCSEL;
    } else {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
                CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCXTEN |
                CKGR_MOR_MOSCXTST(ul_xtal_startup_time);
        /* Wait the Xtal to stabilize */
        while (!(PMC->PMC_SR & PMC_SR_MOSCXTS));

        PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCSEL;
    }
}

/* Disable the external Xtal. */
void pmc_osc_disable_xtal(uint32_t ul_bypass)
{
    /* Disable xtal oscillator */
    if (ul_bypass) {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
                CKGR_MOR_KEY_PASSWD;
    } else {
        PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTEN) |
                CKGR_MOR_KEY_PASSWD;
    }
}

/* Check if the MAINCK is ready. Depending on MOSCEL, MAINCK can be one of Xtal, bypass or internal RC. */
uint32_t pmc_osc_is_ready_mainck()
{
    return PMC->PMC_SR & PMC_SR_MOSCSELS;
}

/* Select Main Crystal or internal RC as main clock source. */
void pmc_mainck_osc_select(uint32_t ul_xtal_rc)
{
    uint32_t mor = PMC->CKGR_MOR;
    if (ul_xtal_rc) {
        mor |=  CKGR_MOR_MOSCSEL;
    } else {
        mor &= ~CKGR_MOR_MOSCSEL;
    }
    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | mor;
}

/* Enable PLLA clock. */
void pmc_enable_pllack(uint32_t mula, uint32_t pllacount, uint32_t diva)
{
    /* first disable the PLL to unlock the lock */
    pmc_disable_pllack();

    PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_DIVA(diva) |
            CKGR_PLLAR_PLLACOUNT(pllacount) | CKGR_PLLAR_MULA(mula);

    while ((PMC->PMC_SR & PMC_SR_LOCKA) == 0);
}

/* Disable PLLA clock. */
void pmc_disable_pllack()
{
    PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA(0);
}

/* Is PLLA locked? */
uint32_t pmc_is_locked_pllack()
{
    return (PMC->PMC_SR & PMC_SR_LOCKA);
}

/* Enable UPLL clock. */
void pmc_enable_upll_clock()
{
    PMC->CKGR_UCKR = CKGR_UCKR_UPLLCOUNT(3) | CKGR_UCKR_UPLLEN;

    /* Wait UTMI PLL Lock Status */
    while (!(PMC->PMC_SR & PMC_SR_LOCKU));
}

/* Disable UPLL clock. */
void pmc_disable_upll_clock()
{
    PMC->CKGR_UCKR &= ~CKGR_UCKR_UPLLEN;
}

/* Is UPLL locked? */
uint32_t pmc_is_locked_upll()
{
    return (PMC->PMC_SR & PMC_SR_LOCKU);
}

/* Enable the specified peripheral clock. */
uint32_t pmc_enable_periph_clk(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) {
        return 1;
    }

    if (ul_id < 32) {
        if ((PMC->PMC_PCSR0 & (1u << ul_id)) != (1u << ul_id)) {
            PMC->PMC_PCER0 = 1 << ul_id;
        }
    } else {
        ul_id -= 32;
        if ((PMC->PMC_PCSR1 & (1u << ul_id)) != (1u << ul_id)) {
            PMC->PMC_PCER1 = 1 << ul_id;
        }
    }

    return 0;
}

/* Disable the specified peripheral clock. */
uint32_t pmc_disable_periph_clk(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) {
        return 1;
    }

    if (ul_id < 32) {
        if ((PMC->PMC_PCSR0 & (1u << ul_id)) == (1u << ul_id)) {
            PMC->PMC_PCDR0 = 1 << ul_id;
        }
    } else {
        ul_id -= 32;
        if ((PMC->PMC_PCSR1 & (1u << ul_id)) == (1u << ul_id)) {
            PMC->PMC_PCDR1 = 1 << ul_id;
        }
    }
    return 0;
}

/* Enable all peripheral clocks. */
void pmc_enable_all_periph_clk()
{
    PMC->PMC_PCER0 = PMC_MASK_STATUS0;
    while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != PMC_MASK_STATUS0);

    PMC->PMC_PCER1 = PMC_MASK_STATUS1;
    while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != PMC_MASK_STATUS1);
}

/* Disable all peripheral clocks. */
void pmc_disable_all_periph_clk()
{
    PMC->PMC_PCDR0 = PMC_MASK_STATUS0;
    while ((PMC->PMC_PCSR0 & PMC_MASK_STATUS0) != 0);

    PMC->PMC_PCDR1 = PMC_MASK_STATUS1;
    while ((PMC->PMC_PCSR1 & PMC_MASK_STATUS1) != 0);
}

/* Check if the specified peripheral clock is enabled. */
uint32_t pmc_is_periph_clk_enabled(uint32_t ul_id)
{
    if (ul_id > MAX_PERIPH_ID) {
        return 0;
    }

    if (ul_id < 32) {
        if ((PMC->PMC_PCSR0 & (1u << ul_id))) {
            return 1;
        } else {
            return 0;
        }
    } else {
        ul_id -= 32;
        if ((PMC->PMC_PCSR1 & (1u << ul_id))) {
            return 1;
        } else {
            return 0;
        }
    }
}

/* Set the prescaler for the specified programmable clock. */
void pmc_pck_set_prescaler(uint32_t ul_id, uint32_t ul_pres)
{
    PMC->PMC_PCK[ul_id] =
            (PMC->PMC_PCK[ul_id] & ~PMC_PCK_PRES_Msk) | ul_pres;
    while ((PMC->PMC_SCER & (PMC_SCER_PCK0 << ul_id))
            && !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)));
}

/* Set the source oscillator for the specified programmable clock. */
void pmc_pck_set_source(uint32_t ul_id, uint32_t ul_source)
{
    PMC->PMC_PCK[ul_id] =
            (PMC->PMC_PCK[ul_id] & ~PMC_PCK_CSS_Msk) | ul_source;
    while ((PMC->PMC_SCER & (PMC_SCER_PCK0 << ul_id))
            && !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)));
}

/* Switch programmable clock source selection to slow clock. */
uint32_t pmc_switch_pck_to_sclk(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_SLOW_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
    !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch programmable clock source selection to main clock. */
uint32_t pmc_switch_pck_to_mainck(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_MAIN_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
    !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch programmable clock source selection to PLLA clock. */
uint32_t pmc_switch_pck_to_pllack(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_PLLA_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
    !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch programmable clock source selection to UPLL clock. */
uint32_t pmc_switch_pck_to_upllck(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_UPLL_CLK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
            !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id));
            --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Switch programmable clock source selection to mck. */
uint32_t pmc_switch_pck_to_mck(uint32_t ul_id, uint32_t ul_pres)
{
    uint32_t ul_timeout;

    PMC->PMC_PCK[ul_id] = PMC_PCK_CSS_MCK | ul_pres;
    for (ul_timeout = PMC_TIMEOUT;
    !(PMC->PMC_SR & (PMC_SR_PCKRDY0 << ul_id)); --ul_timeout) {
        if (ul_timeout == 0) {
            return 1;
        }
    }

    return 0;
}

/* Enable the specified programmable clock. */
void pmc_enable_pck(uint32_t ul_id)
{
    PMC->PMC_SCER = PMC_SCER_PCK0 << ul_id;
}

/* Disable the specified programmable clock. */
void pmc_disable_pck(uint32_t ul_id)
{
    PMC->PMC_SCDR = PMC_SCER_PCK0 << ul_id;
}

/* Enable all programmable clocks. */
void pmc_enable_all_pck()
{
    PMC->PMC_SCER = PMC_SCER_PCK0 | PMC_SCER_PCK1 | PMC_SCER_PCK2;
}

/* Disable all programmable clocks. */
void pmc_disable_all_pck()
{
    PMC->PMC_SCDR = PMC_SCDR_PCK0 | PMC_SCDR_PCK1 | PMC_SCDR_PCK2;
}

/* Check if the specified programmable clock is enabled. */
uint32_t pmc_is_pck_enabled(uint32_t ul_id)
{
    if (ul_id > 2) {
        return 0;
    }

    return (PMC->PMC_SCSR & (PMC_SCSR_PCK0 << ul_id));
}

/* Switch UDP (USB) clock source selection to PLLA clock. */
void pmc_switch_udpck_to_pllack(uint32_t ul_usbdiv)
{
    PMC->PMC_USB = PMC_USB_USBDIV(ul_usbdiv);
}

/* Switch UDP (USB) clock source selection to UPLL clock. */
void pmc_switch_udpck_to_upllck(uint32_t ul_usbdiv)
{
    PMC->PMC_USB = PMC_USB_USBS | PMC_USB_USBDIV(ul_usbdiv);
}

/* Enable UDP (USB) clock. */
void pmc_enable_udpck()
{
    PMC->PMC_SCER = PMC_SCER_USBCLK;
}

/* Disable UDP (USB) clock. */
void pmc_disable_udpck()
{
    PMC->PMC_SCDR = PMC_SCDR_USBCLK;
}

/* Enable PMC interrupts. */
void pmc_enable_interrupt(uint32_t ul_sources)
{
    PMC->PMC_IER = ul_sources;
}

/* Disable PMC interrupts. */
void pmc_disable_interrupt(uint32_t ul_sources)
{
    PMC->PMC_IDR = ul_sources;
}

/* Get PMC interrupt mask. */
uint32_t pmc_get_interrupt_mask()
{
    return PMC->PMC_IMR;
}

/* Get current status. */
uint32_t pmc_get_status()
{
    return PMC->PMC_SR;
}

/* Set the wake-up inputs for fast startup mode registers (event generation). */
void pmc_set_fast_startup_input(uint32_t ul_inputs)
{
    ul_inputs &= PMC_FAST_STARTUP_Msk;
    PMC->PMC_FSMR |= ul_inputs;
}

/* Clear the wake-up inputs for fast startup mode registers (remove event generation). */
void pmc_clr_fast_startup_input(uint32_t ul_inputs)
{
    ul_inputs &= PMC_FAST_STARTUP_Msk;
    PMC->PMC_FSMR &= ~ul_inputs;
}

/*
    * Enable Sleep Mode.
    * Enter condition: (WFE or WFI) + (SLEEPDEEP bit = 0) + (LPM bit = 0)
    *
    * \param uc_type 0 for wait for interrupt, 1 for wait for event.
    * \note For SAM4S, SAM4C, SAM4CM, SAM4CP, SAMV71 and SAM4E series,
    * since only WFI is effective, uc_type = 1 will be treated as uc_type = 0.
    */
void pmc_enable_sleepmode(uint8_t uc_type)
{
    SCB->SCR &= (uint32_t) ~ SCB_SCR_SLEEPDEEP_Msk; // Deep sleep

    (void)(uc_type);
    __DSB();
    __WFI();
}

static uint32_t ul_flash_in_wait_mode = PMC_WAIT_MODE_FLASH_DEEP_POWERDOWN;
/**
 * \brief Set the embedded flash state in wait mode
 *
 * \param ul_flash_state PMC_WAIT_MODE_FLASH_STANDBY flash in standby mode,
 * PMC_WAIT_MODE_FLASH_DEEP_POWERDOWN flash in deep power down mode.
 */
void pmc_set_flash_in_wait_mode(uint32_t ul_flash_state)
{
    ul_flash_in_wait_mode = ul_flash_state;
}

/**
 * \brief Enable Wait Mode. Enter condition: (WAITMODE bit = 1) + FLPM
 *
 * \note In this function, FLPM will retain, WAITMODE bit will be set,
 * Generally, this function will be called by pmc_sleep() in order to
 * complete all sequence entering wait mode.
 * See \ref pmc_sleep() for entering different sleep modes.
 */
void pmc_enable_waitmode()
{
    uint32_t i;

    /* Flash in wait mode */
    i = PMC->PMC_FSMR;
    i &= ~PMC_FSMR_FLPM_Msk;
    (void)ul_flash_in_wait_mode;
    i |= PMC_WAIT_MODE_FLASH_IDLE;
    PMC->PMC_FSMR = i;

    /* Set the WAITMODE bit = 1 */
    PMC->CKGR_MOR |= CKGR_MOR_KEY_PASSWD | CKGR_MOR_WAITMODE;

    /* Waiting for Master Clock Ready MCKRDY = 1 */
    while (!(PMC->PMC_SR & PMC_SR_MCKRDY));

    /* Waiting for MOSCRCEN bit cleared is strongly recommended
    * to ensure that the core will not execute undesired instructions
    */
    for (i = 0; i < 500; i++) {
        __NOP();
    }
    while (!(PMC->CKGR_MOR & CKGR_MOR_MOSCRCEN));

    /* Restore Flash in idle mode */
    i = PMC->PMC_FSMR;
    i &= ~PMC_FSMR_FLPM_Msk;
    i |= PMC_WAIT_MODE_FLASH_IDLE;
    PMC->PMC_FSMR = i;
}

/**
 * \brief Enable Backup Mode. Enter condition: WFE/(VROFF bit = 1) +
 * (SLEEPDEEP bit = 1)
 */
void pmc_enable_backupmode()
{
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    SUPC->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_VROFF_STOP_VREG;
    uint32_t ul_dummy = SUPC->SUPC_MR;
    (void)(ul_dummy);
    __DSB();
    __WFE();
    __WFI();
}

/**
 * \brief Enable Clock Failure Detector.
 */
void pmc_enable_clock_failure_detector()
{
    uint32_t ul_reg = PMC->CKGR_MOR;

    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | CKGR_MOR_CFDEN | ul_reg;
}

/**
 * \brief Disable Clock Failure Detector.
 */
void pmc_disable_clock_failure_detector()
{
    uint32_t ul_reg = PMC->CKGR_MOR & (~CKGR_MOR_CFDEN);

    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | ul_reg;
}

/**
 * \brief Enable Slow Crystal Oscillator Frequency Monitoring.
 */
void pmc_enable_sclk_osc_freq_monitor()
{
    uint32_t ul_reg = PMC->CKGR_MOR;

    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | CKGR_MOR_XT32KFME | ul_reg;
}

/**
 * \brief Disable Slow Crystal Oscillator Frequency Monitoring.
 */
void pmc_disable_sclk_osc_freq_monitor()
{
    uint32_t ul_reg = PMC->CKGR_MOR & (~CKGR_MOR_XT32KFME);

    PMC->CKGR_MOR = CKGR_MOR_KEY_PASSWD | ul_reg;
}

/**
 * \brief Enable or disable write protect of PMC registers.
 *
 * \param ul_enable 1 to enable, 0 to disable.
 */
void pmc_set_writeprotect(uint32_t ul_enable)
{
    if (ul_enable) {
        PMC->PMC_WPMR = PMC_WPMR_WPKEY_PASSWD | PMC_WPMR_WPEN;
    } else {
        PMC->PMC_WPMR = PMC_WPMR_WPKEY_PASSWD;
    }
}

/**
 * \brief Return write protect status.
 *
 * \return Return write protect status.
 */
uint32_t pmc_get_writeprotect_status()
{
    return PMC->PMC_WPSR;
}

/**
 * \brief Enable the specified peripheral clock.
 *
 * \note The ID must NOT be shifted (i.e., 1 << ID_xxx).
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Fail.
 */
uint32_t pmc_enable_sleepwalking(uint32_t ul_id)
{
    uint32_t temp;

    if ((7 <= ul_id) && (ul_id<= 29)) {
        temp = pmc_get_active_status0();
        if (temp & (1 << ul_id)) {
            return 1;
        }
        PMC->PMC_SLPWK_ER0 = 1 << ul_id;
        temp = pmc_get_active_status0();
        if (temp & (1 << ul_id)) {
            pmc_disable_sleepwalking(ul_id);
            return 1;
        }
        return 0;
    }
    else if ((32 <= ul_id) && (ul_id<= 60)) {
        ul_id -= 32;
        temp = pmc_get_active_status1();
        if (temp & (1 << ul_id)) {
            return 1;
        }
        PMC->PMC_SLPWK_ER1 = 1 << ul_id;
        temp = pmc_get_active_status1();
        if (temp & (1 << ul_id)) {
            pmc_disable_sleepwalking(ul_id);
            return 1;
        }
        return 0;
    }
    else {
        return 1;
    }
}

/**
 * \brief Disable the sleepwalking of specified peripheral.
 *
 * \note The ID must NOT be shifted (i.e., 1 << ID_xxx).
 *
 * \param ul_id Peripheral ID (ID_xxx).
 *
 * \retval 0 Success.
 * \retval 1 Invalid parameter.
 */
uint32_t pmc_disable_sleepwalking(uint32_t ul_id)
{
    if ((7 <= ul_id) && (ul_id<= 29)) {
        PMC->PMC_SLPWK_DR0 = 1 << ul_id;
        return 0;
    }
    else if ((32 <= ul_id) && (ul_id<= 60)) {
        ul_id -= 32;
        PMC->PMC_SLPWK_DR1 = 1 << ul_id;
        return 0;
    }
    else {
        return 1;
    }
}

/**
 * \brief Return peripheral sleepwalking enable status.
 *
 * \return the status register value.
 */
uint32_t pmc_get_sleepwalking_status0()
{
    return PMC->PMC_SLPWK_SR0;
}

/**
 * \brief Return peripheral active status.
 *
 * \return the status register value.
 */
uint32_t pmc_get_active_status0()
{
    return PMC->PMC_SLPWK_ASR0;
}

/**
 * \brief Return peripheral sleepwalking enable status.
 *
 * \return the status register value.
 */
uint32_t pmc_get_sleepwalking_status1()
{
    return PMC->PMC_SLPWK_SR1;
}

/**
 * \brief Return peripheral active status.
 *
 * \return the status register value.
 */
uint32_t pmc_get_active_status1()
{
    return PMC->PMC_SLPWK_ASR1;
}