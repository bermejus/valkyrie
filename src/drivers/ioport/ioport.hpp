#pragma once

#include <sam.h>
#include "../pmc/pmc.hpp"

/* IOPORT pin directions */
enum ioport_direction {
    IOPORT_DIR_INPUT,	/* IOPORT input direction */
    IOPORT_DIR_OUTPUT	/* IOPORT output direction */
};

/* IOPORT levels */
enum ioport_value {
    IOPORT_PIN_LEVEL_LOW,	/* IOPORT pin value low */
    IOPORT_PIN_LEVEL_HIGH	/* IOPORT pin value high */
};

/* IOPORT edge sense modes */
enum ioport_sense {
    IOPORT_SENSE_BOTHEDGES,	/* IOPORT sense both rising and falling edges */
    IOPORT_SENSE_FALLING,	/* IOPORT sense falling edges */
    IOPORT_SENSE_RISING,	/* IOPORT sense rising edges */
    IOPORT_SENSE_LEVEL_LOW,	/* IOPORT sense low level */
    IOPORT_SENSE_LEVEL_HIGH	/* IOPORT sense high level */
};

#define IOPORT_CREATE_PIN(port, pin) ((IOPORT_ ## port) * 32 + (pin))
#define IOPORT_BASE_ADDRESS (uintptr_t)PIOA
#define IOPORT_PIO_OFFSET	((uintptr_t)PIOB - (uintptr_t)PIOA)

#define IOPORT_PIOA     0
#define IOPORT_PIOB     1
#define IOPORT_PIOC     2
#define IOPORT_PIOD     3
#define IOPORT_PIOE     4
#define IOPORT_PIOF     5

/* IOPORT mode bit definitions */
#define IOPORT_MODE_MUX_MASK		(0x7 << 0)	/* MUX bits mask */
#define IOPORT_MODE_MUX_BIT0		(1 << 0)	/* MUX BIT0 mask */

#define IOPORT_MODE_MUX_BIT1		(1 << 1)	/* MUX BIT1 mask */

#define IOPORT_MODE_MUX_A			(0 << 0)	/* MUX function A */
#define IOPORT_MODE_MUX_B			(1 << 0)	/* MUX function B */
#define IOPORT_MODE_MUX_C			(2 << 0)	/* MUX function C */
#define IOPORT_MODE_MUX_D			(3 << 0)	/* MUX function D */

#define IOPORT_MODE_PULLUP			(1 << 3)	/* Pull-up */
#define IOPORT_MODE_PULLDOWN		(1 << 4)	/* Pull-down */

#define IOPORT_MODE_OPEN_DRAIN		(1 << 5)	/* Open drain */

#define IOPORT_MODE_GLITCH_FILTER	(1 << 6)	/* Glitch filter */
#define IOPORT_MODE_DEBOUNCE		(1 << 7)	/* Input debounce */

typedef uint32_t ioport_mode_t;
typedef uint32_t ioport_pin_t;
typedef uint32_t ioport_port_t;
typedef uint32_t ioport_port_mask_t;

/*
* Initializes the IOPORT service, ready for use.
*
* This function must be called before using any other functions in the IOPORT
* service.
*/
static inline void ioport_init()
{
#ifdef ID_PIOA
    pmc_enable_periph_clk(ID_PIOA);
#endif
#ifdef ID_PIOB
    pmc_enable_periph_clk(ID_PIOB);
#endif
#ifdef ID_PIOC
    pmc_enable_periph_clk(ID_PIOC);
#endif
#ifdef ID_PIOD
    pmc_enable_periph_clk(ID_PIOD);
#endif
#ifdef ID_PIOE
    pmc_enable_periph_clk(ID_PIOE);
#endif
#ifdef ID_PIOF
    pmc_enable_periph_clk(ID_PIOF);
#endif
}

/* Convert a pin ID into a its port ID. */
static inline ioport_port_t ioport_pin_to_port_id(ioport_pin_t pin)
{
    return pin >> 5;
}

/* Convert a pin ID into a bitmask mask for the given pin on its port. */
static inline ioport_port_mask_t ioport_pin_to_mask(ioport_pin_t pin)
{
    return 1U << (pin & 0x1F);
}

static inline Pio* ioport_port_to_base(ioport_port_t port)
{
    return (Pio*)((uintptr_t)IOPORT_BASE_ADDRESS + (IOPORT_PIO_OFFSET * port));
}

static inline Pio* ioport_pin_to_base(ioport_pin_t pin)
{
    return ioport_port_to_base(ioport_pin_to_port_id(pin));
}

/*
* Enable multiple pins in a single IOPORT port.
* 
* \param port IOPORT port to enable
* \param mask Mask of pins within the port to enable
*/
static inline void ioport_enable_port(ioport_port_t port, ioport_port_mask_t mask)
{
    ioport_port_to_base(port)->PIO_PER = mask;
}

/*
* Disable multiple pins in a single IOPORT port.
* 
* \param port IOPORT port to disable
* \param mask Mask of pins within the port to disable
*/
static inline void ioport_disable_port(ioport_port_t port, ioport_port_mask_t mask)
{
    ioport_port_to_base(port)->PIO_PDR = mask;
}

/*
* Enable an IOPORT pin, based on a pin created with IOPORT_CREATE_PIN().
*/
static inline void ioport_enable_pin(ioport_pin_t pin)
{
    ioport_enable_port(ioport_pin_to_port_id(pin),
                    ioport_pin_to_mask(pin));
}

/*
* Disable an IOPORT pin, based on a pin created with IOPORT_CREATE_PIN().
*/
static inline void ioport_disable_pin(ioport_pin_t pin)
{
    ioport_disable_port(ioport_pin_to_port_id(pin),
                        ioport_pin_to_mask(pin));
}

/*
* Set multiple pin modes in a single IOPORT port, such as pull-up,
* pull-down, etc. configuration.
*
* \param port IOPORT port to configure
* \param mask Pin mask of pins to configure
* \param mode Mode masks to configure for the specified pins (\ref
* ioport_modes)
*/
static inline void ioport_set_port_mode(ioport_port_t port,
        ioport_port_mask_t mask, ioport_mode_t mode)
{
    Pio* base = ioport_port_to_base(port);

    if (mode & IOPORT_MODE_PULLUP)
        base->PIO_PUER = mask;
    else
        base->PIO_PUDR = mask;
    
    if (mode & IOPORT_MODE_PULLDOWN)
        base->PIO_PPDER = mask;
    else
        base->PIO_PPDDR = mask;
    
    if (mode & IOPORT_MODE_OPEN_DRAIN)
        base->PIO_MDER = mask;
    else
        base->PIO_MDDR = mask;
    
    if (mode & (IOPORT_MODE_GLITCH_FILTER | IOPORT_MODE_DEBOUNCE))
        base->PIO_IFER = mask;
    else
        base->PIO_IFDR = mask;
    
    if (mode & IOPORT_MODE_DEBOUNCE)
        base->PIO_IFSCER = mask;
    else
        base->PIO_IFSCDR = mask;
    
    if (mode & IOPORT_MODE_MUX_BIT0)
        base->PIO_ABCDSR[0] |= mask;
    else
        base->PIO_ABCDSR[0] &= ~mask;
    
    if (mode & IOPORT_MODE_MUX_BIT1)
        base->PIO_ABCDSR[1] |= mask;
    else
        base->PIO_ABCDSR[1] &= ~mask;
}

/*
* Set pin mode for one single IOPORT pin.
*
* \param pin IOPORT pin to configure
* \param mode Mode masks to configure for the specified pin (\ref ioport_modes)
*/
static inline void ioport_set_pin_mode(ioport_pin_t pin, ioport_mode_t mode)
{
    ioport_set_port_mode(ioport_pin_to_port_id(pin),
                        ioport_pin_to_mask(pin), mode);
}

/*
* Reset multiple pin modes in a specified IOPORT port to defaults.
*
* \param port IOPORT port to configure
* \param mask Mask of pins whose mode configuration is to be reset
*/
static inline void ioport_reset_port_mode(ioport_port_t port,
        ioport_port_mask_t mask)
{
    ioport_set_port_mode(port, mask, 0);
}

/*
* Reset pin mode configuration for a single IOPORT pin
*
* \param pin IOPORT pin to configure
*/
static inline void ioport_reset_pin_mode(ioport_pin_t pin)
{
    ioport_set_pin_mode(pin, 0);
}

/*
* Set I/O direction for a group of pins in a single IOPORT.
*
* \param port IOPORT port to configure
* \param mask Pin mask of pins to configure
* \param dir Direction to set for the specified pins (\ref ioport_direction)
*/
static inline void ioport_set_port_dir(ioport_port_t port,
        ioport_port_mask_t mask, enum ioport_direction dir)
{
    Pio* base = ioport_port_to_base(port);

    if (dir == IOPORT_DIR_OUTPUT)
        base->PIO_OER = mask;
    else if (dir == IOPORT_DIR_INPUT)
        base->PIO_ODR = mask;
    
    base->PIO_OWER = mask;
}

/*
* Set direction for a single IOPORT pin.
*
* \param pin IOPORT pin to configure
* \param dir Direction to set for the specified pin (\ref ioport_direction)
*/
static inline void ioport_set_pin_dir(ioport_pin_t pin,
        enum ioport_direction dir)
{
    Pio* base = ioport_pin_to_base(pin);

    if (dir == IOPORT_DIR_OUTPUT)
        base->PIO_OER = ioport_pin_to_mask(pin);
    else if (dir == IOPORT_DIR_INPUT)
        base->PIO_ODR = ioport_pin_to_mask(pin);

    base->PIO_OWER = ioport_pin_to_mask(pin);
}

/*
* Set an IOPORT pin to a specified logical value.
*
* \param pin IOPORT pin to configure
* \param level Logical value of the pin
*/
static inline void ioport_set_pin_level(ioport_pin_t pin, bool level)
{
    Pio* base = ioport_pin_to_base(pin);

    if (level)
        base->PIO_SODR = ioport_pin_to_mask(pin);
    else
        base->PIO_CODR = ioport_pin_to_mask(pin);
}

/*
* Set a group of IOPORT pins in a single port to a specified logical
* value.
*
* \param port IOPORT port to write to
* \param mask Pin mask of pins to modify
* \param level Level of the pins to be modified
*/
static inline void ioport_set_port_level(ioport_port_t port,
        ioport_port_mask_t mask, enum ioport_value level)
{
    Pio* base = ioport_port_to_base(port);

    if (level)
        base->PIO_SODR = mask;
    else
        base->PIO_CODR = mask;
}

/*
* Get current value of an IOPORT pin, which has been configured as an
* input.
*
* \param pin IOPORT pin to read
* \return Current logical value of the specified pin
*/
static inline bool ioport_get_pin_level(ioport_pin_t pin)
{
    return ioport_pin_to_base(pin)->PIO_PDSR & ioport_pin_to_mask(pin);
}

/*
* Get current value of several IOPORT pins in a single port, which have
* been configured as an inputs.
*
* \param port IOPORT port to read
* \param mask Pin mask of pins to read
* \return Logical levels of the specified pins from the read port, returned as
* a mask.
*/
static inline ioport_port_mask_t ioport_get_port_level(ioport_pin_t port,
        ioport_port_mask_t mask)
{
    return ioport_port_to_base(port)->PIO_PDSR & mask;
}

/*
* Toggle the value of an IOPORT pin, which has previously configured as
* an output.
*
* \param pin IOPORT pin to toggle
*/
static inline void ioport_toggle_pin_level(ioport_pin_t pin)
{
    Pio* port = ioport_pin_to_base(pin);
    ioport_port_mask_t mask = ioport_pin_to_mask(pin);

    if (port->PIO_PDSR & ioport_pin_to_mask(pin))
        port->PIO_CODR = mask;
    else
        port->PIO_SODR = mask;
}

/*
* Toggle the values of several IOPORT pins located in a single port.
*
* \param port IOPORT port to modify
* \param mask Pin mask of pins to toggle
*/
static inline void ioport_toggle_port_level(ioport_port_t port,
        ioport_port_mask_t mask)
{
    ioport_port_to_base(port)->PIO_ODSR ^= mask;
}

/*
* Set the pin sense mode of a multiple IOPORT pins on a single port.
*
* \param port IOPORT port to configure
* \param mask Bitmask if pins whose edge sense is to be configured
* \param pin_sense Edge to sense for the pins (\ref ioport_sense)
*/
static inline void ioport_set_port_sense_mode(ioport_port_t port,
        ioport_port_mask_t mask,
        enum ioport_sense pin_sense)
{
    Pio *base = ioport_port_to_base(port);
    /*   AIMMR    ELSR    FRLHSR
    *       0       X         X    IOPORT_SENSE_BOTHEDGES (Default)
    *       1       0         0    IOPORT_SENSE_FALLING
    *       1       0         1    IOPORT_SENSE_RISING
    *       1       1         0    IOPORT_SENSE_LEVEL_LOW
    *       1       1         1    IOPORT_SENSE_LEVEL_HIGH
    */
    switch(pin_sense) {
    case IOPORT_SENSE_LEVEL_LOW:
        base->PIO_LSR = mask;
        base->PIO_FELLSR = mask;
        break;
    case IOPORT_SENSE_LEVEL_HIGH:
        base->PIO_LSR = mask;
        base->PIO_REHLSR = mask;
        break;
    case IOPORT_SENSE_FALLING:
        base->PIO_ESR = mask;
        base->PIO_FELLSR = mask;
        break;
    case IOPORT_SENSE_RISING:
        base->PIO_ESR = mask;
        base->PIO_REHLSR = mask;
        break;
    default:
        base->PIO_AIMDR = mask;
        return;
    }
    base->PIO_AIMER = mask;
}

/*
* Set the pin sense mode of a single IOPORT pin.
*
* \param pin IOPORT pin to configure
* \param pin_sense Edge to sense for the pin (\ref ioport_sense)
*/
static inline void ioport_set_pin_sense_mode(ioport_pin_t pin,
        enum ioport_sense pin_sense)
{
    ioport_set_port_sense_mode(ioport_pin_to_port_id(pin),
                            ioport_pin_to_mask(pin),
                            pin_sense);
}