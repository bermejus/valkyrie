#pragma once

#include <sam.h>
#include "../irq/irq.hpp"

/* Address for ARM CPACR */
#define ADDR_CPACR 0xE000ED88

/* CPACR Register */
#define REG_CPACR  (*((volatile uint32_t *)ADDR_CPACR))

/*
    * Enable FPU
    */
static inline void fpu_enable(void)
{
    irqflags_t flags;
    flags = cpu_irq_save();
    REG_CPACR |=  (0xFu << 20);
    __DSB();
    __ISB();
    cpu_irq_restore(flags);
}

/*
    * Disable FPU
    */
static inline void fpu_disable(void)
{
    irqflags_t flags;
    flags = cpu_irq_save();
    REG_CPACR &= ~(0xFu << 20);
    __DSB();
    __ISB();
    cpu_irq_restore(flags);
}

/*
    * Check if FPU is enabled
    *
    * \return Return ture if FPU is enabled, otherwise return false.
    */
static inline bool fpu_is_enabled(void)
{
    return (REG_CPACR & (0xFu << 20));
}