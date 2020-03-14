#include "delay.hpp"

__attribute__((section(".ramfunc")))
__attribute__((optimize("s")))
void portable_delay_cycles(unsigned long cycles)
{
    (void)cycles;

    __asm volatile (
        "loop: DMB	\n"
        "SUBS R0, R0, #1  \n"
        "BNE.N loop         "
    );
}