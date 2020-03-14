#include "timer.hpp"

volatile uint64_t g_ticks = 0;

extern "C" {
    void SysTick_Handler(void)
    {
        g_ticks++;
    }
}

Timer::Timer()
    : start(g_ticks) {}

void Timer::reset()
{
    start = g_ticks;
}

float Timer::elapsed() const
{
    return (g_ticks - start) / 1000.0f;
}

__attribute__((optimize("-O0")))
void Timer::delay(float time) const
{
    while (elapsed() < time) {}
}