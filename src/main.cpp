#include <sam.h>
#include "../external/same70/init.hpp"
#include "drivers/clock/sysclk.hpp"
#include "drivers/led/led.hpp"
#include "utils/timer.hpp"

void init()
{
    SystemCoreClockUpdate();
    sysclk_init();
    board_init();

    SysTick_Config(SystemCoreClock / 1000);
    NVIC_SetPriority(SysTick_IRQn, 0x0);

    Led::init();
}

int main()
{
    init();

    Timer timer;
    timer.delay(1.0f);
    Led::g_on();


    return 0;
}