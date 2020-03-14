#include <sam.h>
#include "../external/same70/init.hpp"
#include "drivers/clock/sysclk.hpp"
#include "drivers/led/led.hpp"
#include "utils/timer.hpp"
#include "drivers/delay/delay.hpp"

void init()
{
    SystemCoreClockUpdate();
    sysclk_init();
    board_init();

    SysTick_Config(SystemCoreClock / 1000);

    Led::init();
}

int main()
{
    init();

    delay_ms(1000);
    Led::g_on();

    return 0;
}