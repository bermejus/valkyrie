 
#pragma once

#include "../pwm/pwm.hpp"

class Buzzer
{
private:
    static pwm_clock_t clock;
    static pwm_channel_t buzzer;

public:
    static void init();
    static void tone(unsigned int freq, unsigned int volume = 100, unsigned int duration = 0);
};