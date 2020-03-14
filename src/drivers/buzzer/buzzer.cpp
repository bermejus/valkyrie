#include "buzzer.hpp"

#include <sam.h>
#include <board_same70.hpp>
#include "../clock/sysclk.hpp"
#include "../delay/delay.hpp"

pwm_clock_t Buzzer::clock;
pwm_channel_t Buzzer::buzzer;

void Buzzer::init()
{
    pmc_enable_periph_clk(ID_PWM0);

    clock = {
        .ul_clka    = 0,
        .ul_clkb    = 1000 * 200,
        .ul_mck     = sysclk_get_peripheral_hz()
    };
    pwm_init_clock_b(PWM0, &clock);

    buzzer.alignment       = PWM_ALIGN_LEFT;
    buzzer.polarity        = PWM_HIGH;
    buzzer.ul_prescaler    = PWM_CMR_CPRE_CLKB;
    buzzer.ul_period       = 200;
    buzzer.ul_duty         = 0;
    buzzer.channel         = BUZZER_CHANNEL;

    pwm_channel_init(PWM0, &buzzer);
    pwm_channel_disable(PWM0, BUZZER_CHANNEL);
}

void Buzzer::tone(unsigned int freq, unsigned int volume, unsigned int duration)
{
    pwm_channel_disable(PWM0, BUZZER_CHANNEL);

    clock.ul_clkb = freq * 200;
    pwm_init_clock_b(PWM0, &clock);

    buzzer.ul_duty = volume;
    pwm_channel_init(PWM0, &buzzer);
    pwm_channel_enable(PWM0, BUZZER_CHANNEL);

    if (duration)
    {
        delay_ms(duration);
        pwm_channel_disable(PWM0, BUZZER_CHANNEL);
    }
}