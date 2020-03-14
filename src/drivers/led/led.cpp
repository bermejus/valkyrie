#include "led.hpp"

#include <sam.h>
#include <board_same70.hpp>
#include "../ioport/ioport.hpp"

#include "../clock/sysclk.hpp"
#include "../pwm/pwm.hpp"

pwm_clock_t clock;
pwm_channel_t led_r, led_g, led_b;

void Led::init()
{
    pmc_enable_periph_clk(ID_PWM1);

    clock = {
        .ul_clka = 0,
        .ul_clkb = 100 * 100,
        .ul_mck = sysclk_get_peripheral_hz()
    };
    pwm_init_clock_b(PWM1, &clock);

    led_r.alignment		= PWM_ALIGN_LEFT;
    led_r.polarity		= PWM_LOW;
    led_r.ul_prescaler	= PWM_CMR_CPRE_CLKB;
    led_r.ul_period		= 100;
    led_r.ul_duty		= 0;
    led_r.channel		= LED_R_CHANNEL;

    pwm_channel_init(PWM1, &led_r);
    pwm_channel_disable(PWM1, LED_R_CHANNEL);

    led_g.alignment		= PWM_ALIGN_LEFT;
    led_g.polarity		= PWM_LOW;
    led_g.ul_prescaler	= PWM_CMR_CPRE_CLKB;
    led_g.ul_period		= 100;
    led_g.ul_duty		= 0;
    led_g.channel		= LED_G_CHANNEL;

    pwm_channel_init(PWM1, &led_g);
    pwm_channel_disable(PWM1, LED_G_CHANNEL);

    led_b.alignment		= PWM_ALIGN_LEFT;
    led_b.polarity		= PWM_LOW;
    led_b.ul_prescaler	= PWM_CMR_CPRE_CLKB;
    led_b.ul_period		= 100;
    led_b.ul_duty		= 0;
    led_b.channel		= LED_B_CHANNEL;

    pwm_channel_init(PWM1, &led_b);
    pwm_channel_disable(PWM1, LED_B_CHANNEL);
}

void Led::on(uint32_t brightness)
{
    pwm_channel_disable(PWM1, LED_R_CHANNEL);
    pwm_channel_disable(PWM1, LED_G_CHANNEL);
    pwm_channel_disable(PWM1, LED_B_CHANNEL);

    led_r.ul_duty = 100 - brightness;
    pwm_channel_init(PWM1, &led_r);

    led_g.ul_duty = 100 - brightness;
    pwm_channel_init(PWM1, &led_g);

    led_b.ul_duty = 100 - brightness;
    pwm_channel_init(PWM1, &led_b);

    pwm_channel_enable(PWM1, LED_R_CHANNEL);
    pwm_channel_enable(PWM1, LED_G_CHANNEL);
    pwm_channel_enable(PWM1, LED_B_CHANNEL);
}

void Led::r_on(uint32_t brightness)
{
    pwm_channel_disable(PWM1, LED_R_CHANNEL);

    led_r.ul_duty = 100 - brightness;
    pwm_channel_init(PWM1, &led_r);
    pwm_channel_enable(PWM1, LED_R_CHANNEL);
}

void Led::g_on(uint32_t brightness)
{
    pwm_channel_disable(PWM1, LED_G_CHANNEL);

    led_g.ul_duty = 100 - brightness;
    pwm_channel_init(PWM1, &led_g);
    pwm_channel_enable(PWM1, LED_G_CHANNEL);
}

void Led::b_on(uint32_t brightness)
{
    pwm_channel_disable(PWM1, LED_B_CHANNEL);

    led_b.ul_duty = 100 - brightness;
    pwm_channel_init(PWM1, &led_b);
    pwm_channel_enable(PWM1, LED_B_CHANNEL);
}

void Led::off()
{
    pwm_channel_disable(PWM1, LED_R_CHANNEL);
    pwm_channel_disable(PWM1, LED_G_CHANNEL);
    pwm_channel_disable(PWM1, LED_B_CHANNEL);
}

void Led::r_off()
{
    pwm_channel_disable(PWM1, LED_R_CHANNEL);
}

void Led::g_off()
{
    pwm_channel_disable(PWM1, LED_G_CHANNEL);
}

void Led::b_off()
{
    pwm_channel_disable(PWM1, LED_B_CHANNEL);
}