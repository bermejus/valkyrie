#pragma once

#include <cstdint>

class Led
{
public:
    static void init();

    static void on(uint32_t brightness = 100);
    static void r_on(uint32_t brightness = 100);
    static void g_on(uint32_t brightness = 100);
    static void b_on(uint32_t brightness = 100);

    static void off();
    static void r_off();
    static void g_off();
    static void b_off();
};