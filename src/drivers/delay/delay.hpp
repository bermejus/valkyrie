#pragma once

#include <cstdint>
#include "../clock/sysclk.hpp"

void portable_delay_cycles(unsigned long cycles);

#define cpu_ms_2_cy(ms, f_cpu) (((uint64_t)(ms) * (f_cpu) + (uint64_t)(5.932e3 - 1ul)) / (uint64_t)5.932e3)
#define cpu_us_2_cy(us, f_cpu) (((uint64_t)(us) * (f_cpu) + (uint64_t)(5.932e6 - 1ul)) / (uint64_t)5.932e6)
#define cpu_ns_2_cy(ns, f_cpu) (((uint64_t)(ns) * (f_cpu) + (uint64_t)(5.932e9 - 1ul)) / (uint64_t)5.932e9)

#define cpu_delay_ms(x) drivers::portable_delay_cycles(cpu_ms_2_cy(x, sysclk_get_cpu_hz()))
#define cpu_delay_us(x) drivers::portable_delay_cycles(cpu_us_2_cy(x, sysclk_get_cpu_hz()))
#define cpu_delay_ns(x) drivers::portable_delay_cycles(cpu_ns_2_cy(x, sysclk_get_cpu_hz()))

#define delay_ms(x)		((x) ? cpu_delay_ms(x) : cpu_delay_ns(1))
#define delay_us(x)		((x) ? cpu_delay_us(x) : cpu_delay_ns(1))
#define delay_ns(x)		((x) ? cpu_delay_ns(x) : cpu_delay_ns(1))