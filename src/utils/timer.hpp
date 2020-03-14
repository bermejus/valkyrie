#pragma once

#include <cstdint>

class Timer
{
private:
    uint64_t start;

public:
    Timer();

    void reset();

    float elapsed() const;

    __attribute__((optimize("-O0")))
    void delay(float time) const;
};