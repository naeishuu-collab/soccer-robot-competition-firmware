#pragma once
#include <stdint.h>

inline int16_t limitMotorPwm(int16_t value) {
    if (value == 0 || value == 256) return value;
    const int16_t magnitude = value < 0 ? -value : value;
    const int16_t limited = magnitude < 120 ? 120 : (magnitude > 200 ? 200 : magnitude);
    return value < 0 ? -limited : limited;
}
