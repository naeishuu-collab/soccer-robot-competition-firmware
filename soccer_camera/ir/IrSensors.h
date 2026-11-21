#pragma once
#include <stdint.h>

constexpr uint8_t kIrSensorCount = 12;
constexpr int16_t kIrCosTable[kIrSensorCount] = {
    1000, 866, 500, 0, -500, -866, -1000, -866, -500, 0, 500, 866
};
constexpr int16_t kIrSinTable[kIrSensorCount] = {
    0, -500, -866, -1000, -866, -500, 0, 500, 866, 1000, 866, 500
};
