#pragma once

#include <stdint.h>

// A LOW is counted only when confirmed by an adjacent 10 us reading,
// matching the diagnostic sketch that produced calibration.json.
struct IrLowWindow {
  uint32_t nextRead = 0;
  uint16_t previousLow = 0;
  uint16_t supportedLow = 0;

  void sample(uint32_t elapsed, uint16_t active, uint16_t (&counts)[12], uint16_t &reads) {
    if (elapsed < nextRead) return;
    const uint16_t adjacent = elapsed - nextRead <= 10 ? previousLow & active : 0;
    add(supportedLow | adjacent, counts);
    previousLow = active;
    supportedLow = adjacent;
    nextRead = elapsed + 10;
    ++reads;
  }

  void finish(uint16_t (&counts)[12]) const { add(supportedLow, counts); }

private:
  static void add(uint16_t bits, uint16_t (&counts)[12]) {
    for (int i = 0; i < 12; ++i) counts[i] += (bits >> i) & 1u;
  }
};
