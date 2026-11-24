#pragma once

#include <math.h>
#include <stdint.h>
#include "ir_profile.h"

// The Mac ranks RMS errors using a common noise scale for each GPIO.
inline int matchIr(const uint16_t (&counts)[12], uint16_t reads) {
  if (!reads) return -1;
  float rates[12];
  const float invReads = 1.0f / reads;
  for (int i = 0; i < 12; ++i) rates[i] = counts[i] * invReads;

  float best = INFINITY;
  float second = INFINITY;
  int bestPose = 0;
  for (int pose = 0; pose < 13; ++pose) {
    float sum = 0;
    for (int i = 0; i < 12; ++i) {
      const float delta = (rates[i] - kIrMeans[pose][i]) * kIrInverseScale[i];
      sum += delta * delta;
    }
    if (sum < best) {
      second = best;
      best = sum;
      bestPose = pose;
    } else if (sum < second) {
      second = sum;
    }
  }
  if (bestPose == 0 || best > 12 * 4.0f * 4.0f) return -1;
  if (sqrtf(second / 12) - sqrtf(best / 12) < 0.25f) return -1;
  return (bestPose - 1) * 30;
}
