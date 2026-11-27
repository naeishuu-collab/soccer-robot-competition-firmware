#pragma once
#include <stdint.h>

// Exact chase (4).zip controller. The input is the raw ZIP IR bearing.
// Call once per accepted IR reply; reset after a stop or lost detection.
class ZipChasePD {
 public:
  void reset() { initialized_ = false; }
  bool update(uint16_t direction, uint16_t count, uint32_t now, int16_t out[4]) {
    if (!count || direction >= 360) {
      reset();
      for (int i = 0; i < 4; ++i) out[i] = 0;
      return false;
    }
    int error = direction > 180 ? int(direction) - 360 : int(direction);
    float derivative = 0;
    uint32_t dt = now - previousMs_;
    if (initialized_ && dt) {
      int delta = error - previousError_;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      derivative = delta * 1000.0f / dt;
    }
    previousError_ = error;
    previousMs_ = now;
    initialized_ = true;
    int turn = int(2.0f * error + 0.2f * derivative);
    if (turn > 250) turn = 250;
    if (turn < -250) turn = -250;
    out[0] = out[1] = limit(175 - turn);
    out[2] = out[3] = limit(175 + turn);
    return true;
  }
 private:
  static int16_t limit(int value) {
    if (!value) return 0;
    int sign = value < 0 ? -1 : 1;
    int magnitude = value < 0 ? -value : value;
    if (magnitude < 140) magnitude = 140;
    if (magnitude > 210) magnitude = 210;
    return int16_t(sign * magnitude);
  }
  bool initialized_ = false;
  int previousError_ = 0;
  uint32_t previousMs_ = 0;
};
