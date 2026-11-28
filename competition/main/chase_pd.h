#pragma once
#include <RobotCommon.h>

struct ChasePD {
  bool initialized = false;
  uint32_t previousMs = 0;
  int error = 0, turn = 0;
  float p = 0, d = 0;
  robot::Pwm pwm{};

  void reset() {
    initialized = false;
    error = turn = 0;
    p = d = 0;
    memset(pwm, 0, sizeof(pwm));
  }
  static int16_t limit(int value) {
    if (!value) return 0;
    int magnitude = value < 0 ? -value : value;
    if (magnitude < 140) magnitude = 140;
    if (magnitude > 210) magnitude = 210;
    return int16_t(value < 0 ? -magnitude : magnitude);
  }
  void update(uint16_t angle, uint16_t strength, uint32_t now) {
    if (!strength || angle >= 360) { reset(); return; }
    const int next = angle > 180 ? int(angle) - 360 : int(angle);
    float rate = 0;
    if (initialized && now != previousMs) {
      int delta = next - error;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      rate = delta * 1000.0f / (now - previousMs);
    }
    error = next;
    previousMs = now;
    initialized = true;
    p = 2.0f * error;
    d = 0.2f * rate;
    turn = int(p + d);
    if (turn < -250) turn = -250;
    if (turn > 250) turn = 250;
    pwm[0] = pwm[1] = limit(175 - turn);
    pwm[2] = pwm[3] = limit(175 + turn);
  }
};
