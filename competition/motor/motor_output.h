#pragma once
#include <RobotCommon.h>

struct MotorOutput {
  robot::Pwm desired{}, applied{};
  uint32_t lastCommandMs = 0, offMs[4]{};
  int8_t lastSign[4]{};
  bool seen = false, watchdog = false, reverseWait = false;
  uint8_t reverseMask = 0;
  static constexpr uint32_t ReverseMs = 200;

  void accept(const robot::Pwm& pwm, uint32_t now) {
    memcpy(desired, pwm, sizeof(desired));
    lastCommandMs = now;
    seen = true;
  }
  void update(uint32_t now) {
    watchdog = seen && now - lastCommandMs >= robot::WatchdogMs;
    if (watchdog) memset(desired, 0, sizeof(desired));
    reverseMask = 0;
    for (int i = 0; i < 4; ++i) {
      if (desired[i] * lastSign[i] < 0 && (applied[i] || now - offMs[i] < ReverseMs))
        reverseMask |= uint8_t(1 << i);
    }
    // A partial command can move the robot outside the path MAIN checked.
    reverseWait = reverseMask != 0;
    for (int i = 0; i < 4; ++i) {
      const int16_t next = reverseWait ? 0 : desired[i];
      if (next) lastSign[i] = next > 0 ? 1 : -1;
      else if (applied[i]) offMs[i] = now;
      applied[i] = next;
    }
  }
};
