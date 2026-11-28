#pragma once
#include <RobotCommon.h>

struct MotorOutput {
  robot::Pwm desired{}, applied{};
  uint32_t lastCommandMs = 0, offMs[4]{};
  int8_t lastSign[4]{};
  bool seen = false, watchdog = false, reverseWait = false;
  uint8_t reverseMask = 0, alternate = 0;
  uint32_t waitCount = 0, waitMs = 0, waitStart = 0, bridgeCount = 0;
  bool bridging = false;
  static constexpr uint32_t ReverseMs = 200;

  void accept(const robot::Pwm& pwm, uint32_t now, uint8_t authorized = 0) {
    memcpy(desired, pwm, sizeof(desired));
    alternate = authorized;
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
    const bool reverse = reverseMask != 0;
    if (reverse && !reverseWait) {
      waitStart = now;
      ++waitCount;
      bridging = alternate == robot::BridgePwm && applied[0] >= alternate &&
          applied[0] == applied[1] && applied[0] == applied[2] && applied[0] == applied[3];
      if (bridging) ++bridgeCount;
    }
    if (!reverse && reverseWait) waitMs += now - waitStart;
    reverseWait = reverse;
    bridging = reverse && bridging && alternate == robot::BridgePwm && now - waitStart < 35;
    for (int i = 0; i < 4; ++i) {
      const int16_t next = reverseWait ? (bridging ? alternate : 0) : desired[i];
      if (next) lastSign[i] = next > 0 ? 1 : -1;
      else if (applied[i]) offMs[i] = now;
      applied[i] = next;
    }
  }
};
