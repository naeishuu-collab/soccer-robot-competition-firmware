#pragma once
#include <RobotCommon.h>

// Set RCJ_PD_LEGACY_DEFAULT to 1 here to restore the original controller.
#ifndef RCJ_PD_LEGACY_DEFAULT
#define RCJ_PD_LEGACY_DEFAULT 0
#endif
namespace PdConfig {
constexpr float Kp = 1.8f, Kd = 0.08f, DAlpha = 0.30f, DLimit = 50.0f;
constexpr int Deadband = 4, TurnLimit = 180, BasePwm = 175, MinPwm = 140;
constexpr float TurnPerSecond = 2000.0f;
}

struct ChasePD {
  bool initialized = false;
  bool legacy = RCJ_PD_LEGACY_DEFAULT;
  uint32_t previousMs = 0;
  int error = 0, turn = 0, rawAngle = 0, basePwm = PdConfig::BasePwm;
  float p = 0, d = 0, dRaw = 0, turnRaw = 0, filteredTurn = 0;
  robot::Pwm pwm{};

  void reset() {
    initialized = false;
    error = turn = rawAngle = 0; basePwm = PdConfig::BasePwm;
    p = d = dRaw = turnRaw = filteredTurn = 0;
    memset(pwm, 0, sizeof(pwm));
  }
  static int16_t limit(int value) {
    if (!value) return 0;
    int magnitude = value < 0 ? -value : value;
    if (magnitude < PdConfig::MinPwm) magnitude = PdConfig::MinPwm;
    if (magnitude > robot::MaxPwm) magnitude = robot::MaxPwm;
    return int16_t(value < 0 ? -magnitude : magnitude);
  }
  void update(uint16_t angle, uint16_t strength, uint32_t now) {
    if (!strength || angle >= 360) { reset(); return; }
    const int next = angle > 180 ? int(angle) - 360 : int(angle);
    const uint32_t dt = now - previousMs;
    float rate = 0;
    if (initialized && dt) {
      int delta = next - error;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      rate = delta * 1000.0f / dt;
    }
    rawAngle = angle; error = next;
    if (legacy) {
      p = 2.0f * error; d = dRaw = 0.2f * rate;
      turnRaw = p + d;
      turn = int(turnRaw);
      if (turn < -250) turn = -250;
      if (turn > 250) turn = 250;
      filteredTurn = float(turn); basePwm = 175;
    } else {
      p = (error >= -PdConfig::Deadband && error <= PdConfig::Deadband) ? 0 : PdConfig::Kp * error;
      dRaw = PdConfig::Kd * rate;
      float boundedD = dRaw;
      if (boundedD > PdConfig::DLimit) boundedD = PdConfig::DLimit;
      if (boundedD < -PdConfig::DLimit) boundedD = -PdConfig::DLimit;
      d = initialized ? d + PdConfig::DAlpha * (boundedD - d) : 0;
      turnRaw = p + d;
      float target = turnRaw;
      if (!p || p * target < 0) target = 0;
      if (target > PdConfig::TurnLimit) target = PdConfig::TurnLimit;
      if (target < -PdConfig::TurnLimit) target = -PdConfig::TurnLimit;
      if (initialized) {
        const float maxChange = PdConfig::TurnPerSecond * (dt > 100 ? 100 : dt) / 1000.0f;
        if (target > filteredTurn + maxChange) target = filteredTurn + maxChange;
        if (target < filteredTurn - maxChange) target = filteredTurn - maxChange;
      }
      filteredTurn = target; turn = int(target);
      int magnitude = turn < 0 ? -turn : turn;
      basePwm = PdConfig::BasePwm - (PdConfig::BasePwm - PdConfig::MinPwm) * magnitude / PdConfig::TurnLimit;
    }
    previousMs = now; initialized = true;
    pwm[0] = pwm[1] = limit(basePwm - turn);
    pwm[2] = pwm[3] = limit(basePwm + turn);
  }
};
