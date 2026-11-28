#pragma once
#include "goal_correction.h"

#ifndef RCJ_PHASE3_ORBIT
#define RCJ_PHASE3_ORBIT 0
#endif
#if RCJ_PHASE3_ORBIT && defined(ARDUINO)
#error Phase 3 is simulation-only until distance, wheel direction and trajectory safety are commissioned
#endif

// Simulation candidate only: neither lateral direction nor IR strength is a calibrated distance.
struct OrbitCandidate {
  bool active = false, cooling = false, completedHold = false;
  int8_t direction = 0;
  uint32_t started = 0, finished = 0;
  static constexpr uint32_t LimitMs = 600, RetryMs = 2000;

  bool update(Decision& out, const robot::Camera& cam, uint32_t cameraAge,
              int ballError, uint32_t now) {
    const robot::Goal& own = (cam.flags & robot::TargetBlue) ? cam.yellow : cam.blue;
    const robot::Goal& attack = (cam.flags & robot::TargetBlue) ? cam.blue : cam.yellow;
    const auto fresh = [cameraAge, &cam](const robot::Goal& goal) {
      return goal.angle < 360 && goal.confidence >= OwnGoalConfig::MinQuality &&
          cameraAge + uint32_t(cam.frameMs) + 10u * goal.age < OwnGoalConfig::MaxAgeMs;
    };
    const bool valid = out.state == State::Chase && !(cam.flags & (robot::Escape | robot::LineHazard)) &&
        fresh(own) && fresh(attack);
    int target = attack.angle < 360 ? (attack.angle > 180 ? int(attack.angle) - 360 : int(attack.angle)) : 0;
    int delta = ballError - target;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    const bool aligned = delta >= -25 && delta <= 25;
    if (!valid || !aligned) completedHold = false;
    const bool completed = active && valid && aligned;
    if (completed) completedHold = true;
    if (active && (!valid || aligned || now - started >= LimitMs)) {
      active = false; cooling = true; finished = now;
    }
    if (completedHold) return true;  // Preserve PD while the attack alignment persists.
    if (!valid || aligned) return false;
    if (cooling) {
      if (now - finished < RetryMs) return false;
      cooling = false;
    }
    if (!active) {
      Decision danger = out;
      int8_t turn = 0;
      if (!avoidOwnGoal(danger, cam, cameraAge, ballError, turn)) return false;
      direction = turn;
      started = now;
      active = true;
    }
    const int16_t duty = int16_t(direction * 140);
    out.pwm[0] = out.pwm[2] = -duty;
    out.pwm[1] = out.pwm[3] = duty;
    return true;
  }
};
