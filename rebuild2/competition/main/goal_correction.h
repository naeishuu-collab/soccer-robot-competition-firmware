#pragma once
#include "control.h"

// Goal packets are optional: all rejected inputs leave the existing decision untouched.
inline void correctGoal(Decision& out, const robot::Camera& cam, uint32_t cameraAge, int ballError) {
  if (out.state != State::Chase || (cam.flags & (robot::Escape | robot::LineHazard)) ||
      ballError < -4 || ballError > 4) return;
  const robot::Goal& goal = (cam.flags & robot::TargetBlue) ? cam.blue : cam.yellow;
  if (goal.angle >= 360 || goal.confidence < 70 ||
      cameraAge + uint32_t(cam.frameMs) + 10u * goal.age >= 300) return;
  int bearing = goal.angle > 180 ? int(goal.angle) - 360 : int(goal.angle);
  int correction = bearing / 3;
  if (correction > 20) correction = 20;
  if (correction < -20) correction = -20;
  const int left = out.pwm[0], right = out.pwm[2];
  if (!correction || left != out.pwm[1] || right != out.pwm[3] || left < 140 || right < 140 ||
      (ballError && correction * ballError < 0) ||
      ((right - left) && (right - left) * correction < 0)) return;
  const int cap = (cam.flags & robot::Slow) ? 140 : robot::MaxPwm;
  int nextLeft = left - correction, nextRight = right + correction;
  if (nextLeft < 140) nextLeft = 140;
  if (nextRight < 140) nextRight = 140;
  if (nextLeft > cap) nextLeft = cap;
  if (nextRight > cap) nextRight = cap;
  out.pwm[0] = out.pwm[1] = int16_t(nextLeft);
  out.pwm[2] = out.pwm[3] = int16_t(nextRight);
}

// Provisional until the goal bearing and motor turn direction are checked on the robot.
namespace OwnGoalConfig {
constexpr int GoalFrontDegrees = 25, BallFrontDegrees = 20, AlignmentDegrees = 15;
constexpr int MinQuality = 70, PivotPwm = 140;
constexpr uint32_t MaxAgeMs = 300;
}
inline bool avoidOwnGoal(Decision& out, const robot::Camera& cam, uint32_t cameraAge,
                         int ballError, int8_t& turnDirection) {
  const robot::Goal& own = (cam.flags & robot::TargetBlue) ? cam.yellow : cam.blue;
  if (out.state != State::Chase || (cam.flags & (robot::Escape | robot::LineHazard)) ||
      own.angle >= 360 || own.confidence < OwnGoalConfig::MinQuality ||
      cameraAge + uint32_t(cam.frameMs) + 10u * own.age >= OwnGoalConfig::MaxAgeMs) {
    turnDirection = 0; return false;
  }
  const int bearing = own.angle > 180 ? int(own.angle) - 360 : int(own.angle);
  const int difference = ballError - bearing;
  if (bearing < -OwnGoalConfig::GoalFrontDegrees || bearing > OwnGoalConfig::GoalFrontDegrees ||
      ballError < -OwnGoalConfig::BallFrontDegrees || ballError > OwnGoalConfig::BallFrontDegrees ||
      difference < -OwnGoalConfig::AlignmentDegrees || difference > OwnGoalConfig::AlignmentDegrees) {
    turnDirection = 0; return false;
  }
  if (!turnDirection) {
    const robot::Goal& attack = (cam.flags & robot::TargetBlue) ? cam.blue : cam.yellow;
    const bool valid = attack.angle < 360 && attack.confidence >= OwnGoalConfig::MinQuality &&
        cameraAge + uint32_t(cam.frameMs) + 10u * attack.age < OwnGoalConfig::MaxAgeMs;
    const int target = valid ? (attack.angle > 180 ? int(attack.angle) - 360 : int(attack.angle)) : 0;
    turnDirection = (target && (!bearing || target * bearing < 0))
        ? (target > 0 ? 1 : -1) : (bearing > 0 ? -1 : 1);
  }
  const int16_t turn = int16_t(turnDirection * OwnGoalConfig::PivotPwm);
  out.pwm[0] = out.pwm[1] = -turn;
  out.pwm[2] = out.pwm[3] = turn;
  return true;
}
