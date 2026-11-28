#pragma once
#include "chase_pd.h"
#include "line_safety.h"

enum class State { Stop, Chase, Escape };
enum class StopReason {
  None, Manual, CameraTimeout, CameraNotReady, CameraSlow, IrTimeout, IrInvalid,
  BallLost, NoEscape, PathBlocked, IrNoReply, DiagnosticIdle, Startup, Count
};
enum class PathMode { Stop, Original, Sweep, ReducedTurn, Straight, Escape, Diagnostic };
enum class Fallback { None, Mask, Unmeasured, Range, Ambiguous, Unsupported };
struct Decision {
  State state = State::Stop;
  StopReason reason = StopReason::CameraTimeout;
  uint16_t required = 0, blocked = 0;
  robot::Pwm pwm{};
  PathMode path = PathMode::Stop;
  Fallback fallback = Fallback::None;
  uint8_t alternate = 0;
};

inline Decision decide(bool manual, bool cameraFresh, const robot::Camera& cam,
                       bool irFresh, bool irInvalid, bool missedIr, bool tracking, const robot::Pwm& pwm, int error = 180,
                       const SweepConfig& config = CompetitionSweep, bool reverseBridge = EnableReverseBridge) {
  (void)error; (void)config; (void)reverseBridge;
  Decision out;
  if (manual) out.reason = StopReason::Manual;
  else if (!cameraFresh) out.reason = StopReason::CameraTimeout;
  else if (!irFresh) out.reason = StopReason::IrTimeout;
  else if (!(cam.flags & robot::Ready))
    out.reason = StopReason::CameraNotReady;
  else if (irInvalid || missedIr) out.reason = StopReason::IrInvalid;
  else if (cam.flags & (robot::Escape | robot::LineHazard)) {
    uint16_t direction = 0;
    if (cam.line != robot::Unknown && cam.line != 90 && cam.line != 270)
      direction = (cam.line < 90 || cam.line > 270) ? robot::Rear : robot::Front;
    const uint16_t reported = cam.escape & (robot::Front | robot::Rear);
    if (!direction && (reported == robot::Front || reported == robot::Rear)) direction = reported;
    // A contradictory explicit camera direction cannot establish a safe retreat.
    else if (direction && reported && !(reported & direction)) direction = 0;
    if (direction) {
      const int16_t duty = direction == robot::Rear ? -175 : 175;
      for (auto& value : out.pwm) value = duty;
      out.state = State::Escape; out.path = PathMode::Escape;
      out.reason = StopReason::None; out.required = direction;
    } else { out.reason = StopReason::NoEscape; out.fallback = Fallback::Ambiguous; }
  } else if (!tracking) out.reason = StopReason::BallLost;
  else {
    memcpy(out.pwm, pwm, sizeof(out.pwm));
    if (cam.flags & robot::Slow) for (auto& value : out.pwm) {
      if (value > 140) value = 140;
      if (value < -140) value = -140;
    }
    out.state = State::Chase; out.path = PathMode::Original; out.reason = StopReason::None;
  }
  return out;
}

// Disabled in field builds. A bench build still requires explicit mode and a USB dead man.
#ifndef RCJ_WHEELS_LIFTED_DIAGNOSTICS
#define RCJ_WHEELS_LIFTED_DIAGNOSTICS 0
#endif
enum class RunMode { Competition, MotorTest, IrPdTest, BasicChase, ManagedCompetition, PdChase, GoalYellow, GoalBlue };

#ifndef RCJ_BASIC_CHASE
#define RCJ_BASIC_CHASE 0
#endif
#ifndef RCJ_MANAGED_COMPETITION
#define RCJ_MANAGED_COMPETITION (!RCJ_BASIC_CHASE && !RCJ_WHEELS_LIFTED_DIAGNOSTICS)
#endif
#if RCJ_MANAGED_COMPETITION && (RCJ_BASIC_CHASE || RCJ_WHEELS_LIFTED_DIAGNOSTICS)
#error Managed competition requires a separate build
#endif
#if RCJ_BASIC_CHASE && RCJ_WHEELS_LIFTED_DIAGNOSTICS
#error BASIC_CHASE and lifted-wheel diagnostics require separate builds
#endif

inline Decision basicDecision(bool armed, int cap, bool manual, bool cameraFresh,
                              const robot::Camera& cam, bool irFresh, bool irInvalid,
                              bool missedIr, bool tracking, const robot::Pwm& pwm) {
  // Keep the existing link, manual and white-line escape priority.
  Decision out = decide(manual, cameraFresh, cam, irFresh, irInvalid, missedIr, false, pwm);
  if (out.reason != StopReason::BallLost && out.state != State::Escape) return out;
  if (!armed || (cap != 140 && cap != 175 && cap != 210)) {
    out = Decision{};
    out.reason = StopReason::DiagnosticIdle;
    return out;
  }
  if (out.state != State::Escape) {
    if (!tracking) return out;
    // V1 cannot distinguish unknown sectors from retained white-line hazards.
    if (cam.version < 2 || (cam.flags & robot::LineHazard) ||
        (cam.line != robot::Unknown && (cam.lineCm == 255 || cam.lineCm <= 35))) {
      out.reason = StopReason::PathBlocked;
      return out;
    }
    if (pwm[0] != pwm[1] || pwm[2] != pwm[3]) {
      out.reason = StopReason::PathBlocked;
      out.fallback = Fallback::Unsupported;
      return out;
    }
    memcpy(out.pwm, pwm, sizeof(out.pwm));
    out.state = State::Chase;
    out.reason = StopReason::None;
    out.path = PathMode::Original;
  }
  for (auto& value : out.pwm) {
    if (value > cap) value = cap;
    if (value < -cap) value = -cap;
  }
  return out;
}

inline Decision benchDecision(RunMode mode, bool armed, int wheel, bool tracking,
                              const robot::Pwm& pwm, bool manual, bool cameraFresh, bool irFresh,
                              bool irInvalid, bool missedIr) {
  // Reuse link/manual stops. Diagnostic motion bypasses image-based direction checks only.
  robot::Camera link;
  link.flags = robot::Ready;
  Decision out = decide(manual, cameraFresh, link, irFresh, irInvalid, missedIr, false, pwm);
  if (out.reason != StopReason::BallLost) return out;
  if (!armed || (mode == RunMode::MotorTest && (wheel < 0 || wheel >= 6))) {
    out.reason = StopReason::DiagnosticIdle;
    return out;
  }
  if (mode == RunMode::IrPdTest) {
    if (!tracking) return out;
    for (int i = 0; i < 4; ++i)
      out.pwm[i] = pwm[i] > 160 ? 160 : pwm[i] < -160 ? -160 : pwm[i];
  } else if (mode == RunMode::MotorTest) {
    if (wheel < 4) out.pwm[wheel] = 160;
    else {
      // Diagnostic candidates only; direction and trajectory are not commissioned.
      const int16_t duty = wheel == 4 ? 160 : -160;
      out.pwm[0] = out.pwm[2] = -duty;
      out.pwm[1] = out.pwm[3] = duty;
    }
  }
  else return out;
  out.state = State::Chase;
  out.reason = StopReason::None;
  out.path = PathMode::Diagnostic;
  return out;
}

// Isolated floor test: ignore only image-derived line gating, never sensor links.
inline Decision trialDecision(bool armed, bool manual, bool cameraFresh, const robot::Camera& cam,
    bool irFresh, bool irInvalid, bool missedIr, bool tracking, const robot::Pwm& pwm,
    RunMode mode, int ballError, uint32_t cameraAge) {
  robot::Camera link = cam;
  link.flags &= ~(robot::Escape | robot::LineHazard);
  if (mode == RunMode::PdChase) { link.flags = robot::Ready; cameraFresh = true; }
  Decision out = decide(manual, cameraFresh, link, irFresh, irInvalid, missedIr, false, pwm);
  if (out.reason != StopReason::BallLost) return out;
  if (!armed) { out.reason = StopReason::DiagnosticIdle; return out; }
  if (!tracking) return out;
  memcpy(out.pwm, pwm, sizeof(out.pwm));
  const robot::Goal& goal = mode == RunMode::GoalBlue ? cam.blue : cam.yellow;
  if ((mode == RunMode::GoalBlue || mode == RunMode::GoalYellow) && ballError >= -30 && ballError <= 30 &&
      goal.angle < 360 && goal.confidence && cameraAge + 10u * goal.age < 300) {
    int bearing = goal.angle > 180 ? int(goal.angle) - 360 : int(goal.angle);
    int correction = bearing;
    if (correction > 40) correction = 40;
    if (correction < -40) correction = -40;
    out.pwm[0] = out.pwm[1] = int16_t(out.pwm[0] - correction);
    out.pwm[2] = out.pwm[3] = int16_t(out.pwm[2] + correction);
  }
  for (auto& value : out.pwm) {
    if (value > robot::MaxPwm) value = robot::MaxPwm;
    if (value < -robot::MaxPwm) value = -robot::MaxPwm;
  }
  out.state = State::Chase; out.reason = StopReason::None; out.path = PathMode::Diagnostic;
  return out;
}
