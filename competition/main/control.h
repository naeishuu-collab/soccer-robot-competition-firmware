#pragma once
#include "chase_pd.h"
#include "line_safety.h"

enum class State { Stop, Chase, Escape };
enum class StopReason {
  None, Manual, CameraTimeout, CameraNotReady, CameraSlow, IrTimeout, IrInvalid,
  BallLost, NoEscape, PathBlocked, IrNoReply, DiagnosticIdle, Count
};
enum class PathMode { Stop, Original, Sweep, ReducedTurn, Straight, Escape, Diagnostic };
enum class Fallback { None, Mask, Unmeasured, Range };
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
  Decision out;
  if (manual) out.reason = StopReason::Manual;
  else if (!cameraFresh) out.reason = StopReason::CameraTimeout;
  else if (cam.flags & robot::Slow) out.reason = StopReason::CameraSlow;
  else if (!(cam.flags & robot::Ready)) out.reason = StopReason::CameraNotReady;
  else if (!irFresh && (!irInvalid || missedIr)) out.reason = StopReason::IrTimeout;
  else if (irInvalid) out.reason = StopReason::IrInvalid;
  else if (missedIr) out.reason = StopReason::IrNoReply;
  else if (cam.flags & robot::Escape) {
    out.required = robot::Front | robot::Rear;
    out.blocked = out.required & ~cam.escape;
    if (cam.escape) {
      // Forward/reverse are the only escape motions enabled for this build.
      int16_t duty = cam.escape & robot::Rear ? -175 : 175;
      for (auto& value : out.pwm) value = duty;
      out.state = State::Escape;
      out.path = PathMode::Escape;
      out.reason = StopReason::None;
      out.required = duty < 0 ? robot::Rear : robot::Front;
      out.blocked = 0;
    } else out.reason = StopReason::NoEscape;
  } else if (!tracking) out.reason = StopReason::BallLost;
  else {
    if (pwm[0] != pwm[1] || pwm[2] != pwm[3]) {
      out.reason = StopReason::PathBlocked;
      return out;
    }
    const bool commissioned = config.halfWidthCm > 0;
    const bool frontClear = cam.safe & robot::Front;
    const bool forwardBall = error >= -45 && error <= 45 && pwm[0] + pwm[2] > 0;
    const robot::Pwm straight{robot::BridgePwm, robot::BridgePwm, robot::BridgePwm, robot::BridgePwm};
    if (commissioned) {
      const auto sweep = candidateRequired(pwm, config);
      out.required = sweep.required;
      out.blocked = out.required & ~cam.safe;
      if (permitsSweep(sweep, cam.safe)) out.path = PathMode::Sweep;
      else {
        out.fallback = sweep.withinRange ? Fallback::Mask : Fallback::Range;
        const int middle = (pwm[0] + pwm[2]) / 2;
        int16_t left = ChasePD::limit((middle + pwm[0]) / 2);
        int16_t right = ChasePD::limit((middle + pwm[2]) / 2);
        if (middle && pwm[0] * pwm[2] < 0) {
          // Preserve the direction of the yaw bias with a bounded same-sign vector.
          const int16_t nearer = middle > 0 ? 140 : -140;
          const int16_t farther = middle > 0 ? 175 : -175;
          left = pwm[0] * middle < 0 ? nearer : farther;
          right = pwm[2] * middle < 0 ? nearer : farther;
        }
        const robot::Pwm reduced{left, left, right, right};
        const auto reducedSweep = candidateRequired(reduced, config);
        if ((left || right) && permitsSweep(reducedSweep, cam.safe)) {
          out.path = PathMode::ReducedTurn;
          out.required = reducedSweep.required;
          memcpy(out.pwm, reduced, sizeof(out.pwm));
        }
      }
    } else if (pwm[0] == pwm[2]) {
      out.required = pwm[0] > 0 ? robot::Front : pwm[0] < 0 ? robot::Rear : 0;
      if (!(out.required & ~cam.safe)) out.path = PathMode::Original;
    } else if (cam.safe == 0xffff) {
      // Allow the original yaw only in fully clear space until its sweep is measured.
      out.required = cam.safe;
      out.path = PathMode::Original;
    } else {
      out.required = robot::Front;
    }
    if (out.path == PathMode::Stop) {
      if (out.fallback == Fallback::None) out.fallback = Fallback::Unmeasured;
      // The camera front bit already checks the five forward rays.
      if (frontClear && forwardBall && pwm[0] > 0 && pwm[2] > 0) {
        const auto sweep = commissioned ? candidateRequired(straight, config) : Sweep{robot::Front, true};
        if (permitsSweep(sweep, cam.safe)) {
          out.path = PathMode::Straight;
          out.required = sweep.required | robot::Front;
          memcpy(out.pwm, straight, sizeof(out.pwm));
        }
      }
    }
    out.blocked = out.required & ~cam.safe;
    if (out.path == PathMode::Stop) {
      out.reason = StopReason::PathBlocked;
      return out;
    }
    if (out.path == PathMode::Original || out.path == PathMode::Sweep)
      memcpy(out.pwm, pwm, sizeof(out.pwm));
    if (reverseBridge && commissioned && frontClear && forwardBall && out.pwm[0] * out.pwm[2] < 0) {
      const auto sweep = candidateRequired(straight, config);
      if (permitsSweep(sweep, cam.safe)) {
        out.alternate = robot::BridgePwm;
        out.required |= sweep.required | robot::Front;
      }
    }
    out.blocked = out.required & ~cam.safe;
    out.state = State::Chase;
    out.reason = StopReason::None;
  }
  return out;
}

// Disabled in field builds. A bench build still requires explicit mode and a USB dead man.
#ifndef RCJ_WHEELS_LIFTED_DIAGNOSTICS
#define RCJ_WHEELS_LIFTED_DIAGNOSTICS 0
#endif
enum class RunMode { Competition, MotorTest, IrPdTest };

inline Decision benchDecision(RunMode mode, bool armed, int wheel, bool tracking,
                              const robot::Pwm& pwm, bool manual, bool cameraFresh, bool irFresh,
                              bool irInvalid, bool missedIr) {
  // Reuse link/manual stops. Diagnostic motion bypasses image-based direction checks only.
  robot::Camera link;
  link.flags = robot::Ready;
  Decision out = decide(manual, cameraFresh, link, irFresh, irInvalid, missedIr, false, pwm);
  if (out.reason != StopReason::BallLost) return out;
  if (!armed || (mode == RunMode::MotorTest && (wheel < 0 || wheel >= 4))) {
    out.reason = StopReason::DiagnosticIdle;
    return out;
  }
  if (mode == RunMode::IrPdTest) {
    if (!tracking) return out;
    for (int i = 0; i < 4; ++i)
      out.pwm[i] = pwm[i] > 160 ? 160 : pwm[i] < -160 ? -160 : pwm[i];
  } else if (mode == RunMode::MotorTest) out.pwm[wheel] = 160;
  else return out;
  out.state = State::Chase;
  out.reason = StopReason::None;
  out.path = PathMode::Diagnostic;
  return out;
}
