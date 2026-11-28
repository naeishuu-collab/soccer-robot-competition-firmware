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
    const int left = pwm[0], right = pwm[2];
    const bool straight = left == right;
    const bool commissioned = config.halfWidthCm > 0;
    const bool forward = left > 0 && right > 0;
    const bool reverse = left < 0 && right < 0;
    const uint16_t lane = forward ? robot::Front : reverse ? robot::Rear : 0;
    const int16_t duty = reverse ? -robot::BridgePwm : robot::BridgePwm;
    const robot::Pwm fallback{duty, duty, duty, duty};

    if (straight && !commissioned) {
      out.required = lane;
      if (!(lane & ~cam.safe)) out.path = PathMode::Original;
    } else if (commissioned) {
      const auto sweep = candidateRequired(pwm, config);
      out.required = sweep.required;
      if (permitsSweep(sweep, cam.safe)) out.path = PathMode::Sweep;
      else {
        out.fallback = sweep.withinRange ? Fallback::Mask : Fallback::Range;
        const int middle = (left + right) / 2;
        int16_t reducedLeft = ChasePD::limit((middle + left) / 2);
        int16_t reducedRight = ChasePD::limit((middle + right) / 2);
        if (middle && left * right < 0) {
          const int16_t nearer = middle > 0 ? 140 : -140;
          const int16_t farther = middle > 0 ? 175 : -175;
          reducedLeft = left * middle < 0 ? nearer : farther;
          reducedRight = right * middle < 0 ? nearer : farther;
        }
        const robot::Pwm reduced{reducedLeft, reducedLeft, reducedRight, reducedRight};
        const auto reducedSweep = candidateRequired(reduced, config);
        if ((reducedLeft || reducedRight) && permitsSweep(reducedSweep, cam.safe)) {
          out.path = PathMode::ReducedTurn;
          out.required = reducedSweep.required;
          memcpy(out.pwm, reduced, sizeof(out.pwm));
        }
      }
    } else {
      out.required = lane;
      out.fallback = Fallback::Unmeasured;
    }

    if (out.path == PathMode::Stop && !straight && (cam.safe & lane)) {
      // Only known translation is used without a measured yaw envelope.
      const auto sweep = commissioned ? candidateRequired(fallback, config) : Sweep{lane, true};
      if (permitsSweep(sweep, cam.safe)) {
        out.path = PathMode::Straight;
        out.required = sweep.required | lane;
        memcpy(out.pwm, fallback, sizeof(out.pwm));
      }
    }
    if (out.path == PathMode::Stop) {
      out.blocked = out.required & ~cam.safe;
      out.reason = StopReason::PathBlocked;
      return out;
    }
    if (out.path == PathMode::Original || out.path == PathMode::Sweep)
      memcpy(out.pwm, pwm, sizeof(out.pwm));
    if (reverseBridge && commissioned && (cam.safe & robot::Front) && error >= -45 && error <= 45 &&
        left + right > 0 && out.pwm[0] * out.pwm[2] < 0) {
      const auto sweep = candidateRequired(fallback, config);
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
