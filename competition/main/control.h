#pragma once
#include "chase_pd.h"
#include "line_safety.h"

enum class State { Stop, Chase, Escape };
enum class StopReason {
  None, Manual, CameraTimeout, CameraNotReady, CameraSlow, IrTimeout, IrInvalid,
  BallLost, NoEscape, PathBlocked, IrNoReply
};
struct Decision {
  State state = State::Stop;
  StopReason reason = StopReason::CameraTimeout;
  uint16_t required = 0, blocked = 0;
  robot::Pwm pwm{};
};

inline Decision decide(bool manual, bool cameraFresh, const robot::Camera& cam,
                       bool irFresh, bool irInvalid, bool missedIr, bool tracking, const robot::Pwm& pwm) {
  Decision out;
  if (manual) out.reason = StopReason::Manual;
  else if (!cameraFresh) out.reason = StopReason::CameraTimeout;
  else if (cam.flags & robot::Slow) out.reason = StopReason::CameraSlow;
  else if (!(cam.flags & robot::Ready)) out.reason = StopReason::CameraNotReady;
  else if (cam.flags & robot::Escape) {
    out.required = robot::Front | robot::Rear;
    out.blocked = out.required & ~cam.escape;
    if (cam.escape) {
      // Forward/reverse are the only escape motions enabled for this build.
      int16_t duty = cam.escape & robot::Rear ? -175 : 175;
      for (auto& value : out.pwm) value = duty;
      out.state = State::Escape;
      out.reason = StopReason::None;
      out.required = duty < 0 ? robot::Rear : robot::Front;
      out.blocked = 0;
    } else out.reason = StopReason::NoEscape;
  } else if (!irFresh && (!irInvalid || missedIr)) out.reason = StopReason::IrTimeout;
  else if (irInvalid) out.reason = StopReason::IrInvalid;
  else if (missedIr) out.reason = StopReason::IrNoReply;
  else if (!tracking) out.reason = StopReason::BallLost;
  else {
    out.required = legacyRequired(pwm);
    out.blocked = out.required & ~cam.safe;
    if (out.blocked || pwm[0] != pwm[1] || pwm[2] != pwm[3]) out.reason = StopReason::PathBlocked;
    else {
      out.state = State::Chase;
      out.reason = StopReason::None;
      memcpy(out.pwm, pwm, sizeof(out.pwm));
    }
  }
  return out;
}
