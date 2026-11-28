#pragma once

const char* const states[] = {"STOP", "CHASE", "ESCAPE"};
const char* const reasons[] = {"NONE", "MANUAL_STOP", "CAMERA_TIMEOUT", "CAMERA_NOT_READY",
  "CAMERA_SLOW", "IR_TIMEOUT", "IR_INVALID", "BALL_LOST", "NO_SAFE_ESCAPE", "PATH_BLOCKED", "IR_NO_REPLY"};
const char* const details[] = {"active", "usb_s", "no_fresh_crc_frame", "camera_ready_flag_clear",
  "capture_and_safety_over_100ms", "no_valid_reply_in_100ms", "malformed_ir_response",
  "no_fresh_ball_bearing", "neither_forward_nor_reverse_clear", "pwm_trajectory_mask_closed", "no_reply_in_reserved_bus_slot"};

struct Event { uint32_t ms; State state; StopReason reason; };
Event events[8]{};
uint8_t eventRead = 0, eventWrite = 0;
uint32_t eventDropped = 0;
State previousState = State::Stop;
StopReason previousReason = StopReason::None;

void logEvent(uint32_t now) {
  if (decision.state == previousState && decision.reason == previousReason) return;
  events[eventWrite] = {now, decision.state, decision.reason};
  eventWrite = (eventWrite + 1) % 8;
  if (eventWrite == eventRead) {
    eventRead = (eventRead + 1) % 8;
    ++eventDropped;
  }
  previousState = decision.state;
  previousReason = decision.reason;
}

int visibleGoal(const Goal& goal, uint32_t now) {
  return haveCamera && goal.angle != Unknown && now - cameraMs + 10u * goal.age < 300
      ? int(goal.angle) : -1;
}

void diagnose(uint32_t now, bool cameraFresh, bool irFresh) {
  if (!Serial) {
    report.pump();
    eventRead = eventWrite;
    return;
  }
  if (!waiting && !report.busy() && (requestedReport || now - reportMs >= 1000)) {
    const int reason = int(decision.reason);
    const int yellow = visibleGoal(camera.yellow, now), blue = visibleGoal(camera.blue, now);
    report.queue(snprintf(report.data, sizeof(report.data),
        "STATE=%s STOP_REASON=%s STOP_DETAIL=%s\n"
        "IR_LINK=%s IR_ANGLE=%d IR_STRENGTH=%u IR_AGE_MS=%ld IR_INVALID_COUNT=%lu\n"
        "CAMERA_LINK=%s CAMERA_AGE_MS=%ld CAMERA_READY=%u CAMERA_SLOW=%u FRAME_MS=%u CAMERA_BAD=%lu\n"
        "WHITE_LINE=%u LINE_ANGLE=%d LINE_DISTANCE=%d SAFE_MASK=%04x REQUIRED_MASK=%04x BLOCKED_MASK=%04x\n"
        "YELLOW_GOAL=%d BLUE_GOAL=%d TARGET_GOAL=%s OPPONENT_ANGLE=%d OWN_ANGLE=%d GOAL_QUALITY=%u,%u\n"
        "PD_ERROR=%d PD_P=%.2f PD_D=%.2f PD_TURN=%d\n"
        "PWM_CALCULATED=%d,%d,%d,%d PWM_AFTER_SAFETY=%d,%d,%d,%d PWM_TRANSMITTED=%d,%d,%d,%d\n"
        "MOTOR_TX_COUNT=%lu EVENT_DROPPED=%lu\n",
        states[int(decision.state)], reasons[reason],
        decision.reason == StopReason::BallLost && irAngle == Unknown && irStrength
            ? "ir_vector_cancelled" : details[reason],
        irFresh && !irInvalid && !missedIr ? "OK" : "ERROR", irAngle < 360 ? int(irAngle) : -1, unsigned(irStrength),
        haveIr ? long(now - irMs) : -1L, (unsigned long)irBad,
        cameraFresh ? "OK" : "ERROR", haveCamera ? long(now - cameraMs) : -1L,
        unsigned(bool(camera.flags & Ready)), unsigned(bool(camera.flags & Slow)),
        unsigned(camera.frameMs), (unsigned long)cameraBad, unsigned(camera.line != Unknown),
        camera.line == Unknown ? -1 : int(camera.line), camera.lineCm == 255 ? -1 : int(camera.lineCm),
        unsigned(camera.safe), unsigned(decision.required), unsigned(decision.blocked),
        yellow, blue, camera.flags & TargetBlue ? "BLUE" : "YELLOW",
        camera.flags & TargetBlue ? blue : yellow, camera.flags & TargetBlue ? yellow : blue,
        yellow < 0 ? 0u : unsigned(camera.yellow.confidence), blue < 0 ? 0u : unsigned(camera.blue.confidence),
        pd.error, double(pd.p), double(pd.d), pd.turn,
        pd.pwm[0], pd.pwm[1], pd.pwm[2], pd.pwm[3],
        decision.pwm[0], decision.pwm[1], decision.pwm[2], decision.pwm[3],
        transmitted[0], transmitted[1], transmitted[2], transmitted[3],
        (unsigned long)motorTxCount, (unsigned long)eventDropped));
    requestedReport = false;
    reportMs = now;
  } else if (!waiting && !report.busy() && eventRead != eventWrite) {
    const Event& event = events[eventRead];
    report.queue(snprintf(report.data, sizeof(report.data), "EVENT_MS=%lu STATE=%s STOP_REASON=%s\n",
        (unsigned long)event.ms, states[int(event.state)], reasons[int(event.reason)]));
    eventRead = (eventRead + 1) % 8;
  }
  if (!waiting) report.pump();
}
