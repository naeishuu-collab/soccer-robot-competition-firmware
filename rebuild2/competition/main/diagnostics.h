#pragma once

const char* const states[] = {"STOP", "CHASE", "ESCAPE"};
const char* const reasons[] = {"NONE", "MANUAL_STOP", "CAMERA_TIMEOUT", "CAMERA_NOT_READY",
  "CAMERA_SLOW", "IR_TIMEOUT", "IR_INVALID", "BALL_LOST", "NO_SAFE_ESCAPE", "PATH_BLOCKED", "IR_NO_REPLY", "DIAGNOSTIC_IDLE", "STARTUP_WAIT"};
const char* const details[] = {"active", "usb_s", "no_fresh_crc_frame", "camera_ready_flag_clear",
  "capture_and_safety_over_100ms", "no_valid_reply_in_100ms", "malformed_ir_response",
  "no_fresh_ball_bearing", "neither_forward_nor_reverse_clear", "pwm_trajectory_mask_closed", "no_reply_in_reserved_bus_slot", "wheel_unselected_or_usb_lease_inactive", "startup_delay"};

struct Event { uint32_t ms; State state; StopReason reason; };
Event events[8]{};
uint8_t eventRead = 0, eventWrite = 0;
uint32_t eventDropped = 0;
State previousState = State::Stop;
StopReason previousReason = StopReason::None;
const char* const paths[] = {"STOP", "ZIP", "SWEEP", "REDUCED_TURN", "STRAIGHT_160", "ESCAPE", "DIAGNOSTIC"};
const char* const modes[] = {"COMPETITION", "MOTOR_TEST", "IR_PD_TEST", "BASIC_CHASE", "MANAGED_COMPETITION_TRIAL", "PD_CHASE", "GOAL_ATTACK_YELLOW", "GOAL_ATTACK_BLUE"};
const char* const fallbackNames[] = {"NONE", "MASK", "UNCOMMISSIONED_SWEEP", "RANGE", "AMBIGUOUS", "UNSUPPORTED_PWM"};
uint32_t stopCount[int(StopReason::Count)]{}, stopMs[int(StopReason::Count)]{}, reasonSince = 0;

void logEvent(uint32_t now) {
  if (decision.state == previousState && decision.reason == previousReason) return;
  if (previousState == State::Stop && previousReason != StopReason::None)
    stopMs[int(previousReason)] += now - reasonSince;
  reasonSince = now;
  if (decision.state == State::Stop) ++stopCount[int(decision.reason)];
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
#if RCJ_SERIAL_TRIAL && RCJ_START_WAIT_MS > 0
  if (!waiting && !report.busy() && startEventRead != startEventWrite) {
    const char* const messages[] = {"", "START_PENDING: 2000ms\n", "STARTED\n", "START_CANCELLED"};
    if (startEvents[startEventRead] == 3) {
      const char* const reasons[] = {"", "MANUAL_STOP", "MODE_CHANGED", "USB_DISCONNECTED", "CAMERA_NOT_READY", "IR_NOT_READY"};
      report.queue(snprintf(report.data, sizeof(report.data), "%s: %s\n", messages[3],
          reasons[startEventReasons[startEventRead]]));
    } else report.queue(snprintf(report.data, sizeof(report.data), "%s", messages[startEvents[startEventRead]]));
    startEventRead = (startEventRead + 1) % 8;
  } else
#endif
  if (!waiting && !report.busy() && (requestedReport || now - reportMs >= 1000)) {
    const int reason = int(decision.reason);
    const int yellow = visibleGoal(camera.yellow, now), blue = visibleGoal(camera.blue, now);
    uint32_t totalCount = 0, totalMs = reason ? now - reasonSince : 0;
    for (int i = 1; i < int(StopReason::Count); ++i) {
      totalCount += stopCount[i];
      totalMs += stopMs[i];
    }
    report.queue(snprintf(report.data, sizeof(report.data),
        "STATE=%s STOP_REASON=%s STOP_DETAIL=%s MODE=%s\n"
        "IR_LINK=%s IR_ANGLE=%d IR_STRENGTH=%u IR_AGE_MS=%ld IR_INVALID_COUNT=%lu IR_POLL_INTERVAL_MS=%lu IR_POLL_MAX_MS=%lu\n"
        "CAMERA_LINK=%s CAMERA_AGE_MS=%ld CAMERA_READY=%u CAMERA_SLOW=%u FRAME_MS=%u CAMERA_BAD=%lu CAMERA_PROTOCOL=%u LINE_HAZARD=%u\n"
        "WHITE_LINE=%u LINE_ANGLE=%d LINE_DISTANCE=%d SAFE_MASK=%04x ESCAPE_MASK=%04x REQUIRED_MASK=%04x BLOCKED_MASK=%04x PATH_MODE=%s FALLBACK_REASON=%s\n"
        "YELLOW_GOAL=%d BLUE_GOAL=%d TARGET_GOAL=%s OPPONENT_ANGLE=%d OWN_ANGLE=%d GOAL_QUALITY=%u,%u\n"
        "IR_RAW_ANGLE=%d IR_FILTERED_ANGLE=%d PD_ERROR=%d PD_P=%.2f PD_D_RAW=%.2f PD_D_FILTERED=%.2f PD_D=%.2f PD_TURN_RAW=%.2f PD_TURN_FILTERED=%.2f PD_TURN=%d BASE_PWM=%d PD_LEGACY=%u\n"
        "PWM_CALCULATED=%d,%d,%d,%d PWM_AFTER_SAFETY=%d,%d,%d,%d PWM_TRANSMITTED=%d,%d,%d,%d\n"
        "MOTOR_TX_COUNT=%lu PWM_ALT_TRANSMITTED=%u EVENT_DROPPED=%lu\nESCAPE_RX_TO_TX_US=%lu ESCAPE_RX_TO_TX_MAX_US=%lu\nSTOP_COUNT=%lu STOP_DURATION_MS=%lu\n",
        states[int(decision.state)], reasons[reason],
        decision.reason == StopReason::BallLost && irAngle == Unknown && irStrength
            ? "ir_vector_cancelled"
            : decision.reason == StopReason::NoEscape && decision.fallback == Fallback::Ambiguous
                ? "escape_both_candidates_bearing_unknown_or_lateral"
            : decision.reason == StopReason::NoEscape && !camera.safe
                ? "camera_reports_no_safe_sector"
                : decision.reason == StopReason::PathBlocked &&
                    (runMode == RunMode::BasicChase || runMode == RunMode::ManagedCompetition) && camera.version < 2
                    ? "camera_protocol_v2_or_later_required"
                : decision.reason == StopReason::PathBlocked &&
                    (runMode == RunMode::BasicChase || runMode == RunMode::ManagedCompetition) && (camera.flags & LineHazard)
                    ? "current_or_held_white_line_hazard"
                : decision.reason == StopReason::PathBlocked && decision.fallback == Fallback::Unsupported
                    ? "wheel_pattern_not_verified"
                : decision.reason == StopReason::PathBlocked && decision.blocked
                    ? "required_corridor_blocked_or_unobserved"
                : decision.reason == StopReason::PathBlocked && decision.fallback == Fallback::Unmeasured && !decision.required
                    ? "yaw_sweep_uncommissioned" : details[reason],
        modes[int(runMode)],
        irFresh && !irInvalid && !missedIr ? "OK" : "ERROR", irAngle < 360 ? int(irAngle) : -1, unsigned(irStrength),
        haveIr ? long(now - irMs) : -1L, (unsigned long)irBad,
        (unsigned long)pollInterval, (unsigned long)pollMax,
        cameraFresh ? "OK" : "ERROR", haveCamera ? long(now - cameraMs) : -1L,
        unsigned(bool(camera.flags & Ready)), unsigned(bool(camera.flags & Slow)),
        unsigned(camera.frameMs), (unsigned long)cameraBad, unsigned(camera.version), unsigned(bool(camera.flags & LineHazard)),
        unsigned(camera.line != Unknown || (camera.flags & Escape)),
        camera.line == Unknown ? -1 : int(camera.line), camera.lineCm == 255 ? -1 : int(camera.lineCm),
        unsigned(camera.safe), unsigned(camera.escape), unsigned(decision.required), unsigned(decision.blocked),
        paths[int(decision.path)], fallbackNames[int(decision.fallback)],
        yellow, blue, targetBlue ? "BLUE" : "YELLOW",
        targetBlue ? blue : yellow, targetBlue ? yellow : blue,
        yellow < 0 ? 0u : unsigned(camera.yellow.confidence), blue < 0 ? 0u : unsigned(camera.blue.confidence),
        irAngle < 360 ? int(irAngle) : -1, pd.initialized ? pd.rawAngle : -1,
        pd.error, double(pd.p), double(pd.dRaw), double(pd.d), double(pd.d), double(pd.turnRaw),
        double(pd.filteredTurn), pd.turn, pd.basePwm, unsigned(pd.legacy),
        pd.pwm[0], pd.pwm[1], pd.pwm[2], pd.pwm[3],
        decision.pwm[0], decision.pwm[1], decision.pwm[2], decision.pwm[3],
        transmitted[0], transmitted[1], transmitted[2], transmitted[3],
        (unsigned long)motorTxCount, unsigned(alternateSent), (unsigned long)eventDropped,
        (unsigned long)escapeRxToTxUs, (unsigned long)escapeRxToTxMaxUs,
        (unsigned long)totalCount, (unsigned long)totalMs));
    for (int i = 1; i < int(StopReason::Count) && report.length < sizeof(report.data) - 1; ++i) {
      if (!stopCount[i]) continue;
      const int used = snprintf(report.data + report.length, sizeof(report.data) - report.length,
          "%s COUNT=%lu DURATION_MS=%lu\n", reasons[i], (unsigned long)stopCount[i],
          (unsigned long)(stopMs[i] + (reason == i ? now - reasonSince : 0)));
      if (used > 0) report.queue(int(report.length) + used);
    }
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
    if (report.length < sizeof(report.data) - 1) {
      const int used = snprintf(report.data + report.length, sizeof(report.data) - report.length,
          "BENCH_PATTERN=%d BENCH_SIGN=%d BENCH_ACTIVE=%u BENCH_START_MS=%lu BENCH_ELAPSED_MS=%lu BENCH_LAST_PWM=%d,%d,%d,%d\n",
          benchWheel + 1, benchSign, unsigned(benchArmed), (unsigned long)benchMs,
          (unsigned long)(benchArmed ? now - benchMs : benchLastDuration),
          benchLastPwm[0], benchLastPwm[1], benchLastPwm[2], benchLastPwm[3]);
      if (used > 0) report.queue(int(report.length) + used);
    }
#endif
#if RCJ_PHASE3_ORBIT
    if (report.length < sizeof(report.data) - 1) {
      const int used = snprintf(report.data + report.length, sizeof(report.data) - report.length,
          "ORBIT_ACTIVE=%u ORBIT_ELAPSED_MS=%lu ORBIT_DIRECTION=%d ORBIT_COOLDOWN=%u\n",
          unsigned(orbit.active), (unsigned long)(orbit.active ? now - orbit.started : 0),
          int(orbit.direction), unsigned(orbit.cooling));
      if (used > 0) report.queue(int(report.length) + used);
    }
#endif
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
