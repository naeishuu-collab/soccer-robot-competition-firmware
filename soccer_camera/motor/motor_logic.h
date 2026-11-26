// MOTOR制御ロジック。motor.inoから呼ばれる純粋C++部で、方向合成・出力制限・逆転保護を行う。
// MAINが送る「前を0度とした方向・出力・旋回量」を4輪それぞれの速度/PWMへ変換する。
// 数値は未校正の暫定値を含むため、実機設定は docs/UPGRADE_2026-11-26_JA.md と照合する。
// Wheel model for the high-minimum-duty motors. Pure C++ (host tested).
// Every number in kWheels is a PLACEHOLDER until measured on the robot (see docs/UPGRADE_2026-11-26_JA.md).
#pragma once
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace wheel {

constexpr float kDegToRad = 0.01745329252f;
constexpr int kDutyLimit = 210;               // Do not raise without hardware verification.
constexpr uint32_t kReversePauseMs = 200;     // Reverse protection, never bypassed by boost.
constexpr uint32_t kBoostLimitMs = 150;       // Hard ceiling for any configured boost.
// A wheel stopped for less than this is still turning and gets no boost; this also bounds boost duty.
constexpr uint32_t kMovingMemoryMs = 80;
constexpr uint32_t kTickMs = 10;
// Start ramp from rest, as in the bench test (100 ms, 10 ms updates). Skipped for line escapes.
constexpr uint32_t kStartRampMs = 100;
// Short brake after stop(). 0 = coast (default): current, heat and distance are unmeasured.
constexpr uint32_t kBrakeLimitMs = 100;
constexpr float kMaxDirectionErrorDeg = 30;   // Larger error: do not move at all.
constexpr float kMaxAvoidErrorDeg = 10;       // Line escape: stricter, and no rotation mixed in.

// speed is permille of the speed reached at the highest table duty.
struct SpeedPoint { uint8_t duty; uint16_t speed; };

struct DirectionConfig {
  uint8_t startDuty;      // duty that breaks a stationary wheel loose
  uint8_t holdDuty;       // lowest duty that keeps a turning wheel turning
  uint16_t gainPermille;  // per-direction correction of the duty above holdDuty
  SpeedPoint table[3];    // duty and speed both strictly increasing, table[0].duty == holdDuty
};

struct WheelConfig {
  int16_t degrees;        // drive axis angle
  int8_t polarity;        // +1: positive geometric speed drives IN1
  DirectionConfig forward, reverse;  // "forward" = IN1 PWM
  uint8_t normalDuty;     // informational reference for tuning
  uint8_t maxDuty;
  uint8_t accelStep;      // duty per 10 ms tick
  uint8_t decelStep;
  uint16_t boostMs;
};

// Channel order (see motor.ino): M3 (GPIO6,7), M2 (12,13), M1 (11,10), M4 (9,8). "forward" = channel IN1,
// which for M1 and M4 is the bench "-" direction.
// 160 moved the robot in four directions on 2026-11-27; it is NOT a measured minimum start or hold duty.
// Table is assumed proportional (speed request 160 -> PWM 160) until the speeds are measured.
#define WHEEL_DIRECTION {180, 160, 1000, {{160, 762}, {185, 881}, {210, 1000}}}
#define WHEEL_DEFAULT(deg, pol) {deg, pol, WHEEL_DIRECTION, WHEEL_DIRECTION, 160, 210, 5, 10, 60}
constexpr WheelConfig kWheels[4] = {
  WHEEL_DEFAULT(315, 1), WHEEL_DEFAULT(45, 1), WHEEL_DEFAULT(135, -1), WHEEL_DEFAULT(225, -1),
};
#undef WHEEL_DEFAULT
#undef WHEEL_DIRECTION

inline const DirectionConfig &side(const WheelConfig &w, float sign) { return sign < 0 ? w.reverse : w.forward; }
inline float minSpeed(const DirectionConfig &d) { return d.table[0].speed / 1000.0f; }

// Inverse of the measured PWM -> speed table. speed is 0..1.
inline int dutyForSpeed(const WheelConfig &w, float speed) {
  if (speed == 0) return 0;
  const DirectionConfig &d = side(w, speed);
  const float want = fabsf(speed) * 1000.0f;
  float duty = d.table[2].duty;
  if (want <= d.table[0].speed) {
    duty = d.table[0].duty;
  } else {
    for (int i = 1; i < 3; ++i) {
      if (want > d.table[i].speed) continue;
      const SpeedPoint &a = d.table[i - 1], &b = d.table[i];
      duty = a.duty + (b.duty - a.duty) * (want - a.speed) / float(b.speed - a.speed);
      break;
    }
  }
  duty = d.holdDuty + (duty - d.holdDuty) * d.gainPermille / 1000.0f;
  if (duty < d.holdDuty) duty = d.holdDuty;
  const int limit = w.maxDuty < kDutyLimit ? w.maxDuty : kDutyLimit;
  const int result = int(lroundf(duty));
  return (speed < 0 ? -1 : 1) * (result > limit ? limit : result);
}

struct Mix {
  float speed[4] = {};       // realizable wheel speed, IN1-positive, -1..1
  float errorDeg = 0;        // translation direction error actually commanded
  bool feasible = true;      // false: nothing within the error limit, all wheels stopped
};

// direction: clockwise from front. power: 0..210 speed request. turn: duty-scaled rotation request.
// previous: last Mix speeds, used as hysteresis so a wheel near zero does not chatter.
inline Mix solve(const WheelConfig (&wheels)[4], int direction, int power, int turn, const float previous[4],
                 float maxError = kMaxDirectionErrorDeg) {
  Mix out;
  const float s = fminf(fmaxf(power / float(kDutyLimit), 0), 1);
  const float r = fminf(fmaxf(turn / float(kDutyLimit), -1), 1);
  if (s == 0 && r == 0) return out;

  float g[4], peak = 0;  // geometric wheel speed: cos(direction - wheel) * s + r
  for (int i = 0; i < 4; ++i) {
    g[i] = s * cosf((direction - wheels[i].degrees) * kDegToRad) + r;
    peak = fmaxf(peak, fabsf(g[i]));
  }
  // The strongest wheel gets the requested speed, but never below what the motors can sustain.
  float floorSpeed = 0;
  for (int i = 0; i < 4; ++i) floorSpeed = fmaxf(floorSpeed, minSpeed(side(wheels[i], g[i] * wheels[i].polarity)));
  const float wantedPeak = fminf(1.0f, fmaxf(fmaxf(s, fabsf(r)), floorSpeed));
  // One common multiplier first: it preserves the ratios whenever the small wheels stay above their floor.
  float smallest = peak;
  for (int i = 0; i < 4; ++i) if (fabsf(g[i]) > 0.1f * peak) smallest = fminf(smallest, fabsf(g[i]));
  float scale = wantedPeak / peak;
  if (smallest * scale < floorSpeed) scale = fminf(floorSpeed / smallest, 1.0f / peak);
  for (int i = 0; i < 4; ++i) g[i] *= scale;

  // Body motion of a wheel set: vx = sum(g*sin)/2, vy = sum(g*cos)/2, r = sum(g)/4.
  auto body = [&](const float *w, float &vx, float &vy, float &rot) {
    vx = vy = rot = 0;
    for (int i = 0; i < 4; ++i) {
      vx += w[i] * sinf(wheels[i].degrees * kDegToRad) / 2;
      vy += w[i] * cosf(wheels[i].degrees * kDegToRad) / 2;
      rot += w[i] / 4;
    }
  };
  float tx, ty, tr;
  body(g, tx, ty, tr);

  // Wheels below the floor can only be stopped or run at the floor: try every combination.
  float low[4];
  int choices[4], total = 1;
  for (int i = 0; i < 4; ++i) {
    low[i] = minSpeed(side(wheels[i], g[i] * wheels[i].polarity));
    const bool below = fabsf(g[i]) > 1e-4f && fabsf(g[i]) < low[i] - 1e-4f;
    choices[i] = below ? 2 : 1;
    total *= choices[i];
  }
  float best[4] = {}, bestCost = 1e9f, bestError = 180;
  for (int combo = 0; combo < total; ++combo) {
    float w[4];
    int rest = combo;
    float switching = 0;
    for (int i = 0; i < 4; ++i) {
      const int pick = rest % choices[i];
      rest /= choices[i];
      if (choices[i] == 1) w[i] = fabsf(g[i]) <= 1e-4f ? 0 : g[i];
      else w[i] = pick ? copysignf(low[i], g[i]) : 0;
      if (previous && (previous[i] != 0) != (w[i] != 0)) switching += 0.02f;
    }
    float vx, vy, rot;
    body(w, vx, vy, rot);
    float error = 0;
    if (s > 0) {
      if (hypotf(vx, vy) < 1e-3f) continue;
      error = fabsf(atan2f(vx * ty - vy * tx, vx * tx + vy * ty)) / kDegToRad;
    } else if (rot * tr <= 0) {
      continue;
    }
    // Wheels fighting each other (the null-space pattern) waste torque.
    float fight = 0;
    for (int i = 0; i < 4; ++i) fight += w[i] * sinf(2 * wheels[i].degrees * kDegToRad);
    fight = fabsf(fight) / 4;
    // Unwanted rotation is weighted heavily: without a gyro nothing else corrects it.
    const float cost = error / 45.0f + 4 * fabsf(rot - tr) + 0.25f * fight + switching +
                       0.2f * fabsf(hypotf(vx, vy) - hypotf(tx, ty));
    if (cost < bestCost) {
      bestCost = cost;
      bestError = error;
      for (int i = 0; i < 4; ++i) best[i] = w[i];
    }
  }
  out.errorDeg = bestError;
  out.feasible = bestCost < 1e9f && bestError <= maxError;
  // Heading correction must never block a translation.
  if (!out.feasible && turn && s > 0) return solve(wheels, direction, power, 0, previous, maxError);
  if (out.feasible) for (int i = 0; i < 4; ++i) out.speed[i] = best[i] * wheels[i].polarity;
  return out;
}

// Stationary -> boost -> running, with a coordinated reverse pause.
class Drive {
public:
  explicit Drive(const WheelConfig (&wheels)[4] = kWheels) : wheels_(wheels) {}
  uint32_t rampMs = kStartRampMs;
  uint32_t brakeMs = 0;
  bool braking(uint32_t now) const {
    return braked_ && uint32_t(now - brakeStartMs_) < (brakeMs < kBrakeLimitMs ? brakeMs : kBrakeLimitMs);
  }

  void command(int direction, int power, int turn, uint32_t now, bool avoid = false) {
    const Mix next = avoid ? solve(wheels_, direction, power, 0, nullptr, kMaxAvoidErrorDeg)
                           : solve(wheels_, direction, power, turn, mix_.speed);
    mix_ = next;
    avoid_ = avoid;
    waiting_ = false;  // recomputed below: returning to the old polarity needs no wait
    bool any = false;
    for (int i = 0; i < 4; ++i) {
      target_[i] = dutyForSpeed(wheels_[i], mix_.speed[i]);
      any = any || target_[i];
    }
    if (!any) { stop(now); return; }
    // Reversing one wheel alone bends the path, so all four wait together.
    for (int i = 0; i < 4; ++i) {
      if (target_[i] * lastSign_[i] < 0 && (current_[i] || uint32_t(now - stoppedMs_[i]) < kReversePauseMs)) {
        const uint32_t until = current_[i] ? now + kReversePauseMs : stoppedMs_[i] + kReversePauseMs;
        halt(now);
        if (!waiting_ || int32_t(until - waitUntil_) > 0) waitUntil_ = until;
        waiting_ = true;
      }
    }
    update(now);
  }

  void stop(uint32_t now) {
    const bool moving = current_[0] || current_[1] || current_[2] || current_[3];
    for (int i = 0; i < 4; ++i) target_[i] = 0;
    halt(now);
    if (moving && brakeMs) {
      braked_ = true;
      brakeStartMs_ = now;
    }
  }

  void update(uint32_t now) {
    if (braked_ && !braking(now)) braked_ = false;
    if (waiting_) {
      if (int32_t(now - waitUntil_) < 0) return;
      waiting_ = false;
    }
    const bool tick = uint32_t(now - tickMs_) >= kTickMs;
    if (tick) tickMs_ = now;
    for (int i = 0; i < 4; ++i) {
      const WheelConfig &w = wheels_[i];
      const int target = target_[i];
      if (!target) { if (current_[i]) haltWheel(i, now); continue; }
      const int sign = target < 0 ? -1 : 1;
      const DirectionConfig &d = side(w, float(sign));
      if (!current_[i]) {
        // "Still turning" needs a recent stop AND a previous run long enough to have broken loose.
        const bool stationary = !lastSign_[i] || !spun_[i] || sign != lastSign_[i] ||
                                uint32_t(now - stoppedMs_[i]) >= kMovingMemoryMs;
        boosting_[i] = stationary && w.boostMs;
        // Wheels started together share the ramp factor, so their ratio holds while accelerating.
        ramp_[i] = stationary && !avoid_ ? rampMs : 0;
        startedMs_[i] = now;
        current_[i] = sign * (ramp_[i] ? rampDuty(startDutyNow(i, d), kTickMs, ramp_[i]) : startDutyNow(i, d));
        lastSign_[i] = sign;
        braked_ = false;
        continue;
      }
      if (ramp_[i]) {
        const uint32_t elapsed = uint32_t(now - startedMs_[i]);
        if (elapsed < ramp_[i] && !avoid_) {
          current_[i] = sign * rampDuty(startDutyNow(i, d), elapsed + kTickMs, ramp_[i]);
        } else {
          current_[i] = sign * startDutyNow(i, d);
          startedMs_[i] = now;  // the boost time counts from the end of the ramp
          ramp_[i] = 0;
        }
        continue;
      }
      if (boosting_[i]) {
        if (uint32_t(now - startedMs_[i]) < boostLimit(w)) continue;
        boosting_[i] = false;
        // Drop straight to a lower running duty; a higher one is approached by the ramp.
        if (abs_(target) <= abs_(current_[i])) { current_[i] = target; continue; }
      }
      // Escaping a line: a wheel faster than its target would bend the path, so drop it at once.
      if (avoid_ && abs_(current_[i]) > abs_(target)) { current_[i] = target; continue; }
      if (!tick) continue;
      const int now_ = abs_(current_[i]), want = abs_(target);
      int duty = want > now_ ? (now_ + w.accelStep < want ? now_ + w.accelStep : want)
                             : (now_ - w.decelStep > want ? now_ - w.decelStep : want);
      if (duty < d.holdDuty) duty = d.holdDuty;  // never dwell where the wheel cannot turn
      current_[i] = sign * duty;
    }
  }

  int duty(int i) const { return current_[i]; }
  int target(int i) const { return target_[i]; }
  const Mix &mix() const { return mix_; }

private:
  static int abs_(int v) { return v < 0 ? -v : v; }
  int startDutyNow(int i, const DirectionConfig &d) const {
    const int target = abs_(target_[i]);
    const int duty = boosting_[i] && d.startDuty > target ? d.startDuty : target;
    return duty > kDutyLimit ? kDutyLimit : duty;
  }
  static int rampDuty(int full, uint32_t elapsed, uint32_t length) {
    const int duty = elapsed >= length ? full : int(full * elapsed / length);
    return duty < 1 ? 1 : duty;
  }
  static uint32_t boostLimit(const WheelConfig &w) { return w.boostMs < kBoostLimitMs ? w.boostMs : kBoostLimitMs; }
  void haltWheel(int i, uint32_t now) {
    spun_[i] = !ramp_[i] && uint32_t(now - startedMs_[i]) >= boostLimit(wheels_[i]);
    ramp_[i] = 0;
    current_[i] = 0;
    boosting_[i] = false;
    stoppedMs_[i] = now;
  }
  void halt(uint32_t now) { for (int i = 0; i < 4; ++i) if (current_[i]) haltWheel(i, now); }

  const WheelConfig (&wheels_)[4];
  Mix mix_;
  int target_[4] = {}, current_[4] = {};
  int lastSign_[4] = {};
  uint32_t stoppedMs_[4] = {}, startedMs_[4] = {}, ramp_[4] = {}, waitUntil_ = 0, tickMs_ = 0, brakeStartMs_ = 0;
  bool boosting_[4] = {}, spun_[4] = {}, waiting_ = false, avoid_ = false, braked_ = false;
};

// USB bench diagnostics. Nothing moves until "diag" is typed, and every motion expires by itself.
// Bench motor M1..M4 -> channel, and the channel sign that equals the bench "+".
constexpr int kChannelOfMotor[4] = {2, 1, 0, 3};
constexpr int kSignOfMotor[4] = {-1, 1, 1, -1};

class Diag {
public:
  static constexpr uint32_t kMaxMs = 1500, kDefaultMs = 700, kKickMs = 100;

  bool active() const { return active_; }
  bool braking(uint32_t now) const { return !running_ && drive_.braking(now); }

  // Returns a message for the serial monitor.
  const char *line(const char *text, uint32_t now) {
    if (!strcmp(text, "diag")) {
      if (!active_) enteredMs_ = now;
      active_ = true;
      end(now);
      return "diag on: p|k|m|s|exit";
    }
    if (!active_) return "type diag first";
    if (!strcmp(text, "exit")) { end(now); active_ = false; return "diag off"; }
    if (!strcmp(text, "s")) { end(now); return "stop"; }
    if ((text[0] == 'p' || text[0] == 'k' || text[0] == 'm') && uint32_t(now - enteredMs_) < kReversePauseMs)
      return "reverse pause, retry";
    int a = 0, b = 0, c = 0, ms = kDefaultMs;
    char way = 0;
    if (sscanf(text, "b %d", &a) == 1) {
      if (a < 0 || a > int(kBrakeLimitMs)) return "out of range";
      drive_.brakeMs = uint32_t(a);
      return "brake time set for diag moves";
    }
    // Bench names: "p M1 + 160", "k M1 + 180 160".
    if (sscanf(text, "p M%d %c %d %d", &a, &way, &b, &ms) >= 3) {
      if (!benchMotor(a, way)) return "out of range";
      return pulse(a, way, b, b, ms, now);
    }
    ms = kDefaultMs;
    if (sscanf(text, "k M%d %c %d %d %d", &a, &way, &b, &c, &ms) >= 4) {
      if (!benchMotor(a, way)) return "out of range";
      return pulse(a, way, b, c, ms, now);
    }
    ms = kDefaultMs;
    if (sscanf(text, "p %d %c %d %d", &a, &way, &b, &ms) >= 3) return pulse(a, way, b, b, ms, now);
    ms = kDefaultMs;
    if (sscanf(text, "k %d %c %d %d %d", &a, &way, &b, &c, &ms) >= 4) return pulse(a, way, b, c, ms, now);
    ms = kDefaultMs;
    if (sscanf(text, "m %d %d %d %d", &a, &b, &c, &ms) >= 3) {
      if (a < 0 || a >= 360 || b < 0 || b > kDutyLimit || c < -40 || c > 40 || !validMs(ms)) return "out of range";
      end(now);
      if (pulsed_ && uint32_t(now - pulseEndMs_) < kReversePauseMs) return "reverse pause, retry";
      wheel_ = -1;
      drive_.command(a, b, c, now);
      start(now, ms);
      if (!drive_.mix().feasible) return "infeasible: stopped";
      snprintf(reply_, sizeof(reply_), "move, direction error %d deg", int(drive_.mix().errorDeg + 0.5f));
      return reply_;
    }
    return "unknown command";
  }

  void update(uint32_t now) {
    if (running_ && uint32_t(now - startMs_) >= durationMs_) end(now);
    if (running_ && wheel_ < 0) drive_.update(now);
  }

  int duty(int i, uint32_t now) const {
    if (!running_) return 0;
    if (wheel_ < 0) return drive_.duty(i);
    if (i != wheel_) return 0;
    return sign_ * (uint32_t(now - startMs_) < kKickMs ? first_ : then_);
  }

private:
  static bool validMs(int ms) { return ms > 0 && ms <= int(kMaxMs); }
  // Rewrites M-number and +/- into channel and f/r.
  static bool benchMotor(int &motor, char &way) {
    if (motor < 1 || motor > 4 || (way != '+' && way != '-')) return false;
    way = (way == '+' ? 1 : -1) * kSignOfMotor[motor - 1] > 0 ? 'f' : 'r';
    motor = kChannelOfMotor[motor - 1];
    return true;
  }
  const char *pulse(int w, char way, int first, int then, int ms, uint32_t now) {
    if (w < 0 || w > 3 || (way != 'f' && way != 'r') || first < 0 || first > kDutyLimit || then < 0 ||
        then > kDutyLimit || !validMs(ms)) return "out of range";
    const int sign = way == 'f' ? 1 : -1;
    end(now);
    if (lastSign_[w] == -sign && uint32_t(now - endMs_[w]) < kReversePauseMs) return "reverse pause, retry";
    wheel_ = w; sign_ = sign; first_ = first; then_ = then;
    lastSign_[w] = sign;
    start(now, ms);
    return "pulse";
  }
  void start(uint32_t now, int ms) { running_ = true; startMs_ = now; durationMs_ = uint32_t(ms); }
  void end(uint32_t now) {
    if (running_ && wheel_ >= 0) {
      endMs_[wheel_] = pulseEndMs_ = now;
      pulsed_ = true;
    } else if (running_) {
      for (int i = 0; i < 4; ++i) {
        const int d = drive_.duty(i);
        if (d) { lastSign_[i] = d < 0 ? -1 : 1; endMs_[i] = now; }
      }
    }
    running_ = false;
    drive_.stop(now);
  }

  Drive drive_;
  bool active_ = false, running_ = false, pulsed_ = false;
  int wheel_ = -1, sign_ = 1, first_ = 0, then_ = 0, lastSign_[4] = {};
  uint32_t startMs_ = 0, durationMs_ = 0, endMs_[4] = {}, pulseEndMs_ = 0, enteredMs_ = 0;
  char reply_[48] = {};
};

}  // namespace wheel
