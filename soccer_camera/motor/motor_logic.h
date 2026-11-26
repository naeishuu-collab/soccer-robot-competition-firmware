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
constexpr uint32_t kBoostRearmMs = 300;       // A stop/start chatter cannot re-boost faster than this.
constexpr uint32_t kMovingMemoryMs = 80;      // A wheel stopped for less than this is still turning.
constexpr uint32_t kTickMs = 10;
constexpr float kMaxDirectionErrorDeg = 30;   // Larger error: do not move at all.

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

// Order: GPIO6/7, GPIO12/13, GPIO11/10, GPIO9/8. 160 is the provisional baseline, not a measured value.
#define WHEEL_DIRECTION {180, 160, 1000, {{160, 650}, {185, 830}, {210, 1000}}}
#define WHEEL_DEFAULT(deg, pol) {deg, pol, WHEEL_DIRECTION, WHEEL_DIRECTION, 160, 210, 5, 10, 60}
constexpr WheelConfig kWheels[4] = {
  WHEEL_DEFAULT(45, 1), WHEEL_DEFAULT(225, -1), WHEEL_DEFAULT(315, 1), WHEEL_DEFAULT(135, -1),
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
inline Mix solve(const WheelConfig (&wheels)[4], int direction, int power, int turn, const float previous[4]) {
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
    const float fight = fabsf(w[0] + w[1] - w[2] - w[3]) / 4;
    const float cost = error / 45.0f + fabsf(rot - tr) + 0.25f * fight + switching +
                       0.2f * fabsf(hypotf(vx, vy) - hypotf(tx, ty));
    if (cost < bestCost) {
      bestCost = cost;
      bestError = error;
      for (int i = 0; i < 4; ++i) best[i] = w[i];
    }
  }
  out.errorDeg = bestError;
  out.feasible = bestCost < 1e9f && bestError <= kMaxDirectionErrorDeg;
  if (out.feasible) for (int i = 0; i < 4; ++i) out.speed[i] = best[i] * wheels[i].polarity;
  return out;
}

// Stationary -> boost -> running, with a coordinated reverse pause.
class Drive {
public:
  explicit Drive(const WheelConfig (&wheels)[4] = kWheels) : wheels_(wheels) {}

  void command(int direction, int power, int turn, uint32_t now) {
    const Mix next = solve(wheels_, direction, power, turn, mix_.speed);
    mix_ = next;
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
    for (int i = 0; i < 4; ++i) target_[i] = 0;
    halt(now);
  }

  void update(uint32_t now) {
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
        const bool stationary = !lastSign_[i] || uint32_t(now - stoppedMs_[i]) >= kMovingMemoryMs ||
                                sign != lastSign_[i];
        const bool armed = !boosted_[i] || uint32_t(now - boostStartMs_[i]) >= kBoostRearmMs;
        boosting_[i] = stationary && armed && w.boostMs;
        if (boosting_[i]) { boosted_[i] = true; boostStartMs_[i] = now; }
        int duty = boosting_[i] ? (d.startDuty > abs_(target) ? d.startDuty : abs_(target)) : abs_(target);
        if (duty > kDutyLimit) duty = kDutyLimit;
        current_[i] = sign * duty;
        lastSign_[i] = sign;
        continue;
      }
      if (boosting_[i]) {
        const uint32_t limit = w.boostMs < kBoostLimitMs ? w.boostMs : kBoostLimitMs;
        if (uint32_t(now - boostStartMs_[i]) < limit) continue;
        boosting_[i] = false;
        current_[i] = target;  // drop straight to the running duty
        continue;
      }
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
  void haltWheel(int i, uint32_t now) { current_[i] = 0; boosting_[i] = false; stoppedMs_[i] = now; }
  void halt(uint32_t now) { for (int i = 0; i < 4; ++i) if (current_[i]) haltWheel(i, now); }

  const WheelConfig (&wheels_)[4];
  Mix mix_;
  int target_[4] = {}, current_[4] = {};
  int lastSign_[4] = {};
  uint32_t stoppedMs_[4] = {}, boostStartMs_[4] = {}, waitUntil_ = 0, tickMs_ = 0;
  bool boosting_[4] = {}, boosted_[4] = {}, waiting_ = false;
};

// USB bench diagnostics. Nothing moves until "diag" is typed, and every motion expires by itself.
class Diag {
public:
  static constexpr uint32_t kMaxMs = 1500, kDefaultMs = 700, kKickMs = 100;

  bool active() const { return active_; }

  // Returns a message for the serial monitor.
  const char *line(const char *text, uint32_t now) {
    if (!strcmp(text, "diag")) { active_ = true; end(now); return "diag on: p|k|m|s|exit"; }
    if (!active_) return "type diag first";
    if (!strcmp(text, "exit")) { end(now); active_ = false; return "diag off"; }
    if (!strcmp(text, "s")) { end(now); return "stop"; }
    int a = 0, b = 0, c = 0, ms = kDefaultMs;
    char way = 0;
    if (sscanf(text, "p %d %c %d %d", &a, &way, &b, &ms) >= 3) return pulse(a, way, b, b, ms, now);
    ms = kDefaultMs;
    if (sscanf(text, "k %d %c %d %d %d", &a, &way, &b, &c, &ms) >= 4) return pulse(a, way, b, c, ms, now);
    ms = kDefaultMs;
    if (sscanf(text, "m %d %d %d %d", &a, &b, &c, &ms) >= 3) {
      if (a < 0 || a >= 360 || b < 0 || b > kDutyLimit || c < -40 || c > 40 || !validMs(ms)) return "out of range";
      end(now);
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
    if (running_ && wheel_ >= 0) endMs_[wheel_] = now;
    running_ = false;
    drive_.stop(now);
  }

  Drive drive_;
  bool active_ = false, running_ = false;
  int wheel_ = -1, sign_ = 1, first_ = 0, then_ = 0, lastSign_[4] = {};
  uint32_t startMs_ = 0, durationMs_ = 0, endMs_[4] = {};
  char reply_[48] = {};
};

}  // namespace wheel
