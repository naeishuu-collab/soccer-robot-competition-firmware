// High-minimum-duty wheel model and bench diagnostics.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <initializer_list>
#include "motor_logic.h"
#include <assert.h>
using namespace wheel;

static bool noLowDuty(const Drive &d) {
  for (int i = 0; i < 4; ++i)
    if (d.duty(i) && (abs(d.duty(i)) < 160 || abs(d.duty(i)) > kDutyLimit)) return false;
  return true;
}

int main() {
  const WheelConfig &w = kWheels[0];
  // Inverse table, never below hold, never above the limit.
  assert(dutyForSpeed(w, 0) == 0);
  assert(dutyForSpeed(w, 0.1f) == 160 && dutyForSpeed(w, -0.1f) == -160);
  assert(dutyForSpeed(w, 0.65f) == 160 && dutyForSpeed(w, 0.83f) == 185 && dutyForSpeed(w, 1.0f) == 210);
  assert(dutyForSpeed(w, 2.0f) == 210);

  // Every direction and power: realizable speeds only, bounded and reported direction error.
  for (int dir = 0; dir < 360; ++dir) {
    for (int power : {1, 80, 120, 160, 210}) {
      for (int turn : {-40, 0, 15, 40}) {
        const Mix m = solve(kWheels, dir, power, turn, nullptr);
        for (int i = 0; i < 4; ++i) {
          assert(fabsf(m.speed[i]) <= 1.0001f);
          assert(m.speed[i] == 0 || fabsf(m.speed[i]) >= 0.649f);
        }
        assert(m.feasible && m.errorDeg <= kMaxDirectionErrorDeg);
        if (turn == 0) {
          assert(m.errorDeg <= 23);
          float spin = 0;
          for (int i = 0; i < 4; ++i) spin += m.speed[i] * kWheels[i].polarity / 4;
          assert(fabsf(spin) < 0.001f);  // translation never adds rotation
        }
        if (!m.feasible) for (int i = 0; i < 4; ++i) assert(m.speed[i] == 0);
      }
    }
  }
  // Cardinal directions are exact with identical wheel speeds, even for an unreachable low request.
  for (int dir : {0, 90, 180, 270}) {
    const Mix m = solve(kWheels, dir, 40, 0, nullptr);
    assert(m.errorDeg < 0.1f);
    for (int i = 0; i < 4; ++i) assert(fabsf(fabsf(m.speed[i]) - fabsf(m.speed[0])) < 1e-4f);
  }
  // Diagonals stop the perpendicular pair instead of feeding it a stall duty.
  Mix diag45 = solve(kWheels, 45, 210, 0, nullptr);
  assert(diag45.errorDeg < 0.1f && diag45.speed[2] == 0 && diag45.speed[3] == 0);
  // Ratio is preserved when both wheel groups are above the floor.
  Mix ratio = solve(kWheels, 8, 210, 0, nullptr);
  const float expected = cosf(37 * kDegToRad) / cosf(53 * kDegToRad);
  assert(ratio.errorDeg < 0.1f && fabsf(fabsf(ratio.speed[0] / ratio.speed[2]) - expected) < 0.01f);
  // Pure rotation runs at a speed the wheels can hold.
  Mix spin = solve(kWheels, 0, 0, 10, nullptr);
  assert(spin.feasible && spin.speed[0] > 0.64f && spin.speed[1] < -0.64f);

  // Boost from rest, then the running duty; 120 requests never reach the pins as 120.
  Drive d;
  d.command(0, 120, 0, 1000);
  assert(d.duty(0) == 180 && noLowDuty(d));
  d.update(1050);
  assert(d.duty(0) == 180);
  d.update(1060);
  assert(d.duty(0) == d.target(0) && d.duty(0) == 160);
  d.command(0, 210, 0, 1070);
  assert(d.duty(0) == 165);  // accel limit
  d.command(0, 160, 0, 1080);
  assert(d.duty(0) >= 160 && noLowDuty(d));
  // Reverse: everything stops for the whole pause, boost cannot bypass it.
  d.command(180, 160, 0, 1100);
  for (int i = 0; i < 4; ++i) assert(d.duty(i) == 0);
  d.command(180, 160, 0, 1299);
  assert(d.duty(0) == 0);
  d.update(1300);
  assert(d.duty(0) < -159);
  // Stop then opposite direction still waits; returning to the old direction does not.
  d.stop(1400);
  d.command(0, 160, 0, 1410);
  assert(d.duty(0) == 0);
  d.command(180, 160, 0, 1420);
  assert(d.duty(0) < 0);
  // A still-turning wheel is not boosted again; a stationary one always is (no stall at hold duty).
  Drive c;
  c.command(0, 160, 0, 1000);
  c.update(1070);
  c.stop(1100);
  c.command(0, 160, 0, 1130);
  assert(c.duty(0) == c.target(0) && c.duty(0) < 180);
  c.stop(1200);
  c.command(0, 160, 0, 1300);
  assert(c.duty(0) == 180);
  // millis wrap during a reverse pause.
  Drive wrap;
  wrap.command(0, 160, 0, UINT32_MAX - 50);
  wrap.command(180, 160, 0, UINT32_MAX - 10);
  wrap.update(100);
  assert(wrap.duty(0) == 0);
  wrap.update(190);
  assert(wrap.duty(0) < 0);

  // Achieved motion reconstructed from the PWM actually requested (through the model table).
  int usable = 0;
  float worst[2] = {};
  for (int avoid = 0; avoid < 2; ++avoid) {
    for (int dir = 0; dir < 360; ++dir) {
      for (int power : {137, 175, 195}) {
        Drive a;
        a.command(dir, power, avoid ? 30 : 0, 1000, avoid != 0);
        a.update(1100);
        float x = 0, y = 0, spin = 0;
        bool any = false;
        for (int i = 0; i < 4; ++i) {
          const int duty = a.duty(i);
          if (!duty) continue;
          any = true;
          const SpeedPoint *p = kWheels[i].forward.table;
          const int d = abs(duty);
          const int k = d <= p[1].duty ? 0 : 1;
          const float speed = (p[k].speed + float(p[k + 1].speed - p[k].speed) * (d - p[k].duty) /
                               (p[k + 1].duty - p[k].duty)) / 1000.0f;
          const float g = (duty < 0 ? -speed : speed) * kWheels[i].polarity;
          x += g * sinf(kWheels[i].degrees * kDegToRad) / 2;
          y += g * cosf(kWheels[i].degrees * kDegToRad) / 2;
          spin += g / 4;
        }
        if (!any) { assert(avoid); continue; }
        const float error = fabsf(remainderf(atan2f(x, y) / kDegToRad - dir, 360));
        worst[avoid] = fmaxf(worst[avoid], error);
        assert(error <= (avoid ? kMaxAvoidErrorDeg : 23.0f) + 0.5f);
        assert(fabsf(spin) < 0.01f);
        if (dir % 45 == 0) { assert(error < 0.5f); }
        if (avoid && power == 175) ++usable;
      }
    }
  }
  printf("achieved error: normal %.1f deg, avoid %.1f deg; avoid usable directions %d/360\n", worst[0], worst[1], usable);

  // Random commands, stops and gaps: duty stays in band and no wheel flips polarity without the pause.
  Drive f;
  uint32_t now = UINT32_MAX - 200000, off[4] = {};
  int last[4] = {};
  unsigned seed = 1;
  auto random = [&](int n) { seed = seed * 1103515245u + 12345u; return int((seed >> 16) % unsigned(n)); };
  bool on[4] = {};
  for (int step = 0; step < 100000; ++step) {
    now += 1 + random(40);
    const int what = random(6);
    if (what < 3) f.command(random(8) * 45 + (what == 2 ? random(45) : 0), 1 + random(210), random(81) - 40, now, random(4) == 0);
    else if (what == 3) f.stop(now);
    else f.update(now);
    for (int i = 0; i < 4; ++i) {
      const int duty = f.duty(i);
      if (duty) {
        assert(abs(duty) >= 160 && abs(duty) <= 210);
        const int sign = duty < 0 ? -1 : 1;
        if (last[i] && sign != last[i]) assert(!on[i] && uint32_t(now - off[i]) >= kReversePauseMs);
        last[i] = sign;
        on[i] = true;
      } else if (on[i]) {
        on[i] = false;
        off[i] = now;
      }
    }
  }

  Diag t;
  assert(!t.active());
  t.line("p 0 f 160", 0);
  assert(t.duty(0, 0) == 0);  // no motion before "diag"
  t.line("diag", 0);
  assert(t.active() && t.duty(0, 0) == 0);
  t.line("p 4 f 160", 300);
  t.line("p 0 f 255", 300);
  t.line("p 0 f 160 5000", 300);
  assert(t.duty(0, 300) == 0);
  t.line("p 0 f 120", 300);
  assert(t.duty(0, 301) == 120 && t.duty(1, 301) == 0);
  t.update(1000);
  assert(t.duty(0, 1000) == 0);
  t.line("p 0 r 120", 1100);
  assert(t.duty(0, 1101) == 0);  // reverse protection
  t.line("k 0 r 180 140 1000", 1300);
  assert(t.duty(0, 1350) == -180 && t.duty(0, 1450) == -140);
  t.line("s", 1500);
  assert(t.duty(0, 1500) == 0);
  t.line("m 90 160 0", 1800);
  assert(t.duty(0, 1800) != 0);
  t.line("p 0 r 160", 1810);
  assert(t.duty(0, 1811) == 0);  // reverses the wheel the move just drove
  t.line("p 2 f 160", 1900);
  t.line("m 270 160 0", 1950);
  assert(t.duty(0, 1951) == 0);  // pulse then move also waits
  t.line("exit", 2000);
  assert(!t.active() && t.duty(0, 2000) == 0);
}
