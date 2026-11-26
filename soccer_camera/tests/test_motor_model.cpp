// High-minimum-duty wheel model and bench diagnostics.
#include <math.h>
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
  c.stop(1010);
  c.command(0, 160, 0, 1050);
  assert(c.duty(0) == c.target(0) && c.duty(0) < 180);
  c.stop(1060);
  c.command(0, 160, 0, 1160);
  assert(c.duty(0) == 180);
  // millis wrap during a reverse pause.
  Drive wrap;
  wrap.command(0, 160, 0, UINT32_MAX - 50);
  wrap.command(180, 160, 0, UINT32_MAX - 10);
  wrap.update(100);
  assert(wrap.duty(0) == 0);
  wrap.update(190);
  assert(wrap.duty(0) < 0);

  Diag t;
  assert(!t.active());
  t.line("p 0 f 160", 0);
  assert(t.duty(0, 0) == 0);  // no motion before "diag"
  t.line("diag", 0);
  assert(t.active() && t.duty(0, 0) == 0);
  t.line("p 4 f 160", 10);
  t.line("p 0 f 255", 10);
  t.line("p 0 f 160 5000", 10);
  assert(t.duty(0, 10) == 0);
  t.line("p 0 f 120", 10);
  assert(t.duty(0, 11) == 120 && t.duty(1, 11) == 0);
  t.update(710);
  assert(t.duty(0, 710) == 0);
  t.line("p 0 r 120", 800);
  assert(t.duty(0, 801) == 0);  // reverse protection
  t.line("k 0 r 180 140 1000", 1000);
  assert(t.duty(0, 1050) == -180 && t.duty(0, 1150) == -140);
  t.line("s", 1200);
  assert(t.duty(0, 1200) == 0);
  t.line("m 90 160 0", 1500);
  assert(t.duty(0, 1500) != 0);
  t.line("p 0 r 160", 1510);
  assert(t.duty(0, 1511) == 0);  // reverses the wheel the move just drove
  t.line("p 2 f 160", 1600);
  t.line("m 270 160 0", 1650);
  assert(t.duty(0, 1651) == 0);  // pulse then move also waits
  t.line("exit", 1700);
  assert(!t.active() && t.duty(0, 1700) == 0);
}
