#include "../main/zip_chase_pd.h"
#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

// Independent transcription of chase (4).zip motor_chase.ino.
static bool initialized;
static int previousError;
static uint32_t previousMs;
static int constrainZip(int value, int low, int high) {
  return value < low ? low : value > high ? high : value;
}
static int duty(int value) {
  if (value > 0) return constrainZip(value, 140, 210);
  if (value < 0) return -constrainZip(-value, 140, 210);
  return 0;
}
static void reference(uint16_t direction, uint16_t count, uint32_t now, int out[4]) {
  if (!count || direction >= 360) {
    initialized = false;
    for (int i = 0; i < 4; ++i) out[i] = 0;
    return;
  }
  int error = direction > 180 ? int(direction) - 360 : direction;
  float derivative = 0;
  if (initialized && now != previousMs) {
    int delta = error - previousError;
    if (delta > 180) delta -= 360;
    if (delta < -180) delta += 360;
    derivative = delta * 1000.0f / (now - previousMs);
  }
  previousError = error; previousMs = now; initialized = true;
  int turn = constrainZip(int(2.0f * error + 0.2f * derivative), -250, 250);
  out[0] = out[1] = duty(175 - turn);
  out[2] = out[3] = duty(175 + turn);
}
int main() {
  ZipChasePD pd;
  uint32_t now = 0xfffffff0u;
  for (unsigned i = 0; i < 10000; ++i) {
    const uint16_t angle = uint16_t((i * 179u) % 361);
    const uint16_t count = i % 37 ? 400 : 0;
    now += (i % 7 == 0 ? 0 : 30);
    int16_t actual[4]; int expected[4];
    pd.update(angle, count, now, actual);
    reference(angle, count, now, expected);
    for (int j = 0; j < 4; ++j) assert(actual[j] == expected[j]);
  }
  puts("ZIP PD and four signed PWM outputs: 10000 comparisons passed");

  FilteredZipChasePD improved;
  int16_t candidate[4] = {};
  improved.update(359, 200, 1000, candidate);
  improved.update(1, 200, 1020, candidate);
  assert(improved.turn() >= -15 && improved.turn() <= 15); // 359 -> 1 is a 2-degree change.
  assert(candidate[0] == candidate[1] && candidate[2] == candidate[3]);
  assert(abs(candidate[0]) <= 210 && abs(candidate[2]) <= 210);
  improved.update(40, 200, 5000, candidate); // a long gap contributes no derivative kick
  assert(improved.derivative() == 0.0f);
  improved.update(50, 200, 5020, candidate);
  assert(improved.update(50, 0, 5040, candidate) == false);
  for (int i = 0; i < 4; ++i) assert(candidate[i] == 0);
  puts("filtered PD candidate: wrap, irregular dt, and loss reset passed");
}

