#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>

using std::min;
constexpr int16_t PWM_MAX = 210, MIN_RUNNING_DUTY = 160;
constexpr float DEG_TO_RAD = 0.01745329252f;
int16_t targetPWM[4] = {};
int8_t signOf(int16_t value) { return int8_t((value > 0) - (value < 0)); }

#include "chase_mixer.h"

int main() {
  const uint8_t expectedForwardPins[] = {6, 12, 11, 9};
  const uint8_t expectedReversePins[] = {7, 13, 10, 8};
  int16_t pwm[4];
  calculateWheelPWM(0, 200, pwm);
  for (int i = 0; i < 4; ++i) {
    assert(MOTOR_IN1[i] == expectedForwardPins[i]);
    assert(MOTOR_IN2[i] == expectedReversePins[i]);
    assert(pwm[i] == 200);
  }
  calculateWheelPWM(0, 160, pwm);
  for (int i = 0; i < 4; ++i) assert(pwm[i] == 160);
  calculateWheelPWM(180, 200, pwm);
  for (int i = 0; i < 4; ++i) assert(pwm[i] == -200);
  calculateWheelPWM(90, 200, pwm);
  assert(pwm[0] > 0 && pwm[1] > 0 && pwm[2] < 0 && pwm[3] < 0);
  calculateWheelPWM(270, 200, pwm);
  assert(pwm[0] < 0 && pwm[1] < 0 && pwm[2] > 0 && pwm[3] > 0);
}
