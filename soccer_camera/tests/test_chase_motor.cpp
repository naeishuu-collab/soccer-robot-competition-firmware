#include <algorithm>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

constexpr uint8_t MSG_POLL = 1, MSG_BALL = 2, MSG_DRIVE = 3;
constexpr int16_t PWM_MAX = 210, MIN_RUNNING_DUTY = 160, RAMP_STEP = 5;
constexpr int8_t WHEEL_SIGN[4] = {1, -1, 1, -1};
constexpr uint32_t RAMP_INTERVAL_MS = 10;
constexpr uint32_t REVERSE_PAUSE_MS = 200, COMMAND_TIMEOUT_MS = 100;
int16_t targetPWM[4]{}, currentPWM[4]{}, appliedPWM[4]{};
int8_t pausedSign[4]{};
uint32_t lastRampMs = 0;
uint32_t reverseUntil[4]{}, nowMs = 1000, invalidFrames = 0, receivedFrames = 0;
uint32_t lastCommandMs = 0, applyCalls = 0, coordinatedReverseUntil = 0, timeoutEvents = 0;
uint16_t receivedDirection = 0, receivedPower = 0;
uint8_t lastSequence = 0;
bool commandReceived = false, timedOut = true;

using std::min;
int constrain(int value, int lo, int hi) { return std::clamp(value, lo, hi); }
int8_t signOf(int16_t value) { return int8_t((value > 0) - (value < 0)); }
uint32_t millis() { return nowMs; }
uint16_t unpackValue(const uint8_t *p) { return (uint16_t(p[0]) << 7) | p[1]; }
void calculateWheelPWM(uint16_t direction, uint16_t power, int16_t output[4]) {
  for (int i = 0; i < 4; ++i) output[i] = int16_t(direction == 90 && i < 2 ? -int(power) : power);
}
void applyAllMotors() {
  ++applyCalls;
  for (int i = 0; i < 4; ++i) appliedPWM[i] = currentPWM[i];
}

#include "chase_drive_handler.h"

int main() {
  uint8_t drive[12] = {0xA5, MSG_DRIVE, 5, 0, 0, 1, 52}; // power 180
  assert(!commandReceived && timedOut && targetPWM[0] == 0);
  handleDrive(drive);
  assert(commandReceived && targetPWM[0] == 180 && receivedPower == 180 && !timedOut);
  drive[8] = 81;
  handleDrive(drive);
  assert(targetPWM[0] == 200 && targetPWM[3] == 160);
  drive[8] = 20;
  handleDrive(drive);
  assert(invalidFrames == 1 && targetPWM[0] == 200);
  drive[8] = 0;
  invalidFrames = 0;

  currentPWM[0] = 180; currentPWM[1] = 150; currentPWM[3] = 180;
  drive[4] = 90;
  drive[5] = 1; drive[6] = 0; drive[7] = 1; // avoidance at reduced power
  handleDrive(drive);
  assert(coordinatedReverseUntil == 1200);
  for (int i = 0; i < 4; ++i) assert(appliedPWM[i] == 0 && currentPWM[i] == 0);
  assert(targetPWM[0] == -128 && targetPWM[3] == 128);
  nowMs = 1010;
  handleDrive(drive);
  assert(coordinatedReverseUntil == 1200);
  drive[7] = 2;
  handleDrive(drive);
  assert(invalidFrames == 1);
  drive[7] = 0;

  nowMs = 1110;
  checkTimeout();
  assert(timedOut && timeoutEvents == 1 && targetPWM[0] == 0);
  drive[7] = 2;
  handleDrive(drive);
  assert(timedOut && targetPWM[0] == 0);
  drive[7] = 0;
  handleDrive(drive);
  assert(!timedOut && receivedPower == 128 && targetPWM[0] == -128);
  drive[5] = 0;
  handleDrive(drive);
  assert(receivedPower == 0 && targetPWM[0] == 0);

  // After a normal stop, same-polarity targets can restart on the next PWM tick.
  nowMs = 1300;
  drive[4] = 0; drive[5] = 1;
  handleDrive(drive);
  updateMotors();
  assert(currentPWM[0] == 128);
  drive[5] = 0;
  handleDrive(drive);
  assert(currentPWM[0] == 0 && pausedSign[0] == 1);
  nowMs = 1310;
  drive[5] = 1;
  handleDrive(drive);
  updateMotors();
  assert(currentPWM[0] == 128 && reverseUntil[0] == 0);

  // A stop followed by an actual change of polarity preserves the dwell.
  nowMs = 1320;
  drive[5] = 0;
  handleDrive(drive);
  nowMs = 1330;
  drive[4] = 90; drive[5] = 1; drive[7] = 1;
  handleDrive(drive);
  updateMotors();
  assert(coordinatedReverseUntil == 1520 && currentPWM[0] == 0 && currentPWM[3] == 0);
  nowMs = 1510;
  handleDrive(drive);
  updateMotors();
  assert(currentPWM[0] == 0 && currentPWM[3] == 0);
  nowMs = 1520;
  updateMotors();
  assert(currentPWM[0] == -128 && currentPWM[3] == 128);
}
