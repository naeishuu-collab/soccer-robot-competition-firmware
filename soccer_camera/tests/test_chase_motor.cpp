#include <stdint.h>
#include <string.h>
#include <assert.h>

constexpr uint8_t MSG_POLL = 1, MSG_BALL = 2, MSG_DRIVE = 3;
constexpr int16_t PWM_MAX = 210;
constexpr uint32_t REVERSE_PAUSE_MS = 200, COMMAND_TIMEOUT_MS = 100;
int16_t targetPWM[4]{}, currentPWM[4]{}, appliedPWM[4]{};
uint32_t reverseUntil[4]{}, nowMs = 1000, invalidFrames = 0, receivedFrames = 0;
uint32_t lastCommandMs = 0, applyCalls = 0, coordinatedReverseUntil = 0, timeoutEvents = 0;
uint16_t receivedDirection = 0, receivedPower = 0;
uint8_t lastSequence = 0;
bool commandReceived = false, timedOut = true;

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
}
