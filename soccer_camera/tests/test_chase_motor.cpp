#include <stdint.h>
#include <assert.h>

constexpr uint8_t MSG_POLL = 1, MSG_BALL = 2, MSG_DRIVE = 3;
constexpr int16_t PWM_MAX = 210;
constexpr uint32_t REVERSE_PAUSE_MS = 200;
int16_t targetPWM[4]{}, currentPWM[4]{}, appliedPWM[4]{};
uint32_t reverseUntil[4]{}, nowMs = 1000, invalidFrames = 0, receivedFrames = 0;
uint32_t lastCommandMs = 0, applyCalls = 0;
uint16_t receivedDirection = 0, receivedPower = 0;
uint8_t lastSequence = 0;
bool commandReceived = false, timedOut = true;

uint32_t millis() { return nowMs; }
uint16_t unpackValue(const uint8_t *p) { return (uint16_t(p[0]) << 7) | p[1]; }
void calculateWheelPWM(uint16_t, uint16_t power, int16_t output[4]) {
  for (int i = 0; i < 4; ++i) output[i] = int16_t(power);
}
void applyAllMotors() {
  ++applyCalls;
  for (int i = 0; i < 4; ++i) appliedPWM[i] = currentPWM[i];
}

#include "chase_drive_handler.h"

int main() {
  uint8_t frame[12] = {0xA5, MSG_DRIVE, 5, 0, 90, 1, 52}; // power 180
  handleDrive(frame);
  assert(targetPWM[0] == 180 && receivedPower == 180 && !timedOut);
  currentPWM[0] = 180; currentPWM[1] = -150;
  frame[5] = frame[6] = 0;
  handleDrive(frame);
  assert(applyCalls == 1 && appliedPWM[0] == 0 && appliedPWM[1] == 0);
  assert(targetPWM[0] == 0 && reverseUntil[0] == 1200 && reverseUntil[1] == 1200);
  nowMs = 1010;
  handleDrive(frame);
  assert(reverseUntil[0] == 1200); // repeated stop does not extend the wait
  frame[7] = 1; currentPWM[0] = 20;
  handleDrive(frame);
  assert(invalidFrames == 1 && currentPWM[0] == 20 && applyCalls == 2);
  frame[7] = 0; nowMs = UINT32_MAX - 199;
  handleDrive(frame);
  assert(reverseUntil[0] == 1); // zero is reserved for no wait
}
