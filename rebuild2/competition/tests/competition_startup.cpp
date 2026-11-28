#define RCJ_START_WAIT_MS 0 // Existing controller regression; start wait is tested separately.
#include <cassert>
#include <iostream>
#define RCJ_MANAGED_COMPETITION 0
#include "../main/main.ino"
using namespace robot;
int main() {
  clockMs = 20; clockUs = 20000; Serial.connected = false; setup();
  assert(manual);
  delayMicroseconds(BusGuardUs + 1); loop();
  assert(decision.reason == StopReason::Manual && decision.pwm[0] == 0);
  clockMs = 3020; clockUs = clockMs * 1000; waiting = false; loop();
  assert(manual && decision.reason == StopReason::Manual && decision.pwm[0] == 0);
  Serial.connected = true;
  const uint8_t go = 'g'; Serial.inject(&go, 1); loop();
  assert(!manual && decision.reason == StopReason::CameraTimeout);
  haveCamera = cameraLive = haveIr = haveBall = true;
  camera.flags = Ready; camera.safe = Front; cameraMs = irMs = pollMs = clockMs;
  irStarted = true; waiting = false; pd.update(0, 100, clockMs); loop();
  assert(decision.state == State::Chase);
  Serial.connected = true;
  const uint8_t stop = 's'; Serial.inject(&stop, 1); loop();
  assert(manual && decision.reason == StopReason::Manual);
  clockMs += 100; clockUs = clockMs * 1000; cameraMs = irMs = pollMs = clockMs; waiting = false; loop();
  assert(manual && decision.reason == StopReason::Manual);
  std::cout << "Conservative startup: manual latch, explicit go and sensor gate OK\n";
}
