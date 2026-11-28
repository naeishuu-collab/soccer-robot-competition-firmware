#include <cassert>
#include <iostream>
#define RCJ_MANAGED_COMPETITION 1
#include "../main/main.ino"
using namespace robot;

int main() {
  clockMs = 1000; clockUs = 1000000; Serial.connected = true; setup();
  assert(manual && runMode == RunMode::ManagedCompetition);
  haveCamera = cameraLive = haveIr = haveBall = true;
  camera.flags = Ready; camera.version = 2; camera.safe = 1;
  cameraMs = irMs = pollMs = 1000; irStarted = true;
  pd.update(15, 100, 1000);
  const uint8_t go = 'g'; Serial.inject(&go, 1);
  delayMicroseconds(BusGuardUs + 1); loop();
  assert(decision.state == State::Chase && decision.pwm[0] == 145 && decision.pwm[2] == 205);
  delayMicroseconds(BusGuardUs + 1); loop();
  Pwm decoded{}; uint8_t alternate = 0;
  assert(Serial1.tx.size() >= MotorSize);
  assert(decodeMotor(Serial1.tx.data() + Serial1.tx.size() - MotorSize, decoded, alternate) == Decode::Ok);
  assert(decoded[0] == 145 && decoded[2] == 205);
  camera.flags |= LineHazard; loop();
  assert(decision.reason == StopReason::PathBlocked);
  camera.flags |= Escape; camera.safe = camera.escape = Rear; camera.line = 0; loop();
  assert(decision.state == State::Escape && decision.pwm[0] == -175);
  camera.flags = Ready; camera.safe = 1; camera.escape = 0; camera.line = Unknown; loop();
  assert(decision.state == State::Chase && decision.pwm[0] == 145 && decision.pwm[2] == 205);
  camera.version = 1; loop();
  assert(decision.reason == StopReason::PathBlocked);
  const uint8_t stop = 's'; Serial.inject(&stop, 1); loop();
  assert(decision.reason == StopReason::Manual);
  std::cout << "Managed competition: partial observation, PD TX, hazard, escape, recovery and stop OK\n";
}
