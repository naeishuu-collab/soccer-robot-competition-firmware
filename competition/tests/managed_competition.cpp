#include <cassert>
#include <iostream>
#define RCJ_MANAGED_COMPETITION 1
#include "../main/main.ino"
using namespace robot;

int main() {
  clockMs = 1000; clockUs = 1000000; Serial.connected = false; setup();
  assert(!manual && runMode == RunMode::ManagedCompetition);
  haveCamera = cameraLive = haveIr = haveBall = true;
  camera.flags = Ready; camera.version = 2; camera.safe = 1;
  cameraMs = irMs = pollMs = 1000; irStarted = true;
  pd.update(15, 100, 1000);
  delayMicroseconds(BusGuardUs + 1); loop();
  assert(decision.reason == StopReason::Startup && decision.pwm[0] == 0);
  clockMs = 3999; clockUs = clockMs * 1000; cameraMs = irMs = pollMs = clockMs; waiting = false; loop();
  assert(decision.reason == StopReason::Startup && decision.pwm[0] == 0);
  clockMs = 4000; clockUs = clockMs * 1000; cameraMs = pollMs = clockMs; haveIr = false; waiting = false; loop();
  assert(decision.reason == StopReason::IrTimeout && decision.pwm[0] == 0);
  haveIr = haveBall = true; irMs = clockMs; pd.update(15, 100, clockMs); loop();
  assert(decision.state == State::Chase && decision.pwm[0] == 145 && decision.pwm[2] == 205);
  delayMicroseconds(BusGuardUs + 1); loop();
  Pwm decoded{}; uint8_t alternate = 0;
  assert(Serial1.tx.size() >= MotorSize);
  assert(decodeMotor(Serial1.tx.data() + Serial1.tx.size() - MotorSize, decoded, alternate) == Decode::Ok);
  assert(decoded[0] == 145 && decoded[2] == 205);
  camera.flags |= LineHazard; loop();
  assert(decision.reason == StopReason::PathBlocked);
  camera.flags |= Escape; camera.safe = camera.escape = 0; camera.line = Unknown; camera.lineCm = 255; loop();
  assert(decision.reason == StopReason::NoEscape && decision.pwm[0] == 0 && decision.blocked == (Front | Rear));
  ChasePD loggedPd; loggedPd.update(29, 6441, clockMs);
  assert(loggedPd.pwm[0] == 140 && loggedPd.pwm[2] == 210);
  Decision logged = basicDecision(true, 210, false, true, camera, true, false, false, true, loggedPd.pwm);
  assert(logged.reason == StopReason::NoEscape && logged.pwm[0] == 0 && logged.pwm[2] == 0);
  camera.flags |= Escape; camera.safe = camera.escape = Rear; camera.line = 0; loop();
  assert(decision.state == State::Escape && decision.pwm[0] == -175);
  camera.flags = Ready; camera.safe = 1; camera.escape = 0; camera.line = Unknown; loop();
  assert(decision.state == State::Chase && decision.pwm[0] == 145 && decision.pwm[2] == 205);
  camera.version = 1; loop();
  assert(decision.reason == StopReason::PathBlocked);
  Serial.connected = true;
  const uint8_t stop = 's'; Serial.inject(&stop, 1); loop();
  assert(decision.reason == StopReason::Manual);
  clockMs += 5000; clockUs = clockMs * 1000; cameraMs = irMs = pollMs = clockMs; waiting = false; loop();
  assert(manual && decision.reason == StopReason::Manual);
  const uint8_t go = 'g'; Serial.inject(&go, 1); loop();
  assert(!manual && decision.state == State::Stop); // Stop cleared PD; next IR sample is required.
  std::cout << "Managed competition: partial observation, PD TX, hazard, escape, recovery and stop OK\n";
}
