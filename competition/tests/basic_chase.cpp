#include <cassert>
#include <iostream>
#define RCJ_BASIC_CHASE 1
#include "../main/main.ino"

using namespace robot;

void inject(char c) { Serial.inject(reinterpret_cast<const uint8_t*>(&c), 1); }
void at(uint32_t ms) { clockMs = ms; clockUs = ms * 1000; }

int main() {
  Pwm curve{140, 140, 210, 210};
  Camera cam;
  cam.flags = Ready;
  cam.safe = 0xffff;
  cam.line = Unknown;
  cam.lineCm = 255;
  auto decideBasic = [&](bool armed, int cap, bool tracking = true) {
    return basicDecision(armed, cap, false, true, cam, true, false, false, tracking, curve);
  };
  assert(decideBasic(false, 210).reason == StopReason::DiagnosticIdle);
  auto d = decideBasic(true, 210);
  assert(d.state == State::Chase && d.pwm[0] == 140 && d.pwm[2] == 210);
  Pwm other{210, 210, -140, -140};
  d = basicDecision(true, 210, false, true, cam, true, false, false, true, other);
  assert(d.state == State::Chase && d.pwm[0] == 210 && d.pwm[2] == -140);
  d = decideBasic(true, 140);
  assert(d.pwm[0] == 140 && d.pwm[2] == 140);
  assert(decideBasic(true, 210, false).reason == StopReason::BallLost);
  cam.safe &= ~Rear;
  assert(decideBasic(true, 210).reason == StopReason::PathBlocked);
  cam.safe = 0xffff;
  cam.line = 90; cam.lineCm = 30;
  assert(decideBasic(true, 210).reason == StopReason::PathBlocked);
  cam.flags = Ready | Escape;
  cam.safe = cam.escape = Rear; cam.line = 0;
  d = decideBasic(true, 140, false);
  assert(d.state == State::Escape && d.pwm[0] == -140);
  cam.escape = 0;
  assert(decideBasic(true, 210).reason == StopReason::NoEscape);
  cam.flags = Ready; cam.safe = 0xffff; cam.line = Unknown; cam.lineCm = 255;
  assert(decideBasic(true, 210).state == State::Chase);
  assert(basicDecision(true, 210, true, true, cam, true, false, false, true, curve).reason == StopReason::Manual);
  assert(basicDecision(true, 210, false, false, cam, true, false, false, true, curve).reason == StopReason::CameraTimeout);
  assert(basicDecision(true, 210, false, true, cam, false, false, false, true, curve).reason == StopReason::IrTimeout);

  Serial.connected = true;
  at(1000); setup();
  haveCamera = cameraLive = haveIr = haveBall = true;
  camera = cam; cameraMs = irMs = 1000;
  irStarted = true; pollMs = 1000;
  pd.update(15, 100, 1000);
  inject('3'); delayMicroseconds(BusGuardUs + 1); loop();
  assert(manual && basicCap == 210 && !basicArmed);
  inject('g'); loop();
  assert(!manual && !basicArmed && decision.state == State::Stop);
  inject('R'); loop();
  assert(basicArmed && decision.state == State::Chase);
  delayMicroseconds(BusGuardUs + 1); loop();
  assert(transmitted[0] == 145 && transmitted[2] == 205);
  Pwm decoded{}; uint8_t alternate = 0;
  assert(Serial1.tx.size() >= MotorSize);
  assert(decodeMotor(Serial1.tx.data() + Serial1.tx.size() - MotorSize, decoded, alternate) == Decode::Ok);
  assert(decoded[0] == 145 && decoded[2] == 205 && alternate == 0);
  inject('g'); loop();
  assert(basicArmed && basicStartMs == 1000);
  inject('R'); at(1200); cameraMs = irMs = 1200; loop();
  assert(manual && !basicArmed && decision.state == State::Stop);
  // A new R cannot rearm after expiration without selecting a stage and releasing stop.
  inject('R'); at(1201); loop();
  assert(manual && !basicArmed);
  manual = false; basicCap = 210; basicArmed = true; basicStartMs = 1201; basicLeaseMs = 1201;
  for (uint32_t t = 1300; t < 3201; t += 100) {
    at(t); cameraMs = irMs = pollMs = t; waiting = false; missedIr = false; basicLeaseMs = t; loop();
    assert(basicArmed && !manual);
  }
  at(3201); cameraMs = irMs = pollMs = basicLeaseMs = 3201; waiting = false; loop();
  assert(manual && !basicArmed && decision.state == State::Stop);
  manual = false; basicCap = 210; basicArmed = true; basicStartMs = basicLeaseMs = 3300;
  at(3300); cameraMs = irMs = pollMs = 3300; waiting = false; Serial.connected = false; loop();
  assert(manual && !basicArmed && decision.state == State::Stop);
  std::cout << "BASIC_CHASE: staged differential TX, line/escape, links and deadman OK\n";
}
