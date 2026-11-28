#define RCJ_MANAGED_COMPETITION 0
#define RCJ_STARTUP_DELAY_MS 0
#include <cassert>
#include <cmath>
#include <iostream>
#include <BoardIO.h>
#include <SerialPIO.h>
#include "../main/control.h"
#include "../motor/motor_output.h"

namespace original_motor {
#include "motor_chase.ino"
}
#undef RS485
namespace original_ir {
#include "ir_chase.ino"
}
#undef RS485
#undef RS485_DE_PIN
#undef RS485_BAUD
#undef SENSOR_COUNT

namespace main_board {
#include "../main/main.ino"
}
namespace motor_board {
#include "../motor/motor.ino"
}
namespace ir_board {
#include "../ir/ir.ino"
}

using namespace robot;
#include "camera_golden.h"

void equal(const Pwm& pwm, int a, int b, int c, int d) {
  if (pwm[0] != a || pwm[1] != b || pwm[2] != c || pwm[3] != d)
    std::cerr << "Applied PWM " << pwm[0] << ',' << pwm[1] << ',' << pwm[2] << ',' << pwm[3]
              << " differs from " << a << ',' << b << ',' << c << ',' << d << " at ms " << millis() << '\n';
  assert(pwm[0] == a && pwm[1] == b && pwm[2] == c && pwm[3] == d);
}

void readZipWheels(Pwm& read) {
  const uint8_t first[] = {6, 12, 11, 9}, second[] = {7, 13, 10, 8};
  for (int i = 0; i < 4; ++i) read[i] = int16_t(int(levels[first[i]]) - int(levels[second[i]]));
}

void comparePd() {
  ChasePD sameTiming, fixedOffset, earlierTiming;
  uint32_t seed = 76543, now = 0xfffffe00;
  unsigned timingDifferences = 0, matches = 0;
  original_motor::setup();
  const uint16_t bearings[] = {0, 90, 180, 270, 359, 0, 179, 181, 180, 0, 360, 12, 11, 88};
  for (unsigned i = 0; i < 20000; ++i) {
    seed = 1664525u * seed + 1013904223u;
    const uint16_t angle = i < 14 ? bearings[i] : uint16_t((seed >> 16) % 360);
    const uint16_t strength = i % 67 == 13 ? 0 : 300;
    now += i < 14 ? (i == 5 ? 0 : 30) : (seed >> 24) % 80;
    clockMs = now;
    original_motor::trackBall(angle, strength);
    Pwm reference;
    readZipWheels(reference);
    sameTiming.update(angle, strength, now);
    equal(sameTiming.pwm, reference[0], reference[1], reference[2], reference[3]);
    fixedOffset.update(angle, strength, now - 1);
    equal(fixedOffset.pwm, reference[0], reference[1], reference[2], reference[3]);
    ++matches;
    // Relay and UART shift the time at the old MOTOR by about one millis() tick,
    // with quantization jitter when the serial arrival crosses a tick boundary.
    earlierTiming.update(angle, strength, now - uint32_t(i % 3 != 0));
    if (memcmp(reference, earlierTiming.pwm, sizeof(reference))) ++timingDifferences;
  }
  assert(timingDifferences > 0);
  std::cout << "ZIP MOTOR: " << matches << " identical; simulated 0/1 ms relay jitter changes "
            << timingDifferences << " PWM results\n";
}

void compareIr() {
  ir_board::setup();
  for (unsigned pattern = 0; pattern < 4096; ++pattern) {
    sensorBits = pattern;
    clockUs = 100000;
    Serial1.tx.clear();
    original_ir::sampleAndReply();
    const auto reference = Serial1.tx;
    assert(reference.size() == 6);
    Serial1.tx.clear();
    clockUs = 100000;
    ir_board::sampleAndReply(clockUs);
    assert(Serial1.tx.size() == 6);
    assert(!memcmp(original_ir::detectCount, ir_board::counts, sizeof(ir_board::counts)));
    if (ir_board::strength && !ir_board::x && !ir_board::y) {
      assert(read16(reference.data() + 1) == 0 && read16(Serial1.tx.data() + 1) == Unknown);
      assert(read16(reference.data() + 3) == read16(Serial1.tx.data() + 3));
    } else assert(reference == Serial1.tx);
  }
  assert(ir_board::invalid > 0);
  Serial1 = FakeSerial{};
  sensorBits = 1;
  const uint8_t badPoll[] = {0xa5, 0x5a, 0x02, 0xfe};
  Serial1.inject(badPoll, sizeof(badPoll));
  ir_board::loop();
  assert(ir_board::polls == 0 && Serial1.tx.empty());
  Serial1.inject(Poll, sizeof(Poll));
  ir_board::loop();
  assert(ir_board::polls == 1 && Serial1.tx.size() == 6);
  std::cout << "ZIP IR: 4096 sensor patterns, " << ir_board::invalid - 1 << " cancellations use FFFF\n";
}

void cameraWire() {
  static_assert(sizeof(cameraGolden) == CameraSize);
  Camera camera;
  assert(decodeCamera(cameraGolden, camera) == Decode::Ok);
  assert(camera.sequence == 19 && camera.safe == Rear && camera.escape == Rear &&
         camera.line == 359 && camera.lineCm == 24 && camera.frameMs == 66);
  assert(camera.yellow.angle == 75 && camera.yellow.confidence == 65 && camera.yellow.age == 8);
  assert(camera.blue.angle == Unknown && camera.blue.age == 255);
  assert(decodeCamera(cameraGoldenV2, camera) == Decode::Ok && camera.version == 3 &&
         (camera.flags & LineHazard) && camera.safe == 0 && camera.line == Unknown);
  uint8_t version2[CameraSize];
  memcpy(version2, cameraGolden, sizeof(version2));
  version2[20] = 2;
  version2[3] |= LineHazard;
  seal(version2, sizeof(version2));
  assert(decodeCamera(version2, camera) == Decode::Ok && camera.version == 2 && (camera.flags & LineHazard));
  version2[20] = 3; put16(version2 + 8, 0); version2[10] = 255; seal(version2, sizeof(version2));
  assert(decodeCamera(version2, camera) == Decode::Ok && camera.line == 0 && camera.lineCm == 255);
  version2[20] = 2; seal(version2, sizeof(version2));
  assert(decodeCamera(version2, camera) == Decode::Fields);
  version2[20] = 1; seal(version2, sizeof(version2));
  assert(decodeCamera(version2, camera) == Decode::Fields);
  uint8_t damaged[CameraSize];
  memcpy(damaged, cameraGolden, sizeof(damaged));
  damaged[4] ^= 1;
  assert(decodeCamera(damaged, camera) == Decode::Crc);
  damaged[3] |= Slow;
  seal(damaged, sizeof(damaged));
  assert(decodeCamera(damaged, camera) == Decode::Fields);
  std::cout << "OpenMV packet: Python encoder / C++ decoder, CRC and fields OK\n";
}

void at(uint32_t ms) { clockMs = ms; clockUs = ms * 1000; }

void cameraFrame(uint8_t sequence, uint8_t flags, uint16_t mask, uint16_t escape = 0, uint16_t line = Unknown, uint8_t cm = 255) {
  uint8_t data[CameraSize] = {0xc6, 0x6c, sequence, flags};
  put16(data + 4, mask);
  put16(data + 6, escape);
  put16(data + 8, line);
  data[10] = cm;
  data[11] = flags & Slow ? 101 : 45;
  put16(data + 12, Unknown);
  put16(data + 16, Unknown);
  data[15] = data[19] = 255;
  data[20] = 1;
  seal(data, sizeof(data));
  main_board::cameraSerial.inject(data, sizeof(data));
}

void irFrame(uint16_t angle, uint16_t strength) {
  uint8_t data[] = {0xaa, 0, 0, 0, 0, 0x55};
  put16(data + 1, angle);
  put16(data + 3, strength);
  Serial1.inject(data, sizeof(data));
}

uint8_t transmittedChasePacket[MotorSize]{};

void mainLoop() {
  using namespace main_board;
  Serial1 = FakeSerial{};
  at(1);
  setup();
  delayMicroseconds(BusGuardUs);
  loop();
  assert(decision.reason == StopReason::CameraTimeout && motorTxCount == 1);
  equal(transmitted, 0, 0, 0, 0);
  at(2);
  cameraFrame(1, Ready, 0xffff);
  loop();
  assert(waiting && decision.reason == StopReason::IrTimeout);
  at(8);
  irFrame(0, 200);
  loop();
  assert(decision.state == State::Chase);
  at(9);
  loop();
  equal(transmitted, 175, 175, 175, 175);
  at(32); loop();  // Next IR poll.
  at(38); irFrame(0, 0); loop();
  assert(decision.reason == StopReason::BallLost);
  assert(!pd.initialized);
  at(39); loop();
  equal(transmitted, 0, 0, 0, 0);
  at(52); loop();
  assert(decision.reason == StopReason::BallLost);
  at(53); loop();
  assert(decision.reason == StopReason::BallLost);
  equal(transmitted, 0, 0, 0, 0);
  at(62); loop();  // Poll, then a 12 ms silent slot.
  at(75); loop();
  assert(decision.reason == StopReason::IrNoReply);
  at(93); loop();
  at(94); loop();
  at(99); irFrame(361, 250); loop();
  assert(decision.reason == StopReason::IrInvalid);
  at(115); cameraFrame(2, Ready | Escape, Rear, Rear); loop();
  assert(decision.reason == StopReason::IrInvalid);
  at(124); loop();
  at(125); loop();
  assert(waiting);
  at(130); irFrame(0, 0); loop();  // A valid no-ball reply still permits escape.
  assert(decision.state == State::Escape);
  at(131); loop();
  equal(transmitted, -175, -175, -175, -175);
  assert(escapeRxToTxUs > 0);
  at(155); loop();
  assert(waiting);
  const uint32_t sentBefore = motorTxCount;
  Serial.rx.push_back('s');
  at(156); loop();
  assert(decision.reason == StopReason::Manual && motorTxCount == sentBefore);
  at(160); irFrame(340, 4520); loop();
  at(161); loop();
  equal(transmitted, 0, 0, 0, 0);
  assert(motorTxCount == sentBefore + 1);
  Serial.rx.push_back('g');
  at(162); cameraFrame(3, Ready | Escape, 0, 0, 180, 19); loop();
  assert(decision.reason == StopReason::NoEscape && decision.blocked == 0x0101);
  equal(pd.pwm, 210, 210, 140, 140);
  equal(transmitted, 0, 0, 0, 0);
  at(163); cameraFrame(4, Ready | Escape, Front, Front, 180, 19); loop();
  assert(decision.path == PathMode::Escape && decision.required == Front);
  equal(transmitted, 175, 175, 175, 175);
  at(164); cameraFrame(5, Ready, 0x8007); loop();
  assert(decision.state == State::Chase && decision.path == PathMode::Straight);
  equal(decision.pwm, 160, 160, 160, 160);
  at(170); cameraFrame(6, Ready | Slow, 0); loop();
  assert(decision.reason == StopReason::CameraSlow);
  at(171); cameraFrame(7, 0, 0); loop();
  assert(decision.reason == StopReason::CameraNotReady);
  haveBall = false;
  pd.reset();
  at(172); cameraFrame(8, Ready, 0x8007); loop();
  assert(decision.reason == StopReason::BallLost);
  const uint32_t oldPoll = pollMs;
  for (uint32_t t = 173; t <= 194 && pollMs == oldPoll; ++t) { at(t); loop(); }
  assert(waiting);
  const uint32_t freshPoll = pollMs;
  at(freshPoll + 5); irFrame(2, 6909); loop();
  equal(pd.pwm, 171, 171, 179, 179);
  assert(pd.pwm[0] != pd.pwm[2] && decision.path == PathMode::Straight && decision.required == Front);
  equal(decision.pwm, 160, 160, 160, 160);
  at(freshPoll + 6); loop();
  equal(transmitted, 160, 160, 160, 160);
  assert(Serial1.tx.size() >= sizeof(transmittedChasePacket));
  memcpy(transmittedChasePacket, Serial1.tx.data() + Serial1.tx.size() - sizeof(transmittedChasePacket),
         sizeof(transmittedChasePacket));
  at(freshPoll + 7); cameraFrame(8, Ready, 0x8007); loop();
  assert(cameraMs == 172);  // Duplicates don't extend freshness.
  at(372); loop();
  assert(decision.reason == StopReason::CameraTimeout);
  Serial.connected = true;
  Serial.room = 0;
  Serial.rx.push_back('i');
  at(385); loop();
  assert(Serial.tx.empty());  // Diagnostics cannot wait on a saturated USB.
  Serial.room = 512;
  for (uint32_t t = 398; t < 520; ++t) { at(t); loop(); }
  const std::string log(Serial.tx.begin(), Serial.tx.end());
  assert(log.find("STOP_REASON=CAMERA_TIMEOUT") != std::string::npos);
  assert(log.find("PWM_TRANSMITTED=0,0,0,0") != std::string::npos);
  assert(log.find("PATH_MODE=STOP") != std::string::npos);
  assert(log.find("IR_POLL_INTERVAL_MS=") != std::string::npos);
  assert(log.find("MANUAL_STOP COUNT=1 DURATION_MS=") != std::string::npos);
  std::cout << "MAIN loop: startup, tracking, packet timing, safety, manual stop, USB OK\n";
}

void motorLoop() {
  using namespace motor_board;
  Serial1 = FakeSerial{};
  at(1); setup(); loop();
  equal(written, 0, 0, 0, 0);
  Serial1.inject(transmittedChasePacket, sizeof(transmittedChasePacket));
  at(2); loop();
  equal(written, 160, 160, 160, 160);
  assert(rxCount == 1 && levels[6] == 160 && levels[11] == 160);
  std::cout << "MAIN to MOTOR: partial observation, PD, UART/RS485 and nonzero applied PWM OK\n";
  rxCount = 0;
  Pwm command{175, 175, 210, 140};
  uint8_t bytes[MotorSize];
  encodeMotor(command, bytes);
  bytes[3] ^= 1;
  Serial1.inject(bytes, sizeof(bytes));
  at(2); loop();
  assert(crcCount == 1 && rxCount == 0);
  put16(bytes + 2, 421);
  seal(bytes, sizeof(bytes));
  Serial1.inject(bytes, sizeof(bytes));
  at(3); loop();
  assert(invalidCount == 2 && rxCount == 0);
  encodeMotor(command, bytes);
  Serial1.inject(bytes, 5);  // One partial command followed by a valid one.
  Serial1.inject(bytes, sizeof(bytes));
  at(4); loop();
  assert(rxCount == 1);
  equal(written, 175, 175, 210, 140);
  assert(levels[6] == 175 && levels[10] == 0 && levels[11] == 210);
  command[0] = -140;
  encodeMotor(command, bytes);
  Serial1.inject(bytes, sizeof(bytes));
  at(5); loop();
  assert(output.reverseMask == 1 && output.reverseWait);
  equal(written, 0, 0, 0, 0);
  for (uint32_t now : {100u, 190u, 204u, 205u}) {
    Serial1.inject(bytes, sizeof(bytes));
    at(now); loop();
  }
  equal(written, -140, 175, 210, 140);
  assert(levels[6] == 0 && levels[7] == 140);
  at(305); loop();
  equal(written, 0, 0, 0, 0);
  assert(output.watchdog && lastStatus == Status::Watchdog);
  Serial.tx.clear();
  Serial.rx.push_back('i');
  loop();
  for (unsigned i = 0; i < 20; ++i) loop();
  const std::string log(Serial.tx.begin(), Serial.tx.end());
  assert(log.find("STOP_REASON=WATCHDOG") != std::string::npos);
  std::cout << "MOTOR loop: CRC, range, truncation, startup, reversal and watchdog OK\n";
}

void benchPolicy() {
  const Pwm pwm{210, 210, 140, 140};
  auto decision = benchDecision(RunMode::MotorTest, false, 0, true, pwm, false, true, true, false, false);
  assert(decision.reason == StopReason::DiagnosticIdle);
  equal(decision.pwm, 0, 0, 0, 0);
  decision = benchDecision(RunMode::MotorTest, true, 2, false, pwm, false, true, true, false, false);
  equal(decision.pwm, 0, 0, 160, 0);
  decision = benchDecision(RunMode::MotorTest, true, 4, false, pwm, false, true, true, false, false);
  equal(decision.pwm, -160, 160, -160, 160);
  decision = benchDecision(RunMode::MotorTest, true, 5, false, pwm, false, true, true, false, false);
  equal(decision.pwm, 160, -160, 160, -160);
  decision = benchDecision(RunMode::MotorTest, false, 4, false, pwm, false, true, true, false, false);
  assert(decision.reason == StopReason::DiagnosticIdle);
  equal(decision.pwm, 0, 0, 0, 0);
  assert(benchDecision(RunMode::MotorTest, true, 4, false, pwm, true, true, true, false, false).reason == StopReason::Manual);
  assert(benchDecision(RunMode::MotorTest, true, 4, false, pwm, false, false, true, false, false).reason == StopReason::CameraTimeout);
  assert(benchDecision(RunMode::MotorTest, true, 5, false, pwm, false, true, false, false, true).reason == StopReason::IrTimeout);
  assert(benchDecision(RunMode::MotorTest, true, 6, false, pwm, false, true, true, false, false).reason == StopReason::DiagnosticIdle);
  decision = benchDecision(RunMode::IrPdTest, true, -1, true, pwm, false, true, true, false, false);
  equal(decision.pwm, 160, 160, 140, 140);
  assert(benchDecision(RunMode::IrPdTest, true, -1, true, pwm, false, false, true, false, false)
      .reason == StopReason::CameraTimeout);
  assert(benchDecision(RunMode::IrPdTest, true, -1, true, pwm, false, true, false, false, true)
      .reason == StopReason::IrTimeout);
  assert(benchDecision(RunMode::MotorTest, true, 0, true, pwm, true, true, true, false, false)
      .reason == StopReason::Manual);
}

void reverseBridge() {
  const Pwm forward{175,175,175,175}, turn{-140,-140,210,210}, stop{};
  MotorOutput previous, next;
  for (auto* output : {&previous, &next}) {
    output->accept(forward, 0);
    output->update(0);
  }
  previous.accept(turn, 30);
  next.accept(turn, 30, BridgePwm);
  previous.update(30);
  next.update(30);
  equal(previous.applied, 0,0,0,0);
  equal(next.applied, 160,160,160,160);
  assert(next.bridging && next.reverseMask == 3 && next.waitCount == 1);

  previous.accept(forward, 60);
  previous.update(60);
  equal(previous.applied, 175,175,175,175);
  auto transient = next;
  transient.accept(forward, 60);
  transient.update(60);
  equal(transient.applied, 175,175,175,175);
  assert(!transient.reverseWait && transient.waitMs == 30);
  auto revoked = next;
  revoked.accept(stop, 31);
  revoked.update(31);
  equal(revoked.applied, 0,0,0,0);

  next.accept(turn, 60, BridgePwm);
  next.update(64);
  equal(next.applied, 160,160,160,160);
  next.update(65);
  equal(next.applied, 0,0,0,0);
  for (uint32_t now : {120u, 200u, 264u, 265u}) {
    next.accept(turn, now, BridgePwm);
    next.update(now);
    if (now < 265) equal(next.applied, 0,0,0,0);
  }
  equal(next.applied, -140,-140,210,210);
  assert(next.waitCount == 1 && next.waitMs == 235 && next.bridgeCount == 1);

  uint8_t packet[MotorSize], alternate;
  Pwm unpacked;
  encodeMotor(turn, packet, BridgePwm);
  assert(decodeMotor(packet, unpacked, alternate) == Decode::Ok && alternate == BridgePwm);
  encodeMotor(forward, packet, BridgePwm);
  assert(decodeMotor(packet, unpacked, alternate) == Decode::Fields);
  encodeMotor(turn, packet, 3);
  assert(decodeMotor(packet, unpacked, alternate) == Decode::Fields);
  std::cout << "Reverse comparison: 30 ms transient full stop 30 -> 0; persistent reversal waits 200 from zero\n";
}

void pollFairness() {
  using namespace main_board;
  Serial.connected = false;
  waiting = false;
  haveCamera = cameraLive = haveIr = haveBall = true;
  irInvalid = missedIr = manual = sentAheadOfPoll = false;
  camera.flags = Ready;
  camera.safe = 0xffff;
  cameraMs = irMs = 1000;
  irStarted = true;
  pollMs = 970;
  Serial1.tx.clear();
  at(1000);
  pd.reset();
  pd.update(0, 200, 1000);
  loop();
  assert(Serial1.tx.size() == MotorSize && Serial1.tx[0] == MotorSync);
  pd.update(15, 200, 1000);  // PWM changes again before the due poll.
  delayMicroseconds(BusGuardUs + 1);
  loop();
  assert(waiting && pollInterval == 30);
  assert(Serial1.tx.size() == MotorSize + sizeof(Poll));
  assert(!memcmp(Serial1.tx.data() + MotorSize, Poll, sizeof(Poll)));
  std::cout << "RS485: a due poll follows at most one pending moving command\n";
}

int main() {
  comparePd();
  compareIr();
  cameraWire();
  mainLoop();
  motorLoop();
  reverseBridge();
  benchPolicy();
  pollFairness();
}
