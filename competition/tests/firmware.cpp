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

void cameraFrame(uint8_t sequence, uint8_t flags, uint16_t mask, uint16_t escape = 0) {
  uint8_t data[CameraSize] = {0xc6, 0x6c, sequence, flags};
  put16(data + 4, mask);
  put16(data + 6, escape);
  put16(data + 8, Unknown);
  data[10] = 255;
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
  assert(decision.state == State::Chase);
  at(52); loop();
  assert(decision.state == State::Chase);
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
  assert(decision.state == State::Escape);
  at(116); loop();
  equal(transmitted, -175, -175, -175, -175);
  at(123); loop();  // Motor refresh may take the first opportunity.
  at(124); loop();
  assert(waiting);
  const uint32_t sentBefore = motorTxCount;
  Serial.rx.push_back('s');
  at(126); loop();
  assert(decision.reason == StopReason::Manual && motorTxCount == sentBefore);
  at(130); irFrame(0, 200); loop();
  at(131); loop();
  equal(transmitted, 0, 0, 0, 0);
  assert(motorTxCount == sentBefore + 1);
  Serial.rx.push_back('g');
  at(132); cameraFrame(3, Ready | Escape, 0); loop();
  assert(decision.reason == StopReason::NoEscape);
  at(140); cameraFrame(4, Ready | Slow, 0); loop();
  assert(decision.reason == StopReason::CameraSlow);
  at(141); cameraFrame(5, 0, 0); loop();
  assert(decision.reason == StopReason::CameraNotReady);
  at(142); cameraFrame(6, Ready, Front); loop();
  assert(decision.state == State::Chase);  // No goals and only the intended heading observed.
  at(190); cameraFrame(6, Ready, Front); loop();
  assert(cameraMs == 142);  // Duplicates don't extend freshness.
  at(342); loop();
  assert(decision.reason == StopReason::CameraTimeout);
  Serial.connected = true;
  Serial.room = 0;
  Serial.rx.push_back('i');
  at(355); loop();
  assert(Serial.tx.empty());  // Diagnostics cannot wait on a saturated USB.
  Serial.room = 512;
  at(368);
  for (unsigned i = 0; i < 80; ++i) loop();
  const std::string log(Serial.tx.begin(), Serial.tx.end());
  assert(log.find("STOP_REASON=CAMERA_TIMEOUT") != std::string::npos);
  assert(log.find("PWM_TRANSMITTED=0,0,0,0") != std::string::npos);
  std::cout << "MAIN loop: startup, tracking, packet timing, safety, manual stop, USB OK\n";
}

void motorLoop() {
  using namespace motor_board;
  Serial1 = FakeSerial{};
  at(1); setup(); loop();
  equal(written, 0, 0, 0, 0);
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

int main() {
  comparePd();
  compareIr();
  cameraWire();
  mainLoop();
  motorLoop();
}
