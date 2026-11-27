// Runs the real MAIN, MOTOR and IR sketches (setup/loop, ROBOT_UPGRADE=1) wired together on the host.
#define ROBOT_UPGRADE 1
#include <Arduino.h>
#include <SerialPIO.h>
#include <Wire.h>
#include <stdio.h>
#include "main_logic.h"
#include "motor_logic.h"
#include "verified_motors.h"
#include "ir_logic.h"

namespace mainboard {
#include "main.ino"
}
namespace motorboard {
#include "motor.ino"
}
namespace irboard {
#include "ir.ino"
}

static int failed = 0;
#define CHECK(name, condition) \
  do { if (!(condition)) { ++failed; printf("FAIL %s (line %d, t=%u)\n", name, __LINE__, mockMs); } } while (0)

struct Frame { uint16_t direction, power; int turn; bool avoid; };

// Each board has its own copy of the shared mock UART.
struct Rig {
  MockSerial mainBus, motorBus, irBus;
  std::vector<Frame> sent;
  bool camera = true, ir = true, corrupt = false, noBall = false;
  uint16_t mask = 0xFFFF, escape = 0xFFFF, ballAngle = 0;
  uint8_t sequence = 0;
  uint32_t lastCameraMs = 0, replyAt = 0;
  bool pollPending = false, sentDuringReply = false;

  void cameraFrame() {
    uint8_t f[brain::kCameraSize] = {0xC5, 0x5C, brain::kCameraVersion, brain::kCameraSize, sequence++, 0};
    f[5] = uint8_t(1 | 16 | (escape != 0xFFFF ? 2 : 0));
    auto put = [&](int at, uint16_t v) { f[at] = uint8_t(v); f[at + 1] = uint8_t(v >> 8); };
    put(6, mask); put(8, escape); put(10, 0xFFFF);
    f[12] = 255;
    for (int at : {20, 26}) put(at, 0xFFFF);
    f[22] = 255; f[25] = 255;
    put(14, 0); f[16] = 255; f[17] = 90; f[18] = 20; f[19] = 0;  // yellow goal ahead
    put(30, brain::crc16(f, brain::kCameraSize - 2));
    if (corrupt) { f[7] ^= 1; corrupt = false; }
    mainboard::cameraSerial.feed(f, sizeof(f));
  }

  // One millisecond of the whole robot.
  void tick() {
    ++mockMs;
    if (camera && mockMs - lastCameraMs >= 50) { lastCameraMs = mockMs; cameraFrame(); }
    if (pollPending && mockMs >= replyAt) {
      pollPending = false;
      irx::Result r;
      r.angle = uint16_t((360 - ballAngle) % 360);  // MAIN mirrors the IR angle
      r.strength = noBall ? 0 : 400; r.confidence = noBall ? 0 : 90;
      uint8_t wire[irx::kReplySize];
      irx::encode(r, wire);
      mainBus.feed(wire, sizeof(wire));
    }
    std::swap(Serial1, mainBus);
    mainboard::loop();
    std::vector<uint8_t> out;
    out.swap(Serial1.tx);
    std::swap(Serial1, mainBus);
    for (size_t i = 0; i < out.size();) {
      if (i + 4 <= out.size() && out[i] == 0xA5 && out[i + 1] == 0x5A) {
        if (ir) { pollPending = true; replyAt = mockMs + 6; }
        i += 4;
      } else if (i + 12 <= out.size() && out[i] == 0xA5 && out[i + 1] == 3) {
        const uint8_t *f = &out[i];
        if (pollPending) sentDuringReply = true;
        sent.push_back({uint16_t(f[3] << 7 | f[4]), uint16_t(f[5] << 7 | f[6]),
                        f[8] ? int(f[8]) - 61 : 0, f[7] == 1});
        motorBus.feed(f, 12);
        i += 12;
      } else {
        ++i;
      }
    }
    std::swap(Serial1, motorBus);
    motorboard::loop();
    std::swap(Serial1, motorBus);
  }
  void run(uint32_t ms) { while (ms--) tick(); }
  static bool still() {
    for (int pin = 6; pin <= 13; ++pin) if (mockPwm[pin]) return false;
    return true;
  }
  // Never above the limit; below 160 only during a start ramp (100 ms plus one command period).
  static bool legalDuty() {
    static uint32_t since[16] = {};
    static bool low[16] = {};
    for (int pin = 6; pin <= 13; ++pin) {
      if (mockPwm[pin] > 210) return false;
      const bool now = mockPwm[pin] && mockPwm[pin] < 160;
      if (now && !low[pin]) since[pin] = mockMs;
      low[pin] = now;
      if (now && mockMs - since[pin] > 140) return false;
    }
    return true;
  }
};

static void sendRaw(Rig &rig, uint16_t direction, uint16_t power, int turn, bool avoid) {
  static uint8_t seq = 0;
  uint8_t f[12] = {0xA5, 3, uint8_t(seq++ & 0x7F), uint8_t(direction >> 7), uint8_t(direction & 127),
                   uint8_t(power >> 7), uint8_t(power & 127), uint8_t(avoid), uint8_t(turn + 61)};
  f[11] = mainboard::calculateCRC(f);
  rig.motorBus.feed(f, 12);
  std::swap(Serial1, rig.motorBus);
  motorboard::loop();
  std::swap(Serial1, rig.motorBus);
}

int main() {
  mockMs = 0xFFFFFFFFu - 3000;  // the whole scenario crosses the millis wrap
  for (int pin = 18; pin <= 29; ++pin) mockPin[pin] = HIGH;
  mainboard::setup();
  motorboard::setup();
  Rig rig;

  rig.run(1500);
  CHECK("drives toward the ball", !rig.sent.empty() && rig.sent.back().power > 0 && rig.sent.back().direction == 0);
  CHECK("motors run inside the duty band", !Rig::still() && Rig::legalDuty());
  CHECK("never transmits inside the IR reply window", !rig.sentDuringReply);

  rig.camera = false;
  rig.run(400);
  CHECK("camera loss stops", rig.sent.back().power == 0 && Rig::still());
  rig.camera = true;
  rig.run(500);
  CHECK("camera recovery resumes", !Rig::still());
  rig.ir = false;
  rig.run(400);
  CHECK("IR loss stops", Rig::still());
  rig.ir = true;
  rig.run(500);
  CHECK("IR recovery resumes", !Rig::still());
  CHECK("still inside the reply window rule", !rig.sentDuringReply);

  // One corrupted camera frame is absorbed by the freshness window.
  rig.corrupt = true;
  int stopped = 0;
  for (int i = 0; i < 300; ++i) { rig.tick(); stopped += Rig::still(); }
  CHECK("a single bad CRC does not stop", stopped == 0);

  // IR silent with a healthy camera: nothing tracks a ball, and stop frames keep flowing.
  rig.ir = false;
  rig.run(400);
  const size_t before = rig.sent.size();
  rig.run(600);
  CHECK("IR loss with a good camera stays stopped", Rig::still() && rig.sent.back().power == 0 &&
        rig.sent.size() > before + 5);
  rig.ir = true;
  rig.run(500);

  // A lost ball: the patrol must keep the motors turning, not sit in reverse pauses.
  rig.noBall = true;
  rig.run(700);
  stopped = 0;
  for (int i = 0; i < 4000; ++i) { rig.tick(); stopped += Rig::still(); }
  CHECK("searching keeps moving", stopped == 0 && Rig::legalDuty());
  rig.noBall = false;
  rig.run(700);

  rig.mask = uint16_t(1u << 8); rig.escape = 180;
  size_t from = rig.sent.size();
  bool legal = true;
  for (int i = 0; i < 600; ++i) { rig.tick(); legal = legal && Rig::legalDuty(); }
  CHECK("escape is sent as a strict avoid command", rig.sent.back().avoid && rig.sent.back().direction == 180 &&
        rig.sent.back().turn == 0 && rig.sent.size() > from);
  CHECK("escape runs all wheels in reverse", mockPwm[7] && mockPwm[13] && mockPwm[10] && mockPwm[8] && legal);
  rig.escape = 0xFFFF; rig.mask = 0;
  rig.run(100);
  CHECK("no open direction stops", Rig::still());
  rig.mask = 0xFFFF;
  rig.run(500);

  // BNO absent for the whole run so far: no heading, no turning, little I2C traffic.
  CHECK("absent BNO is probed at most about once a second", Wire.transactions < 20);
  Wire.present = true; Wire.yawRaw = 90 * 16;
  rig.run(1500);
  CHECK("BNO arriving mid-run causes no spin", rig.sent.back().turn == 0);
  Wire.present = false;
  bool moved = false;
  for (int i = 0; i < 1500; ++i) { rig.tick(); moved = moved || !Rig::still(); }  // includes a recover reversal
  CHECK("BNO leaving again causes no spin", rig.sent.back().turn == 0 && moved);

  Serial.type("d");
  rig.run(300);
  CHECK("USB hold stops", rig.sent.back().power == 0 && Rig::still());
  Serial.type("g");
  moved = false;
  for (int i = 0; i < 500; ++i) { rig.tick(); moved = moved || !Rig::still(); }
  CHECK("USB resume", moved);

  // MOTOR alone (MAIN silent): watchdog, strict avoid, zero power.
  rig.camera = false;
  Serial.rx.clear();
  mockMs += 500;
  std::swap(Serial1, rig.motorBus);
  motorboard::loop();
  std::swap(Serial1, rig.motorBus);
  CHECK("watchdog stops a silent bus", Rig::still());
  mockMs += 500;
  sendRaw(rig, 22, 175, 0, true);
  CHECK("inexact avoid direction does not move", Rig::still());
  mockMs += 500;
  sendRaw(rig, 90, 175, 30, true);
  CHECK("avoid ignores rotation", pinsMatchVerified(1, mockPwm[10]) && !Rig::still());
  mockMs += 500;
  sendRaw(rig, 0, 0, 30, false);
  CHECK("zero power always means stop", Rig::still());

  // Bench-verified directions through the real MOTOR loop.
  rig.camera = false;
  Serial.rx.clear();
  for (int row = 0; row < 4; ++row) {
    mockMs += 500;
    std::swap(Serial1, rig.motorBus);
    for (int i = 0; i < 500; ++i) {
      if (i % 20 == 0) {
        uint8_t f[12] = {0xA5, 3, uint8_t(i / 20), uint8_t((row * 90) >> 7), uint8_t((row * 90) & 127), 1, 32, 0, 61};
        f[11] = mainboard::calculateCRC(f);
        Serial1.feed(f, 12);
      }
      ++mockMs;
      motorboard::loop();
    }
    std::swap(Serial1, rig.motorBus);
    CHECK("upgrade output equals the bench-verified pattern at PWM 160", pinsMatchVerified(row, 160));
  }
  mockMs += 500;
  std::swap(Serial1, rig.motorBus);
  motorboard::loop();
  std::swap(Serial1, rig.motorBus);

  // IR sketch: both poll commands answer in their own format; a quiet bus gets no unsolicited bytes.
  irboard::setup();
  mockPin[18] = LOW;
  Serial1 = MockSerial();
  const uint8_t legacyPoll[] = {0xA5, 0x5A, 0x01, 0xFE}, newPoll[] = {0xA5, 0x5A, 0x02, 0xFD};
  Serial1.feed(legacyPoll, 4);
  irboard::loop();
  CHECK("legacy IR reply", Serial1.tx.size() == 6 && Serial1.tx[0] == 0xAA && Serial1.tx[5] == 0x55);
  Serial1.tx.clear();
  Serial1.feed(newPoll, 4);
  irboard::loop();
  brain::IrReceiver receiver;
  brain::IrData data;
  bool decoded = false;
  for (uint8_t b : Serial1.tx) decoded = receiver.push(b, data) || decoded;
  CHECK("extended IR reply decodes on MAIN", Serial1.tx.size() == 10 && decoded && data.angle == 0 && data.confidence > 50);
  Serial1.tx.clear();
  Serial.type("v");
  mockMs += 600;
  irboard::loop();
  CHECK("IR bench mode transmits nothing", Serial1.tx.empty());

  printf("loop checks: %d failed\n", failed);
  return failed ? 1 : 0;
}
