#define ROBOT_ZIP_TRACK 1
// Runs the real MAIN, MOTOR and IR sketches (setup/loop, ROBOT_UPGRADE=1) wired together on the host.
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
  bool camera = true, ir = true, corrupt = false, noBall = false, goal = true;
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
    put(14, goal ? 0 : 0xFFFF); f[16] = 255; f[17] = goal ? 90 : 0; f[18] = 20; f[19] = goal ? 0 : 255;
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
      const uint16_t raw = uint16_t((360 - ballAngle) % 360);
      const uint16_t strength = noBall ? 0 : 400;
      const uint8_t wire[] = {0xAA, uint8_t(raw >> 8), uint8_t(raw), uint8_t(strength >> 8), uint8_t(strength), 0x55};
      mainBus.feed(wire, sizeof(wire));
    }
    std::swap(Serial1, mainBus);
    mainboard::loop();
    std::vector<uint8_t> out;
    out.swap(Serial1.tx);
    std::swap(Serial1, mainBus);
    for (size_t i = 0; i < out.size();) {
      if (i + 4 <= out.size() && out[i] == 0xA5 && out[i + 1] == 0x5A) {
        CHECK("ZIP IR poll", out[i + 2] == 1 && out[i + 3] == 0xFE);
        if (ir) { pollPending = true; replyAt = mockMs + 6; }
        i += 4;
      } else if (i + 12 <= out.size() && out[i] == 0xA5 && out[i + 1] == 4) {
        const uint8_t *f = &out[i];
        if (pollPending) sentDuringReply = true;
        const int left = int(f[3] << 7 | f[4]) - 210;
        const int right = int(f[7] << 7 | f[8]) - 210;
        sent.push_back({uint16_t(left + 210), uint16_t(right + 210), 0, false});
        motorBus.feed(f, 12);
        i += 12;
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

int main() {
  mockMs = 1000;
  mainboard::setup(); motorboard::setup();
  const int16_t forward[] = {175,175,175,175}, reverse[] = {-175,-175,-175,-175};
  const int16_t turning[] = {140,140,210,210};
  CHECK("forward sector permits straight", mainboard::zipPathClear(1, forward));
  CHECK("rear sector permits reverse", mainboard::zipPathClear(1u << 8, reverse));
  CHECK("front closed blocks forward", !mainboard::zipPathClear(1u << 8, forward));
  CHECK("rear closed blocks reverse", !mainboard::zipPathClear(1, reverse));
  CHECK("turn requires calibrated full sweep", !mainboard::zipPathClear(0xfffe, turning));
  brain::CameraParser parser; brain::CameraData parsed;
  uint8_t camera[brain::kCameraSize] = {0xC5,0x5C,brain::kCameraVersion,brain::kCameraSize,1,1};
  camera[6] = camera[7] = 0xff; camera[8] = camera[9] = camera[10] = camera[11] = 0xff;
  camera[12] = 255;
  for (int at : {14,20,26}) { camera[at] = 255; camera[at+1] = 255; }
  camera[16] = camera[19] = camera[22] = camera[25] = 255;
  const uint16_t crc = brain::crc16(camera, brain::kCameraSize - 2);
  camera[30] = uint8_t(crc); camera[31] = uint8_t(crc >> 8);
  for (int i = 0; i < 8; ++i) parser.push(camera[i], 0, parsed);
  bool accepted = false;
  for (int i = 0; i < brain::kCameraSize; ++i) accepted |= parser.push(camera[i], 25, parsed);
  CHECK("partial stale camera frame resynchronises", accepted && parsed.received && parsed.mask == 0xffff);
  Rig rig; rig.goal = false; rig.run(600);
  CHECK("raw IR to ZIP forward PWM", mockPwm[6] == 175 && mockPwm[12] == 175 && mockPwm[11] == 175 && mockPwm[9] == 175);
  CHECK("no overlapping IR response", !rig.sentDuringReply);
  rig.ballAngle = 90; rig.run(300);
  // raw bearing is 270: error -90 -> ZIP left 210, right -140.
  CHECK("angle correction equals ZIP output", mockPwm[6] == 210 && mockPwm[12] == 210 && mockPwm[10] == 140 && mockPwm[8] == 140);
  rig.noBall = true; rig.run(100); CHECK("no ball stops", Rig::still());
  rig.noBall = false; rig.ballAngle = 0; rig.run(200); CHECK("ball returns", !Rig::still());
  rig.ir = false; rig.run(250); CHECK("IR timeout stops", Rig::still());
  rig.ir = true; rig.run(250); CHECK("IR resumes", !Rig::still());
  rig.camera = false; rig.run(300); CHECK("camera timeout stops", Rig::still());
  rig.camera = true; rig.run(250); CHECK("camera resumes", !Rig::still());
  rig.mask = 1; rig.run(150); CHECK("unrelated closed sectors allow straight motion", !Rig::still());
  rig.mask = 0; rig.run(150); CHECK("unsafe mask stops", Rig::still());
  rig.mask = 0xffff; rig.run(200); CHECK("mask returns", !Rig::still());
  rig.mask = uint16_t(1u << 8); rig.escape = 180; rig.run(600);
  CHECK("line escape moves away", mockPwm[7] && mockPwm[13] && mockPwm[10] && mockPwm[8]);
  rig.escape = 0xffff; rig.mask = 0xffff; rig.run(600);
  CHECK("tracking resumes after escape", mockPwm[6] && mockPwm[12] && mockPwm[11] && mockPwm[9]);
  Serial.type("d"); rig.run(100); CHECK("manual stop", Rig::still());
  Serial.type("g"); rig.run(150); CHECK("manual resume", !Rig::still());
  mockMs += 101; std::swap(Serial1, rig.motorBus); motorboard::loop(); std::swap(Serial1, rig.motorBus);
  CHECK("motor watchdog", Rig::still());
  printf("ZIP integrated checks: %d failed\n", failed);
  return failed ? 1 : 0;
}
