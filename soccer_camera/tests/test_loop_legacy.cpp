// The default build (ROBOT_UPGRADE=0): real MAIN and MOTOR loops wired together on the host.
#define ROBOT_UPGRADE 0
#include <Arduino.h>
#include <SerialPIO.h>
#include <Wire.h>
#include <stdio.h>
#include "main_logic.h"
#include "motor_logic.h"

namespace mainboard {
#include "main.ino"
}
namespace motorboard {
#include "motor.ino"
}

static int failed = 0;
#define CHECK(name, condition) \
  do { if (!(condition)) { ++failed; printf("FAIL %s (line %d, t=%u)\n", name, __LINE__, mockMs); } } while (0)

struct Rig {
  MockSerial mainBus, motorBus;
  bool camera = true, ir = true, ball = true, replay = false, pollPending = false;
  uint8_t mode = 2, sectors = 0xFF, sequence = 0;
  int8_t x = 0, y = 0;
  uint32_t lastCameraMs = 0, replyAt = 0;
  uint16_t direction = 0, power = 0;
  bool avoid = false;

  void tick() {
    ++mockMs;
    if (camera && mockMs - lastCameraMs >= 50) {
      lastCameraMs = mockMs;
      if (!replay) ++sequence;
      uint8_t f[8] = {0x20, uint8_t(x), uint8_t(y), sectors, sequence, mode, uint8_t(mode == 1 ? 0x41 : 0x40), 0};
      for (int i = 0; i < 7; ++i) f[7] ^= f[i];
      mainboard::cameraSerial.feed(f, 8);
    }
    if (pollPending && mockMs >= replyAt) {
      pollPending = false;
      const uint8_t reply[] = {0xAA, 0, 0, 0, uint8_t(ball ? 50 : 0), 0x55};
      mainBus.feed(reply, 6);
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
        direction = uint16_t(out[i + 3] << 7 | out[i + 4]);
        power = uint16_t(out[i + 5] << 7 | out[i + 6]);
        avoid = out[i + 7] == 1;
        motorBus.feed(&out[i], 12);
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
};

int main() {
  mockMs = 0xFFFFFFFFu - 1500;
  mainboard::setup();
  motorboard::setup();
  Rig rig;
  rig.ball = false;
  rig.run(800);
  CHECK("no motion at power-on without a ball", Rig::still() && rig.power == 0);
  rig.ball = true;
  rig.run(600);
  CHECK("tracks the ball", rig.power == 160 && !Rig::still());
  rig.ball = false;
  rig.run(400);
  CHECK("short ball loss holds the bearing", rig.power == 160);
  rig.run(400);
  CHECK("longer ball loss stops", rig.power == 0 && Rig::still());
  rig.ball = true;
  rig.run(600);
  rig.mode = 0; rig.sectors = 0;
  rig.run(120);
  CHECK("camera WAIT stops", rig.power == 0 && Rig::still());
  rig.mode = 1; rig.y = -35; rig.sectors = 1u << 4;
  rig.run(600);
  CHECK("escape backwards at the running floor", rig.avoid && rig.direction == 180 && rig.power == 160 &&
        mockPwm[7] == 160 && mockPwm[13] == 160 && mockPwm[10] == 160 && mockPwm[8] == 160);
  rig.mode = 2; rig.y = 0; rig.sectors = 0xFF;
  rig.run(600);
  rig.camera = false;
  rig.run(400);
  CHECK("camera loss stops", Rig::still());
  rig.camera = true;
  rig.run(600);
  rig.replay = true;
  rig.run(600);
  CHECK("a repeated camera sequence stops", Rig::still());
  rig.replay = false;
  rig.run(600);
  CHECK("fresh frames resume", !Rig::still());
  rig.ir = false;
  rig.run(400);
  CHECK("IR loss stops", Rig::still());
  rig.ir = true;
  rig.run(600);

  Serial.type("s");
  rig.run(300);
  CHECK("USB hold stops", Rig::still());
  Serial.type("g");
  rig.run(600);

  // MOTOR diagnostics inside the real loop: entry stops, no immediate motion, exit cools down.
  rig.camera = false;
  Serial.rx.clear();
  auto motorStep = [&](const char *text) {
    Serial.type(text);
    std::swap(Serial1, rig.motorBus);
    motorboard::loop();
    std::swap(Serial1, rig.motorBus);
  };
  motorStep("diag\np 0 r 160\n");
  CHECK("pasted diag command does not reverse at once", Rig::still());
  mockMs += 300;
  motorStep("p 0 f 150\n");
  CHECK("diag pulse drives one wheel", mockPwm[6] == 150 && !mockPwm[12] && !mockPwm[11] && !mockPwm[9]);
  mockMs += 800;
  motorStep("");
  CHECK("diag pulse expires", Rig::still());
  motorStep("exit\n");
  CHECK("diag exit leaves the motors off", Rig::still());

  printf("legacy loop checks: %d failed\n", failed);
  return failed ? 1 : 0;
}
