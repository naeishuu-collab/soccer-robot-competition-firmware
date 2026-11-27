#define ROBOT_ZIP_TRACK 0
#define ROBOT_ZIP_COMPAT 1
#include <Arduino.h>
#include <SerialPIO.h>
#include <Wire.h>
#include <assert.h>
#include "main.ino"

static bool cameraOn = true, irOn = true, line = false, pending = false;
static uint32_t replyAt = 0;
static uint8_t sequence = 0;
static uint16_t lastAngle = 0, lastCount = 0;
static int commands = 0;

static void sendCameraFrame() {
  uint8_t f[brain::kCameraSize] = {0xC5, 0x5C, brain::kCameraVersion, brain::kCameraSize, sequence++, 1};
  auto put = [&](int at, uint16_t v) { f[at] = uint8_t(v); f[at + 1] = uint8_t(v >> 8); };
  put(6, line ? 0xFFFE : 0xFFFF); put(8, 0xFFFF); put(10, 0xFFFF);
  f[12] = 255;
  for (int at : {14, 20, 26}) put(at, 0xFFFF);
  f[16] = f[19] = f[22] = f[25] = 255;
  put(30, brain::crc16(f, brain::kCameraSize - 2));
  cameraSerial.feed(f, sizeof(f));
}

static void run(int ms) {
  while (ms--) {
    ++mockMs;
    if (cameraOn && mockMs % 50 == 0) sendCameraFrame();
    if (pending && mockMs >= replyAt) {
      pending = false;
      if (irOn) { const uint8_t reply[] = {0xAA, 0x01, 0x0E, 0x01, 0xF4, 0x55}; Serial1.feed(reply, sizeof(reply)); }
    }
    loop();
    auto &out = Serial1.tx;
    for (size_t i = 0; i < out.size();) {
      if (i + 4 <= out.size() && out[i] == 0xA5 && out[i + 1] == 0x5A) {
        assert(out[i + 2] == 1 && out[i + 3] == 0xFE);
        pending = true; replyAt = mockMs + 6; i += 4;
      } else {
        assert(i + 8 <= out.size() && out[i] == 0xC3 && out[i + 1] == 0x3C && out[i + 7] == 0x5A);
        assert(out[i + 6] == uint8_t(out[i + 2] ^ out[i + 3] ^ out[i + 4] ^ out[i + 5]));
        lastAngle = uint16_t(out[i + 2] << 8 | out[i + 3]);
        lastCount = uint16_t(out[i + 4] << 8 | out[i + 5]);
        ++commands; i += 8;
      }
    }
    out.clear();
  }
}

int main() {
  setup();
  run(200);
  assert(commands && lastAngle == 270 && lastCount == 500);  // exact ZIP relay, BNO off and no goal
  line = true; run(100);
  assert(lastCount == 0 && Serial.text.find("line or unsafe direction") != std::string::npos);
  line = false; run(100); assert(lastAngle == 270 && lastCount == 500);
  Serial.type("s"); run(50); assert(lastCount == 0 && Serial.text.find("manual stop") != std::string::npos);
  Serial.type("g"); run(100); assert(lastCount == 500);
  irOn = false; run(40); assert(lastCount == 500);
  run(100);
  assert(lastCount == 0 && Serial.text.find("IR link lost") != std::string::npos);
  irOn = true; run(100); assert(lastCount == 500);
  cameraOn = false; run(300);
  assert(lastCount == 0 && Serial.text.find("camera link") != std::string::npos);
  puts("ZIP compatibility loop passed");
}
