// I2C with one BNO055-like device whose presence, yaw and calibration the test controls.
#pragma once
#include "Arduino.h"
struct MockWire {
  bool present = false;
  uint16_t yawRaw = 0;       // degrees * 16
  uint8_t calibration = 0xFF;
  uint32_t transactions = 0;
  uint8_t reg = 0, count = 0;
  std::deque<uint8_t> out;
  void setSDA(int) {}
  void setSCL(int) {}
  void begin() {}
  void setClock(uint32_t) {}
  void setTimeout(uint32_t, bool = false) {}
  void beginTransmission(uint8_t) { count = 0; ++transactions; }
  void write(uint8_t v) { if (!count++) reg = v; }
  int endTransmission(bool = true) { return present ? 0 : 4; }
  uint8_t requestFrom(uint8_t, uint8_t size) {
    out.clear();
    if (!present) return 0;
    if (reg == 0x00) out = {0xA0};
    else if (reg == 0x35) out = {calibration};
    else out = {uint8_t(yawRaw & 0xFF), uint8_t(yawRaw >> 8)};
    return size;
  }
  int read() { if (out.empty()) return 0; const int v = out.front(); out.pop_front(); return v; }
};
inline MockWire Wire;
