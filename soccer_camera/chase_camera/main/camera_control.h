#pragma once
#include <stdint.h>
#include <math.h>

struct CameraState {
  uint8_t mode = 0;
  int8_t x = 0;
  int8_t y = 0;
  uint32_t receivedMs = 0;
  bool received = false;
};

inline bool validCameraFrame(const uint8_t *f) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < 7; ++i) crc ^= f[i];
  if (f[0] != 0x20 || f[7] != crc) return false;
  for (uint8_t i = 1; i <= 3; ++i)
    if (f[i] > 100 && f[i] < 156) return false;
  const uint8_t mode = f[5];
  if (mode > 4 || f[6] != (mode == 1 ? 0x21 : 0x20) || f[4] != (mode == 4)) return false;
  if (mode == 1) return f[3] == 0;
  if (mode == 3) return f[1] == 0 && f[2] == 0;
  return f[1] == 0 && f[2] == 0 && f[3] == 0;
}

inline bool cameraAllowsMotion(const CameraState &camera, uint32_t nowMs) {
  return camera.received && uint32_t(nowMs - camera.receivedMs) <= 200 && camera.mode != 0;
}

inline uint16_t cameraAvoidDirection(int8_t x, int8_t y) {
  float angle = atan2f(float(x), float(y)) * 57.2957795f;
  if (angle < 0) angle += 360;
  return uint16_t(angle + 0.5f) % 360;
}
