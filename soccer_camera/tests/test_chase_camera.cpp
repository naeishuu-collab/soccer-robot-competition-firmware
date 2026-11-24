#include <stdint.h>
#include <math.h>
#include "chase_camera_logic.h"
#include <assert.h>

static void seal(uint8_t *frame) {
  frame[7] = 0;
  for (int i = 0; i < 7; ++i) frame[7] ^= frame[i];
}

int main() {
  CameraState camera;
  assert(!cameraAllowsMotion(camera, 0));
  uint8_t frame[8] = {0x20, 0, 0, 0xFF, 0, 2, 0x40, 0};
  seal(frame);
  assert(validCameraFrame(frame));
  frame[1] = 1;
  assert(!validCameraFrame(frame));
  seal(frame);
  assert(!validCameraFrame(frame));
  frame[5] = 1; frame[6] = 0x41; seal(frame);
  assert(validCameraFrame(frame));
  camera.received = true; camera.mode = 2; camera.receivedMs = 10; camera.sectors = 0xFF;
  assert(cameraAllowsMotion(camera, 210));
  assert(!cameraAllowsMotion(camera, 211));
  camera.mode = 0;
  assert(!cameraAllowsMotion(camera, 10));
  assert(cameraAvoidDirection(0, 1) == 0);
  assert(cameraAvoidDirection(1, 0) == 90);
  assert(cameraAvoidDirection(0, -1) == 180);
  assert(cameraAvoidDirection(-1, 0) == 270);
  uint16_t direction, power;
  camera.mode = 2;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(direction == 270 && power == 180);
  chooseChaseMotion(camera, 10, false, 270, 180, 128, direction, power);
  assert(power == 0);
  chooseChaseMotion(camera, 211, true, 270, 180, 128, direction, power);
  assert(power == 0);
  camera.mode = 1; camera.x = 1; camera.y = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(direction == 90 && power == 128);
  camera.sectors &= ~(1u << 2);
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(power == 0);
  camera.mode = 2;
  chooseChaseMotion(camera, 10, true, 90, 180, 128, direction, power);
  assert(power == 0);
  camera.mode = 1;
  camera.x = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(power == 0);
  camera.mode = 2; camera.receivedMs = UINT32_MAX - 50;
  assert(cameraAllowsMotion(camera, 149));
  assert(!cameraAllowsMotion(camera, 150));
  chooseChaseMotion(camera, 149, true, 360, 180, 128, direction, power);
  assert(power == 0);
  frame[5] = 5; frame[6] = 0x20; seal(frame);
  assert(!validCameraFrame(frame));
  frame[5] = 1; frame[6] = 0x20; seal(frame);
  assert(!validCameraFrame(frame));
  frame[6] = 0x41; frame[3] = 0; seal(frame);
  assert(!validCameraFrame(frame));
  IrReplyReceiver ir;
  uint16_t angle = 0, strength = 0;
  assert(!ir.push(0x55, angle, strength));
  // Start/end marker values are also legal inside the count.
  const uint8_t valid[] = {0xAA, 1, 103, 0xAA, 0x55, 0x55};
  for (unsigned i = 0; i < sizeof(valid) - 1; ++i) {
    assert(!ir.push(valid[i], angle, strength));
  }
  assert(ir.push(valid[5], angle, strength));
  assert(angle == 359 && strength == 0xAA55);

  const uint8_t invalidAngle[] = {0xAA, 1, 104, 0, 1, 0x55};
  for (uint8_t byte : invalidAngle) assert(!ir.push(byte, angle, strength));
  assert(angle == 359 && strength == 0xAA55);

  const uint8_t invalidEnd[] = {0xAA, 0, 90, 0, 1, 0xAA};
  for (uint8_t byte : invalidEnd) assert(!ir.push(byte, angle, strength));
  for (unsigned i = 1; i < sizeof(valid) - 1; ++i) {
    assert(!ir.push(valid[i], angle, strength));
  }
  assert(ir.push(valid[5], angle, strength));

  assert(!ir.push(0xAA, angle, strength));
  ir.reset();
  assert(!ir.push(0x55, angle, strength));
}
