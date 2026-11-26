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
  assert(cameraAllowsMotion(camera, 10));
  assert(cameraAvoidDirection(0, 1) == 0);
  assert(cameraAvoidDirection(1, 0) == 90);
  assert(cameraAvoidDirection(0, -1) == 180);
  assert(cameraAvoidDirection(-1, 0) == 270);
  assert(headingCorrection(2, 358, 1) == 4);
  assert(headingDifference(2, 358) == 4);
  assert(headingDifference(358, 2) == -4);
  assert(headingCorrection(358, 2, 1) == -4);
  assert(headingCorrection(90, 0, 1) == 40);
  assert(headingCorrection(2, 358, -1) == -4);
  uint16_t direction, power;
  camera.mode = 2;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(direction == 270 && power == 180);
  camera.sectors = 1u << 1;
  chooseChaseMotion(camera, 10, true, 30, 180, 128, direction, power);
  assert(direction == 45 && power == 180);
  chooseChaseMotion(camera, 10, true, 20, 180, 128, direction, power);
  assert(direction == 45 && power == 180);
  camera.sectors = 0xFF;
  chooseChaseMotion(camera, 10, true, 359, 180, 128, direction, power);
  assert(direction == 0 && power == 180);
  chooseChaseMotion(camera, 10, false, 270, 180, 128, direction, power);
  assert(power == 0);  // no ball bearing: do not drive
  camera.mode = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(power == 0);  // camera cannot confirm a safe direction
  camera.mode = 2;
  chooseChaseMotion(camera, 211, true, 270, 180, 128, direction, power);
  assert(power == 0);
  camera.mode = 1; camera.x = 1; camera.y = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(direction == 90 && power == 128);
  camera.sectors &= ~(1u << 2);
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(power == 128);
  camera.mode = 2;
  chooseChaseMotion(camera, 10, true, 90, 180, 128, direction, power);
  assert(power == 180);
  camera.mode = 1;
  camera.x = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, 128, direction, power);
  assert(power == 128);
  camera.mode = 2; camera.receivedMs = UINT32_MAX - 50;
  assert(cameraAllowsMotion(camera, 149));
  assert(!cameraAllowsMotion(camera, 150));
  chooseChaseMotion(camera, 149, true, 360, 180, 128, direction, power);
  assert(power == 0);
  chooseChaseMotion(camera, 149, true, 90, 180, 128, direction, power);
  assert(power == 180);
  frame[5] = 5; frame[6] = 0x20; seal(frame);
  assert(!validCameraFrame(frame));
  frame[5] = 1; frame[6] = 0x20; seal(frame);
  assert(!validCameraFrame(frame));
  frame[6] = 0x41; frame[3] = 0; seal(frame);
  assert(!validCameraFrame(frame));
  IrBallLock lock;
  assert(!lock.active(0));
  lock.update(10, true, 358);
  lock.update(40, true, 3);
  assert(lock.active(40) && lock.direction() == 3);
  lock.update(70, true, 178);
  assert(lock.active(70) && lock.direction() == 3);
  lock.update(100, false, 0);
  assert(lock.active(130) && !lock.active(131));
  lock.update(131, true, 177);
  assert(lock.active(131) && lock.direction() == 177);
  lock.update(160, true, 310);
  lock.update(190, true, 322);
  assert(lock.active(190) && lock.direction() == 322);
  lock.reset();
  assert(!lock.active(190));
  lock.update(UINT32_MAX - 20, true, 90);
  assert(lock.active(69) && !lock.active(70));
  lock.update(70, true, 360);
  assert(!lock.active(70));

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
