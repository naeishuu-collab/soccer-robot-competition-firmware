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
  uint8_t frame[8] = {0x20, 0, 0, 0, 0, 2, 0x20, 0};
  seal(frame);
  assert(validCameraFrame(frame));
  frame[1] = 1;
  assert(!validCameraFrame(frame));
  seal(frame);
  assert(!validCameraFrame(frame));
  frame[5] = 1; frame[6] = 0x21; seal(frame);
  assert(validCameraFrame(frame));
  camera.received = true; camera.mode = 2; camera.receivedMs = 10;
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
  chooseChaseMotion(camera, 10, true, 270, 180, direction, power);
  assert(direction == 270 && power == 180);
  chooseChaseMotion(camera, 10, false, 270, 180, direction, power);
  assert(power == 0);
  chooseChaseMotion(camera, 211, true, 270, 180, direction, power);
  assert(power == 0);
  camera.mode = 1; camera.x = 1; camera.y = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, direction, power);
  assert(direction == 90 && power == 180);
  camera.x = 0;
  chooseChaseMotion(camera, 10, true, 270, 180, direction, power);
  assert(power == 0);
}
