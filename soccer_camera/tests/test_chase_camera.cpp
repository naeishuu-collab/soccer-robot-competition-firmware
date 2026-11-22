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
  camera.mode = 2; camera.receivedMs = UINT32_MAX - 50;
  assert(cameraAllowsMotion(camera, 149));
  assert(!cameraAllowsMotion(camera, 150));
  chooseChaseMotion(camera, 149, true, 360, 180, direction, power);
  assert(power == 0);
  frame[5] = 5; frame[6] = 0x20; seal(frame);
  assert(!validCameraFrame(frame));
  frame[5] = 1; frame[6] = 0x20; seal(frame);
  assert(!validCameraFrame(frame));
  frame[6] = 0x21; frame[3] = 1; seal(frame);
  assert(!validCameraFrame(frame));
  uint8_t reply[12] = {};
  assert(chooseIrDirection(359, reply) == 359);
  reply[7] = 0; reply[8] = 1; reply[9] = 1;
  assert(chooseIrDirection(359, reply) == 1);
  reply[8] = 10;
  assert(chooseIrDirection(359, reply) == 359);
  reply[9] = 0;
  assert(chooseIrDirection(359, reply) == 359);
  reply[9] = 1; reply[7] = 3; reply[8] = 0;
  assert(chooseIrDirection(359, reply) == 359);
  for (int measured = 0; measured < 360; ++measured) {
    for (int offset = -10; offset <= 10; ++offset) {
      const int predicted = (measured + offset + 360) % 360;
      reply[7] = uint8_t(predicted >> 7); reply[8] = uint8_t(predicted & 127);
      reply[9] = 1; reply[10] = 0;
      assert(chooseIrDirection(uint16_t(measured), reply) ==
             (offset >= -5 && offset <= 5 ? predicted : measured));
    }
  }
}
