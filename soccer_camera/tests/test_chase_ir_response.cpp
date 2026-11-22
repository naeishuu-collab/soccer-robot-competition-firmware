#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include "chase_ir_prediction.h"

constexpr uint8_t FRAME_SIZE = 12, FRAME_START = 0xA5, MSG_BALL = 2;
constexpr uint8_t RS485_DE_PIN = 2, HIGH = 1, LOW = 0;
uint16_t ballDirection = 359, ballStrength = 400, predictedDirection = 1;
bool predictionValid = true, hasSample = true;
uint32_t lastSampleUs = 1000, nowUs = 26000, replyCount = 0;
uint8_t sent[FRAME_SIZE]{};
uint32_t micros() { return nowUs; }
void digitalWrite(uint8_t, uint8_t) {}
void delayMicroseconds(uint32_t) {}
void packValue(uint8_t *p, uint16_t v) { p[0] = (v >> 7) & 127; p[1] = v & 127; }
uint8_t frameCrc(const uint8_t *) { return 42; }
struct MockSerial {
  void write(const uint8_t *p, size_t n) { assert(n == FRAME_SIZE); memcpy(sent, p, n); }
  void flush() {}
} Serial1;

#include "chase_ir_response.h"

int main() {
  sendBall(130);
  assert(sent[0] == FRAME_START && sent[1] == MSG_BALL && sent[2] == 2 && sent[11] == 42);
  assert(sent[3] == 2 && sent[4] == 103 && sent[5] == 3 && sent[6] == 16);
  assert(sent[7] == 0 && sent[8] == 1 && sent[9] == 1 && sent[10] == 0);
  nowUs++;
  sendBall(3);
  for (int i = 3; i <= 10; ++i) assert(sent[i] == 0);
  hasSample = false; nowUs = lastSampleUs;
  sendBall(4);
  for (int i = 3; i <= 10; ++i) assert(sent[i] == 0);
  hasSample = true; ballStrength = 0;
  sendBall(5);
  assert(sent[5] == 0 && sent[6] == 0 && sent[7] == 0 && sent[8] == 0 && sent[9] == 0);
  assert(replyCount == 4);
}
