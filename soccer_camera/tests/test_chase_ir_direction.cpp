#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <assert.h>
#include <string.h>
#include "chase_ir_config.h"
#include "chase_ir_prediction.h"

using std::min;
constexpr float PI = 3.14159265358979323846f;
uint16_t detectCount[SENSOR_COUNT]{}, ballDirection = 0, ballStrength = 0;
uint16_t predictedDirection = 0, directionCoherence = 0;
uint32_t totalDetectRaw = 0;
bool predictionValid = false;
IrPredictor predictor;
uint32_t micros() { return 30000; }

#include "chase_ir_direction.h"

int main() {
  for (int i = 0; i < SENSOR_COUNT; ++i) {
    assert(activeSensorMask(UINT32_MAX ^ (1u << (18 + i))) == (1u << i));
    memset(detectCount, 0, sizeof(detectCount));
    detectCount[i] = 100;
    predictor.reset();
    calculateDirection();
    assert(sensorPins[i] == 18 + i);
    assert(ballDirection == (360 - 30 * i) % 360);
    assert(ballStrength > 0 && !predictionValid);
  }
  assert(activeSensorMask(UINT32_MAX) == 0);
  memset(detectCount, 0, sizeof(detectCount));
  detectCount[0] = 100;
  uint16_t unused;
  predictor.reset();
  predictor.update(358, 10000, unused);
  predictor.update(359, 20000, unused);
  calculateDirection();
  assert(ballDirection == 0 && !predictionValid);
  detectCount[1] = 100;
  predictor.reset();
  calculateDirection();
  assert(ballDirection == 345);
  memset(detectCount, 0, sizeof(detectCount));
  calculateDirection();
  assert(ballStrength == 0 && !predictionValid && predictor.count == 0);
}
