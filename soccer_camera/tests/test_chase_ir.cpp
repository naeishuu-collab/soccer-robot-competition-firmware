#include <stdint.h>
#include <math.h>
#include <assert.h>
#include "chase_ir_prediction.h"

int main() {
  assert(vectorCoherencePermille(0, 0, 0) == 0);
  assert(vectorCoherencePermille(1000, 0, 1) == 1000);
  assert(vectorCoherencePermille(0, 0, 2) == 0);
  assert(vectorCoherencePermille(1000, 0, 2) == 500);
  IrPredictor p;
  uint16_t prediction = 0;
  assert(!p.update(357, 10000, prediction) && prediction == 357);
  assert(!p.update(359, 20000, prediction));
  assert(p.update(1, 30000, prediction) && prediction == 3);
  assert(!p.update(0, 40000, prediction) && prediction == 0);
  p.reset();
  assert(!p.update(20, 10000, prediction));
  assert(!p.update(20, 20000, prediction));
  assert(!p.update(20, 30000, prediction));
  p.reset();
  assert(!p.update(0, 10000, prediction));
  assert(!p.update(5, 20000, prediction));
  assert(p.update(10, 30000, prediction) && prediction == 15);
  assert(!p.update(300, 40000, prediction));
  p.reset();
  assert(!p.update(2, UINT32_MAX - 15000, prediction));
  assert(!p.update(1, UINT32_MAX - 5000, prediction));
  assert(p.update(0, 4999, prediction) && prediction == 359);
  p.reset();
  assert(!p.update(5, 10000, prediction));
  assert(!p.update(6, 60000, prediction));
  assert(!p.update(7, 70000, prediction));
  assert(!p.update(360, 80000, prediction) && p.count == 0);
}
