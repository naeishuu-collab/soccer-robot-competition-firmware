#include <stdint.h>
#include <math.h>
#include <assert.h>
#include "no_camera_ir_logic.h"
int main() {
  IrTestController c;
  Motion motion{1, 1, 1};
  c.enableAutoStart();
  assert(!c.armed() && !c.update(true, true, 0, motion));
  assert(motion.x == 0 && motion.y == 0 && motion.turn == 0);
  assert(!c.armOnFirstFreshBall(false, true, 0));
  assert(!c.armOnFirstFreshBall(true, false, 0));
  assert(!c.armOnFirstFreshBall(true, true, 360));
  assert(c.armOnFirstFreshBall(true, true, 90));
  assert(c.update(true, true, 90, motion) && motion.x > 0);
  assert(!c.update(false, true, 90, motion));
  assert(motion.x == 0 && motion.y == 0 && motion.turn == 0);
  assert(c.update(true, true, 90, motion));
  c.disableAutoStart(); c.disarm();
  assert(!c.armOnFirstFreshBall(true, true, 0));
  assert(!c.update(true, true, 0, motion));
  assert(motion.x == 0 && motion.y == 0 && motion.turn == 0);
  c.enableAutoStart();
  assert(c.armOnFirstFreshBall(true, true, 0));
}
