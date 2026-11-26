// Keep driving while the links and observations are valid; still stop where safety needs it.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <initializer_list>
#include "main_logic.h"
#include "world.h"
using namespace brain;

static int failed = 0;
#define CHECK(name, condition) \
  do { if (!(condition)) { ++failed; printf("FAIL %s (line %d)\n", name, __LINE__); } } while (0)

static float apart(float a, float b) { return fabsf(wrap(a - b)); }
static void cameraBall(World &w, float bearing) {
  w.cam.ball.seen = true; w.cam.ball.bearing = bearing; w.cam.ball.confidence = 80;
}
// Steps with a fresh camera ball observation every frame.
static Output runWithCameraBall(World &w, int steps) {
  Output o;
  for (int i = 0; i < steps; ++i) { w.cam.ball.stampMs = w.now + 20; o = w.run(); }
  return o;
}

static void mustContinue() {
  World ir;
  ir.goal(0); cameraBall(ir, 0); ir.irOn = false;
  CHECK("camera ball replaces a dead IR link", runWithCameraBall(ir, 20).power > 0);

  World lost;
  lost.goal(0); lost.ir.angle = 90; lost.run(10);
  lost.ball = false;
  int moving = 0;
  bool directions[360] = {};
  Output o;
  for (int i = 0; i < 400; ++i) { o = lost.run(); if (o.power) { ++moving; directions[o.direction] = true; } }
  int distinct = 0;
  for (bool d : directions) distinct += d;
  CHECK("a lost ball is searched for, not waited for", moving > 380 && o.power && o.tactic == Tactic::SEARCH);
  CHECK("the search changes direction", distinct >= 3);
  lost.ball = true; lost.ir.angle = 270;
  o = lost.run();
  CHECK("a found ball is reacquired at once", strcmp(name(lost.robot.ball.state), "REACQUIRE") == 0 && o.power &&
        o.tactic != Tactic::SEARCH && o.tactic != Tactic::ATTACK);
  lost.run(10);
  CHECK("then tracked", lost.robot.ball.state == BallState::TRACK);

  World unknown;
  unknown.ir.angle = 20;
  bool never = true, always = true;
  for (int i = 0; i < 200; ++i) {
    o = unknown.run();
    if (i > 5) always = always && o.power;
    never = never && (!o.power || apart(o.direction, 20) >= 90);
  }
  CHECK("unknown attack direction: reposition instead of stopping", always);
  CHECK("unknown attack direction: never approach the ball", never);

  World own;
  own.goal(0); own.own(0); own.ir.angle = 0;
  o = own.run(10);
  CHECK("own goal ahead: another way is chosen", o.power && apart(o.direction, 0) >= 45);

  World blocked;
  blocked.goal(0); blocked.ir.angle = 0; blocked.cam.mask = uint16_t(1u << 8);
  o = blocked.run(10);
  CHECK("wanted direction closed: take the open one, even backwards", o.power && o.direction == 180 && o.avoid);

  World vanish;
  vanish.goal(0); vanish.ir.angle = 0; vanish.run(10);
  vanish.cam.yellow.seen = false;
  o = vanish.run(35);
  CHECK("goal briefly hidden: keep attacking on memory", o.tactic == Tactic::ATTACK && o.power);

  World slow;
  slow.goal(0); slow.ir.angle = 0; slow.run(10);
  slow.cam.ready = false; slow.cam.slow = true; slow.cam.mask = 0;
  o = slow.run(3);
  CHECK("one slow camera frame does not stop", o.power && o.avoid);
  o = slow.run(8);
  CHECK("held line data expires", !o.power);

  World farOrbit;
  farOrbit.goal(0); farOrbit.ir.angle = 90; farOrbit.cam.ball.distanceCm = 100;
  cameraBall(farOrbit, 90);
  o = runWithCameraBall(farOrbit, 20);
  CHECK("orbiting a far ball keeps moving", o.tactic == Tactic::ORBIT && o.power);

  World held;
  held.ir.angle = 20; held.run(10);
  held.cam.ready = false; held.cam.slow = true;
  o = held.run(2);
  CHECK("held line data still repositions in an exact direction", o.power && o.direction % 45 == 0 &&
        apart(o.direction, 20) >= 89);

  CHECK("near lines a bench-verified direction beats a slightly closer diagonal",
        safeDirection(uint16_t((1u << 0) | (1u << 2)), 30) == 0 && nearestExact(uint16_t((1u << 0) | (1u << 2)), 30) == 0);
  CHECK("a diagonal is still used when clearly better",
        safeDirection(uint16_t((1u << 0) | (1u << 2)), 44) == 45);

  World escape;
  escape.cam.escaping = true; escape.cam.escape = 113;
  escape.cam.mask = uint16_t((1u << 5) | (1u << 4)); escape.goal(0);
  o = escape.run(3);
  CHECK("inexact escape: use the exact open neighbour", o.power && o.direction == 90 && o.avoid);
}

static void mustStop() {
  World w;
  w.goal(0); w.ir.angle = 0; w.run(10);
  w.cam.ready = false; w.cam.slow = false; w.cam.mask = 0;
  CHECK("floor not recognised stops at once", !w.run().power);
  w.cam.ready = true;
  CHECK("no safe direction stops", !w.run(2).power);
  w.cam.mask = 0xFFFF;
  w.run(5);
  w.irOn = false;
  CHECK("IR dead without a substitute stops", !w.run(10).power);
  w.irOn = true; w.run(5);
  w.camOn = false;
  CHECK("camera loss stops", !w.run(15).power);

  World never;
  never.ball = false; never.goal(0);
  CHECK("no ball ever seen: wait for the first sighting", !never.run(100).power);

  World trap;
  trap.goal(180); trap.own(0); trap.ir.angle = 0; trap.cam.mask = 1u;
  CHECK("only open direction pushes toward our goal: stop", !trap.run(10).power);

  World lost;
  lost.goal(0); lost.ir.angle = 0; lost.run(10);
  lost.ball = false; lost.run(100);
  lost.irOn = false;
  CHECK("searching with no ball sensor left stops", !lost.run(10).power);
}

int main() {
  mustContinue();
  mustStop();
  printf("continuity checks: %d failed\n", failed);
  return failed ? 1 : 0;
}
