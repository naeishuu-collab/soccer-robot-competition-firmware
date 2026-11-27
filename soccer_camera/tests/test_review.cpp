// Pre-hardware safety review regressions. Every CHECK reproduces a defect found in e1cdd05.
#include <initializer_list>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_logic.h"
#include "main_logic.h"
#include "world.h"
using namespace brain;

static int failed = 0;
#define CHECK(name, condition) \
  do { if (!(condition)) { ++failed; printf("FAIL %s (line %d)\n", name, __LINE__); } } while (0)

static void motor() {
  using namespace wheel;
  Drive stall;
  stall.rampMs = 0;
  stall.command(0, 160, 0, 1000);
  stall.stop(1020);  // stopped before the boost could break the wheels loose
  stall.command(0, 160, 0, 1050);
  CHECK("restart after an unfinished boost is boosted again", stall.duty(0) == 180);

  Drive jump;
  jump.rampMs = 0;
  jump.command(0, 120, 0, 1000);
  jump.command(0, 210, 0, 1020);
  jump.update(1065);
  CHECK("boost end respects the acceleration limit", jump.duty(0) <= 185);

  // From a lopsided mix into a line escape: faster wheels must come down at once.
  Drive lop;
  lop.rampMs = 0;
  lop.command(30, 210, 0, 1000);
  lop.update(1100);
  lop.command(45, 175, 0, 1200, true);
  lop.update(1201);
  CHECK("escape starts with matched wheel duties", abs(lop.duty(1)) == abs(lop.duty(3)) && lop.duty(1) == lop.target(1) &&
        !lop.duty(0) && !lop.duty(2));  // 45 degrees drives M2 and M4 only

  Diag d;
  d.line("diag", 5000);
  d.line("p 0 r 160", 5001);
  CHECK("diag entry waits out the reverse pause", d.duty(0, 5002) == 0);
}

static void heading() {
  // BNO returns after a long outage with no vision: its zero says nothing about the field.
  World w;
  w.cam.heading = 0; w.cam.headingConfidence = 80; w.run(20);
  w.cam.heading = kUnknown; w.cam.headingConfidence = 0; w.run(200);
  w.bnoOn = true; w.yaw = 180; w.ir.angle = 0;
  Output o = w.run(30);
  CHECK("unreferenced BNO is not a field heading", !w.robot.heading.absolute && !isfinite(o.goal) && !o.turn);

  World b;
  b.bnoOn = true; b.yaw = 0; b.run(30);
  CHECK("power-on facing is not assumed to be the attack direction", !b.robot.heading.absolute);

  World s;
  s.cam.heading = 0; s.cam.headingConfidence = 80; s.bnoOn = true; s.run(30);
  s.cam.heading = kUnknown; s.cam.headingConfidence = 0; s.run(10);
  s.yaw = 90; s.run(3);
  CHECK("a persistent BNO step is not adopted at once",
        !s.robot.heading.absolute || fabsf(s.robot.heading.heading()) < 5);

  World m;
  m.cam.heading = 0; m.cam.headingConfidence = 90; m.bnoOn = true; m.goal(0); m.run(30);
  m.cam.heading = 120; m.run(8);
  CHECK("an outlier vision heading does not drag the gyro",
        m.robot.heading.mode == Mode::VISION_ONLY || fabsf(m.robot.heading.heading()) < 3);
  o = m.run(60);
  CHECK("no turning while the two estimates disagree", m.robot.heading.disagreement && !o.turn);

  m.cam.heading = kUnknown; m.cam.headingConfidence = 0;
  o = m.run(40);
  CHECK("a disagreement is not forgotten when the camera estimate drops out",
        m.robot.heading.disagreement && !o.turn);

  World v;
  v.cam.heading = 0; v.cam.headingConfidence = 90; v.run(20);
  v.cam.heading = 150; v.run(1);
  CHECK("one jumped vision frame is not adopted",
        !v.robot.heading.absolute || fabsf(v.robot.heading.heading()) < 5);
}

static void tactics() {
  World u;
  u.ir.angle = 180;
  Output o = u.run(10);
  CHECK("unknown attack direction without BNO follows IR", o.power && o.direction == 180 && !o.turn);

  World orbit;
  orbit.goal(0); orbit.ir.angle = 180;
  orbit.cam.mask = uint16_t(1u << 8);  // only straight at the ball (toward our own goal) is open
  o = orbit.run(5);
  CHECK("blocked orbit never drives straight into the ball", !o.power);

  World own;
  own.goal(0); own.own(0); own.ir.angle = 0;  // contradictory goals: our goal is ahead too
  o = own.run(10);
  CHECK("never push toward a visible own goal", !o.power || fabsf(wrap(o.direction - 0.0f)) >= 45);

  World line;
  line.goal(0); line.ir.angle = 30; line.cam.heading = 40; line.cam.headingConfidence = 90;
  line.cam.mask = 0x00FF;
  o = line.run(20);
  CHECK("near lines: exact motor directions, strict flag, no turn",
        !o.power || (o.direction % 45 == 0 && o.avoid && !o.turn));

  World esc;
  esc.cam.escaping = true; esc.cam.escape = 113; esc.cam.mask = uint16_t(1u << 5);
  CHECK("an escape the wheels cannot follow exactly stops", !esc.run().power);

  World stale;
  stale.goal(0); stale.run(3);
  stale.camOn = false;
  stale.cam.yellow.seen = false;
  stale.run(1);
  stale.cam.receivedMs = stale.now;
  stale.now += 1100;
  stale.cam.receivedMs = stale.now;
  stale.ir.stampMs = stale.now;
  o = stale.robot.step(stale.cam, stale.ir, stale.bno, stale.now);
  CHECK("goal memory is short", !isfinite(o.goal));
}

static void parser() {
  uint8_t f[kCameraSize] = {0xC5, 0x5C, kCameraVersion, kCameraSize, 9, 1};
  f[6] = f[7] = 0xFF;
  for (int at : {8, 10, 14, 20, 26}) f[at] = f[at + 1] = 0xFF;
  f[19] = f[25] = 255;
  const uint16_t crc = crc16(f, kCameraSize - 2);
  f[30] = uint8_t(crc); f[31] = uint8_t(crc >> 8);
  CameraParser p;
  CameraData d;
  bool first = false, again = false;
  for (uint8_t b : f) first = p.push(b, 100, d) || first;
  for (uint8_t b : f) again = p.push(b, 900, d) || again;
  CHECK("valid frame accepted", first);
  CHECK("a replayed sequence never refreshes the link", !again);
}

int main() {
  motor();
  heading();
  tactics();
  parser();
  printf("review checks: %d failed\n", failed);
  return failed ? 1 : 0;
}
