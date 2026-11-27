// Upgrade path: protocols, IR filter, heading modes, ball tracking, tactics.
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "ir_logic.h"
#include "main_logic.h"
#include "world.h"
#include <assert.h>
using namespace brain;

static void loadGolden(uint8_t *packet) {
  char path[512];
  strncpy(path, __FILE__, sizeof(path) - 40);
  path[sizeof(path) - 40] = 0;
  char *end = path + strlen(path);
  while (end > path && end[-1] != '/' && end[-1] != '\\') --end;
  strcpy(end, "camera_v2_golden.txt");
  FILE *f = fopen(path, "r");
  assert(f);
  for (int i = 0; i < kCameraSize; ++i) {
    unsigned value = 0;
    const int read = fscanf(f, "%x", &value);
    assert(read == 1);
    packet[i] = uint8_t(value);
  }
  fclose(f);
}

static bool feed(CameraParser &p, const uint8_t *data, int size, uint32_t now, CameraData &out) {
  bool got = false;
  for (int i = 0; i < size; ++i) got = p.push(data[i], now, out) || got;
  return got;
}

static void protocols() {
  uint8_t golden[kCameraSize];
  loadGolden(golden);  // produced by the Python encoder; test_competition_vision.py checks the same file
  CameraParser p;
  CameraData d;
  assert(feed(p, golden, kCameraSize, 100, d));
  assert(d.ready && !d.escaping && d.targetBlue && d.targetValid && d.mask == 0xFF0F && d.escape == kUnknown);
  assert(d.line == 350 && d.lineCm == 31 && d.lineCount == 1);
  assert(d.yellow.distanceCm == 120 && d.blue.bearing == 180 && d.blue.distanceCm == -1 && d.blue.stampMs == 70);
  assert(d.heading == 355 && d.headingConfidence == 55 && d.frameMs == 33);
  assert(!feed(p, golden, kCameraSize, 150, d));  // repeated sequence is stale
  // Every single-byte corruption and truncation is rejected; garbage prefixes resynchronize.
  for (int i = 0; i < kCameraSize; ++i) {
    uint8_t bad[kCameraSize];
    memcpy(bad, golden, sizeof(bad));
    bad[i] ^= 0x01;
    CameraParser q;
    CameraData e;
    assert(!feed(q, bad, kCameraSize, 100, e) && !e.received);
    assert(feed(q, golden, kCameraSize, 200, e));
  }
  CameraParser q;
  CameraData e;
  const uint8_t junk[] = {0x20, 0, 0, 0xFF, 0, 2, 0x40, 0x9D, 0xC5, 0xC5, 0x5C, 9};
  assert(!feed(q, junk, sizeof(junk), 100, e));
  assert(feed(q, golden, kCameraSize, 101, e));
  CameraParser r;
  CameraData f;
  feed(r, golden, 20, 100, f);
  assert(feed(r, golden, kCameraSize, 200, f));  // partial frame times out

  // A version 2 camera (which still carried ball fields) is rejected as a whole.
  uint8_t old[37] = {0xC5, 0x5C, 2, 37};
  CameraParser o;
  CameraData g;
  assert(!feed(o, old, sizeof(old), 100, g) && !g.received);

  irx::Result sent;
  sent.angle = 270; sent.strength = 0xAA55; sent.confidence = 88; sent.faults = 0x801;
  uint8_t wire[irx::kReplySize];
  irx::encode(sent, wire);
  IrReceiver receiver;
  IrData got;
  bool ok = false;
  for (uint8_t byte : wire) ok = receiver.push(byte, got);
  assert(ok && got.angle == 270 && got.strength == 0xAA55 && got.confidence == 88 && got.faults == 0x801);
  wire[2] ^= 1;
  receiver.reset();
  for (uint8_t byte : wire) assert(!receiver.push(byte, got));
  const uint8_t legacy[] = {0xAA, 0, 90, 0, 5, 0x55};
  receiver.reset();
  for (uint8_t byte : legacy) assert(!receiver.push(byte, got));
}

static void irFilterTests() {
  uint16_t count[12] = {};
  irx::Filter reacquire;
  count[0] = 500; reacquire.update(count, 1000);
  count[0] = 0; assert(reacquire.update(count, 1000).strength == 0);
  count[1] = 500;
  assert(reacquire.update(count, 1000).angle == 30);
  count[1] = 0;
  irx::Filter f;
  count[3] = 400;
  irx::Result r = f.update(count, 1000);
  assert(r.angle == 90 && r.confidence == 100 && r.strength == 400);
  // Two nearly equal sources on opposite sides: ambiguous, so low confidence.
  count[9] = 380;
  r = f.update(count, 1000);
  assert(r.confidence < 10);
  // A weak reflection on the far side is ignored; sample-count normalization.
  irx::Filter g;
  memset(count, 0, sizeof(count));
  count[0] = 800; count[6] = 60;
  r = g.update(count, 2000);
  assert(r.angle == 0 && r.confidence == 100 && r.strength == 400);
  // Wraparound smoothing and fast following.
  irx::Filter h;
  memset(count, 0, sizeof(count));
  count[11] = 500; h.update(count, 1000);
  count[11] = 0; count[1] = 500;
  r = h.update(count, 1000);
  assert(r.angle > 330 || r.angle <= 30);
  count[1] = 0; count[6] = 500;
  assert(h.update(count, 1000).angle == 180);
  // Stuck sensor: excluded after persisting, restored after it behaves.
  irx::Filter s;
  memset(count, 0, sizeof(count));
  count[5] = 1000; count[0] = 400;
  for (int i = 0; i < 100; ++i) r = s.update(count, 1000);
  assert((r.faults & (1 << 5)) && r.angle == 0);
  count[5] = 300;
  for (int i = 0; i < 10; ++i) r = s.update(count, 1000);
  assert(!r.faults);
  // Dead sensor between two active neighbours.
  irx::Filter dead;
  memset(count, 0, sizeof(count));
  count[1] = 600; count[3] = 600;
  r = dead.update(count, 1000);  // usable from the first poll, before the sensor is declared dead
  assert(r.angle == 60 && r.confidence >= 50 && !r.faults);
  for (int i = 0; i < 100; ++i) r = dead.update(count, 1000);
  assert(r.faults == (1 << 2) && r.angle == 60);
  memset(count, 0, sizeof(count));
  r = dead.update(count, 1000);
  assert(r.strength == 0 && r.confidence == 0);

  // Interpolation between two neighbouring sensors, including across the 330/0 seam.
  irx::Filter mid;
  count[11] = 400; count[0] = 400;
  r = mid.update(count, 1000);
  assert(r.angle == 345 && r.confidence >= 90);
  // A strong reflection (40% of the ball) in a separate run does not bend the bearing.
  irx::Filter refl;
  memset(count, 0, sizeof(count));
  count[2] = 500; count[3] = 500; count[8] = 400;
  r = refl.update(count, 1000);
  assert(r.angle == 75 && r.confidence >= 50);
  // With two similar runs the previous bearing wins.
  irx::Filter keep;
  memset(count, 0, sizeof(count));
  count[0] = 500;
  keep.update(count, 1000);
  count[6] = 560;
  r = keep.update(count, 1000);
  assert(r.angle == 0 && r.confidence < 25);
  // Every sensor active in turn gives its own angle with no lag on a large move.
  for (int i = 0; i < 12; ++i) {
    memset(count, 0, sizeof(count));
    count[i] = 500;
    irx::Filter one;
    assert(one.update(count, 1000).angle == i * 30);
  }
  // Flooded receivers (ambient IR) are a sensor failure, not a ball.
  irx::Filter flood;
  for (int i = 0; i < 12; ++i) count[i] = i < 9 ? 1000 : 0;
  r = flood.update(count, 1000);
  assert(r.strength == 0 && r.confidence == 0 && r.faults == 0xFFF);
  // All lit weakly (no gap at all) has no direction.
  irx::Filter all;
  for (int i = 0; i < 12; ++i) count[i] = 200;
  r = all.update(count, 1000);
  assert(r.confidence < 5);
}

static void headingModes() {
  World w;
  w.ir.angle = 0;
  const Output circle = w.run(10);  // no heading or goal: follow the IR ball
  assert(circle.power && circle.direction == 0 && circle.turn == 0);
  w.robot.cfg.blindTracking = true;
  Output o = w.run(10);
  assert(w.robot.heading.mode == Mode::DEGRADED && o.turn == 0 && o.power > 0);  // restricted IR tracking
  assert(o.power == w.robot.cfg.speedSlow);
  // Vision only, goal-relative: not a heading.
  w.goal(20);
  o = w.run(10);
  assert(w.robot.heading.mode == Mode::VISION_ONLY && !w.robot.heading.absolute);
  // Absolute vision heading across the 0/360 seam.
  w.cam.heading = 340; w.cam.headingConfidence = 80;
  o = w.run(30);
  assert(w.robot.heading.absolute && fabsf(w.robot.heading.heading() + 20) < 0.1f && o.turn > 0);
  // BNO recovers with an unrelated zero: no jump, then FUSED.
  w.bnoOn = true; w.yaw = 200;
  o = w.run(30);
  assert(w.robot.heading.mode == Mode::FUSED && fabsf(w.robot.heading.heading() + 20) < 1);
  // Persistent disagreement with confident vision: fall back to vision.
  w.cam.heading = 90; w.cam.headingConfidence = 90;
  w.run(60);
  assert(w.robot.heading.mode == Mode::VISION_ONLY && w.robot.heading.disagreement);
  w.cam.headingConfidence = 55;
  w.run(2);
  assert(w.robot.heading.mode == Mode::BNO_ONLY);
  // Vision lost: BNO only.
  w.cam.heading = kUnknown; w.cam.headingConfidence = 0;
  w.run(10);
  assert(w.robot.heading.mode == Mode::BNO_ONLY && w.robot.heading.absolute);
  // BNO disconnect while running: degraded, no invented heading, no turn.
  w.bnoOn = false;
  w.cam.yellow.seen = false;
  o = w.run(20);
  assert(w.robot.heading.mode == Mode::DEGRADED && !w.robot.heading.absolute && o.turn == 0);

  // Power-on with BNO only, across the seam: first sample is the reference; turn slews.
  World b;
  b.robot.cfg.bnoPowerOnReference = true;
  b.bnoOn = true; b.yaw = 359;
  b.run(20);
  assert(b.robot.heading.mode == Mode::BNO_ONLY && fabsf(b.robot.heading.heading()) < 0.1f);
  b.yaw = 14; o = b.run();
  assert(abs(o.turn) <= b.robot.cfg.turnSlew);
  o = b.run(20);
  assert(o.turn < 0 && o.turn >= -b.robot.cfg.turnMax);
  b.yaw = 4; o = b.run(20);
  assert(o.turn < 0);  // hysteresis keeps correcting between exit and enter
  b.yaw = 359; o = b.run(20);
  assert(o.turn == 0);
  // A single wild sample is ignored.
  b.yaw = 180; b.run();
  assert(fabsf(b.robot.heading.heading()) < 1);
  b.yaw = 359; b.run(15);  // re-acquired against the recent estimate
  assert(fabsf(b.robot.heading.heading()) < 1);
  b.yaw = 14; o = b.run(20);
  assert(o.turn < 0);
  b.bnoOn = false; b.bno.ok = false;
  o = b.run();
  assert(o.power && !o.turn && b.robot.heading.mode == Mode::DEGRADED);
  b.bnoOn = true;
  o = b.run(20);
  assert(o.power && b.robot.heading.mode == Mode::BNO_ONLY && b.robot.heading.absolute);
  b.irOn = false;
  o = b.run(30);
  assert(o.tactic == Tactic::SAFE_STOP && !o.power && !o.turn && strcmp(o.reason, "IR link lost") == 0);
}

static void safety() {
  World w;
  w.goal(0);
  w.camOn = false;
  Output o = w.run();
  w.now += 300;
  o = w.robot.step(w.cam, w.ir, w.bno, w.now);
  assert(o.tactic == Tactic::SAFE_STOP && !o.power && !o.turn && strcmp(o.reason, "camera link") == 0);
  w.camOn = true; w.irOn = false;
  o = w.run(10);
  assert(o.tactic == Tactic::SAFE_STOP && !o.power);
  w.irOn = true; w.cam.ready = false; w.cam.mask = 0;
  assert(w.run().power == 0);
  w.cam.ready = true;
  assert(w.run().tactic == Tactic::SAFE_STOP);  // nothing open
  w.cam.mask = 1u << 8; w.cam.escaping = true; w.cam.escape = 180;
  o = w.run();
  assert(o.tactic == Tactic::AVOID_LINE && o.direction == 180 && o.power == w.robot.cfg.speedAvoid && o.avoid && !o.turn);
  w.cam.escape = 0;  // listed escape is closed: use the open exact direction instead
  o = w.run();
  assert(o.power && o.direction == 180 && o.avoid);
  w.cam.mask = 1u << 5;  // only an in-between heading open: nothing the wheels follow exactly
  assert(!w.run().power);
  // Line mask beats the ball.
  w.cam.escaping = false; w.cam.escape = kUnknown;
  w.cam.mask = uint16_t(~((1u << 15) | 1u | (1u << 1) | (1u << 2) | (1u << 14)));
  w.ir.angle = 0;
  o = w.run(15);
  assert(o.power == w.robot.cfg.speedSlow && sectorOpen(w.cam.mask, sectorOf(o.direction)) && o.direction != 0);
  assert(o.direction % 45 == 0 && o.avoid && !o.turn);
  w.cam.mask = 1u << 8;  // only straight back, ball ahead: back off rather than stop
  o = w.run();
  assert(o.power && o.direction == 180);
  for (int mask = 0; mask < 65536; mask += 7) {
    for (int wanted = 0; wanted < 360; wanted += 13) {
      const int d = safeDirection(uint16_t(mask), float(wanted));
      assert(d < 0 || sectorOpen(uint16_t(mask), sectorOf(float(d))));
    }
  }
}

static void ballStates() {
  World w;
  w.ball = false;
  Output o = w.run(5);
  assert(w.robot.ball.state == BallState::WAIT && !o.power);  // never seen: wait
  w.ball = true; w.ir.angle = 30; w.run(5);
  assert(w.robot.ball.state == BallState::TRACK);
  w.ir.angle = 200; w.run();
  assert(fabsf(wrap(w.robot.ball.bearing - 30)) < 5);  // one reflection ignored
  w.run(2);
  assert(fabsf(wrap(w.robot.ball.bearing - 200)) < 5);
  w.ball = false;
  w.run(7);
  assert(w.robot.ball.state == BallState::WAIT);
  w.run(15);
  o = w.run(300);
  assert(w.robot.ball.state == BallState::WAIT && !o.power);
  w.ball = true; w.ir.angle = 90; w.run(8);
  assert(w.robot.ball.state == BallState::TRACK && fabsf(wrap(w.robot.ball.bearing - 90)) < 5);  // reacquired
  // Moving ball: positive rate, lead ahead, wraparound.
  World m;
  for (int i = 0; i < 30; ++i) { m.ir.angle = uint16_t((340 + i * 2) % 360); m.run(); }
  assert(m.robot.ball.rate > 40 && wrap(m.robot.ball.lead() - m.robot.ball.bearing) > 0);
  // Low confidence IR is not used.
  World c;
  c.ir.confidence = 10;
  c.run(3);
  assert(c.robot.ball.state == BallState::WAIT);
  // Failed IR sensors: do not track, and say so. A few excluded sensors are tolerated.
  World broken;
  broken.goal(0); broken.run(5);
  broken.ir.faults = 0x00F;
  Output failedOut = broken.run(3);
  assert(!failedOut.power && strcmp(failedOut.reason, "IR sensors failed") == 0);
  broken.ir.faults = 0x007;
  assert(broken.run(10).power);
  broken.irOn = false;
  failedOut = broken.run(10);
  assert(!failedOut.power && strcmp(failedOut.reason, "IR link lost") == 0);
  broken.irOn = true; broken.ball = false;
  failedOut = broken.run(60);
  assert(!failedOut.power && strcmp(failedOut.reason, "no ball detected") == 0);
  // A single jumped goal frame does not swing the robot: heading control slews.
  World jump;
  jump.goal(0); jump.run(20);
  jump.goal(150);
  const Output once = jump.run(1);
  assert(abs(once.turn) <= jump.robot.cfg.turnSlew);
}

static void tactics() {
  World w;
  w.goal(0);
  w.ir.angle = 0;
  Output o = w.run(5);
  assert(o.tactic == Tactic::ATTACK && o.direction == 0 && o.power == w.robot.cfg.speedAttack);
  o = w.run(160);
  assert(w.robot.ball.state == BallState::TRACK);
  World stall;
  stall.goal(0);
  bool recovered = false;
  for (int i = 0; i < 200; ++i) {
    o = stall.run();
    if (o.tactic == Tactic::RECOVER) { recovered = true; assert(o.direction == 180); }
  }
  assert(recovered);
  // Ball to the right of the goal line: go further right to get behind it.
  World r;
  r.goal(0); r.ir.angle = 90;
  o = r.run(5);
  assert(o.tactic == Tactic::ORBIT && o.direction == 180);
  r.ir.angle = 270; o = r.run(5);
  assert(o.tactic == Tactic::ORBIT && o.direction == 180);
  World a;
  a.goal(0); a.ir.angle = 30;
  o = a.run(5);
  assert(o.tactic == Tactic::ALIGN && o.direction > 30 && o.direction <= 90);
  // Ball straight behind: one side is chosen and kept; a line flips it.
  World b;
  b.goal(0); b.ir.angle = 180;
  const uint16_t first = b.run(5).direction;
  assert(first == 270 || first == 90);
  assert(b.run(20).direction == first);
  b.cam.mask = first == 270 ? 0x01FF : 0xFF01;  // close the chosen side
  o = b.run(5);
  assert(o.power && o.direction != first);
  // Only our own goal visible: attack away from it.
  World own;
  own.cam.blue.seen = true; own.cam.blue.bearing = 0; own.cam.blue.confidence = 90;
  own.ir.angle = 0;
  own.cam.blue.distanceCm = 100;
  o = own.run(10);
  assert(o.tactic == Tactic::ORBIT && fabsf(o.goal - 180) < 1 && (o.direction == 90 || o.direction == 270));
  // Ball lost: patrol, but not into our own goal.
  own.ball = false;
  own.cam.blue.distanceCm = -1;
  own.run(30);
  for (int i = 0; i < 200; ++i) {
    o = own.run();
    assert(o.tactic == Tactic::SAFE_STOP && !o.power);
  }

  // Telemetry: golden bytes shared with the camera, and a port that accepts one byte at a time.
  Telemetry tm;
  tm.sequence = 5; tm.upgrade = tm.irLink = tm.cameraLink = true;
  tm.ball = 271; tm.ballConfidence = 88; tm.ballAgeMs = 25; tm.direction = 45; tm.power = 160; tm.turn = -9;
  tm.tactic = uint8_t(Tactic::ATTACK); tm.mode = uint8_t(Mode::VISION_ONLY); tm.bno = 0;
  struct Slow {
    std::vector<uint8_t> out; int room = 0;
    int availableForWrite() { return room; }
    void write(uint8_t b) { out.push_back(b); --room; }
  } slow;
  TelemetrySender sender;
  sender.load(tm);
  sender.pump(slow);
  assert(slow.out.empty() && sender.busy());  // full FIFO: nothing written, nothing blocks
  Telemetry other = tm; other.ball = 10;
  for (int i = 0; i < 30; ++i) { slow.room = 1; sender.pump(slow); sender.load(i < 5 ? other : tm); if (slow.out.size() >= 19) break; }
  char path[512];
  strncpy(path, __FILE__, sizeof(path) - 40);
  path[sizeof(path) - 40] = 0;
  char *end = path + strlen(path);
  while (end > path && end[-1] != '/' && end[-1] != '\\') --end;
  strcpy(end, "telemetry_golden.txt");
  FILE *g = fopen(path, "r");
  assert(g);
  for (int i = 0; i < kTelemetrySize; ++i) {
    unsigned value = 0;
    const int read = fscanf(g, "%x", &value);
    assert(read == 1 && slow.out[i] == value);
  }
  fclose(g);
  // millis wrap keeps working.
  World z;
  z.now = UINT32_MAX - 100;
  z.goal(0);
  o = z.run(20);
  assert(o.tactic == Tactic::ATTACK);
}

int main() {
  Config cfg;
  cfg.headingKp = 0; cfg.headingKd = 0.05f; cfg.turnMin = 0; cfg.turnMax = 250; cfg.turnSlew = 250;
  Turn turn;
  assert(wrap(-179.0f - 179.0f) == 2.0f);
  assert(turn.update(true, 20, cfg) == 0);
  assert(turn.update(true, 40, cfg) == 10);
  assert(turn.update(false, 0, cfg) == 0);
  assert(turn.update(true, -179, cfg) == 0);
  protocols();
  irFilterTests();
  headingModes();
  safety();
  ballStates();
  tactics();
  puts("upgrade tests passed");
}
