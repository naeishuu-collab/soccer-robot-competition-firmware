// Simulated sensor inputs for the MAIN upgrade logic.
#pragma once
#include "main_logic.h"

struct World {
  brain::CameraData cam;
  brain::IrData ir;
  brain::BnoSample bno;
  brain::Brain robot;
  uint32_t now = 1000;
  float yaw = 0;
  bool bnoOn = false, camOn = true, irOn = true, ball = true;
  World() {
    cam.received = cam.ready = cam.targetValid = true;
    cam.mask = 0xFFFF;
    ir.received = true;
    ir.confidence = 90;
  }
  void goal(float bearing) { cam.yellow.seen = true; cam.yellow.bearing = bearing; cam.yellow.confidence = 90; }
  void own(float bearing) { cam.blue.seen = true; cam.blue.bearing = bearing; cam.blue.confidence = 90; }
  brain::Output run(int steps = 1) {
    brain::Output out;
    for (int i = 0; i < steps; ++i) {
      now += 20;
      if (camOn) { cam.receivedMs = now; ++cam.sequence; cam.yellow.stampMs = cam.blue.stampMs = now; }
      if (irOn) { ir.stampMs = now; ir.strength = ball ? 500 : 0; }
      if (bnoOn) { bno.ok = true; bno.yaw = yaw; bno.stampMs = now; }
      out = robot.step(cam, ir, bno, now);
    }
    return out;
  }
};
