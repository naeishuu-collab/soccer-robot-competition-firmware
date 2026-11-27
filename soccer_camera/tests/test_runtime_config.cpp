#include "../libraries/RobotDiagnostics/src/RuntimeConfig.h"
#include <cassert>
#include <cstring>

int main() {
  using namespace robotcfg;
  assert(kBuildFeatures.valid());
  assert(!ENABLE_BNO055 && !ENABLE_LINE_BOARD);
  FeatureSet noCamera = kBuildFeatures;
  noCamera.camera = false;
  assert(!noCamera.valid());
  FeatureSet noLineGuard = kBuildFeatures;
  noLineGuard.lineEscape = false;
  assert(!noLineGuard.valid());
  FeatureSet searchOnly = kBuildFeatures;
  searchOnly.ballSearch = true;
  assert(searchOnly.valid());
  FeatureSet obstacle = kBuildFeatures;
  obstacle.obstacleAvoid = true;
  assert(!obstacle.valid());

  TestMode mode = TestMode::STOP;
  assert(parseMode("CHASE_ONLY", mode) && mode == TestMode::CHASE_ONLY);
  assert(parseMode("COMPETITION", mode) && mode == TestMode::COMPETITION);
  assert(!parseMode("UNKNOWN", mode));
  assert(std::strcmp(name(TestMode::ORBIT_TEST), "ORBIT_TEST") == 0);

  ModeGate gate;
  assert(!gate.mayMove(true, true));
  gate.select(TestMode::CHASE_ONLY);
  assert(!gate.resume()); // mode changes require fresh sensor state
  gate.acknowledgeFreshSensors();
  assert(gate.resume());
  assert(gate.mayMove(true, true));
  assert(!gate.mayMove(false, true));
  assert(!gate.mayMove(true, false));
  const uint16_t generation = gate.generation();
  gate.select(TestMode::ATTACK_TEST);
  assert(gate.generation() == uint16_t(generation + 1));
  assert(!gate.armed() && !gate.mayMove(true, true));
  gate.acknowledgeFreshSensors();
  assert(gate.resume());
  gate.stop();
  assert(gate.mode() == TestMode::STOP && !gate.armed());
  assert(!gate.resume());
}
