#include "../libraries/RobotDiagnostics/src/RuntimeConfig.h"
#include "world.h"
#include <cassert>

int main() {
  static_assert(ENABLE_BNO055 == 0, "BNO055 must stay disabled");
  static_assert(ENABLE_LINE_BOARD == 0, "LINE board must stay disabled");
  static_assert(ENABLE_OBSTACLE_AVOID == 0, "unvalidated obstacle input must stay disabled");
  static_assert(ENABLE_BALL_SEARCH == 0, "search is hardware-pending");
  static_assert(ENABLE_ORBIT == 0, "orbit is hardware-pending");
  static_assert(ENABLE_ATTACK == 0, "attack is hardware-pending");
  assert(robotcfg::kBuildFeatures.valid());
  World world;
  world.goal(0);
  world.ir.angle = 0;
  brain::Output output = world.run(5);
  assert(output.tactic != brain::Tactic::ATTACK);
  world.ir.angle = 90;
  output = world.run(2);
  assert(output.tactic != brain::Tactic::ORBIT && output.tactic != brain::Tactic::ATTACK);
  world.ball = false;
  output = world.run(2);
  assert(output.tactic == brain::Tactic::SAFE_STOP && !output.power);
}
