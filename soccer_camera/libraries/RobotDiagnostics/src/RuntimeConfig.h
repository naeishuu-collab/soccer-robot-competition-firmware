#pragma once
#include <stdint.h>

// Compile-time capability switches. Experimental tactics stay disabled until bench/hardware validation.
#ifndef ENABLE_BNO055
#define ENABLE_BNO055 0
#endif
#ifndef ENABLE_LINE_BOARD
#define ENABLE_LINE_BOARD 0
#endif
#ifndef ENABLE_IR
#define ENABLE_IR 1
#endif
#ifndef ENABLE_MOTOR
#define ENABLE_MOTOR 1
#endif
#ifndef ENABLE_PD_CHASE
#define ENABLE_PD_CHASE 1
#endif
#ifndef ENABLE_CAMERA
#define ENABLE_CAMERA 1
#endif
#ifndef ENABLE_LINE_ESCAPE
#define ENABLE_LINE_ESCAPE 1
#endif
#ifndef ENABLE_OBSTACLE_AVOID
#define ENABLE_OBSTACLE_AVOID 0
#endif
#ifndef ENABLE_BALL_SEARCH
#define ENABLE_BALL_SEARCH 0
#endif
#ifndef ENABLE_GOAL_DETECTION
#define ENABLE_GOAL_DETECTION 1
#endif
#ifndef ENABLE_ORBIT
#define ENABLE_ORBIT 0
#endif
#ifndef ENABLE_ATTACK
#define ENABLE_ATTACK 0
#endif
#ifndef ENABLE_DIAGNOSTICS
#define ENABLE_DIAGNOSTICS 1
#endif

#define ROBOT_CONFIG_ID "RCJ-IR-SW-2026-11-27-A"

namespace robotcfg {

struct FeatureSet {
  bool ir, motor, pd, camera, lineEscape, obstacleAvoid, ballSearch, goal, orbit, attack, diagnostics;

  constexpr bool valid() const {
    if (!ir || !motor || !pd) return false;
    if (lineEscape && !camera) return false;
    if (goal && !camera) return false;
    if ((orbit || attack) && (!camera || !goal || !pd)) return false;
    // This firmware has no independently validated obstacle sensor input.
    if (obstacleAvoid) return false;
    return true;
  }
};

constexpr FeatureSet kBuildFeatures = {
  ENABLE_IR != 0, ENABLE_MOTOR != 0, ENABLE_PD_CHASE != 0, ENABLE_CAMERA != 0,
  ENABLE_LINE_ESCAPE != 0, ENABLE_OBSTACLE_AVOID != 0, ENABLE_BALL_SEARCH != 0,
  ENABLE_GOAL_DETECTION != 0, ENABLE_ORBIT != 0, ENABLE_ATTACK != 0, ENABLE_DIAGNOSTICS != 0
};

enum class TestMode : uint8_t {
  STOP, MOTOR_TEST, COMM_TEST, IR_TEST, CAMERA_TEST, CHASE_ONLY, LINE_TEST,
  SEARCH_TEST, GOAL_TEST, ORBIT_TEST, ATTACK_TEST, COMPETITION
};

inline const char *name(TestMode mode) {
  switch (mode) {
    case TestMode::STOP: return "STOP";
    case TestMode::MOTOR_TEST: return "MOTOR_TEST";
    case TestMode::COMM_TEST: return "COMM_TEST";
    case TestMode::IR_TEST: return "IR_TEST";
    case TestMode::CAMERA_TEST: return "CAMERA_TEST";
    case TestMode::CHASE_ONLY: return "CHASE_ONLY";
    case TestMode::LINE_TEST: return "LINE_TEST";
    case TestMode::SEARCH_TEST: return "SEARCH_TEST";
    case TestMode::GOAL_TEST: return "GOAL_TEST";
    case TestMode::ORBIT_TEST: return "ORBIT_TEST";
    case TestMode::ATTACK_TEST: return "ATTACK_TEST";
    case TestMode::COMPETITION: return "COMPETITION";
  }
  return "INVALID";
}

inline bool parseMode(const char *text, TestMode &mode) {
  static const TestMode modes[] = {
    TestMode::STOP, TestMode::MOTOR_TEST, TestMode::COMM_TEST, TestMode::IR_TEST,
    TestMode::CAMERA_TEST, TestMode::CHASE_ONLY, TestMode::LINE_TEST, TestMode::SEARCH_TEST,
    TestMode::GOAL_TEST, TestMode::ORBIT_TEST, TestMode::ATTACK_TEST, TestMode::COMPETITION
  };
  for (uint8_t i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i) {
    const char *a = text;
    const char *b = name(modes[i]);
    while (*a && *b && *a == *b) { ++a; ++b; }
    if (!*a && !*b) { mode = modes[i]; return true; }
  }
  return false;
}

// A mode change always drops motion authorization. Caller must stop motors, clear the old
// command and controller memory, then call acknowledgeFreshSensors() before explicit resume.
class ModeGate {
 public:
  explicit ModeGate(TestMode initial = TestMode::STOP)
      : mode_(initial), armed_(false), fresh_(false), generation_(0) {}

  void select(TestMode mode) {
    mode_ = mode;
    armed_ = false;
    fresh_ = false;
    ++generation_;
  }
  void acknowledgeFreshSensors() { fresh_ = true; }
  bool resume() {
    if (!fresh_ || mode_ == TestMode::STOP) return false;
    armed_ = true;
    return true;
  }
  void stop() { mode_ = TestMode::STOP; armed_ = fresh_ = false; ++generation_; }
  bool mayMove(bool configurationValid, bool interlocksValid) const {
    return armed_ && fresh_ && mode_ != TestMode::STOP && configurationValid && interlocksValid;
  }
  TestMode mode() const { return mode_; }
  bool armed() const { return armed_; }
  uint16_t generation() const { return generation_; }

 private:
  TestMode mode_;
  bool armed_, fresh_;
  uint16_t generation_;
};

}  // namespace robotcfg

static_assert(robotcfg::kBuildFeatures.valid(), "Invalid feature configuration; review dependencies");
static_assert(!ENABLE_BNO055 && !ENABLE_LINE_BOARD, "BNO055 and LINE board are not part of the competition build");
