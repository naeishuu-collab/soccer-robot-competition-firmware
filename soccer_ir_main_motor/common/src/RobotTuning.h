#pragma once

#include <stdint.h>

namespace soccer {

constexpr uint32_t kRs485Baud = 115200;

// MAIN: 制御・通信周期と姿勢補正。
constexpr uint32_t kControlPeriodUs = 10000;
constexpr uint32_t kIrPollPeriodMs = 20;
constexpr uint32_t kIrReplyTimeoutUs = 15000;
constexpr uint32_t kLineReplyTimeoutUs = 8000;
constexpr uint32_t kIrFreshTimeoutMs = 60;
// 実球の近接信号を測定するまでキックしない。
constexpr uint16_t kIrKickCloseSignal = 0;
constexpr uint32_t kLineFreshTimeoutMs = 100;
constexpr uint32_t kHeadingPollPeriodMs = 20;
constexpr uint32_t kHeadingFreshTimeoutMs = 100;
constexpr uint32_t kDiagnosticPeriodMs = 250;
constexpr uint32_t kMotorCommandPeriodMs = 40;
constexpr uint32_t kMotorConsoleTimeoutMs = 300;
// 旋回符号が実測未確認の間は補正を無効にする。実測後のみ -1 または +1。
constexpr int8_t kBnoHeadingTurnSign = 0;
constexpr float kIrTestDriveMagnitude = 0.50f;
constexpr float kHeadingTurnGain = 0.012f;
constexpr float kHeadingTurnLimit = 0.35f;
static_assert(kBnoHeadingTurnSign >= -1 && kBnoHeadingTurnSign <= 1,
              "BNO turn sign must be -1, 0 (unverified), or +1");

// IR: サンプリング窓、検出しきい値、予測。
constexpr uint32_t SAMPLE_TIME_US = 2000;
constexpr uint32_t CALIBRATION_BATCH_US = 50000;
constexpr uint8_t SENSOR_HISTORY_SIZE = 16;
constexpr uint8_t RELATIVE_SIGNAL_HISTORY_SIZE = 16;
constexpr uint8_t PRED_HISTORY_SIZE = 6;
static_assert(SENSOR_HISTORY_SIZE > 0 && RELATIVE_SIGNAL_HISTORY_SIZE > 0 && PRED_HISTORY_SIZE > 0,
              "IR history sizes must be nonzero");

constexpr uint32_t MIN_TOTAL_DETECT = 3;

constexpr float PREDICT_TIME_SEC = 0.030f;
constexpr float MAX_ANGULAR_SPEED = 720.0f;

// MOTOR: 指令監視、加減速、走行モード別PWM。
constexpr uint32_t kCommandTimeoutMs = 100;
// MAINが停止/故障したときの暴走防止。false にすれば旧挙動（物理スイッチのみ）に戻せる。
constexpr bool kSoftwareWatchdogEnabled = true;
constexpr uint32_t kMotorPeriodUs = 5000;
constexpr uint8_t kRampStep = 5;
constexpr uint32_t kRampIntervalMs = 10;
// 暫定: これらはPWM上限の初期値であり、実測車輪速度ではない。低速ログ採取後に引き上げる。
constexpr uint8_t kAutoWheelLiftPwmCap = 48;
constexpr uint8_t kAutoFloorPwmCap = 24;

constexpr uint8_t kIrTestPwmCap = 210;
constexpr uint8_t kIrTestPwmFloor = 120;
static_assert(kIrTestPwmFloor <= kIrTestPwmCap, "IR test PWM floor must not exceed cap");

constexpr uint32_t kTb67h450PwmFrequencyHz = 20000;
constexpr uint32_t kTb67h450ReversalDwellMs = 200;

} // namespace soccer
