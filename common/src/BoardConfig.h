#pragma once

#include <stdint.h>
#include "RobotControl.h"

namespace soccer { namespace board {

// MAIN, IR, and LINE pin maps follow the current project configuration; verify physical continuity on the board.
// The MOTOR PWM map below is user-reported for the manufactured revision; its RS485 and safety routes remain unknown.
constexpr int8_t kMainRs485TxPin = 0;
constexpr int8_t kMainRs485RxPin = 1;
constexpr int8_t kMainRs485DePin = 8;

constexpr int8_t kIrRs485TxPin = 0;
constexpr int8_t kIrRs485RxPin = 1;
constexpr int8_t kIrRs485DePin = 2;

constexpr int8_t kLineRs485TxPin = 0;
constexpr int8_t kLineRs485RxPin = 1;
constexpr int8_t kLineRs485DePin = 27;
constexpr int8_t kLineSensorPins[24] = {
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
};
constexpr bool kLineSensorEnabled[24] = {
    true, false, true, true,
    true, false, true, true,
    false, false, true, false,
    true, true, true, true,
    true, true, true, true,
    true, false, true, true
};
constexpr uint32_t kLineSensorEnabledMask = makeLineSensorEnabledMask(kLineSensorEnabled);

constexpr int8_t kMotorRs485TxPin = -1;
constexpr int8_t kMotorRs485RxPin = -1;
constexpr int8_t kMotorRs485DePin = -1;
constexpr int8_t kMotorRs485SharedDePin = -1;
// The user-reported J2 left-front test pins are retained for test provenance.
constexpr int8_t kMotorTestIn1CandidatePin = 13;
constexpr int8_t kMotorTestIn2CandidatePin = 12;
constexpr uint8_t kMotorGpioCount = 0; // Package and complete route/reservation map are unverified; keeps validation fail-closed.
constexpr bool kMotorRs485PinsVerified = false;
constexpr uint64_t kMotorPwmPinMask = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) |
                                      (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13);
constexpr uint64_t kMotorReservedPinMask = 0;
constexpr uint64_t kMotorSafetyReservedPinMask = kMotorPwmPinMask;

constexpr float kMotorWheelRadiusMm = 30.0f;
constexpr float kMotorCenterToWheelMm = 94.762f;

// No MOTOR safety input pins or electrical bias have been confirmed on the manufactured board.
constexpr int8_t kMotorArmPin = -1;
constexpr int8_t kMotorEStopPin = -1;
constexpr int8_t kMotorClearFaultPin = -1;
constexpr int8_t kMotorArmActiveLevel = -1;
constexpr int8_t kMotorEStopActiveLevel = -1;
constexpr int8_t kMotorClearFaultActiveLevel = -1;
constexpr bool kMotorSafetyInputBiasVerified = false;

// MAIN also stays unable to arm until the physical switch wiring and safe input bias are measured.
constexpr int8_t kMainArmPin = -1;
constexpr int8_t kMainEStopPin = -1;
constexpr int8_t kMainClearFaultPin = -1;
constexpr int8_t kMainArmActiveLevel = -1;
constexpr int8_t kMainEStopActiveLevel = -1;
constexpr int8_t kMainClearFaultActiveLevel = -1;
constexpr bool kMainSafetyInputBiasVerified = false;
constexpr uint32_t kMainSafetyReservedPinMask = (1u << 0) | (1u << 1) | (1u << 4) |
                                                (1u << 5) | (1u << 8);

// BNO055 signals are on the MAIN processor in the ir&main schematic.
constexpr int8_t kBnoSdaPin = 4;
constexpr int8_t kBnoSclPin = 5;
constexpr uint32_t kBnoI2cClockHz = 50000;
constexpr uint8_t kBnoAddress = 0x28;
constexpr bool kBnoOffsetsVerified = false; // Legacy offsets remain inactive until a measured calibration is recorded.
constexpr bool kAttackHeadingConfigured = false;
constexpr float kAttackHeadingDeg = 0.0f; // Relative to the first settled heading; ignored until configured.

constexpr bool kLineDebounceConfigured = false; // Candidate filter stays off until sensor edge timing is measured.
constexpr uint8_t kLineDebounceSamples = 2;
constexpr bool kLinePolarityConfigured = false;
constexpr bool kLineDetectHigh = true; // Candidate only; ignored until polarity is measured.
// 添付soccer.zipのsoccer/line2/line2.kicad_pcbにあるQ1..Q24の座標から算出した基板基準角度[1/100度]。中心(100,100)、基板上方向0°、時計回りで1°単位に丸めた値。
// PCBリビジョン、Q番号と実配線chの対応、機体前方オフセット、入力極性は未確認。故障6ch(1,5,8,9,11,21)は kLineSensorEnabled により制御には使われない。
constexpr int32_t kLineBoardAngleCdeg[24] = {
    1100, 2100, 4100, 5100, 7100, 8100, 10100, 11100, 13100, 14100, 16100, 17100,
    19100, 20100, 22100, 23100, 25100, 26100, 28100, 29100, 31100, 32100, 34100, 35100
};
// 機体前方 = 基板角度 - オフセット。基板上方向と機体前方の差は未確定(以前の「0番が正面」申告との11度差を解消しない)。
// 実測で確定するまで kLineFrontOffsetConfigured=false とし、角度は無効=方向推定せず停止側に倒す。
constexpr bool kLineFrontOffsetConfigured = false;
constexpr int16_t kLineFrontOffsetCdeg = 0;
struct LineAngleTable { int16_t v[24]; };
constexpr LineAngleTable makeLineAngleTable(int16_t offsetCdeg) {
    LineAngleTable t{};
    for (uint8_t i = 0; i < 24; ++i) {
        int32_t a = static_cast<int32_t>(kLineBoardAngleCdeg[i]) - offsetCdeg;
        while (a >= 18000) a -= 36000;
        while (a < -18000) a += 36000;
        t.v[i] = static_cast<int16_t>(a);
    }
    return t;
}
constexpr LineAngleTable kLineAngleTable = makeLineAngleTable(kLineFrontOffsetCdeg);
constexpr const int16_t (&kLineSensorAngleCdeg)[24] = kLineAngleTable.v;
constexpr bool kLineAnglesConfigured = false; // 前方オフセット実測後に kLineFrontOffsetConfigured と同時に true にする。
constexpr uint32_t kLineSensorAngleValidMask = kLineSensorEnabledMask;
static_assert(!kLineAnglesConfigured || kLineFrontOffsetConfigured, "LINE angles need a measured front offset");
constexpr bool kMotorHardwareConfigured = false;
constexpr bool kCompetitionArmed = false;
constexpr uint32_t kMotorPwmFrequencyHz = kTb67h450PwmFrequencyHz;
constexpr bool kKickerHardwareConfigured = false;
constexpr bool kKickerRequestPolicyConfigured = false;
constexpr bool kKickerHeadingWindowConfigured = false;
constexpr float kKickerHeadingToleranceDeg = 0.0f;
constexpr bool kDribblerHardwareConfigured = false;

}} // namespace soccer::board
