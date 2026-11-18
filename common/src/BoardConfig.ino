#pragma once

#include <stdint.h>
#include "RobotControl.ino"

namespace soccer { namespace board {

// MAIN, IR, and LINE pin maps follow the current project configuration; verify physical continuity on the board.
// MOTOR routes below use the user's reference code; RS485 and safety wiring remain physically unverified.
constexpr int8_t kMainRs485TxPin = 0;
constexpr int8_t kMainRs485RxPin = 1;
constexpr int8_t kMainRs485DePin = 8;
constexpr int8_t kKickerPin = 11; // User-reported MAIN wiring; circuit review is still pending.

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

constexpr int8_t kMotorRs485TxPin = 0;
constexpr int8_t kMotorRs485RxPin = 1;
constexpr int8_t kMotorRs485DePin = 2;
constexpr int8_t kMotorRs485SharedDePin = -1;
// Matches the user-reported working MOTOR test code (RP2354B + TB67H450 x4).
constexpr int8_t kMotorIn1Pins[4] = {6, 12, 11, 9};
constexpr int8_t kMotorIn2Pins[4] = {7, 13, 10, 8};
// The user-reported J2 left-front pins are retained for test provenance.
constexpr int8_t kMotorTestIn1CandidatePin = 12;
constexpr int8_t kMotorTestIn2CandidatePin = 13;
constexpr uint8_t kMotorGpioCount = 30; // The final ZIP's MOTOR parent-board BOM specifies RP2354A.
constexpr bool kMotorRs485PinsConfigured = true; // From the reference code, not a hardware test.
constexpr uint64_t kMotorPwmPinMask = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) |
                                      (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13);
constexpr uint64_t kMotorReservedPinMask = (1u << 0) | (1u << 1) | (1u << 2);
constexpr uint64_t kMotorSafetyReservedPinMask = kMotorPwmPinMask | kMotorReservedPinMask;

constexpr float kMotorWheelRadiusMm = 30.0f;
constexpr float kMotorCenterToWheelMm = 94.762f;
// Candidate mapping used by the host integration model; polarity and roller directions need a four-wheel test.
constexpr WheelConfig kMotorWheelConfig[4] = {
    {1, -1, 45, 1}, {-1, 1, 225, 1}, {1, 1, 135, 1}, {-1, -1, 315, 1}
};
constexpr bool kMotorWheelConfigConfigured = false;

// No MOTOR safety input pins or electrical bias have been confirmed on the manufactured board.
constexpr int8_t kMotorArmPin = -1;
constexpr int8_t kMotorEStopPin = -1;
constexpr int8_t kMotorClearFaultPin = -1;
constexpr int8_t kMotorArmActiveLevel = -1;
constexpr int8_t kMotorEStopActiveLevel = -1;
constexpr int8_t kMotorClearFaultActiveLevel = -1;
constexpr bool kMotorSafetyInputBiasVerified = false;

// MAIN has no physical E-stop; keep its safety GPIOs unassigned and competition AUTO fail-closed.
constexpr int8_t kMainArmPin = -1;
constexpr int8_t kMainEStopPin = -1;
constexpr int8_t kMainClearFaultPin = -1;
constexpr int8_t kMainArmActiveLevel = -1;
constexpr int8_t kMainEStopActiveLevel = -1;
constexpr int8_t kMainClearFaultActiveLevel = -1;
constexpr bool kMainSafetyInputBiasVerified = false;
constexpr uint32_t kMainSafetyReservedPinMask = (1u << 0) | (1u << 1) | (1u << 4) |
                                                (1u << 5) | (1u << 8) | (1u << 11);

// BNO055 signals are on the MAIN processor in the ir&main schematic.
constexpr int8_t kBnoSdaPin = 4;
constexpr int8_t kBnoSclPin = 5;
constexpr uint32_t kBnoI2cClockHz = 50000;
constexpr uint8_t kBnoAddress = 0x28;
constexpr bool kBnoOffsetsVerified = true; // Offsets from the user-reported working BNO055 test code.
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
constexpr bool kMotorHardwareConfigured = true; // Uses the user's four-wheel tested TB67H450 pin map.
constexpr bool kCompetitionArmed = false;
constexpr uint32_t kMotorPwmFrequencyHz = kTb67h450PwmFrequencyHz;
// Compile-time setup gates; runtime sensor freshness and the physical ARM/E-stop path are still required.
constexpr bool kAutoWheelLiftReady = kCompetitionArmed && kMotorWheelConfigConfigured &&
                                     kMainSafetyInputBiasVerified && kMotorSafetyInputBiasVerified &&
                                     kLinePolarityConfigured;
constexpr bool kAutoFloorReady = kAutoWheelLiftReady &&
                                 kLineSensorEnabledMask == 0x00FFFFFFu && kLineFrontOffsetConfigured &&
                                 kLineAnglesConfigured && kLineDebounceConfigured &&
                                 kBnoOffsetsVerified && kAttackHeadingConfigured;
static_assert(!kAutoFloorReady || kAutoWheelLiftReady, "floor AUTO needs wheel-lift AUTO prerequisites");
// Firmware gates enabled for the reported MAIN GPIO11 kicker connection.
// The high-voltage charge/discharge circuit still needs its separate hardware review.
constexpr bool kKickerHardwareConfigured = true;
constexpr bool kKickerRequestPolicyConfigured = true;
constexpr bool kKickerHeadingWindowConfigured = false; // BNO055 unverified; use IR bearing instead.
constexpr float kKickerHeadingToleranceDeg = 0.0f;
// Kick timing: the IR ball bearing must stay inside this front window while approaching.
constexpr float kKickerBallAngleToleranceDeg = 30.0f;
// Conservative unverified pulse width; cooldown removed at the operator's request (fire as fast as pulses allow).
constexpr uint32_t kKickerPulseMs = 30;
constexpr uint32_t kKickerCooldownMs = 0;
constexpr bool kDribblerHardwareConfigured = false;

}} // namespace soccer::board
