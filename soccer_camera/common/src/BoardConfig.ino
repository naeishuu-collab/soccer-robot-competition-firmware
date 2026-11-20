#pragma once

#include <stdint.h>
#include "RobotControl.ino"

namespace soccer { namespace board {

// MAIN/IR/LINEのピン配置は現行プロジェクト設定に従う。基板の物理導通は要確認。
// 以下のMOTOR配線はユーザーの参考コード準拠。RS485と安全回路の実配線は未検証。
constexpr int8_t kMainRs485TxPin = 0;
constexpr int8_t kMainRs485RxPin = 1;
constexpr int8_t kMainRs485DePin = 8;
constexpr int8_t kKickerPin = 11; // ユーザー報告のMAIN配線。回路レビューは未完。

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
// ユーザー報告の動作MOTORテストコードに一致 (RP2354B + TB67H450 x4)。
constexpr int8_t kMotorIn1Pins[4] = {6, 12, 11, 8};
constexpr int8_t kMotorIn2Pins[4] = {7, 13, 10, 9};
// ユーザー報告のJ2左前ピンは、テストの来歴として残している。
constexpr int8_t kMotorTestIn1CandidatePin = 12;
constexpr int8_t kMotorTestIn2CandidatePin = 13;
constexpr uint8_t kMotorGpioCount = 30; // 最終ZIPのMOTOR親基板BOMはRP2354Aを指定。
constexpr bool kMotorRs485PinsConfigured = true; // 参考コード由来。ハードウェア試験済みではない。
constexpr uint64_t kMotorPwmPinMask = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) |
                                      (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13);
constexpr uint64_t kMotorReservedPinMask = (1u << 0) | (1u << 1) | (1u << 2);
constexpr uint64_t kMotorSafetyReservedPinMask = kMotorPwmPinMask | kMotorReservedPinMask;

constexpr float kMotorWheelRadiusMm = 30.0f;
constexpr float kMotorCenterToWheelMm = 94.762f;
// chase.zipの移動方向とPWM極性。統合後の回転方向は実機で要確認。
constexpr WheelConfig kMotorWheelConfig[4] = {
    {1, -1, 45, 1}, {-1, -1, 315, 1}, {-1, 1, 225, 1}, {1, 1, 135, 1}
};
constexpr bool kMotorWheelConfigConfigured = false;

// 量産基板ではMOTORの安全入力ピンも電気的バイアスも未確認。
constexpr int8_t kMotorArmPin = -1;
constexpr int8_t kMotorEStopPin = -1;
constexpr int8_t kMotorClearFaultPin = -1;
constexpr int8_t kMotorArmActiveLevel = -1;
constexpr int8_t kMotorEStopActiveLevel = -1;
constexpr int8_t kMotorClearFaultActiveLevel = -1;
constexpr bool kMotorSafetyInputBiasVerified = false;

// MAINには物理E-stopがない。安全GPIOは未割当てのままにし、競技用AUTOは不成立時に停止する fail-close とする。
constexpr int8_t kMainArmPin = -1;
constexpr int8_t kMainEStopPin = -1;
constexpr int8_t kMainClearFaultPin = -1;
constexpr int8_t kMainArmActiveLevel = -1;
constexpr int8_t kMainEStopActiveLevel = -1;
constexpr int8_t kMainClearFaultActiveLevel = -1;
constexpr bool kMainSafetyInputBiasVerified = false;
constexpr uint32_t kMainSafetyReservedPinMask = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) |
                                                (1u << 4) | (1u << 5) | (1u << 8) | (1u << 11);

// ir&main 回路図では BNO055 の信号は MAIN プロセッサ側に接続されている。
constexpr int8_t kBnoSdaPin = 4;
constexpr int8_t kBnoSclPin = 5;
constexpr uint32_t kBnoI2cClockHz = 50000;
constexpr uint8_t kBnoAddress = 0x28;
constexpr bool kBnoOffsetsVerified = true; // ユーザー報告の動作BNO055テストコード由来のオフセット。
constexpr bool kAttackHeadingConfigured = false;
constexpr float kAttackHeadingDeg = 0.0f; // 最初に安定した方位を基準とする。設定されるまで無視。

constexpr bool kLineDebounceConfigured = false; // 候補フィルタ。センサーエッジタイミング実測まで無効。
constexpr uint8_t kLineDebounceSamples = 2;
constexpr bool kLinePolarityConfigured = false;
constexpr bool kLineDetectHigh = true; // 候補値のみ。極性実測まで無視される。
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
constexpr bool kMotorHardwareConfigured = true; // ユーザーが4輪で動作確認したTB67H450ピン配置を使用。
constexpr bool kCompetitionArmed = false;
constexpr uint32_t kMotorPwmFrequencyHz = kTb67h450PwmFrequencyHz;
// コンパイル時の設定ゲート。実行時のセンサー鮮度と物理ARM/非常停止の経路も別途必要。
constexpr bool kAutoWheelLiftReady = kCompetitionArmed && kMotorWheelConfigConfigured &&
                                     kMainSafetyInputBiasVerified && kMotorSafetyInputBiasVerified &&
                                     kLinePolarityConfigured;
constexpr bool kAutoFloorReady = kAutoWheelLiftReady &&
                                 kLineSensorEnabledMask == 0x00FFFFFFu && kLineFrontOffsetConfigured &&
                                 kLineAnglesConfigured && kLineDebounceConfigured &&
                                 kBnoOffsetsVerified && kAttackHeadingConfigured;
static_assert(!kAutoFloorReady || kAutoWheelLiftReady, "floor AUTO needs wheel-lift AUTO prerequisites");
// 報告された MAIN GPIO11 のキッカー配線に合わせてファームウェア側ゲートを有効化。
// 高電圧の充放電回路は別途ハードウェアレビューが必要。
constexpr bool kKickerHardwareConfigured = true;
constexpr bool kKickerRequestPolicyConfigured = true;
constexpr bool kKickerHeadingWindowConfigured = false; // BNO055は未検証のため、代わりにIR方位を使う。
constexpr float kKickerHeadingToleranceDeg = 0.0f;
// キック判定: 接近中はIRボール方位がこの正面範囲内に留まる必要がある。
constexpr float kKickerBallAngleToleranceDeg = 30.0f;
// 安全側に倒した未検証のパルス幅。クールダウンは運用者要望で削除済み（パルス間隔だけが制限）。
constexpr uint32_t kKickerPulseMs = 30;
constexpr uint32_t kKickerCooldownMs = 0;
constexpr bool kDribblerHardwareConfigured = false;

}} // namespace soccer::board
