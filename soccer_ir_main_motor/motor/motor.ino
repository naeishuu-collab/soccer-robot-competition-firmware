#include <Arduino.h>

#include "src/BoardConfig.h"
#include "src/Rs485.h"

using namespace soccer;

constexpr Tb67h450ChannelConfig kDriver[4] = {
    {board::kMotorIn1Pins[0], board::kMotorIn2Pins[0], kTb67h450MaxDuty},
    {board::kMotorIn1Pins[1], board::kMotorIn2Pins[1], kTb67h450MaxDuty},
    {board::kMotorIn1Pins[2], board::kMotorIn2Pins[2], kTb67h450MaxDuty},
    {board::kMotorIn1Pins[3], board::kMotorIn2Pins[3], kTb67h450MaxDuty}
};
constexpr bool kDriverTableConfigured = true;
int16_t motorCurrent[4]{};
uint32_t reverseUntil[4]{};
bool reversing[4]{};
uint32_t lastRampMs[4]{};
MotorCommandPriority commandPriority;

bool motorRs485PinsReady() {
    return validRs485PinConfig(board::kMotorRs485PinsConfigured, board::kMotorGpioCount,
                               board::kMotorRs485TxPin, board::kMotorRs485RxPin, board::kMotorRs485DePin);
}

bool motorHardwareReady() {
    return board::kMotorHardwareConfigured && motorRs485PinsReady() && kDriverTableConfigured &&
           validMotorPwmSettings(board::kMotorPwmFrequencyHz, kMotorPwmRange, F_CPU) &&
           validMotorPinMap(kDriver, board::kMotorPwmPinMask,
                            board::kMotorReservedPinMask, board::kMotorGpioCount);
}

int rs485Available(void*) { return Serial1.available(); }
int rs485Read(void*) { return Serial1.read(); }
void rs485Direction(void*, bool tx) {
    if (motorRs485PinsReady()) digitalWrite(board::kMotorRs485DePin, tx ? HIGH : LOW);
}
size_t rs485Write(void*, const uint8_t* bytes, size_t length) { return Serial1.write(bytes, length); }
void rs485Flush(void*) { Serial1.flush(); }
void rs485Delay(void*, uint32_t us) { delayMicroseconds(us); }
Rs485Io rs485Io{&Serial1, rs485Available, rs485Read, rs485Direction, rs485Write, rs485Flush, rs485Delay};
Rs485Driver rs485{rs485Io};
MotorCommandFrameParser parser;
int16_t command[kMotorCommandWheelCount]{};
bool commandEnabled = false;
bool hasCommand = false;
bool hadValidCommand = false;
uint32_t lastCommandMs = 0;
uint32_t lastTickUs = 0;
uint32_t lastDebugMs = 0;
uint32_t commandTimeouts = 0;
uint16_t rejectedPackets = 0;

void motorOutput(uint8_t index, int16_t value) {
    if (value != kMotorBrakeCommand && board::kMotorReverseOutput[index]) value = -value;
    const int8_t in1 = kDriver[index].pinIn1;
    const int8_t in2 = kDriver[index].pinIn2;
    if (value == kMotorBrakeCommand) {
        analogWrite(in1, 0);
        analogWrite(in2, 0);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, HIGH);
    } else if (value == 0) {
        analogWrite(in1, 0);
        analogWrite(in2, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    } else if (value > 0) {
        analogWrite(in2, 0);
        digitalWrite(in2, LOW);
        analogWrite(in1, value);
    } else {
        analogWrite(in1, 0);
        digitalWrite(in1, LOW);
        analogWrite(in2, -value);
    }
}

void writeSafeStop() {
    const uint32_t nowMs = millis();
    const bool hardwareReady = motorHardwareReady();
    for (uint8_t i = 0; i < 4; ++i) {
        motorCurrent[i] = 0;
        reversing[i] = false;
        reverseUntil[i] = 0;
        lastRampMs[i] = nowMs;
        if (hardwareReady) motorOutput(i, 0);
    }
}

bool updateMotorOutput(uint8_t index, uint32_t nowMs) {
    const int16_t requested = command[index];
    if (requested == kMotorBrakeCommand) {
        motorCurrent[index] = kMotorBrakeCommand;
        reversing[index] = false;
        lastRampMs[index] = nowMs;
        motorOutput(index, requested);
        return true;
    }
    if (requested == 0) {
        motorCurrent[index] = 0;
        reversing[index] = false;
        lastRampMs[index] = nowMs;
        motorOutput(index, 0);
        return true;
    }
    int16_t current = motorCurrent[index];
    if (current == kMotorBrakeCommand) {
        current = 0;
        motorCurrent[index] = 0;
        motorOutput(index, 0);
    }

    if (reversing[index]) {
        if (static_cast<int32_t>(nowMs - reverseUntil[index]) < 0) return true;
        reversing[index] = false;
        current = 0;
    }
    if (current != 0 && ((current > 0) != (requested > 0))) {
        motorCurrent[index] = 0;
        motorOutput(index, 0);
        reversing[index] = true;
        reverseUntil[index] = nowMs + kTb67h450ReversalDwellMs;
        return true;
    }

    // For requested AUTO IR / normal running values, skip the slow 0-to-floor ramp.
    // Keep lower manual commands on the configured gradual ramp.
    const int16_t startFloor = static_cast<int16_t>(kIrTestPwmFloor);
    if (current == 0 && (requested >= startFloor || requested <= -startFloor)) {
        current = requested > 0 ? startFloor : -startFloor;
        motorCurrent[index] = current;
        lastRampMs[index] = nowMs;
        motorOutput(index, current);
        return true;
    }

    if (static_cast<uint32_t>(nowMs - lastRampMs[index]) < kRampIntervalMs) return true;
    lastRampMs[index] = nowMs;
    if (current < requested) {
        const int16_t next = static_cast<int16_t>(current + kRampStep);
        current = next > requested ? requested : next;
    } else if (current > requested) {
        const int16_t next = static_cast<int16_t>(current - kRampStep);
        current = next < requested ? requested : next;
    }
    motorCurrent[index] = current;
    motorOutput(index, current);
    return true;
}

bool updateMotorOutputs(uint32_t nowMs) {
    if (!motorHardwareReady()) return false;
    for (uint8_t i = 0; i < 4; ++i)
        if (!updateMotorOutput(i, nowMs)) return false;
    return true;
}

void rejectMotorFrame() {
    ++rejectedPackets;
    commandPriority.latchStop();
    hasCommand = false;
    hadValidCommand = false;
    commandEnabled = false;
    for (int16_t& value : command) value = 0;
    writeSafeStop();
}

bool allocateMotionPwm(int16_t values[kMotorCommandWheelCount]) {
    Motion motion;
    motion.x = static_cast<float>(values[0]) / kMotorMotionScale;
    motion.y = static_cast<float>(values[1]) / kMotorMotionScale;
    motion.turn = static_cast<float>(values[2]) / kMotorMotionScale;
    const int16_t mode = values[3];
    if (!motorCommandValuesForMotion(motion, true, board::kMotorWheelConfig, values)) return false;

    const uint8_t pwmCap = mode == kAutoIrTest ? kIrTestPwmCap
                           : mode == kAutoFloor ? kAutoFloorPwmCap : kAutoWheelLiftPwmCap;
    if (mode == kAutoIrTest) {
        scaleMotorCommandValuesToPeak(values, kIrTestPwmCap);
        floorMotorCommandValues(values, kIrTestPwmFloor);
    }
    capMotorCommandValues(values, pwmCap);
    return true;
}

void receiveCommands(uint32_t nowMs) {
    // MAINからの指令が途切れたら安全停止する（競技中の暴走防止）。
    // kSoftwareWatchdogEnabled=false で旧挙動に戻せるが、その場合は物理スイッチが唯一の停止手段。
    if (kSoftwareWatchdogEnabled && hasCommand &&
        static_cast<uint32_t>(nowMs - lastCommandMs) > kCommandTimeoutMs) {
        ++commandTimeouts;
        rejectMotorFrame();
        return;
    }
    uint8_t byte = 0;
    uint8_t consumed = 0;
    while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
        int16_t next[kMotorCommandWheelCount]{};
        uint8_t type = 0;
        const MotorFrameStatus status = parser.feed(byte, nowMs, next, type);
        if (status == kMotorFrameInvalid ||
            (status == kMotorFrameReady && type == kMotorMotionCommand && !allocateMotionPwm(next))) {
            rejectMotorFrame();
            continue;
        }
        if (status != kMotorFrameReady) continue;
        bool nextEnabled = false;
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) {
            nextEnabled |= next[i] != 0;
        }
        if (!commandPriority.accept(nextEnabled)) {
            hasCommand = true;
            hadValidCommand = true;
            lastCommandMs = nowMs;
            continue;
        }
        commandEnabled = nextEnabled;
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) command[i] = next[i];
        hasCommand = true;
        hadValidCommand = true;
        lastCommandMs = nowMs;
    }
    if (parser.poll(nowMs)) {
        rejectMotorFrame();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.print("BOOT MOTOR; watchdog_ms=");
    Serial.println(kSoftwareWatchdogEnabled ? kCommandTimeoutMs : 0);
    if (motorRs485PinsReady()) {
        pinMode(board::kMotorRs485DePin, OUTPUT);
        digitalWrite(board::kMotorRs485DePin, LOW);
        Serial1.setTX(board::kMotorRs485TxPin);
        Serial1.setRX(board::kMotorRs485RxPin);
        Serial1.begin(kRs485Baud);
    }
    const bool outputReady = motorHardwareReady();
    if (outputReady) {
        // ユーザー報告の動作テストコードに合わせる: 周波数を先に、次にレンジを設定。
        analogWriteFreq(board::kMotorPwmFrequencyHz);
        analogWriteRange(kMotorPwmRange);
    }
    for (uint8_t i = 0; i < 4; ++i) {
        digitalWrite(kDriver[i].pinIn1, LOW);
        digitalWrite(kDriver[i].pinIn2, LOW);
        pinMode(kDriver[i].pinIn1, OUTPUT);
        pinMode(kDriver[i].pinIn2, OUTPUT);
        if (outputReady) {
            analogWrite(kDriver[i].pinIn1, 0);
            analogWrite(kDriver[i].pinIn2, 0);
        }
        motorCurrent[i] = 0;
        reversing[i] = false;
        reverseUntil[i] = 0;
        lastRampMs[i] = millis();
    }
    writeSafeStop();
}

void loop() {
    const uint32_t nowUs = micros();
    if (static_cast<uint32_t>(nowUs - lastTickUs) < kMotorPeriodUs) {
        if (motorRs485PinsReady()) receiveCommands(millis());
        return;
    }
    lastTickUs = nowUs;
    const uint32_t nowMs = millis();
    if (motorRs485PinsReady()) receiveCommands(nowMs);
    const bool driveAllowed = motorHardwareReady() && hasCommand && commandEnabled;
    if (!driveAllowed || commandPriority.stopPending()) {
        writeSafeStop(); // ローカルタイムアウトや無効/未設定時は直前の駆動命令をクリアする。
    } else if (!updateMotorOutputs(nowMs)) {
        ++rejectedPackets;
        commandPriority.latchStop();
        writeSafeStop();
    }
    commandPriority.completeOutputTick();
    if (static_cast<uint32_t>(nowMs - lastDebugMs) >= 1000) {
        lastDebugMs = nowMs;
        if (Serial && Serial.availableForWrite() >= 200) {
            Serial.print(" output_ready="); Serial.print(motorHardwareReady());
            Serial.print(" command_active="); Serial.print(driveAllowed);
            Serial.print(" checksum/format/range/frame_timeout/command_timeout=");
            Serial.print(parser.stats().checksumErrors); Serial.print('/');
            Serial.print(parser.stats().formatErrors); Serial.print('/');
            Serial.print(parser.stats().rangeErrors); Serial.print('/');
            Serial.print(parser.stats().timeouts); Serial.print('/'); Serial.print(commandTimeouts);
            const Rs485Diagnostics& bus = rs485.diagnostics();
            Serial.print(" RS485_rx/busy/errors="); Serial.print(bus.rxBytes); Serial.print('/');
            Serial.print(bus.txBusy); Serial.print('/'); Serial.print(bus.rxErrors + bus.txErrors);
            Serial.print(" rejected="); Serial.println(rejectedPackets);
        }
    }
}
