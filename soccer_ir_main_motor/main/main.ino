#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

#include "src/BoardConfig.h"
#include "src/Rs485.h"
#include "src/RobotDevices.h"

using namespace soccer;

constexpr uint8_t kMotorConsoleLineCapacity = 48;
constexpr int kBnoSensorId = 55;
constexpr float kBnoJumpThresholdDegrees = 30.0f;
constexpr float kBnoJumpConfirmToleranceDegrees = 5.0f;
constexpr uint32_t kBnoStartupSettleMs = 500;

constexpr int16_t kAccelOffsetX = 12;
constexpr int16_t kAccelOffsetY = -18;
constexpr int16_t kAccelOffsetZ = 3;
constexpr int16_t kMagOffsetX = -228;
constexpr int16_t kMagOffsetY = 1434;
constexpr int16_t kMagOffsetZ = 2039;
constexpr int16_t kGyroOffsetX = -1;
constexpr int16_t kGyroOffsetY = 0;
constexpr int16_t kGyroOffsetZ = -3;
constexpr int16_t kAccelRadius = 1000;
constexpr int16_t kMagRadius = 441;

PacketParser lineParser;
SequenceTracker lineSequence;
Rs485MasterArbiter busArbiter;
enum BusPhase : uint8_t { kBusIdle, kBusNeedLine };
BusPhase busPhase = kBusIdle;
MainController controller;
HeadingController irTestHeadingController;
AutoMode autoMode = kAutoIrTest;
IrTestController irTestController;
KickerController kickerController;
ManualArmLatch operatorArm;
ManualActionLatch clearFaultAction;
TacticalState previousControllerState = kDisarmed;
LineObservation lineData{};
bool lineHasPacket = false;
uint32_t lastLinePacketMs = 0;
uint32_t linePacketPeriodMs = 0;
uint32_t linePeerRestartsInferred = 0;
bool linePeerResyncPending = false;
uint32_t lineRejectedPackets = 0;
uint32_t lineUnsolicitedPackets = 0;
uint32_t linePollTimeouts = 0;
uint8_t linePollSequence = 0;
uint8_t activeLinePollSequence = 0;
uint32_t busRequestStartedUs = 0;
uint32_t nextSensorPollMs = 0;
uint32_t motorFramesSent = 0;
uint32_t motorTxErrors = 0;
uint32_t lastMotorCommandMs = 0;
bool motorCommandSent = false;
bool motorStopPending = false;
Motion motorMotion{};
bool motorEnabled = false;
int16_t motorConsoleValues[kMotorCommandWheelCount]{};
char motorConsoleLine[kMotorConsoleLineCapacity]{};
uint8_t motorConsoleLineLength = 0;
bool motorConsoleOverflow = false;
bool bootAutoArmNoticeSent = false;
bool motorConsoleCommandSeen = false;
uint32_t lastIrPacketMs = 0;
uint32_t irRequests = 0;
uint32_t irMatchedResponses = 0;
uint32_t irResponseTimeouts = 0;
uint32_t irTxErrors = 0;
uint32_t irUnsolicitedResponses = 0;
uint32_t irRecoveriesInferred = 0;
uint32_t irLastLatencyUs = 0;
uint32_t irMaxLatencyUs = 0;
uint16_t irDirection = 0;
uint16_t irRelativeStrength = 0;
bool irPacketSeen = false;
bool irBallFound = false;
bool irLinkStale = true;
bool irPending = false;
LegacyIrParser irParser;
uint32_t lastHeadingPollMs = 0;
uint32_t lastHeadingSampleMs = 0;
Bno055Sample headingSample{};
uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
uint32_t maxControlUs = 0;
uint16_t controlOverruns = 0;
uint8_t diagnosticPhase = 0;

Adafruit_BNO055 bnoSensor(kBnoSensorId, board::kBnoAddress, &Wire);
struct ArduinoBnoDevices {
    Adafruit_BNO055* sensor;
    bool active = false;
    uint8_t activeAddress = 0;
    uint32_t settleStartedMs = 0;
    bool baseHeadingSet = false;
    float baseHeading = 0.0f;
    float previousHeading = 0.0f;
    bool jumpPending = false;
    float jumpCandidate = 0.0f;
    float previousSampleHeading = 0.0f;
    float previousSampleDelta = 0.0f;
    bool previousSampleSet = false;
    bool previousSampleDeltaSet = false;
};
ArduinoBnoDevices bnoDevices{&bnoSensor};
Bno055Monitor bnoMonitor;

int rs485Available(void*) { return Serial1.available(); }
int rs485Read(void*) { return Serial1.read(); }
void setRs485Tx(void*, bool tx) { digitalWrite(board::kMainRs485DePin, tx ? HIGH : LOW); }
size_t writeRs485(void*, const uint8_t* bytes, size_t length) { return Serial1.write(bytes, length); }
void flushRs485(void*) { Serial1.flush(); }
void waitRs485(void*, uint32_t us) { delayMicroseconds(us); }
Rs485Io rs485Io{&Serial1, rs485Available, rs485Read, setRs485Tx, writeRs485, flushRs485, waitRs485};
Rs485Driver rs485{rs485Io};

void stopIrTest() {
    irTestController.disarm();
    motorMotion = {};
    motorEnabled = false;
    motorStopPending = true;
}

constexpr int8_t kMainSafetyPins[3] = {board::kMainArmPin, board::kMainEStopPin,
                                       board::kMainClearFaultPin};
constexpr int8_t kMainSafetyActiveLevels[3] = {board::kMainArmActiveLevel, board::kMainEStopActiveLevel,
                                               board::kMainClearFaultActiveLevel};

bool mainSafetyInputsReady() {
    return validSafetyInputConfig(kMainSafetyPins, kMainSafetyActiveLevels,
                                  board::kMainSafetyInputBiasVerified,
                                  board::kMainSafetyReservedPinMask);
}

bool mainSafetyInputActive(uint8_t index) {
    return digitalRead(kMainSafetyPins[index]) == kMainSafetyActiveLevels[index];
}

uint32_t irSuccessPercent() {
    return irRequests ? static_cast<uint32_t>((static_cast<uint64_t>(irMatchedResponses) * 100u) / irRequests) : 0;
}

void finishIrCycle(uint32_t nowMs) {
    if (autoMode == kAutoIrTest) {
        busPhase = kBusIdle;
        nextSensorPollMs = nowMs + kIrPollPeriodMs;
    } else {
        busPhase = kBusNeedLine;
    }
}

bool readBnoRegister(uint8_t address, uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return false;
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool bnoBegin(void* context, uint8_t address) {
    ArduinoBnoDevices* devices = static_cast<ArduinoBnoDevices*>(context);
    if (!devices || !devices->sensor || address != board::kBnoAddress) return false;
    devices->active = false;
    devices->activeAddress = 0;
    uint8_t chipId = 0;
    if (!readBnoRegister(address, 0x00, chipId) || chipId != 0xA0) return false;
    if (!devices->sensor->begin(OPERATION_MODE_IMUPLUS)) return false;

    adafruit_bno055_offsets_t offsets{};
    offsets.accel_offset_x = kAccelOffsetX;
    offsets.accel_offset_y = kAccelOffsetY;
    offsets.accel_offset_z = kAccelOffsetZ;
    offsets.gyro_offset_x = kGyroOffsetX;
    offsets.gyro_offset_y = kGyroOffsetY;
    offsets.gyro_offset_z = kGyroOffsetZ;
    offsets.mag_offset_x = kMagOffsetX;
    offsets.mag_offset_y = kMagOffsetY;
    offsets.mag_offset_z = kMagOffsetZ;
    offsets.accel_radius = kAccelRadius;
    offsets.mag_radius = kMagRadius;
    if (board::kBnoOffsetsVerified) devices->sensor->setSensorOffsets(offsets);
    devices->sensor->setExtCrystalUse(true);

    devices->active = true;
    devices->activeAddress = address;
    devices->settleStartedMs = millis();
    devices->baseHeadingSet = false;
    devices->jumpPending = false;
    devices->previousSampleSet = false;
    devices->previousSampleDeltaSet = false;
    return true;
}

bool bnoRead(void* context, float& heading, uint8_t calibration[4]) {
    ArduinoBnoDevices* devices = static_cast<ArduinoBnoDevices*>(context);
    if (!devices || !devices->active) return false;
    uint8_t chipId = 0;
    if (!readBnoRegister(devices->activeAddress, 0x00, chipId) || chipId != 0xA0) {
        devices->active = false;
        devices->activeAddress = 0;
        return false;
    }

    if (static_cast<uint32_t>(millis() - devices->settleStartedMs) < kBnoStartupSettleMs) {
        heading = NAN;
        for (uint8_t i = 0; i < 4; ++i) calibration[i] = 0;
        return true;
    }

    sensors_event_t event{};
    if (!devices->sensor->getEvent(&event, Adafruit_BNO055::VECTOR_EULER)) {
        devices->active = false;
        return false;
    }
    devices->sensor->getCalibration(&calibration[0], &calibration[1], &calibration[2], &calibration[3]);
    float rawHeading = normalizeDegrees(event.orientation.x);
    if (!isfinite(rawHeading)) {
        devices->active = false;
        return false;
    }

    if (!devices->baseHeadingSet) {
        devices->baseHeading = rawHeading;
        devices->previousHeading = rawHeading;
        devices->previousSampleHeading = rawHeading;
        devices->baseHeadingSet = true;
        devices->previousSampleSet = true;
        heading = 0.0f;
        return true;
    }

    const float acceptedDelta = shortestAngleError(rawHeading, devices->previousHeading);
    const float sampleDelta = devices->previousSampleSet
                                  ? shortestAngleError(rawHeading, devices->previousSampleHeading)
                                  : 0.0f;
    const bool continuingFastTurn = devices->previousSampleDeltaSet &&
                                    fabsf(sampleDelta) > 0.1f &&
                                    sampleDelta * devices->previousSampleDelta > 0.0f &&
                                    fabsf(fabsf(sampleDelta) - fabsf(devices->previousSampleDelta)) <=
                                        kBnoJumpConfirmToleranceDegrees;
    float filteredHeading = rawHeading;
    if (fabsf(acceptedDelta) > kBnoJumpThresholdDegrees) {
        // 暫定: 20msあたり30度。ジャイロのログで高速旋回を確認してから引き上げる。
        if (continuingFastTurn ||
            (devices->jumpPending &&
             fabsf(shortestAngleError(rawHeading, devices->jumpCandidate)) <=
                 kBnoJumpConfirmToleranceDegrees)) {
            const float step = acceptedDelta > 0.0f ? kBnoJumpThresholdDegrees : -kBnoJumpThresholdDegrees;
            filteredHeading = normalizeDegrees(devices->previousHeading + step);
            devices->previousHeading = filteredHeading;
            devices->jumpPending = fabsf(shortestAngleError(rawHeading, filteredHeading)) >
                                   kBnoJumpThresholdDegrees;
            if (devices->jumpPending) devices->jumpCandidate = rawHeading;
        } else {
            devices->jumpCandidate = rawHeading;
            devices->jumpPending = true;
            filteredHeading = devices->previousHeading;
        }
    } else {
        devices->previousHeading = rawHeading;
        devices->jumpPending = false;
    }
    devices->previousSampleHeading = rawHeading;
    devices->previousSampleSet = true;
    devices->previousSampleDelta = sampleDelta;
    devices->previousSampleDeltaSet = true;
    heading = normalizeDegrees(filteredHeading - devices->baseHeading);
    return true;
}

Bno055Io bnoIo{&bnoDevices, bnoBegin, bnoRead};

void pollHeading(uint32_t nowMs) {
    if (static_cast<uint32_t>(nowMs - lastHeadingPollMs) < kHeadingPollPeriodMs) return;
    lastHeadingPollMs = nowMs;
    bnoMonitor.poll(bnoIo, board::kBnoAddress, nowMs, headingSample);
    if (headingSample.connected) lastHeadingSampleMs = nowMs;
}

void acceptIrResponse(uint16_t direction, uint16_t strength, uint32_t nowMs) {
    if (busArbiter.activePeer() != kIrRs485Peer || !busArbiter.complete(kIrRs485Peer)) {
        ++irUnsolicitedResponses;
        return;
    }
    irLastLatencyUs = micros() - busRequestStartedUs;
    if (irLastLatencyUs > irMaxLatencyUs) irMaxLatencyUs = irLastLatencyUs;
    ++irMatchedResponses;
    irPending = false;
    irDirection = direction;
    irRelativeStrength = strength; // 従来のDIST字段は相対検出信号であり、cmではない。
    irBallFound = strength != 0;
    if (irLinkStale) ++irRecoveriesInferred;
    irLinkStale = false;
    irPacketSeen = true;
    lastIrPacketMs = nowMs;
    finishIrCycle(nowMs);
}

void finishLineCycle(uint32_t nowMs) {
    busArbiter.complete(kLineRs485Peer);
    busPhase = kBusIdle;
    nextSensorPollMs = nowMs + kIrPollPeriodMs;
}

void acceptLineResponse(const Packet& packet, uint32_t nowMs) {
    if (packet.type != kLineStatus) return;
    if (busArbiter.activePeer() != kLineRs485Peer || packet.sequence != activeLinePollSequence) {
        ++lineUnsolicitedPackets;
        return;
    }
    if (lineHasPacket && !linePeerResyncPending &&
        static_cast<uint32_t>(nowMs - lastLinePacketMs) > kLineFreshTimeoutMs) {
        lineSequence.resetPeer();
        linePeerResyncPending = true;
        ++linePeerRestartsInferred;
    }
    if (!lineSequence.accept(packet.sequence)) {
        ++lineRejectedPackets;
        return;
    }
    LineObservation decoded{};
    if (!decodeLinePacket(packet, decoded, board::kLineSensorAngleCdeg,
                          board::kLineAnglesConfigured ? board::kLineSensorAngleValidMask : 0,
                          board::kLineSensorEnabledMask)) {
        ++lineRejectedPackets;
        return;
    }
    linePacketPeriodMs = lineHasPacket ? static_cast<uint32_t>(nowMs - lastLinePacketMs) : 0;
    lineData = decoded;
    lineHasPacket = true;
    linePeerResyncPending = false;
    lastLinePacketMs = nowMs;
    finishLineCycle(nowMs);
}

void receiveSharedBus(uint32_t nowMs) {
    uint8_t byte = 0;
    uint8_t consumed = 0;
    while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
        uint16_t direction = 0, strength = 0;
        // IRの解析エラー／応答欠落は累計診断に残すだけとし、停止やARM解除はしない。
        if (irParser.feed(byte, nowMs, direction, strength)) acceptIrResponse(direction, strength, nowMs);

        Packet packet{};
        const ParserStats before = lineParser.stats();
        const bool ready = lineParser.feed(byte, nowMs, packet);
        const ParserStats& after = lineParser.stats();
        if (after.crcErrors != before.crcErrors || after.formatErrors != before.formatErrors ||
            after.timeouts != before.timeouts) ++lineRejectedPackets;
        if (ready) acceptLineResponse(packet, nowMs);
    }
    irParser.poll(nowMs);
    if (lineParser.poll(nowMs)) ++lineRejectedPackets;
}

bool sendIrPoll(uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs) || !busArbiter.acquire(kIrRs485Peer)) return false;
    const uint8_t request = 0x01;
    if (!rs485.send(&request, 1, nowUs)) {
        busArbiter.complete(kIrRs485Peer);
        ++irTxErrors;
        return false;
    }
    ++irRequests;
    irPending = true;
    busRequestStartedUs = micros();
    return true;
}

bool sendLinePoll(uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs) || !busArbiter.acquire(kLineRs485Peer)) return false;
    const Packet request = makeLineRequest(linePollSequence);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(request, bytes, sizeof(bytes));
    if (!length || !rs485.send(bytes, length, nowUs)) {
        busArbiter.complete(kLineRs485Peer);
        ++lineRejectedPackets;
        return false;
    }
    activeLinePollSequence = linePollSequence++;
    busRequestStartedUs = micros();
    return true;
}

uint8_t encodeMotorMotionFrame(const Motion& motion, AutoMode mode,
                               uint8_t* bytes, uint8_t capacity) {
    if (!validMotion(motion)) return 0;
    const float axes[] = {motion.x, motion.y, motion.turn};
    int16_t values[kMotorCommandWheelCount]{};
    for (uint8_t i = 0; i < 3; ++i) {
        values[i] = static_cast<int16_t>(roundf(axes[i] * kMotorMotionScale));
    }
    values[3] = mode;
    return encodeMotorCommandFrame(values, bytes, capacity, kMotorMotionCommand);
}

bool sendMotorCommand(uint32_t nowMs, uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs)) return false;
    uint8_t bytes[kMotorCommandFrameSize]{};
    uint8_t length = 0;
    if (!motorStopPending && motorConsoleCommandSeen) {
        length = encodeMotorCommandFrame(motorConsoleValues, bytes, sizeof(bytes));
    } else if (!motorStopPending && motorEnabled) {
        length = encodeMotorMotionFrame(motorMotion, autoMode, bytes, sizeof(bytes));
        if (!length) ++motorTxErrors;
    }
    if (!length) {
        const int16_t stop[kMotorCommandWheelCount]{};
        length = encodeMotorCommandFrame(stop, bytes, sizeof(bytes));
    }
    if (!length || !rs485.send(bytes, length, nowUs)) {
        ++motorTxErrors;
        return false;
    }
    ++motorFramesSent;
    lastMotorCommandMs = nowMs;
    motorCommandSent = true;
    motorStopPending = false;
    return true;
}

void pollMotorConsole(uint32_t nowMs) {
    // USB入力が来たら起動時の自動ARMを取り消す。
    if (Serial.available() > 0) {
        irTestController.disableAutoStart();
        if (!bootAutoArmNoticeSent) {
            bootAutoArmNoticeSent = true;
            if (Serial && Serial.availableForWrite() >= 64)
                Serial.println("USB input received; boot AUTO IR auto-arm disabled");
        }
    }
    while (Serial.available()) {
        const int readValue = Serial.read();
        if (readValue < 0) break;
        const char ch = static_cast<char>(readValue);
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (motorConsoleLineLength + 1 < kMotorConsoleLineCapacity) {
                motorConsoleLine[motorConsoleLineLength++] = ch;
            } else {
                motorConsoleOverflow = true;
            }
            continue;
        }
        motorConsoleLine[motorConsoleLineLength] = '\0';
        int16_t parsed[kMotorCommandWheelCount]{};
        const MainConsoleCommand command = motorConsoleOverflow
                                               ? kConsoleInvalid
                                               : parseMainConsoleCommand(motorConsoleLine, parsed);
        // 起動時の自動ARMは、上の最初の1バイト受信時点で既に解除されている。
        if (command == kConsoleAutoWheelLift || command == kConsoleAutoFloor ||
            command == kConsoleAutoIrTest) {
            autoMode = autoModeForCommand(command, board::kAutoWheelLiftReady, board::kAutoFloorReady);
            irTestController.disarm();
            motorConsoleCommandSeen = false;
            motorMotion = {};
            motorEnabled = false;
            motorStopPending = true;
            for (int16_t& value : motorConsoleValues) value = 0;
            if (Serial && Serial.availableForWrite() >= 72) {
                if (autoMode == kAutoStopped) Serial.println("AUTO_BLOCKED; hardware configuration gate closed");
                else if (autoMode == kAutoIrTest) Serial.println("AUTO IR selected; stopped; send ARM IR with a fresh ball");
                else Serial.println(autoMode == kAutoFloor ? "AUTO FLOOR selected; physical ARM still required"
                                                           : "AUTO FLOAT selected; physical ARM still required");
            }
        } else if (command == kConsoleArmIrTest) {
            const bool irFresh = irPacketSeen &&
                                 static_cast<uint32_t>(nowMs - lastIrPacketMs) <= kIrFreshTimeoutMs;
            if (autoMode == kAutoIrTest &&
                irTestController.arm(irFresh, irBallFound, irDirection)) {
                if (Serial && Serial.availableForWrite() >= 72)
                    Serial.println("IR TEST armed; 4-way translation; IR loss -> stop; S stops");
            } else {
                autoMode = kAutoStopped;
                motorConsoleCommandSeen = false;
                for (int16_t& value : motorConsoleValues) value = 0;
                stopIrTest();
                if (Serial && Serial.availableForWrite() >= 64)
                    Serial.println("IR TEST blocked; select AUTO IR and wait for a fresh ball");
            }
        } else {
            autoMode = kAutoStopped;
            irTestController.disarm();
            if (command == kConsoleManualMotor || command == kConsoleStop) {
                for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) motorConsoleValues[i] = parsed[i];
            } else {
                for (int16_t& value : motorConsoleValues) value = 0;
                if (Serial && Serial.availableForWrite() >= 32) Serial.println("MOTOR_CMD_INVALID; stopped");
            }
            motorConsoleCommandSeen = true;
            if (command == kConsoleStop || command == kConsoleInvalid) motorStopPending = true;
        }
        motorConsoleLineLength = 0;
        motorConsoleOverflow = false;
    }
    // ソフトウェアコマンドタイムアウトは意図的に削除。手動停止は'S'のみ。
}

void pollSharedBus(uint32_t nowMs) {
    receiveSharedBus(nowMs);
    const uint32_t nowUs = micros();
    if (busArbiter.activePeer() == kIrRs485Peer &&
        static_cast<uint32_t>(nowUs - busRequestStartedUs) >= kIrReplyTimeoutUs) {
        busArbiter.complete(kIrRs485Peer);
        irPending = false;
        ++irResponseTimeouts;
        // 単発の応答欠落では停止しない。探索モードへ移行して競技を継続する。
        finishIrCycle(nowMs);
    } else if (busArbiter.activePeer() == kLineRs485Peer &&
               static_cast<uint32_t>(nowUs - busRequestStartedUs) >= kLineReplyTimeoutUs) {
        ++linePollTimeouts;
        finishLineCycle(nowMs);
    }

    if (busArbiter.idle() && autoMode == kAutoIrTest && busPhase == kBusNeedLine) {
        busPhase = kBusIdle;
        nextSensorPollMs = nowMs + kIrPollPeriodMs;
    }
    if (busArbiter.idle() && motorStopPending) {
        sendMotorCommand(nowMs, nowUs);
    } else if (busArbiter.idle() && busPhase == kBusNeedLine) {
        sendLinePoll(nowUs);
    } else if (busArbiter.idle() && busPhase == kBusIdle &&
               static_cast<int32_t>(nowMs - nextSensorPollMs) >= 0) {
        sendIrPoll(nowUs);
    } else if (busArbiter.idle() && busPhase == kBusIdle &&
               (!motorCommandSent || static_cast<uint32_t>(nowMs - lastMotorCommandMs) >= kMotorCommandPeriodMs)) {
        sendMotorCommand(nowMs, nowUs);
    }

    if (irPacketSeen && static_cast<uint32_t>(nowMs - lastIrPacketMs) > kIrFreshTimeoutMs) {
        irBallFound = false;
        irLinkStale = true;
    }
}

void printLineLevels() {
    Serial.print("line_raw[0..23]=");
    for (uint8_t i = 0; i < 24; ++i) Serial.print((lineData.rawLevels & (1u << i)) ? 'H' : 'L');
}

void setup() {
    Serial.begin(115200);
    irTestController.enableAutoStart();
    if (Serial && Serial.availableForWrite() >= 48)
        Serial.println("BOOT MAIN; AUTO IR waits for first fresh ball; diagnostics active");
    if (Serial && Serial.availableForWrite() >= 64)
        Serial.println("USB: AUTO [FLOAT] / AUTO FLOOR / AUTO IR; ARM IR; S=stop; M <v0> <v1> <v2> <v3>");
    if (Serial && Serial.availableForWrite() >= 64)
        Serial.println("MOTOR M range -255..255, 256=brake; no command timeout (S / power switch only)");
    if (Serial && Serial.availableForWrite() >= 96) {
        Serial.print("BOOT AUTO IR: fresh ball starts; IR loss -> stop; PWM ");
        Serial.print(kIrTestPwmFloor);
        Serial.print("..");
        Serial.println(kIrTestPwmCap);
    }

    pinMode(board::kMainRs485DePin, OUTPUT);
    digitalWrite(board::kMainRs485DePin, LOW);
    pinMode(board::kKickerPin, OUTPUT);
    digitalWrite(board::kKickerPin, LOW);
    if (mainSafetyInputsReady())
        for (int8_t pin : kMainSafetyPins) pinMode(pin, INPUT);
    Serial1.setTX(board::kMainRs485TxPin);
    Serial1.setRX(board::kMainRs485RxPin);
    Serial1.begin(kRs485Baud);

    Wire.setSDA(board::kBnoSdaPin);
    Wire.setSCL(board::kBnoSclPin);
    Wire.begin();
    Wire.setClock(board::kBnoI2cClockHz);
}

void loop() {
    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();
    pollMotorConsole(nowMs);
    pollSharedBus(nowMs);
    pollHeading(nowMs);
    if (static_cast<uint32_t>(nowUs - lastControlUs) < kControlPeriodUs) return;

    const uint32_t startedUs = micros();
    if (lastControlUs && static_cast<uint32_t>(nowUs - lastControlUs) > 2 * kControlPeriodUs)
        ++controlOverruns;
    lastControlUs = nowUs;

    MainInput input{};
    const bool safetyInputsReady = mainSafetyInputsReady();
    const bool emergencyStopActive = !safetyInputsReady || mainSafetyInputActive(1);
    const bool armPressed = safetyInputsReady && mainSafetyInputActive(0);
    const bool clearFaultPressed = safetyInputsReady && mainSafetyInputActive(2);
    const bool armPermitted = operatorArm.update(safetyInputsReady, armPressed,
                                                  !safetyInputsReady || emergencyStopActive);
    if (previousControllerState == kFault) {
        input.clearFault = clearFaultAction.update(safetyInputsReady, clearFaultPressed,
                                                    !safetyInputsReady || emergencyStopActive);
    } else {
        clearFaultAction.reset();
    }
    input.nowMs = nowMs;
    const bool autoConfigured = autoMode == kAutoWheelLift ? board::kAutoWheelLiftReady
                               : autoMode == kAutoFloor ? board::kAutoFloorReady : false;
    input.armed = autoConfigured && armPermitted;
    input.fault = !safetyInputsReady;
    input.lineFresh = lineHasPacket && static_cast<uint32_t>(nowMs - lastLinePacketMs) <= kLineFreshTimeoutMs;
    input.line = lineData;
    input.irFresh = irPacketSeen && static_cast<uint32_t>(nowMs - lastIrPacketMs) <= kIrFreshTimeoutMs;
    input.ballFound = input.irFresh && irBallFound;
    input.ballAngleDeg = irDirection;
    input.relativeSignal = irRelativeStrength; // センサーの相対値。物理距離ではない。
    input.headingValid = headingSample.headingValid &&
                         static_cast<uint32_t>(nowMs - lastHeadingSampleMs) <= kHeadingFreshTimeoutMs;
    input.headingRequired = board::kAttackHeadingConfigured;
    input.headingDeg = headingSample.headingDeg;
    input.headingTargetDeg = board::kAttackHeadingConfigured ? board::kAttackHeadingDeg : NAN;
    input.emergencyStop = emergencyStopActive;

    const MainOutput output = controller.step(input);
    previousControllerState = output.state;

    KickerInput kickerInput{};
    // LINE基板は故障中のため使用しない。
    kickerInput.commandFresh = input.irFresh && (!input.headingRequired || input.headingValid);
    kickerInput.armed = input.armed;
    kickerInput.emergencyStop = input.emergencyStop;
    kickerInput.fault = input.fault || output.state == kFault;
    kickerInput.clearFault = input.clearFault;
    kickerInput.hardwareConfigured = board::kKickerHardwareConfigured;
    kickerInput.ballFresh = input.irFresh;
    kickerInput.ballDetected = input.ballFound;
    kickerInput.headingValid = input.headingValid;
    kickerInput.headingAligned = board::kKickerHeadingWindowConfigured && board::kAttackHeadingConfigured &&
        isfinite(board::kKickerHeadingToleranceDeg) && board::kKickerHeadingToleranceDeg > 0.0f &&
        fabsf(shortestAngleError(input.headingDeg, board::kAttackHeadingDeg)) <= board::kKickerHeadingToleranceDeg;
    // LINE基板を使用しないので、白線ガードは常にクリア。
    kickerInput.lineClear = true;
    // キック判定は未検証のBNO055ではなくIR方位から行う。ボールが正面にあること。
    kickerInput.ballAligned = input.ballFound && input.ballAngleDeg < 360 &&
        fabsf(shortestAngleError(static_cast<float>(input.ballAngleDeg), 0.0f)) <=
            board::kKickerBallAngleToleranceDeg;
    kickerInput.request = kickerRequestAllowed(board::kKickerRequestPolicyConfigured,
                                                output.state == kApproachBall, kickerInput);
    // AUTO IRはLINEなしで動作する。
    if (autoMode == kAutoIrTest) {
        kickerInput.commandFresh = input.irFresh;
        kickerInput.armed = irTestController.armed();
        kickerInput.emergencyStop = false;
        kickerInput.fault = false;
        // LINEとカメラは使わない。近接信号と正面方位が必要。
        kickerInput.lineClear = true;
        const bool irBallFront = irBallCloseAndFront(input.irFresh, input.ballFound,
            input.ballAngleDeg, input.relativeSignal, kIrKickCloseSignal);
        kickerInput.ballFresh = input.irFresh;
        kickerInput.ballDetected = input.ballFound;
        kickerInput.ballAligned = irBallFront;
        // ジャイロ方位ゲートは使わず、IR方位が合ったら蹴る。
        kickerInput.headingValid = true;
        kickerInput.headingAligned = true;
        kickerInput.request = irBallFront &&
            (kickerController.state() == kKickerReady || kickerController.state() == kKickerFiring);
    }
    const KickerOutput kickerOutput = kickerController.update(kickerInput, nowMs,
                                                              board::kKickerPulseMs,
                                                              board::kKickerCooldownMs);
    digitalWrite(board::kKickerPin, kickerOutput.pulseActive() ? HIGH : LOW);

    if (output.state == kFault) operatorArm.reset();
    else clearFaultAction.reset();
    const bool wasMotorEnabled = motorEnabled;
    motorMotion = output.motion;
    motorEnabled = autoConfigured && input.armed && output.state != kFault;
    if (autoMode == kAutoIrTest) {
        // 台上試験専用の例外。LINEガードもMAIN非常停止も使わない。MOTORの電源遮断には手が届く状態を保つ。
        irTestController.armOnFirstFreshBall(input.irFresh, input.ballFound, irDirection);
        motorEnabled = irTestController.update(input.irFresh, input.ballFound, irDirection, motorMotion);
        float headingTurn = 0.0f;
        if (motorEnabled && (!input.headingValid ||
            !irTestHeadingController.update(input.headingDeg, 0.0f, true, headingTurn))) {
            motorEnabled = false;
        }
        if (motorEnabled) {
            const float relativeHeading = shortestAngleError(input.headingDeg, 0.0f);
            if ((relativeHeading >= 90.0f && headingTurn > 0.0f) ||
                (relativeHeading <= -90.0f && headingTurn < 0.0f)) {
                headingTurn = 0.0f;
            }
            motorMotion.turn = headingTurn;
        } else {
            motorMotion = {};
            irTestHeadingController.reset();
        }
        if (wasMotorEnabled && !motorEnabled) motorStopPending = true;
    }

    const uint32_t elapsed = micros() - startedUs;
    if (elapsed > maxControlUs) maxControlUs = elapsed;
    if (elapsed > kControlPeriodUs) ++controlOverruns;
    if (static_cast<uint32_t>(nowMs - lastDebugMs) >= kDiagnosticPeriodMs) {
        lastDebugMs = nowMs;
        if (Serial && Serial.availableForWrite() >= 240) {
            if (diagnosticPhase == 0) {
                Serial.print("DIAG IR req/match/success%/timeouts/recovery="); Serial.print(irRequests); Serial.print('/');
                Serial.print(irMatchedResponses); Serial.print('/'); Serial.print(irSuccessPercent()); Serial.print('/');
                Serial.print(irResponseTimeouts); Serial.print('/'); Serial.print(irRecoveriesInferred);
                Serial.print(" latency_us(last/max)=");
                Serial.print(irLastLatencyUs); Serial.print('/'); Serial.print(irMaxLatencyUs);
                Serial.print(" dir="); Serial.print(irDirection); Serial.print(" relative="); Serial.print(irRelativeStrength);
                Serial.print(" bad/timeout/tx/unmatched="); Serial.print(irParser.errors()); Serial.print('/');
                Serial.print(irParser.timeouts()); Serial.print('/'); Serial.print(irTxErrors); Serial.print('/');
                Serial.println(irUnsolicitedResponses);
            } else if (diagnosticPhase == 1) {
                Serial.print("DIAG LINE age/period_ms=");
                Serial.print(lineHasPacket ? static_cast<uint32_t>(nowMs - lastLinePacketMs) : 0xFFFFFFFFu);
                Serial.print('/'); Serial.print(linePacketPeriodMs);
                Serial.print(" crc/timeout/seq_gap/reject/unmatched/poll_timeout/resync="); Serial.print(lineParser.stats().crcErrors);
                Serial.print('/'); Serial.print(lineParser.stats().timeouts); Serial.print('/');
                Serial.print(lineSequence.gaps()); Serial.print('/'); Serial.print(lineSequence.rejected());
                Serial.print('/'); Serial.print(lineRejectedPackets); Serial.print('/'); Serial.print(lineUnsolicitedPackets);
                Serial.print('/'); Serial.print(linePollTimeouts); Serial.print('/'); Serial.print(linePeerRestartsInferred);
                Serial.print(" count/angle/ambiguous="); Serial.print(lineData.count); Serial.print('/');
                Serial.print(lineData.angleCdeg); Serial.print('/'); Serial.print(lineData.ambiguous);
                Serial.print(" polarity="); Serial.println(lineData.polarityValid ? (lineData.activeHigh ? "HIGH" : "LOW") : "UNKNOWN");
                printLineLevels(); Serial.println();
            } else if (diagnosticPhase == 2) {
                Serial.print("DIAG BNO connected/valid/address="); Serial.print(headingSample.connected); Serial.print('/');
                Serial.print(headingSample.headingValid); Serial.print("/0x"); Serial.print(headingSample.address, HEX);
                Serial.print(" age_ms="); Serial.print(headingSample.connected ? 0 : static_cast<uint32_t>(nowMs - lastHeadingSampleMs));
                Serial.print(" cal(S/G/A/M)="); Serial.print(headingSample.calibration[0]); Serial.print('/');
                Serial.print(headingSample.calibration[1]); Serial.print('/'); Serial.print(headingSample.calibration[2]);
                Serial.print('/'); Serial.print(headingSample.calibration[3]);
                Serial.print(" heading_rel="); Serial.println(headingSample.headingDeg);
            } else {
                Serial.print("DIAG auto/state=");
                Serial.print(autoMode == kAutoIrTest ? "IR_TEST" :
                             autoMode == kAutoWheelLift ? "FLOAT" : autoMode == kAutoFloor ? "FLOOR" : "OFF");
                Serial.print('/');
                Serial.print(static_cast<uint8_t>(output.state)); Serial.print(" MOTOR_frames_no_ACK/errors=");
                Serial.print(motorFramesSent); Serial.print('/'); Serial.print(motorTxErrors);
                Serial.print(" IR_test_armed="); Serial.print(irTestController.armed());
                Serial.print(" bus_rx/tx/busy/arbiter_denied=");
                Serial.print(rs485.diagnostics().rxBytes); Serial.print('/'); Serial.print(rs485.diagnostics().txFrames);
                Serial.print('/'); Serial.print(rs485.diagnostics().txBusy); Serial.print('/');
                Serial.print(busArbiter.collisionsPrevented());
                Serial.print(" control_max_us/overrun="); Serial.print(maxControlUs); Serial.print('/');
                Serial.print(controlOverruns); Serial.print(" kicker_state/mock_pulse/event=");
                Serial.print(static_cast<uint8_t>(kickerOutput.state)); Serial.print('/');
                Serial.print(kickerOutput.mockPulseActive); Serial.print('/');
                Serial.println(static_cast<uint8_t>(kickerOutput.event));
            }
            diagnosticPhase = static_cast<uint8_t>((diagnosticPhase + 1) & 0x03u);
        }
    }
}
