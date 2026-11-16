#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <BoardConfig.h>
#include <RobotControl.h>
#include <RobotDevices.h>
#include <Rs485.h>

using namespace soccer;

// Keep diagnostics the default until the carriers, stop circuit, and driver are validated.
constexpr bool kDiagnosticMode = true;
constexpr uint32_t kControlPeriodUs = 10000;
constexpr uint32_t kIrPollPeriodMs = 20;
constexpr uint32_t kIrReplyTimeoutUs = 15000;
constexpr uint32_t kLineReplyTimeoutUs = 8000;
constexpr uint32_t kIrFreshTimeoutMs = 60;
constexpr uint32_t kLineFreshTimeoutMs = 100;
constexpr uint32_t kHeadingPollPeriodMs = 20;
constexpr uint32_t kHeadingFreshTimeoutMs = 100;
constexpr uint32_t kDiagnosticPeriodMs = 250;
constexpr uint32_t kMotorCommandPeriodMs = 40;
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
uint8_t motorSequence = 0;
uint32_t motorFramesSent = 0;
uint32_t motorTxErrors = 0;
uint32_t lastMotorCommandMs = 0;
bool motorCommandSent = false;
Motion motorMotion{};
bool motorEnabled = false;
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
        // shortcut: 30 degrees per 20 ms; raise only after gyro logs confirm faster real turns.
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
    irRelativeStrength = strength; // The legacy DIST field is a relative detector signal, not centimetres.
    irBallFound = strength != 0;
    if (irLinkStale) ++irRecoveriesInferred;
    irLinkStale = false;
    irPacketSeen = true;
    lastIrPacketMs = nowMs;
    busPhase = kBusNeedLine;
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

bool sendMotorCommand(uint32_t nowMs, uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs)) return false;
    const Packet packet = makeMotorPacket(motorMotion, motorEnabled, motorSequence);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(packet, bytes, sizeof(bytes));
    if (!length || !rs485.send(bytes, length, nowUs)) {
        ++motorTxErrors;
        return false;
    }
    ++motorSequence;
    ++motorFramesSent;
    lastMotorCommandMs = nowMs;
    motorCommandSent = true;
    return true;
}

void pollSharedBus(uint32_t nowMs) {
    receiveSharedBus(nowMs);
    const uint32_t nowUs = micros();
    if (busArbiter.activePeer() == kIrRs485Peer &&
        static_cast<uint32_t>(nowUs - busRequestStartedUs) >= kIrReplyTimeoutUs) {
        busArbiter.complete(kIrRs485Peer);
        irPending = false;
        ++irResponseTimeouts;
        busPhase = kBusNeedLine;
    } else if (busArbiter.activePeer() == kLineRs485Peer &&
               static_cast<uint32_t>(nowUs - busRequestStartedUs) >= kLineReplyTimeoutUs) {
        ++linePollTimeouts;
        finishLineCycle(nowMs);
    }

    if (busArbiter.idle() && busPhase == kBusNeedLine) {
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
    if (Serial && Serial.availableForWrite() >= 64)
        Serial.println("BOOT MAIN; MOTOR OUTPUT DISABLED; diagnostic mode");

    pinMode(board::kMainRs485DePin, OUTPUT);
    digitalWrite(board::kMainRs485DePin, LOW);
    if (mainSafetyInputsReady())
        for (int8_t pin : kMainSafetyPins) pinMode(pin, INPUT);
    Serial1.setTX(board::kMainRs485TxPin);
    Serial1.setRX(board::kMainRs485RxPin);
    Serial1.begin(115200);

    Wire.setSDA(board::kBnoSdaPin);
    Wire.setSCL(board::kBnoSclPin);
    Wire.begin();
    Wire.setClock(board::kBnoI2cClockHz);
}

void loop() {
    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();
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
    input.armed = !kDiagnosticMode && board::kCompetitionArmed && armPermitted;
    input.fault = !safetyInputsReady;
    input.lineFresh = lineHasPacket && static_cast<uint32_t>(nowMs - lastLinePacketMs) <= kLineFreshTimeoutMs;
    input.line = lineData;
    input.irFresh = irPacketSeen && static_cast<uint32_t>(nowMs - lastIrPacketMs) <= kIrFreshTimeoutMs;
    input.ballFound = input.irFresh && irBallFound;
    input.ballAngleDeg = irDirection;
    input.relativeSignal = irRelativeStrength; // Relative sensor signal; not a physical distance.
    input.headingValid = headingSample.headingValid &&
                         static_cast<uint32_t>(nowMs - lastHeadingSampleMs) <= kHeadingFreshTimeoutMs;
    input.headingRequired = board::kAttackHeadingConfigured;
    input.headingDeg = headingSample.headingDeg;
    input.headingTargetDeg = board::kAttackHeadingConfigured ? board::kAttackHeadingDeg : NAN;
    input.emergencyStop = emergencyStopActive;

    const MainOutput output = controller.step(input);
    previousControllerState = output.state;

    KickerInput kickerInput{};
    kickerInput.commandFresh = input.irFresh && input.lineFresh && (!input.headingRequired || input.headingValid);
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
    kickerInput.lineClear = input.lineFresh && !input.line.detected;
    kickerInput.request = kickerRequestAllowed(board::kKickerRequestPolicyConfigured,
                                                output.state == kApproachBall, kickerInput);
    const KickerOutput kickerOutput = kickerController.update(kickerInput, nowMs);

    if (output.state == kFault) operatorArm.reset();
    else clearFaultAction.reset();
    if (kDiagnosticMode) {
        motorMotion = {};
        motorEnabled = false;
    } else {
        motorMotion = output.motion;
        motorEnabled = input.armed && output.state != kFault;
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
                Serial.print("DIAG mode/state="); Serial.print(kDiagnosticMode ? "DIAG" : "TACTIC"); Serial.print('/');
                Serial.print(static_cast<uint8_t>(output.state)); Serial.print(" MOTOR_frames_no_ACK/errors=");
                Serial.print(motorFramesSent); Serial.print('/'); Serial.print(motorTxErrors);
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
