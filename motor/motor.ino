#include <Arduino.h>
#include <hardware/gpio.h>
#include <BoardConfig.h>
#include <RobotControl.h>
#include <Rs485.h>

using namespace soccer;

constexpr uint32_t kCommandTimeoutMs = 100;
constexpr uint32_t kMotorPeriodUs = 5000;
constexpr float kMaxWheelStepPerTick = 0.08f; // shortcut: initial ramp; tune against the loaded robot before enabling.

// User-reported current-revision MOTOR PWM routing; physical output remains gated by the unverified safety config.
constexpr WheelConfig kWheels[4] = {};
constexpr Tb67h450ChannelConfig kDriver[4] = {
    {13, 12, kTb67h450MaxDuty}, // Left front, J2
    {11, 10, kTb67h450MaxDuty}, // Right front, J6
    {9, 8, kTb67h450MaxDuty},   // Left rear, J7
    {7, 6, kTb67h450MaxDuty}    // Right rear, J1
};
constexpr bool kWheelsConfigured = false;
constexpr bool kDriverTableConfigured = true;
constexpr int8_t kMotorSafetyPins[3] = {board::kMotorArmPin, board::kMotorEStopPin, board::kMotorClearFaultPin};
constexpr int8_t kMotorSafetyActiveLevels[3] = {board::kMotorArmActiveLevel, board::kMotorEStopActiveLevel,
                                                board::kMotorClearFaultActiveLevel};
MotorSafetyGate safetyGate;
MotorCommandPriority commandPriority;
MotorWheelSlew wheelSlew[4];
Tb67h450Mode appliedMode[4]{};
uint32_t neutralSinceMs[4]{};
bool neutralSinceValid[4]{};
ManualActionLatch clearFaultAction;

bool motorRs485PinsReady() {
    return validRs485PinConfig(board::kMotorRs485PinsVerified, board::kMotorGpioCount,
                               board::kMotorRs485TxPin, board::kMotorRs485RxPin, board::kMotorRs485DePin);
}

bool motorSafetyInputsReady() {
    return validSafetyInputConfig(kMotorSafetyPins, kMotorSafetyActiveLevels,
                                  board::kMotorSafetyInputBiasVerified,
                                  board::kMotorSafetyReservedPinMask, board::kMotorGpioCount);
}

bool motorSafetyInputActive(uint8_t index) {
    return digitalRead(kMotorSafetyPins[index]) == kMotorSafetyActiveLevels[index];
}

bool motorHardwareReady() {
    return motorSafetyInputsReady() &&
           validMotorHardwareConfig(board::kMotorHardwareConfigured, kWheelsConfigured,
                                    kDriverTableConfigured, board::kMotorPwmFrequencyHz,
                                    kMotorPwmRange, F_CPU, kWheels, kDriver, board::kMotorPwmPinMask,
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
PacketParser parser;
SequenceTracker sequenceTracker;
Motion command{};
bool commandEnabled = false;
bool hasCommand = false;
bool hadValidCommand = false;
uint32_t lastCommandMs = 0;
uint32_t lastTickUs = 0;
uint32_t lastDebugMs = 0;
uint32_t commandTimeouts = 0;
uint16_t rejectedPackets = 0;
float wheelOutput[4] = {};

uint64_t driverPinMask(const Tb67h450ChannelConfig& channel) {
    return (uint64_t{1} << static_cast<uint8_t>(channel.pinIn1)) |
           (uint64_t{1} << static_cast<uint8_t>(channel.pinIn2));
}

void selectDriverSioLow(const Tb67h450ChannelConfig& channel) {
    gpio_put_masked64(driverPinMask(channel), 0);
    gpio_set_function(channel.pinIn1, GPIO_FUNC_SIO);
    gpio_set_function(channel.pinIn2, GPIO_FUNC_SIO);
}

void writeTb67h450Stop(uint8_t index, uint32_t nowMs) {
    if (appliedMode[index] != kTb67h450Stop || !neutralSinceValid[index]) {
        neutralSinceMs[index] = nowMs;
        neutralSinceValid[index] = true;
    }
    const Tb67h450ChannelConfig& channel = kDriver[index];
    if (appliedMode[index] == kTb67h450Brake) selectDriverSioLow(channel);
    analogWrite(channel.pinIn1, 0);
    analogWrite(channel.pinIn2, 0);
    appliedMode[index] = kTb67h450Stop;
}

void writeSafeStop() {
    const uint32_t nowMs = millis();
    const bool hardwareReady = motorHardwareReady();
    for (uint8_t i = 0; i < 4; ++i) {
        wheelOutput[i] = 0.0f;
        wheelSlew[i].reset();
        if (hardwareReady) writeTb67h450Stop(i, nowMs);
        else {
            appliedMode[i] = kTb67h450Stop;
            if (!neutralSinceValid[i]) neutralSinceMs[i] = nowMs;
            neutralSinceValid[i] = true;
        }
    }
}

void writeTb67h450(uint8_t index, const Tb67h450Output& requested, uint32_t nowMs) {
    const Tb67h450ChannelConfig& channel = kDriver[index];
    Tb67h450Output output = requested;
    if (output.mode == kTb67h450Stop) {
        writeTb67h450Stop(index, nowMs);
        return;
    }
    if (output.mode != appliedMode[index]) {
        if (appliedMode[index] != kTb67h450Stop || !neutralSinceValid[index] ||
            static_cast<uint32_t>(nowMs - neutralSinceMs[index]) < kTb67h450ReversalDwellMs) {
            writeTb67h450Stop(index, nowMs);
            return;
        }
    }
    if (output.mode == kTb67h450Brake && appliedMode[index] == kTb67h450Brake) return;
    switch (output.mode) {
        case kTb67h450Forward:
            analogWrite(channel.pinIn2, 0);
            analogWrite(channel.pinIn1, output.in1);
            break;
        case kTb67h450Reverse:
            analogWrite(channel.pinIn1, 0);
            analogWrite(channel.pinIn2, output.in2);
            break;
        case kTb67h450Brake:
            selectDriverSioLow(channel);
            gpio_put_masked64(driverPinMask(channel), driverPinMask(channel));
            break;
        case kTb67h450Stop:
            break;
        default:
            safetyGate.tripFault();
            writeTb67h450Stop(index, nowMs);
            return;
    }
    appliedMode[index] = output.mode;
    neutralSinceValid[index] = false;
}

void writeWheels(uint32_t nowMs) {
    if (!motorHardwareReady()) return;
    for (uint8_t i = 0; i < 4; ++i) {
        Tb67h450Output output{};
        if (!tb67h450OutputForWheel(wheelOutput[i], kDriver[i].maxDuty, output)) {
            safetyGate.tripFault();
            writeSafeStop();
            return;
        }
        writeTb67h450(i, output, nowMs);
    }
}

void receiveCommands(uint32_t nowMs) {
    if (expireMotorCommand(safetyGate, hadValidCommand, nowMs, lastCommandMs, kCommandTimeoutMs)) {
        ++commandTimeouts;
        hasCommand = false;
        commandEnabled = false;
        hadValidCommand = false;
        commandPriority.latchStop();
        sequenceTracker.resetPeer(); // MAIN may restart its sequence after the local command timeout.
        writeSafeStop();
    }
    uint8_t byte = 0;
    uint8_t consumed = 0;
    while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
        Packet packet{};
        const ParserStats before = parser.stats();
        const bool ready = parser.feed(byte, nowMs, packet);
        const ParserStats& after = parser.stats();
        if (after.crcErrors != before.crcErrors || after.formatErrors != before.formatErrors ||
            after.timeouts != before.timeouts) {
            ++rejectedPackets;
            hasCommand = false;
            commandEnabled = false;
            commandPriority.latchStop();
            safetyGate.tripFault();
            writeSafeStop();
            continue;
        }
        if (!ready) continue;
        if (packet.type != kMotorCommand) continue;
        Motion next{};
        bool enable = false;
        if (!sequenceTracker.accept(packet.sequence) || !decodeMotorPacket(packet, next, enable)) {
            ++rejectedPackets;
            hasCommand = false;
            commandEnabled = false;
            commandPriority.latchStop();
            safetyGate.tripFault();
            writeSafeStop();
            continue;
        }
        if (!commandPriority.accept(next, enable)) continue;
        command = next;
        commandEnabled = enable;
        hasCommand = true;
        hadValidCommand = true;
        lastCommandMs = nowMs;
    }
    if (parser.poll(nowMs)) {
        ++rejectedPackets;
        hasCommand = false;
        commandEnabled = false;
        commandPriority.latchStop();
        safetyGate.tripFault();
        writeSafeStop();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("BOOT MOTOR; driver output gate=" "disabled until measured config is explicitly enabled");
    if (motorRs485PinsReady()) {
        Serial1.setTX(board::kMotorRs485TxPin);
        Serial1.setRX(board::kMotorRs485RxPin);
        Serial1.begin(115200);
        pinMode(board::kMotorRs485DePin, OUTPUT);
        digitalWrite(board::kMotorRs485DePin, LOW);
    }
    if (motorSafetyInputsReady())
        for (int8_t pin : kMotorSafetyPins) pinMode(pin, INPUT);
    if (motorHardwareReady()) {
        analogWriteRange(kMotorPwmRange);
        analogWriteFreq(board::kMotorPwmFrequencyHz);
        for (uint8_t i = 0; i < 4; ++i) {
            digitalWrite(kDriver[i].pinIn1, LOW);
            digitalWrite(kDriver[i].pinIn2, LOW);
            pinMode(kDriver[i].pinIn1, OUTPUT);
            pinMode(kDriver[i].pinIn2, OUTPUT);
            analogWrite(kDriver[i].pinIn1, 0);
            analogWrite(kDriver[i].pinIn2, 0);
            appliedMode[i] = kTb67h450Stop;
            neutralSinceMs[i] = millis();
            neutralSinceValid[i] = true;
        }
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
    const bool fresh = hasCommand && !motorCommandExpired(nowMs, lastCommandMs, kCommandTimeoutMs);
    const bool safetyInputsReady = motorSafetyInputsReady();
    const bool emergencyStop = !safetyInputsReady || motorSafetyInputActive(1);
    bool clearFaultRequested = false;
    if (safetyGate.state() == kMotorFault) {
        clearFaultRequested = clearFaultAction.update(safetyInputsReady,
                                                       safetyInputsReady && motorSafetyInputActive(2),
                                                       emergencyStop);
    } else {
        clearFaultAction.reset();
    }
    const MotorSafetyState previousSafetyState = safetyGate.state();
    const MotorSafetyInput safetyInput{
        safetyInputsReady,
        motorHardwareReady(),
        safetyInputsReady && motorSafetyInputActive(0),
        emergencyStop,
        clearFaultRequested,
        fresh,
        commandEnabled,
        false
    };
    const bool driveAllowed = safetyGate.update(safetyInput);
    if (previousSafetyState == kMotorFault && safetyGate.state() == kMotorDisarmed)
        clearFaultAction.reset();
    if (!driveAllowed) {
        writeSafeStop(); // A local timeout or a disabled/unconfigured system clears the last drive command.
    } else if (!safeMotorUpdate(command, true, true, kWheels, wheelSlew, wheelOutput,
                                kMaxWheelStepPerTick, nowMs, kTb67h450ReversalDwellMs)) {
        ++rejectedPackets;
        safetyGate.tripFault();
        writeSafeStop();
    } else {
        writeWheels(nowMs);
    }
    commandPriority.completeOutputTick();
    if (static_cast<uint32_t>(nowMs - lastDebugMs) >= 1000) {
        lastDebugMs = nowMs;
        if (Serial && Serial.availableForWrite() >= 140) {
            Serial.print("safety="); Serial.print(safetyGate.state());
            Serial.print(" output_ready="); Serial.print(motorHardwareReady());
            Serial.print(" armed="); Serial.print(driveAllowed);
            Serial.print(" crc="); Serial.print(parser.stats().crcErrors);
            Serial.print(" seq_gap="); Serial.print(sequenceTracker.gaps());
            Serial.print(" seq_reject="); Serial.print(sequenceTracker.rejected());
            Serial.print(" frame_timeout/command_timeout=");
            Serial.print(parser.stats().timeouts); Serial.print('/'); Serial.print(commandTimeouts);
            const Rs485Diagnostics& bus = rs485.diagnostics();
            Serial.print(" RS485_rx/busy/errors="); Serial.print(bus.rxBytes); Serial.print('/');
            Serial.print(bus.txBusy); Serial.print('/'); Serial.print(bus.rxErrors + bus.txErrors);
            Serial.print(" rejected="); Serial.println(rejectedPackets);
        }
    }
}
