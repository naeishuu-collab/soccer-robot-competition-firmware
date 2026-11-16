#include <Arduino.h>
#include <BoardConfig.h>
#include <RobotControl.h>
#include <Rs485.h>

using namespace soccer;

constexpr uint32_t kSamplePeriodMs = 20;
constexpr uint32_t kRequestTimeoutMs = 100;
constexpr uint32_t kDiagnosticPeriodMs = 500;

int rs485Available(void*) { return Serial1.available(); }
int rs485Read(void*) { return Serial1.read(); }
void rs485Direction(void*, bool tx) { digitalWrite(board::kLineRs485DePin, tx ? HIGH : LOW); }
size_t rs485Write(void*, const uint8_t* bytes, size_t length) { return Serial1.write(bytes, length); }
void rs485Flush(void*) { Serial1.flush(); }
void rs485Delay(void*, uint32_t us) { delayMicroseconds(us); }
Rs485Io rs485Io{&Serial1, rs485Available, rs485Read, rs485Direction, rs485Write, rs485Flush, rs485Delay};
Rs485Driver rs485{rs485Io};
PacketParser parser;
SequenceTracker requestSequence;

uint32_t lastScanMs = 0;
uint32_t lastDiagnosticMs = 0;
uint32_t rawState = 0;
uint32_t filteredState = 0;
LineDebouncer lineDebouncer;
uint32_t sampleDurationUs = 0;
uint32_t lastRequestMs = 0;
uint32_t rejectedRequests = 0;
uint32_t responseErrors = 0;
bool requestSeen = false;

LineObservation currentLine() {
    return calculateLine(filteredState, board::kLinePolarityConfigured, board::kLineDetectHigh,
                         board::kLineSensorAngleCdeg,
                         board::kLineAnglesConfigured ? board::kLineSensorAngleValidMask : 0,
                         board::kLineSensorEnabledMask);
}

void sendLineResponse(const Packet& request) {
    if (requestSeen && static_cast<uint32_t>(millis() - lastRequestMs) > kRequestTimeoutMs)
        requestSequence.resetPeer();
    if (!decodeLineRequest(request) || !requestSequence.accept(request.sequence)) {
        ++rejectedRequests;
        return;
    }
    requestSeen = true;
    lastRequestMs = millis();
    const Packet response = makeLinePacket(currentLine(), request.sequence);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(response, bytes, sizeof(bytes));
    if (!length || !rs485.send(bytes, length, micros(), true)) ++responseErrors;
}

void receiveRequests(uint32_t nowMs) {
    uint8_t byte = 0;
    uint8_t consumed = 0;
    while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
        Packet packet{};
        const ParserStats before = parser.stats();
        const bool ready = parser.feed(byte, nowMs, packet);
        const ParserStats& after = parser.stats();
        if (after.crcErrors != before.crcErrors || after.formatErrors != before.formatErrors ||
            after.timeouts != before.timeouts) {
            ++rejectedRequests;
            continue;
        }
        if (ready && packet.type == kLineRequest) sendLineResponse(packet);
    }
    if (parser.poll(nowMs)) ++rejectedRequests;
}

void printRawLevels() {
    Serial.print("raw[0..23]=");
    for (uint8_t i = 0; i < 24; ++i) Serial.print((rawState & (1u << i)) ? 'H' : 'L');
}

void setup() {
    Serial.begin(115200);
    for (uint8_t pin : board::kLineSensorPins) pinMode(pin, INPUT);
    pinMode(board::kLineRs485DePin, OUTPUT);
    digitalWrite(board::kLineRs485DePin, LOW);
    Serial1.setTX(board::kLineRs485TxPin);
    Serial1.setRX(board::kLineRs485RxPin);
    Serial1.begin(115200);
    Serial.println("BOOT LINE; response only to CRC-protected MAIN poll");
}

void loop() {
    const uint32_t now = millis();
    receiveRequests(now);
    if (static_cast<uint32_t>(now - lastScanMs) < kSamplePeriodMs) return;
    lastScanMs = now;

    const uint32_t startedUs = micros();
    rawState = 0;
    for (uint8_t i = 0; i < 24; ++i) {
        if (digitalRead(board::kLineSensorPins[i]) == HIGH) rawState |= (1u << i);
    }
    sampleDurationUs = micros() - startedUs;
    const uint32_t enabledState = rawState & board::kLineSensorEnabledMask;
    filteredState = board::kLineDebounceConfigured
                        ? lineDebouncer.update(enabledState, board::kLineDebounceSamples)
                        : enabledState;
    const LineObservation line = currentLine();
    if (static_cast<uint32_t>(now - lastDiagnosticMs) >= kDiagnosticPeriodMs) {
        lastDiagnosticMs = now;
        if (Serial && Serial.availableForWrite() >= 128) {
            printRawLevels();
            Serial.print(" count="); Serial.print(line.count);
            Serial.print(" detected="); Serial.print(line.detected);
            Serial.print(" direction_valid="); Serial.print(line.directionValid);
            Serial.print(" ambiguous="); Serial.print(line.ambiguous);
            Serial.print(" angle_cdeg="); Serial.print(line.angleCdeg);
            Serial.print(" polarity="); Serial.print(line.polarityValid ? (line.activeHigh ? "HIGH" : "LOW") : "UNKNOWN");
            Serial.print(" sample_us="); Serial.print(sampleDurationUs);
            const Rs485Diagnostics& bus = rs485.diagnostics();
            Serial.print(" RS485 tx/rx/busy/errors/crc/timeout/reject=");
            Serial.print(bus.txFrames); Serial.print('/'); Serial.print(bus.rxBytes); Serial.print('/');
            Serial.print(bus.txBusy); Serial.print('/'); Serial.print(bus.txErrors + responseErrors); Serial.print('/');
            Serial.print(parser.stats().crcErrors); Serial.print('/'); Serial.print(parser.stats().timeouts); Serial.print('/');
            Serial.println(rejectedRequests);
        }
    }
}
