// 既定: MAIN基板 (GP0 TX, GP1 RX, GP8 DE)。使用前に物理DE経路を確認すること。
constexpr uint8_t kRs485TxPin = 0;
constexpr uint8_t kRs485RxPin = 1;
constexpr uint8_t kRs485DePin = 8;
constexpr uint32_t kBaud = 115200;
constexpr uint32_t kProbePeriodMs = 1000;
constexpr uint32_t kEchoTimeoutMs = 500;
constexpr size_t kFrameLength = sizeof("RS485,00\r\n") - 1;
static_assert(kFrameLength == 10, "Update the RX checker if the probe format changes");

const char kHexDigits[] = "0123456789ABCDEF";
char expectedFrame[] = "RS485,00\r\n";
uint8_t nextSequence = 0;
size_t echoIndex = 0;
uint32_t nextProbeMs = 0;
uint32_t echoDeadlineMs = 0;
bool testRunning = false;
bool waitingForEcho = false;

void sendProbe(uint32_t nowMs) {
    expectedFrame[6] = kHexDigits[nextSequence >> 4];
    expectedFrame[7] = kHexDigits[nextSequence & 0x0F];
    ++nextSequence;

    digitalWrite(kRs485DePin, HIGH);
    delayMicroseconds(20);
    Serial1.write(reinterpret_cast<const uint8_t*>(expectedFrame), kFrameLength);
    Serial1.flush();
    delayMicroseconds(20);
    digitalWrite(kRs485DePin, LOW);

    echoIndex = 0;
    waitingForEcho = true;
    echoDeadlineMs = nowMs + kEchoTimeoutMs;
    if (Serial && Serial.availableForWrite() >= static_cast<int>(kFrameLength + 5)) {
        Serial.print("TX: ");
        Serial.write(reinterpret_cast<const uint8_t*>(expectedFrame), kFrameLength);
    }
}

void pollUsbCommands() {
    while (Serial.available()) {
        const char command = static_cast<char>(Serial.read());
        if (command == 'T' || command == 't') {
            testRunning = true;
            waitingForEcho = false;
            echoIndex = 0;
            nextProbeMs = millis();
            if (Serial && Serial.availableForWrite() >= 32)
                Serial.println("RS485 test started; S stops");
        } else if (command == 'S' || command == 's') {
            testRunning = false;
            waitingForEcho = false;
            echoIndex = 0;
            if (Serial && Serial.availableForWrite() >= 24)
                Serial.println("RS485 test stopped");
        }
    }
}

void pollEcho(uint32_t nowMs) {
    while (Serial1.available()) {
        const int value = Serial1.read();
        if (value < 0 || !waitingForEcho) continue;

        if (value == static_cast<uint8_t>(expectedFrame[echoIndex])) {
            ++echoIndex;
        } else {
            echoIndex = value == static_cast<uint8_t>(expectedFrame[0]) ? 1 : 0;
        }

        if (echoIndex == kFrameLength) {
            waitingForEcho = false;
            echoIndex = 0;
            if (Serial && Serial.availableForWrite() >= 64)
                Serial.println("PASS: exact RS485 echo received; TX and RX paths work");
            nextProbeMs = nowMs + kProbePeriodMs;
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(kRs485DePin, OUTPUT);
    digitalWrite(kRs485DePin, LOW);
    Serial1.setTX(kRs485TxPin);
    Serial1.setRX(kRs485RxPin);
    Serial1.begin(kBaud);
    if (Serial && Serial.availableForWrite() >= 48)
        Serial.println("RS485 TX/echo check; send T to start, S to stop");
}

void loop() {
    pollUsbCommands();
    const uint32_t nowMs = millis();
    pollEcho(nowMs);

    if (waitingForEcho && static_cast<int32_t>(nowMs - echoDeadlineMs) >= 0) {
        waitingForEcho = false;
        echoIndex = 0;
        if (Serial && Serial.availableForWrite() >= 32)
            Serial.println("FAIL: echo timeout");
        nextProbeMs = nowMs + kProbePeriodMs;
    }

    if (testRunning && !waitingForEcho && static_cast<int32_t>(nowMs - nextProbeMs) >= 0)
        sendProbe(nowMs);
}
