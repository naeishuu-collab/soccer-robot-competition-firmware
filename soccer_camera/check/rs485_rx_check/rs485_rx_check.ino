// 既定: MOTOR基板 (GP0 TX, GP1 RX, GP2 DE)。使用前に物理DE経路を確認すること。
constexpr uint8_t kRs485TxPin = 0;
constexpr uint8_t kRs485RxPin = 1;
constexpr uint8_t kRs485DePin = 2;
constexpr uint32_t kBaud = 115200;
constexpr size_t kProbeLength = 10;
static_assert(kProbeLength == 10, "Must match the TX checker probe format");

char rxFrame[kProbeLength];
size_t rxLength = 0;
bool rxOverflow = false;

bool isHexDigit(char value) {
    return (value >= '0' && value <= '9') || (value >= 'A' && value <= 'F');
}

bool isTestProbe() {
    return rxLength == kProbeLength && rxFrame[0] == 'R' && rxFrame[1] == 'S' &&
           rxFrame[2] == '4' && rxFrame[3] == '8' && rxFrame[4] == '5' && rxFrame[5] == ',' &&
           isHexDigit(rxFrame[6]) && isHexDigit(rxFrame[7]) &&
           rxFrame[8] == '\r' && rxFrame[9] == '\n';
}

void echoProbe() {
    delayMicroseconds(200);  // 送信側が共有ハーフデュプレックスバスを解放するのを待つ。
    digitalWrite(kRs485DePin, HIGH);
    delayMicroseconds(20);
    Serial1.write(reinterpret_cast<const uint8_t*>(rxFrame), rxLength);
    Serial1.flush();
    delayMicroseconds(20);
    digitalWrite(kRs485DePin, LOW);
}

void pollRs485() {
    while (Serial1.available()) {
        const int value = Serial1.read();
        if (value < 0) break;
        const char byte = static_cast<char>(value);

        if (byte == '\n') {
            if (!rxOverflow && rxLength < sizeof(rxFrame)) {
                rxFrame[rxLength++] = byte;
                if (isTestProbe()) {
                    echoProbe();
                    if (Serial && Serial.availableForWrite() >= static_cast<int>(rxLength + 32)) {
                        Serial.print("RX: ");
                        Serial.write(reinterpret_cast<const uint8_t*>(rxFrame), rxLength);
                        Serial.println("ECHO sent");
                    }
                } else {
                    if (Serial && Serial.availableForWrite() >= 48)
                        Serial.println("RX ignored: not an RS485 test probe");
                }
            } else {
                if (Serial && Serial.availableForWrite() >= 32)
                    Serial.println("RX ignored: frame too long");
            }
            rxLength = 0;
            rxOverflow = false;
        } else if (!rxOverflow) {
            if (rxLength < sizeof(rxFrame)) rxFrame[rxLength++] = byte;
            else rxOverflow = true;
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
    if (Serial && Serial.availableForWrite() >= 56)
        Serial.println("RS485 RX/echo check; listening for RS485,HH probe");
}

void loop() {
    pollRs485();
}
