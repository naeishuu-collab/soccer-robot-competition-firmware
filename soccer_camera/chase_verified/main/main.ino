#include <Arduino.h>
#include <string.h>

// ============================================================
// MAIN - Direction / Power Controller
// RP2354B
// ============================================================

// RS485
constexpr uint8_t PIN_RS485_TX = 0;
constexpr uint8_t PIN_RS485_RX = 1;
constexpr uint8_t PIN_RS485_EN = 8;
constexpr uint32_t RS485_BAUD = 115200;

// Protocol
constexpr uint8_t FRAME_START = 0xA5;
constexpr uint8_t FRAME_SIZE = 12;

constexpr uint8_t MSG_POLL = 1;
constexpr uint8_t MSG_BALL = 2;
constexpr uint8_t MSG_DRIVE = 3;

constexpr uint32_t RX_INTERBYTE_TIMEOUT_US = 3000;

// Timing
constexpr uint32_t IR_REPLY_WINDOW_US = 10000;
constexpr uint32_t CONTROL_CYCLE_US = 20000;

// Movement
constexpr uint16_t MOVE_POWER = 180;
constexpr uint16_t MAX_POWER = 200;

static_assert(MOVE_POWER <= MAX_POWER,
              "MOVE_POWER exceeds MAX_POWER");

// IR orientation
constexpr int16_t IR_FRONT_DEGREES = 0;
constexpr int8_t IR_ANGLE_SIGN = 1;

// ============================================================
// CRC-7
// ============================================================

uint8_t calculateCRC(const uint8_t *frame) {
  uint8_t crc = 0;

  for (uint8_t i = 1; i < FRAME_SIZE - 1; ++i) {
    crc ^= frame[i];

    for (uint8_t bit = 0; bit < 7; ++bit) {
      crc = ((crc << 1) ^
             ((crc & 0x40) ? 0x09 : 0)) & 0x7F;
    }
  }

  return crc;
}

// ============================================================
// Frame receiver
// ============================================================

class FrameReceiver {
public:
  void reset() {
    used_ = 0;

    while (Serial1.available()) {
      Serial1.read();
    }
  }

  bool receive(uint8_t *output) {
    while (Serial1.available()) {
      const uint32_t now = micros();

      if (used_ > 0 &&
          now - lastByteUs_ > RX_INTERBYTE_TIMEOUT_US) {
        used_ = 0;
      }

      const uint8_t value = Serial1.read();
      lastByteUs_ = now;

      if (value == FRAME_START) {
        buffer_[0] = value;
        used_ = 1;
        continue;
      }

      if (value >= 0x80 || used_ == 0) {
        used_ = 0;
        continue;
      }

      buffer_[used_++] = value;

      if (used_ == FRAME_SIZE) {
        used_ = 0;

        if (calculateCRC(buffer_) ==
            buffer_[FRAME_SIZE - 1]) {
          memcpy(output, buffer_, FRAME_SIZE);
          return true;
        }
      }
    }

    return false;
  }

private:
  uint8_t buffer_[FRAME_SIZE] = {};
  uint8_t used_ = 0;
  uint32_t lastByteUs_ = 0;
};

FrameReceiver receiver;

// ============================================================
// Encoding
// ============================================================

void encodeValue(uint8_t *destination, uint16_t value) {
  destination[0] = (value >> 7) & 0x7F;
  destination[1] = value & 0x7F;
}

uint16_t decodeValue(const uint8_t *source) {
  return (uint16_t(source[0]) << 7) | source[1];
}

// ============================================================
// RS485 transmission
// ============================================================

void transmitFrame(uint8_t *frame) {
  frame[FRAME_SIZE - 1] = calculateCRC(frame);

  digitalWrite(PIN_RS485_EN, HIGH);
  delayMicroseconds(20);

  Serial1.write(frame, FRAME_SIZE);
  Serial1.flush();

  delayMicroseconds(20);
  digitalWrite(PIN_RS485_EN, LOW);
}

// ============================================================
// Direction conversion
// ============================================================

uint16_t convertDirection(uint16_t irAngle) {
  int32_t direction =
      IR_ANGLE_SIGN *
      (int32_t(irAngle) - IR_FRONT_DEGREES);

  direction %= 360;

  if (direction < 0) {
    direction += 360;
  }

  return uint16_t(direction);
}

// ============================================================
// DRIVE frame
// ============================================================

void createDriveFrame(uint8_t sequence,
                      uint16_t direction,
                      uint16_t power,
                      uint8_t frame[FRAME_SIZE]) {
  memset(frame, 0, FRAME_SIZE);

  frame[0] = FRAME_START;
  frame[1] = MSG_DRIVE;
  frame[2] = sequence & 0x7F;

  encodeValue(&frame[3], direction);
  encodeValue(&frame[5], power);
}

// ============================================================
// Setup
// ============================================================

void setup() {
  digitalWrite(PIN_RS485_EN, LOW);
  pinMode(PIN_RS485_EN, OUTPUT);
  digitalWrite(PIN_RS485_EN, LOW);

  Serial1.setTX(PIN_RS485_TX);
  Serial1.setRX(PIN_RS485_RX);
  Serial1.begin(RS485_BAUD);

  Serial.begin(115200);

  delay(10);
  receiver.reset();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  static uint8_t sequence = 0;
  static uint32_t lastLogMs = 0;

  const uint32_t cycleStart = micros();

  sequence = (sequence + 1) & 0x7F;

  // 1. POLL IR
  uint8_t poll[FRAME_SIZE] = {};

  poll[0] = FRAME_START;
  poll[1] = MSG_POLL;
  poll[2] = sequence;

  receiver.reset();
  transmitFrame(poll);

  // 2. Receive BALL
  const uint32_t replyStart = micros();

  bool received = false;
  uint16_t angle = 0;
  uint16_t strength = 0;

  uint8_t reply[FRAME_SIZE];

  while (micros() - replyStart < IR_REPLY_WINDOW_US) {
    if (!receiver.receive(reply)) {
      continue;
    }

    if (reply[1] != MSG_BALL ||
        reply[2] != sequence) {
      continue;
    }

    const uint16_t candidateAngle =
        decodeValue(&reply[3]);

    const uint16_t candidateStrength =
        decodeValue(&reply[5]);

    if (candidateAngle >= 360 ||
        candidateStrength > 1200) {
      continue;
    }

    received = true;
    angle = candidateAngle;
    strength = candidateStrength;
  }

  // 3. Decide movement direction and power
  uint16_t direction = 0;
  uint16_t power = 0;

  if (received && strength > 0) {
    direction = convertDirection(angle);
    power = MOVE_POWER;
  }

  // 4. Send direction/power to MD
  uint8_t drive[FRAME_SIZE];

  createDriveFrame(
    sequence,
    direction,
    power,
    drive
  );

  transmitFrame(drive);

  // 5. Debug
  const uint32_t nowMs = millis();

  if (nowMs - lastLogMs >= 250 &&
      Serial &&
      Serial.availableForWrite() >= 64) {
    lastLogMs = nowMs;

    Serial.printf(
      "IR=%s angle=%u strength=%u "
      "DIR=%u POWER=%u\n",
      !received ? "timeout" :
      strength > 0 ? "ball" : "none",
      unsigned(angle),
      unsigned(strength),
      unsigned(direction),
      unsigned(power)
    );
  }

  // 6. Maintain cycle
  while (micros() - cycleStart < CONTROL_CYCLE_US) {
    delayMicroseconds(50);
  }
}
