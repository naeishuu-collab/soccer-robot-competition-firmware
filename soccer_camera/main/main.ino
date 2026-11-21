#include <Arduino.h>
#include <string.h>
#include <math.h>

struct CameraState;

// BEGIN HOST-TESTED CAMERA LOGIC
struct CameraState {
  uint8_t mode = 0;
  int8_t x = 0;
  int8_t y = 0;
  uint32_t receivedMs = 0;
  bool received = false;
};

inline bool validCameraFrame(const uint8_t *f) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < 7; ++i) crc ^= f[i];
  if (f[0] != 0x20 || f[7] != crc) return false;
  for (uint8_t i = 1; i <= 3; ++i)
    if (f[i] > 100 && f[i] < 156) return false;
  const uint8_t mode = f[5];
  if (mode > 4 || f[6] != (mode == 1 ? 0x21 : 0x20) || f[4] != (mode == 4)) return false;
  if (mode == 1) return f[3] == 0;
  if (mode == 3) return f[1] == 0 && f[2] == 0;
  return f[1] == 0 && f[2] == 0 && f[3] == 0;
}

inline bool cameraAllowsMotion(const CameraState &camera, uint32_t nowMs) {
  return camera.received && uint32_t(nowMs - camera.receivedMs) <= 200 && camera.mode != 0;
}

inline uint16_t cameraAvoidDirection(int8_t x, int8_t y) {
  float angle = atan2f(float(x), float(y)) * 57.2957795f;
  if (angle < 0) angle += 360;
  return uint16_t(angle + 0.5f) % 360;
}

inline void chooseChaseMotion(const CameraState &camera, uint32_t nowMs, bool ballFound,
                              uint16_t ballDirection, uint16_t movePower,
                              uint16_t &direction, uint16_t &power) {
  direction = 0;
  power = 0;
  if (!ballFound || ballDirection >= 360 || !cameraAllowsMotion(camera, nowMs)) return;
  if (camera.mode == 1) {
    if (camera.x == 0 && camera.y == 0) return;
    direction = cameraAvoidDirection(camera.x, camera.y);
  } else {
    direction = ballDirection;
  }
  power = movePower;
}
// END HOST-TESTED CAMERA LOGIC

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

CameraState camera;
uint8_t cameraFrame[8] = {};
uint8_t cameraUsed = 0;
uint32_t cameraLastByteMs = 0;

void receiveCamera() {
  const uint32_t now = millis();
  if (cameraUsed && uint32_t(now - cameraLastByteMs) >= 20) cameraUsed = 0;
  while (Serial2.available()) {
    const uint8_t byte = Serial2.read();
    if (!cameraUsed && byte != 0x20) continue;
    cameraLastByteMs = now;
    cameraFrame[cameraUsed++] = byte;
    if (cameraUsed != sizeof(cameraFrame)) continue;
    if (validCameraFrame(cameraFrame)) {
      camera.mode = cameraFrame[5];
      camera.x = int8_t(cameraFrame[1]);
      camera.y = int8_t(cameraFrame[2]);
      camera.receivedMs = now;
      camera.received = true;
      cameraUsed = 0;
    } else {
      uint8_t next = 1;
      while (next < sizeof(cameraFrame) && cameraFrame[next] != 0x20) ++next;
      cameraUsed = sizeof(cameraFrame) - next;
      memmove(cameraFrame, cameraFrame + next, cameraUsed);
    }
  }
}

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
                      uint8_t frame[]) {
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

  Serial2.setTX(2);
  Serial2.setRX(3);
  Serial2.begin(9600);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);

  Serial.begin(115200);

  receiver.reset();
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  static uint8_t sequence = 0;
  static uint32_t lastLogMs = 0;

  const uint32_t cycleStart = micros();

  receiveCamera();

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
    // Allow the IR transceiver to release the shared RS485 bus.
    delayMicroseconds(40);
    break;
  }

  // 3. Decide movement direction and power
  uint16_t direction = 0;
  uint16_t power = 0;

  receiveCamera();
  chooseChaseMotion(camera, millis(), received && strength > 0, convertDirection(angle),
                    MOVE_POWER, direction, power);

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
