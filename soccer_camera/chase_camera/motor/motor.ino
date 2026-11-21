#include <Arduino.h>
#include <math.h>
#include <string.h>

// ============================================================
// MOTOR - RP2350 / TB67H450 x 4
//
// Channel order:
// [0] M4 Right Rear  GPIO6,7
// [1] M1 Left Front  GPIO12,13
// [2] M2 Right Front GPIO11,10
// [3] M3 Left Rear   GPIO8,9
//
// Direction:
// Front=0, Right=90, Back=180, Left=270
//
// RS485: MAIN-compatible DRIVE
// ============================================================

// RS485
constexpr uint8_t RS485_RX_PIN = 1;
constexpr uint8_t RS485_EN_PIN = 2;
constexpr uint32_t RS485_BAUD = 115200;

// Motor pins
const uint8_t MOTOR_IN1[4] = {6, 12, 11, 8};
const uint8_t MOTOR_IN2[4] = {7, 13, 10, 9};

// PWM
constexpr bool OUTPUTS_ENABLED = true;
constexpr uint32_t PWM_FREQ = 20000;
constexpr int16_t PWM_MAX = 210;
constexpr int16_t MIN_RUNNING_DUTY = 150;

// Provisional drive-axis angles.
// These require physical verification.
//
// Assumed original positional mapping:
// Right Rear  = 45
// Left Rear   = 135
// Left Front  = 225
// Right Front = 315
//
// Reordered to GPIO channel order.
constexpr int16_t WHEEL_DEGREES[4] = {
  45, 225, 315, 135
};

// Motor polarity: provisional.
constexpr int8_t WHEEL_SIGN[4] = {
  1, 1, 1, 1
};

// Timing
constexpr uint32_t COMMAND_TIMEOUT_MS = 100;
constexpr uint32_t RAMP_INTERVAL_MS = 10;
constexpr int16_t RAMP_STEP = 5;
constexpr uint32_t REVERSE_PAUSE_MS = 200;

// RS485 protocol
constexpr uint8_t FRAME_START = 0xA5;
constexpr uint8_t FRAME_SIZE = 12;

constexpr uint8_t MSG_POLL = 1;
constexpr uint8_t MSG_BALL = 2;
constexpr uint8_t MSG_DRIVE = 3;

constexpr uint32_t RX_TIMEOUT_US = 3000;

// Debug
constexpr uint32_t DEBUG_INTERVAL_MS = 1000;

// State
int16_t targetPWM[4] = {};
int16_t currentPWM[4] = {};

uint32_t reverseUntil[4] = {};

uint32_t lastCommandMs = 0;
uint32_t lastRampMs = 0;
uint32_t lastDebugMs = 0;

uint32_t receivedFrames = 0;
uint32_t crcErrors = 0;
uint32_t timeoutEvents = 0;
uint32_t invalidFrames = 0;

uint8_t lastSequence = 0;

uint16_t receivedDirection = 0;
uint16_t receivedPower = 0;

bool commandReceived = false;
bool timedOut = true;

static_assert(MIN_RUNNING_DUTY <= PWM_MAX,
              "Invalid PWM range");

// ============================================================
// CRC-7
// ============================================================

uint8_t frameCrc(const uint8_t *frame) {
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
  bool receive(uint8_t *output) {
    while (Serial1.available() > 0) {
      const uint32_t now = micros();

      if (used > 0 &&
          now - lastByteUs > RX_TIMEOUT_US) {
        used = 0;
      }

      const uint8_t value = Serial1.read();
      lastByteUs = now;

      if (value == FRAME_START) {
        buffer[0] = value;
        used = 1;
        continue;
      }

      if (value >= 0x80 || used == 0) {
        used = 0;
        continue;
      }

      buffer[used++] = value;

      if (used == FRAME_SIZE) {
        used = 0;

        if (frameCrc(buffer) ==
            buffer[FRAME_SIZE - 1]) {
          memcpy(output, buffer, FRAME_SIZE);
          return true;
        }

        ++crcErrors;
      }
    }

    return false;
  }

private:
  uint8_t buffer[FRAME_SIZE] = {};
  uint8_t used = 0;
  uint32_t lastByteUs = 0;
};

FrameReceiver receiver;

// ============================================================
// Helpers
// ============================================================

int8_t signOf(int16_t value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

int16_t clampPWM(int16_t value) {
  return constrain(value, -PWM_MAX, PWM_MAX);
}

uint16_t unpackValue(const uint8_t *data) {
  return (uint16_t(data[0]) << 7) | data[1];
}

// ============================================================
// Omni-wheel mixing
// ============================================================

void calculateWheelPWM(uint16_t direction,
                       uint16_t power,
                       int16_t output[4]) {
  for (uint8_t i = 0; i < 4; ++i) {
    output[i] = 0;
  }

  if (power == 0) {
    return;
  }

  const float requestedPower =
      float(min(power, uint16_t(PWM_MAX)));

  float projection[4];
  float maxMagnitude = 0.0f;

  for (uint8_t i = 0; i < 4; ++i) {
    const float radians =
        (int(direction) - WHEEL_DEGREES[i]) *
        DEG_TO_RAD;

    projection[i] =
        cosf(radians) * WHEEL_SIGN[i];

    const float magnitude = fabsf(projection[i]);

    if (magnitude > maxMagnitude) {
      maxMagnitude = magnitude;
    }
  }

  if (maxMagnitude < 0.001f) {
    return;
  }

  const float scale =
      requestedPower / maxMagnitude;

  for (uint8_t i = 0; i < 4; ++i) {
    const int16_t duty = int16_t(
        lroundf(projection[i] * scale)
    );

    if (duty > 0) {
      output[i] = constrain(
          duty, MIN_RUNNING_DUTY, PWM_MAX);

    } else if (duty < 0) {
      output[i] = constrain(
          duty, -PWM_MAX, -MIN_RUNNING_DUTY);

    } else {
      output[i] = 0;
    }
  }
}

// ============================================================
// Motor output
// ============================================================

void applyMotor(uint8_t motor, int16_t duty) {
  duty = clampPWM(duty);

  if (!OUTPUTS_ENABLED) {
    analogWrite(MOTOR_IN1[motor], 0);
    analogWrite(MOTOR_IN2[motor], 0);
    return;
  }

  if (duty > 0) {
    analogWrite(MOTOR_IN2[motor], 0);
    analogWrite(MOTOR_IN1[motor], duty);

  } else if (duty < 0) {
    analogWrite(MOTOR_IN1[motor], 0);
    analogWrite(MOTOR_IN2[motor], -duty);

  } else {
    analogWrite(MOTOR_IN1[motor], 0);
    analogWrite(MOTOR_IN2[motor], 0);
  }
}

void applyAllMotors() {
  for (uint8_t i = 0; i < 4; ++i) {
    applyMotor(i, currentPWM[i]);
  }
}

// ============================================================
// Receive DRIVE
// ============================================================

void handleDrive(const uint8_t *frame) {
  if (frame[1] == MSG_POLL ||
      frame[1] == MSG_BALL) {
    return;
  }

  if (frame[1] != MSG_DRIVE) {
    ++invalidFrames;
    return;
  }

  const uint16_t direction =
      unpackValue(&frame[3]);

  const uint16_t power =
      unpackValue(&frame[5]);

  if (direction >= 360 || power > PWM_MAX) {
    ++invalidFrames;
    return;
  }

  for (uint8_t i = 7; i <= 10; ++i) {
    if (frame[i] != 0) {
      ++invalidFrames;
      return;
    }
  }

  int16_t nextTarget[4];

  calculateWheelPWM(
      direction,
      power,
      nextTarget
  );

  for (uint8_t i = 0; i < 4; ++i) {
    targetPWM[i] = nextTarget[i];
  }

  receivedDirection = direction;
  receivedPower = power;

  lastSequence = frame[2];
  lastCommandMs = millis();

  commandReceived = true;
  timedOut = false;

  ++receivedFrames;
}

// ============================================================
// Communication timeout
// ============================================================

void checkTimeout() {
  if (!commandReceived || timedOut) {
    return;
  }

  if (millis() - lastCommandMs >=
      COMMAND_TIMEOUT_MS) {
    timedOut = true;
    ++timeoutEvents;

    receivedPower = 0;

    for (uint8_t i = 0; i < 4; ++i) {
      targetPWM[i] = 0;
      currentPWM[i] = 0;
      reverseUntil[i] = 0;
    }

    applyAllMotors();
  }
}

// ============================================================
// Motor ramp
// ============================================================

void updateMotors() {
  const uint32_t now = millis();

  if (now - lastRampMs < RAMP_INTERVAL_MS) {
    return;
  }

  lastRampMs = now;

  if (timedOut) {
    return;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    const int16_t target = targetPWM[i];
    int16_t current = currentPWM[i];

    if (reverseUntil[i] != 0) {
      currentPWM[i] = 0;

      if (int32_t(now - reverseUntil[i]) < 0) {
        continue;
      }

      reverseUntil[i] = 0;
      current = 0;
    }

    if (current != 0 &&
        target != 0 &&
        signOf(current) != signOf(target)) {
      currentPWM[i] = 0;
      reverseUntil[i] = now + REVERSE_PAUSE_MS;
      continue;
    }

    if (current < target) {
      current += RAMP_STEP;
      if (current > target) current = target;

    } else if (current > target) {
      current -= RAMP_STEP;
      if (current < target) current = target;
    }

    currentPWM[i] = current;
  }

  applyAllMotors();
}

// ============================================================
// USB debug
// ============================================================

void printDebug() {
  if (!Serial) return;
  if (Serial.availableForWrite() < 180) return;

  const uint32_t age = commandReceived
      ? millis() - lastCommandMs
      : 0;

  char line[256];

  const int len = snprintf(
      line,
      sizeof(line),
      "MD RX=%lu SEQ=%u CRC=%lu BAD=%lu "
      "TIMEOUT=%lu AGE=%lums STATUS=%s "
      "DIR=%u POWER=%u "
      "TARGET=%d,%d,%d,%d "
      "PWM=%d,%d,%d,%d\n",
      static_cast<unsigned long>(receivedFrames),
      unsigned(lastSequence),
      static_cast<unsigned long>(crcErrors),
      static_cast<unsigned long>(invalidFrames),
      static_cast<unsigned long>(timeoutEvents),
      static_cast<unsigned long>(age),
      timedOut ? "STOP" : "RUN",
      unsigned(receivedDirection),
      unsigned(receivedPower),
      targetPWM[0], targetPWM[1],
      targetPWM[2], targetPWM[3],
      currentPWM[0], currentPWM[1],
      currentPWM[2], currentPWM[3]
  );

  if (len > 0 &&
      len < int(sizeof(line)) &&
      Serial.availableForWrite() >= len) {
    Serial.write(
        reinterpret_cast<const uint8_t *>(line),
        size_t(len)
    );
  }
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  pinMode(RS485_EN_PIN, OUTPUT);
  digitalWrite(RS485_EN_PIN, LOW);

  Serial1.setRX(RS485_RX_PIN);
  Serial1.begin(RS485_BAUD);

  analogWriteFreq(PWM_FREQ);
  analogWriteRange(255);

  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTOR_IN1[i], OUTPUT);
    pinMode(MOTOR_IN2[i], OUTPUT);

    analogWrite(MOTOR_IN1[i], 0);
    analogWrite(MOTOR_IN2[i], 0);
  }

  Serial.println("MD - OMNI PWM 120..200");
  Serial.println("RS485 115200 / PWM 20kHz");
  Serial.println("Debug interval: 1000ms");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  uint8_t frame[FRAME_SIZE];

  if (receiver.receive(frame)) {
    handleDrive(frame);
  }

  checkTimeout();
  updateMotors();

  if (millis() - lastDebugMs >=
      DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printDebug();
  }
}
