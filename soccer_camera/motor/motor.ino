#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "motor_logic.h"

// 0 = field-tested legacy mixer (default). 1 = wheel model with start boost (verify on hardware first).
#ifndef ROBOT_UPGRADE
#define ROBOT_UPGRADE 0
#endif

// ============================================================
// MOTOR - RP2350 / TB67H450 x 4
//
// Channel order:
// [0] GPIO6,7    [1] GPIO12,13
// [2] GPIO11,10  [3] GPIO9,8
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
const uint8_t MOTOR_IN1[4] = {6, 12, 11, 9};
const uint8_t MOTOR_IN2[4] = {7, 13, 10, 8};

// PWM
constexpr bool OUTPUTS_ENABLED = true;
constexpr uint32_t PWM_FREQ = 20000;
constexpr int16_t PWM_MAX = 210;
constexpr int16_t MIN_RUNNING_DUTY = 160;

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

// Forward on the actual robot uses IN1 for all four channels.
constexpr int8_t WHEEL_SIGN[4] = {
  1, -1, 1, -1
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
int8_t pausedSign[4] = {};
uint32_t coordinatedReverseUntil = 0;

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

// BEGIN HOST-TESTED MIXER
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
  const float minimumDuty = min(requestedPower, float(MIN_RUNNING_DUTY));
  constexpr float startProjection = 0.17365f; // sin(10 deg)
  constexpr float keepProjection = 0.08716f;  // sin(5 deg)

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

  for (uint8_t i = 0; i < 4; ++i) {
    const float magnitude = fabsf(projection[i]);
    const int8_t sign = projection[i] < 0 ? -1 : 1;
    const bool sameDirection = signOf(targetPWM[i]) == sign;
    if (magnitude < (sameDirection ? keepProjection : startProjection)) continue;

    // Preserve the stronger wheel's duty while suppressing chatter near zero.
    const float fraction = magnitude / maxMagnitude;
    output[i] = int16_t(sign * lroundf(minimumDuty + (requestedPower - minimumDuty) * fraction));
  }
}
// END HOST-TESTED MIXER

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

// BEGIN HOST-TESTED DRIVE HANDLER
void pauseMotor(uint8_t motor, uint32_t now) {
  pausedSign[motor] = signOf(currentPWM[motor]);
  currentPWM[motor] = 0;
  reverseUntil[motor] = now + REVERSE_PAUSE_MS;
  if (reverseUntil[motor] == 0) reverseUntil[motor] = 1;
}

uint32_t reverseWaitMs(uint8_t motor, int16_t target, uint32_t now) {
  if (!reverseUntil[motor] || int32_t(pausedSign[motor]) * target >= 0) return 0;
  const int32_t remaining = int32_t(reverseUntil[motor] - now);
  return remaining > 0 ? uint32_t(remaining) : 0;
}

void stopAllMotors(uint32_t now) {
  for (uint8_t i = 0; i < 4; ++i) {
    targetPWM[i] = 0;
    if (currentPWM[i] != 0) pauseMotor(i, now);
  }
  applyAllMotors();
}

bool parseDrive(const uint8_t *frame, uint16_t &direction, uint16_t &power, int &turn, bool &avoid) {
  if (frame[1] == MSG_POLL ||
      frame[1] == MSG_BALL) {
    return false;
  }

  if (frame[1] != MSG_DRIVE) {
    ++invalidFrames;
    return false;
  }

  direction = unpackValue(&frame[3]);
  power = unpackValue(&frame[5]);

  if (direction >= 360 || power > PWM_MAX) {
    ++invalidFrames;
    return false;
  }

  avoid = frame[7] == 1;
  if (frame[7] > 1 || (frame[8] && (frame[8] < 21 || frame[8] > 101)) || frame[9] || frame[10] ||
      (avoid && !power)) {
    ++invalidFrames;
    return false;
  }
  turn = frame[8] ? int(frame[8]) - 61 : 0;
  return true;
}

void acceptCommand(uint8_t sequence, uint16_t direction, uint16_t power, uint32_t now) {
  receivedDirection = direction;
  receivedPower = power;
  lastSequence = sequence;
  lastCommandMs = now;
  commandReceived = true;
  timedOut = false;
  ++receivedFrames;
}

void handleDrive(const uint8_t *frame) {
  uint16_t direction = 0, power = 0;
  int turn = 0;
  bool avoid = false;
  if (!parseDrive(frame, direction, power, turn, avoid)) return;

  const uint32_t now = millis();
  if (power == 0) {
    stopAllMotors(now);
  } else {
    int16_t nextTarget[4];
    calculateWheelPWM(direction, power, nextTarget);
    for (uint8_t i = 0; i < 4; ++i) {
      nextTarget[i] = int16_t(constrain(int(nextTarget[i]) + turn * WHEEL_SIGN[i], -int(PWM_MAX), int(PWM_MAX)));
    }
    if (avoid) {
      uint32_t waitMs = 0;
      for (uint8_t i = 0; i < 4; ++i) {
        const bool reversing = int32_t(currentPWM[i]) * nextTarget[i] < 0;
        const uint32_t wheelWait = reversing ? REVERSE_PAUSE_MS
                                             : reverseWaitMs(i, nextTarget[i], now);
        if (wheelWait > waitMs) waitMs = wheelWait;
      }
      const int32_t currentWait = coordinatedReverseUntil
                                      ? int32_t(coordinatedReverseUntil - now) : 0;
      if (waitMs && int32_t(waitMs) > currentWait) {
        stopAllMotors(now);
        coordinatedReverseUntil = now + waitMs;
        if (!coordinatedReverseUntil) coordinatedReverseUntil = 1;
      }
    }
    memcpy(targetPWM, nextTarget, sizeof(targetPWM));
  }

  acceptCommand(frame[2], direction, power, now);
}

// ============================================================
// Communication timeout
// ============================================================

void checkTimeout() {
  if (!commandReceived || timedOut) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastCommandMs >= COMMAND_TIMEOUT_MS) {
    timedOut = true;
    ++timeoutEvents;

    receivedPower = 0;

    stopAllMotors(now);
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

  if (timedOut) return;
  if (coordinatedReverseUntil) {
    if (int32_t(now - coordinatedReverseUntil) < 0) return;
    coordinatedReverseUntil = 0;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    const int16_t target = targetPWM[i];
    const int16_t current = currentPWM[i];

    if (reverseUntil[i]) {
      if (int32_t(now - reverseUntil[i]) >= 0) {
        reverseUntil[i] = 0;
      } else if (target == 0 || reverseWaitMs(i, target, now)) {
        continue;
      } else {
        reverseUntil[i] = 0;
      }
    }

    if (current == 0) {
      currentPWM[i] = int16_t(signOf(target) * min(abs(target), int(MIN_RUNNING_DUTY)));
    } else if (signOf(current) != signOf(target)) {
      pauseMotor(i, now);
    } else {
      const int delta = constrain(target - current, -int(RAMP_STEP), int(RAMP_STEP));
      currentPWM[i] = int16_t(current + delta);
    }
  }

  applyAllMotors();
}

// END HOST-TESTED DRIVE HANDLER

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
      timedOut || !receivedPower ? "STOP" : "RUN",
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
// Wheel-model path and USB diagnostics
// ============================================================

wheel::Drive modelDrive;
wheel::Diag diag;
uint32_t startRequestUs = 0, startLatencyUs = 0;
uint32_t diagExitMs = 0;
bool diagCooling = false;  // after diag, wait one reverse pause before obeying MAIN

void readDiagSerial(uint32_t now) {
  static char text[32];
  static uint8_t used = 0;
  while (Serial.available()) {
    const char c = char(Serial.read());
    if (c != '\n' && c != '\r') {
      if (used < sizeof(text) - 1) text[used++] = c;
      continue;
    }
    if (!used) continue;
    text[used] = 0;
    used = 0;
    const bool wasActive = diag.active();
    Serial.println(diag.line(text, now));
    if (diag.active() != wasActive) {
      stopAllMotors(now);
      modelDrive.stop(now);
      timedOut = true;  // leaving diag waits for a fresh MAIN command
      if (wasActive) {
        diagExitMs = now;
        diagCooling = true;
        applyAllMotors();
      }
    }
  }
}

void runDiag(uint32_t now) {
  uint8_t frame[FRAME_SIZE];
  receiver.receive(frame);  // drain the bus; commands are ignored
  diag.update(now);
  for (uint8_t i = 0; i < 4; ++i) applyMotor(i, int16_t(diag.duty(i, now)));
}

void runModel(uint32_t now) {
  uint8_t frame[FRAME_SIZE];
  if (receiver.receive(frame)) {
    uint16_t direction = 0, power = 0;
    int turn = 0;
    bool avoid = false;
    if (parseDrive(frame, direction, power, turn, avoid)) {
      const bool wasStill = !(currentPWM[0] || currentPWM[1] || currentPWM[2] || currentPWM[3]);
      if (power || turn) {
        if (wasStill && !startRequestUs) startRequestUs = micros();
        modelDrive.command(direction, power, turn, now);
      } else {
        modelDrive.stop(now);
        startRequestUs = 0;
      }
      acceptCommand(frame[2], direction, power, now);
    }
  }
  if (commandReceived && !timedOut && now - lastCommandMs >= COMMAND_TIMEOUT_MS) {
    timedOut = true;
    ++timeoutEvents;
    receivedPower = 0;
    modelDrive.stop(now);
    startRequestUs = 0;
  }
  if (!timedOut) modelDrive.update(now);
  for (uint8_t i = 0; i < 4; ++i) {
    targetPWM[i] = int16_t(modelDrive.target(i));
    currentPWM[i] = int16_t(modelDrive.duty(i));
  }
  applyAllMotors();
  if (startRequestUs && (currentPWM[0] || currentPWM[1] || currentPWM[2] || currentPWM[3])) {
    startLatencyUs = micros() - startRequestUs;
    startRequestUs = 0;
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

  Serial.println("MD - OMNI PWM max 210 / startup duty up to 160");
  Serial.println("RS485 115200 / PWM 20kHz");
  Serial.println("Debug interval: 1000ms");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  const uint32_t now = millis();
  readDiagSerial(now);
  if (diagCooling && now - diagExitMs >= REVERSE_PAUSE_MS) diagCooling = false;
  if (diag.active()) {
    runDiag(now);
  } else if (diagCooling) {
    uint8_t frame[FRAME_SIZE];
    receiver.receive(frame);
  } else if (ROBOT_UPGRADE) {
    runModel(now);
  } else {
    uint8_t frame[FRAME_SIZE];
    checkTimeout();
    if (receiver.receive(frame)) {
      handleDrive(frame);
    }
    updateMotors();
  }

  if (millis() - lastDebugMs >=
      DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printDebug();
    if (ROBOT_UPGRADE && Serial && Serial.availableForWrite() >= 64) {
      Serial.printf("MODEL err=%d feasible=%d start_us=%lu\n", int(modelDrive.mix().errorDeg),
                    int(modelDrive.mix().feasible), static_cast<unsigned long>(startLatencyUs));
    }
  }
}
