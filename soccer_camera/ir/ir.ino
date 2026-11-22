
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <hardware/gpio.h>

// ============================================================
// IR SUB - RP2354A
// TSSP58038 x 12
//
// GPIO18 = Front (0 deg)
// GPIO21 = Left  (270 deg)
// GPIO24 = Back  (180 deg)
// GPIO27 = Right (90 deg)
//
// Direction: clockwise positive
// Sampling: 5000 us
// RS485: MAIN-compatible POLL/BALL
// ============================================================

// ------------------------------------------------------------
// IR sensors
// ------------------------------------------------------------

// BEGIN HOST-TESTED IR SENSOR CONFIG
constexpr uint8_t SENSOR_COUNT = 12;

const uint8_t sensorPins[SENSOR_COUNT] = {
  18, 19, 20, 21,
  22, 23, 24, 25,
  26, 27, 28, 29
};

// Original vector table.
// Sensor index increases counterclockwise physically.
const int16_t cosTable[SENSOR_COUNT] = {
   1000,  866,  500,    0,
   -500, -866, -1000, -866,
   -500,    0,  500,  866
};

const int16_t sinTable[SENSOR_COUNT] = {
      0,  500,  866, 1000,
    866,  500,    0, -500,
   -866, -1000, -866, -500
};

// ------------------------------------------------------------
// Sampling
// ------------------------------------------------------------

constexpr uint32_t SAMPLE_TIME_US = 5000;
constexpr uint16_t MIN_TOTAL_DETECT = 1;

// Physical sensor arrangement is counterclockwise.
// Convert calculated angle to clockwise.
constexpr bool REVERSE_DIRECTION = true;

inline uint16_t activeSensorMask(uint32_t pins) {
  return uint16_t((~pins >> 18) & 0xFFFu);
}
// END HOST-TESTED IR SENSOR CONFIG

// BEGIN HOST-TESTED IR PREDICTION
inline bool irSampleFresh(bool sampled, uint32_t nowUs, uint32_t sampleUs) {
  return sampled && uint32_t(nowUs - sampleUs) <= 25000;
}

inline uint16_t vectorCoherencePermille(int32_t x, int32_t y, uint32_t total) {
  if (!total) return 0;
  const float ratio = sqrtf(float(x) * float(x) + float(y) * float(y)) / float(total);
  return uint16_t(ratio >= 1000.0f ? 1000 : lroundf(ratio));
}

struct IrPredictor {
  uint16_t angles[3] = {};
  uint32_t times[3] = {};
  uint8_t count = 0;

  void reset() { count = 0; }

  bool update(uint16_t angle, uint32_t now, uint16_t &predicted) {
    predicted = angle;
    if (angle >= 360) { reset(); return false; }
    if (count == 3) {
      angles[0] = angles[1]; angles[1] = angles[2];
      times[0] = times[1]; times[1] = times[2];
      count = 2;
    }
    angles[count] = angle;
    times[count++] = now;
    if (count < 3) return false;
    const uint32_t dt1 = times[1] - times[0], dt2 = times[2] - times[1];
    if (dt1 < 4000 || dt1 > 40000 || dt2 < 4000 || dt2 > 40000) return false;
    int16_t d1 = int16_t(angles[1]) - int16_t(angles[0]);
    int16_t d2 = int16_t(angles[2]) - int16_t(angles[1]);
    if (d1 > 180) d1 -= 360; else if (d1 < -180) d1 += 360;
    if (d2 > 180) d2 -= 360; else if (d2 < -180) d2 += 360;
    const float v1 = float(d1) * 1000000.0f / float(dt1);
    const float v2 = float(d2) * 1000000.0f / float(dt2);
    if (d1 == 0 || d2 == 0 || (d1 > 0) != (d2 > 0) ||
        fabsf(v1) > 720.0f || fabsf(v2) > 720.0f || fabsf(v1 - v2) > 300.0f) return false;
    float lead = (v1 + v2) * 0.005f;
    if (lead > 5.0f) lead = 5.0f;
    if (lead < -5.0f) lead = -5.0f;
    int value = int(angle) + int(lroundf(lead));
    predicted = uint16_t((value + 360) % 360);
    return true;
  }
};
// END HOST-TESTED IR PREDICTION

// ------------------------------------------------------------
// RS485
// ------------------------------------------------------------

constexpr uint8_t RS485_TX_PIN = 0;
constexpr uint8_t RS485_RX_PIN = 1;
constexpr uint8_t RS485_DE_PIN = 2;

constexpr uint32_t RS485_BAUD = 115200;

// ------------------------------------------------------------
// Protocol
// ------------------------------------------------------------

constexpr uint8_t FRAME_START = 0xA5;
constexpr uint8_t FRAME_SIZE = 12;

constexpr uint8_t MSG_POLL = 1;
constexpr uint8_t MSG_BALL = 2;
constexpr uint8_t MSG_DRIVE = 3;

constexpr uint32_t RX_TIMEOUT_US = 3000;

// ------------------------------------------------------------
// Debug
// ------------------------------------------------------------

constexpr uint32_t DEBUG_INTERVAL_MS = 100;

// ------------------------------------------------------------
// State
// ------------------------------------------------------------

uint16_t detectCount[SENSOR_COUNT] = {};

uint16_t ballDirection = 0;
uint16_t ballStrength = 0;
uint16_t predictedDirection = 0;
bool predictionValid = false;
IrPredictor predictor;
uint32_t lastSampleUs = 0;
bool hasSample = false;

uint32_t totalDetectRaw = 0;
uint16_t directionCoherence = 0;

uint32_t lastDebugMs = 0;

uint32_t pollCount = 0;
uint32_t replyCount = 0;
uint32_t crcErrors = 0;
uint32_t invalidFrames = 0;

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
// Pack 14-bit value
// ============================================================

void packValue(uint8_t *destination,
               uint16_t value) {
  destination[0] = (value >> 7) & 0x7F;
  destination[1] = value & 0x7F;
}

// ============================================================
// Send BALL response
// ============================================================

// BEGIN HOST-TESTED BALL RESPONSE
void sendBall(uint8_t sequence) {
  uint8_t frame[FRAME_SIZE] = {};

  frame[0] = FRAME_START;
  frame[1] = MSG_BALL;
  frame[2] = sequence & 0x7F;

  const bool fresh = irSampleFresh(hasSample, micros(), lastSampleUs);
  packValue(&frame[3], fresh ? ballDirection : 0);
  packValue(&frame[5], fresh ? ballStrength : 0);

  if (fresh && ballStrength && predictionValid) {
    packValue(&frame[7], predictedDirection);
    frame[9] = 1;
  }

  frame[11] = frameCrc(frame);

  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(20);

  Serial1.write(frame, FRAME_SIZE);
  Serial1.flush();

  delayMicroseconds(20);
  digitalWrite(RS485_DE_PIN, LOW);

  ++replyCount;
}
// END HOST-TESTED BALL RESPONSE

// ============================================================
// Handle received frames
// ============================================================

void handleFrame(const uint8_t *frame) {
  if (frame[1] != MSG_POLL) {
    return;
  }

  ++pollCount;
  sendBall(frame[2]);
}

// ============================================================
// Sample sensors
// ============================================================

void sampleSensors() {
  memset(detectCount, 0, sizeof(detectCount));

  const uint32_t startTime = micros();

  while ((uint32_t)(micros() - startTime) <
         SAMPLE_TIME_US) {
    const uint16_t active = activeSensorMask(gpio_get_all());
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
      if ((active & (1u << i)) && detectCount[i] != UINT16_MAX) ++detectCount[i];
  }
  lastSampleUs = micros();
}

// ============================================================
// Calculate direction
// ============================================================

// BEGIN HOST-TESTED IR DIRECTION
void calculateDirection() {
  int32_t vectorX = 0;
  int32_t vectorY = 0;

  uint32_t total = 0;
  uint8_t activeSensors = 0;

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t count = detectCount[i];

    if (count) ++activeSensors;

    total += count;

    vectorX += int32_t(count) * cosTable[i];
    vectorY += int32_t(count) * sinTable[i];
  }

  totalDetectRaw = total;
  directionCoherence = vectorCoherencePermille(vectorX, vectorY, total);

  if (total < MIN_TOTAL_DETECT ||
      (vectorX == 0 && vectorY == 0)) {
    ballDirection = 0;
    ballStrength = 0;
    predictionValid = false;
    predictor.reset();
    return;
  }

  float angle =
      atan2f(float(vectorY), float(vectorX)) *
      180.0f / PI;

  if (angle < 0.0f) {
    angle += 360.0f;
  }

  uint16_t direction =
      uint16_t(angle + 0.5f);

  if (direction >= 360) {
    direction = 0;
  }

  // Convert counterclockwise to clockwise.
  if (REVERSE_DIRECTION) {
    direction = (360 - direction) % 360;
  }

  ballDirection = direction;

  // Keep within MAIN's accepted range.
  ballStrength =
      uint16_t(min(total, uint32_t(1200)));
  predictionValid = predictor.update(ballDirection, micros(), predictedDirection) && activeSensors > 1;
}
// END HOST-TESTED IR DIRECTION

// ============================================================
// USB debug
// ============================================================

void printDebug() {
  if (!Serial) return;
  static uint8_t phase = 0;
  char line[96];
  int length;
  if (phase == 0) {
    length = snprintf(line, sizeof(line), "IR DIR=%u PRED=%u VALID=%u STR=%u COH=%u TOTAL=%lu\n",
        unsigned(ballDirection), unsigned(predictedDirection), unsigned(predictionValid),
        unsigned(ballStrength), unsigned(directionCoherence), static_cast<unsigned long>(totalDetectRaw));
  } else if (phase < 3) {
    const uint8_t first = phase == 1 ? 0 : 6;
    length = snprintf(line, sizeof(line), "IR COUNT%u=%u,%u,%u,%u,%u,%u\n", unsigned(first),
        unsigned(detectCount[first]), unsigned(detectCount[first + 1]), unsigned(detectCount[first + 2]),
        unsigned(detectCount[first + 3]), unsigned(detectCount[first + 4]), unsigned(detectCount[first + 5]));
  } else {
    length = snprintf(line, sizeof(line), "IR POLL=%lu REPLY=%lu CRC=%lu BAD=%lu\n",
        static_cast<unsigned long>(pollCount), static_cast<unsigned long>(replyCount),
        static_cast<unsigned long>(crcErrors), static_cast<unsigned long>(invalidFrames));
  }
  if (length > 0 && length < int(sizeof(line)) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t *>(line), size_t(length));
  }
  phase = (phase + 1) % 4;
}

// ============================================================
// Setup
// ============================================================

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  Serial1.setTX(RS485_TX_PIN);
  Serial1.setRX(RS485_RX_PIN);
  Serial1.begin(RS485_BAUD);

  Serial.println();
  Serial.println("================================");
  Serial.println("IR SUB - RP2354A");
  Serial.println("TSSP58038 x 12");
  Serial.println("GPIO18 = FRONT 0");
  Serial.println("GPIO21 = LEFT 270");
  Serial.println("GPIO27 = RIGHT 90");
  Serial.println("SAMPLING = 5000us");
  Serial.println("RS485 POLL/BALL 115200");
  Serial.println("================================");
}

// ============================================================
// Loop
// ============================================================

void loop() {
  uint8_t frame[FRAME_SIZE];

  // Process pending POLL.
  while (receiver.receive(frame)) {
    handleFrame(frame);
  }

  // Sample all 12 IR sensors.
  sampleSensors();

  // Calculate ball direction.
  calculateDirection();
  hasSample = true;

  // Respond to POLL received during sampling.
  while (receiver.receive(frame)) {
    handleFrame(frame);
  }

  // USB debug output.
  if (millis() - lastDebugMs >=
      DEBUG_INTERVAL_MS) {
    lastDebugMs = millis();
    printDebug();
  }
}
