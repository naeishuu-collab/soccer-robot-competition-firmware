
#include <Arduino.h>
#include <math.h>
#include <BoardConfig.h>
#include <RobotProtocol.h>
#include <Rs485.h>

using namespace soccer;

// ========================================
// IR SENSOR SUB - ADVANCED TRACKING
// RP2354A / TSSP58038 x 12
//
// RS485:
// TX = GPIO0
// RX = GPIO1
// DE = GPIO2
//
// Request: 0x01
// Response:
// AA PRED_H PRED_L DIST_H DIST_L 55
// ========================================

#define SENSOR_COUNT 12
#define RS485 Serial1
#define RS485_DE_PIN board::kIrRs485DePin
#define RS485_BAUD 115200

int rs485Available(void*) { return RS485.available(); }
int rs485Read(void*) { return RS485.read(); }
void rs485Direction(void*, bool tx) { digitalWrite(RS485_DE_PIN, tx ? HIGH : LOW); }
size_t rs485Write(void*, const uint8_t* bytes, size_t length) { return RS485.write(bytes, length); }
void rs485Flush(void*) { RS485.flush(); }
void rs485Delay(void*, uint32_t us) { delayMicroseconds(us); }
Rs485Io rs485Io{&RS485, rs485Available, rs485Read, rs485Direction, rs485Write, rs485Flush, rs485Delay};
Rs485Driver rs485{rs485Io};
SharedBusIrRequestParser irRequestParser;
uint32_t rs485TxErrors = 0;

#define SAMPLE_TIME_US 5000
#define SENSOR_HISTORY_SIZE 16
#define RELATIVE_SIGNAL_HISTORY_SIZE 16
#define PRED_HISTORY_SIZE 6

#define MIN_TOTAL_DETECT 10

#define PREDICT_TIME_SEC 0.030f
#define MAX_ANGULAR_SPEED 720.0f

const uint8_t sensorPins[SENSOR_COUNT] = {
  18,19,20,21,22,23,
  24,25,26,27,28,29
};

const int16_t cosTable[SENSOR_COUNT] = {
  1000,866,500,0,-500,-866,
  -1000,-866,-500,0,500,866
};

const int16_t sinTable[SENSOR_COUNT] = {
  0,500,866,1000,866,500,
  0,-500,-866,-1000,-866,-500
};

// ========================================
// Sensor history
// ========================================

uint16_t detectCount[SENSOR_COUNT] = {};

uint16_t sensorHistory
  [SENSOR_COUNT][SENSOR_HISTORY_SIZE] = {};

uint32_t sensorSum[SENSOR_COUNT] = {};

uint8_t sensorHead = 0;
uint8_t sensorHistoryCount = 0;

// ========================================
// Relative-signal history
// ========================================

uint32_t relativeSignalHistory[RELATIVE_SIGNAL_HISTORY_SIZE] = {};

uint8_t relativeSignalHead = 0;
uint8_t relativeSignalCount = 0;

// ========================================
// Prediction history
// ========================================

float angleHistory[PRED_HISTORY_SIZE] = {};
float relativeSignalPredictionHistory[PRED_HISTORY_SIZE] = {};
uint32_t timeHistory[PRED_HISTORY_SIZE] = {};

uint8_t predHead = 0;
uint8_t predCount = 0;

// ========================================
// Output values
// ========================================

float rawAngle = 0;
float predictedAngle = 0;
float filteredRelativeSignal = 0;
float predictedRelativeSignal = 0;

uint32_t rawRelativeSignal = 0;
uint32_t totalDetect = 0;
float directionCoherence = 0.0f; // Geometric sensor agreement, not a probability.
uint32_t lastSensorSampleMs = 0;
bool sensorSampled = false;

bool ballFound = false;

uint32_t lastDebugTime = 0;

// ========================================
// Reset tracking
// ========================================

void resetTracking() {
  sensorHead = 0;
  sensorHistoryCount = 0;

  relativeSignalHead = 0;
  relativeSignalCount = 0;

  predHead = 0;
  predCount = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorSum[i] = 0;

    for (int j = 0; j < SENSOR_HISTORY_SIZE; j++) {
      sensorHistory[i][j] = 0;
    }
  }

  rawAngle = 0;
  directionCoherence = 0.0f;
  predictedAngle = 0;
  rawRelativeSignal = 0;
  filteredRelativeSignal = 0;
  predictedRelativeSignal = 0;
  ballFound = false;
}

// ========================================
// Sensor sampling
// ========================================

void sampleSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    detectCount[i] = 0;
  }

  uint32_t start = micros();

  while ((uint32_t)(micros() - start) < SAMPLE_TIME_US) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (digitalRead(sensorPins[i]) == LOW) {
        if (detectCount[i] < 65535) {
          detectCount[i]++;
        }
      }
    }
  }
}

// ========================================
// Update sensor history
// ========================================

void updateSensorHistory() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    uint16_t oldValue = sensorHistory[i][sensorHead];

    if (sensorHistoryCount == SENSOR_HISTORY_SIZE) {
      sensorSum[i] -= oldValue;
    }

    sensorHistory[i][sensorHead] = detectCount[i];
    sensorSum[i] += detectCount[i];
  }

  sensorHead =
    (sensorHead + 1) % SENSOR_HISTORY_SIZE;

  if (sensorHistoryCount < SENSOR_HISTORY_SIZE) {
    sensorHistoryCount++;
  }
}

// ========================================
// Direction calculation
// ========================================

void calculateDirection() {
  const IrDirectionEstimate estimate = estimateIrDirection(
    detectCount, cosTable, sinTable, SENSOR_COUNT, MIN_TOTAL_DETECT);
  totalDetect = estimate.totalWeight;
  directionCoherence = estimate.coherence;
  ballFound = estimate.valid;
  if (ballFound) rawAngle = estimate.angleDeg;
}

// ========================================
// Top 6 relative-signal indicator
// ========================================

uint32_t calculateRawRelativeSignal() {
  uint32_t temp[SENSOR_COUNT];

  for (int i = 0; i < SENSOR_COUNT; i++) {
    temp[i] = sensorSum[i];
  }

  uint64_t sumTop6 = 0;

  for (int k = 0; k < 6; k++) {
    int maxIndex = 0;

    for (int i = 1; i < SENSOR_COUNT; i++) {
      if (temp[i] > temp[maxIndex]) {
        maxIndex = i;
      }
    }

    sumTop6 += temp[maxIndex];
    temp[maxIndex] = 0;
  }

  uint32_t avg = (uint32_t)(sumTop6 / 6);

  // Relative detector indicator only; no physical distance without calibration
  return avg;
}

// ========================================
// Relative-signal history
// ========================================

void pushRelativeSignal(uint32_t value) {
  relativeSignalHistory[relativeSignalHead] = value;

  relativeSignalHead =
    (relativeSignalHead + 1) % RELATIVE_SIGNAL_HISTORY_SIZE;

  if (relativeSignalCount < RELATIVE_SIGNAL_HISTORY_SIZE) {
    relativeSignalCount++;
  }
}

// ========================================
// Weighted relative-signal average
// ========================================

float getWeightedRelativeSignal() {
  if (relativeSignalCount == 0) return 0;

  float recentSum = 0;
  float middleSum = 0;
  float oldSum = 0;

  int recentN = 0;
  int middleN = 0;
  int oldN = 0;

  int samples = min((int)relativeSignalCount, 8);

  for (int i = 0; i < samples; i++) {
    int idx =
      (relativeSignalHead - 1 - i + RELATIVE_SIGNAL_HISTORY_SIZE)
      % RELATIVE_SIGNAL_HISTORY_SIZE;

    float value = relativeSignalHistory[idx];

    if (i == 0) {
      recentSum += value;
      recentN++;
    }
    else if (i < 4) {
      middleSum += value;
      middleN++;
    }
    else {
      oldSum += value;
      oldN++;
    }
  }

  float weighted = 0;
  float weightTotal = 0;

  if (recentN > 0) {
    weighted += (recentSum / recentN) * 10.0f;
    weightTotal += 10.0f;
  }

  if (middleN > 0) {
    weighted += (middleSum / middleN) * 2.0f;
    weightTotal += 2.0f;
  }

  if (oldN > 0) {
    weighted += (oldSum / oldN) * 0.4f;
    weightTotal += 0.4f;
  }

  return weighted / weightTotal;
}

// ========================================
// Prediction history
// ========================================

void pushPrediction(
  float angle,
  float relativeSignal,
  uint32_t timestamp
) {
  angleHistory[predHead] = angle;
  relativeSignalPredictionHistory[predHead] = relativeSignal;
  timeHistory[predHead] = timestamp;

  predHead = (predHead + 1) % PRED_HISTORY_SIZE;

  if (predCount < PRED_HISTORY_SIZE) {
    predCount++;
  }
}

// ========================================
// Future bearing and relative-signal trend prediction
// ========================================

void calculatePrediction() {
  const IrPrediction prediction = predictIr(angleHistory, relativeSignalPredictionHistory,
                                             timeHistory, predHead, predCount, PRED_HISTORY_SIZE,
                                             PREDICT_TIME_SEC, MAX_ANGULAR_SPEED);
  predictedAngle = prediction.valid ? prediction.angleDeg : 0.0f;
  predictedRelativeSignal = prediction.valid ? prediction.relativeSignal : 0.0f;
}

// ========================================
// Full IR processing
// ========================================

void updateIR() {
  sampleSensors();
  lastSensorSampleMs = millis();
  sensorSampled = true;
  calculateDirection();

  if (!ballFound) {
    resetTracking();
    return;
  }

  updateSensorHistory();

  rawRelativeSignal = calculateRawRelativeSignal();
  pushRelativeSignal(rawRelativeSignal);

  filteredRelativeSignal = getWeightedRelativeSignal();

  pushPrediction(
    rawAngle,
    filteredRelativeSignal,
    micros()
  );

  calculatePrediction();
}

// ========================================
// RS485 response
// ========================================

void sendIRResponse() {
  uint16_t direction = 0;
  uint16_t relativeSignal = 0;

  if (ballFound) {
    direction =
      (uint16_t)(predictedAngle + 0.5f) % 360;

    float limited = constrain(
      filteredRelativeSignal,
      1.0f,
      65535.0f
    );

    relativeSignal = (uint16_t)(limited + 0.5f);
  }

  const uint8_t response[6] = {0xAA, static_cast<uint8_t>(direction >> 8), static_cast<uint8_t>(direction),
                               static_cast<uint8_t>(relativeSignal >> 8), static_cast<uint8_t>(relativeSignal), 0x55};
  if (!rs485.send(response, sizeof(response), micros(), true)) ++rs485TxErrors;
}

// ========================================
// RS485 request handler
// ========================================

void handleRS485() {
  uint8_t byte = 0;
  uint8_t consumed = 0;
  while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
    if (irRequestParser.feed(byte, millis())) sendIRResponse();
  }
  irRequestParser.poll(millis());
}

// ========================================
// Debug
// ========================================

void printDebug() {
  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - lastDebugTime) < 100) return;
  lastDebugTime = nowMs;
  if (!Serial) return;

  if (Serial.availableForWrite() >= 128) {
    Serial.print("FOUND:");
    Serial.print(ballFound);
    Serial.print(" RAW_ANG:");
    Serial.print(rawAngle, 1);
    Serial.print(" PRED_ANG:");
    Serial.print(predictedAngle, 1);
    Serial.print(" TOTAL:");
    Serial.println(totalDetect);
  }
  if (Serial.availableForWrite() >= 128) {
    Serial.print("RAW_REL_SIGNAL:");
    Serial.print(rawRelativeSignal);
    Serial.print(" FILTER_REL_SIGNAL:");
    Serial.println(filteredRelativeSignal, 1);
  }
  if (Serial.availableForWrite() >= 128) {
    Serial.print("PRED_REL_SIGNAL:");
    Serial.print(predictedRelativeSignal, 1);
    Serial.print(" DIRECTION_COHERENCE:");
    Serial.print(directionCoherence, 3);
    Serial.print(" SENSOR_AGE_MS:");
    Serial.println(sensorSampled ? static_cast<uint32_t>(nowMs - lastSensorSampleMs) : 0xFFFFFFFFu);
  }
  if (Serial.availableForWrite() >= 128) {
    const Rs485Diagnostics& bus = rs485.diagnostics();
    Serial.print("RS485 tx/rx/busy/errors/frame-timeout=");
    Serial.print(bus.txFrames); Serial.print('/'); Serial.print(bus.rxBytes); Serial.print('/');
    Serial.print(bus.txBusy); Serial.print('/'); Serial.print(bus.txErrors + rs485TxErrors); Serial.print('/');
    Serial.println(irRequestParser.stats().timeouts);
  }
}

// ========================================
// SETUP
// ========================================

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  RS485.setTX(board::kIrRs485TxPin);
  RS485.setRX(board::kIrRs485RxPin);
  RS485.begin(RS485_BAUD);

  resetTracking();

  if (Serial && Serial.availableForWrite() >= 64) Serial.println("IR ADVANCED TRACKING START");
}

// ========================================
// LOOP
// ========================================

void loop() {
  handleRS485();
  updateIR();
  handleRS485();
  printDebug();
}

