#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <hardware/gpio.h>
#include "ir_low_window.h"
#include "ir_match.h"

#include "src/BoardConfig.h"
#include "src/Rs485.h"

using namespace soccer;

// IRセンサーボード - 高度追跡処理
// RP2354A / TSSP58038 x 12
//
// RS485:
// TX = GPIO0
// RX = GPIO1
// DE = GPIO2
//
// 要求: 0x01
// 応答:
// AA PRED_H PRED_L DIST_H DIST_L 55

#define SENSOR_COUNT 12
#define RS485 Serial1
#define RS485_DE_PIN board::kIrRs485DePin

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

// センサー履歴

uint16_t detectCount[SENSOR_COUNT] = {};
uint16_t calibrationWindowCounts[SENSOR_COUNT] = {};
uint16_t calibrationWindowReads = 0;
uint16_t calibrationBatchCounts[SENSOR_COUNT] = {};
uint16_t calibrationBatchReads = 0;
uint32_t calibrationBatchStartUs = 0;
int calibratedDirection = -1;

uint16_t sensorHistory
  [SENSOR_COUNT][SENSOR_HISTORY_SIZE] = {};

uint32_t sensorSum[SENSOR_COUNT] = {};

uint8_t sensorHead = 0;
uint8_t sensorHistoryCount = 0;

// 相対信号の履歴

uint32_t relativeSignalHistory[RELATIVE_SIGNAL_HISTORY_SIZE] = {};

uint8_t relativeSignalHead = 0;
uint8_t relativeSignalCount = 0;

// 予測履歴

float angleHistory[PRED_HISTORY_SIZE] = {};
float relativeSignalPredictionHistory[PRED_HISTORY_SIZE] = {};
uint32_t timeHistory[PRED_HISTORY_SIZE] = {};

uint8_t predHead = 0;
uint8_t predCount = 0;

// 出力値

float rawAngle = 0;
float predictedAngle = 0;
float filteredRelativeSignal = 0;
float predictedRelativeSignal = 0;

uint32_t rawRelativeSignal = 0;
uint32_t totalDetect = 0;
float directionCoherence = 0.0f; // センサー幾何学的一致度。確率ではない。
uint32_t lastSensorSampleMs = 0;
bool sensorSampled = false;

bool ballFound = false;

uint32_t lastDebugTime = 0;

// USBシリアル校正ログ。CAL START 中だけ通常デバッグ表示を停止する。
bool calibrationStreamEnabled = false;
const char* calibrationCondition = "UNSET";
int16_t calibrationAngleDeg = -1;
char calibrationCommandBuffer[32] = {};
uint8_t calibrationCommandLength = 0;
bool calibrationCommandOverflow = false;
uint32_t calibrationSampleSequence = 0;
uint32_t lastCalibrationPrintMs = 0;
uint32_t sensorSampleWindowUs = 0;
uint32_t sensorSampleSweeps = 0;

// 追跡状態のリセット

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

// センサーサンプリング

uint16_t activeSensorMask(uint32_t gpioLevels) {
  return static_cast<uint16_t>((~gpioLevels >> 18) & 0x0FFFu);
}

void sampleSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    detectCount[i] = 0;
  }
  memset(calibrationWindowCounts, 0, sizeof(calibrationWindowCounts));
  calibrationWindowReads = 0;

  uint32_t start = micros();
  uint32_t sweeps = 0;
  IrLowWindow calibrationWindow;

  for (;;) {
    const uint32_t elapsed = static_cast<uint32_t>(micros() - start);
    if (elapsed >= SAMPLE_TIME_US) break;

    // 全センサーを同時刻に読む。校正窓もZIPの10 us LOW確認方式に合わせる。
    const uint16_t active = activeSensorMask(gpio_get_all());
    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (active & (1u << i)) {
        if (detectCount[i] < 65535) {
          detectCount[i]++;
        }
      }
    }
    calibrationWindow.sample(elapsed, active, calibrationWindowCounts, calibrationWindowReads);
    ++sweeps;
  }

  calibrationWindow.finish(calibrationWindowCounts);
  sensorSampleWindowUs = micros() - start;
  sensorSampleSweeps = sweeps;
}

void updateCalibrationProfile() {
  if (!calibrationBatchStartUs) calibrationBatchStartUs = micros();

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint32_t sum = static_cast<uint32_t>(calibrationBatchCounts[i]) + calibrationWindowCounts[i];
    calibrationBatchCounts[i] = static_cast<uint16_t>(min(sum, 65535u));
  }
  calibrationBatchReads = static_cast<uint16_t>(min(
    static_cast<uint32_t>(calibrationBatchReads) + calibrationWindowReads, 65535u));

  const uint32_t nowUs = micros();
  if (static_cast<uint32_t>(nowUs - calibrationBatchStartUs) < CALIBRATION_BATCH_US) return;
  calibratedDirection = matchIr(calibrationBatchCounts, calibrationBatchReads);
  memset(calibrationBatchCounts, 0, sizeof(calibrationBatchCounts));
  calibrationBatchReads = 0;
  calibrationBatchStartUs = nowUs;
}

// センサー履歴の更新

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

// 方向計算

void calculateDirection() {
  // chase ZIPのセンサー表は反時計回り。通信方位は正面0度・時計回りに変換する。
  const IrDirectionEstimate estimate = estimateIrDirection(
    detectCount, cosTable, sinTable, SENSOR_COUNT, MIN_TOTAL_DETECT);
  totalDetect = estimate.totalWeight;
  directionCoherence = estimate.coherence;
  ballFound = estimate.valid;
  if (ballFound) {
    rawAngle = calibratedDirection >= 0
      ? static_cast<float>(calibratedDirection)
      : normalizeDegrees(360.0f - estimate.angleDeg);
  }
}

// 相対信号が上位6個のインジケータ

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

  // 相対的な検出指標のみ。較正なしでは物理距離にならない。
  return avg;
}

// 相対信号の履歴

void pushRelativeSignal(uint32_t value) {
  relativeSignalHistory[relativeSignalHead] = value;

  relativeSignalHead =
    (relativeSignalHead + 1) % RELATIVE_SIGNAL_HISTORY_SIZE;

  if (relativeSignalCount < RELATIVE_SIGNAL_HISTORY_SIZE) {
    relativeSignalCount++;
  }
}

// 相対信号の重み付き平均

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

// 予測履歴

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

// 将来方位と相対信号の推移予測

void calculatePrediction() {
  const IrPrediction prediction = predictIr(angleHistory, relativeSignalPredictionHistory,
                                             timeHistory, predHead, predCount, PRED_HISTORY_SIZE,
                                             PREDICT_TIME_SEC, MAX_ANGULAR_SPEED);
  predictedAngle = prediction.valid ? prediction.angleDeg : 0.0f;
  predictedRelativeSignal = prediction.valid ? prediction.relativeSignal : 0.0f;
}

// IR処理まとめ

void updateIR() {
  sampleSensors();
  updateCalibrationProfile();
  lastSensorSampleMs = millis();
  sensorSampled = true;
  calculateDirection();

  if (!ballFound) {
    calibratedDirection = -1;
    memset(calibrationBatchCounts, 0, sizeof(calibrationBatchCounts));
    calibrationBatchReads = 0;
    calibrationBatchStartUs = micros();
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

// RS485応答

void sendIRResponse() {
  uint16_t direction = 0;
  uint16_t relativeSignal = 0;

  if (ballFound) {
    direction =
      (uint16_t)(rawAngle + 0.5f) % 360;

    // 非有限値が下の整数変換に渡らないようにする。通常値はそのまま。
    const float signal = isfinite(filteredRelativeSignal) ? filteredRelativeSignal : 0.0f;
    float limited = constrain(
      signal,
      1.0f,
      65535.0f
    );

    relativeSignal = (uint16_t)(limited + 0.5f);
  }

  const uint8_t response[6] = {0xAA, static_cast<uint8_t>(direction >> 8), static_cast<uint8_t>(direction),
                               static_cast<uint8_t>(relativeSignal >> 8), static_cast<uint8_t>(relativeSignal), 0x55};
  if (!rs485.send(response, sizeof(response), micros(), true)) ++rs485TxErrors;
}

// RS485要求処理

void handleRS485() {
  uint8_t byte = 0;
  uint8_t consumed = 0;
  while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
    if (irRequestParser.feed(byte, millis())) sendIRResponse();
  }
  irRequestParser.poll(millis());
}

// デバッグ出力

bool parseCalibrationAngle(const char* command, int16_t& angleDeg) {
  const char prefix[] = "CAL ANGLE ";
  for (uint8_t i = 0; i < sizeof(prefix) - 1; ++i) {
    if (command[i] != prefix[i]) return false;
  }

  const char* p = command + sizeof(prefix) - 1;
  if (*p < '0' || *p > '9') return false;
  int16_t value = 0;
  do {
    value = static_cast<int16_t>(value * 10 + (*p - '0'));
    if (value >= 360) return false;
    ++p;
  } while (*p >= '0' && *p <= '9');
  if (*p != '\0') return false;
  angleDeg = value;
  return true;
}

void processCalibrationCommand(char* command) {
  if (calibrationCommandOverflow) return;

  // コマンドはASCII大文字小文字を区別しない。
  for (char* p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = static_cast<char>(*p - 'a' + 'A');
  }

  if (strcmp(command, "CAL START") == 0) {
    calibrationStreamEnabled = true;
    calibrationSampleSequence = 0;
    lastCalibrationPrintMs = millis() - 100;
    Serial.println("CAL_HEADER,condition,angle_deg,sample_ms,seq,window_us,sweeps,total_low,s0,s1,s2,s3,s4,s5,s6,s7,s8,s9,s10,s11");
  } else if (strcmp(command, "CAL STOP") == 0) {
    calibrationStreamEnabled = false;
  } else if (strcmp(command, "CAL DARK") == 0) {
    calibrationCondition = "DARK";
  } else if (strcmp(command, "CAL SIGNAL") == 0) {
    calibrationCondition = "SIGNAL";
  } else {
    int16_t angleDeg = -1;
    if (parseCalibrationAngle(command, angleDeg)) calibrationAngleDeg = angleDeg;
  }
}

void pollCalibrationConsole() {
  if (!Serial) return;
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0) break;
    const char ch = static_cast<char>(incoming);
    if (ch == '\r') continue;
    if (ch == '\n') {
      calibrationCommandBuffer[calibrationCommandLength] = '\0';
      processCalibrationCommand(calibrationCommandBuffer);
      calibrationCommandLength = 0;
      calibrationCommandOverflow = false;
      continue;
    }
    if (calibrationCommandLength + 1 < sizeof(calibrationCommandBuffer)) {
      calibrationCommandBuffer[calibrationCommandLength++] = ch;
    } else {
      calibrationCommandOverflow = true;
    }
  }
}

void printCalibrationSample() {
  if (!calibrationStreamEnabled || !Serial) return;
  const uint32_t nowMs = millis();
  if (static_cast<uint32_t>(nowMs - lastCalibrationPrintMs) < 100) return;
  lastCalibrationPrintMs = nowMs;

  uint32_t totalLow = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) totalLow += detectCount[i];

  Serial.print("CAL,");
  Serial.print(calibrationCondition);
  Serial.print(',');
  Serial.print(calibrationAngleDeg);
  Serial.print(',');
  Serial.print(nowMs);
  Serial.print(',');
  Serial.print(calibrationSampleSequence++);
  Serial.print(',');
  Serial.print(sensorSampleWindowUs);
  Serial.print(',');
  Serial.print(sensorSampleSweeps);
  Serial.print(',');
  Serial.print(totalLow);
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(',');
    Serial.print(detectCount[i]);
  }
  Serial.println();
}

void printDebug() {
  if (calibrationStreamEnabled) return;
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
    Serial.print(" CAL_DIR:");
    Serial.print(calibratedDirection);
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

// SETUP

void setup() {
  Serial.begin(115200);
  calibrationBatchStartUs = micros();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);

  RS485.setTX(board::kIrRs485TxPin);
  RS485.setRX(board::kIrRs485RxPin);
  RS485.begin(kRs485Baud);

  resetTracking();

  if (Serial && Serial.availableForWrite() >= 64) Serial.println("IR ADVANCED TRACKING START");
}

// ループ処理

void loop() {
  pollCalibrationConsole();
  handleRS485();
  updateIR();
  handleRS485();
  printCalibrationSample();
  printDebug();
}
