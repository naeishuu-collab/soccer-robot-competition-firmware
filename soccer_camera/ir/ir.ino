// IR基板: 12個の受光センサーを読み、MAINからの問い合わせにボールの有無・方位を返す。
// 通常系は互換性重視の既存応答、拡張フィルターは ir_logic.h とMAIN拡張モードでのみ使用する。
// MAINは定期ポーリングするため、この基板は要求を受けてから応答し、RS-485上の衝突を避ける。
#include <Arduino.h>
#include "ir_logic.h"

// Keep the chase (3) per-poll sampling and raw sensor-index angle.
constexpr uint8_t SENSOR_COUNT = 12;
constexpr uint8_t RS485_TX_PIN = 0;
constexpr uint8_t RS485_RX_PIN = 1;
constexpr uint8_t RS485_DE_PIN = 2;
constexpr uint32_t RS485_BAUD = 115200;

const uint8_t sensorPins[SENSOR_COUNT] = {18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};
const int16_t cosTable[SENSOR_COUNT] = {1000, 866, 500, 0, -500, -866, -1000, -866, -500, 0, 500, 866};
const int16_t sinTable[SENSOR_COUNT] = {0, 500, 866, 1000, 866, 500, 0, -500, -866, -1000, -866, -500};
const uint32_t SAMPLE_TIME_US = 5000;
uint16_t detectCount[SENSOR_COUNT];

irx::Filter irFilter;
bool verbose = false;  // USB 'v' toggles per-sensor output

enum RequestState { WAIT_POLL, READ_POLL_2, READ_POLL_CMD, READ_POLL_END };
RequestState requestState = WAIT_POLL;

void sendIR(uint16_t direction, uint16_t totalDetect) {
  // 方向と検出強度を固定長応答にして、RS-485送信中だけドライバを送信側へ切り替える。
  digitalWrite(RS485_DE_PIN, HIGH);
  delayMicroseconds(20);
  Serial1.write(0xAA);
  Serial1.write(direction >> 8); Serial1.write(direction & 0xFF);
  Serial1.write(totalDetect >> 8); Serial1.write(totalDetect & 0xFF);
  Serial1.write(0x55);
  Serial1.flush();
  delayMicroseconds(20);
  digitalWrite(RS485_DE_PIN, LOW);
}

void sampleAndReply() {
  // 旧プロトコルの1回分: 5ms測定 → センサー別回数を方向ベクトルへ合成 → MAINへ応答。
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) detectCount[i] = 0;
  uint32_t startTime = micros();
  while ((uint32_t)(micros() - startTime) < SAMPLE_TIME_US) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      if (digitalRead(sensorPins[i]) == LOW) detectCount[i]++;
    }
  }

  // 12方向の検出数を単位ベクトルに掛けて足すと、受光が強い方向へ合成ベクトルが向く。
  int32_t vectorX = 0, vectorY = 0;
  uint32_t totalDetect = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    totalDetect += detectCount[i];
    vectorX += (int32_t)detectCount[i] * cosTable[i];
    vectorY += (int32_t)detectCount[i] * sinTable[i];
  }
  if (totalDetect > 65535) totalDetect = 65535;

  uint16_t direction = 0;
  if (vectorX || vectorY) {
    float angle = atan2((float)vectorY, (float)vectorX) * 180.0 / PI;
    if (angle < 0) angle += 360.0;
    direction = (uint16_t)(angle + 0.5);
    if (direction >= 360) direction = 0;
  } else {
    // Opposing detections can cancel out; there is then no usable direction.
    totalDetect = 0;
  }
  sendIR(direction, (uint16_t)totalDetect);
  if (Serial && Serial.availableForWrite() >= 48) {
    Serial.printf("IR reply: direction=%u detect=%u\n", unsigned(direction), unsigned(totalDetect));
  }
}

// Command 0x02: normalized, filtered reply with confidence and fault mask.
void sampleFiltered(bool reply) {
  // 拡張要求時は測定回数もir_logic.hへ渡し、正規化/フィルター/故障判定後の値を符号化する。
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) detectCount[i] = 0;
  uint32_t samples = 0;
  const uint32_t startTime = micros();
  while ((uint32_t)(micros() - startTime) < SAMPLE_TIME_US) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      if (digitalRead(sensorPins[i]) == LOW) detectCount[i]++;
    }
    ++samples;
  }
  const irx::Result result = irFilter.update(detectCount, samples);
  if (reply) {
    uint8_t frame[irx::kReplySize];
    irx::encode(result, frame);
    digitalWrite(RS485_DE_PIN, HIGH);
    delayMicroseconds(20);
    Serial1.write(frame, sizeof(frame));
    Serial1.flush();
    delayMicroseconds(20);
    digitalWrite(RS485_DE_PIN, LOW);
  }
  static uint32_t lastPrintMs = 0;
  if (verbose && millis() - lastPrintMs >= 200 && Serial && Serial.availableForWrite() >= 160) {
    lastPrintMs = millis();
    // Unfiltered angle from this sample window, for comparison with the filtered one.
    int32_t x = 0, y = 0;
    Serial.print("permille");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      Serial.printf(" %u", irFilter.raw(i));
      x += int32_t(irFilter.raw(i)) * cosTable[i];
      y += int32_t(irFilter.raw(i)) * sinTable[i];
    }
    const int raw = x || y ? (int(lroundf(atan2f(float(y), float(x)) * 57.29578f)) + 360) % 360 : -1;
    Serial.printf("\nraw=%d filtered=%u strength=%u confidence=%u faults=%03x\n", raw, result.angle,
                  result.strength, result.confidence, result.faults);
  }
}

void processRequestByte(uint8_t data) {
  // MAINのA5 5A command terminatorを状態機械で1 byteずつ読み、完全一致した要求だけ実行する。
  static uint8_t command = 1;
  switch (requestState) {
    case WAIT_POLL: if (data == 0xA5) requestState = READ_POLL_2; break;
    case READ_POLL_2:
      requestState = data == 0x5A ? READ_POLL_CMD : (data == 0xA5 ? READ_POLL_2 : WAIT_POLL);
      break;
    case READ_POLL_CMD:
      command = data;
      requestState = data == 0x01 || data == 0x02 ? READ_POLL_END : WAIT_POLL;
      break;
    case READ_POLL_END:
      requestState = WAIT_POLL;
      if (command == 0x01 && data == 0xFE) sampleAndReply();
      if (command == 0x02 && data == 0xFD) sampleFiltered(true);
      break;
  }
}

void setup() {
  // センサー入力、RS-485方向制御、UARTを初期化。起動時は必ず受信状態にする。
  Serial.begin(115200);
  for (uint8_t pin : sensorPins) pinMode(pin, INPUT);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  Serial1.setTX(RS485_TX_PIN);
  Serial1.setRX(RS485_RX_PIN);
  Serial1.begin(RS485_BAUD);
}

void loop() {
  // 通信要求を優先して処理し、USB診断はバスが静かなときだけローカル測定する。
  static uint32_t lastBusMs = 0;
  while (Serial1.available()) {
    processRequestByte((uint8_t)Serial1.read());
    lastBusMs = millis();
  }
  while (Serial.available()) if (Serial.read() == 'v') verbose = !verbose;
  // Bench diagnostics without MAIN: sample locally while the bus is quiet; nothing is transmitted.
  if (verbose && millis() - lastBusMs >= 500) sampleFiltered(false);
}
