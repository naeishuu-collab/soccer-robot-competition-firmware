#include <Arduino.h>

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

enum RequestState { WAIT_POLL, READ_POLL_2, READ_POLL_CMD, READ_POLL_END };
RequestState requestState = WAIT_POLL;

void sendIR(uint16_t direction, uint16_t totalDetect) {
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
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) detectCount[i] = 0;
  uint32_t startTime = micros();
  while ((uint32_t)(micros() - startTime) < SAMPLE_TIME_US) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      if (digitalRead(sensorPins[i]) == LOW) detectCount[i]++;
    }
  }

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

void processRequestByte(uint8_t data) {
  switch (requestState) {
    case WAIT_POLL: if (data == 0xA5) requestState = READ_POLL_2; break;
    case READ_POLL_2:
      requestState = data == 0x5A ? READ_POLL_CMD : (data == 0xA5 ? READ_POLL_2 : WAIT_POLL);
      break;
    case READ_POLL_CMD: requestState = data == 0x01 ? READ_POLL_END : WAIT_POLL; break;
    case READ_POLL_END:
      requestState = WAIT_POLL;
      if (data == 0xFE) sampleAndReply();
      break;
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t pin : sensorPins) pinMode(pin, INPUT);
  pinMode(RS485_DE_PIN, OUTPUT);
  digitalWrite(RS485_DE_PIN, LOW);
  Serial1.setTX(RS485_TX_PIN);
  Serial1.setRX(RS485_RX_PIN);
  Serial1.begin(RS485_BAUD);
}

void loop() {
  while (Serial1.available()) processRequestByte((uint8_t)Serial1.read());
}
