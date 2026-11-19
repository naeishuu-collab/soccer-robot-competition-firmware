
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

// ============================================
// MOTOR基板テスト
// RP2354B + TB67H450 x 4
// USBシリアルコマンドテスト
// ============================================

// 実機動作確認済みGPIO
const uint8_t IN1[4] = {6, 12, 11, 9};
const uint8_t IN2[4] = {7, 13, 10, 8};

// PWM設定
const uint32_t PWM_FREQ = 20000;
const int PWM_MAX = 255;
const int TEST_PWM = 200;

// 安全設定
const uint32_t TEST_DURATION_MS = 5000;
const uint32_t REVERSE_WAIT_MS = 200;
const uint32_t RAMP_INTERVAL_MS = 10;
const int RAMP_STEP = 5;

// コマンド入力
char inputBuffer[48];
uint8_t inputIndex = 0;
bool inputOverflow = false;

// モーター状態
int16_t target[4] = {};
int16_t current[4] = {};
uint32_t reverseUntil[4] = {};
uint32_t lastRamp[4] = {};
bool reversing[4] = {};

bool testActive = false;
uint32_t testStartedMs = 0;

// ============================================
// モーター出力
// ============================================

void motorOutput(uint8_t i, int16_t value) {
  if (value == 256) {
    analogWrite(IN1[i], 0);
    analogWrite(IN2[i], 0);
    digitalWrite(IN1[i], HIGH);
    digitalWrite(IN2[i], HIGH);
    return;
  }

  if (value == 0) {
    analogWrite(IN1[i], 0);
    analogWrite(IN2[i], 0);
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);
    return;
  }

  if (value > 0) {
    analogWrite(IN2[i], 0);
    digitalWrite(IN2[i], LOW);
    analogWrite(IN1[i], value);
  } else {
    analogWrite(IN1[i], 0);
    digitalWrite(IN1[i], LOW);
    analogWrite(IN2[i], -value);
  }
}

// ============================================
// 全停止
// ============================================

void stopAll() {
  for (uint8_t i = 0; i < 4; i++) {
    target[i] = 0;
    current[i] = 0;
    reversing[i] = false;
    reverseUntil[i] = 0;
    motorOutput(i, 0);
  }

  testActive = false;
}

// ============================================
// モーター更新
// ============================================

void updateMotor(uint8_t i, uint32_t now) {
  int16_t t = target[i];
  int16_t c = current[i];

  if (t == 0) {
    current[i] = 0;
    reversing[i] = false;
    motorOutput(i, 0);
    return;
  }

  if (reversing[i]) {
    if ((int32_t)(now - reverseUntil[i]) < 0) {
      return;
    }

    reversing[i] = false;
    c = 0;
  }

  if (c != 0 && ((c > 0) != (t > 0))) {
    current[i] = 0;
    motorOutput(i, 0);
    reversing[i] = true;
    reverseUntil[i] = now + REVERSE_WAIT_MS;
    return;
  }

  if (abs(t) <= 200) {
    current[i] = t;
    motorOutput(i, t);
    return;
  }

  if (abs(c) < 200) {
    c = t > 0 ? 200 : -200;
    current[i] = c;
    motorOutput(i, c);
    lastRamp[i] = now;
    return;
  }

  if (now - lastRamp[i] < RAMP_INTERVAL_MS) {
    return;
  }

  lastRamp[i] = now;

  if (c < t) {
    c = min((int)c + RAMP_STEP, (int)t);
  } else if (c > t) {
    c = max((int)c - RAMP_STEP, (int)t);
  }

  current[i] = c;
  motorOutput(i, c);
}

// ============================================
// テスト開始
// ============================================

void startTest(uint8_t motorIndex, int direction) {
  stopAll();

  target[motorIndex] = direction * TEST_PWM;
  testStartedMs = millis();
  testActive = true;

  Serial.print(direction > 0 ? "FORWARD MOTOR " : "REVERSE MOTOR ");
  Serial.println(motorIndex + 1);
}

// ============================================
// コマンド処理
// ============================================

void processCommand(char* command) {
  // 末尾の空白を削除
  size_t len = strlen(command);

  while (len > 0 && command[len - 1] == ' ') {
    command[--len] = '\0';
  }

  // 小文字を大文字へ変換
  for (size_t i = 0; i < len; i++) {
    if (command[i] >= 'a' && command[i] <= 'z') {
      command[i] -= 32;
    }
  }

  if (strcmp(command, "STOP ALL") == 0 ||
      strcmp(command, "STOP") == 0 ||
      strcmp(command, "S") == 0) {
    stopAll();
    Serial.println("STOP ALL");
    return;
  }

  if (strcmp(command, "HELP") == 0) {
    Serial.println("FORWARD MOTOR 1-4");
    Serial.println("REVERSE MOTOR 1-4");
    Serial.println("STOP ALL");
    Serial.println("BRAKE ALL");
    return;
  }

  if (strcmp(command, "BRAKE ALL") == 0) {
    stopAll();

    for (uint8_t i = 0; i < 4; i++) {
      motorOutput(i, 256);
    }

    Serial.println("BRAKE ALL");
    return;
  }

  const char* forwardPrefix = "FORWARD MOTOR ";
  const char* reversePrefix = "REVERSE MOTOR ";

  int direction = 0;
  const char* number = nullptr;

  if (strncmp(command, forwardPrefix, strlen(forwardPrefix)) == 0) {
    direction = 1;
    number = command + strlen(forwardPrefix);
  } else if (strncmp(command, reversePrefix, strlen(reversePrefix)) == 0) {
    direction = -1;
    number = command + strlen(reversePrefix);
  }

  if (number != nullptr &&
      number[0] >= '1' &&
      number[0] <= '4' &&
      number[1] == '\0') {
    startTest(number[0] - '1', direction);
    return;
  }

  Serial.println("INVALID COMMAND. Type HELP");
}

// ============================================
// USBシリアル受信
// ============================================

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      inputBuffer[inputIndex] = '\0';

      if (inputOverflow) {
        Serial.println("INPUT TOO LONG");
      } else if (inputIndex > 0) {
        processCommand(inputBuffer);
      }

      inputIndex = 0;
      inputOverflow = false;
      continue;
    }

    if (inputIndex < sizeof(inputBuffer) - 1) {
      inputBuffer[inputIndex++] = c;
    } else {
      inputOverflow = true;
    }
  }
}

// ============================================
// 初期化
// ============================================

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(IN1[i], LOW);
    digitalWrite(IN2[i], LOW);

    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
  }

  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_MAX);

  stopAll();

  Serial.println();
  Serial.println("MOTOR TEST READY");
  Serial.println("Type HELP for commands");
  Serial.println("Automatic stop after 5 seconds");
}

// ============================================
// メインループ
// ============================================

void loop() {
  readSerial();

  uint32_t now = millis();

  // 5秒経過で自動停止
  if (testActive &&
      now - testStartedMs >= TEST_DURATION_MS) {
    stopAll();
    Serial.println("AUTO STOP");
  }

  if (testActive) {
    for (uint8_t i = 0; i < 4; i++) {
      updateMotor(i, now);
    }
  }
}