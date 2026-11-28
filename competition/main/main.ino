#include <BoardIO.h>
#include <SerialPIO.h>
#include "control.h"

using namespace robot;
constexpr uint8_t BusEnable = 8;
constexpr uint32_t BallHoldMs = 45, MotorRefreshMs = 30;

SerialPIO cameraSerial(2, 3, 256);
Receiver<CameraSize, 0xc6, 0x6c> cameraRx;
Camera camera;
ChasePD pd;
Decision decision;
UsbReport<1024> report;
Pwm transmitted{};
uint32_t cameraMs = 0, irMs = 0, ballMs = 0, pollMs = 0, pollUs = 0, releaseUs = 0;
uint32_t motorMs = 0, motorTxCount = 0, cameraBad = 0, irBad = 0, reportMs = 0;
uint16_t irAngle = Unknown, irStrength = 0;
bool manual = false, haveCamera = false, cameraLive = false, haveIr = false, irInvalid = false, haveBall = false;
bool missedIr = false, slotMalformed = false;
bool waiting = false, irStarted = false, requestedReport = false;
uint8_t irBytes[6]{}, irLength = 0;

#include "diagnostics.h"

void readSensors() {
  for (int n = 0; n < 96 && cameraSerial.available(); ++n) {
    if (!cameraRx.push(uint8_t(cameraSerial.read()), micros())) continue;
    Camera sample;
    if (decodeCamera(cameraRx.bytes, sample) != Decode::Ok) {
      ++cameraBad;
      cameraRx.resync();
    } else if (!haveCamera || sample.sequence != camera.sequence) {
      camera = sample;
      cameraMs = millis();
      haveCamera = cameraLive = true;
    }
  }
  while (Serial1.available()) {
    const uint8_t byte = uint8_t(Serial1.read());
    if (!waiting || micros() - pollUs >= IrSlotUs) continue;
    if (!irLength && byte != 0xaa) continue;
    irBytes[irLength++] = byte;
    if (irLength != sizeof(irBytes)) continue;
    irLength = 0;
    const uint16_t direction = read16(irBytes + 1), strength = read16(irBytes + 3);
    if (byte != 0x55 || !bearing(direction) || (!strength && direction != 0 && direction != Unknown)) {
      irInvalid = true;
      haveBall = false;
      pd.reset();
      ++irBad;
      slotMalformed = true;
      continue;  // Keep the slot reserved when the complete reply is uncertain.
    }
    waiting = false;
    releaseUs = micros();
    haveIr = true;
    irInvalid = missedIr = false;
    irMs = millis();
    irAngle = direction;
    irStrength = strength;
    if (direction < 360 && strength) {
      pd.update(direction, strength, irMs);
      ballMs = irMs;
      haveBall = true;
    } else {
      // Restart D on reacquisition, while keeping the last PWM on a short zero-count miss.
      pd.initialized = false;
      if (strength) { haveBall = false; pd.reset(); }  // vector cancellation
    }
  }
  if (waiting && micros() - pollUs >= IrSlotUs) {
    waiting = false;
    releaseUs = micros();
    missedIr = !slotMalformed;
    if (missedIr) { haveBall = false; pd.reset(); }
  }
}

void setup() {
  digitalWrite(11, LOW);
  pinMode(11, OUTPUT);  // Existing kicker output stays low.
  rs485Begin(BusEnable);
  cameraSerial.begin(Baud);
  Serial.begin(Baud);
  releaseUs = micros();
}

void loop() {
  // A queued 'g' cannot cancel a manual stop before zero reaches the bus.
  const bool lastCommandMoving = transmitted[0] || transmitted[1] || transmitted[2] || transmitted[3];
  for (int n = 0; n < 16 && !(manual && lastCommandMoving) && Serial.available(); ++n) {
    const int command = Serial.read();
    if (command == 's' || command == 'g') {
      manual = command == 's';
      if (manual) { pd.reset(); haveBall = false; }
      break;  // Apply a stop before consuming another operator command.
    }
    if (command == 'i') requestedReport = true;
  }
  readSensors();
  const uint32_t now = millis();
  if (now - cameraMs >= CameraTimeoutMs) cameraLive = false;
  const bool irFresh = haveIr && now - irMs < IrTimeoutMs;
  if (!irFresh || (haveBall && now - ballMs >= BallHoldMs)) {
    haveBall = false;
    pd.reset();
  }
  decision = decide(manual, cameraLive, camera, irFresh, irInvalid, missedIr, haveBall, pd.pwm);

  if (!waiting && micros() - releaseUs >= BusGuardUs) {
    if (!motorTxCount || memcmp(decision.pwm, transmitted, sizeof(transmitted)) ||
        now - motorMs >= MotorRefreshMs) {
      uint8_t packet[MotorSize];
      encodeMotor(decision.pwm, packet);
      rs485Send(BusEnable, packet, sizeof(packet));
      memcpy(transmitted, decision.pwm, sizeof(transmitted));
      motorMs = millis();
      ++motorTxCount;
      releaseUs = micros();
    } else if (!irStarted || now - pollMs >= PollMs) {
      rs485Send(BusEnable, Poll, sizeof(Poll));
      pollUs = micros();
      pollMs = millis();
      waiting = irStarted = true;
      irLength = 0;
      slotMalformed = false;
    }
  }
  logEvent(now);
  diagnose(now, cameraLive, irFresh);
}
