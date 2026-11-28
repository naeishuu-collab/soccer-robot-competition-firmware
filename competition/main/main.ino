#include <BoardIO.h>
#include <SerialPIO.h>
#include "control.h"

using namespace robot;
constexpr uint8_t BusEnable = 8;
constexpr uint32_t MotorRefreshMs = 30;
#ifndef RCJ_STARTUP_DELAY_MS
#define RCJ_STARTUP_DELAY_MS 3000
#endif
uint32_t bootMs = 0;

SerialPIO cameraSerial(2, 3, 256);
Receiver<CameraSize, 0xc6, 0x6c> cameraRx;
Camera camera;
ChasePD pd;
Decision decision;
UsbReport<1536> report;
Pwm transmitted{};
uint8_t alternateSent = 0;
RunMode runMode = RCJ_BASIC_CHASE ? RunMode::BasicChase :
                  RCJ_MANAGED_COMPETITION ? RunMode::ManagedCompetition : RunMode::Competition;
#if RCJ_BASIC_CHASE
constexpr uint32_t BasicLeaseMs = 200, BasicMaxRunMs = 2000;
uint32_t basicLeaseMs = 0, basicStartMs = 0;
int basicCap = 0;
bool basicArmed = false;
#endif
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
constexpr uint32_t BenchLeaseMs = 200;
uint32_t benchMs = 0;
bool benchArmed = false;
int benchWheel = -1;
#endif
uint32_t pollInterval = 0, pollMax = 0;
bool sentAheadOfPoll = false, lastTxWasEscape = false;
uint32_t cameraMs = 0, irMs = 0, pollMs = 0, pollUs = 0, releaseUs = 0;
uint32_t cameraUs = 0, escapeRxToTxUs = 0, escapeRxToTxMaxUs = 0;
uint32_t motorMs = 0, motorTxCount = 0, cameraBad = 0, irBad = 0, reportMs = 0;
uint16_t irAngle = Unknown, irStrength = 0;
bool manual = RCJ_WHEELS_LIFTED_DIAGNOSTICS || RCJ_BASIC_CHASE;
bool haveCamera = false, cameraLive = false, haveIr = false, irInvalid = false, haveBall = false;
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
      cameraUs = micros();
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
      haveBall = true;
    } else {
      haveBall = false;
      pd.reset();
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
  bootMs = millis();
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
#if RCJ_BASIC_CHASE
      if (command == 'g' && !manual) continue;
#endif
      manual = command == 's';
      if (manual) { pd.reset(); haveBall = false; }
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
      benchArmed = false;
#endif
#if RCJ_BASIC_CHASE
      basicArmed = false;
      if (manual) basicCap = 0;
#endif
      break;  // Apply a stop before consuming another operator command.
    }
    if (command == 'i') requestedReport = true;
#if RCJ_BASIC_CHASE
    if (manual && command >= '1' && command <= '3') {
      const int caps[] = {140, 175, 210};
      basicCap = caps[command - '1'];
      requestedReport = true;
    }
    if (!manual && basicCap && command == 'R' && Serial) {
      const uint32_t commandMs = millis();
      if (basicArmed && (commandMs - basicLeaseMs >= BasicLeaseMs ||
                         commandMs - basicStartMs >= BasicMaxRunMs)) {
        basicArmed = false;
        basicCap = 0;
        manual = true;
      } else {
        if (!basicArmed) { basicStartMs = commandMs; basicArmed = true; }
        basicLeaseMs = commandMs;
      }
    }
#endif
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
    if (manual && (command == 'M' || command == 'P' || command == 'C')) {
      runMode = command == 'M' ? RunMode::MotorTest : command == 'P' ? RunMode::IrPdTest : RunMode::Competition;
      benchWheel = -1;
      requestedReport = true;
      break;
    }
    if (manual && runMode == RunMode::MotorTest && command >= '1' && command <= '6')
      benchWheel = command - '1';
    if (!manual && runMode != RunMode::Competition && command == 'R') {
      benchMs = millis();
      benchArmed = true;
    }
#endif
  }
  readSensors();
  const uint32_t now = millis();
  if (now - cameraMs >= CameraTimeoutMs) cameraLive = false;
  const bool irFresh = haveIr && now - irMs < IrTimeoutMs;
  if (!irFresh) {
    haveBall = false;
    pd.reset();
  }
  decision = decide(manual, cameraLive, camera, irFresh, irInvalid, missedIr, haveBall, pd.pwm, pd.error);

#if RCJ_MANAGED_COMPETITION
  decision = basicDecision(true, 210, manual, cameraLive, camera,
                           irFresh, irInvalid, missedIr, haveBall, pd.pwm);
#endif

#if RCJ_BASIC_CHASE
  if (basicArmed && (!Serial || now - basicLeaseMs >= BasicLeaseMs ||
      now - basicStartMs >= BasicMaxRunMs || !cameraLive || !irFresh || irInvalid || missedIr ||
      !(camera.flags & Ready) || (camera.flags & Slow))) {
    basicArmed = false;
    manual = true;
    basicCap = 0;
  }
  decision = basicDecision(basicArmed, basicCap, manual, cameraLive, camera,
                           irFresh, irInvalid, missedIr, haveBall, pd.pwm);
#endif

#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
  if (runMode != RunMode::Competition) {
    if (now - benchMs >= BenchLeaseMs || !cameraLive || !irFresh || irInvalid || missedIr)
      benchArmed = false;
    decision = benchDecision(runMode, benchArmed, benchWheel, haveBall, pd.pwm,
                             manual, cameraLive, irFresh, irInvalid, missedIr);
  }
#endif

#if !RCJ_BASIC_CHASE && !RCJ_WHEELS_LIFTED_DIAGNOSTICS
  if (!manual && now - bootMs < RCJ_STARTUP_DELAY_MS) {
    decision = Decision{};
    decision.reason = StopReason::Startup;
  }
#endif

  if (!waiting && micros() - releaseUs >= BusGuardUs) {
    const bool duePoll = !irStarted || now - pollMs >= PollMs;
    const bool changed = memcmp(decision.pwm, transmitted, sizeof(transmitted)) ||
                         decision.alternate != alternateSent;
    const bool urgentStop = decision.state == State::Stop && lastCommandMoving;
    const bool urgentEscape = decision.state == State::Escape && changed;
    if (!motorTxCount || urgentStop || (urgentEscape && !lastTxWasEscape) || ((changed || now - motorMs >= MotorRefreshMs) &&
                                      (!duePoll || !sentAheadOfPoll))) {
      uint8_t packet[MotorSize];
      encodeMotor(decision.pwm, packet, decision.alternate);
      rs485Send(BusEnable, packet, sizeof(packet));
      if (urgentEscape) {
        escapeRxToTxUs = micros() - cameraUs;
        if (escapeRxToTxUs > escapeRxToTxMaxUs) escapeRxToTxMaxUs = escapeRxToTxUs;
      }
      memcpy(transmitted, decision.pwm, sizeof(transmitted));
      alternateSent = decision.alternate;
      lastTxWasEscape = decision.state == State::Escape;
      sentAheadOfPoll = duePoll;
      motorMs = millis();
      ++motorTxCount;
      releaseUs = micros();
    } else if (duePoll) {
      rs485Send(BusEnable, Poll, sizeof(Poll));
      pollUs = micros();
      const uint32_t sentMs = millis();
      if (irStarted) {
        pollInterval = sentMs - pollMs;
        if (pollInterval > pollMax) pollMax = pollInterval;
      }
      pollMs = sentMs;
      sentAheadOfPoll = false;
      waiting = irStarted = true;
      irLength = 0;
      slotMalformed = false;
    }
  }
  logEvent(now);
  diagnose(now, cameraLive, irFresh);
}
