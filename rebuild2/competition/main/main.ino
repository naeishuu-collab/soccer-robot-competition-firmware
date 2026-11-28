#include <BoardIO.h>
#include <SerialPIO.h>
#include "control.h"
#include "goal_correction.h"
#include "orbit_candidate.h"

using namespace robot;
#ifndef RCJ_SERIAL_TRIAL
#define RCJ_SERIAL_TRIAL 1
#endif
#if RCJ_SERIAL_TRIAL && (RCJ_BASIC_CHASE || RCJ_WHEELS_LIFTED_DIAGNOSTICS)
#error Serial trial requires its own build
#endif
#ifndef RCJ_START_WAIT_MS
#define RCJ_START_WAIT_MS 20
#endif
#if RCJ_SERIAL_TRIAL && RCJ_START_WAIT_MS > 0
bool startPending = false, startRequiresUsb = false;
uint32_t startRequestMs = 0;
uint8_t startEvents[8]{}, startEventReasons[8]{}, startEventRead = 0, startEventWrite = 0;
void startEvent(uint8_t event, uint8_t reason = 0) {
  uint8_t next = (startEventWrite + 1) % 8;
  if (next == startEventRead) startEventRead = (startEventRead + 1) % 8;
  startEvents[startEventWrite] = event; startEventReasons[startEventWrite] = reason; startEventWrite = next;
}
enum StartCancelReason : uint8_t { StartCancelledByStop = 1, StartCancelledByMode, StartCancelledByUsb,
  StartCancelledByCamera, StartCancelledByIr };
void cancelStart(uint8_t reason) { if (startPending) { startPending = false; startEvent(3, reason); } }
#endif
#if RCJ_SERIAL_TRIAL
bool trialArmed = false;
#endif
constexpr uint8_t BusEnable = 8;
constexpr uint32_t MotorRefreshMs = 30;
#ifndef RCJ_STARTUP_DELAY_MS
#define RCJ_STARTUP_DELAY_MS 3000
#endif
uint32_t bootMs = 0;

SerialPIO cameraSerial(2, 3, 256);
Receiver<CameraSize, 0xc6, 0x6c> cameraRx;
Camera camera;
bool targetBlue = true;
int8_t ownGoalTurn = 0;
#if RCJ_PHASE3_ORBIT
OrbitCandidate orbit;
#endif
// Frame time includes capture and scan. Reject saturated or stale observations.
constexpr uint32_t ObservationMaxMs = 200, BearingMemoryMs = 150;
uint32_t bearingMs = 0;
uint16_t lastLineBearing = Unknown;
bool haveLineBearing = false;
ChasePD pd;
Decision decision;
UsbReport<2048> report;
Pwm transmitted{};
uint8_t alternateSent = 0;
RunMode runMode = RunMode::Competition;
#if RCJ_BASIC_CHASE
constexpr uint32_t BasicLeaseMs = 200, BasicMaxRunMs = 2000;
uint32_t basicLeaseMs = 0, basicStartMs = 0;
int basicCap = 0;
bool basicArmed = false;
#endif
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
constexpr uint32_t BenchLeaseMs = 150;
uint32_t benchMs = 0, benchLastDuration = 0;
bool benchArmed = false, benchFired = false;
int benchWheel = -1, benchSign = 1;
Pwm benchLastPwm{};
#endif
uint32_t pollInterval = 0, pollMax = 0;
bool sentAheadOfPoll = false, lastTxWasEscape = false;
uint32_t cameraMs = 0, irMs = 0, pollMs = 0, pollUs = 0, releaseUs = 0;
uint32_t cameraUs = 0, escapeRxToTxUs = 0, escapeRxToTxMaxUs = 0;
uint32_t motorMs = 0, motorTxCount = 0, cameraBad = 0, irBad = 0, reportMs = 0;
uint16_t irAngle = Unknown, irStrength = 0;
bool manual = RCJ_WHEELS_LIFTED_DIAGNOSTICS || RCJ_BASIC_CHASE || RCJ_SERIAL_TRIAL;
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
      if ((sample.flags & Ready) && sample.frameMs < ObservationMaxMs) {
        if (!(sample.flags & (Escape | LineHazard))) haveLineBearing = false;
        else if (sample.line < 360) {
          lastLineBearing = sample.line;
          bearingMs = cameraMs - sample.frameMs;
          haveLineBearing = true;
        }
      }
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
#if RCJ_SERIAL_TRIAL && RCJ_START_WAIT_MS > 0
  startPending = true;
  startRequiresUsb = false;
  startRequestMs = bootMs;
  startEvent(1);
#endif
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
#if RCJ_SERIAL_TRIAL
    if (command == 'h') Serial.println("g start; s stop; b blue; y yellow; i info");
    if (command == 'b' || command == 'y') {
      targetBlue = command == 'b'; ownGoalTurn = 0; requestedReport = true;
#if RCJ_PHASE3_ORBIT
      orbit = OrbitCandidate{};
#endif
      continue;
    }
#endif
#if RCJ_SERIAL_TRIAL && RCJ_START_WAIT_MS > 0
    if (command == 'g') {
      if (manual && !startPending) { startPending = true; startRequiresUsb = true; startRequestMs = millis(); startEvent(1); }
      continue;
    }
    if (command == 's') cancelStart(StartCancelledByStop);
#endif
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
    if (manual && (command == 'b' || command == 'y')) {
      targetBlue = command == 'b'; requestedReport = true; continue;
    }
    if (command == 'g') {
      if (manual && runMode == RunMode::MotorTest && benchWheel >= 0 && Serial) {
        manual = false; benchFired = false; benchArmed = false; benchMs = millis();
      }
      continue;
    }
#endif
    if (command == 's' || command == 'g') {
#if RCJ_BASIC_CHASE
      if (command == 'g' && !manual) continue;
#endif
#if RCJ_SERIAL_TRIAL
      if (command == 'g' && !manual) continue;
      trialArmed = command == 'g' && runMode != RunMode::Competition && runMode != RunMode::ManagedCompetition;
#endif
      manual = command == 's';
      if (manual) { pd.reset(); haveBall = false; }
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
      if (benchArmed) benchLastDuration = millis() - benchMs;
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
    if (manual && (command == 'M' || command == 'C')) {
      runMode = command == 'M' ? RunMode::MotorTest : RunMode::Competition;
      benchWheel = -1;
      requestedReport = true;
      break;
    }
    if (manual && runMode == RunMode::MotorTest && command >= '0' && command <= '9')
      benchWheel = command == '0' ? 9 : command - '1';
    if (manual && runMode == RunMode::MotorTest && (command == '+' || command == '-'))
      benchSign = command == '+' ? 1 : -1;
    if (!manual && runMode == RunMode::MotorTest && command == 'R' && !benchFired && Serial) {
      benchMs = millis(); benchArmed = true; benchFired = true; benchLastDuration = 0;
    }
#endif
  }
  readSensors();
  const uint32_t now = millis();
  if (now - cameraMs >= CameraTimeoutMs) cameraLive = false;
  const bool observationFresh = cameraLive && (camera.flags & Ready) &&
      camera.frameMs < ObservationMaxMs && now - cameraMs + camera.frameMs < ObservationMaxMs;
  Camera controlCamera = camera;
  const uint16_t reported = camera.escape & (Front | Rear);
  if ((camera.flags & (Escape | LineHazard)) && camera.line == Unknown &&
      reported != Front && reported != Rear && haveLineBearing && now - bearingMs < BearingMemoryMs)
    controlCamera.line = lastLineBearing;
  const bool irFresh = haveIr && now - irMs < IrTimeoutMs;
  if (!irFresh) {
    haveBall = false;
    pd.reset();
  }
#if RCJ_SERIAL_TRIAL && RCJ_START_WAIT_MS > 0
  if (startPending && startRequiresUsb && !Serial) cancelStart(StartCancelledByUsb);
  if (startPending && now - startRequestMs >= RCJ_START_WAIT_MS && now - bootMs >= RCJ_STARTUP_DELAY_MS) {
    // Keep the start request pending until both sensor links and camera readiness return.
    if (observationFresh &&
        irFresh && !irInvalid && !missedIr) {
      startPending = false; manual = false; startEvent(2);
    }
  }
#endif
#if RCJ_WHEELS_LIFTED_DIAGNOSTICS
  // Lifted-wheel diagnostics only: loss of a link, USB or a line hazard cancels the pulse.
  if (!Serial || !observationFresh || !irFresh || irInvalid || missedIr ||
      (controlCamera.flags & (Escape | LineHazard)) ||
      (benchArmed && now - benchMs >= BenchLeaseMs) ||
      (!manual && !benchFired && now - benchMs >= 2000)) {
    if (benchArmed) benchLastDuration = now - benchMs;
    benchArmed = false; manual = true;
  }
  decision = benchDecision(runMode, benchArmed, benchWheel < 6 ? benchWheel : 0,
      haveBall, pd.pwm, manual, observationFresh, irFresh, irInvalid, missedIr);
  if (decision.state == State::Chase && runMode == RunMode::MotorTest) {
    const int16_t duty = int16_t(120 * benchSign);
    if (benchWheel < 4) { for (auto& value : decision.pwm) value = 0; decision.pwm[benchWheel] = duty; }
    else if (benchWheel < 6) {
      const int16_t side = benchWheel == 4 ? duty : -duty;
      decision.pwm[0] = decision.pwm[2] = -side; decision.pwm[1] = decision.pwm[3] = side;
    } else if (benchWheel < 8) {
      const int16_t straight = benchWheel == 6 ? duty : -duty;
      for (auto& value : decision.pwm) value = straight;
    } else {
      const int16_t turn = benchWheel == 8 ? duty : -duty;
      decision.pwm[0] = decision.pwm[1] = -turn; decision.pwm[2] = decision.pwm[3] = turn;
    }
    memcpy(benchLastPwm, decision.pwm, sizeof(benchLastPwm));
  }
#else
  decision = decide(manual, observationFresh, controlCamera, irFresh, irInvalid, missedIr, haveBall, pd.pwm, pd.error);
  Camera goalCamera = camera;
  if (targetBlue) goalCamera.flags |= TargetBlue;
  else goalCamera.flags &= ~TargetBlue;
  #if RCJ_PHASE3_ORBIT
  if (!orbit.update(decision, goalCamera, now - cameraMs, pd.error, now))
  #endif
  if (!avoidOwnGoal(decision, goalCamera, now - cameraMs, pd.error, ownGoalTurn))
    correctGoal(decision, goalCamera, now - cameraMs, pd.error);
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
