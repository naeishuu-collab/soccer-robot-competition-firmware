// MAIN基板: IRのボール方位とOpenMVの走行可能方向を統合し、MOTOR基板へ移動指令を送る。
// 関連: main_logic.h は拡張制御用の純粋ロジック、ir/ir.ino はIR応答、
// openmv/goal_wall_navigation.py は床・白線の判定を担当する。通常設定は実機確認済み系統。
#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <Wire.h>
#include <SerialPIO.h>
#include "main_logic.h"
#include "zip_chase_pd.h"
#include <RuntimeConfig.h>
#include <Diagnostics.h>

#ifndef ROBOT_ZIP_TRACK
#define ROBOT_ZIP_TRACK 0
#endif
#ifndef ROBOT_TEST_MAX_PWM
#define ROBOT_TEST_MAX_PWM 210
#endif
static_assert(ROBOT_TEST_MAX_PWM >= 140 && ROBOT_TEST_MAX_PWM <= 210, "Invalid test PWM limit");

// 0 = field-tested chase (default). 1 = camera protocol 2, filtered IR, heading modes and tactics.
// Build MAIN, MOTOR and IR together and set PROTOCOL = 2 on the camera. Not verified on hardware.
#ifndef ROBOT_UPGRADE
#define ROBOT_UPGRADE 1
#endif

// Display telemetry to the camera on GP2 (0 = never transmit). USB 'v' toggles the PC viewer lines.
#ifndef ROBOT_TELEMETRY
#define ROBOT_TELEMETRY 1
#endif

// Legacy ZIP motor protocol; default uses raw ZIP IR plus the omni DRIVE protocol.
#ifndef ROBOT_ZIP_COMPAT
#define ROBOT_ZIP_COMPAT 0
#endif

#ifndef ROBOT_RAW_IR
#define ROBOT_RAW_IR 1
#endif

struct CameraState;

// BEGIN HOST-TESTED CAMERA LOGIC
struct CameraState {
  uint8_t mode = 0;
  int8_t x = 0;
  int8_t y = 0;
  uint8_t sectors = 0;
  uint8_t sequence = 0;
  uint32_t receivedMs = 0;
  bool received = false;
};

inline bool validCameraFrame(const uint8_t *f) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < 7; ++i) crc ^= f[i];
  if (f[0] != 0x20 || f[7] != crc) return false;
  const uint8_t mode = f[5];
  if (mode > 2 || f[6] != (mode == 1 ? 0x41 : 0x40)) return false;
  if (mode == 0) return f[1] == 0 && f[2] == 0 && f[3] == 0;
  if (!f[3]) return false;
  if (mode == 2) return f[1] == 0 && f[2] == 0;
  return abs(int(int8_t(f[1]))) <= 35 && abs(int(int8_t(f[2]))) <= 35 && (f[1] || f[2]);
}

inline bool cameraFresh(const CameraState &camera, uint32_t nowMs) {
  return camera.received && uint32_t(nowMs - camera.receivedMs) <= 200;
}

inline bool cameraAllowsMotion(const CameraState &camera, uint32_t nowMs) {
  return cameraFresh(camera, nowMs);
}

inline uint8_t nearestSector(uint16_t direction) {
  return ((direction + 22) / 45) % 8;
}

inline bool sectorClear(const CameraState &camera, uint16_t direction) {
  return camera.sectors & (1u << nearestSector(direction));
}

inline uint16_t cameraAvoidDirection(int8_t x, int8_t y) {
  float angle = atan2f(float(x), float(y)) * 57.2957795f;
  if (angle < 0) angle += 360;
  return uint16_t(angle + 0.5f) % 360;
}

inline float headingDifference(float heading, float reference) {
  float difference = heading - reference;
  if (difference > 180) difference -= 360;
  if (difference < -180) difference += 360;
  return difference;
}

inline int8_t headingCorrection(float reference, float heading, int sign) {
  float error = headingDifference(reference, heading);
  int turn = int(lroundf(error * sign));
  if (turn > 40) turn = 40;
  if (turn < -40) turn = -40;
  return int8_t(turn);
}

// chase (3) reply: 0xAA, big-endian angle, big-endian count, 0x55.
class IrReplyReceiver {
public:
  void reset() { used_ = 0; }

  bool push(uint8_t byte, uint16_t &angle, uint16_t &strength) {
    if (used_ == 0) {
      if (byte == 0xAA) used_ = 1;
      return false;
    }
    if (used_ <= sizeof(payload_)) {
      payload_[used_++ - 1] = byte;
      return false;
    }

    used_ = byte == 0xAA ? 1 : 0;
    if (byte != 0x55) return false;

    const uint16_t candidate = (uint16_t(payload_[0]) << 8) | payload_[1];
    if (candidate >= 360) return false;
    angle = candidate;
    strength = (uint16_t(payload_[2]) << 8) | payload_[3];
    return true;
  }

private:
  uint8_t payload_[4] = {};
  uint8_t used_ = 0;
};

// Confirm sudden bearing changes without losing the last valid IR direction.
class IrBallLock {
public:
  void reset() {
    direction_ = kNoDirection;
   candidate_ = kNoDirection;
  }

  void update(uint32_t now, bool detected, uint16_t direction) {
    if (!detected || direction >= kNoDirection) {
      reset();
      return;
    }
    if (!active(now) || angleDistance(direction, direction_) <= 45 ||
        (candidate_ != kNoDirection && angleDistance(direction, candidate_) <= 25)) {
      direction_ = direction;
      acceptedMs_ = now;
      candidate_ = kNoDirection;
    } else {
      candidate_ = direction;
    }
  }

  bool active(uint32_t now) const {
    return direction_ != kNoDirection && uint32_t(now - acceptedMs_) <= 90;
  }

  uint16_t direction() const { return direction_; }

private:
  static constexpr uint16_t kNoDirection = 360;

  static uint16_t angleDistance(uint16_t a, uint16_t b) {
    const uint16_t delta = a > b ? a - b : b - a;
    return delta > 180 ? 360 - delta : delta;
  }

  uint16_t direction_ = kNoDirection;
  uint16_t candidate_ = kNoDirection;
  uint32_t acceptedMs_ = 0;
};

inline void chooseChaseMotion(const CameraState &camera, uint32_t nowMs, bool ballFound,
                              uint16_t ballDirection, uint16_t movePower, uint16_t avoidPower,
                              uint16_t &direction, uint16_t &power) {
  direction = 0;
  power = 0;
  // Mode 0: the camera cannot confirm any safe direction.
  if (!cameraAllowsMotion(camera, nowMs) || camera.mode == 0) return;
  if (camera.mode == 1 && (camera.x || camera.y)) {
    direction = cameraAvoidDirection(camera.x, camera.y);
  } else if (ballFound && ballDirection < 360) {
    direction = ballDirection;
  } else {
    return;
  }
  // At the configured PWM floor the wheel mixer reliably resolves the eight sector centers.
  direction = uint16_t(nearestSector(direction) * 45);
  if (camera.mode != 0 && camera.sectors && !sectorClear(camera, direction)) {
    for (uint16_t step = 1; step <= 4; ++step) {
      const uint16_t right = (direction + step * 45) % 360;
      const uint16_t left = (direction + 360 - step * 45) % 360;
      if (sectorClear(camera, right)) { direction = right; break; }
      if (sectorClear(camera, left)) { direction = left; break; }
    }
  }
  power = camera.mode == 1 ? avoidPower : movePower;
}
// END HOST-TESTED CAMERA LOGIC

// ============================================================
// MAIN - Direction / Power Controller
// RP2354B
// ============================================================

// RS485
constexpr uint8_t PIN_RS485_TX = 0;
constexpr uint8_t PIN_RS485_RX = 1;
constexpr uint8_t PIN_RS485_EN = 8;
constexpr uint32_t RS485_BAUD = 115200;

// Protocol
constexpr uint8_t FRAME_START = 0xA5;
constexpr uint8_t FRAME_SIZE = 12;

constexpr uint8_t MSG_DRIVE = 3;

// Match the timing of the field-tested IR poll.
constexpr uint32_t IR_POLL_INTERVAL_MS = 30;
constexpr uint32_t IR_REPLY_TIMEOUT_MS = 100;
// Upgrade path: the reply normally arrives in ~6 ms; waiting longer would gap the 100 ms MOTOR watchdog.
constexpr uint32_t UPGRADE_IR_WAIT_MS = 25;

// Movement
// Initial commissioning limits. Verify that both settings move reliably on the floor.
constexpr uint16_t MOVE_POWER = 160;
// Not below the MOTOR running floor: 128 could leave the robot stalled on a line.
constexpr uint16_t AVOID_POWER = 160;
constexpr uint16_t MAX_POWER = 200;

static_assert(AVOID_POWER <= MOVE_POWER && MOVE_POWER <= MAX_POWER,
              "MOVE_POWER exceeds MAX_POWER");

// IR orientation
constexpr int16_t IR_FRONT_DEGREES = 0;
// The raw ZIP angle puts physical GPIO21 (left) at 90 degrees.
constexpr int8_t IR_ANGLE_SIGN = -1;

constexpr uint8_t BNO_ADDRESS = 0x28;
#ifndef ROBOT_BNO_ENABLED
#define ROBOT_BNO_ENABLED 0
#endif
constexpr bool BNO_ENABLED = ROBOT_BNO_ENABLED;
constexpr int8_t HEADING_TURN_SIGN = 1; // Verify direction with wheels raised.
float headingReference = NAN;
float headingCurrent = NAN;
int8_t headingTurn = 0;
uint32_t headingLastReadMs = 0;

void updateHeading(uint32_t now) {
  if (now < 500 || now - headingLastReadMs < 30) return;
  headingLastReadMs = now;
  Wire.beginTransmission(BNO_ADDRESS);
  Wire.write(0x1A);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(BNO_ADDRESS, uint8_t(2)) != 2) {
    headingTurn = 0;
    headingCurrent = NAN;
    return;
  }
  const uint16_t raw = uint16_t(Wire.read()) | (uint16_t(Wire.read()) << 8);
  if (raw >= 5760) { headingTurn = 0; headingCurrent = NAN; return; }
  const float heading = raw / 16.0f;
  headingCurrent = heading;
  if (!isfinite(headingReference)) headingReference = heading;
  headingTurn = headingCorrection(headingReference, heading, HEADING_TURN_SIGN);
}

void printHeading(uint32_t now) {
  static uint32_t lastPrintMs = 0;
  if (now - lastPrintMs < 500 || !Serial) return;
  char line[100];
  int length;
  if (isfinite(headingCurrent) && isfinite(headingReference)) {
    length = snprintf(line, sizeof(line), "BNO heading=%.1f relative=%+.1f reference=%.1f turn=%d\n",
                      headingCurrent, headingDifference(headingCurrent, headingReference),
                      headingReference, int(headingTurn));
  } else {
    length = snprintf(line, sizeof(line), "BNO: waiting for valid data (I2C 0x28)\n");
  }
  if (length > 0 && length < int(sizeof(line)) && Serial.availableForWrite() >= length) {
    Serial.write(reinterpret_cast<const uint8_t *>(line), size_t(length));
    lastPrintMs = now;
  }
}

// ============================================================
// CRC-7
// ============================================================

uint8_t calculateCRC(const uint8_t *frame) {
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

IrReplyReceiver irReceiver;
constexpr uint32_t CAMERA_BAUD = ROBOT_UPGRADE ? 115200 : 9600;
SerialPIO cameraSerial(2, 3, 256);
bool holdStop = false;  // USB 'd' / 's': diagnostics and soft stop, 'g' resumes
bool usbViewer = false;  // USB 'v': print VIZ lines for tools/viewer.html
bool usbDiagnosticRequested = false;
brain::TelemetrySender telemetry;
constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;

robotdiag::StopReason diagnosticStopReason(const char *reason) {
  if (!reason || !*reason) return robotdiag::StopReason::NONE;
  if (strstr(reason, "manual")) return robotdiag::StopReason::MANUAL_STOP;
  if (strstr(reason, "camera link")) return robotdiag::StopReason::CAMERA_TIMEOUT;
  if (strstr(reason, "expired") || strstr(reason, "slow")) return robotdiag::StopReason::CAMERA_SLOW;
  if (strstr(reason, "floor")) return robotdiag::StopReason::CAMERA_NOT_READY;
  if (strstr(reason, "line")) return robotdiag::StopReason::LINE_DETECTED;
  if (strstr(reason, "safe")) return robotdiag::StopReason::NO_SAFE_DIRECTION;
  if (strstr(reason, "IR link")) return robotdiag::StopReason::IR_TIMEOUT;
  if (strstr(reason, "IR sensor")) return robotdiag::StopReason::IR_INVALID;
  if (strstr(reason, "ball")) return robotdiag::StopReason::BALL_LOST;
  return robotdiag::StopReason::INTERNAL_ERROR;
}

CameraState camera;
uint8_t cameraFrame[8] = {};
uint8_t cameraUsed = 0;
uint32_t cameraLastByteMs = 0;

void receiveCamera() {
  // 0x20から8 byteを組み立て、チェック値とモードが正しいフレームだけcamera状態へ反映する。
  const uint32_t now = millis();
  if (cameraUsed && uint32_t(now - cameraLastByteMs) >= 20) cameraUsed = 0;
  while (cameraSerial.available()) {
    const uint8_t byte = cameraSerial.read();
    if (!cameraUsed && byte != 0x20) continue;
    cameraLastByteMs = now;
    cameraFrame[cameraUsed++] = byte;
    if (cameraUsed != sizeof(cameraFrame)) continue;
    if (validCameraFrame(cameraFrame)) {
      cameraUsed = 0;
      if (camera.received && cameraFrame[4] == camera.sequence) continue;  // replays never refresh the link
      const uint8_t mode = cameraFrame[5];
      camera.mode = mode;
      camera.x = int8_t(cameraFrame[1]);
      camera.y = int8_t(cameraFrame[2]);
      camera.sectors = cameraFrame[3];
      camera.sequence = cameraFrame[4];
      camera.receivedMs = now;
      camera.received = true;
    } else {
      uint8_t next = 1;
      while (next < sizeof(cameraFrame) && cameraFrame[next] != 0x20) ++next;
      cameraUsed = sizeof(cameraFrame) - next;
      memmove(cameraFrame, cameraFrame + next, cameraUsed);
    }
  }
}

// ============================================================
// Encoding
// ============================================================

void encodeValue(uint8_t *destination, uint16_t value) {
  destination[0] = (value >> 7) & 0x7F;
  destination[1] = value & 0x7F;
}

void sendRs485(const uint8_t *data, uint8_t size) {
  digitalWrite(PIN_RS485_EN, HIGH);
  delayMicroseconds(20);

  Serial1.write(data, size);
  Serial1.flush();

  delayMicroseconds(20);
  digitalWrite(PIN_RS485_EN, LOW);
}

// ============================================================
// Direction conversion
// ============================================================

uint16_t convertDirection(uint16_t irAngle) {
  int32_t direction =
      IR_ANGLE_SIGN *
      (int32_t(irAngle) - IR_FRONT_DEGREES);

  direction %= 360;

  if (direction < 0) {
    direction += 360;
  }

  return uint16_t(direction);
}

// ============================================================
// DRIVE frame
// ============================================================

void sendDrive(uint8_t sequence, uint16_t direction, uint16_t power, bool avoid = false) {
  // 方位/出力をMOTOR共通形式へ詰めてCRCを付け、RS-485で送信する唯一の通常出力口。
  uint8_t frame[FRAME_SIZE] = {};
  if (holdStop) direction = power = 0;
  frame[0] = FRAME_START;
  frame[1] = MSG_DRIVE;
  frame[2] = sequence & 0x7F;
  encodeValue(&frame[3], direction);
  encodeValue(&frame[5], power);
  frame[7] = power && avoid ? 1 : 0;
  frame[8] = power ? uint8_t(headingTurn + 61) : 0;
  frame[FRAME_SIZE - 1] = calculateCRC(frame);
  sendRs485(frame, sizeof(frame));
}

// ============================================================
// Setup
// ============================================================

void setup() {
  // RS-485/UART/安全出力を初期化する。通常系か拡張系かはloop()で選び、ここでは両者の準備を行う。
  digitalWrite(PIN_RS485_EN, LOW);
  pinMode(PIN_RS485_EN, OUTPUT);
  digitalWrite(PIN_RS485_EN, LOW);

  Serial1.setTX(PIN_RS485_TX);
  Serial1.setRX(PIN_RS485_RX);
  Serial1.begin(RS485_BAUD);

  cameraSerial.begin(CAMERA_BAUD);
  digitalWrite(11, LOW);
  pinMode(11, OUTPUT);

  Serial.begin(115200);
  if (BNO_ENABLED) {
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    Wire.setClock(50000);
    Wire.setTimeout(5, true);
  }

}

// ============================================================
// Visualization (display only: nothing here feeds back into control)
// ============================================================

bool telemetryDue(uint32_t now) {
  static uint32_t lastMs = 0;
  if (!(ROBOT_TELEMETRY || usbViewer) || now - lastMs < TELEMETRY_INTERVAL_MS) return false;
  lastMs = now;
  return true;
}

void publishTelemetry(brain::Telemetry &t, int yellow, int yellowConfidence, int blue, int blueConfidence,
                      const char *tactic, const char *mode) {
  static uint8_t sequence = 0;
  t.sequence = sequence++;
  t.hold = holdStop;
  if (holdStop) t.direction = t.power = 0, t.turn = 0;
  if (ROBOT_TELEMETRY) telemetry.load(t);
  if (usbViewer && Serial && Serial.availableForWrite() >= 120) {
    Serial.printf("VIZ,1,%d,%d,%d,%u,%lu,%d,%u,%d,%s,%s,%u,%d,%d,%d,%d,%d\n", int(t.irLink), int(t.cameraLink), t.ball,
                  t.ball >= 0 ? t.ballConfidence : 0, static_cast<unsigned long>(t.ballAgeMs),
                  t.power ? int(t.direction) : -1, t.power, t.turn, tactic, mode, t.bno, yellow, yellowConfidence, blue,
                  blueConfidence, int(t.hold));
  }
}

// ============================================================
// Upgrade path (ROBOT_UPGRADE=1)
// ============================================================

bool bnoWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BNO_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool bnoRead(uint8_t reg, uint8_t *data, uint8_t size) {
  Wire.beginTransmission(BNO_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(BNO_ADDRESS, size) != size) return false;
  for (uint8_t i = 0; i < size; ++i) data[i] = Wire.read();
  return true;
}

// Probe -> config -> IMUPLUS (magnetometer unused, so motor fields do not steer it) -> read.
// No delay(); a missing sensor costs one I2C timeout (5 ms) per second.
void updateBno(uint32_t now, brain::BnoSample &sample) {
  static uint8_t stage = 0;
  static uint32_t nextMs = 500;
  if (int32_t(now - nextMs) < 0) return;
  uint8_t data[2] = {};
  uint8_t calibration = 0;
  bool ok = false;
  if (stage == 0) {
    ok = bnoRead(0x00, data, 1) && data[0] == 0xA0 && bnoWrite(0x3D, 0x00);
    nextMs = now + 25;
  } else if (stage == 1) {
    ok = bnoWrite(0x3D, 0x08);
    nextMs = now + 30;
  } else {
    ok = bnoRead(0x35, &calibration, 1) && bnoRead(0x1A, data, 2);
    const uint16_t raw = uint16_t(data[0] | data[1] << 8);
    sample.ok = ok && raw < 5760 && ((calibration >> 4) & 3) == 3;  // gyro calibrated
    sample.yaw = raw / 16.0f;
    sample.stampMs = now;
    nextMs = now + 20;
  }
  if (!ok) {
    stage = 0;
    sample.ok = false;
    nextMs = now + 1000;
  } else if (stage < 2) {
    ++stage;
  }
}

void upgradeLoop() {
  // 拡張系の順序: カメラとIRの受信 → BNO読取 → brain.step() → MOTOR送信 → 次のIRポーリング。
  static brain::Brain robot;
  static brain::CameraParser parser;
  static brain::CameraData cameraData;
  static brain::IrReceiver irFrame;
  static IrReplyReceiver rawIrFrame;
  static brain::IrData irData;
  static brain::BnoSample bno;
  static brain::Output out;
  static uint8_t sequence = 0;
  static uint32_t lastPollMs = 0, lastDriveMs = 0, lastLogMs = 0, lastUs = 0, loopMaxUs = 0;
  static uint32_t pollUs = 0, irRoundTripUs = 0, cameraGapMaxMs = 0, brainMaxUs = 0;
  static uint32_t irPolls = 0, irReplies = 0, irTimeouts = 0;
  static bool waitingForIr = false, urgent = false;
  static bool configured = false;
  static uint8_t previousTactic = 0xff;
  static uint8_t transitionFrom = 0xff;
  static const char *transitionReason = "startup";
  static const char *previousStop = nullptr;
  static robotdiag::EventRing<16> events;
  static bool previousIrValid = false, previousCameraValid = false;
  static uint16_t previousPower = 0;
  if (!configured) { robot.cfg.blindTracking = true; configured = true; }

  const uint32_t us = micros();
  if (lastUs && us - lastUs > loopMaxUs) loopMaxUs = us - lastUs;
  lastUs = us;
  telemetry.pump(cameraSerial);

  while (cameraSerial.available()) {
    const uint32_t previous = cameraData.receivedMs;
    if (parser.push(uint8_t(cameraSerial.read()), millis(), cameraData)) {
      if (previous && cameraData.receivedMs - previous > cameraGapMaxMs) cameraGapMaxMs = cameraData.receivedMs - previous;
      if (!cameraData.ready || cameraData.escaping) urgent = true;
    }
  }
  const uint32_t now = millis();
  while (Serial1.available()) {
    const uint8_t byte = Serial1.read();
    brain::IrData reply = irData;
    uint16_t rawAngle = 0, rawStrength = 0;
    const bool received = waitingForIr && (ROBOT_RAW_IR
      ? rawIrFrame.push(byte, rawAngle, rawStrength) : irFrame.push(byte, reply));
    if (received) {
      if (ROBOT_RAW_IR) { reply.angle = rawAngle; reply.strength = rawStrength;
                          reply.confidence = rawStrength ? 100 : 0; reply.faults = 0; }
      reply.received = true;
      reply.angle = convertDirection(reply.angle);
      reply.stampMs = now;
      irData = reply;
      irRoundTripUs = micros() - pollUs;
      ++irReplies;
      waitingForIr = false;
      delayMicroseconds(50);
    }
  }
  if (waitingForIr && now - lastPollMs >= UPGRADE_IR_WAIT_MS) { waitingForIr = false; ++irTimeouts; }
  if (waitingForIr) return;  // never transmit inside the IR reply window

  const bool irValid = irData.received && uint32_t(now - irData.stampMs) <= robot.cfg.irTimeoutMs;
  const bool cameraValid = cameraData.received && uint32_t(now - cameraData.receivedMs) <= robot.cfg.cameraTimeoutMs;
  if (irValid != previousIrValid) {
    events.push(now, robotdiag::EventKind::SENSOR_VALIDITY, 1, irValid ? 1 : 0);
    previousIrValid = irValid;
  }
  if (cameraValid != previousCameraValid) {
    events.push(now, robotdiag::EventKind::SENSOR_VALIDITY, 2, cameraValid ? 1 : 0);
    previousCameraValid = cameraValid;
  }

  if (BNO_ENABLED) {
  updateBno(now, bno);
  } else {
  bno.ok = false;
  }
  if (urgent || now - lastDriveMs >= 20) {
    const uint32_t start = micros();
    out = robot.step(cameraData, irData, bno, millis());
    if (micros() - start > brainMaxUs) brainMaxUs = micros() - start;
    headingTurn = int8_t(out.turn);
    if (holdStop) { out.direction = out.power = 0; out.turn = 0; out.tactic = brain::Tactic::SAFE_STOP; out.reason = "manual stop"; }
    const uint8_t tactic = uint8_t(out.tactic);
    if (tactic != previousTactic) {
      events.push(now, robotdiag::EventKind::STATE, tactic);
      transitionFrom = previousTactic;
      transitionReason = out.reason;
      previousTactic = tactic;
    }
    if (out.reason != previousStop) {
      events.push(now, robotdiag::EventKind::STOP, uint16_t(diagnosticStopReason(out.reason)));
      previousStop = out.reason;
    }
    if (out.power != previousPower && (!out.power || !previousPower || abs(int(out.power) - int(previousPower)) >= 20)) {
      events.push(now, robotdiag::EventKind::PWM, out.direction, int16_t(out.power));
      previousPower = out.power;
    }
    sendDrive(sequence++, out.direction, out.power, out.avoid);
    lastDriveMs = now;
    urgent = false;
  }
  if (telemetryDue(now)) {
    brain::Telemetry t;
    t.upgrade = true;
    t.irLink = irData.received && now - irData.stampMs <= robot.cfg.irTimeoutMs;
    t.cameraLink = cameraData.received && now - cameraData.receivedMs <= robot.cfg.cameraTimeoutMs;
    int faults = 0;
    for (uint16_t f = irData.faults; f; f &= f - 1) ++faults;
    // Only a current IR detection is shown; predictions and old bearings are not sent as the ball.
    if (t.irLink && irData.strength && faults <= robot.cfg.irMaxFaults) {
      t.ball = irData.angle;
      t.ballConfidence = ROBOT_RAW_IR ? 0 : irData.confidence; // raw ZIP reply has no confidence field
      t.ballAgeMs = now - irData.stampMs;
    }
    t.direction = out.direction;
    t.power = out.power;
    t.turn = out.turn;
    t.tactic = uint8_t(out.tactic);
    t.mode = uint8_t(robot.heading.mode);
    const bool gyro = robot.heading.mode == brain::Mode::FUSED || robot.heading.mode == brain::Mode::BNO_ONLY;
    t.bno = !BNO_ENABLED ? 0 : (bno.ok && gyro ? 2 : 1);
    const brain::Seen &y = cameraData.yellow, &bl = cameraData.blue;
    publishTelemetry(t, t.cameraLink && y.seen ? int(y.bearing) : -1, y.confidence,
                     t.cameraLink && bl.seen ? int(bl.bearing) : -1, bl.confidence, brain::name(out.tactic),
                     brain::name(robot.heading.mode));
  }
  if (now - lastPollMs >= IR_POLL_INTERVAL_MS) {
    const uint8_t poll[] = {0xA5, 0x5A, uint8_t(ROBOT_RAW_IR ? 0x01 : 0x02), uint8_t(ROBOT_RAW_IR ? 0xFE : 0xFD)};
    irFrame.reset();
    rawIrFrame.reset();
    sendRs485(poll, sizeof(poll));
    ++irPolls;
    lastPollMs = millis();
    pollUs = micros();
    waitingForIr = true;
  }
  static const char *loggedReason = "startup";
  if (Serial && (strcmp(out.reason, loggedReason) != 0 || now - lastLogMs >= 1000) && Serial.availableForWrite() >= 64) {
    Serial.printf("OMNI c2be6ff+fix %s hold=%u\n", *out.reason ? out.reason : "clear", unsigned(holdStop));
    loggedReason = out.reason;
    lastLogMs = now;
  }
  static uint32_t detailMs = 0;
  static uint8_t detailPage = 0;
  if (Serial && now - detailMs >= 250 && Serial.availableForWrite() >= 64) {
    switch (detailPage++ % 4) {
      case 0: Serial.printf("cam=%lu ready=%u slow=%u esc=%u mask=%04x\n", (unsigned long)(cameraData.received ? now-cameraData.receivedMs : 0xffffffffUL), unsigned(cameraData.ready), unsigned(cameraData.slow), unsigned(cameraData.escaping), unsigned(cameraData.mask)); break;
      case 1: Serial.printf("line=%u count=%u camErr=%lu gap=%lu\n", unsigned(cameraData.line), unsigned(cameraData.lineCount), (unsigned long)parser.errors, (unsigned long)cameraGapMaxMs); break;
      case 2: Serial.printf("ir=%u/%u age=%lu bno=%u\n", unsigned(irData.angle), unsigned(irData.strength), (unsigned long)(irData.received ? now-irData.stampMs : 0xffffffffUL), unsigned(bno.ok)); break;
      default: Serial.printf("drive=%u/%u turn=%d loop=%luus\n", unsigned(out.direction), unsigned(holdStop ? 0 : out.power), holdStop ? 0 : out.turn, (unsigned long)loopMaxUs); break;
    }
    detailMs = now;
  }
  if (usbDiagnosticRequested && Serial && Serial.availableForWrite() >= 220) {
    const uint32_t irAge = irData.received ? now - irData.stampMs : 0xffffffffUL;
    const uint32_t camAge = cameraData.received ? now - cameraData.receivedMs : 0xffffffffUL;
    uint16_t conditions = 0;
    if (holdStop) conditions |= 1u << 0;
    if (!irValid) conditions |= 1u << 1;
    if (irData.faults) conditions |= 1u << 2;
    if (!irData.strength) conditions |= 1u << 3;
    if (!cameraValid) conditions |= 1u << 4;
    if (!cameraData.ready) conditions |= 1u << 5;
    if (cameraData.slow) conditions |= 1u << 6;
    if (cameraData.escaping || cameraData.lineCount) conditions |= 1u << 7;
    if (!cameraData.mask) conditions |= 1u << 8;
    Serial.printf("[MAIN] fw=%s git=%s config=%s uptime_ms=%lu mode=%s state=%s reason=%s hold=%u\n",
                  ROBOT_FW_VERSION, ROBOT_GIT_COMMIT, ROBOT_CONFIG_ID, (unsigned long)now, brain::name(robot.heading.mode),
                  brain::name(out.tactic), *out.reason ? out.reason : "NONE", unsigned(holdStop));
    Serial.printf("[STATE] previous=%u current=%u transition_reason=%s [PD] active=0 (upgrade path)\n",
                  unsigned(transitionFrom), unsigned(out.tactic), transitionReason);
    Serial.printf("[IR] valid=%u angle=%u strength=%u age_ms=%lu [CAMERA] valid=%u ready=%u slow=%u age_ms=%lu line=%u mask=%04x\n",
                  unsigned(irValid), unsigned(irData.angle),
                  unsigned(irData.strength), (unsigned long)irAge,
                  unsigned(cameraData.received), unsigned(cameraData.ready), unsigned(cameraData.slow),
                  (unsigned long)camAge, unsigned(cameraData.lineCount), unsigned(cameraData.mask));
    Serial.printf("[LINE] direction=%u distance_cm=%u BNO=%u/%u [FEATURES] upgrade=%u raw_ir=%u zip_pd=%u zip_compat=%u telemetry=%u\n",
                  unsigned(cameraData.line), unsigned(cameraData.lineCm), unsigned(BNO_ENABLED), unsigned(bno.ok),
                  unsigned(ROBOT_UPGRADE), unsigned(ROBOT_RAW_IR), unsigned(ROBOT_ZIP_TRACK),
                  unsigned(ROBOT_ZIP_COMPAT), unsigned(ROBOT_TELEMETRY));
    Serial.printf("[MOTOR_TX] direction=%u power=%u turn=%d avoid=%u [HISTORY] count=%u overwritten=%lu\n",
                  unsigned(out.direction), unsigned(holdStop ? 0 : out.power), int(holdStop ? 0 : out.turn),
                  unsigned(out.avoid), unsigned(events.size()), (unsigned long)events.overwritten());
    Serial.printf("[LINKS] ir_poll=%lu ir_reply=%lu ir_timeout=%lu ir_rtt_us=%lu camera_crc_err=%lu camera_gap_max_ms=%lu loop_max_us=%lu brain_max_us=%lu\n",
                  (unsigned long)irPolls, (unsigned long)irReplies, (unsigned long)irTimeouts,
                  (unsigned long)irRoundTripUs, (unsigned long)parser.errors, (unsigned long)cameraGapMaxMs,
                  (unsigned long)loopMaxUs, (unsigned long)brainMaxUs);
    Serial.printf("[GOAL] yellow_valid=%u yellow_deg=%.1f yellow_conf=%u blue_valid=%u blue_deg=%.1f blue_conf=%u [NOT_READY_MASK] %03x\n",
                  unsigned(cameraData.yellow.seen), cameraData.yellow.bearing, cameraData.yellow.confidence,
                  unsigned(cameraData.blue.seen), cameraData.blue.bearing, cameraData.blue.confidence, conditions);
    const size_t first = events.size() > 6 ? events.size() - 6 : 0;
    for (size_t i = first; i < events.size(); ++i) {
      const robotdiag::Event &event = events.oldest(i);
      if (event.kind == robotdiag::EventKind::STOP)
        Serial.printf("[STOP_REASON] age_ms=%lu code=%u name=%s\n", (unsigned long)(now - event.atMs),
                      unsigned(event.code), robotdiag::name(robotdiag::StopReason(event.code)));
      else
        Serial.printf("[EVENT] age_ms=%lu kind=%u code=%u value=%d\n",
                      (unsigned long)(now - event.atMs), unsigned(event.kind), unsigned(event.code), int(event.value));
    }
    usbDiagnosticRequested = false;
  }

}

void zipLoop() {
  static brain::CameraParser parser;
  static brain::CameraData cam;
  static IrReplyReceiver receiver;
  static uint32_t pollMs = 0, irMs = 0, logMs = 0;
  static uint32_t polls = 0, replies = 0, missed = 0, motorFrames = 0;
  static uint16_t sentAngle = 0, sentCount = 0;
  static uint16_t angle = 0, strength = 0;
  static bool waiting = false, haveIr = false, moving = false;
  static const char *lastReason = "";

  telemetry.pump(cameraSerial);
  while (cameraSerial.available()) parser.push(uint8_t(cameraSerial.read()), millis(), cam);
  const uint32_t now = millis();
  bool reply = false;
  while (Serial1.available()) {
    const uint8_t byte = uint8_t(Serial1.read());
    if (waiting && uint32_t(now - pollMs) < UPGRADE_IR_WAIT_MS && receiver.push(byte, angle, strength)) {
      waiting = false;
      haveIr = true;
      irMs = now;
      reply = true;
      ++replies;
      delayMicroseconds(50);
    }
  }
  if (waiting && uint32_t(now - pollMs) >= UPGRADE_IR_WAIT_MS) {
    waiting = false;
    ++missed;
    // Keep the last valid reply only until IR_REPLY_TIMEOUT_MS. A valid zero count
    // still immediately means no ball.
  }
  const char *reason = "";
  if (holdStop) reason = "manual stop";
  else if (!cam.received || uint32_t(now - cam.receivedMs) > 200) reason = "camera link";
  else if (!cam.ready || cam.slow) reason = "floor not recognised";
  else if (cam.escaping || cam.mask != 0xFFFF || cam.line != brain::kUnknown || cam.lineCount) reason = "line or unsafe direction";
  else if (!haveIr || uint32_t(now - irMs) > IR_REPLY_TIMEOUT_MS) reason = "IR link lost";
  else if (!strength) reason = "no ball detected";

  if (!waiting && (reply || (moving && *reason))) {
    const uint16_t d = *reason ? 0 : angle;
    const uint16_t count = *reason ? 0 : strength;
    uint8_t frame[] = {0xC3, 0x3C, uint8_t(d >> 8), uint8_t(d), uint8_t(count >> 8), uint8_t(count), 0, 0x5A};
    frame[6] = frame[2] ^ frame[3] ^ frame[4] ^ frame[5];
    sendRs485(frame, sizeof(frame));
    moving = count != 0;
    sentAngle = d; sentCount = count; ++motorFrames;
  }
  if (Serial && (strcmp(reason, lastReason) != 0 || uint32_t(now - logMs) >= 1000) && Serial.availableForWrite() >= 64) {
    Serial.printf("ZIP c2be6ff+fix reason=%s hold=%u\n", *reason ? reason : "clear", unsigned(holdStop));
    logMs = now;
    lastReason = reason;
  }
  // One short line per iteration; never wait for a USB reader.
  static uint32_t detailMs = 0;
  static uint8_t detailPage = 0;
  if (Serial && uint32_t(now - detailMs) >= 250 && Serial.availableForWrite() >= 64) {
    switch (detailPage++ % 4) {
      case 0: Serial.printf("cam ready=%u slow=%u escape=%u mask=%04x\n", unsigned(cam.ready), unsigned(cam.slow), unsigned(cam.escaping), unsigned(cam.mask)); break;
      case 1: Serial.printf("line=%u count=%u age=%lu err=%lu\n", unsigned(cam.line), unsigned(cam.lineCount), (unsigned long)(cam.received ? now-cam.receivedMs : 0xffffffffUL), (unsigned long)parser.errors); break;
      case 2: Serial.printf("ir=%u/%u age=%lu bno=off\n", unsigned(angle), unsigned(strength), (unsigned long)(haveIr ? now-irMs : 0xffffffffUL)); break;
      default: Serial.printf("sent=%u/%u poll=%lu rx=%lu miss=%lu tx=%lu\n", unsigned(sentAngle), unsigned(sentCount), (unsigned long)polls, (unsigned long)replies, (unsigned long)missed, (unsigned long)motorFrames); break;
    }
    detailMs = now;
  }
  if (telemetryDue(now)) {
    brain::Telemetry t;
    t.upgrade = true;
    t.irLink = haveIr && uint32_t(now - irMs) <= IR_REPLY_TIMEOUT_MS;
    t.cameraLink = cam.received && uint32_t(now - cam.receivedMs) <= 200;
    if (t.irLink && strength) { t.ball = angle; t.ballAgeMs = now - irMs; }
    t.direction = moving ? angle : 0;
    t.power = moving ? 175 : 0;  // ZIP MOTOR's nominal forward duty; actual duties are local.
    t.tactic = uint8_t(moving ? brain::Tactic::APPROACH : brain::Tactic::SAFE_STOP);
    t.mode = uint8_t(brain::Mode::DEGRADED);
    t.bno = 0;
    publishTelemetry(t, cam.yellow.seen ? int(cam.yellow.bearing) : -1, cam.yellow.confidence,
                     cam.blue.seen ? int(cam.blue.bearing) : -1, cam.blue.confidence,
                     moving ? "APPROACH" : "SAFE_STOP", "DEGRADED");
  }
  if (!waiting && uint32_t(now - pollMs) >= 20) {
    const uint8_t poll[] = {0xA5, 0x5A, 0x01, 0xFE};
    receiver.reset();
    sendRs485(poll, sizeof(poll));
    ++polls;
    pollMs = millis();
    waiting = true;
  }
}

// Direct PWM protocol: type 4, four signed duties encoded as 7-bit pairs with offset 210.
// Old MOTOR ignores type 4; the matching MOTOR firmware is required.
void sendPwm(uint8_t sequence, const int16_t pwm[4]) {
  uint8_t frame[FRAME_SIZE] = {};
  frame[0] = FRAME_START; frame[1] = 4; frame[2] = sequence & 0x7f;
  for (int i = 0; i < 4; ++i) encodeValue(&frame[3 + 2 * i], uint16_t(pwm[i] + 210));
  frame[11] = calculateCRC(frame);
  sendRs485(frame, sizeof(frame));
}

// The ZIP mapping drives channels 0/1 together and 2/3 together. With no
// footprint calibration, a turn needs all sectors; straight travel needs its sector.
bool zipPathClear(uint16_t mask, const int16_t pwm[4]) {
  if (pwm[0] != pwm[1] || pwm[2] != pwm[3]) return false;
  if (pwm[0] == pwm[2]) {
    if (pwm[0] > 0) return brain::sectorOpen(mask, 0);
    if (pwm[0] < 0) return brain::sectorOpen(mask, 8);
    return true;
  }
  return mask == 0xffff;
}

void chaseLoop() {
  static brain::CameraParser parser;
  static brain::CameraData cam;
  static IrReplyReceiver receiver;
  static ZipChasePD pd;
  static uint32_t pollMs = 0, irMs = 0, txMs = 0;
  static bool waiting = false, haveIr = false, newReply = false;
  static uint16_t angle = 0, strength = 0;
  static int16_t pwm[4] = {};
  static int16_t pdCalculated[4] = {}, safetyPWM[4] = {}, sentPWM[4] = {};
  static uint32_t pdStampMs = 0;
  static uint8_t sequence = 0;
  telemetry.pump(cameraSerial);
  while (cameraSerial.available()) parser.push(uint8_t(cameraSerial.read()), millis(), cam);
  uint32_t now = millis();
  while (Serial1.available()) {
    const uint8_t byte = uint8_t(Serial1.read());
    if (waiting && uint32_t(now - pollMs) < UPGRADE_IR_WAIT_MS && receiver.push(byte, angle, strength)) {
      waiting = false; haveIr = true; irMs = now; newReply = true;
      delayMicroseconds(50);
    }
  }
  if (waiting && uint32_t(now - pollMs) >= UPGRADE_IR_WAIT_MS) waiting = false;
  if (waiting) return;
  const bool freshCam = cam.received && uint32_t(now - cam.receivedMs) <= 200 && cam.ready && !cam.slow;
  const bool freshIr = haveIr && uint32_t(now - irMs) <= IR_REPLY_TIMEOUT_MS;
  const bool allowed = !holdStop && freshCam && freshIr && strength && !cam.escaping;
  const char *stopReason = "NONE";
  if (holdStop) stopReason = "MANUAL_STOP";
  else if (!freshCam) stopReason = "CAMERA_TIMEOUT";
  else if (!cam.ready) stopReason = "CAMERA_NOT_READY";
  else if (cam.slow) stopReason = "CAMERA_SLOW";
  else if (cam.escaping) stopReason = "LINE_DETECTED";
  else if (!freshIr) stopReason = "IR_TIMEOUT";
  else if (!strength) stopReason = "BALL_LOST";
  if (!allowed) {
    pd.reset();
    for (int i = 0; i < 4; ++i) pwm[i] = 0;
    newReply = false;
  } else if (newReply) {
    pd.update(angle, strength, now, pwm);
    for (int i = 0; i < 4; ++i) pdCalculated[i] = pwm[i];
    pdStampMs = now;
    newReply = false;
  }
  if (allowed && !zipPathClear(cam.mask, pwm)) {
    stopReason = "NO_SAFE_DIRECTION";
    pd.reset();
    for (int i = 0; i < 4; ++i) pwm[i] = 0;
  }
  for (int i = 0; i < 4; ++i) safetyPWM[i] = pwm[i];
  if (uint32_t(now - txMs) >= 20) {
    int16_t output[4] = {pwm[0], pwm[1], pwm[2], pwm[3]};
    if (!holdStop && freshCam && freshIr && cam.escaping &&
        (cam.escape == 0 || cam.escape == 180) &&
        brain::sectorOpen(cam.mask, brain::sectorOf(cam.escape))) {
      const int16_t escapeDuty = cam.escape == 0 ? 175 : -175;
      for (int i = 0; i < 4; ++i) output[i] = escapeDuty;
    }
    for (int i = 0; i < 4; ++i)
      output[i] = int16_t(constrain(int(output[i]), -ROBOT_TEST_MAX_PWM, ROBOT_TEST_MAX_PWM));
    sendPwm(sequence++, output);
    for (int i = 0; i < 4; ++i) sentPWM[i] = output[i];
    txMs = now;
  }
  if (usbDiagnosticRequested && Serial && Serial.availableForWrite() >= 200) {
    Serial.printf("[PD] valid=%u angle=%u strength=%u P=%.1f D=%.1f turn=%d age_ms=%lu\n",
                  unsigned(freshIr && strength), unsigned(angle), unsigned(strength), pd.proportional(),
                  pd.derivative(), pd.turn(), (unsigned long)(pdStampMs ? now - pdStampMs : 0xffffffffUL));
    Serial.printf("[PWM_CALC] %d,%d,%d,%d [PWM_SAFE] %d,%d,%d,%d [PWM_TX] %d,%d,%d,%d\n",
                  pdCalculated[0], pdCalculated[1], pdCalculated[2], pdCalculated[3],
                  safetyPWM[0], safetyPWM[1], safetyPWM[2], safetyPWM[3],
                  sentPWM[0], sentPWM[1], sentPWM[2], sentPWM[3]);
    Serial.printf("[STOP_REASON] %s camera_age_ms=%lu ir_age_ms=%lu mask=%04x line=%u\n", stopReason,
                  (unsigned long)(cam.received ? now - cam.receivedMs : 0xffffffffUL),
                  (unsigned long)(haveIr ? now - irMs : 0xffffffffUL), unsigned(cam.mask), unsigned(cam.lineCount));
    usbDiagnosticRequested = false;
  }
  if (uint32_t(now - pollMs) >= IR_POLL_INTERVAL_MS) {
    const uint8_t poll[] = {0xA5, 0x5A, 0x01, 0xFE};
    receiver.reset(); sendRs485(poll, sizeof(poll));
    pollMs = millis(); waiting = true;
  }
}

// ============================================================
// Main loop
// ============================================================

void loop() {
  // 通常系の順序: USB停止入力 → カメラ受信 → IR応答処理 → 追跡方位と安全方向を決定 → MOTORへ送信。
  static uint8_t sequence = 0;
  static uint32_t lastPollMs = 0;
  static uint32_t lastLogMs = 0;
  static uint32_t lastIrReplyMs = 0;
  static bool irReplied = false;
  static bool waitingForIr = false;
  static IrBallLock ballLock;
  static uint16_t lastDriveDirection = 0;
  static uint16_t lastDrivePower = 0;
  static uint16_t lastBallDirection = 360;
  static uint32_t lastBallMs = 0;

  while (Serial.available()) {
    const int command = Serial.read();
    if (command == 'd' || command == 's') holdStop = true;
    if (command == 'g') holdStop = false;
    if (command == 'v') usbViewer = !usbViewer;
    if (command == 'i') usbDiagnosticRequested = true;
  }
  if (ROBOT_ZIP_TRACK) {
    chaseLoop();
    return;
  }
  if (ROBOT_ZIP_COMPAT) {
    zipLoop();
    return;
  }
  if (ROBOT_UPGRADE) {
    upgradeLoop();
    return;
  }

  // 最新カメラ状態を先に更新し、以降の移動判断で古い床/白線情報を使わないようにする。
  receiveCamera();
  const uint32_t now = millis();
  if (BNO_ENABLED) {
    updateHeading(now);
    printHeading(now);
  }
  uint16_t rawAngle = 0;
  uint16_t strength = 0;
  bool received = false;
  const bool replyExpired = waitingForIr && now - lastPollMs >= IR_REPLY_TIMEOUT_MS;
  // IRは非同期応答なので、ポーリング中だけ受信器へ流し、1フレーム完成時に方位ロックを更新する。
  while (Serial1.available()) {
    const uint8_t byte = Serial1.read();
    if (waitingForIr && !replyExpired && irReceiver.push(byte, rawAngle, strength)) {
      received = true;
      break;
    }
  }

  if (received) {
    lastIrReplyMs = now;
    irReplied = true;
    ballLock.update(now, strength > 0, convertDirection(rawAngle));
    waitingForIr = false;
    delayMicroseconds(50);  // Let the IR transceiver release the shared bus.
  } else if (replyExpired) {
    ballLock.reset();
    waitingForIr = false;
  }

  if (now - lastLogMs >= 500 && Serial) {
    char line[100];
    const int length = snprintf(line, sizeof(line), "CAM=%s mode=%u IR=%s ball=%u DIR=%u PWM=%u\n",
                                cameraFresh(camera, now) ? "OK" : "NO",
                                unsigned(camera.mode),
                                irReplied && uint32_t(now - lastIrReplyMs) < IR_REPLY_TIMEOUT_MS ? "OK" : "NO",
                                unsigned(ballLock.active(now) ? ballLock.direction() : 360),
                                unsigned(lastDriveDirection), unsigned(lastDrivePower));
    if (length > 0 && length < int(sizeof(line)) && Serial.availableForWrite() >= length) {
      Serial.write(reinterpret_cast<const uint8_t *>(line), size_t(length));
      lastLogMs = now;
    }
  }

  telemetry.pump(cameraSerial);
  if (telemetryDue(now)) {
    brain::Telemetry t;
    t.irLink = irReplied && uint32_t(now - lastIrReplyMs) < IR_REPLY_TIMEOUT_MS;
    t.cameraLink = cameraFresh(camera, now);
    if (t.irLink && ballLock.active(now)) {
      t.ball = ballLock.direction();  // the legacy reply carries no confidence
      t.ballAgeMs = now - lastBallMs;
    }
    t.direction = lastDriveDirection;
    t.power = lastDrivePower;
    t.bno = BNO_ENABLED && isfinite(headingCurrent) ? 2 : (BNO_ENABLED ? 1 : 0);
    publishTelemetry(t, -1, 0, -1, 0, "LEGACY", "LEGACY");
  }

  // Never transmit during the IR reply window. Send a pending stop before a new drive.
  if (waitingForIr) return;
  if (replyExpired || (lastDrivePower && (!cameraAllowsMotion(camera, now) || camera.mode == 0))) {
    sendDrive(sequence++, 0, 0);
    lastDrivePower = 0;
  } else if (received) {
    uint16_t direction = 0;
    uint16_t power = 0;
    if (ballLock.active(now)) {
      lastBallDirection = ballLock.direction();
      lastBallMs = now;
    }
    const bool ballHeld = strength > 0 && ballLock.active(now);
    chooseChaseMotion(camera, now, ballHeld, lastBallDirection,
                      MOVE_POWER, AVOID_POWER, direction, power);
    sendDrive(sequence++, direction, power, camera.mode == 1);
    lastDriveDirection = direction;
    lastDrivePower = power;

  }

  if (now - lastPollMs >= IR_POLL_INTERVAL_MS) {
    const uint8_t poll[] = {0xA5, 0x5A, 0x01, 0xFE};
    irReceiver.reset();
    sendRs485(poll, sizeof(poll));
    lastPollMs = millis();
    waitingForIr = true;
  }
}
