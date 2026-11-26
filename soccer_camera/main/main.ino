#include <Arduino.h>
#include <string.h>
#include <math.h>
#include <Wire.h>
#include <SerialPIO.h>
#include "main_logic.h"

// 0 = field-tested chase (default). 1 = camera protocol 2, filtered IR, heading modes and tactics.
// Build MAIN, MOTOR and IR together and set PROTOCOL = 2 on the camera. Not verified on hardware.
#ifndef ROBOT_UPGRADE
#define ROBOT_UPGRADE 0
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
      candidate_ = kNoDirection;
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
constexpr uint32_t BALL_HOLD_MS = 500;  // keep the last bearing this long after the ball lock

static_assert(AVOID_POWER <= MOVE_POWER && MOVE_POWER <= MAX_POWER,
              "MOVE_POWER exceeds MAX_POWER");

// IR orientation
constexpr int16_t IR_FRONT_DEGREES = 0;
// The raw ZIP angle puts physical GPIO21 (left) at 90 degrees.
constexpr int8_t IR_ANGLE_SIGN = -1;

constexpr uint8_t BNO_ADDRESS = 0x28;
constexpr bool BNO_ENABLED = false; // Set true after the BNO055 is available again.
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

CameraState camera;
uint8_t cameraFrame[8] = {};
uint8_t cameraUsed = 0;
uint32_t cameraLastByteMs = 0;

void receiveCamera() {
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
    Wire.setTimeout(5);
    Wire.beginTransmission(BNO_ADDRESS);
    Wire.write(0x3D);
    Wire.write(0x08); // IMUPLUS
    Wire.endTransmission();
    delay(30);
  }
  if (ROBOT_UPGRADE) {
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    Wire.setClock(50000);
    Wire.setTimeout(5, true);
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
  static brain::Brain robot;
  static brain::CameraParser parser;
  static brain::CameraData cameraData;
  static brain::IrReceiver irFrame;
  static brain::IrData irData;
  static brain::BnoSample bno;
  static brain::Output out;
  static uint8_t sequence = 0;
  static uint32_t lastPollMs = 0, lastDriveMs = 0, lastLogMs = 0, lastUs = 0, loopMaxUs = 0;
  static uint32_t pollUs = 0, irRoundTripUs = 0, cameraGapMaxMs = 0, brainMaxUs = 0;
  static bool waitingForIr = false, urgent = false;

  const uint32_t us = micros();
  if (lastUs && us - lastUs > loopMaxUs) loopMaxUs = us - lastUs;
  lastUs = us;

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
    if (waitingForIr && irFrame.push(byte, reply)) {
      reply.received = true;
      reply.angle = convertDirection(reply.angle);
      reply.stampMs = now;
      irData = reply;
      irRoundTripUs = micros() - pollUs;
      waitingForIr = false;
      delayMicroseconds(50);
    }
  }
  if (waitingForIr && now - lastPollMs >= UPGRADE_IR_WAIT_MS) waitingForIr = false;
  if (waitingForIr) return;  // never transmit inside the IR reply window

  updateBno(now, bno);
  if (urgent || now - lastDriveMs >= 20) {
    const uint32_t start = micros();
    out = robot.step(cameraData, irData, bno, millis());
    if (micros() - start > brainMaxUs) brainMaxUs = micros() - start;
    headingTurn = int8_t(out.turn);
    sendDrive(sequence++, out.direction, out.power, out.avoid);
    lastDriveMs = now;
    urgent = false;
  }
  if (now - lastPollMs >= IR_POLL_INTERVAL_MS) {
    const uint8_t poll[] = {0xA5, 0x5A, 0x02, 0xFD};
    irFrame.reset();
    sendRs485(poll, sizeof(poll));
    lastPollMs = millis();
    pollUs = micros();
    waitingForIr = true;
  }
  if (now - lastLogMs >= 500 && Serial && Serial.availableForWrite() >= 240) {
    lastLogMs = now;
    Serial.printf("%s%s %s mode=%s%s dir=%u spd=%u turn=%d ball=%s %d c%u d%d goal=%d\n",
                  holdStop ? "HOLD " : "", brain::name(out.tactic), out.reason, brain::name(robot.heading.mode),
                  robot.heading.disagreement ? "(mismatch)" : "", out.direction, out.power, out.turn,
                  brain::name(robot.ball.state), int(robot.ball.bearing), robot.ball.confidence,
                  robot.ball.distanceCm, isfinite(out.goal) ? int(out.goal) : -1);
    Serial.printf("cam=%u ready=%d mask=%04x err=%lu gap=%lums ir c%u f%03x rtt=%luus bno=%d loop=%luus brain=%luus\n",
                  unsigned(now - cameraData.receivedMs), int(cameraData.ready), cameraData.mask,
                  static_cast<unsigned long>(parser.errors), static_cast<unsigned long>(cameraGapMaxMs),
                  irData.confidence, irData.faults, static_cast<unsigned long>(irRoundTripUs), int(bno.ok),
                  static_cast<unsigned long>(loopMaxUs), static_cast<unsigned long>(brainMaxUs));
    loopMaxUs = brainMaxUs = cameraGapMaxMs = 0;
  }
}

// ============================================================
// Main loop
// ============================================================

void loop() {
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
  }
  if (ROBOT_UPGRADE) {
    upgradeLoop();
    return;
  }

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
    const bool ballHeld = lastBallDirection < 360 && now - lastBallMs <= BALL_HOLD_MS;
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
