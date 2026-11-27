// MAIN拡張制御用ロジック（ROBOT_UPGRADE=1）。main.inoから呼ばれる純粋C++部で、基板I/Oを持たない。
// カメラの拡張パケットと ir/ir_logic.h の拡張応答を解釈し、方位推定・ボール追跡・戦術判断をまとめる。
// 標準構成では無効。数値や機能の一部は実機未検証なので、有効化時はMAIN/MOTOR/IR/OpenMVを揃える。
// Upgrade-path logic for MAIN (ROBOT_UPGRADE=1). Pure C++ (host tested).
// Angles: degrees clockwise from the robot's front. All gains and limits below are unverified.
#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

namespace brain {

constexpr uint16_t kUnknown = 0xFFFF;

struct Config {
  // Speed requests for the MOTOR wheel model (210 = full). Values below the motors' holding speed are raised there.
  uint16_t speedSlow = 160, speedNormal = 160, speedAttack = 195, speedAvoid = 175;
  int turnSign = 1;            // +1 = clockwise by wheel geometry; verify with wheels raised
  float headingKp = 0.5f, headingKd = 0.05f;
  float headingEnter = 10, headingExit = 5;
  int turnMin = 8, turnMax = 30, turnSlew = 3;  // turn units, per 20 ms step
  uint32_t cameraTimeoutMs = 200, irTimeoutMs = 100;
  uint32_t searchLegMs = 400, attackStallMs = 3000, recoverMs = 400;
  uint32_t lineHoldMs = 150;  // a slow camera frame may reuse line data this old, never older
  uint32_t goalMemoryMs = 1000, goalMemoryGyroMs = 3000;
  uint8_t ballMinConfidence = 25, goalMinConfidence = 40, headingMinConfidence = 50;
  uint8_t irMaxFaults = 3;  // more excluded IR sensors than this: the ball bearing is not trusted
  float attackEnterDeg = 15, attackExitDeg = 30, alignDeg = 45;
  // Both default to the cautious choice; enable only after the matching bench check.
  bool blindTracking = false;        // drive at the ball while the attack direction is unknown
  bool bnoPowerOnReference = false;  // robot is always powered up facing the attack goal
  uint8_t ownGoalGuardDeg = 45;
};

inline float wrap(float a) { a = fmodf(a + 180, 360); return (a < 0 ? a + 360 : a) - 180; }
inline float norm(float a) { a = fmodf(a, 360); return a < 0 ? a + 360 : a; }
inline float clampf(float v, float limit) { return v > limit ? limit : (v < -limit ? -limit : v); }

// ---------------- Camera protocol 3 (goals, lines, heading; the camera never reports a ball) ----------------
constexpr uint8_t kCameraVersion = 3, kCameraSize = 32;

struct Seen {
  bool seen = false;
  float bearing = 0;
  int distanceCm = -1;  // -1: unknown
  uint8_t confidence = 0, width = 255;
  uint32_t stampMs = 0;
};

struct CameraData {
  bool received = false, ready = false, escaping = false, targetValid = false, targetBlue = false, slow = false;
  uint8_t sequence = 0, lineCount = 0, lineCm = 255, frameMs = 0, headingConfidence = 0;
  uint16_t mask = 0, escape = kUnknown, line = kUnknown, heading = kUnknown;
  Seen yellow, blue;
  uint32_t receivedMs = 0;
};

inline uint16_t crc16(const uint8_t *data, int size) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < size; ++i) {
    crc ^= uint16_t(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
  }
  return crc;
}

class CameraParser {
public:
  uint32_t errors = 0;

  bool push(uint8_t byte, uint32_t now, CameraData &out) {
    if (used_ && uint32_t(now - lastMs_) >= 20) used_ = 0;
    lastMs_ = now;
    if ((used_ == 0 && byte != 0xC5)) return false;
    buffer_[used_++] = byte;
    if (!headerPlausible()) { resync(); return false; }
    if (used_ < kCameraSize) return false;
    CameraData next;
    if (!decode(next, now)) { ++errors; resync(); return false; }
    used_ = 0;
    // A repeated sequence number is a stale frame, not fresh data, however long ago it first arrived.
    if (out.received && next.sequence == out.sequence) return false;
    out = next;
    return true;
  }

private:
  bool headerPlausible() const {
    return !((used_ >= 2 && buffer_[1] != 0x5C) || (used_ >= 3 && buffer_[2] != kCameraVersion) ||
             (used_ >= 4 && buffer_[3] != kCameraSize));
  }
  void resync() {
    uint8_t next = 1;
    for (;;) {
      while (next < used_ && buffer_[next] != 0xC5) ++next;
      const uint8_t left = uint8_t(used_ - next);
      memmove(buffer_, buffer_ + next, left);
      used_ = left;
      if (!used_ || headerPlausible()) return;
      next = 1;
    }
  }
  uint16_t u16(int i) const { return uint16_t(buffer_[i] | buffer_[i + 1] << 8); }
  static bool angleOk(uint16_t a) { return a < 360 || a == kUnknown; }
  bool seen(Seen &s, int at, uint8_t ageUnits, uint32_t now) const {
    const uint16_t bearing = u16(at);
    const uint8_t distance = buffer_[at + 2], confidence = buffer_[at + 3];
    if (!angleOk(bearing) || confidence > 100) return false;
    s.seen = bearing != kUnknown && ageUnits != 255;
    s.bearing = s.seen ? bearing : 0;
    s.distanceCm = distance == 255 ? -1 : distance;
    s.confidence = s.seen ? confidence : 0;
    s.stampMs = now - uint32_t(s.seen ? ageUnits : 0) * 10;
    return true;
  }
  bool decode(CameraData &d, uint32_t now) const {
    if (crc16(buffer_, kCameraSize - 2) != u16(kCameraSize - 2)) return false;
    const uint8_t flags = buffer_[5];
    if (flags & 0xE0) return false;
    d.received = true;
    d.receivedMs = now;
    d.sequence = buffer_[4];
    d.ready = flags & 1;
    d.escaping = flags & 2;
    d.targetBlue = flags & 4;
    d.slow = flags & 8;
    d.targetValid = flags & 16;
    d.mask = u16(6);
    d.escape = u16(8);
    d.line = u16(10);
    d.lineCm = buffer_[12];
    d.lineCount = buffer_[13];
    if (!angleOk(d.escape) || !angleOk(d.line) || d.escaping != (d.escape != kUnknown) ||
        (!d.ready && d.mask)) return false;
    if (!seen(d.yellow, 14, buffer_[19], now) || !seen(d.blue, 20, buffer_[25], now)) return false;
    d.yellow.width = buffer_[18];
    d.blue.width = buffer_[24];
    d.heading = u16(26);
    d.headingConfidence = buffer_[28];
    d.frameMs = buffer_[29];
    return angleOk(d.heading);
  }

  uint8_t buffer_[kCameraSize] = {};
  uint8_t used_ = 0;
  uint32_t lastMs_ = 0;
};

// ---------------- IR extended reply (see ir/ir_logic.h) ----------------
struct IrData {
  bool received = false;   // the IR board answered at least once
  uint16_t angle = 0;      // already converted to robot coordinates
  uint16_t strength = 0, faults = 0;
  uint8_t confidence = 0;
  uint32_t stampMs = 0;
};

class IrReceiver {
public:
  void reset() { used_ = 0; }
  bool push(uint8_t byte, IrData &r) {
    if (!used_ && byte != 0xAB) return false;
    b_[used_++] = byte;
    if (used_ < 10) return false;
    used_ = 0;
    uint8_t crc = 0;
    for (int i = 1; i < 8; ++i) {
      crc ^= b_[i];
      for (int bit = 0; bit < 8; ++bit) crc = uint8_t((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
    }
    const uint16_t angle = uint16_t(b_[1] << 8 | b_[2]);
    if (b_[9] != 0x55 || crc != b_[8] || angle >= 360 || b_[5] > 100) return false;
    r.angle = angle;
    r.strength = uint16_t(b_[3] << 8 | b_[4]);
    r.confidence = b_[5];
    r.faults = uint16_t(b_[6] << 8 | b_[7]);
    return true;
  }

private:
  uint8_t b_[10] = {};
  uint8_t used_ = 0;
};

// ---------------- Telemetry to the camera / PC (display only, see openmv/visualize.py) ----------------
constexpr uint8_t kTelemetrySize = 19;
struct Telemetry {
  uint8_t sequence = 0;
  bool upgrade = false, irLink = false, cameraLink = false, hold = false;
  int ball = -1;             // IR bearing, robot degrees; -1: no CURRENT detection
  uint8_t ballConfidence = 255;  // 255: not available
  uint32_t ballAgeMs = 0;
  uint16_t direction = 0, power = 0;
  int turn = 0;
  uint8_t tactic = 255, mode = 255;  // enum values, 255: legacy control
  uint8_t bno = 0;                   // 0 disabled, 1 no usable data, 2 in use
};

inline void encodeTelemetry(const Telemetry &t, uint8_t *f) {
  f[0] = 0xD7; f[1] = 0x7D; f[2] = 1; f[3] = kTelemetrySize; f[4] = t.sequence;
  const bool ball = t.ball >= 0;
  f[5] = uint8_t((t.upgrade ? 1 : 0) | (t.irLink ? 2 : 0) | (t.cameraLink ? 4 : 0) | (ball ? 8 : 0) | (t.hold ? 16 : 0));
  const uint16_t bearing = ball ? uint16_t(t.ball) : kUnknown;
  const uint16_t direction = t.power ? t.direction : kUnknown;
  f[6] = uint8_t(bearing); f[7] = uint8_t(bearing >> 8);
  f[8] = ball ? t.ballConfidence : 0;
  f[9] = ball ? uint8_t(t.ballAgeMs / 10 > 254 ? 254 : t.ballAgeMs / 10) : 255;
  f[10] = uint8_t(direction); f[11] = uint8_t(direction >> 8);
  f[12] = uint8_t(t.power > 255 ? 255 : t.power);
  f[13] = uint8_t(t.turn + 128);
  f[14] = t.tactic; f[15] = t.mode; f[16] = t.bno;
  const uint16_t crc = crc16(f, kTelemetrySize - 2);
  f[17] = uint8_t(crc); f[18] = uint8_t(crc >> 8);
}

// Never blocks the control loop: writes only what the transmit FIFO can take right now.
class TelemetrySender {
public:
  bool busy() const { return used_ < size_; }
  void load(const Telemetry &t) {
    if (busy()) return;  // drop rather than queue stale data
    encodeTelemetry(t, frame_);
    used_ = 0;
    size_ = kTelemetrySize;
  }
  template <class Port> void pump(Port &port) {
    while (busy() && port.availableForWrite() > 0) port.write(frame_[used_++]);
  }

private:
  uint8_t frame_[kTelemetrySize] = {};
  uint8_t used_ = 0, size_ = 0;
};

// ---------------- BNO055 sample screening ----------------
struct BnoSample {
  bool ok = false;           // connected, gyro calibrated, value in range
  float yaw = 0;
  uint32_t stampMs = 0;
};

// ---------------- Heading / mode selection ----------------
enum class Mode { FUSED, BNO_ONLY, VISION_ONLY, DEGRADED };
inline const char *name(Mode m) {
  static const char *const n[] = {"FUSED", "BNO_ONLY", "VISION_ONLY", "DEGRADED"};
  return n[int(m)];
}

// Counts consecutive good / bad observations so one sample never switches the mode.
struct Gate {
  bool open = false;
  uint8_t good = 0, bad = 0;
  void feed(bool ok, uint8_t enter, uint8_t leave) {
    if (ok) { bad = 0; if (good < 255) ++good; if (good >= enter) open = true; }
    else { good = 0; if (bad < 255) ++bad; if (bad >= leave) open = false; }
  }
};

class Heading {
public:
  Mode mode = Mode::DEGRADED;
  bool absolute = false;  // heading() is a field heading; otherwise only goal-relative control is possible
  bool disagreement = false;
  float heading() const { return heading_; }

  void update(const BnoSample &bno, const CameraData &cam, bool cameraFresh, bool goalVisible,
              const Config &cfg, uint32_t now) {
    // Vision: one vote per camera frame. A jump restarts the continuity count.
    const bool vNow = cameraFresh && cam.heading != kUnknown && cam.headingConfidence >= cfg.headingMinConfidence;
    if (!cameraFresh) { vision_ = Gate(); }
    else if (cam.sequence != visionSequence_ || !visionStarted_) {
      visionStarted_ = true;
      visionSequence_ = cam.sequence;
      if (vNow) {
        if (haveVision_ && fabsf(wrap(cam.heading - visionHeading_)) > 30) vision_ = Gate();
        visionHeading_ = cam.heading;
        haveVision_ = true;
      }
      vision_.feed(vNow, 5, 5);
    }
    const bool visionOk = vision_.open && cameraFresh;
    // BNO: one vote per new sample, with a rate-of-change check.
    if (bno.stampMs != bnoStamp_) {
      bool ok = bno.ok && isfinite(bno.yaw) && bno.yaw >= 0 && bno.yaw < 360;
      const uint32_t dt = bno.stampMs - bnoStamp_;
      const bool usableRaw = ok;
      if (ok && haveRaw_ && dt < 100 && fabsf(wrap(bno.yaw - raw_)) > 2 + dt * 0.8f) ok = false;  // > 800 deg/s
      bnoStamp_ = bno.stampMs;
      if (usableRaw) { raw_ = bno.yaw; haveRaw_ = true; }
      if (ok) yaw_ = bno.yaw;  // a rejected sample never reaches the estimate
      else if (usableRaw) bno_ = Gate();  // a step (sensor reset?) must be re-acquired, not followed
      const bool was = bno_.open;
      bno_.feed(ok, 10, 3);
      if (bno_.open && !was) {
        // (Re)acquired: choose the zero so the estimate does not jump.
        if (visionOk) {
          offset_ = wrap(yaw_ - visionHeading_);
          referenced_ = true;
        } else if (haveHeading_ && uint32_t(now - headingMs_) < 2000) {
          offset_ = wrap(yaw_ - heading_);
        } else {
          // After a long gap the gyro zero says nothing about the field.
          offset_ = yaw_;
          referenced_ = cfg.bnoPowerOnReference && !haveHeading_;
        }
        disagree_ = Gate();
      }
    }
    if (!bno.ok || !isfinite(bno.yaw) || bno.yaw < 0 || bno.yaw >= 360 ||
        uint32_t(now - bnoStamp_) > 200) bno_ = Gate();

    absolute = true;
    if (bno_.open) {
      if (!referenced_ && visionOk) {
        offset_ = wrap(yaw_ - visionHeading_);
        referenced_ = true;
      }
      float estimate = wrap(yaw_ - offset_);
      if (visionOk) {
        const float diff = wrap(estimate - visionHeading_);
        disagree_.feed(fabsf(diff) > 30, 25, 25);
        disagreement = disagree_.open;
        if (!disagreement) {
          // Complementary filter: gyro fast, vision slow; outliers never pull the gyro.
          if (fabsf(diff) <= 30) offset_ = wrap(offset_ + 0.01f * diff);
          mode = Mode::FUSED;
        } else if (cam.headingConfidence >= 70) {
          mode = Mode::VISION_ONLY;
          estimate = visionHeading_;
        } else {
          mode = Mode::BNO_ONLY;
        }
      } else {
        disagreement = disagree_.open;  // stays until re-acquisition or renewed agreement
        mode = Mode::BNO_ONLY;
      }
      if (referenced_) {
        store(estimate, now);
      } else {
        heading_ = estimate;  // only good for rotating the ball track
        absolute = false;
      }
    } else if (visionOk) {
      disagreement = false;
      mode = Mode::VISION_ONLY;
      referenced_ = true;
      store(wrap(visionHeading_), now);
    } else {
      disagreement = false;
      absolute = false;
      mode = goalVisible ? Mode::VISION_ONLY : Mode::DEGRADED;
    }
  }

private:
  void store(float h, uint32_t now) { heading_ = h; haveHeading_ = true; headingMs_ = now; }
  Gate bno_, vision_, disagree_;
  float heading_ = 0, yaw_ = 0, raw_ = 0, offset_ = 0, visionHeading_ = 0;
  uint32_t bnoStamp_ = 0, headingMs_ = 0;
  uint8_t visionSequence_ = 0;
  bool haveRaw_ = false, haveHeading_ = false, haveVision_ = false, visionStarted_ = false, referenced_ = false;
};

// PD turn command with deadband hysteresis, a feasible minimum and a slew limit.
class Turn {
public:
  int update(bool enabled, float error, const Config &cfg, float gain = 1) {
    int wanted = 0;
    if (enabled) {
      if (fabsf(error) > cfg.headingEnter) engaged_ = true;
      else if (fabsf(error) < cfg.headingExit) engaged_ = false;
      if (engaged_) {
        const float rate = have_ ? wrap(error - previous_) / 0.02f : 0;
        float u = gain * (cfg.headingKp * error + cfg.headingKd * clampf(rate, 200));
        if (u * error < 0) u = 0;  // damping never reverses the correction
        u = clampf(u, float(cfg.turnMax));
        if (u != 0 && fabsf(u) < cfg.turnMin) u = copysignf(float(cfg.turnMin), u);
        wanted = int(lroundf(u)) * cfg.turnSign;
      }
      previous_ = error;
    } else {
      engaged_ = false;
    }
    have_ = enabled;
    if (!enabled) value_ = 0;  // disabling (line escape, no estimate) is immediate
    else if (wanted > value_ + cfg.turnSlew) value_ += cfg.turnSlew;
    else if (wanted < value_ - cfg.turnSlew) value_ -= cfg.turnSlew;
    else value_ = wanted;
    return value_;
  }

private:
  float previous_ = 0;
  int value_ = 0;
  bool engaged_ = false, have_ = false;
};

// ---------------- Ball tracking and prediction (IR only) ----------------
// WAIT only before the first sighting; afterwards TRACK -> PREDICT -> SEARCH -> REACQUIRE -> TRACK.
enum class BallState { TRACK, PREDICT, SEARCH, REACQUIRE, WAIT };
inline const char *name(BallState s) {
  static const char *const n[] = {"TRACK", "PREDICT", "SEARCH", "REACQUIRE", "WAIT"};
  return n[int(s)];
}

struct Observation {
  bool valid = false;
  float bearing = 0;
  uint8_t confidence = 0;
  uint32_t stampMs = 0;
};

class Ball {
public:
  BallState state = BallState::WAIT;
  uint8_t confidence = 0;
  float bearing = 0;  // filtered, or predicted while lost
  float rate = 0;     // deg/s relative to the robot (includes the robot's own motion)

  // The only ball source is the IR board. headingKnown/heading let the track be rotated with the robot
  // (BNO); otherwise prediction stays short.
  void update(const Observation &ir, bool headingKnown, float heading, const Config &cfg, uint32_t now) {
    if (headingKnown && headingWasKnown_ && ever_) filtered_ = norm(filtered_ - wrap(heading - heading_));
    headingWasKnown_ = headingKnown;
    heading_ = heading;

    if (ir.valid && ir.confidence >= cfg.ballMinConfidence && ir.stampMs != irStamp_) {
      irStamp_ = ir.stampMs;
      measure(ir.bearing, ir.confidence, now);
    }

    if (!ever_) { state = BallState::WAIT; return; }
    const uint32_t lost = now - seenMs_;
    const uint32_t predictMs = headingWasKnown_ ? 100 + 2u * confidence : 100 + confidence;
    if (lost <= 100) {
      if (reacquiring_ && uint32_t(now - reacquiredMs_) >= 100) reacquiring_ = false;
      state = reacquiring_ ? BallState::REACQUIRE : BallState::TRACK;
      bearing = filtered_;
    } else if (lost <= predictMs) {
      state = BallState::PREDICT;
      bearing = norm(filtered_ + clampf(rate * lost / 1000.0f, 30));
    } else {
      state = BallState::SEARCH;  // `bearing` stays as the search anchor
      reacquiring_ = false;
      rate = 0;
    }
  }

  // Where to aim for a moving ball, a short step ahead.
  float lead() const { return norm(bearing + clampf(rate * 0.08f, 15)); }

private:
  void measure(float z, uint8_t conf, uint32_t now) {
    const float dt = ever_ ? (now - measuredMs_) / 1000.0f : 1;
    if (!ever_ || dt > 0.3f) {
      filtered_ = z;
      rate = 0;
      jump_ = false;
    } else {
      const float predicted = norm(filtered_ + rate * dt);
      const float residual = wrap(z - predicted);
      if (fabsf(residual) > 45) {
        // A sudden change needs two agreeing observations (reflections are single).
        if (!jump_ || fabsf(wrap(z - jumpBearing_)) > 25) {
          jump_ = true;
          jumpBearing_ = z;
          return;
        }
        filtered_ = z;
        rate = 0;
        jump_ = false;
      } else {
        jump_ = false;
        const float alpha = 0.4f + 0.5f * fminf(fabsf(residual) / 30, 1);
        filtered_ = norm(predicted + alpha * residual);
        if (dt > 0.005f) rate = clampf(rate + 0.15f * residual / dt, 360);
      }
    }
    if (state == BallState::SEARCH) {
      reacquiring_ = true;
      reacquiredMs_ = now;
    }
    ever_ = true;
    confidence = conf;
    measuredMs_ = seenMs_ = now;
  }

  float filtered_ = 0, heading_ = 0, jumpBearing_ = 0;
  uint32_t irStamp_ = 0, measuredMs_ = 0, seenMs_ = 0;
  uint32_t reacquiredMs_ = 0;
  bool ever_ = false, jump_ = false, headingWasKnown_ = false, reacquiring_ = false;
};

// ---------------- Tactics, line safety, speed ----------------
enum class Tactic { SEARCH, APPROACH, ORBIT, ALIGN, ATTACK, RECOVER, AVOID_LINE, SAFE_STOP };
inline const char *name(Tactic t) {
  static const char *const n[] = {"SEARCH", "APPROACH", "ORBIT", "ALIGN", "ATTACK", "RECOVER", "AVOID_LINE", "SAFE_STOP"};
  return n[int(t)];
}

inline bool sectorOpen(uint16_t mask, int sector) { return mask & (1u << (sector & 15)); }
inline int sectorOf(float direction) { return int(norm(direction) / 22.5f + 0.5f) & 15; }

// Best direction that the line mask opens and `veto` accepts, or -1. In-between angles can come out up
// to ~23 degrees wrong, so away from a fully open field only multiples of 45 (exact on all wheels) are used.
// Cost: angle from `wanted`, a reversal pause against `previous`, and closeness to the nearest line.
template <class Veto>
inline int chooseDirection(uint16_t mask, float wanted, Veto veto, int previous = -1, int line = -1,
                           bool exact = false) {
  if (mask == 0xFFFF && !exact && !veto(norm(wanted))) return int(norm(wanted) + 0.5f) % 360;
  int best = -1;
  float bestCost = 1e9f;
  for (int k = 0; k < 16; k += 2) {
    const float candidate = k * 22.5f;
    if (!sectorOpen(mask, k) || veto(candidate)) continue;
    float cost = fabsf(wrap(candidate - wanted));
    if (previous >= 0 && fabsf(wrap(candidate - previous)) > 100) cost += 40;
    if (line >= 0) cost += fmaxf(0, 90 - fabsf(wrap(candidate - line))) * 0.5f;
    if (k % 4) cost += 25;  // diagonals drive two wheels only and are not bench tested
    if (cost < bestCost) { bestCost = cost; best = int(candidate); }
  }
  return best;
}
inline int nearestExact(uint16_t mask, float wanted) {
  int best = -1;
  float bestCost = 1e9f;
  for (int k = 0; k < 16; k += 2) {
    const float cost = fabsf(wrap(k * 22.5f - wanted)) + (k % 4 ? 25 : 0);
    if (sectorOpen(mask, k) && cost < bestCost) { bestCost = cost; best = k / 2 * 45; }
  }
  return best;
}
inline int safeDirection(uint16_t mask, float wanted) {
  return chooseDirection(mask, wanted, [](float) { return false; });
}

struct Output {
  uint16_t direction = 0, power = 0;
  int turn = 0;
  bool avoid = false;
  Tactic tactic = Tactic::SAFE_STOP;
  const char *reason = "";
  float goal = NAN;  // attack bearing used, NAN if unknown
};

class Brain {
public:
  Config cfg;
  Heading heading;
  Ball ball;

  Output step(const CameraData &cam, const IrData &ir, const BnoSample &bno, uint32_t now) {
    const bool camFresh = cam.received && uint32_t(now - cam.receivedMs) <= cfg.cameraTimeoutMs;
    const bool irFresh = ir.received && uint32_t(now - ir.stampMs) <= cfg.irTimeoutMs;
    const Seen &target = cam.targetBlue ? cam.blue : cam.yellow;
    const Seen &own = cam.targetBlue ? cam.yellow : cam.blue;
    auto usable = [&](const Seen &s) {
      return camFresh && cam.targetValid && s.seen && s.confidence >= cfg.goalMinConfidence &&
             uint32_t(now - s.stampMs) <= 500;
    };
    heading.update(bno, cam, camFresh, usable(target) || usable(own), cfg, now);

    Observation i;
    const bool irSensorsOk = faultCount(ir.faults) <= cfg.irMaxFaults;
    i.valid = irFresh && irSensorsOk && ir.strength > 0;
    i.bearing = ir.angle; i.confidence = ir.confidence; i.stampMs = ir.stampMs;
    const bool gyro = heading.mode == Mode::FUSED || heading.mode == Mode::BNO_ONLY;
    if (!i.valid || i.confidence < cfg.ballMinConfidence) ball = Ball();
    else ball.update(i, gyro, heading.heading(), cfg, now);

    const bool gyroNow = gyro && !heading.disagreement;
    // Attack direction: the goal itself, else opposite our own goal, else the heading, else memory.
    const bool trusted = heading.absolute && !heading.disagreement;
    float goal = NAN;
    if (usable(target)) goal = target.bearing;
    else if (usable(own)) goal = norm(own.bearing + 180);
    else if (trusted) goal = norm(-heading.heading());
    if (isfinite(goal)) {
      goalMemory_ = goal;
      goalMs_ = now;
      goalHeading_ = heading.heading();
      goalGyro_ = gyroNow;
      haveGoal_ = true;
    } else if (haveGoal_ && !heading.disagreement) {
      // The gyro lets the remembered bearing turn with the robot, so it can be kept longer.
      const bool turned = goalGyro_ && gyroNow;
      if (uint32_t(now - goalMs_) <= (turned ? cfg.goalMemoryGyroMs : cfg.goalMemoryMs))
        goal = turned ? norm(goalMemory_ - wrap(heading.heading() - goalHeading_)) : goalMemory_;
    }

    Output out;
    out.goal = goal;
    // 1. communication and recognition failures
    if (!camFresh) return stop(out, "camera link");
    bool held = false;
    if (cam.ready) {
      lineMask_ = cam.mask;
      lineEscape_ = cam.escaping ? cam.escape : kUnknown;
      lineBearing_ = cam.line;
      lineMs_ = cam.receivedMs;
      haveLine_ = true;
    } else if (cam.slow && haveLine_ && uint32_t(now - lineMs_) <= cfg.lineHoldMs) {
      held = true;  // one late frame: keep the last good line data, briefly
    } else {
      haveLine_ = false;
      return stop(out, cam.slow ? "line data expired" : "floor not recognised");
    }
    const uint16_t mask = lineMask_;
    if (!mask) return stop(out, "no safe direction");
    // No IR observation means no ball: the camera cannot substitute. "No ball seen" is a different state.
    if (!irFresh) return stop(out, "IR link lost");
    if (!irSensorsOk) return stop(out, "IR sensors failed");

    const int line = lineBearing_ == kUnknown ? -1 : int(lineBearing_);
    // 2. line escape, with no heading correction
    if (lineEscape_ != kUnknown) {
      const uint16_t escape = lineEscape_;
      const int d = escape % 45 == 0 && sectorOpen(mask, sectorOf(escape))
                        ? int(escape)
                        : nearestExact(mask, float(escape));
      if (d < 0) return stop(out, "no safe escape");
      out.direction = uint16_t(d);
      out.power = cfg.speedAvoid;
      out.avoid = true;
      return finish(out, Tactic::AVOID_LINE, "line", false, 0);
    }

    if (!i.valid || i.confidence < cfg.ballMinConfidence) return stop(out, "no ball detected");

    if (recovering_ && uint32_t(now - recoverMs_) >= cfg.recoverMs) recovering_ = false;
    if (ball.state == BallState::WAIT) return finish(out, Tactic::SEARCH, "no ball seen yet", false, 0);

    float wanted = 0;
    uint16_t speed = cfg.speedNormal;
    Tactic tactic = Tactic::APPROACH;
    const char *note = "";
    bool keepAway = false;  // never get closer to the ball
    float orbitLimit = 45;  // a rerouted orbit never heads closer to the ball than this, or than planned
    if (ball.state == BallState::SEARCH) {
      // Octagon patrol from the last bearing. 45 degree steps need no reverse pause (a wheel changing
      // sign has been idle for a whole leg), and the loop returns near its start.
      tactic = Tactic::SEARCH;
      if (previous_ != Tactic::SEARCH) {
        searchMs_ = now;
        searchAnchor_ = int(ball.bearing / 45 + 0.5f) % 8 * 45;
      }
      wanted = norm(float(searchAnchor_) + side_ * 45.0f * (uint32_t(now - searchMs_) / cfg.searchLegMs % 8));
      speed = cfg.speedSlow;
      note = "no ball detected: searching";
    } else if (recovering_) {
      tactic = Tactic::RECOVER;
      wanted = norm(ball.bearing + 180);
    } else if (!isfinite(goal)) {
      if (cfg.blindTracking || !gyro) {
        wanted = ball.lead();
        note = "goal unknown: blind tracking";
      } else {
        // Circle the ball without touching it until a goal is seen again.
        tactic = Tactic::ORBIT;
        wanted = norm(ball.bearing + side_ * 90.0f);
        keepAway = true;
        note = "goal unknown: repositioning";
      }
      speed = cfg.speedSlow;
    } else {
      const float delta = wrap(ball.bearing - goal);
      const float size = fabsf(delta);
      const bool attacking = ball.state != BallState::REACQUIRE &&
                             (previous_ == Tactic::ATTACK ? size <= cfg.attackExitDeg : size <= cfg.attackEnterDeg);
      if (size < 150) side_ = delta < 0 ? -1 : 1;  // keep the side while the ball is nearly straight behind
      if (attacking) {
        tactic = Tactic::ATTACK;
        wanted = norm(ball.bearing + clampf(delta * 0.5f, 15));
        speed = cfg.speedAttack;
      } else if (size < cfg.alignDeg) {
        // IR strength is not a calibrated distance, so there is no far/near switching.
        tactic = Tactic::ALIGN;
        wanted = norm(ball.lead() + clampf(delta * 1.5f, 60));
      } else {
        tactic = Tactic::ORBIT;
        const float offset = size < 90 ? size : 90;
        wanted = norm(ball.bearing + side_ * offset);
        orbitLimit = fminf(orbitLimit, offset);
      }
    }

    // Stalled attack: back off briefly and try again.
    if (tactic == Tactic::ATTACK) {
      if (previous_ != Tactic::ATTACK) attackMs_ = now;
      else if (uint32_t(now - attackMs_) >= cfg.attackStallMs) { recovering_ = true; recoverMs_ = now; }
    }

    // A weaker, older sighting of our own goal is still enough to refuse pushing toward it.
    const bool ownSeen = cam.targetValid && own.seen && own.confidence >= 25 && uint32_t(now - own.stampMs) <= 800;
    const bool ballToOwn = ownSeen && tactic != Tactic::SEARCH && fabsf(wrap(ball.bearing - own.bearing)) < 60;
    const bool ownNear = tactic == Tactic::SEARCH && ownSeen && own.distanceCm < 40;  // unknown counts as near
    auto veto = [&](float d) {
      const float fromBall = fabsf(wrap(d - ball.bearing));
      const bool intoOwn = ownSeen && fabsf(wrap(d - own.bearing)) < cfg.ownGoalGuardDeg;
      if ((ballToOwn && (intoOwn || fromBall < 60)) || (ownNear && intoOwn)) return true;
      if (keepAway) return fromBall < 89.5f;
      return tactic == Tactic::ORBIT && fromBall < orbitLimit - 0.5f;
    };
    // Near a line, and on held line data, only exact directions: the MOTOR applies its strict rule there.
    const int direction = chooseDirection(mask, wanted, veto, previousDirection_, line, held);
    if (direction < 0) return stop(out, "every open direction is unsafe");
    if (fabsf(wrap(direction - wanted)) > 30 && !*note) note = "rerouted";
    const float headingError = heading.absolute ? -heading.heading() : (usable(target) ? wrap(target.bearing) : 0);
    if (mask != 0xFFFF || cam.slow || held || ball.confidence < 40 ||
        (heading.absolute && fabsf(headingError) > 45)) speed = cfg.speedSlow < speed ? cfg.speedSlow : speed;
    out.direction = uint16_t(direction);
    out.power = speed;
    out.avoid = mask != 0xFFFF || held;  // near a line the MOTOR applies its strict feasibility rule
    // Absolute heading: hold it. Goal-relative only: face the goal while lining up, at half gain.
    const bool relative = !heading.absolute && usable(target) &&
                          (tactic == Tactic::ALIGN || tactic == Tactic::ATTACK);
    const bool correct = !out.avoid && !heading.disagreement && (heading.absolute || relative);
    // Camera-only headings are approximate: correct them at half gain, like a goal-relative bearing.
    const bool half = relative || heading.mode == Mode::VISION_ONLY;
    return finish(out, tactic, note, correct, headingError, half ? 0.5f : 1.0f);
  }

private:
  static int faultCount(uint16_t mask) {
    int n = 0;
    for (; mask; mask &= uint16_t(mask - 1)) ++n;
    return n;
  }
  Output &stop(Output &out, const char *reason) { ball = Ball(); return finish(out, Tactic::SAFE_STOP, reason, false, 0); }
  Output &finish(Output &out, Tactic tactic, const char *reason, bool correct, float error, float gain = 1) {
    out.tactic = tactic;
    out.reason = reason;
    out.turn = turn_.update(correct && out.power, error, cfg, gain);
    if (!out.power) out.direction = 0;
    previousDirection_ = out.power ? int(out.direction) : -1;
    previous_ = tactic;
    return out;
  }

  Turn turn_;
  Tactic previous_ = Tactic::SAFE_STOP;
  float goalMemory_ = 0, goalHeading_ = 0;
  int side_ = 1, searchAnchor_ = 0, previousDirection_ = -1;
  uint32_t goalMs_ = 0, attackMs_ = 0, recoverMs_ = 0, searchMs_ = 0, lineMs_ = 0;
  uint16_t lineMask_ = 0, lineEscape_ = kUnknown, lineBearing_ = kUnknown;
  bool haveGoal_ = false, goalGyro_ = false, recovering_ = false, haveLine_ = false;
};

}  // namespace brain
