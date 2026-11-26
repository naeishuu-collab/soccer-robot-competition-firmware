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
  uint16_t speedSlow = 137, speedNormal = 160, speedFast = 185, speedAttack = 195, speedAvoid = 175;
  int turnSign = 1;            // verify with wheels raised
  float headingKp = 0.5f, headingKd = 0.05f;
  float headingEnter = 10, headingExit = 5;
  int turnMin = 8, turnMax = 30, turnSlew = 3;  // turn units, per 20 ms step
  uint32_t cameraTimeoutMs = 200, irTimeoutMs = 100;
  uint32_t searchMs = 1500, attackStallMs = 3000, recoverMs = 400;
  uint8_t ballMinConfidence = 25, goalMinConfidence = 40, headingMinConfidence = 50;
  float attackEnterDeg = 15, attackExitDeg = 30, alignDeg = 45;
  uint8_t nearCm = 25, farCm = 50;
};

inline float wrap(float a) { a = fmodf(a + 180, 360); return (a < 0 ? a + 360 : a) - 180; }
inline float norm(float a) { a = fmodf(a, 360); return a < 0 ? a + 360 : a; }
inline float clampf(float v, float limit) { return v > limit ? limit : (v < -limit ? -limit : v); }

// ---------------- Camera protocol 2 ----------------
constexpr uint8_t kCameraSize = 37;

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
  Seen ball, yellow, blue;
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
    // A repeated sequence number is a stale frame, not fresh data.
    if (out.received && next.sequence == out.sequence && uint32_t(now - out.receivedMs) <= 200) return false;
    out = next;
    return true;
  }

private:
  bool headerPlausible() const {
    return !((used_ >= 2 && buffer_[1] != 0x5C) || (used_ >= 3 && buffer_[2] != 2) ||
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
    if (!seen(d.ball, 14, buffer_[18], now) || !seen(d.yellow, 19, buffer_[24], now) ||
        !seen(d.blue, 25, buffer_[30], now)) return false;
    d.yellow.width = buffer_[23];
    d.blue.width = buffer_[29];
    d.heading = u16(31);
    d.headingConfidence = buffer_[33];
    d.frameMs = buffer_[34];
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
    // Vision: one vote per camera frame.
    const bool vNow = cameraFresh && cam.heading != kUnknown && cam.headingConfidence >= cfg.headingMinConfidence;
    if (!cameraFresh) { vision_ = Gate(); }
    else if (cam.sequence != visionSequence_ || !visionStarted_) {
      visionStarted_ = true;
      visionSequence_ = cam.sequence;
      vision_.feed(vNow, 5, 5);
      if (vNow) visionHeading_ = cam.heading;
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
      const bool was = bno_.open;
      bno_.feed(ok, 10, 3);
      if (bno_.open && !was) {
        // (Re)acquired: choose the zero so the estimate does not jump.
        if (visionOk) offset_ = wrap(yaw_ - visionHeading_);
        else if (haveHeading_ && uint32_t(now - headingMs_) < 2000) offset_ = wrap(yaw_ - heading_);
        else offset_ = yaw_;  // power-on: the robot was placed facing the attack goal
        disagree_ = Gate();
      }
    }
    if (uint32_t(now - bnoStamp_) > 200) bno_ = Gate();

    absolute = true;
    if (bno_.open) {
      float estimate = wrap(yaw_ - offset_);
      if (visionOk) {
        const float diff = wrap(estimate - visionHeading_);
        disagree_.feed(fabsf(diff) > 30, 25, 25);
        disagreement = disagree_.open;
        if (!disagreement) {
          offset_ = wrap(offset_ + 0.01f * diff);  // complementary filter: gyro fast, vision slow
          mode = Mode::FUSED;
        } else if (cam.headingConfidence >= 70) {
          mode = Mode::VISION_ONLY;
          estimate = visionHeading_;
        } else {
          mode = Mode::BNO_ONLY;
        }
      } else {
        disagreement = false;
        mode = Mode::BNO_ONLY;
      }
      store(estimate, now);
    } else if (visionOk) {
      disagreement = false;
      mode = Mode::VISION_ONLY;
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
  bool haveRaw_ = false, haveHeading_ = false, visionStarted_ = false;
};

// PD turn command with deadband hysteresis, a feasible minimum and a slew limit.
class Turn {
public:
  int update(bool enabled, float error, const Config &cfg) {
    int wanted = 0;
    if (enabled) {
      if (fabsf(error) > cfg.headingEnter) engaged_ = true;
      else if (fabsf(error) < cfg.headingExit) engaged_ = false;
      if (engaged_) {
        const float rate = have_ ? wrap(error - previous_) / 0.02f : 0;
        float u = cfg.headingKp * error + cfg.headingKd * clampf(rate, 200);
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

// ---------------- Ball fusion and prediction ----------------
enum class BallState { TRACK, PREDICT, SEARCH, WAIT };
inline const char *name(BallState s) {
  static const char *const n[] = {"TRACK", "PREDICT", "SEARCH", "WAIT"};
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
  int distanceCm = -1;
  float bearing = 0;  // filtered, or predicted while lost
  float rate = 0;     // deg/s relative to the robot (includes the robot's own motion)

  // headingKnown/heading let the track be rotated with the robot (BNO); otherwise prediction stays short.
  void update(const Observation &ir, const Observation &cam, int cameraCm, bool headingKnown, float heading,
              const Config &cfg, uint32_t now) {
    if (headingKnown && headingWasKnown_ && ever_) filtered_ = norm(filtered_ - wrap(heading - heading_));
    headingWasKnown_ = headingKnown;
    heading_ = heading;

    const bool useIr = ir.valid && ir.confidence >= cfg.ballMinConfidence && ir.stampMs != irStamp_;
    const bool useCam = cam.valid && cam.confidence >= cfg.ballMinConfidence &&
                        (!camStamp_ || int32_t(cam.stampMs - camStamp_) > 15);
    if (useIr) irStamp_ = ir.stampMs, lastIr_ = ir;
    if (useCam) camStamp_ = cam.stampMs, lastCam_ = cam;
    if (useIr || useCam) {
      // The other sensor's latest reading still counts while it is recent.
      const bool a = irStamp_ && uint32_t(now - lastIr_.stampMs) <= 100;
      const bool b = camStamp_ && uint32_t(now - lastCam_.stampMs) <= 150;
      float z;
      int conf;
      if (a && b && fabsf(wrap(lastIr_.bearing - lastCam_.bearing)) <= 30) {
        const float total = float(lastIr_.confidence + lastCam_.confidence);
        z = norm(lastIr_.bearing + wrap(lastCam_.bearing - lastIr_.bearing) * lastCam_.confidence / total);
        const int hi = lastIr_.confidence > lastCam_.confidence ? lastIr_.confidence : lastCam_.confidence;
        conf = hi + int(total - hi) / 2;
      } else if (a && b) {
        // Disagreement: quality weighted by continuity with the track; trust it less.
        auto score = [&](const Observation &o) {
          const bool near = ever_ && fabsf(wrap(o.bearing - filtered_)) <= 30;
          return o.confidence * (near ? 1.5f : 1.0f);
        };
        const Observation &o = score(lastIr_) >= score(lastCam_) ? lastIr_ : lastCam_;
        z = o.bearing;
        conf = o.confidence * 7 / 10;
      } else {
        const Observation &o = useIr ? lastIr_ : lastCam_;
        z = o.bearing;
        conf = o.confidence;
      }
      measure(z, uint8_t(conf > 100 ? 100 : conf), now);
    }
    if (cam.valid && uint32_t(now - cam.stampMs) <= 150 && cameraCm >= 0) {
      distanceCm = distanceCm < 0 ? cameraCm : (distanceCm + cameraCm) / 2;
      distanceMs_ = now;
    }
    if (distanceCm >= 0 && uint32_t(now - distanceMs_) > 300) distanceCm = -1;

    if (!ever_) { state = BallState::WAIT; return; }
    const uint32_t lost = now - seenMs_;
    const uint32_t predictMs = headingWasKnown_ ? 100 + 2u * confidence : 100 + confidence;
    if (lost <= 100) {
      state = BallState::TRACK;
      bearing = filtered_;
    } else if (lost <= predictMs) {
      state = BallState::PREDICT;
      bearing = norm(filtered_ + clampf(rate * lost / 1000.0f, 30));
    } else if (lost <= predictMs + cfg.searchMs) {
      state = BallState::SEARCH;
    } else {
      state = BallState::WAIT;
      ever_ = false;
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
    ever_ = true;
    confidence = conf;
    measuredMs_ = seenMs_ = now;
  }

  Observation lastIr_, lastCam_;
  float filtered_ = 0, heading_ = 0, jumpBearing_ = 0;
  uint32_t irStamp_ = 0, camStamp_ = 0, measuredMs_ = 0, seenMs_ = 0, distanceMs_ = 0;
  bool ever_ = false, jump_ = false, headingWasKnown_ = false;
};

// ---------------- Tactics, line safety, speed ----------------
enum class Tactic { SEARCH, APPROACH, ORBIT, ALIGN, ATTACK, RECOVER, AVOID_LINE, SAFE_STOP };
inline const char *name(Tactic t) {
  static const char *const n[] = {"SEARCH", "APPROACH", "ORBIT", "ALIGN", "ATTACK", "RECOVER", "AVOID_LINE", "SAFE_STOP"};
  return n[int(t)];
}

inline bool sectorOpen(uint16_t mask, int sector) { return mask & (1u << (sector & 15)); }
inline int sectorOf(float direction) { return int(norm(direction) / 22.5f + 0.5f) & 15; }

// Returns a direction allowed by the line mask nearest to `wanted`, or -1. The MOTOR may be off by
// up to ~23 degrees at in-between angles, so near lines only exact multiples of 45 or wide openings count.
inline int safeDirection(uint16_t mask, float wanted) {
  if (mask == 0xFFFF) return int(norm(wanted) + 0.5f) % 360;
  auto wide = [&](int k) { return sectorOpen(mask, k - 1) && sectorOpen(mask, k) && sectorOpen(mask, k + 1); };
  if (wide(sectorOf(wanted))) return int(norm(wanted) + 0.5f) % 360;
  int best = -1;
  float bestError = 91;
  for (int k = 0; k < 16; ++k) {
    if (!sectorOpen(mask, k) || (k % 2 && !wide(k))) continue;
    const float error = fabsf(wrap(k * 22.5f - wanted));
    if (error < bestError) { bestError = error; best = int(k * 22.5f + 0.5f); }
  }
  return best;
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
             uint32_t(now - s.stampMs) <= 800;
    };
    heading.update(bno, cam, camFresh, usable(target) || usable(own), cfg, now);

    Observation i, c;
    i.valid = irFresh && ir.strength > 0;
    i.bearing = ir.angle; i.confidence = ir.confidence; i.stampMs = ir.stampMs;
    c.valid = camFresh && cam.ball.seen;
    c.bearing = cam.ball.bearing; c.confidence = cam.ball.confidence; c.stampMs = cam.ball.stampMs;
    const bool gyro = heading.mode == Mode::FUSED || heading.mode == Mode::BNO_ONLY;
    ball.update(i, c, camFresh ? cam.ball.distanceCm : -1, gyro, heading.heading(), cfg, now);

    // Attack direction: the goal itself, else opposite our own goal, else the heading, else brief memory.
    float goal = NAN;
    if (usable(target)) goal = target.bearing;
    else if (usable(own)) goal = norm(own.bearing + 180);
    else if (heading.absolute) goal = norm(-heading.heading());
    if (isfinite(goal)) { goalMemory_ = goal; goalMs_ = now; haveGoal_ = true; }
    else if (haveGoal_ && uint32_t(now - goalMs_) <= 1000) goal = goalMemory_;

    Output out;
    out.goal = goal;
    // 1. communication and recognition failures
    if (!camFresh) return finish(out, Tactic::SAFE_STOP, "camera link", false, 0);
    if (!irFresh) return finish(out, Tactic::SAFE_STOP, "IR link", false, 0);
    if (!cam.ready) return finish(out, Tactic::SAFE_STOP, "line unknown", false, 0);
    if (!cam.mask) return finish(out, Tactic::SAFE_STOP, "no safe direction", false, 0);
    // 2. line escape, with no heading correction
    if (cam.escaping) {
      if (!sectorOpen(cam.mask, sectorOf(cam.escape))) return finish(out, Tactic::SAFE_STOP, "escape closed", false, 0);
      out.direction = cam.escape;
      out.power = cfg.speedAvoid;
      out.avoid = true;
      return finish(out, Tactic::AVOID_LINE, "line", false, 0);
    }

    if (recovering_ && uint32_t(now - recoverMs_) >= cfg.recoverMs) recovering_ = false;
    if (ball.state == BallState::WAIT) return finish(out, Tactic::SEARCH, "waiting for ball", false, 0);

    float wanted = 0;
    uint16_t speed = cfg.speedNormal;
    Tactic tactic = Tactic::APPROACH;
    if (ball.state == BallState::SEARCH) {
      // Fall back toward our own goal for a limited time; only with a calibrated distance to it.
      if (!usable(own) || own.distanceCm < 40)
        return finish(out, Tactic::SEARCH, "ball lost", false, 0);
      tactic = Tactic::SEARCH;
      wanted = own.bearing;
    } else if (recovering_) {
      tactic = Tactic::RECOVER;
      wanted = norm(ball.bearing + 180);
    } else if (!isfinite(goal)) {
      wanted = ball.lead();  // restricted tracking: neither goal nor heading known
      speed = cfg.speedSlow;
    } else {
      const float delta = wrap(ball.bearing - goal);
      const float size = fabsf(delta);
      const bool attacking = previous_ == Tactic::ATTACK ? size <= cfg.attackExitDeg : size <= cfg.attackEnterDeg;
      const bool far = ball.distanceCm > cfg.farCm;
      if (size < 150) side_ = delta < 0 ? -1 : 1;  // keep the side while the ball is nearly straight behind
      if (attacking) {
        tactic = Tactic::ATTACK;
        wanted = norm(ball.bearing + clampf(delta * 0.5f, 15));
        speed = cfg.speedAttack;
      } else if (size < cfg.alignDeg) {
        tactic = far ? Tactic::APPROACH : Tactic::ALIGN;
        wanted = norm((far ? ball.lead() : ball.bearing) + clampf(delta * (far ? 0.5f : 1.5f), 60));
        speed = far ? cfg.speedFast : cfg.speedNormal;
      } else {
        tactic = Tactic::ORBIT;
        float offset = size < 90 ? size : 90;
        if (ball.distanceCm > cfg.nearCm) offset *= float(cfg.nearCm) / ball.distanceCm + 0.2f;
        if (offset > 90) offset = 90;
        wanted = norm(ball.bearing + side_ * offset);
        // Near a line, go round the other way rather than clipping the preferred side.
        const int first = safeDirection(cam.mask, wanted);
        if (first < 0 || fabsf(wrap(first - wanted)) > 45) {
          const float other = norm(ball.bearing - side_ * offset);
          const int second = safeDirection(cam.mask, other);
          if (second >= 0 && fabsf(wrap(second - other)) <= 45) { side_ = -side_; wanted = other; }
        }
        speed = far ? cfg.speedFast : cfg.speedNormal;
      }
    }

    // Stalled attack: back off briefly and try again.
    if (tactic == Tactic::ATTACK) {
      if (previous_ != Tactic::ATTACK) attackMs_ = now;
      else if (uint32_t(now - attackMs_) >= cfg.attackStallMs) { recovering_ = true; recoverMs_ = now; }
    }

    const int direction = safeDirection(cam.mask, wanted);
    if (direction < 0) return finish(out, Tactic::SAFE_STOP, "path blocked by line", false, 0);
    const float headingError = heading.absolute ? -heading.heading() : (usable(target) ? wrap(target.bearing) : 0);
    if (cam.mask != 0xFFFF || cam.slow || ball.confidence < 40 ||
        (heading.absolute && fabsf(headingError) > 45)) speed = cfg.speedSlow < speed ? cfg.speedSlow : speed;
    out.direction = uint16_t(direction);
    out.power = speed;
    // Absolute heading: hold it. Goal-relative only: face the goal while lining up, at half gain.
    const bool relative = !heading.absolute && usable(target) &&
                          (tactic == Tactic::ALIGN || tactic == Tactic::ATTACK);
    const bool correct = (heading.absolute && heading.mode != Mode::DEGRADED) || relative;
    return finish(out, tactic, "", correct, relative ? headingError / 2 : headingError);
  }

private:
  Output &finish(Output &out, Tactic tactic, const char *reason, bool correct, float error) {
    out.tactic = tactic;
    out.reason = reason;
    out.turn = turn_.update(correct && out.power, error, cfg);
    if (!out.power) out.direction = 0;
    previous_ = tactic;
    return out;
  }

  Turn turn_;
  Tactic previous_ = Tactic::SAFE_STOP;
  float goalMemory_ = 0;
  int side_ = 1;
  uint32_t goalMs_ = 0, attackMs_ = 0, recoverMs_ = 0;
  bool haveGoal_ = false, recovering_ = false;
};

}  // namespace brain
