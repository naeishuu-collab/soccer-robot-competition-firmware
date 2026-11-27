// 拡張IR処理: ir.inoから使う、センサー値の補正・反射ノイズ抑制・信頼度・故障検出のロジック。
// MAIN側の拡張受信処理（main/main_logic.h）と応答形式を共有する。通常の旧コマンド応答には適用しない。
// Filtered IR bearing with confidence and sensor fault handling (poll command 0x02). Host tested.
// The legacy command 0x01 reply does not use any of this.
#pragma once
#include <stdint.h>
#include <math.h>

namespace irx {

constexpr int kSensors = 12;
constexpr uint8_t kReplySize = 10;
// Per-sensor sensitivity correction, permille. 1000 = uncorrected; measure before changing.
constexpr uint16_t kGain[kSensors] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
constexpr float kReflectionRatio = 0.2f;  // below this share of the peak: treated as reflection/noise
constexpr uint16_t kNoise = 10;           // permille
constexpr float kFullLevel = 100.0f;      // peak permille giving full confidence; measure with the real ball
constexpr uint16_t kStuckPolls = 100;     // ~3 s at the 30 ms poll interval
constexpr int kFloodSensors = 8;          // this many saturated at once: ambient IR or wiring, not a ball

struct Result {
  uint16_t angle = 0;       // raw sensor-index angle, same convention as the legacy reply
  uint16_t strength = 0;    // summed detection permille; NOT a distance
  uint8_t confidence = 0;   // 0..100, vector concentration
  uint16_t faults = 0;      // bit per excluded sensor
};

class Filter {
public:
  // count[i]: samples with sensor i active, out of `samples`.
  Result update(const uint16_t *count, uint32_t samples) {
    Result out;
    if (!samples) return out;
    float level[kSensors];
    for (int i = 0; i < kSensors; ++i) {
      raw_[i] = uint16_t(count[i] * 1000UL / samples);
      level[i] = fminf(raw_[i] * kGain[i] / 1000.0f, 1000.0f);
    }
    int saturated = 0;
    for (int i = 0; i < kSensors; ++i) saturated += raw_[i] >= 980;
    if (saturated >= kFloodSensors) {
      valid_ = false;
      out.faults = (1u << kSensors) - 1;  // report every sensor unusable; MAIN does not track
      return out;
    }
    detectFaults();
    float peak = 0;
    for (int i = 0; i < kSensors; ++i) {
      if (faults_ & (1u << i)) {
        // Replace an excluded sensor by its neighbours so its absence does not bias the bearing.
        const int a = (i + kSensors - 1) % kSensors, b = (i + 1) % kSensors;
        const bool ga = !(faults_ & (1u << a)), gb = !(faults_ & (1u << b));
        const float na = ga ? raw_[a] * kGain[a] / 1000.0f : 0, nb = gb ? raw_[b] * kGain[b] / 1000.0f : 0;
        level[i] = (ga && gb) ? (na + nb) / 2 : 0;
      }
      peak = fmaxf(peak, level[i]);
    }
    // A ball lights one run of neighbouring sensors; a reflection shows up as a separate run. Take the
    // strongest run (favouring the previous bearing) and interpolate inside it with its vector sum.
    bool lit[kSensors];
    for (int i = 0; i < kSensors; ++i) lit[i] = level[i] >= kNoise && level[i] >= peak * kReflectionRatio;
    // One silent sensor between two lit ones (dead but not yet excluded) does not split a run.
    bool seen[kSensors];
    for (int i = 0; i < kSensors; ++i) seen[i] = lit[i];
    for (int i = 0; i < kSensors; ++i)
      if (seen[(i + kSensors - 1) % kSensors] && seen[(i + 1) % kSensors]) lit[i] = true;
    int start = 0;
    while (start < kSensors && lit[start]) ++start;  // begin at a gap so no run is split at index 0
    float x = 0, y = 0, total = 0, bestScore = 0, bestTotal = 0, secondTotal = 0;
    float cx = 0, cy = 0, ct = 0;
    for (int n = 1; n <= kSensors; ++n) {
      const int i = (start + n) % kSensors;
      if (start < kSensors && !lit[i]) {
        close(cx, cy, ct, x, y, bestScore, bestTotal, secondTotal);
        total = bestTotal;
        cx = cy = ct = 0;
        continue;
      }
      if (!lit[i]) continue;
      const float radians = i * 6.28318531f / kSensors;
      cx += level[i] * cosf(radians);
      cy += level[i] * sinf(radians);
      ct += level[i];
    }
    close(cx, cy, ct, x, y, bestScore, bestTotal, secondTotal);
    total = bestTotal;
    out.faults = faults_;
    if (total <= 0 || hypotf(x, y) < 1) {
      ++missed_;
      valid_ = false;  // reacquire without dragging an old bearing
      return out;
    }
    missed_ = 0;
    const float concentration = fminf(hypotf(x, y) / total, 1.0f);
    const float ux = x / hypotf(x, y), uy = y / hypotf(x, y);
    // Adaptive circular EMA: follow large moves at once, smooth small jitter.
    float alpha = 1;
    if (valid_) {
      const float change = fabsf(atan2f(ex_ * uy - ey_ * ux, ex_ * ux + ey_ * uy)) * 57.2957795f;
      alpha = fminf(1.0f, 0.35f + change / 45.0f);
    }
    ex_ = valid_ ? ex_ + alpha * (ux - ex_) : ux;
    ey_ = valid_ ? ey_ + alpha * (uy - ey_) : uy;
    const float length = hypotf(ex_, ey_);
    if (length < 0.1f) { ex_ = ux; ey_ = uy; } else { ex_ /= length; ey_ /= length; }
    valid_ = true;
    float angle = atan2f(ey_, ex_) * 57.2957795f;
    if (angle < 0) angle += 360;
    out.angle = uint16_t(angle + 0.5f) % 360;
    out.strength = uint16_t(total);
    // A second run nearly as strong makes the choice doubtful.
    const float ratio = secondTotal / total;
    const float clear = ratio > 0.5f ? fmaxf(0.0f, 1.0f - ratio) : 1.0f;  // the previous run may be the weaker one
    out.confidence = uint8_t(concentration * clear * 100 * fminf(1.0f, peak / kFullLevel) + 0.5f);
    return out;
  }

  uint16_t raw(int i) const { return raw_[i]; }

private:
  void close(float cx, float cy, float ct, float &x, float &y, float &bestScore, float &bestTotal,
             float &secondTotal) const {
    if (ct <= 0) return;
    const float length = hypotf(cx, cy);
    const bool continues = valid_ && length > 0 && (cx * ex_ + cy * ey_) / length > 0.87f;  // within 30 deg
    const float score = ct * (continues ? 1.3f : 1.0f);
    if (score > bestScore) {
      if (bestTotal > secondTotal) secondTotal = bestTotal;
      bestScore = score;
      bestTotal = ct;
      x = cx;
      y = cy;
    } else if (ct > secondTotal) {
      secondTotal = ct;
    }
  }

  // A sensor that is saturated while the opposite side is also silent-or-not for seconds is stuck.
  // A sensor that stays silent while both neighbours see the ball strongly is dead.
  void detectFaults() {
    int saturated = 0;
    for (int i = 0; i < kSensors; ++i) saturated += raw_[i] >= 980;
    for (int i = 0; i < kSensors; ++i) {
      const int a = (i + kSensors - 1) % kSensors, b = (i + 1) % kSensors;
      const bool stuck = raw_[i] >= 980 && saturated <= 2 && raw_[a] < 500 && raw_[b] < 500;
      const bool dead = raw_[i] == 0 && raw_[a] > 300 && raw_[b] > 300;
      if (stuck || dead) {
        if (bad_[i] < kStuckPolls) ++bad_[i];
        good_[i] = 0;
        if (bad_[i] >= kStuckPolls) faults_ |= 1u << i;
      } else {
        bad_[i] = 0;
        // Recovery needs proof of life: a changing, partial detection.
        if (raw_[i] > kNoise && raw_[i] < 980 && ++good_[i] >= 10) faults_ &= ~(1u << i);
      }
    }
  }

  uint16_t raw_[kSensors] = {};
  uint16_t bad_[kSensors] = {}, good_[kSensors] = {};
  uint16_t faults_ = 0;
  float ex_ = 0, ey_ = 0;
  uint8_t missed_ = 0;
  bool valid_ = false;
};

inline uint8_t crc8(const uint8_t *data, int size) {
  uint8_t crc = 0;
  for (int i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) crc = uint8_t((crc << 1) ^ ((crc & 0x80) ? 0x07 : 0));
  }
  return crc;
}

inline void encode(const Result &r, uint8_t *f) {
  f[0] = 0xAB;
  f[1] = r.angle >> 8; f[2] = r.angle & 0xFF;
  f[3] = r.strength >> 8; f[4] = r.strength & 0xFF;
  f[5] = r.confidence;
  f[6] = r.faults >> 8; f[7] = r.faults & 0xFF;
  f[8] = crc8(f + 1, 7);
  f[9] = 0x55;
}

}  // namespace irx
