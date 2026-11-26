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
    float x = 0, y = 0, total = 0;
    for (int i = 0; i < kSensors; ++i) {
      if (level[i] < kNoise || level[i] < peak * kReflectionRatio) continue;
      const float radians = i * 6.28318531f / kSensors;
      x += level[i] * cosf(radians);
      y += level[i] * sinf(radians);
      total += level[i];
    }
    out.faults = faults_;
    if (total <= 0 || hypotf(x, y) < 1) {
      if (++missed_ >= 3) valid_ = false;  // reacquire without dragging an old bearing
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
    out.confidence = uint8_t(concentration * 100 * fminf(1.0f, peak / kFullLevel) + 0.5f);
    return out;
  }

  uint16_t raw(int i) const { return raw_[i]; }

private:
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
