#pragma once
#include <math.h>
#include <RobotCommon.h>

// Commissioned sweep. Fields describe worst-case travel, including latency and coast,
// over the complete stopping interval. Zero commissioning values deny motion.
struct SweepConfig {
  float halfWidthCm, halfLengthCm, fullTravelCm, fullYawDeg, marginCm, observedCm;
};
struct Sweep {
  uint16_t required;
  bool withinRange;
};
constexpr SweepConfig CompetitionSweep{};
constexpr bool EnableReverseBridge = false;
inline bool permitsSweep(const Sweep& sweep, uint16_t safe) {
  return sweep.withinRange && !(sweep.required & ~safe);
}
inline Sweep candidateRequired(const robot::Pwm& w, const SweepConfig& cfg) {
  if (!(w[0] || w[1] || w[2] || w[3])) return {0, true};
  if (!(cfg.halfWidthCm > 0 && cfg.halfLengthCm > 0 && cfg.fullTravelCm > 0 &&
        cfg.fullYawDeg > 0 && cfg.marginCm > 0 && cfg.observedCm > 0 && cfg.observedCm <= 35 &&
        isfinite(cfg.halfWidthCm + cfg.halfLengthCm + cfg.fullTravelCm + cfg.fullYawDeg + cfg.marginCm))) return {0xffff, false};
  constexpr float rad = 0.01745329252f;
  // Channels are left rear, left front, right front reversed, right rear reversed.
  const float x = (-w[0] + w[1] - w[2] + w[3]) / 840.0f;
  const float y = (w[0] + w[1] + w[2] + w[3]) / 840.0f;
  const float yaw = fabsf((w[0] + w[1] - w[2] - w[3]) / 840.0f) * cfg.fullYawDeg * rad;
  const float corner = hypotf(cfg.halfWidthCm, cfg.halfLengthCm);
  const float travel = hypotf(x, y) * cfg.fullTravelCm;
  // Bound both yaw signs and the full sector width. Unknown directions stay closed.
  const float spread = fminf(3.14159265f, yaw + 11.25f * rad);
  const float offset = 2 * sinf(spread / 2);
  const float rotation = 2 * corner * sinf(fminf(3.14159265f, yaw) / 2);
  uint16_t required = 0;
  for (int sector = 0; sector < 16; ++sector) {
    const float h = sector * 22.5f * rad;
    const float projection = cfg.fullTravelCm * (x * sinf(h) + y * cosf(h));
    if (projection + travel * offset + rotation + cfg.marginCm > 0)
      required |= uint16_t(1u << sector);
  }
  return {required, corner + travel + cfg.marginCm <= cfg.observedCm};
}
