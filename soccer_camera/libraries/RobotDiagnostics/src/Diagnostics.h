#pragma once
#include <stddef.h>
#include <stdint.h>

#ifndef ROBOT_FW_VERSION
#define ROBOT_FW_VERSION "0.1.0"
#endif
#ifndef ROBOT_GIT_COMMIT
#define ROBOT_GIT_COMMIT "unembedded"
#endif

namespace robotdiag {
enum class StopReason : uint8_t {
  NONE, MANUAL_STOP, IR_TIMEOUT, IR_INVALID, BALL_LOST, CAMERA_TIMEOUT,
  CAMERA_NOT_READY, CAMERA_SLOW, CAMERA_INVALID, LINE_DETECTED,
  NO_SAFE_DIRECTION, ROTATION_BLOCKED, MOTOR_TIMEOUT, INVALID_FRAME,
  INVALID_PWM, REVERSE_WAIT, CONFIG_ERROR, INTERNAL_ERROR
};
inline const char *name(StopReason reason) {
  static const char *const names[] = {
    "NONE", "MANUAL_STOP", "IR_TIMEOUT", "IR_INVALID", "BALL_LOST",
    "CAMERA_TIMEOUT", "CAMERA_NOT_READY", "CAMERA_SLOW", "CAMERA_INVALID",
    "LINE_DETECTED", "NO_SAFE_DIRECTION", "ROTATION_BLOCKED", "MOTOR_TIMEOUT",
    "INVALID_FRAME", "INVALID_PWM", "REVERSE_WAIT", "CONFIG_ERROR", "INTERNAL_ERROR"
  };
  const uint8_t value = static_cast<uint8_t>(reason);
  return value < sizeof(names) / sizeof(names[0]) ? names[value] : "INTERNAL_ERROR";
}
enum class EventKind : uint8_t { STATE, STOP, COMM_ERROR, SENSOR_VALIDITY, PWM, CONTROL };
struct Event { uint32_t atMs = 0; EventKind kind = EventKind::CONTROL; uint16_t code = 0; int16_t value = 0; };
// Fixed-size RAM history; oldest records are overwritten without blocking control.
template <size_t Capacity> class EventRing {
  static_assert(Capacity > 0, "EventRing needs storage");
 public:
  void push(uint32_t atMs, EventKind kind, uint16_t code, int16_t value = 0) {
    records_[next_] = {atMs, kind, code, value};
    next_ = (next_ + 1) % Capacity;
    if (size_ < Capacity) ++size_; else ++overwritten_;
  }
  size_t size() const { return size_; }
  uint32_t overwritten() const { return overwritten_; }
  const Event &oldest(size_t index) const { return records_[(next_ + Capacity - size_ + index) % Capacity]; }
 private:
  Event records_[Capacity] = {};
  size_t next_ = 0, size_ = 0;
  uint32_t overwritten_ = 0;
};
}  // namespace robotdiag
