#pragma once
#include <stdint.h>

// Exact chase (4).zip controller. The input is the raw ZIP IR bearing.
// Call once per accepted IR reply; reset after a stop or lost detection.
class ZipChasePD {
 public:
  void reset() { initialized_ = false; }
  bool update(uint16_t direction, uint16_t count, uint32_t now, int16_t out[4]) {
    if (!count || direction >= 360) {
      reset();
      for (int i = 0; i < 4; ++i) out[i] = 0;
      return false;
    }
    int error = direction > 180 ? int(direction) - 360 : int(direction);
    float derivative = 0;
    uint32_t dt = now - previousMs_;
    if (initialized_ && dt) {
      int delta = error - previousError_;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      derivative = delta * 1000.0f / dt;
    }
    previousError_ = error;
    previousMs_ = now;
    initialized_ = true;
    proportional_ = 2.0f * error;
    derivative_ = 0.2f * derivative;
    int turn = int(2.0f * error + 0.2f * derivative);
    if (turn > 250) turn = 250;
    if (turn < -250) turn = -250;
    turn_ = turn;
    out[0] = out[1] = limit(175 - turn);
    out[2] = out[3] = limit(175 + turn);
    return true;
  }
  float proportional() const { return proportional_; }
  float derivative() const { return derivative_; }
  int turn() const { return turn_; }
 private:
  static int16_t limit(int value) {
    if (!value) return 0;
    int sign = value < 0 ? -1 : 1;
    int magnitude = value < 0 ? -value : value;
    if (magnitude < 140) magnitude = 140;
    if (magnitude > 210) magnitude = 210;
    return int16_t(sign * magnitude);
  }
  bool initialized_ = false;
  int previousError_ = 0;
  uint32_t previousMs_ = 0;
  float proportional_ = 0, derivative_ = 0;
  int turn_ = 0;
};

// Isolated candidate for evaluation. The ZIP reference above stays byte-for-byte authoritative
// and ENABLE_EXPERIMENTAL_PD_FILTER defaults to 0. This version rejects implausible dt for D,
// wraps bearings at 0/360, and low-pass filters angular velocity; its PWM envelope is unchanged.
class FilteredZipChasePD {
 public:
  void reset() { initialized_ = false; filteredRate_ = 0; }
  bool update(uint16_t direction, uint16_t count, uint32_t now, int16_t out[4]) {
    if (!count || direction >= 360) {
      reset();
      for (int i = 0; i < 4; ++i) out[i] = 0;
      proportional_ = derivative_ = 0;
      turn_ = 0;
      return false;
    }
    const int error = direction > 180 ? int(direction) - 360 : int(direction);
    const uint32_t dt = now - previousMs_;
    float rate = 0;
    if (initialized_ && dt >= 5 && dt <= 100) {
      int delta = error - previousError_;
      if (delta > 180) delta -= 360;
      if (delta < -180) delta += 360;
      rate = float(delta) * 1000.0f / float(dt);
      const float alpha = float(dt) / (30.0f + float(dt));
      filteredRate_ += alpha * (rate - filteredRate_);
    } else {
      // First sample, duplicate timestamp or long/short scheduling gap: do not amplify the D term.
      filteredRate_ = 0;
    }
    previousError_ = error;
    previousMs_ = now;
    initialized_ = true;
    proportional_ = 2.0f * error;
    derivative_ = 0.2f * filteredRate_;
    int turn = int(proportional_ + derivative_);
    if (turn > 250) turn = 250;
    if (turn < -250) turn = -250;
    turn_ = turn;
    out[0] = out[1] = limit(175 - turn);
    out[2] = out[3] = limit(175 + turn);
    return true;
  }
  float proportional() const { return proportional_; }
  float derivative() const { return derivative_; }
  int turn() const { return turn_; }

 private:
  static int16_t limit(int value) {
    if (!value) return 0;
    const int sign = value < 0 ? -1 : 1;
    int magnitude = value < 0 ? -value : value;
    if (magnitude < 140) magnitude = 140;
    if (magnitude > 210) magnitude = 210;
    return int16_t(sign * magnitude);
  }
  bool initialized_ = false;
  int previousError_ = 0;
  uint32_t previousMs_ = 0;
  float filteredRate_ = 0, proportional_ = 0, derivative_ = 0;
  int turn_ = 0;
};
