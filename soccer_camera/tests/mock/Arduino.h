// Minimal Arduino stand-in so the real sketches (setup/loop) run on the host.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <deque>
#include <vector>
#include <algorithm>

constexpr int HIGH = 1, LOW = 0, INPUT = 0, OUTPUT = 1;
constexpr float DEG_TO_RAD = 0.01745329252f;
constexpr float PI = 3.14159265358979f;

inline uint32_t mockMs = 0, mockUs = 0;
inline int mockPwm[64] = {}, mockPin[64] = {};
inline uint32_t millis() { return mockMs; }
inline uint32_t micros() { return mockUs += 10; }
inline void delay(uint32_t ms) { mockMs += ms; }
inline void delayMicroseconds(uint32_t us) { mockUs += us; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { mockPin[pin] = value; }
inline int digitalRead(int pin) { return mockPin[pin]; }
inline void analogWrite(int pin, int value) { mockPwm[pin] = value; }
inline void analogWriteFreq(uint32_t) {}
inline void analogWriteRange(uint32_t) {}
template <class T, class L, class H> T constrain(T v, L lo, H hi) { return v < T(lo) ? T(lo) : (v > T(hi) ? T(hi) : v); }
using std::min;
using std::max;

struct MockSerial {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  void begin(uint32_t) {}
  void setTX(int) {}
  void setRX(int) {}
  int available() { return int(rx.size()); }
  int read() { if (rx.empty()) return -1; const int v = rx.front(); rx.pop_front(); return v; }
  int availableForWrite() { return 1024; }
  size_t write(uint8_t b) { tx.push_back(b); return 1; }
  size_t write(const uint8_t *p, size_t n) { tx.insert(tx.end(), p, p + n); return n; }
  void flush() {}
  void print(const char *) {}
  void println(const char *) {}
  void printf(const char *, ...) {}
  void feed(const uint8_t *p, size_t n) { rx.insert(rx.end(), p, p + n); }
  void type(const char *s) { while (*s) rx.push_back(uint8_t(*s++)); }
  explicit operator bool() const { return true; }
};
inline MockSerial Serial, Serial1;
