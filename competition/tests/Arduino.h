#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>
#include <string>

constexpr int LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1;
constexpr double PI = 3.14159265358979323846;
inline uint32_t clockUs, clockMs;
inline unsigned levels[64]{}, sensorBits = 0;

struct FakeSerial {
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
  bool connected = false;
  int room = 512;
  void setTX(int) {}
  void setRX(int) {}
  void begin(uint32_t) {}
  auto available() const { return rx.size(); }
  int read() { int value = rx.front(); rx.pop_front(); return value; }
  explicit operator bool() const { return connected; }
  int availableForWrite() const { return room; }
  size_t write(uint8_t byte) { return write(&byte, 1); }
  size_t write(const uint8_t* data, size_t size) {
    tx.insert(tx.end(), data, data + size);
    return size;
  }
  void flush() {}
  template<class T> void print(T) {}
  template<class T> void println(T) {}
  void inject(const uint8_t* data, size_t size) { rx.insert(rx.end(), data, data + size); }
};

inline FakeSerial Serial, Serial1;
inline uint32_t millis() { return clockMs; }
inline uint32_t micros() { return clockUs; }
inline void delayMicroseconds(unsigned us) { clockUs += us; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int duty) { levels[pin] = unsigned(duty); }
inline int digitalRead(int pin) {
  ++clockUs;
  return sensorBits & (1u << (pin - 18)) ? LOW : HIGH;
}
inline void analogWrite(int pin, int duty) { levels[pin] = unsigned(duty); }
inline void analogWriteFreq(int) {}
inline void analogWriteRange(int) {}
inline int constrain(int val, int lo, int hi) { return std::clamp(val, lo, hi); }
