#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace robot {
using Pwm = int16_t[4];
constexpr int MaxPwm = 210;
constexpr uint16_t Unknown = 0xffff;
constexpr uint8_t Poll[] = {0xa5, 0x5a, 0x01, 0xfe};
constexpr uint32_t Baud = 115200;
constexpr uint32_t PollMs = 30, IrTimeoutMs = 100, CameraTimeoutMs = 200, WatchdogMs = 100;
constexpr uint32_t IrSlotUs = 12000, BusGuardUs = 50;
constexpr uint8_t MotorSize = 13, CameraSize = 23, MotorSync = 0xd7, BridgePwm = 160;
constexpr uint8_t Ready = 1, Slow = 2, Escape = 4, TargetBlue = 8;
constexpr uint16_t Front = 1, Rear = 1 << 8;

inline uint16_t read16(const uint8_t* bytes) {
  return uint16_t(uint16_t(bytes[0]) << 8 | bytes[1]);
}
inline void put16(uint8_t* bytes, uint16_t value) {
  bytes[0] = uint8_t(value >> 8);
  bytes[1] = uint8_t(value);
}
inline uint16_t crc16(const uint8_t* data, size_t size) {
  uint16_t crc = 0xffff;
  while (size--) {
    crc ^= uint16_t(*data++) << 8;
    for (int bit = 0; bit < 8; ++bit)
      crc = uint16_t((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
  }
  return crc;
}
inline void seal(uint8_t* bytes, size_t size) {
  put16(bytes + size - 2, crc16(bytes, size - 2));
}
inline bool validCrc(const uint8_t* bytes, size_t size) {
  return read16(bytes + size - 2) == crc16(bytes, size - 2);
}
inline void encodeMotor(const Pwm& pwm, uint8_t (&bytes)[MotorSize], uint8_t alternate = 0) {
  bytes[0] = MotorSync;
  bytes[1] = 0x69;
  for (int i = 0; i < 4; ++i) put16(bytes + 2 + 2 * i, uint16_t(pwm[i] + MaxPwm));
  bytes[10] = alternate;
  seal(bytes, MotorSize);
}
enum class Decode { Ok, Crc, Fields };
inline Decode decodeMotor(const uint8_t* bytes, Pwm& pwm, uint8_t& alternate) {
  if (!validCrc(bytes, MotorSize)) return Decode::Crc;
  if (bytes[0] != MotorSync || bytes[1] != 0x69 || (bytes[10] && bytes[10] != BridgePwm)) return Decode::Fields;
  for (int i = 0; i < 4; ++i)
    if (read16(bytes + 2 + 2 * i) > 2 * MaxPwm) return Decode::Fields;
  for (int i = 0; i < 4; ++i) pwm[i] = int16_t(int(read16(bytes + 2 + 2 * i)) - MaxPwm);
  alternate = bytes[10];
  if (alternate && (pwm[0] != pwm[1] || pwm[2] != pwm[3] || pwm[0] * pwm[2] >= 0))
    return Decode::Fields;
  return Decode::Ok;
}

struct Goal { uint16_t angle; uint8_t confidence, age; };
struct Camera {
  uint8_t sequence = 0, flags = 0, lineCm = 255, frameMs = 0;
  uint16_t safe = 0, escape = 0, line = Unknown;
  Goal yellow{Unknown, 0, 255}, blue{Unknown, 0, 255};
};
inline bool bearing(uint16_t value) { return value < 360 || value == Unknown; }
inline Decode decodeCamera(const uint8_t* bytes, Camera& camera) {
  if (!validCrc(bytes, CameraSize)) return Decode::Crc;
  const uint8_t flags = bytes[3];
  const uint16_t safe = read16(bytes + 4), escape = read16(bytes + 6);
  const uint16_t line = read16(bytes + 8), yellow = read16(bytes + 12), blue = read16(bytes + 16);
  if (flags & 0xf0 || !bearing(line) || !bearing(yellow) || !bearing(blue) ||
      (line == Unknown) != (bytes[10] == 255) ||
      bool(flags & Slow) != (bytes[11] > 100) || bytes[14] > 100 || bytes[18] > 100 ||
      (yellow == Unknown ? bytes[14] != 0 || bytes[15] != 255 : bytes[15] > 30) ||
      (blue == Unknown ? bytes[18] != 0 || bytes[19] != 255 : bytes[19] > 30) ||
      bytes[20] != 1 || (escape & ~(Front | Rear)) || (escape & ~safe) ||
      (!(flags & Escape) && escape) || ((!(flags & Ready) || (flags & Slow)) && (safe || escape)))
    return Decode::Fields;
  camera = {bytes[2], flags, bytes[10], bytes[11], safe, escape, line,
            {yellow, bytes[14], bytes[15]}, {blue, bytes[18], bytes[19]}};
  return Decode::Ok;
}

// Consume the entire fixed packet, including payload bytes equal to the sync word.
template <size_t Size, uint8_t First, uint8_t Second>
struct Receiver {
  uint8_t bytes[Size]{};
  size_t length = 0;
  uint32_t lastUs = 0;
  bool push(uint8_t value, uint32_t now) {
    if (length && now - lastUs > 4000) length = 0;
    lastUs = now;
    if (length == 0 && value != First) return false;
    if (length == 1 && value != Second) {
      length = value == First ? 1 : 0;
      return false;
    }
    bytes[length++] = value;
    if (length != Size) return false;
    length = 0;
    return true;
  }
  // After corruption, retain a possible header swallowed by a truncated packet.
  void resync() {
    for (size_t i = 1; i + 1 < Size; ++i) {
      if (bytes[i] == First && bytes[i + 1] == Second) {
        length = Size - i;
        memmove(bytes, bytes + i, length);
        return;
      }
    }
    length = bytes[Size - 1] == First ? 1 : 0;
    if (length) bytes[0] = First;
  }
};
}
