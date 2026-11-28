#pragma once
#include <Arduino.h>
#include "RobotCommon.h"

namespace robot {
inline void rs485Begin(uint8_t enable) {
  digitalWrite(enable, LOW);
  pinMode(enable, OUTPUT);
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(Baud);
}
inline void rs485Send(uint8_t enable, const uint8_t* bytes, size_t size) {
  digitalWrite(enable, HIGH);
  delayMicroseconds(20);
  Serial1.write(bytes, size);
  Serial1.flush();
  delayMicroseconds(20);
  digitalWrite(enable, LOW);
}

// One bounded buffer. Drain short chunks only when USB has room.
template <size_t Size>
struct UsbReport {
  char data[Size]{};
  size_t length = 0, sent = 0;
  bool busy() const { return sent < length; }
  void queue(int count) {
    length = count < 0 ? 0 : (size_t(count) < Size ? size_t(count) : Size - 1);
    sent = 0;
  }
  void pump() {
    if (!busy()) return;
    if (!Serial) { sent = length; return; }
    size_t chunk = length - sent;
    if (chunk > 32) chunk = 32;
    if (Serial.availableForWrite() >= int(chunk))
      sent += Serial.write(reinterpret_cast<const uint8_t*>(data + sent), chunk);
  }
};
}
