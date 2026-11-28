#pragma once
#include "Arduino.h"

struct SerialPIO : FakeSerial {
  SerialPIO(int, int, int) {}
};
