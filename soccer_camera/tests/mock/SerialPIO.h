#pragma once
#include "Arduino.h"
struct SerialPIO : MockSerial {
  SerialPIO(int, int, int) {}
};
