#include <BoardIO.h>
#include <math.h>

using namespace robot;
constexpr uint8_t BusEnable = 2;
constexpr uint8_t Pins[] = {18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};
constexpr int16_t Cosine[] = {1000, 866, 500, 0, -500, -866, -1000, -866, -500, 0, 500, 866};
constexpr int16_t Sine[] = {0, 500, 866, 1000, 866, 500, 0, -500, -866, -1000, -866, -500};

Receiver<4, 0xa5, 0x5a> request;
UsbReport<320> report;
uint16_t counts[12]{}, angle = Unknown, strength = 0;
int32_t x = 0, y = 0;
uint32_t polls = 0, replies = 0, invalid = 0, reportMs = 0;
bool requestedReport = false;

void sampleAndReply(uint32_t pollUs) {
  memset(counts, 0, sizeof(counts));
  const uint32_t start = micros();
  while (micros() - start < 5000) {
    for (int i = 0; i < 12; ++i)
      if (digitalRead(Pins[i]) == LOW) ++counts[i];
  }
  x = y = 0;
  uint32_t total = 0;
  for (int i = 0; i < 12; ++i) {
    total += counts[i];
    x += int32_t(counts[i]) * Cosine[i];
    y += int32_t(counts[i]) * Sine[i];
  }
  strength = total > 65535 ? 65535 : uint16_t(total);
  angle = 0;
  if (total && (x || y)) {
    float degrees = float(atan2(float(y), float(x)) * 180.0 / PI);
    if (degrees < 0) degrees += 360.0;
    angle = uint16_t(degrees + 0.5);
    if (angle >= 360) angle = 0;
  } else if (total) {
    angle = Unknown;  // Distinct from a genuine front bearing.
    ++invalid;
  }
  if (micros() - pollUs > 8000) { ++invalid; return; }
  uint8_t frame[] = {0xaa, 0, 0, 0, 0, 0x55};
  put16(frame + 1, angle);
  put16(frame + 3, strength);
  rs485Send(BusEnable, frame, sizeof(frame));
  ++replies;
}

void setup() {
  rs485Begin(BusEnable);
  for (uint8_t pin : Pins) pinMode(pin, INPUT);
  Serial.begin(Baud);
}

void loop() {
  while (Serial1.available()) {
    const uint32_t now = micros();
    if (!request.push(uint8_t(Serial1.read()), now)) continue;
    if (!memcmp(request.bytes, Poll, sizeof(Poll))) {
      ++polls;
      sampleAndReply(now);
    } else { ++invalid; request.resync(); }
  }
  if (Serial.available() && Serial.read() == 'i') requestedReport = true;
  const uint32_t now = millis();
  if (Serial && !report.busy() && (requestedReport || now - reportMs >= 1000)) {
    report.queue(snprintf(report.data, sizeof(report.data),
        "COUNTS=%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u VECTOR=%ld,%ld IR_ANGLE=%d IR_STRENGTH=%u "
        "REQUEST_COUNT=%lu RESPONSE_COUNT=%lu INVALID_COUNT=%lu\n",
        unsigned(counts[0]), unsigned(counts[1]), unsigned(counts[2]), unsigned(counts[3]),
        unsigned(counts[4]), unsigned(counts[5]), unsigned(counts[6]), unsigned(counts[7]),
        unsigned(counts[8]), unsigned(counts[9]), unsigned(counts[10]), unsigned(counts[11]),
        long(x), long(y), strength && angle < 360 ? int(angle) : -1, unsigned(strength),
        (unsigned long)polls, (unsigned long)replies, (unsigned long)invalid));
    requestedReport = false;
    reportMs = now;
  }
  report.pump();
}
