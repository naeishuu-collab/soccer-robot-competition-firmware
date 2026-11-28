#include <BoardIO.h>
#include "motor_output.h"

using namespace robot;
constexpr uint8_t BusEnable = 2;
// Wire order: physical M3, M2, M1, M4, with the ZIP pin order within each pair.
constexpr uint8_t In1[] = {6, 12, 11, 9}, In2[] = {7, 13, 10, 8};
Receiver<MotorSize, 0xd6, 0x69> receiver;
MotorOutput output;
Pwm received{}, written{};
UsbReport<384> report;
uint32_t rxCount = 0, invalidCount = 0, crcCount = 0, reportMs = 0;
bool requestedReport = false;
enum class Status { Startup, Watchdog, Reverse, Zero, Active };
Status lastStatus = Status::Startup;
uint32_t eventMs = 0, eventCount = 0;

void applyOutputs(uint32_t now) {
  output.update(now);
  for (int i = 0; i < 4; ++i) {
    const int16_t pwm = output.applied[i];
    if (pwm == written[i]) continue;
    if (pwm <= 0) analogWrite(In1[i], 0);
    if (pwm >= 0) analogWrite(In2[i], 0);
    if (pwm > 0) analogWrite(In1[i], pwm);
    if (pwm < 0) analogWrite(In2[i], -pwm);
    written[i] = pwm;
  }
  const Status status = !output.seen ? Status::Startup : output.watchdog ? Status::Watchdog :
      output.reverseWait ? Status::Reverse :
      (written[0] || written[1] || written[2] || written[3]) ? Status::Active : Status::Zero;
  if (status != lastStatus) {
    lastStatus = status;
    eventMs = now;
    ++eventCount;
  }
}

void setup() {
  for (int i = 0; i < 4; ++i) {
    digitalWrite(In1[i], LOW);
    digitalWrite(In2[i], LOW);
    pinMode(In1[i], OUTPUT);
    pinMode(In2[i], OUTPUT);
  }
  analogWriteFreq(20000);
  analogWriteRange(255);
  rs485Begin(BusEnable);
  Serial.begin(Baud);
}

void loop() {
  applyOutputs(millis());  // Enforce the watchdog even if RX stays busy.
  for (int n = 0; n < 64 && Serial1.available(); ++n) {
    if (!receiver.push(uint8_t(Serial1.read()), micros())) continue;
    Pwm pwm;
    const Decode result = decodeMotor(receiver.bytes, pwm);
    if (result == Decode::Ok) {
      ++rxCount;
      memcpy(received, pwm, sizeof(received));
      output.accept(pwm, millis());
      applyOutputs(millis());
    } else {
      ++invalidCount;
      if (result == Decode::Crc) ++crcCount;
      receiver.resync();
    }
  }
  if (Serial.available() && Serial.read() == 'i') requestedReport = true;
  const uint32_t now = millis();
  if (Serial && !report.busy() && (requestedReport || now - reportMs >= 1000)) {
    const char* const names[] = {"STARTUP", "WATCHDOG", "REVERSE_WAIT", "ZERO_COMMAND", "NONE"};
    report.queue(snprintf(report.data, sizeof(report.data),
        "RX_COUNT=%lu INVALID_FRAME_COUNT=%lu CRC_ERROR_COUNT=%lu LAST_COMMAND_AGE_MS=%ld\n"
        "PWM_RECEIVED=%d,%d,%d,%d PWM_TARGET=%d,%d,%d,%d PWM_APPLIED=%d,%d,%d,%d\n"
        "REVERSE_WAIT=%u REVERSE_MASK=%x WATCHDOG_STOP=%u STOP_REASON=%s EVENT_COUNT=%lu LAST_EVENT_MS=%lu\n",
        (unsigned long)rxCount, (unsigned long)invalidCount, (unsigned long)crcCount,
        output.seen ? long(now - output.lastCommandMs) : -1L,
        received[0], received[1], received[2], received[3],
        output.desired[0], output.desired[1], output.desired[2], output.desired[3],
        written[0], written[1], written[2], written[3], unsigned(output.reverseWait),
        unsigned(output.reverseMask), unsigned(output.watchdog), names[int(lastStatus)],
        (unsigned long)eventCount, (unsigned long)eventMs));
    requestedReport = false;
    reportMs = now;
  }
  report.pump();
}
