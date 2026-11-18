#include "../common/src/RobotControl.ino"
#include "../common/src/BoardConfig.ino"
#include "../common/src/RobotDevices.ino"
#include "../common/src/Rs485.ino"
#include <math.h>
#include <stdio.h>
#include <string.h>

using namespace soccer;

static unsigned testCases = 0;
static unsigned assertions = 0;
static unsigned failures = 0;
static const char* currentCase = "";

#define CHECK(expr) do { \
    ++assertions; \
    if (!(expr)) { \
        ++failures; \
        printf("FAIL [%s] line %d: %s\n", currentCase, __LINE__, #expr); \
    } \
} while (0)

// Synthetic host-only values. The tested runtime profile is not derived from these pins or wheel angles.
static const uint32_t kTestPwmPinMask = (1u << 14) | (1u << 15) | (1u << 16) | (1u << 17) |
                                        (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21);
static const uint32_t kTestReservedPinMask = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 25);
static const uint32_t kTestSafetyReservedPinMask = kTestPwmPinMask | kTestReservedPinMask;
static const WheelConfig kTestWheels[4] = {
    {1, 1, 135, 1}, {-1, 1, 225, 1}, {-1, -1, 315, 1}, {1, -1, 45, 1}
};

static const Tb67h450ChannelConfig kTestDrivers[4] = {
    {14, 15, kTb67h450MaxDuty}, {16, 17, kTb67h450MaxDuty},
    {18, 19, kTb67h450MaxDuty}, {20, 21, kTb67h450MaxDuty}
};

// Host fixtures use the current PCB-angle candidate table; runtime remains gated until measured.
static const int16_t* const kLineTestAngles = board::kLineSensorAngleCdeg;
static const uint32_t kLineTestAngleMask = board::kLineSensorAngleValidMask;

static LineObservation lineFromActive(uint32_t activeBits, bool activeHigh = true) {
    const uint32_t raw = activeHigh ? activeBits : (~activeBits & 0x00FFFFFFu);
    return calculateLine(raw, true, activeHigh, kLineTestAngles, kLineTestAngleMask,
                         board::kLineSensorEnabledMask);
}

static void testLineFaultMaskExcludesFailedSensors() {
    CHECK(countBits24(board::kLineSensorEnabledMask) == 18);
    CHECK(board::kLineSensorEnabledMask == 0x00DFF4DDu);
    const uint32_t failed = (1u << 1) | (1u << 5) | (1u << 8) |
                            (1u << 9) | (1u << 11) | (1u << 21);
    const LineObservation onlyFailed = calculateLine(failed, true, true, kLineTestAngles,
                                                       kLineTestAngleMask, board::kLineSensorEnabledMask);
    CHECK(onlyFailed.rawLevels == 0 && onlyFailed.bits == 0);
    CHECK(!onlyFailed.detected && onlyFailed.count == 0 && !onlyFailed.directionValid);

    const uint32_t validBit = 1u << 2;
    const LineObservation clean = calculateLine(validBit, true, true, kLineTestAngles,
                                                 kLineTestAngleMask, board::kLineSensorEnabledMask);
    const LineObservation withFailedNoise = calculateLine(validBit | failed, true, true, kLineTestAngles,
                                                           kLineTestAngleMask, board::kLineSensorEnabledMask);
    CHECK(clean.detected && withFailedNoise.detected);
    CHECK(withFailedNoise.bits == validBit && withFailedNoise.count == clean.count);
    CHECK(withFailedNoise.angleCdeg == clean.angleCdeg);

    const LineObservation unknownPolarity = calculateLine(failed, false, true, kLineTestAngles,
                                                            kLineTestAngleMask, board::kLineSensorEnabledMask);
    CHECK(!unknownPolarity.polarityValid && !unknownPolarity.detected && unknownPolarity.count == 0);
    const LineObservation faultGap = calculateLine((1u << 8) | (1u << 9), true, true, kLineTestAngles,
                                                    kLineTestAngleMask, board::kLineSensorEnabledMask);
    CHECK(!faultGap.detected && faultGap.count == 0);

    const LineObservation allHealthy = calculateLine(board::kLineSensorEnabledMask, true, true,
                                                       kLineTestAngles, kLineTestAngleMask,
                                                       board::kLineSensorEnabledMask);
    CHECK(allHealthy.detected && allHealthy.count == 18);
    Packet packet = makeLinePacket(withFailedNoise, 1);
    LineObservation decoded{};
    CHECK(decodeLinePacket(packet, decoded, kLineTestAngles, kLineTestAngleMask,
                           board::kLineSensorEnabledMask));
    CHECK(decoded.rawLevels == validBit && decoded.bits == validBit);
    Packet badPacket = makeLinePacket(clean, 2);
    badPacket.payload[0] |= static_cast<uint8_t>(1u << 1);
    CHECK(!decodeLinePacket(badPacket, decoded, kLineTestAngles, kLineTestAngleMask,
                            board::kLineSensorEnabledMask));
    packet.payload[6] = 2; // A peer that counts a disabled input is rejected.
    CHECK(!decodeLinePacket(packet, decoded, kLineTestAngles, kLineTestAngleMask,
                            board::kLineSensorEnabledMask));
}

static void testLineCandidateAnglesUseSignedCentidegreesAndStayGated() {
    const int16_t expected[24] = {
        1100, 2100, 4100, 5100, 7100, 8100, 10100, 11100, 13100, 14100, 16100, 17100,
        -16900, -15900, -13900, -12900, -10900, -9900, -7900, -6900, -4900, -3900, -1900, -900
    };
    for (uint8_t i = 0; i < 24; ++i) CHECK(board::kLineSensorAngleCdeg[i] == expected[i]);
    const board::LineAngleTable offset = board::makeLineAngleTable(1100);
    CHECK(offset.v[0] == 0 && offset.v[23] == -2000);
    CHECK(!board::kLineFrontOffsetConfigured && !board::kLineAnglesConfigured &&
          !board::kLinePolarityConfigured && !board::kCompetitionArmed);
    CHECK(board::kMotorHardwareConfigured && !board::kKickerHardwareConfigured);
}

static bool parseFrame(const uint8_t* bytes, uint8_t length, PacketParser& parser,
                       Packet& decoded, uint32_t startMs = 0) {
    bool ready = false;
    for (uint8_t i = 0; i < length; ++i)
        ready = parser.feed(bytes[i], startMs + i, decoded) || ready;
    return ready;
}

static void recalculateCrc(uint8_t* bytes, uint8_t length) {
    const uint16_t crc = crc16(bytes + 2, static_cast<uint8_t>(length - 4));
    bytes[length - 2] = static_cast<uint8_t>(crc);
    bytes[length - 1] = static_cast<uint8_t>(crc >> 8);
}

static void feedIr(const uint8_t bytes[6], uint32_t startMs, LegacyIrParser& parser,
                   uint16_t& angle, uint16_t& strength, bool& ready) {
    ready = false;
    for (uint8_t i = 0; i < 6; ++i)
        ready = parser.feed(bytes[i], startMs + i, angle, strength) || ready;
}

static void testCrcKnownVectorAndFrameRoundTrip() {
    const uint8_t text[] = "123456789";
    CHECK(crc16(text, 9) == 0x29B1);
    const Packet packet = makeLineRequest(255);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(packet, bytes, sizeof(bytes));
    CHECK(length == 8);
    PacketParser parser;
    Packet decoded{};
    CHECK(parseFrame(bytes, length, parser, decoded));
    CHECK(decodeLineRequest(decoded) && decoded.sequence == 255);
}

static void testBadCrcRejected() {
    Packet packet = makeLineRequest(9);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(packet, bytes, sizeof(bytes));
    bytes[length - 1] ^= 0x80;
    PacketParser parser;
    Packet decoded{};
    CHECK(!parseFrame(bytes, length, parser, decoded));
    CHECK(parser.stats().crcErrors == 1);
}

static void testDroppedPacketSequenceGap() {
    PacketParser parser;
    SequenceTracker sequence;
    Packet decoded{};
    uint8_t bytes[kMaxFrame]{};
    Packet p = makeLineRequest(20);
    uint8_t length = encodePacket(p, bytes, sizeof(bytes));
    CHECK(parseFrame(bytes, length, parser, decoded, 0));
    CHECK(sequence.accept(decoded.sequence));
    p.sequence = 22;  // Sequence 21 represents the dropped frame.
    length = encodePacket(p, bytes, sizeof(bytes));
    CHECK(parseFrame(bytes, length, parser, decoded, 20));
    CHECK(sequence.accept(decoded.sequence));
    CHECK(sequence.gaps() == 1);
}

static void testPartialFrameWaitsForRemainder() {
    Packet packet = makeLineRequest(4);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(packet, bytes, sizeof(bytes));
    PacketParser parser;
    Packet decoded{};
    bool ready = false;
    for (uint8_t i = 0; i < 5; ++i) ready |= parser.feed(bytes[i], i, decoded);
    CHECK(!ready);
    for (uint8_t i = 5; i < length; ++i) ready |= parser.feed(bytes[i], i, decoded);
    CHECK(ready && decoded.sequence == 4);
}

static void testPartialFrameTimeout() {
    PacketParser parser;
    Packet ignored{};
    CHECK(!parser.feed(kFrameStart0, 0, ignored));
    CHECK(!parser.feed(kFrameStart1, 1, ignored));
    CHECK(!parser.feed(kProtocolVersion, 2, ignored));
    CHECK(!parser.feed(kFrameStart0, 11, ignored));
    CHECK(parser.stats().timeouts == 1);
}

static void testMotorCommandFrameRoundTripAndValidation() {
    int16_t requested[4] = {-255, 1, 255, kMotorBrakeCommand};
    uint8_t bytes[kMotorCommandFrameSize]{};
    CHECK(encodeMotorCommandFrame(requested, bytes, sizeof(bytes)) == kMotorCommandFrameSize);
    CHECK(bytes[0] == 0xAA && bytes[1] == 4 && bytes[2] == 1 && bytes[3] == 0xFF);
    CHECK(bytes[4] == 1 && bytes[5] == 0 && bytes[6] == 0xFF && bytes[7] == 0);
    CHECK(bytes[8] == 0 && bytes[9] == 1 && bytes[11] == 0x55);
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < 10; ++i) checksum ^= bytes[i];
    CHECK(bytes[10] == checksum);

    MotorCommandFrameParser parser;
    int16_t decoded[4]{};
    MotorFrameStatus status = kMotorFramePending;
    for (uint8_t i = 0; i < sizeof(bytes); ++i)
        status = parser.feed(bytes[i], i, decoded);
    CHECK(status == kMotorFrameReady);
    for (uint8_t i = 0; i < 4; ++i) CHECK(decoded[i] == requested[i]);

    requested[0] = 257;
    CHECK(encodeMotorCommandFrame(requested, bytes, sizeof(bytes)) == 0);
    requested[0] = 0;
    CHECK(encodeMotorCommandFrame(requested, bytes, sizeof(bytes)) == kMotorCommandFrameSize);
    bytes[10] ^= 1;
    MotorCommandFrameParser badChecksum;
    for (uint8_t i = 0; i < sizeof(bytes); ++i)
        status = badChecksum.feed(bytes[i], i, decoded);
    CHECK(status == kMotorFrameInvalid && badChecksum.stats().checksumErrors == 1);

    CHECK(encodeMotorCommandFrame(requested, bytes, sizeof(bytes)) == kMotorCommandFrameSize);
    bytes[11] = 0;
    MotorCommandFrameParser badEnd;
    for (uint8_t i = 0; i < sizeof(bytes); ++i)
        status = badEnd.feed(bytes[i], i, decoded);
    CHECK(status == kMotorFrameInvalid && badEnd.stats().formatErrors == 1);

    requested[0] = 0;
    CHECK(encodeMotorCommandFrame(requested, bytes, sizeof(bytes)) == kMotorCommandFrameSize);
    bytes[2] = 1; // 257 is rejected even when the XOR is correct.
    bytes[3] = 1;
    checksum = 0;
    for (uint8_t i = 0; i < 10; ++i) checksum ^= bytes[i];
    bytes[10] = checksum;
    MotorCommandFrameParser badRange;
    for (uint8_t i = 0; i < sizeof(bytes); ++i)
        status = badRange.feed(bytes[i], i, decoded);
    CHECK(status == kMotorFrameInvalid && badRange.stats().rangeErrors == 1);

    MotorCommandFrameParser timedOut;
    CHECK(timedOut.feed(kMotorCommandStart, 0, decoded) == kMotorFramePending);
    CHECK(timedOut.poll(kFrameTimeoutMs + 1));
    CHECK(timedOut.stats().timeouts == 1);
}

static void testMotorParserIgnoresSharedBusTraffic() {
    MotorCommandFrameParser parser;
    int16_t decoded[4]{};
    const uint8_t irResponse[] = {0xAA, 0x00, 0xB4, 0x12, 0x34, 0x55};
    for (uint8_t i = 0; i < sizeof(irResponse); ++i)
        CHECK(parser.feed(irResponse[i], i, decoded) == kMotorFramePending);
    CHECK(parser.stats().formatErrors == 0);

    Packet line{};
    line.type = kLineStatus;
    line.length = 2;
    line.payload[0] = 0xAA;
    line.payload[1] = 0x04;
    uint8_t lineBytes[kMaxFrame]{};
    const uint8_t lineLength = encodePacket(line, lineBytes, sizeof(lineBytes));
    CHECK(lineLength == 10);
    for (uint8_t i = 0; i < lineLength; ++i)
        CHECK(parser.feed(lineBytes[i], 10 + i, decoded) == kMotorFramePending);
    CHECK(parser.stats().formatErrors == 0);

    int16_t requested[4] = {8, -8, 0, 4};
    uint8_t motorBytes[kMotorCommandFrameSize]{};
    CHECK(encodeMotorCommandFrame(requested, motorBytes, sizeof(motorBytes)) == kMotorCommandFrameSize);
    MotorFrameStatus status = kMotorFramePending;
    for (uint8_t i = 0; i < sizeof(motorBytes); ++i)
        status = parser.feed(motorBytes[i], 30 + i, decoded);
    CHECK(status == kMotorFrameReady);
    for (uint8_t i = 0; i < 4; ++i) CHECK(decoded[i] == requested[i]);

    MotorCommandFrameParser invalidIr;
    const uint8_t invalidIrResponse[] = {0xAA, 0x01, 0x68, 0x12, 0x34, 0x55}; // Direction 360 is invalid.
    for (uint8_t i = 0; i < sizeof(invalidIrResponse); ++i)
        status = invalidIr.feed(invalidIrResponse[i], i, decoded);
    CHECK(status == kMotorFrameInvalid && invalidIr.stats().formatErrors == 1);
}

static void testMotorConsoleCommandParsing() {
    int16_t values[4] = {9, 9, 9, 9};
    CHECK(parseMotorConsoleCommand("M -255 0 200 256", values));
    CHECK(values[0] == -255 && values[1] == 0 && values[2] == 200 && values[3] == 256);
    CHECK(parseMotorConsoleCommand(" s ", values));
    for (int16_t value : values) CHECK(value == 0);
    CHECK(parseMotorConsoleCommand("m +1 -2 +3 -4 \t", values));
    CHECK(values[0] == 1 && values[1] == -2 && values[2] == 3 && values[3] == -4);
    CHECK(!parseMotorConsoleCommand("M 1 2 3", values));
    CHECK(!parseMotorConsoleCommand("M 1 2 3 4 5", values));
    CHECK(!parseMotorConsoleCommand("M 257 0 0 0", values));
    CHECK(!parseMotorConsoleCommand("M -256 0 0 0", values));
    CHECK(!parseMotorConsoleCommand("M 1 x 3 4", values));
    CHECK(!parseMotorConsoleCommand("STOP", values));
}

static void testMainConsoleAutoModesAndFailClosedGates() {
    int16_t values[4] = {9, 9, 9, 9};
    CHECK(parseMainConsoleCommand(" AUTO ", values) == kConsoleAutoWheelLift);
    for (int16_t value : values) CHECK(value == 0);
    CHECK(parseMainConsoleCommand("auto float", values) == kConsoleAutoWheelLift);
    CHECK(parseMainConsoleCommand("AUTO FLOOR\t", values) == kConsoleAutoFloor);
    CHECK(parseMainConsoleCommand("AUTO IR", values) == kConsoleAutoIrTest);
    CHECK(parseMainConsoleCommand("arm ir", values) == kConsoleArmIrTest);
    CHECK(parseMainConsoleCommand("ARM IR NOW", values) == kConsoleInvalid);
    CHECK(parseMainConsoleCommand("AUTO FLOOR NOW", values) == kConsoleInvalid);
    CHECK(parseMainConsoleCommand("AUTOX", values) == kConsoleInvalid);
    CHECK(parseMainConsoleCommand("M 100 100 100 100", values) == kConsoleManualMotor);
    CHECK(values[0] == 100 && values[1] == 100 && values[2] == 100 && values[3] == 100);
    CHECK(parseMainConsoleCommand("S", values) == kConsoleStop);
    CHECK(autoModeForCommand(kConsoleAutoWheelLift, true, false) == kAutoWheelLift);
    CHECK(autoModeForCommand(kConsoleAutoFloor, true, false) == kAutoStopped);
    CHECK(autoModeForCommand(kConsoleAutoFloor, true, true) == kAutoFloor);
    CHECK(autoModeForCommand(kConsoleAutoIrTest, false, false) == kAutoIrTest);
    CHECK(autoModeForCommand(kConsoleStop, true, true) == kAutoStopped);
    CHECK(autoModeForCommand(kConsoleManualMotor, true, true) == kAutoStopped);
    CHECK(autoModeForCommand(kConsoleInvalid, true, true) == kAutoStopped);
    CHECK(!board::kAutoWheelLiftReady && !board::kAutoFloorReady);
    CHECK(board::kLineSensorEnabledMask != 0x00FFFFFFu);
    CHECK(!board::kLineDebounceConfigured);
}

static void testUsbInputCancelsBootAutoArm() {
    // MAIN calls disableAutoStart() on the first USB byte, not only on a complete line.
    IrTestController controller;
    Motion motion{1.0f, 1.0f, 1.0f};
    controller.disableAutoStart();
    for (uint16_t i = 0; i < 1000; ++i) {
        CHECK(!controller.armOnFirstFreshBall(true, true, static_cast<uint16_t>(i % 360)));
        CHECK(!controller.armed());
    }
    // An explicit manual ARM remains available after the boot auto-arm is cancelled.
    CHECK(controller.arm(true, true, 0));
    CHECK(controller.update(true, true, 0, motion));
    CHECK(controller.armed());
}

static void testIrTestContinuousTrackingAndCoarseSteering() {
    IrTestController controller;
    Motion motion{1.0f, 1.0f, 1.0f};
    IrTestController bootController;
    CHECK(!bootController.armOnFirstFreshBall(false, true, 0));
    CHECK(!bootController.armOnFirstFreshBall(true, false, 0));
    CHECK(bootController.armOnFirstFreshBall(true, true, 0));
    CHECK(!bootController.armOnFirstFreshBall(true, true, 0));
    for (uint16_t i = 0; i < 600; ++i) CHECK(bootController.update(true, true, 0, motion));
    CHECK(bootController.armed());
    CHECK(!bootController.update(false, true, 0, motion));
    CHECK(!bootController.armOnFirstFreshBall(true, true, 0));
    IrTestController cancelledBootController;
    cancelledBootController.disableAutoStart();
    CHECK(!cancelledBootController.armOnFirstFreshBall(true, true, 0));
    CHECK(cancelledBootController.arm(true, true, 0));

    CHECK(!controller.update(true, true, 0, motion));
    CHECK(motion.x == 0 && motion.y == 0 && motion.turn == 0);
    CHECK(!controller.arm(false, true, 0));
    CHECK(!controller.arm(true, false, 0));
    CHECK(!controller.arm(true, true, 360));
    CHECK(controller.arm(true, true, 359));
    CHECK(controller.update(true, true, 0, motion));
    CHECK(motion.x == 0 && motion.y > 0 && motion.turn == 0);
    CHECK(controller.update(true, true, 30, motion));
    CHECK(motion.x == 0 && motion.y == 0 && motion.turn < 0);
    CHECK(controller.update(true, true, 330, motion));
    CHECK(motion.x == 0 && motion.y == 0 && motion.turn > 0);
    CHECK(!controller.update(false, true, 30, motion));
    CHECK(!controller.armed() && motion.x == 0 && motion.y == 0 && motion.turn == 0);
    CHECK(!controller.update(true, true, 30, motion));

    CHECK(controller.arm(true, true, 0));
    CHECK(!controller.update(true, false, 0, motion));
    CHECK(!controller.armed());
    CHECK(controller.arm(true, true, 0));
    for (uint16_t i = 0; i < 600; ++i) CHECK(controller.update(true, true, 0, motion));
    CHECK(controller.armed());
    CHECK(!controller.update(true, false, 0, motion));
    CHECK(!controller.armed());

    CHECK(controller.arm(true, true, 30));
    CHECK(!controller.update(true, true, 360, motion));
    CHECK(!controller.armed());
    CHECK(!irTestMotionForBearing(360, motion));
    CHECK(motion.x == 0 && motion.y == 0 && motion.turn == 0);
}

static void testAutoMotorPwmCaps() {
    int16_t values[4] = {255, -255, 24, -24};
    capMotorCommandValues(values, kIrTestPwmCap);
    CHECK(values[0] == 16 && values[1] == -16 && values[2] == 16 && values[3] == -16);
    values[0] = 255; values[1] = -255; values[2] = 24; values[3] = -24;
    capMotorCommandValues(values, 48);
    CHECK(values[0] == 48 && values[1] == -48 && values[2] == 24 && values[3] == -24);
    capMotorCommandValues(values, 24);
    CHECK(values[0] == 24 && values[1] == -24 && values[2] == 24 && values[3] == -24);
}

static void testSequenceWrapAndRejectDuplicates() {
    SequenceTracker sequence;
    CHECK(sequence.accept(254));
    CHECK(sequence.accept(255));
    CHECK(sequence.accept(0));
    CHECK(sequence.accept(1));
    CHECK(!sequence.accept(1));
    CHECK(!sequence.accept(0));
    CHECK(sequence.rejected() == 2);
    sequence.resetPeer();
    CHECK(sequence.accept(0)); // A timed-out peer may restart its sequence at zero.
    CHECK(sequence.gaps() == 0 && sequence.rejected() == 2);
}

struct MockRs485 {
    char events[32]{};
    uint8_t used = 0;
    uint32_t delays[3]{};
    uint8_t delayCount = 0;
    bool shortWrite = false;
    uint8_t rx[32]{};
    uint8_t rxCount = 0;
    uint8_t rxIndex = 0;
};
static void rs485Event(MockRs485* mock, char event) { mock->events[mock->used++] = event; }
static int rs485Available(void* context) {
    MockRs485* mock = static_cast<MockRs485*>(context);
    return mock->rxCount - mock->rxIndex;
}
static int rs485Read(void* context) {
    MockRs485* mock = static_cast<MockRs485*>(context);
    return mock->rxIndex < mock->rxCount ? mock->rx[mock->rxIndex++] : -1;
}
static void rs485Direction(void* context, bool tx) { rs485Event(static_cast<MockRs485*>(context), tx ? 'T' : 'R'); }
static size_t rs485Write(void* context, const uint8_t*, size_t length) {
    MockRs485* mock = static_cast<MockRs485*>(context);
    rs485Event(mock, 'W');
    return mock->shortWrite ? length - 1 : length;
}
static void rs485Flush(void* context) { rs485Event(static_cast<MockRs485*>(context), 'F'); }
static void rs485Delay(void* context, uint32_t us) {
    MockRs485* mock = static_cast<MockRs485*>(context);
    rs485Event(mock, 'D');
    mock->delays[mock->delayCount++] = us;
}

static void testRs485HalfDuplexOrderingAndFailure() {
    MockRs485 mock{};
    Rs485Io io{&mock, rs485Available, rs485Read, rs485Direction, rs485Write, rs485Flush, rs485Delay};
    Rs485Driver bus{io};
    const uint8_t bytes[] = {0xA5, 0x5A, 0x00};
    CHECK(bus.ready());
    CHECK(bus.send(bytes, sizeof(bytes), 0, false, 12, 14, 16));
    mock.events[mock.used] = '\0';
    CHECK(strcmp(mock.events, "TDWFDRD") == 0);
    CHECK(mock.delays[0] == 12 && mock.delays[1] == 14 && mock.delays[2] == 16);
    CHECK(bus.diagnostics().txFrames == 1 && bus.diagnostics().txBytes == sizeof(bytes));
    mock = {};
    mock.shortWrite = true;
    io.context = &mock;
    Rs485Driver failedBus{io};
    CHECK(!failedBus.send(bytes, sizeof(bytes), 0));
    mock.events[mock.used] = '\0';
    CHECK(strcmp(mock.events, "TDWFDRD") == 0); // DE returns low even after a short write.
    CHECK(failedBus.diagnostics().txErrors == 1);
}

static void testRs485ReceiveTimeoutAndCollisionGuard() {
    MockRs485 mock{};
    mock.rx[0] = 0x42;
    mock.rxCount = 1;
    Rs485Driver bus{{&mock, rs485Available, rs485Read, rs485Direction, rs485Write, rs485Flush, rs485Delay}};
    uint8_t byte = 0;
    CHECK(bus.receiveByte(byte, 1000) && byte == 0x42);
    CHECK(bus.diagnostics().rxBytes == 1 && bus.linkFresh(1050, 100));
    CHECK(!bus.canTransmit(1050)); // Master waits for the quiet guard after a received frame.
    const uint8_t request = 0x01;
    CHECK(!bus.send(&request, 1, 1050));
    CHECK(bus.diagnostics().txBusy == 1);
    CHECK(bus.canTransmit(1100));
    CHECK(bus.send(&request, 1, 1100));
    CHECK(!bus.linkFresh(1101, 100));
}

static void testSharedBusIrRequestDemultiplexing() {
    SharedBusIrRequestParser parser;
    const Packet linePoll = makeLineRequest(7);
    uint8_t wire[kMaxFrame]{};
    uint8_t length = encodePacket(linePoll, wire, sizeof(wire));
    bool request = false;
    for (uint8_t i = 0; i < length; ++i) request |= parser.feed(wire[i], i);
    CHECK(!request); // The 0x01 protocol-version byte inside a framed LINE poll is not an IR request.

    const int16_t motorValues[4] = {1, -1, kMotorBrakeCommand, 0};
    uint8_t motorFrame[kMotorCommandFrameSize]{};
    length = encodeMotorCommandFrame(motorValues, motorFrame, sizeof(motorFrame));
    for (uint8_t i = 0; i < length; ++i) request |= parser.feed(motorFrame[i], 20 + i);
    CHECK(!request);
    CHECK(parser.feed(0x01, 40));

    SharedBusIrRequestParser partial;
    partial.feed(kFrameStart0, 0);
    partial.feed(kFrameStart1, 1);
    partial.feed(kProtocolVersion, 2);
    partial.feed(kLineRequest, 3);
    partial.feed(0, 4);
    CHECK(partial.poll(13));
    CHECK(partial.stats().timeouts == 1);
    CHECK(partial.feed(0x01, 14));
}

static void testIrRequestAndLegacySixByteResponse() {
    SharedBusIrRequestParser requestParser;
    CHECK(requestParser.feed(0x01, 0));
    const uint8_t response[6] = {0xAA, 0x01, 0x67, 0x12, 0x34, 0x55};
    LegacyIrParser responseParser;
    uint16_t direction = 0, strength = 0;
    bool ready = false;
    for (uint8_t i = 0; i < sizeof(response); ++i)
        ready |= responseParser.feed(response[i], i, direction, strength);
    CHECK(ready && direction == 359 && strength == 0x1234);
}

static void testUnknownMessageTypeRejected() {
    Packet packet = makeLineRequest(1);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(packet, bytes, sizeof(bytes));
    bytes[3] = 0x7F;
    recalculateCrc(bytes, length);
    PacketParser parser;
    Packet decoded{};
    CHECK(!parseFrame(bytes, length, parser, decoded));
    CHECK(parser.stats().formatErrors == 1);
}

static void testOversizedPayloadLengthRejected() {
    PacketParser parser;
    Packet decoded{};
    CHECK(!parser.feed(kFrameStart0, 0, decoded));
    CHECK(!parser.feed(kFrameStart1, 1, decoded));
    CHECK(!parser.feed(kProtocolVersion, 2, decoded));
    CHECK(!parser.feed(0x7F, 3, decoded));
    CHECK(!parser.feed(kMaxPayload + 1, 4, decoded));
    CHECK(parser.stats().formatErrors == 1);
}

static void testIrZeroAnd359Boundaries() {
    LegacyIrParser parser;
    uint16_t angle = 999, strength = 0;
    bool ready = false;
    const uint8_t zero[] = {0xAA, 0, 0, 0, 5, 0x55};
    feedIr(zero, 0, parser, angle, strength, ready);
    CHECK(ready && angle == 0 && strength == 5);
    const uint8_t edge[] = {0xAA, 1, 0x67, 0, 7, 0x55};
    feedIr(edge, 10, parser, angle, strength, ready);
    CHECK(ready && angle == 359 && strength == 7);
}

static void testIrNoBallAndInvalidAngle() {
    LegacyIrParser parser;
    uint16_t angle = 999, strength = 999;
    bool ready = false;
    const uint8_t noBall[] = {0xAA, 0, 0, 0, 0, 0x55};
    feedIr(noBall, 0, parser, angle, strength, ready);
    CHECK(ready && angle == 0 && strength == 0);
    const uint8_t invalid[] = {0xAA, 1, 0x68, 0, 8, 0x55};
    feedIr(invalid, 10, parser, angle, strength, ready);
    CHECK(!ready && parser.errors() == 1);
}

static void testIrDirectionCoherenceMetric() {
    const int16_t cosine[12] = {1000, 866, 500, 0, -500, -866, -1000, -866, -500, 0, 500, 866};
    const int16_t sine[12] = {0, 500, 866, 1000, 866, 500, 0, -500, -866, -1000, -866, -500};
    uint16_t weights[12] = {};
    weights[0] = 40;
    IrDirectionEstimate estimate = estimateIrDirection(weights, cosine, sine, 12, 10);
    CHECK(estimate.valid && estimate.totalWeight == 40);
    CHECK(fabsf(estimate.angleDeg) < 0.01f && fabsf(estimate.coherence - 1.0f) < 0.001f);
    weights[6] = 40;
    estimate = estimateIrDirection(weights, cosine, sine, 12, 10);
    CHECK(!estimate.valid && estimate.coherence < 0.001f);
    weights[0] = weights[6] = 0;
    estimate = estimateIrDirection(weights, cosine, sine, 12, 1);
    CHECK(!estimate.valid && estimate.totalWeight == 0 && estimate.coherence == 0);
    CHECK(!estimateIrDirection(nullptr, cosine, sine, 12, 1).valid);
}

static void testIrPredictionWrapsBearingBoundsSpeedAndRecoversFromClockWrap() {
    float angles[4] = {359.0f, 1.0f, 0.0f, 0.0f};
    float signals[4] = {10.0f, 20.0f, 0.0f, 0.0f};
    uint32_t times[4] = {0xFFFFFF00u, 99744u, 0, 0};
    IrPrediction prediction = predictIr(angles, signals, times, 2, 2, 4, 0.03f, 720.0f);
    CHECK(prediction.valid && fabsf(prediction.angleDeg - 1.6f) < 0.02f);
    CHECK(fabsf(prediction.relativeSignal - 23.0f) < 0.02f);

    angles[0] = 0.0f; angles[1] = 90.0f;
    signals[0] = 0.0f; signals[1] = 20.0f;
    times[0] = 0; times[1] = 50000u;
    prediction = predictIr(angles, signals, times, 2, 2, 4, 0.1f, 720.0f);
    CHECK(prediction.valid && fabsf(prediction.angleDeg - 162.0f) < 0.02f);
    CHECK(fabsf(prediction.relativeSignal - 60.0f) < 0.02f);

    times[1] = 500u;
    prediction = predictIr(angles, signals, times, 2, 2, 4, 0.1f, 720.0f);
    CHECK(prediction.valid && prediction.angleDeg == 90.0f);
    CHECK(!predictIr(angles, signals, times, 0, 1, 0, 0.1f, 720.0f).valid);
}

static void testLineDebounceRejectsShortTransitions() {
    LineDebouncer filter;
    CHECK(filter.update(0x1u, 2) == 0x1u); // First scan seeds the stable state immediately.
    CHECK(filter.update(0x0u, 2) == 0x1u);
    CHECK(filter.update(0x1u, 2) == 0x1u); // A one-scan glitch is rejected.
    CHECK(filter.update(0x0u, 2) == 0x1u);
    CHECK(filter.update(0x0u, 2) == 0x0u);
    CHECK(filter.update(0x3u, 2) == 0x0u);
    CHECK(filter.update(0x1u, 2) == 0x1u); // Each sensor bit has its own history.
    CHECK(filter.update(0xFF000001u, 1) == 0x1u); // Levels above sensor 23 are ignored.
    filter.reset();
    CHECK(filter.update(0x4u, 2) == 0x4u);
}

static void testLineSingleSensor() {
    const LineObservation line = lineFromActive(1u);
    CHECK(line.detected && !line.ambiguous && line.count == 1);
    CHECK(line.angleCdeg == 1100);
    const LineObservation right = lineFromActive(1u << 6);
    const LineObservation left = lineFromActive(1u << 18);
    CHECK(right.angleCdeg >= 10099 && right.angleCdeg <= 10101);
    CHECK(left.angleCdeg >= -7901 && left.angleCdeg <= -7899);
}

static void testLinePollResponseCrcAndSequence() {
    Packet poll = makeLineRequest(42);
    uint8_t bytes[kMaxFrame]{};
    uint8_t length = encodePacket(poll, bytes, sizeof(bytes));
    CHECK(length == 8);
    PacketParser lineParser;
    Packet receivedPoll{};
    CHECK(parseFrame(bytes, length, lineParser, receivedPoll));
    CHECK(decodeLineRequest(receivedPoll) && receivedPoll.sequence == 42);

    const LineObservation expected = lineFromActive(1u << 6);
    Packet response = makeLinePacket(expected, receivedPoll.sequence);
    length = encodePacket(response, bytes, sizeof(bytes));
    PacketParser mainParser;
    Packet receivedResponse{};
    CHECK(parseFrame(bytes, length, mainParser, receivedResponse));
    LineObservation decoded{};
    CHECK(decodeLinePacket(receivedResponse, decoded, kLineTestAngles, kLineTestAngleMask));
    CHECK(receivedResponse.sequence == 42 && decoded.rawLevels == expected.rawLevels);
    CHECK(decoded.angleCdeg == expected.angleCdeg && decoded.directionValid);
}

static void testLineMultipleAdjacentSensors() {
    const LineObservation line = lineFromActive((1u << 2) | (1u << 3));
    CHECK(line.detected && !line.ambiguous && line.count == 2);
    CHECK(line.angleCdeg > 4500 && line.angleCdeg < 4700); // Adjacent healthy sensors use their non-uniform angles.
}

static void testLineOpposingSensorsAreAmbiguous() {
    const LineObservation line = lineFromActive((1u << 0) | (1u << 12));
    CHECK(line.detected && line.ambiguous && line.count == 2);
    CHECK(line.angleCdeg == kNoLineAngle);
}

static void testLineAngleWrapAndPacketConsistency() {
    const LineObservation aroundZero = lineFromActive((1u << 23) | 1u);
    CHECK(aroundZero.detected && !aroundZero.ambiguous && aroundZero.directionValid);
    CHECK(aroundZero.angleCdeg > 0 && aroundZero.angleCdeg < 200);
    Packet linePacket = makeLinePacket(aroundZero, 7);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(linePacket, bytes, sizeof(bytes));
    PacketParser parser;
    Packet decoded{};
    CHECK(parseFrame(bytes, length, parser, decoded));
    LineObservation received{};
    CHECK(decodeLinePacket(decoded, received, kLineTestAngles, kLineTestAngleMask));
    received.angleCdeg += 1000;
    linePacket = makeLinePacket(received, 8);
    CHECK(!decodeLinePacket(linePacket, received, kLineTestAngles, kLineTestAngleMask));
}

static void testLineUnknownPolarityAndAngleConfiguration() {
    const LineObservation unknownPolarity = calculateLine(0x00000005u, false, true,
                                                            kLineTestAngles, kLineTestAngleMask,
                                                            board::kLineSensorEnabledMask);
    CHECK(!unknownPolarity.polarityValid && !unknownPolarity.detected && unknownPolarity.count == 2);
    CHECK(unknownPolarity.angleCdeg == kNoLineAngle && !unknownPolarity.directionValid);
    const LineObservation noAngles = calculateLine(1u, true, true, nullptr, 0);
    CHECK(noAngles.detected && !noAngles.directionValid && noAngles.angleCdeg == kNoLineAngle);
    const LineObservation lowActive = lineFromActive(1u << 6, false);
    CHECK(lowActive.detected && lowActive.bits == (1u << 6) && lowActive.rawLevels != lowActive.bits);
    Packet p = makeLinePacket(unknownPolarity, 1);
    LineObservation decoded{};
    CHECK(decodeLinePacket(p, decoded, nullptr, 0, board::kLineSensorEnabledMask) &&
          !decoded.polarityValid && decoded.rawLevels == 5);
}

static MainInput safeArmedInput() {
    MainInput input{};
    input.armed = true;
    input.emergencyStop = false;
    input.clearFault = false;
    input.lineFresh = true;
    input.line = lineFromActive(0);
    input.irFresh = true;
    input.ballFound = true;
    input.ballAngleDeg = 0;
    input.relativeSignal = 500;
    input.headingValid = true;
    input.headingDeg = 0;
    input.headingTargetDeg = 0;
    return input;
}

static void testLineDetectedWithoutAnglesStopsSafely() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.line = calculateLine(1u, true, true, nullptr, 0);
    const MainOutput output = controller.step(input);
    CHECK(output.state == kAvoidLine);
    CHECK(output.motion.x == 0 && output.motion.y == 0 && output.motion.turn == 0);
}

static void testLineAvoidancePriorityHoldAndAmbiguity() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.line = lineFromActive(1u);
    MainOutput output = controller.step(input);
    CHECK(output.state == kAvoidLine && output.motion.y < 0);
    input.line = lineFromActive(0);
    input.nowMs = 10;
    CHECK(controller.step(input).state == kAvoidLine);
    input.nowMs = 209;
    CHECK(controller.step(input).state == kAvoidLine);
    input.nowMs = 210;
    CHECK(controller.step(input).state == kApproachBall);

    MainController ambiguousController;
    input = safeArmedInput();
    input.line = lineFromActive((1u << 0) | (1u << 12));
    output = ambiguousController.step(input);
    CHECK(output.state == kAvoidLine);
    CHECK(output.motion.x == 0 && output.motion.y == 0);

    MainController rightLineController;
    input = safeArmedInput();
    input.line = lineFromActive(1u << 6);
    output = rightLineController.step(input);
    CHECK(output.state == kAvoidLine && output.motion.x < 0 && output.motion.y > 0.05f);
}

static void testOmniForwardAndBackward() {
    float forward[4]{}, backward[4]{};
    CHECK(omniKinematics({0, 1, 0}, kTestWheels, forward));
    CHECK(omniKinematics({0, -1, 0}, kTestWheels, backward));
    for (uint8_t i = 0; i < 4; ++i) CHECK(fabsf(forward[i] + backward[i]) < 0.001f);
}

static void testOmniLeftAndRight() {
    float right[4]{}, left[4]{};
    CHECK(omniKinematics({1, 0, 0}, kTestWheels, right));
    CHECK(omniKinematics({-1, 0, 0}, kTestWheels, left));
    for (uint8_t i = 0; i < 4; ++i) CHECK(fabsf(right[i] + left[i]) < 0.001f);
}

static void testOmniClockwiseAndCounterclockwise() {
    float ccw[4]{}, cw[4]{};
    CHECK(omniKinematics({0, 0, 1}, kTestWheels, ccw));
    CHECK(omniKinematics({0, 0, -1}, kTestWheels, cw));
    for (uint8_t i = 0; i < 4; ++i) CHECK(ccw[i] > 0 && cw[i] < 0);
}

static void testOmniBlendNormalization() {
    float wheels[4]{};
    CHECK(omniKinematics({1, 1, 1}, kTestWheels, wheels));
    float largest = 0;
    for (float value : wheels) {
        CHECK(isfinite(value) && fabsf(value) <= 1.0f);
        if (fabsf(value) > largest) largest = fabsf(value);
    }
    CHECK(fabsf(largest - 1.0f) < 0.001f);
    CHECK(!omniKinematics({1.01f, 0, 0}, kTestWheels, wheels));
}

static void testMotorSlewLimit() {
    float wheels[4]{};
    MotorWheelSlew slew[4]{};
    CHECK(safeMotorUpdate({0, 1, 0}, true, true, kTestWheels, slew, wheels, 0.1f, 0, 200));
    for (float value : wheels) CHECK(fabsf(value) <= 0.1001f);
    CHECK(safeMotorUpdate({0, 1, 0}, true, true, kTestWheels, slew, wheels, 0.1f, 5, 200));
    for (float value : wheels) CHECK(fabsf(value) <= 0.2001f);
    float reversing = 0.0f;
    MotorWheelSlew wheel;
    CHECK(wheel.update(1.0f, 1.0f, 0, 200, reversing) && reversing == 1.0f);
    CHECK(wheel.update(-1.0f, 0.5f, 10, 200, reversing) && reversing == 0.5f);
    CHECK(wheel.update(-1.0f, 0.5f, 20, 200, reversing) && reversing == 0.0f);
    CHECK(wheel.update(-1.0f, 0.5f, 219, 200, reversing) && reversing == 0.0f);
    CHECK(wheel.update(-1.0f, 0.5f, 220, 200, reversing) && reversing == -0.5f);
    CHECK(wheel.update(1.0f, 1.0f, 300, 200, reversing) && reversing == 0.0f);
    CHECK(wheel.update(1.0f, 1.0f, 301, 200, reversing) && reversing == 0.0f);
    CHECK(wheel.update(1.0f, 1.0f, 500, 200, reversing) && reversing == 1.0f);
    CHECK(!wheel.update(NAN, 0.1f, 501, 200, reversing) && reversing == 0.0f);
}

static void testTb67h450TruthTableAndDutyLimits() {
    Tb67h450Output output{};
    CHECK(tb67h450Encode(kTb67h450Stop, 0, 128, output));
    CHECK(output.mode == kTb67h450Stop && output.in1 == 0 && output.in2 == 0);
    CHECK(tb67h450Encode(kTb67h450Forward, 128, 128, output));
    CHECK(output.mode == kTb67h450Forward && output.in1 == 128 && output.in2 == 0);
    CHECK(tb67h450Encode(kTb67h450Reverse, 128, 128, output));
    CHECK(output.mode == kTb67h450Reverse && output.in1 == 0 && output.in2 == 128);
    CHECK(tb67h450Encode(kTb67h450Brake, 0, 128, output));
    CHECK(output.mode == kTb67h450Brake && output.in1 == 255 && output.in2 == 255);
    CHECK(tb67h450Encode(kTb67h450Forward, 0, 128, output));
    CHECK(output.mode == kTb67h450Stop && output.in1 == 0 && output.in2 == 0);
    CHECK(tb67h450Encode(kTb67h450Forward, 255, 255, output) && output.in1 == 255);
    CHECK(!tb67h450Encode(kTb67h450Reverse, 256, 255, output));
    CHECK(!tb67h450Encode(kTb67h450Stop, 1, 255, output));
    CHECK(!tb67h450Encode(kTb67h450Brake, 1, 255, output));
    CHECK(!tb67h450Encode(kTb67h450Forward, 1, 256, output));
    CHECK(!tb67h450Encode(kTb67h450Forward, 1, 0, output));
    CHECK(!tb67h450Encode(static_cast<Tb67h450Mode>(255), 0, 128, output));
    CHECK(!tb67h450RequiresNeutralDwell(kTb67h450Forward, kTb67h450Brake, false, 0, 0, 200));
    CHECK(!tb67h450RequiresNeutralDwell(kTb67h450Stop, kTb67h450Brake, false, 0, 0, 200));
    CHECK(tb67h450RequiresNeutralDwell(kTb67h450Forward, kTb67h450Reverse, true, 500, 0, 200));
    CHECK(tb67h450RequiresNeutralDwell(kTb67h450Stop, kTb67h450Forward, false, 500, 0, 200));
    CHECK(tb67h450RequiresNeutralDwell(kTb67h450Stop, kTb67h450Forward, true, 199, 0, 200));
    CHECK(!tb67h450RequiresNeutralDwell(kTb67h450Stop, kTb67h450Forward, true, 200, 0, 200));
    CHECK(!tb67h450RequiresNeutralDwell(kTb67h450Forward, kTb67h450Forward, true, 0, 0, 200));
    CHECK(tb67h450RequiresNeutralDwell(kTb67h450Brake, kTb67h450Forward, true, 500, 500, 200));
    CHECK(tb67h450OutputForWheel(0.0f, 128, output) && output.mode == kTb67h450Stop);
    CHECK(tb67h450OutputForWheel(0.5f, 128, output) && output.mode == kTb67h450Forward && output.in1 == 64);
    CHECK(tb67h450OutputForWheel(-1.0f, 128, output) && output.mode == kTb67h450Reverse && output.in2 == 128);
    CHECK(!tb67h450OutputForWheel(1.01f, 128, output));
    CHECK(!tb67h450OutputForWheel(NAN, 128, output));
    CHECK(kMotorPwmRange == 255 && kTb67h450PwmFrequencyHz == 20000 &&
          kTb67h450MaxDuty == 255 && kTb67h450ReversalDwellMs == 200);
}

static void testMotorStopPriority() {
    MotorCommandPriority priority;
    CHECK(priority.accept(true));
    CHECK(priority.accept(false) && priority.stopPending());
    CHECK(!priority.accept(true)); // A stop cannot be overwritten before the output tick.
    CHECK(priority.accept(false) && priority.stopPending());
    priority.completeOutputTick();
    CHECK(!priority.stopPending() && priority.accept(true));
    priority.latchStop();
    CHECK(!priority.accept(true));
}

static void testHeadingBoundaryAndJumpRejection() {
    CHECK(normalizeDegrees(360.0f) == 0.0f);
    CHECK(fabsf(shortestAngleError(1, 359) - 2) < 0.001f);
    CHECK(fabsf(shortestAngleError(359, 1) + 2) < 0.001f);
    HeadingController heading;
    float turn = 1;
    CHECK(heading.update(359, 0, true, turn));
    CHECK(heading.update(1, 0, true, turn));
    CHECK(fabsf(turn + 0.012f) < 0.001f);
    CHECK(!heading.update(100, 0, true, turn) && turn == 0);
    CHECK(heading.update(100.5f, 0, true, turn)); // A stable second sample confirms a real heading step.
    CHECK(!heading.update(NAN, 0, true, turn) && turn == 0);
}

struct MockBno {
    uint8_t availableAddress = 0;
    bool readFails = false;
    float heading = 0.0f;
    uint8_t calibration[4] = {3, 3, 3, 3};
    uint8_t beginCalls = 0;
};
static bool mockBnoBegin(void* context, uint8_t address) {
    MockBno* mock = static_cast<MockBno*>(context);
    ++mock->beginCalls;
    return mock->availableAddress == address;
}
static bool mockBnoRead(void* context, float& heading, uint8_t calibration[4]) {
    MockBno* mock = static_cast<MockBno*>(context);
    if (mock->readFails || !mock->availableAddress) return false;
    heading = mock->heading;
    memcpy(calibration, mock->calibration, 4);
    return true;
}

static void testBnoInitFailureDisconnectInvalidDataAndRecovery() {
    MockBno mock{};
    Bno055Io io{&mock, mockBnoBegin, mockBnoRead};
    Bno055Monitor monitor;
    Bno055Sample sample{};
    CHECK(!monitor.poll(io, kBnoAddressAuto, 0, sample)); // Neither strap address responds.
    CHECK(!monitor.initialized() && mock.beginCalls == 2);
    mock.availableAddress = 0x29;
    CHECK(!monitor.poll(io, kBnoAddressAuto, 999, sample));
    CHECK(mock.beginCalls == 2); // Retry delay prevents repeated bus probing.
    mock.heading = 359.8f;
    CHECK(monitor.poll(io, kBnoAddressAuto, 1000, sample));
    CHECK(sample.connected && sample.headingValid && sample.address == 0x29);
    CHECK(fabsf(sample.headingDeg - 359.8f) < 0.01f);

    mock.readFails = true;
    CHECK(!monitor.poll(io, kBnoAddressAuto, 1020, sample));
    CHECK(!sample.connected && !monitor.initialized());
    mock.readFails = false;
    CHECK(!monitor.poll(io, kBnoAddressAuto, 2019, sample));
    CHECK(monitor.poll(io, kBnoAddressAuto, 2020, sample)); // Reinitialized after an outage.

    mock.heading = NAN;
    CHECK(!monitor.poll(io, kBnoAddressAuto, 2040, sample));
    CHECK(sample.connected && !sample.headingValid);
    mock.heading = 90;
    mock.calibration[0] = 2;
    CHECK(!monitor.poll(io, kBnoAddressAuto, 2060, sample)); // Unverified calibration is not valid heading.
    CHECK(sample.connected && !sample.headingValid);
    mock.calibration[0] = 3;
    CHECK(monitor.poll(io, 0x29, 2080, sample)); // Explicit address override is supported.
}

static KickerInput readyKickerInput() {
    KickerInput input{};
    input.commandFresh = true;
    input.armed = true;
    input.emergencyStop = false;
    input.hardwareConfigured = true; // Model only; no physical output adapter exists.
    input.ballFresh = true;
    input.ballDetected = true;
    input.headingValid = true;
    input.headingAligned = true;
    input.lineClear = true;
    return input;
}

static void testMainKickerRequestPolicyGates() {
    CHECK(!board::kKickerRequestPolicyConfigured);
    CHECK(!board::kKickerHeadingWindowConfigured);
    KickerInput input = readyKickerInput();
    CHECK(!kickerRequestAllowed(false, true, input));
    CHECK(!kickerRequestAllowed(true, false, input));
    CHECK(kickerRequestAllowed(true, true, input));
    input.emergencyStop = true;
    CHECK(!kickerRequestAllowed(true, true, input));
    input.emergencyStop = false;
    input.lineClear = false;
    CHECK(!kickerRequestAllowed(true, true, input));
}

static void testKickerAndDribblerOutputGates() {
    KickerController kicker;
    CHECK(!board::kKickerHardwareConfigured);
    CHECK(!board::kBnoOffsetsVerified);
    KickerInput input = readyKickerInput();
    input.hardwareConfigured = false;
    input.request = true;
    KickerOutput output = kicker.update(input, 1000);
    CHECK(output.state == kKickerDisarmed && !output.mockPulseActive);
    input.hardwareConfigured = true;
    output = kicker.update(input, 1001); // A request held while disabled is not queued.
    CHECK(output.state == kKickerReady && !output.mockPulseActive);
    output = kicker.update(input, 1002);
    CHECK(output.state == kKickerReady && !output.mockPulseActive);
    input.request = false;
    kicker.update(input, 1003);
    input.request = true;
    output = kicker.update(input, 1004);
    CHECK(output.state == kKickerFiring && output.mockPulseActive);
    output = kicker.update(input, 1063);
    CHECK(output.state == kKickerFiring && output.mockPulseActive);
    output = kicker.update(input, 1064);
    CHECK(output.state == kKickerCooldown && !output.mockPulseActive);
    output = kicker.update(input, 2063);
    CHECK(output.state == kKickerCooldown && !output.mockPulseActive);
    output = kicker.update(input, 2064);
    CHECK(output.state == kKickerReady && !output.mockPulseActive);
    CHECK(kicker.eventHistoryCount() >= 4);
    KickerEventRecord event{};
    CHECK(kicker.eventHistoryAt(kicker.eventHistoryCount() - 1, event));
    CHECK(event.event == kKickerBecameReady && event.state == kKickerReady);

    const auto blocked = [](KickerInput blockedInput) {
        KickerController controller;
        blockedInput.request = false;
        controller.update(blockedInput, 0);
        blockedInput.request = true;
        return controller.update(blockedInput, 1);
    };
    input.request = false;
    input.commandFresh = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.armed = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.hardwareConfigured = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.ballFresh = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.ballDetected = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.headingValid = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.headingAligned = false;
    CHECK(!blocked(input).mockPulseActive);
    input = readyKickerInput(); input.lineClear = false;
    CHECK(!blocked(input).mockPulseActive);

    KickerController latched;
    input = readyKickerInput();
    latched.update(input, 0);
    input.request = true;
    CHECK(latched.update(input, 1).mockPulseActive);
    input.emergencyStop = true;
    output = latched.update(input, 10);
    CHECK(output.state == kKickerFault && !output.mockPulseActive);
    input.emergencyStop = false;
    input.clearFault = true;
    CHECK(latched.update(input, 11).state == kKickerFault); // Held request cannot clear or retrigger.
    input.request = false;
    CHECK(latched.update(input, 12).state == kKickerDisarmed);
    input.clearFault = false;
    CHECK(latched.update(input, 13).state == kKickerReady);
    input.request = true;
    CHECK(latched.update(input, 14).mockPulseActive);

    KickerController wrapped;
    input = readyKickerInput();
    const uint32_t nearWrap = UINT32_MAX - 40u;
    CHECK(wrapped.update(input, nearWrap).state == kKickerReady);
    input.request = true;
    CHECK(wrapped.update(input, nearWrap + 1u).mockPulseActive);
    output = wrapped.update(input, 20u);
    CHECK(output.state == kKickerCooldown && !output.mockPulseActive);

    DribblerController dribbler;
    CHECK(dribbler.update(0.6f, true, true, false, false, false) == 0.0f);
    CHECK(dribbler.update(0.6f, true, true, false, false, true) == 0.6f);
    CHECK(dribbler.update(NAN, true, true, false, false, true) == 0.0f);
    CHECK(dribbler.update(1.1f, true, true, false, false, true) == 0.0f);
    CHECK(dribbler.update(0.6f, false, true, false, false, true) == 0.0f);
    CHECK(dribbler.update(0.6f, true, false, false, false, true) == 0.0f);
    CHECK(dribbler.update(0.6f, true, true, true, false, true) == 0.0f);
    CHECK(dribbler.update(0.6f, true, true, false, true, true) == 0.0f);
}
static void testSearchApproachAndDisarmedStates() {
    MainController controller;
    MainInput input{};
    input.emergencyStop = false;
    MainOutput output = controller.step(input);
    CHECK(output.state == kDisarmed && output.motion.y == 0 && output.motion.turn == 0);
    input = safeArmedInput();
    input.headingValid = true;
    input.headingDeg = 0;
    output = controller.step(input);
    CHECK(output.state == kApproachBall && output.motion.y > 0);
    input.ballFound = false;
    input.nowMs = 100;
    output = controller.step(input);
    CHECK(output.state == kSearchBall && output.motion.x > 0 && output.motion.turn == 0);
    input.nowMs = 800;
    output = controller.step(input);
    CHECK(output.state == kSearchBall && output.motion.x < 0 && output.motion.turn == 0);
}

static void testLineSensorCommunicationLossLatchesFault() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.lineFresh = false;
    MainOutput output = controller.step(input);
    CHECK(output.state == kFault && output.motion.x == 0 && output.motion.y == 0 && output.motion.turn == 0);
    input.lineFresh = true;
    output = controller.step(input);
    CHECK(output.state == kFault); // Requires explicit recovery and a subsequent re-arm.
}

static void testIrSensorCommunicationLossLatchesFault() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.irFresh = false;
    MainOutput output = controller.step(input);
    CHECK(output.state == kFault && output.motion.x == 0 && output.motion.y == 0 && output.motion.turn == 0);
    input.irFresh = true;
    output = controller.step(input);
    CHECK(output.state == kFault); // Fresh data alone cannot restart motion after a fault.
}

static void testUnknownLinePolarityFaultsArmedRobot() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.line = calculateLine(0x00000001u, false, true, kLineTestAngles, kLineTestAngleMask);
    const MainOutput output = controller.step(input);
    CHECK(output.state == kFault && output.motion.x == 0 && output.motion.y == 0);
}

static void testMotorCommunicationLossStopsImmediately() {
    float wheels[4]{};
    MotorWheelSlew slew[4]{};
    CHECK(safeMotorUpdate({0, 1, 0}, true, true, kTestWheels, slew, wheels, 1.0f, 0, 200));
    CHECK(fabsf(wheels[0]) > 0.5f);
    CHECK(safeMotorUpdate({}, true, true, kTestWheels, slew, wheels, 0.1f, 2, 200));
    for (float value : wheels) CHECK(value == 0.0f);
    CHECK(safeMotorUpdate({0, -1, 0}, true, true, kTestWheels, slew, wheels, 0.1f, 201, 200));
    for (float value : wheels) CHECK(value == 0.0f);
    CHECK(safeMotorUpdate({0, -1, 0}, true, true, kTestWheels, slew, wheels, 0.1f, 202, 200));
    for (float value : wheels) CHECK(fabsf(value) <= 0.1001f);
    CHECK(safeMotorUpdate({0, -1, 0}, false, true, kTestWheels, slew, wheels, 0.1f, 203, 200));
    for (float value : wheels) CHECK(value == 0.0f);
}

static void testInvalidMotorCommandsRejected() {
    int16_t values[4] = {7, 7, 7, 7};
    CHECK(!motorCommandValuesForMotion({NAN, 0, 0}, true, board::kMotorWheelConfig, values));
    for (int16_t value : values) CHECK(value == 0);
    CHECK(motorCommandValuesForMotion({0, 0.5f, 0}, true, board::kMotorWheelConfig, values));
    CHECK(values[0] == 90 && values[1] == -90 && values[2] == 90 && values[3] == -90);
    CHECK(motorCommandValuesForMotion({}, false, nullptr, values));
    for (int16_t value : values) CHECK(value == 0);
}

static void testMotorStartupOutputGate() {
    const auto valid = [](bool outputEnabled, bool wheelsConfigured, bool tableConfigured,
                          uint32_t frequency, uint16_t range, const WheelConfig wheels[4],
                           const Tb67h450ChannelConfig driver[4]) {
        return validMotorHardwareConfig(outputEnabled, wheelsConfigured, tableConfigured,
                                        frequency, range, 125000000, wheels, driver,
                                        kTestPwmPinMask, kTestReservedPinMask);
    };
    CHECK(!valid(false, true, true, 20000, kMotorPwmRange, kTestWheels, kTestDrivers));
    CHECK(!valid(true, true, true, 99, kMotorPwmRange, kTestWheels, kTestDrivers));
    CHECK(!valid(true, true, true, 1000001, kMotorPwmRange, kTestWheels, kTestDrivers));
    CHECK(!valid(true, true, true, 20000, 0, kTestWheels, kTestDrivers));
    CHECK(valid(true, true, true, 100, kMotorPwmRange, kTestWheels, kTestDrivers));
    CHECK(!valid(true, true, true, 1000000, kMotorPwmRange, kTestWheels, kTestDrivers));
    CHECK(valid(true, true, true, 20000, kMotorPwmRange, kTestWheels, kTestDrivers));
    CHECK(validMotorPwmSettings(100, 255, 125000000));
    CHECK(validMotorPwmSettings(490196, 255, 125000000));
    CHECK(!validMotorPwmSettings(490197, 255, 125000000));
    CHECK(!validMotorPwmSettings(1000000, 255, 125000000));
    CHECK(!validMotorPwmSettings(99, 255, 125000000));
    CHECK(!validMotorPwmSettings(20000, 254, 125000000));
    Tb67h450ChannelConfig duplicate[4] = {
        kTestDrivers[0], kTestDrivers[1], kTestDrivers[2], kTestDrivers[3]
    };
    duplicate[3].pinIn2 = duplicate[0].pinIn1;
    CHECK(!valid( true, true, true, 20000, kMotorPwmRange, kTestWheels, duplicate));
    CHECK(!valid(true, false, true, 20000, kMotorPwmRange, kTestWheels, kTestDrivers));
    WheelConfig duplicateWheel[4] = {kTestWheels[0], kTestWheels[1], kTestWheels[2], kTestWheels[3]};
    duplicateWheel[3].positionX = duplicateWheel[0].positionX;
    duplicateWheel[3].positionY = duplicateWheel[0].positionY;
    CHECK(!valid(true, true, true, 20000, kMotorPwmRange, duplicateWheel, kTestDrivers));
    Tb67h450ChannelConfig invalidLimit[4] = {kTestDrivers[0], kTestDrivers[1], kTestDrivers[2], kTestDrivers[3]};
    invalidLimit[1].maxDuty = 256;
    CHECK(!valid(true, true, true, 20000, kMotorPwmRange, kTestWheels, invalidLimit));
    Tb67h450ChannelConfig reserved[4] = {kTestDrivers[0], kTestDrivers[1], kTestDrivers[2], kTestDrivers[3]};
    reserved[0].pinIn1 = 25;
    CHECK(!validMotorPinMap(reserved, kTestPwmPinMask, kTestReservedPinMask));
    reserved[0].pinIn1 = 2;
    CHECK(!validMotorPinMap(reserved, kTestPwmPinMask, kTestReservedPinMask));
    reserved[0].pinIn1 = 29;
    CHECK(!validMotorPinMap(reserved, kTestPwmPinMask, kTestReservedPinMask));
}

static void testMotorRouteAndPackageValidation() {
    CHECK(board::kMotorRs485TxPin == 0 && board::kMotorRs485RxPin == 1 && board::kMotorRs485DePin == 2);
    CHECK(board::kMotorGpioCount == 30 && board::kMotorRs485PinsConfigured);
    CHECK(board::kMotorPwmPinMask == 0x3FC0u && board::kMotorReservedPinMask == 0x7u);
    CHECK(board::kMotorSafetyReservedPinMask == (board::kMotorPwmPinMask | board::kMotorReservedPinMask));
    CHECK(board::kMotorHardwareConfigured && !board::kMotorWheelConfigConfigured);
    CHECK(board::kMotorWheelRadiusMm == 30.0f && board::kMotorCenterToWheelMm == 94.762f);
    CHECK(board::kMotorIn1Pins[0] == 6 && board::kMotorIn2Pins[0] == 7);
    CHECK(board::kMotorIn1Pins[1] == 12 && board::kMotorIn2Pins[1] == 13);
    CHECK(board::kMotorIn1Pins[2] == 11 && board::kMotorIn2Pins[2] == 10);
    CHECK(board::kMotorIn1Pins[3] == 8 && board::kMotorIn2Pins[3] == 9);
    CHECK(board::kMotorTestIn1CandidatePin == 12 && board::kMotorTestIn2CandidatePin == 13);
    CHECK(board::kMotorPwmFrequencyHz == kTb67h450PwmFrequencyHz);
    CHECK(validRs485PinConfig(board::kMotorRs485PinsConfigured, board::kMotorGpioCount,
                               board::kMotorRs485TxPin, board::kMotorRs485RxPin, board::kMotorRs485DePin));
    CHECK(validRs485PinConfig(true, 48, 45, 46, 47));
    CHECK(!validRs485PinConfig(false, 48, 45, 46, 47));
    CHECK(!validRs485PinConfig(true, 48, 45, 45, 47));
    CHECK(!validRs485PinConfig(true, 48, 45, 46, 48));

    const uint64_t mask = (uint64_t{1} << 40) | (uint64_t{1} << 41) |
                          (uint64_t{1} << 42) | (uint64_t{1} << 43) |
                          (uint64_t{1} << 44) | (uint64_t{1} << 45) |
                          (uint64_t{1} << 46) | (uint64_t{1} << 47);
    const Tb67h450ChannelConfig wideDrivers[4] = {
        {40, 41, kTb67h450MaxDuty}, {42, 43, kTb67h450MaxDuty},
        {44, 45, kTb67h450MaxDuty}, {46, 47, kTb67h450MaxDuty}
    };
    CHECK(validMotorPinMap(wideDrivers, mask, 0, 48));
    CHECK(!validMotorPinMap(wideDrivers, mask, 0, 30));
    CHECK(!validMotorPinMap(wideDrivers, mask, uint64_t{1} << 47, 48));
    CHECK(validMotorHardwareConfig(true, true, true, 20000, kMotorPwmRange, 125000000,
                                   kTestWheels, wideDrivers, mask, 0, 48));
    CHECK(!validMotorHardwareConfig(true, true, true, 20000, kMotorPwmRange, 125000000,
                                    kTestWheels, wideDrivers, mask, 0, 0));
    Tb67h450ChannelConfig splitBank[4] = {
        {31, 32, kTb67h450MaxDuty}, {33, 34, kTb67h450MaxDuty},
        {35, 36, kTb67h450MaxDuty}, {37, 38, kTb67h450MaxDuty}
    };
    const uint64_t splitBankMask = (uint64_t{1} << 31) | (uint64_t{1} << 32) |
                                   (uint64_t{1} << 33) | (uint64_t{1} << 34) |
                                   (uint64_t{1} << 35) | (uint64_t{1} << 36) |
                                   (uint64_t{1} << 37) | (uint64_t{1} << 38);
    CHECK(!validMotorPinMap(splitBank, splitBankMask, 0, 48));
}

static void testSafetyInputPinValidation() {
    const int8_t pins[3] = {22, 23, 24};
    const int8_t active[3] = {0, 0, 0};
    CHECK(validSafetyInputConfig(pins, active, true, kTestSafetyReservedPinMask));
    CHECK(!validSafetyInputConfig(pins, active, false, kTestSafetyReservedPinMask));
    const int8_t missing[3] = {-1, 23, 24};
    CHECK(!validSafetyInputConfig(missing, active, true, kTestSafetyReservedPinMask));
    const int8_t duplicate[3] = {22, 22, 24};
    CHECK(!validSafetyInputConfig(duplicate, active, true, kTestSafetyReservedPinMask));
    const int8_t reserved[3] = {22, 25, 24};
    CHECK(!validSafetyInputConfig(reserved, active, true, kTestSafetyReservedPinMask));
    const int8_t unknownPolarity[3] = {0, -1, 0};
    CHECK(!validSafetyInputConfig(pins, unknownPolarity, true, kTestSafetyReservedPinMask));
}

static void testMotorCommandTimeoutLatchesBeforeRecovery() {
    CHECK(!motorCommandExpired(299, 0, 300));
    CHECK(motorCommandExpired(300, 0, 300));
    CHECK(!motorCommandExpired(0x0000000Fu, 0xFFFFFF10u, 0x100u));
    CHECK(motorCommandExpired(0x00000010u, 0xFFFFFF10u, 0x100u));

    MotorSafetyGate gate;
    MotorSafetyInput input{};
    input.inputsConfigured = true;
    input.hardwareReady = true;
    input.emergencyStop = false;
    input.commandFresh = true;
    input.commandEnabled = true;
    CHECK(!gate.update(input)); // ARM must first be observed released.
    input.armPressed = true;
    CHECK(gate.update(input));
    CHECK(gate.state() == kMotorArmed);
    CHECK(expireMotorCommand(gate, true, 300, 0, 300));
    CHECK(gate.state() == kMotorFault);
    CHECK(!gate.update(input)); // A command received in the same loop cannot clear the latched fault.
}

static void testManualArmRequiresFreshPress() {
    ManualArmLatch arm;
    CHECK(!arm.update(false, false, false));
    CHECK(!arm.update(true, true, false)); // A held button at boot cannot arm.
    CHECK(!arm.update(true, false, false));
    CHECK(arm.update(true, true, false));
    CHECK(arm.update(true, false, false));
    CHECK(!arm.update(true, true, true)); // Fault drops the latch.
    CHECK(!arm.update(true, true, false));
    CHECK(!arm.update(true, false, false));
    CHECK(arm.update(true, true, false));
}

static void testManualClearRequiresFreshPress() {
    ManualActionLatch clear;
    CHECK(!clear.update(true, true, false)); // A clear button held at boot is ignored.
    CHECK(!clear.update(true, false, false));
    CHECK(clear.update(true, true, false));
    CHECK(clear.update(true, false, false)); // The request stays available while a fault is being cleared.
    clear.reset();
    CHECK(!clear.update(true, true, false));
    CHECK(!clear.update(true, true, true)); // An asserted E-stop cancels the clear request.
    CHECK(!clear.update(true, true, false));
    CHECK(!clear.update(true, false, false));
    CHECK(clear.update(true, true, false));
}

static void testMotorSafetyGateLatchesAndRequiresRearm() {
    MotorSafetyGate gate;
    MotorSafetyInput input{};
    input.inputsConfigured = true;
    input.hardwareReady = true;
    input.emergencyStop = false;
    input.commandFresh = true;
    input.commandEnabled = true;
    input.armPressed = true;
    CHECK(!gate.update(input)); // A button held at power-up is not a new ARM action.
    input.armPressed = false;
    CHECK(!gate.update(input));
    input.armPressed = true;
    CHECK(gate.update(input));
    CHECK(gate.state() == kMotorArmed);
    input.emergencyStop = true;
    CHECK(!gate.update(input));
    CHECK(gate.state() == kMotorFault);
    input.emergencyStop = false;
    input.armPressed = false;
    input.commandEnabled = false;
    input.clearFaultPressed = true;
    CHECK(!gate.update(input));
    CHECK(gate.state() == kMotorDisarmed);
    input.clearFaultPressed = false;
    input.commandEnabled = true;
    CHECK(!gate.update(input));
    input.armPressed = true;
    CHECK(gate.update(input));
    input.commandFresh = false; // 100 ms timeout after arming latches a fault.
    CHECK(!gate.update(input));
    CHECK(gate.state() == kMotorFault);
    input.commandFresh = true;
    CHECK(!gate.update(input)); // A fresh MAIN command cannot automatically restart it.

    MotorSafetyGate configGate;
    MotorSafetyInput invalidConfig{};
    invalidConfig.inputsConfigured = true;
    invalidConfig.commandFresh = true;
    invalidConfig.commandEnabled = true;
    CHECK(!configGate.update(invalidConfig));
    CHECK(configGate.state() == kMotorFault); // MAIN enable with an invalid motor configuration faults closed.
}

static void testHeadingFaultAndExplicitRecovery() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.headingRequired = true;
    input.headingValid = false;
    CHECK(controller.step(input).state == kFault);
    input.headingValid = true;
    input.headingDeg = 0;
    input.clearFault = true;
    input.nowMs = 100;
    CHECK(controller.step(input).state == kFault);
    input.nowMs = 599;
    CHECK(controller.step(input).state == kFault);
    input.nowMs = 600;
    CHECK(controller.step(input).state == kDisarmed);
    input.nowMs = 601;
    CHECK(controller.step(input).state == kDisarmed); // A held ARM cannot bypass the required re-arm cycle.
    input.armed = false;
    input.nowMs = 602;
    CHECK(controller.step(input).state == kDisarmed);
    input.armed = true;
    input.clearFault = false;
    input.nowMs = 603;
    CHECK(controller.step(input).state == kApproachBall);

    MainController jumpFaultController;
    input = safeArmedInput();
    input.headingRequired = true;
    input.headingValid = true;
    input.headingDeg = 0;
    CHECK(jumpFaultController.step(input).state == kApproachBall);
    input.nowMs = 10;
    input.headingDeg = 100;
    CHECK(jumpFaultController.step(input).state == kFault);
    input.clearFault = true;
    input.nowMs = 20;
    input.headingDeg = 100.5f;
    CHECK(jumpFaultController.step(input).state == kFault);
    input.nowMs = 519;
    input.headingDeg = 100.6f;
    CHECK(jumpFaultController.step(input).state == kFault);
    input.nowMs = 520;
    CHECK(jumpFaultController.step(input).state == kDisarmed);
}

static void testEmergencyStopLatchesFault() {
    MainController controller;
    MainInput input = safeArmedInput();
    input.emergencyStop = true;
    CHECK(controller.step(input).state == kFault);
    input.emergencyStop = false;
    input.clearFault = true;
    input.lineFresh = true;
    input.line = lineFromActive(0);
    input.nowMs = 1;
    CHECK(controller.step(input).state == kFault);
}

static void testUnspecifiedSafetyInputsFailClosed() {
    MotorSafetyInput motorInput{};
    CHECK(motorInput.emergencyStop);
    CHECK(!motorInput.clearFaultPressed);
    motorInput.inputsConfigured = true;
    motorInput.hardwareReady = true;
    motorInput.commandFresh = true;
    motorInput.commandEnabled = true;
    motorInput.armPressed = true;
    MotorSafetyGate motorGate;
    CHECK(!motorGate.update(motorInput));
    CHECK(motorGate.state() == kMotorFault);

    MainInput mainInput{};
    CHECK(mainInput.emergencyStop);
    CHECK(!mainInput.clearFault);
    mainInput.armed = true;
    mainInput.lineFresh = true;
    mainInput.line = lineFromActive(0);
    mainInput.irFresh = true;
    mainInput.ballFound = true;
    mainInput.headingValid = true;
    MainController mainController;
    const MainOutput output = mainController.step(mainInput);
    CHECK(output.state == kFault);
    CHECK(output.motion.x == 0 && output.motion.y == 0 && output.motion.turn == 0);
}

using TestFn = void (*)();
struct TestCase { const char* name; TestFn run; };

int main() {
    const TestCase cases[] = {
        {"CRC vector and packet round trip", testCrcKnownVectorAndFrameRoundTrip},
        {"CRC corruption", testBadCrcRejected},
        {"Dropped packet sequence gap", testDroppedPacketSequenceGap},
        {"Partial frame", testPartialFrameWaitsForRemainder},
        {"Partial frame timeout", testPartialFrameTimeout},
        {"MOTOR frame round trip, range, checksum and timeout", testMotorCommandFrameRoundTripAndValidation},
        {"MOTOR ignores valid shared-bus IR and LINE frames", testMotorParserIgnoresSharedBusTraffic},
        {"MOTOR direct console command parsing", testMotorConsoleCommandParsing},
        {"AUTO command modes and fail-closed gates", testMainConsoleAutoModesAndFailClosedGates},
        {"USB input cancels boot AUTO IR auto-arm", testUsbInputCancelsBootAutoArm},
        {"IR test mode continuous tracking and coarse steering", testIrTestContinuousTrackingAndCoarseSteering},
        {"AUTO PWM caps", testAutoMotorPwmCaps},
        {"RS485 half-duplex order and failure recovery", testRs485HalfDuplexOrderingAndFailure},
        {"RS485 receive timeout and collision guard", testRs485ReceiveTimeoutAndCollisionGuard},
        {"Shared bus IR request demultiplexing", testSharedBusIrRequestDemultiplexing},
        {"IR request and legacy six-byte response", testIrRequestAndLegacySixByteResponse},
        {"Sequence wrap and duplicate", testSequenceWrapAndRejectDuplicates},
        {"Invalid message type", testUnknownMessageTypeRejected},
        {"Invalid payload length", testOversizedPayloadLengthRejected},
        {"IR 0 and 359 degree boundaries", testIrZeroAnd359Boundaries},
        {"IR no-ball and invalid angle", testIrNoBallAndInvalidAngle},
        {"IR direction coherence metric", testIrDirectionCoherenceMetric},
        {"IR prediction wrap, time wrap and speed limit", testIrPredictionWrapsBearingBoundsSpeedAndRecoversFromClockWrap},
        {"LINE debounce rejects short transitions", testLineDebounceRejectsShortTransitions},
        {"Line single sensor", testLineSingleSensor},
        {"LINE poll response CRC and sequence", testLinePollResponseCrcAndSequence},
        {"Line adjacent sensors", testLineMultipleAdjacentSensors},
        {"Line opposing ambiguity", testLineOpposingSensorsAreAmbiguous},
        {"Line angle wrap and consistency", testLineAngleWrapAndPacketConsistency},
        {"Line unknown polarity and angles", testLineUnknownPolarityAndAngleConfiguration},
        {"LINE failed-sensor mask and 18 live inputs", testLineFaultMaskExcludesFailedSensors},
        {"LINE candidate PCB angles and fail-closed gates", testLineCandidateAnglesUseSignedCentidegreesAndStayGated},
        {"Line without direction stops", testLineDetectedWithoutAnglesStopsSafely},
        {"Line priority, hold and ambiguous stop", testLineAvoidancePriorityHoldAndAmbiguity},
        {"Omni forward and backward", testOmniForwardAndBackward},
        {"Omni left and right", testOmniLeftAndRight},
        {"Omni clockwise and counter-clockwise", testOmniClockwiseAndCounterclockwise},
        {"Omni blend and normalization", testOmniBlendNormalization},
        {"Motor acceleration limit", testMotorSlewLimit},
        {"TB67H450 truth table and PWM limits", testTb67h450TruthTableAndDutyLimits},
        {"MOTOR STOP command priority", testMotorStopPriority},
        {"Heading boundaries and jump", testHeadingBoundaryAndJumpRejection},
        {"BNO init, disconnect, validation and recovery", testBnoInitFailureDisconnectInvalidDataAndRecovery},
        {"Kicker and dribbler gates", testKickerAndDribblerOutputGates},
        {"MAIN kicker request policy gates", testMainKickerRequestPolicyGates},
        {"Disarm, approach and search", testSearchApproachAndDisarmedStates},
        {"Sensor communication loss", testLineSensorCommunicationLossLatchesFault},
        {"IR sensor communication loss", testIrSensorCommunicationLossLatchesFault},
        {"Unknown line polarity fails safe", testUnknownLinePolarityFaultsArmedRobot},
        {"Motor communication loss", testMotorCommunicationLossStopsImmediately},
        {"Invalid motor commands", testInvalidMotorCommandsRejected},
        {"Motor startup output gate", testMotorStartupOutputGate},
        {"MOTOR timeout latch and recovery boundary", testMotorCommandTimeoutLatchesBeforeRecovery},
        {"MOTOR route and package validation", testMotorRouteAndPackageValidation},
        {"Safety input pin validation", testSafetyInputPinValidation},
        {"Manual ARM requires fresh press", testManualArmRequiresFreshPress},
        {"Manual clear requires fresh press", testManualClearRequiresFreshPress},
        {"MOTOR safety fault latch and re-arm", testMotorSafetyGateLatchesAndRequiresRearm},
        {"Heading fault recovery and re-arm", testHeadingFaultAndExplicitRecovery},
        {"Emergency stop latch", testEmergencyStopLatchesFault},
        {"Unspecified safety inputs fail closed", testUnspecifiedSafetyInputsFailClosed}
    };
    for (const TestCase& test : cases) {
        currentCase = test.name;
        const unsigned before = failures;
        test.run();
        ++testCases;
        printf("%s %s\n", failures == before ? "PASS" : "FAIL", test.name);
    }
    printf("UNIT SUMMARY: %u cases, %u checks, %u failed\n",
           testCases, assertions, failures);
    return failures ? 1 : 0;
}
