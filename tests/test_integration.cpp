#include "../common/src/RobotControl.ino"
#include "../common/src/BoardConfig.ino"
#include "../common/src/RobotDevices.ino"
#include "../common/src/Rs485.ino"
#include <math.h>
#include <stdio.h>

using namespace soccer;

static constexpr uint32_t kMotorCommandTimeoutMs = 300;
static constexpr uint32_t kMotorRampIntervalMs = 10;

static unsigned testCases = 0;
static unsigned checks = 0;
static unsigned failures = 0;
static const char* currentCase = "";

#define CHECK(expr) do { \
    ++checks; \
    if (!(expr)) { \
        ++failures; \
        printf("FAIL [%s] line %d: %s\n", currentCase, __LINE__, #expr); \
    } \
} while (0)

// Integration fixtures exercise the current PCB-angle candidate, while BoardConfig keeps it disabled.
static const int16_t* const kLineTestAngles = board::kLineSensorAngleCdeg;
static const uint32_t kLineTestAngleMask = board::kLineSensorAngleValidMask;
static LineObservation testLine(uint32_t activeBits) {
    return calculateLine(activeBits, true, true, kLineTestAngles, kLineTestAngleMask,
                         board::kLineSensorEnabledMask);
}

class MockRobot {
public:
    bool injectIr(uint16_t angle, uint16_t strength, uint32_t nowMs) {
        const uint8_t wire[6] = {0xAA, static_cast<uint8_t>(angle >> 8), static_cast<uint8_t>(angle),
                                 static_cast<uint8_t>(strength >> 8), static_cast<uint8_t>(strength), 0x55};
        bool ready = false;
        uint16_t parsedAngle = 0, parsedStrength = 0;
        for (uint8_t i = 0; i < 6; ++i)
            ready = irParser_.feed(wire[i], nowMs + i, parsedAngle, parsedStrength) || ready;
        if (!ready) return false;
        irSeen_ = true;
        lastIrMs_ = nowMs;
        irAngle_ = parsedAngle;
        irStrength_ = parsedStrength;
        return true;
    }

    bool injectLine(uint32_t bits, uint32_t nowMs) {
        return injectLineObservation(testLine(bits), nowMs);
    }

    bool injectUnknownPolarity(uint32_t rawLevels, uint32_t nowMs) {
        return injectLineObservation(calculateLine(rawLevels, false, true, kLineTestAngles,
                                                   kLineTestAngleMask, board::kLineSensorEnabledMask), nowMs);
    }

    void setHeadingTarget(float target) { input_.headingTargetDeg = target; }

    MainOutput tick(uint32_t nowMs, bool sendMotor = true) {
        input_.nowMs = nowMs;
        input_.lineFresh = lineSeen_ && static_cast<uint32_t>(nowMs - lastLineMs_) <= 100;
        input_.line = line_;
        input_.irFresh = irSeen_ && static_cast<uint32_t>(nowMs - lastIrMs_) <= 60;
        input_.ballFound = input_.irFresh && irStrength_ != 0;
        input_.ballAngleDeg = irAngle_;
        input_.relativeSignal = irStrength_;
        const MainOutput output = controller_.step(input_);
        if (sendMotor) transmitMotor(output.motion, input_.armed && output.state != kFault, nowMs);
        return output;
    }

    void configure(bool armed, bool headingRequired = true) {
        input_.armed = armed;
        input_.emergencyStop = false;
        input_.clearFault = false;
        input_.headingRequired = headingRequired;
        input_.headingValid = true;
        input_.headingDeg = 0.0f;
        input_.headingTargetDeg = 0.0f;
    }

    void setEmergencyStop(bool active) { input_.emergencyStop = active; }
    void setClearFault(bool active) { input_.clearFault = active; }
    void setHeading(bool required, bool valid, float degrees) {
        input_.headingRequired = required;
        input_.headingValid = valid;
        input_.headingDeg = degrees;
    }

    bool motorTick(uint32_t nowMs, bool outputGate = true) {
        const bool fresh = motorSeen_ && !motorCommandExpired(nowMs, lastMotorMs_, kMotorCommandTimeoutMs);
        if (!fresh || !outputGate || !motorEnabled_) {
            for (uint8_t i = 0; i < 4; ++i) {
                wheelOutputs_[i] = 0.0f;
                motorCurrent_[i] = 0;
                reversing_[i] = false;
                reverseUntil_[i] = 0;
            }
            return true;
        }
        for (uint8_t i = 0; i < 4; ++i) {
            const int16_t requested = motorValues_[i];
            if (requested == kMotorBrakeCommand) {
                wheelOutputs_[i] = 0.0f;
                motorCurrent_[i] = kMotorBrakeCommand;
                reversing_[i] = false;
                lastRampMs_[i] = nowMs;
                continue;
            }
            if (requested == 0) {
                wheelOutputs_[i] = 0.0f;
                motorCurrent_[i] = 0;
                reversing_[i] = false;
                lastRampMs_[i] = nowMs;
                continue;
            }
            int16_t current = motorCurrent_[i];
            if (current == kMotorBrakeCommand) {
                current = 0;
                motorCurrent_[i] = 0;
                wheelOutputs_[i] = 0.0f;
            }
            if (reversing_[i]) {
                if (static_cast<int32_t>(nowMs - reverseUntil_[i]) < 0) continue;
                reversing_[i] = false;
                current = 0;
            }
            if (current != 0 && ((current > 0) != (requested > 0))) {
                motorCurrent_[i] = 0;
                wheelOutputs_[i] = 0.0f;
                reversing_[i] = true;
                reverseUntil_[i] = nowMs + kTb67h450ReversalDwellMs;
                continue;
            }
            const int16_t magnitude = requested < 0 ? -requested : requested;
            if (magnitude <= 200) {
                motorCurrent_[i] = requested;
            } else if ((current < 0 ? -current : current) < 200) {
                current = requested > 0 ? 200 : -200;
                motorCurrent_[i] = current;
                lastRampMs_[i] = nowMs;
            } else if (static_cast<uint32_t>(nowMs - lastRampMs_[i]) >= kMotorRampIntervalMs) {
                lastRampMs_[i] = nowMs;
                if (current < requested) {
                    const int16_t next = static_cast<int16_t>(current + 5);
                    current = next > requested ? requested : next;
                } else if (current > requested) {
                    const int16_t next = static_cast<int16_t>(current - 5);
                    current = next < requested ? requested : next;
                }
                motorCurrent_[i] = current;
            }
            wheelOutputs_[i] = static_cast<float>(motorCurrent_[i]) / 255.0f;
        }
        return true;
    }

    const Motion& motorMotion() const { return motorMotion_; }
    const float* wheels() const { return wheelOutputs_; }
    bool motorEnabled() const { return motorEnabled_; }
    bool motorSeen() const { return motorSeen_; }

private:
    bool injectLineObservation(const LineObservation& line, uint32_t nowMs) {
        const Packet packet = makeLinePacket(line, lineSequenceTx_++);
        uint8_t wire[kMaxFrame]{};
        const uint8_t length = encodePacket(packet, wire, sizeof(wire));
        Packet decoded{};
        bool ready = false;
        for (uint8_t i = 0; i < length; ++i)
            ready = lineParser_.feed(wire[i], nowMs + i, decoded) || ready;
        if (!ready || !lineSequenceRx_.accept(decoded.sequence)) return false;
        if (!decodeLinePacket(decoded, line_, kLineTestAngles, kLineTestAngleMask,
                              board::kLineSensorEnabledMask)) return false;
        lineSeen_ = true;
        lastLineMs_ = nowMs;
        return true;
    }
    void transmitMotor(const Motion& motion, bool enabled, uint32_t nowMs) {
        int16_t values[kMotorCommandWheelCount]{};
        if (!motorCommandValuesForMotion(motion, enabled, board::kMotorWheelConfig, values)) return;
        uint8_t wire[kMotorCommandFrameSize]{};
        const uint8_t length = encodeMotorCommandFrame(values, wire, sizeof(wire));
        if (!length) return;
        MotorFrameStatus status = kMotorFramePending;
        for (uint8_t i = 0; i < length; ++i)
            status = motorParser_.feed(wire[i], nowMs, motorValues_);
        if (status != kMotorFrameReady) return;
        motorMotion_ = motion;
        motorEnabled_ = false;
        for (int16_t value : motorValues_) motorEnabled_ |= value != 0;
        motorSeen_ = true;
        lastMotorMs_ = nowMs;
    }

    MainController controller_{};
    MainInput input_{};
    LegacyIrParser irParser_{};
    PacketParser lineParser_{};
    MotorCommandFrameParser motorParser_{};
    SequenceTracker lineSequenceRx_{};
    LineObservation line_{};
    bool irSeen_ = false;
    uint32_t lastIrMs_ = 0;
    uint16_t irAngle_ = 0;
    uint16_t irStrength_ = 0;
    bool lineSeen_ = false;
    uint32_t lastLineMs_ = 0;
    bool motorSeen_ = false;
    uint32_t lastMotorMs_ = 0;
    uint8_t lineSequenceTx_ = 0;
    Motion motorMotion_{};
    bool motorEnabled_ = false;
    int16_t motorValues_[kMotorCommandWheelCount]{};
    uint32_t lastRampMs_[4]{};
    int16_t motorCurrent_[4]{};
    bool reversing_[4]{};
    uint32_t reverseUntil_[4]{};
    float wheelOutputs_[4]{};
};

static void testIrMainMotorFrontAndSideTracking() {
    MockRobot front;
    front.configure(true);
    CHECK(front.injectLine(0, 0));
    CHECK(front.injectIr(0, 500, 0));
    MainOutput output = front.tick(0);
    CHECK(output.state == kApproachBall && output.motion.y > 0);
    CHECK(front.motorSeen() && front.motorEnabled());
    CHECK(front.motorTick(10));
    bool moving = false;
    for (uint8_t i = 0; i < 4; ++i) moving |= fabsf(front.wheels()[i]) > 0.01f;
    CHECK(moving);

    MockRobot right;
    right.configure(true);
    CHECK(right.injectLine(0, 0) && right.injectIr(90, 400, 0));
    output = right.tick(0);
    CHECK(output.state == kApproachBall && output.motion.x > 0);

    MockRobot left;
    left.configure(true);
    CHECK(left.injectLine(0, 0) && left.injectIr(270, 400, 0));
    output = left.tick(0);
    CHECK(output.state == kApproachBall && output.motion.x < 0);
}

static void testBallLostWithHeadingMockSearchesSafely() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(0, 0) && robot.injectIr(0, 0, 0));
    MainOutput output = robot.tick(0);
    CHECK(output.state == kSearchBall);
    CHECK(fabsf(output.motion.x) == 0.15f);
    CHECK(output.motion.turn == 0); // Heading controller holds the mock attack heading.
    CHECK(robot.motorTick(10));
}

static void testLineOverridesBallAndHoldsBeforeRecovery() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(1u, 0) && robot.injectIr(0, 500, 0));
    MainOutput output = robot.tick(0);
    CHECK(output.state == kAvoidLine && output.motion.y < 0);
    CHECK(robot.motorTick(0));

    CHECK(robot.injectLine(0, 10) && robot.injectIr(0, 500, 10));
    output = robot.tick(10);
    CHECK(output.state == kAvoidLine && output.motion.x == 0 && output.motion.y == 0);
    CHECK(robot.injectLine(0, 209) && robot.injectIr(0, 500, 209));
    CHECK(robot.tick(209).state == kAvoidLine);
    CHECK(robot.injectLine(0, 210) && robot.injectIr(0, 500, 210));
    output = robot.tick(210);
    CHECK(output.state == kApproachBall && output.motion.y > 0);
}

static void testFailedLineSensorsCannotOverrideBallTracking() {
    MockRobot robot;
    robot.configure(true);
    const uint32_t failed = (1u << 1) | (1u << 5) | (1u << 8) |
                            (1u << 9) | (1u << 11) | (1u << 21);
    CHECK(robot.injectLine(failed, 0) && robot.injectIr(90, 400, 0));
    const MainOutput output = robot.tick(0);
    CHECK(output.state == kApproachBall && output.motion.x > 0);
}

static void testIrCommunicationLossProducesIdleStop() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(0, 0) && robot.injectIr(0, 500, 0));
    CHECK(robot.tick(0).state == kApproachBall);
    MainOutput output = robot.tick(61); // IR response freshness expires after 60 ms.
    CHECK(output.state == kFault);
    CHECK(output.motion.x == 0 && output.motion.y == 0 && output.motion.turn == 0);
    CHECK(robot.motorTick(61));
    for (uint8_t i = 0; i < 4; ++i) CHECK(robot.wheels()[i] == 0.0f);
}

static void testBnoHeadingControlsTurnAlongsideBallApproach() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(0, 0) && robot.injectIr(0, 500, 0));
    robot.setHeading(true, true, 10.0f); // Mock BNO055 is 10 degrees clockwise of the configured 0-degree target.
    const MainOutput output = robot.tick(0);
    CHECK(output.state == kApproachBall && output.motion.y > 0 && output.motion.turn < 0);
    CHECK(robot.motorSeen() && robot.motorEnabled());
    CHECK(fabsf(robot.motorMotion().turn - output.motion.turn) < 0.001f);
}

static void testUnknownLinePolarityStopsIntegrationPath() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectUnknownPolarity(1u, 0) && robot.injectIr(0, 500, 0));
    const MainOutput output = robot.tick(0);
    CHECK(output.state == kFault && output.motion.y == 0);
    CHECK(robot.motorSeen() && !robot.motorEnabled() && robot.motorTick(0));
}

static void testLineCommunicationLossFaultsAndDisablesMotor() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(0, 0) && robot.injectIr(0, 500, 0));
    CHECK(robot.tick(0).state == kApproachBall);
    MainOutput output = robot.tick(101); // LINE frame freshness expires after 100 ms.
    CHECK(output.state == kFault && output.motion.y == 0);
    CHECK(!robot.motorEnabled());
    CHECK(robot.motorTick(101));
    for (uint8_t i = 0; i < 4; ++i) CHECK(robot.wheels()[i] == 0.0f);
}

static void testMainToMotorLossUsesLocalWatchdog() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(0, 0) && robot.injectIr(0, 500, 0));
    CHECK(robot.tick(0).state == kApproachBall);
    CHECK(robot.motorTick(0));
    bool moving = false;
    for (uint8_t i = 0; i < 4; ++i) moving |= fabsf(robot.wheels()[i]) > 0.01f;
    CHECK(moving);
    CHECK(robot.motorTick(299)); // The command is still fresh just before the local timeout.
    CHECK(robot.motorTick(300)); // No new MAIN packet: MOTOR clears output at 300 ms.
    for (uint8_t i = 0; i < 4; ++i) CHECK(robot.wheels()[i] == 0.0f);
}

static void testBnoFaultMultipleFaultsAndEmergencyStop() {
    MockRobot bno;
    bno.configure(true);
    CHECK(bno.injectLine(0, 0) && bno.injectIr(0, 500, 0));
    CHECK(bno.tick(0).state == kApproachBall);
    bno.setHeading(true, false, NAN);
    MainOutput output = bno.tick(10);
    CHECK(output.state == kFault && output.motion.y == 0);
    CHECK(!bno.motorEnabled() && bno.motorTick(10));
    for (uint8_t i = 0; i < 4; ++i) CHECK(bno.wheels()[i] == 0.0f);

    MockRobot multiple;
    multiple.configure(true);
    CHECK(multiple.injectLine(0, 0) && multiple.injectIr(180, 500, 0));
    CHECK(multiple.tick(0).state == kApproachBall);
    multiple.setEmergencyStop(true);
    output = multiple.tick(10);
    CHECK(output.state == kFault && !multiple.motorEnabled());
    multiple.setEmergencyStop(false);
    output = multiple.tick(101); // Both IR and LINE are now stale as well.
    CHECK(output.state == kFault && output.motion.x == 0 && output.motion.y == 0);
    CHECK(multiple.motorTick(101));
    for (uint8_t i = 0; i < 4; ++i) CHECK(multiple.wheels()[i] == 0.0f);
}

static void testExplicitEmergencyStopAndRearmPath() {
    MockRobot robot;
    robot.configure(true);
    CHECK(robot.injectLine(0, 0) && robot.injectIr(0, 500, 0));
    CHECK(robot.tick(0).state == kApproachBall);
    robot.setEmergencyStop(true);
    CHECK(robot.tick(1).state == kFault);
    robot.setEmergencyStop(false);
    robot.setClearFault(true);
    CHECK(robot.injectLine(0, 2));
    CHECK(robot.tick(2).state == kFault);
    CHECK(robot.injectLine(0, 501));
    CHECK(robot.injectIr(0, 0, 501));
    CHECK(robot.tick(501).state == kFault);
    CHECK(robot.injectLine(0, 502));
    CHECK(robot.injectIr(0, 0, 502));
    CHECK(robot.tick(502).state == kDisarmed);
    CHECK(robot.tick(503).state == kDisarmed); // Held arm cannot restart automatically.
}

static void testAccessoryCommandsRemainGatedInMockIntegration() {
    KickerController kicker;
    DribblerController dribbler;
    KickerInput input{};
    input.request = true;
    input.commandFresh = true;
    input.armed = true;
    input.emergencyStop = false;
    input.ballFresh = true;
    input.ballDetected = true;
    input.headingValid = true;
    input.headingAligned = true;
    input.lineClear = true;
    input.hardwareConfigured = board::kKickerHardwareConfigured;
    KickerOutput output = kicker.update(input, 0);
    CHECK(!output.mockPulseActive && output.state == kKickerDisarmed);
    CHECK(dribbler.update(0.8f, true, true, false, false, false) == 0.0f);
    input.hardwareConfigured = true; // The MCU build still has no physical adapter.
    output = kicker.update(input, 1);
    CHECK(!output.mockPulseActive && output.state == kKickerReady); // Held request is not queued.
    input.emergencyStop = true;
    output = kicker.update(input, 10);
    CHECK(!output.mockPulseActive && output.state == kKickerFault);
    CHECK(dribbler.update(0.8f, true, true, true, false, true) == 0.0f);
}

static void testSharedBusArbitrationPreventsOverlappingReplies() {
    Rs485MasterArbiter bus;
    CHECK(bus.acquire(kIrRs485Peer));
    CHECK(!bus.idle()); // MAIN waits for IR before it can poll LINE or send MOTOR.
    CHECK(!bus.acquire(kLineRs485Peer));
    CHECK(!bus.complete(kLineRs485Peer));
    CHECK(bus.activePeer() == kIrRs485Peer);
    CHECK(bus.complete(kIrRs485Peer));
    CHECK(bus.idle());
    CHECK(bus.acquire(kLineRs485Peer));
    CHECK(bus.complete(kLineRs485Peer));
    CHECK(bus.idle() && bus.collisionsPrevented() == 1);
}

static void testThirtyMinuteMotorSimulation() {
    const WheelConfig wheels[4] = {
        {1, 1, 135, 1}, {-1, 1, 225, 1}, {-1, -1, 315, 1}, {1, -1, 45, 1}
    };
    MotorWheelSlew slew[4]{};
    float outputs[4]{};
    bool valid = true;
    for (uint32_t nowMs = 0; nowMs <= 30u * 60u * 1000u; nowMs += 5) {
        Motion command{};
        switch ((nowMs / 1000) % 4) {
            case 0: command.y = 0.5f; break;
            case 1: command.y = -0.5f; break;
            case 2: command.x = 0.5f; break;
            default: command.x = -0.5f; break;
        }
        valid &= safeMotorUpdate(command, true, true, wheels, slew, outputs, 0.08f,
                                 nowMs, kTb67h450ReversalDwellMs);
        for (uint8_t i = 0; i < 4; ++i) {
            Tb67h450Output pins{};
            valid &= isfinite(outputs[i]) && fabsf(outputs[i]) <= 1.0f &&
                     tb67h450OutputForWheel(outputs[i], kTb67h450MaxDuty, pins);
        }
    }
    CHECK(valid);
    CHECK(fabsf(outputs[0]) <= 0.5f && fabsf(outputs[1]) <= 0.5f &&
          fabsf(outputs[2]) <= 0.5f && fabsf(outputs[3]) <= 0.5f);
}

using TestFn = void (*)();
struct TestCase { const char* name; TestFn run; };

int main() {
    const TestCase cases[] = {
        {"IR packet to MAIN to MOTOR: front and sides", testIrMainMotorFrontAndSideTracking},
        {"Ball loss enters heading-held search", testBallLostWithHeadingMockSearchesSafely},
        {"LINE priority, hold and safe recovery", testLineOverridesBallAndHoldsBeforeRecovery},
        {"Failed LINE sensors do not create a false boundary", testFailedLineSensorsCannotOverrideBallTracking},
        {"IR loss faults and stops tracking", testIrCommunicationLossProducesIdleStop},
        {"LINE loss faults and disables MOTOR", testLineCommunicationLossFaultsAndDisablesMotor},
        {"MAIN command loss uses MOTOR watchdog", testMainToMotorLossUsesLocalWatchdog},
        {"BNO heading and ball motion reach MOTOR", testBnoHeadingControlsTurnAlongsideBallApproach},
        {"Unknown LINE polarity stops integration", testUnknownLinePolarityStopsIntegrationPath},
        {"BNO invalidity, multiple faults and E-stop", testBnoFaultMultipleFaultsAndEmergencyStop},
        {"E-stop clear requires explicit re-arm", testExplicitEmergencyStopAndRearmPath},
        {"Kicker/dribbler remain gated", testAccessoryCommandsRemainGatedInMockIntegration},
        {"Shared bus arbitration prevents overlapping replies", testSharedBusArbitrationPreventsOverlappingReplies},
        {"Thirty-minute four-wheel drive simulation", testThirtyMinuteMotorSimulation}
    };
    for (const TestCase& test : cases) {
        currentCase = test.name;
        const unsigned before = failures;
        test.run();
        ++testCases;
        printf("%s %s\n", failures == before ? "PASS" : "FAIL", test.name);
    }
    printf("INTEGRATION SUMMARY: %u scenarios, %u checks, %u failed\n",
           testCases, checks, failures);
    return failures ? 1 : 0;
}


