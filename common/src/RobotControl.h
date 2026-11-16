#pragma once

#include "RobotProtocol.h"
#include <math.h>

namespace soccer {

constexpr int16_t kNoLineAngle = static_cast<int16_t>(0x8000);
constexpr uint8_t kLineFlagDetected = 0x01;
constexpr uint8_t kLineFlagAmbiguous = 0x02;
constexpr uint8_t kLineFlagDirectionValid = 0x04;
constexpr uint8_t kLineFlagPolarityValid = 0x08;
constexpr uint8_t kLineFlagActiveHigh = 0x10;
constexpr uint8_t kMotorFlagEnable = 0x01;

struct LineObservation {
    uint32_t rawLevels = 0; // Reportable 24-bit input levels after configured filtering and disabled-sensor masking.
    uint32_t bits = 0;      // Active white-line mask, populated only after polarity is configured.
    uint8_t count = 0;
    bool detected = false;
    bool ambiguous = false;
    bool polarityValid = false;
    bool activeHigh = false;
    bool directionValid = false;
    int16_t angleCdeg = kNoLineAngle;  // Direction of detected white line, clockwise-positive from front.
};

class LineDebouncer {
public:
    uint32_t update(uint32_t sample, uint8_t requiredSamples) {
        sample &= 0x00FFFFFFu;
        if (!initialized_ || requiredSamples <= 1) {
            stable_ = sample;
            initialized_ = true;
            for (uint8_t i = 0; i < 24; ++i) counts_[i] = 0;
            return stable_;
        }
        for (uint8_t i = 0; i < 24; ++i) {
            const uint32_t bit = 1u << i;
            const bool sampled = (sample & bit) != 0;
            const bool stable = (stable_ & bit) != 0;
            if (sampled == stable) {
                counts_[i] = 0;
                continue;
            }
            const bool candidate = (candidate_ & bit) != 0;
            if (counts_[i] == 0 || candidate != sampled) {
                if (sampled) candidate_ |= bit;
                else candidate_ &= ~bit;
                counts_[i] = 1;
            } else if (counts_[i] < requiredSamples) {
                ++counts_[i];
            }
            if (counts_[i] >= requiredSamples) {
                if (sampled) stable_ |= bit;
                else stable_ &= ~bit;
                counts_[i] = 0;
            }
        }
        return stable_;
    }

    void reset() {
        stable_ = candidate_ = 0;
        initialized_ = false;
        for (uint8_t i = 0; i < 24; ++i) counts_[i] = 0;
    }

private:
    uint32_t stable_ = 0;
    uint32_t candidate_ = 0;
    uint8_t counts_[24]{};
    bool initialized_ = false;
};

struct IrDirectionEstimate {
    float angleDeg = 0.0f;
    float coherence = 0.0f; // Geometric sensor agreement, not a calibrated probability.
    uint32_t totalWeight = 0;
    bool valid = false;
};

inline IrDirectionEstimate estimateIrDirection(const uint16_t* weights, const int16_t* cosine,
                                               const int16_t* sine, uint8_t sensorCount,
                                               uint32_t minimumWeight) {
    IrDirectionEstimate result{};
    if (!weights || !cosine || !sine || sensorCount == 0) return result;
    float vectorX = 0.0f, vectorY = 0.0f, weightedMagnitude = 0.0f;
    for (uint8_t i = 0; i < sensorCount; ++i) {
        const float weight = weights[i];
        vectorX += weight * cosine[i];
        vectorY += weight * sine[i];
        weightedMagnitude += weight * sqrtf(static_cast<float>(cosine[i]) * cosine[i] +
                                             static_cast<float>(sine[i]) * sine[i]);
        result.totalWeight += weights[i];
    }
    if (result.totalWeight < minimumWeight || weightedMagnitude <= 0.0f) return result;
    const float magnitude = sqrtf(vectorX * vectorX + vectorY * vectorY);
    result.coherence = magnitude / weightedMagnitude;
    if (result.coherence > 1.0f) result.coherence = 1.0f;
    if (magnitude <= 0.0f) return result;
    result.angleDeg = atan2f(vectorY, vectorX) * 57.29577951308232f;
    if (result.angleDeg < 0.0f) result.angleDeg += 360.0f;
    result.valid = isfinite(result.angleDeg) && result.angleDeg < 360.0f;
    return result;
}

inline uint8_t countBits24(uint32_t value) {
    uint8_t count = 0;
    value &= 0x00FFFFFFu;
    while (value) { count += static_cast<uint8_t>(value & 1u); value >>= 1; }
    return count;
}

constexpr uint32_t makeLineSensorEnabledMask(const bool enabled[24]) {
    uint32_t mask = 0;
    for (uint8_t i = 0; i < 24; ++i)
        if (enabled[i]) mask |= (uint32_t{1} << i);
    return mask;
}

// Angles are per-sensor centidegrees, measured clockwise from robot-front; validMask marks surveyed entries.
inline LineObservation calculateLine(uint32_t rawLevels, bool polarityValid, bool activeHigh,
                                     const int16_t angleCdeg[24], uint32_t validAngleMask,
                                     uint32_t enabledSensorMask = 0x00FFFFFFu) {
    LineObservation result{};
    enabledSensorMask &= 0x00FFFFFFu;
    result.rawLevels = rawLevels & enabledSensorMask & 0x00FFFFFFu;
    result.polarityValid = polarityValid;
    result.activeHigh = polarityValid && activeHigh;
    if (!polarityValid) {
        result.count = countBits24(result.rawLevels & enabledSensorMask); // Diagnostic count of enabled HIGH levels only.
        return result;
    }
    result.bits = (activeHigh ? result.rawLevels : (~result.rawLevels & 0x00FFFFFFu)) & enabledSensorMask;
    result.count = countBits24(result.bits);
    result.detected = result.count != 0;
    if (!result.detected) return result;
    if (!angleCdeg || (validAngleMask & result.bits) != result.bits) return result;
    float x = 0.0f, y = 0.0f;
    for (uint8_t i = 0; i < 24; ++i) {
        if (!(result.bits & (1u << i))) continue;
        if (angleCdeg[i] <= -18000 || angleCdeg[i] >= 18000) return result;
        const float angle = static_cast<float>(angleCdeg[i]) * 0.0001745329252f;
        x += sinf(angle);
        y += cosf(angle);
    }
    if (x * x + y * y < 0.0225f * result.count * result.count) {
        result.ambiguous = true;
        return result;
    }
    float degrees = atan2f(x, y) * 57.2957795f;
    if (degrees > 180.0f) degrees -= 360.0f;
    if (degrees < -180.0f) degrees += 360.0f;
    result.angleCdeg = static_cast<int16_t>(degrees * 100.0f);
    result.directionValid = true;
    return result;
}

inline Packet makeLinePacket(const LineObservation& line, uint8_t sequence) {
    Packet p{};
    p.type = kLineStatus;
    p.sequence = sequence;
    p.length = 7;
    p.payload[0] = static_cast<uint8_t>(line.rawLevels);
    p.payload[1] = static_cast<uint8_t>(line.rawLevels >> 8);
    p.payload[2] = static_cast<uint8_t>(line.rawLevels >> 16);
    writeU16LE(p.payload + 3, static_cast<uint16_t>(line.angleCdeg));
    p.payload[5] = static_cast<uint8_t>((line.detected ? kLineFlagDetected : 0) |
                                        (line.ambiguous ? kLineFlagAmbiguous : 0) |
                                        (line.directionValid ? kLineFlagDirectionValid : 0) |
                                        (line.polarityValid ? kLineFlagPolarityValid : 0) |
                                        (line.activeHigh ? kLineFlagActiveHigh : 0));
    p.payload[6] = line.count;
    return p;
}

inline Packet makeLineRequest(uint8_t sequence) {
    Packet p{};
    p.type = kLineRequest;
    p.sequence = sequence;
    return p;
}

inline bool decodeLineRequest(const Packet& p) {
    return p.type == kLineRequest && p.length == 0;
}

inline bool decodeLinePacket(const Packet& p, LineObservation& line,
                             const int16_t angleCdeg[24] = nullptr, uint32_t validAngleMask = 0,
                             uint32_t enabledSensorMask = 0x00FFFFFFu) {
    enabledSensorMask &= 0x00FFFFFFu;
    if (p.type != kLineStatus || p.length != 7 || (p.payload[5] & ~0x1Fu) ||
        p.payload[6] > countBits24(enabledSensorMask)) return false;
    line.rawLevels = static_cast<uint32_t>(p.payload[0]) | (static_cast<uint32_t>(p.payload[1]) << 8) |
                     (static_cast<uint32_t>(p.payload[2]) << 16);
    if (line.rawLevels & ~enabledSensorMask) return false;
    line.count = p.payload[6];
    line.detected = (p.payload[5] & kLineFlagDetected) != 0;
    line.ambiguous = (p.payload[5] & kLineFlagAmbiguous) != 0;
    line.directionValid = (p.payload[5] & kLineFlagDirectionValid) != 0;
    line.polarityValid = (p.payload[5] & kLineFlagPolarityValid) != 0;
    line.activeHigh = (p.payload[5] & kLineFlagActiveHigh) != 0;
    line.angleCdeg = readI16LE(p.payload + 3);
    if (line.polarityValid) {
        line.bits = (line.activeHigh ? line.rawLevels : (~line.rawLevels & 0x00FFFFFFu)) & enabledSensorMask;
        if (line.count != countBits24(line.bits) || line.detected != (line.count != 0)) return false;
        const bool allAnglesKnown = angleCdeg && (validAngleMask & line.bits) == line.bits;
        if (line.detected && allAnglesKnown) {
            const LineObservation calculated = calculateLine(line.rawLevels, true, line.activeHigh,
                                                             angleCdeg, validAngleMask, enabledSensorMask);
            if (line.directionValid != calculated.directionValid || line.ambiguous != calculated.ambiguous ||
                line.angleCdeg != calculated.angleCdeg) return false;
        } else if (line.directionValid || line.ambiguous || line.angleCdeg != kNoLineAngle) {
            return false;
        }
    } else {
        line.bits = 0;
        if (line.detected || line.ambiguous || line.directionValid || line.angleCdeg != kNoLineAngle ||
            line.activeHigh || line.count != countBits24(line.rawLevels & enabledSensorMask)) return false;
    }
    if (line.directionValid) {
        if (!line.detected || line.ambiguous || line.angleCdeg <= -18000 || line.angleCdeg >= 18000) return false;
    } else if (line.angleCdeg != kNoLineAngle) return false;
    if (line.ambiguous && !line.detected) return false;
    return true;
}

struct Motion {
    float x = 0.0f;    // Right, normalized -1..1.
    float y = 0.0f;    // Forward, normalized -1..1.
    float turn = 0.0f; // Counter-clockwise, normalized -1..1.
};

inline bool validMotion(const Motion& m) {
    return isfinite(m.x) && isfinite(m.y) && isfinite(m.turn) &&
           fabsf(m.x) <= 1.0f && fabsf(m.y) <= 1.0f && fabsf(m.turn) <= 1.0f;
}

inline Packet makeMotorPacket(const Motion& motion, bool enabled, uint8_t sequence) {
    Packet p{};
    if (!validMotion(motion)) return p;
    p.type = kMotorCommand;
    p.sequence = sequence;
    p.length = 7;
    const float values[3] = {motion.x, motion.y, motion.turn};
    for (uint8_t i = 0; i < 3; ++i) {
        const int16_t scaled = static_cast<int16_t>(values[i] * 1000.0f);
        writeI16LE(p.payload + 2 * i, scaled);
    }
    p.payload[6] = enabled ? kMotorFlagEnable : 0;
    return p;
}

inline bool decodeMotorPacket(const Packet& p, Motion& motion, bool& enabled) {
    if (p.type != kMotorCommand || p.length != 7 || (p.payload[6] & ~kMotorFlagEnable)) return false;
    motion.x = readI16LE(p.payload) / 1000.0f;
    motion.y = readI16LE(p.payload + 2) / 1000.0f;
    motion.turn = readI16LE(p.payload + 4) / 1000.0f;
    enabled = (p.payload[6] & kMotorFlagEnable) != 0;
    return validMotion(motion);
}

struct WheelConfig {
    float positionX;
    float positionY;
    float driveAngleDeg;
    int8_t polarity;
};

enum Tb67h450Mode : uint8_t { kTb67h450Stop, kTb67h450Forward, kTb67h450Reverse, kTb67h450Brake };

struct Tb67h450Output {
    uint8_t in1 = 0;
    uint8_t in2 = 0;
    Tb67h450Mode mode = kTb67h450Stop;
};

struct Tb67h450ChannelConfig {
    int8_t pinIn1 = -1;
    int8_t pinIn2 = -1;
    uint16_t maxDuty = 128;
};

constexpr uint16_t kMotorPwmRange = 255;
constexpr uint16_t kTb67h450MaxDuty = 128;
constexpr uint32_t kTb67h450PwmFrequencyHz = 20000;
constexpr uint32_t kTb67h450ReversalDwellMs = 200;

inline bool tb67h450Encode(Tb67h450Mode mode, uint16_t duty, uint16_t dutyLimit,
                           Tb67h450Output& output) {
    output = {};
    if (dutyLimit == 0 || dutyLimit > kTb67h450MaxDuty || duty > dutyLimit) return false;
    switch (mode) {
        case kTb67h450Stop:
            if (duty != 0) return false;
            output = {};
            return true;
        case kTb67h450Forward:
            output = duty == 0 ? Tb67h450Output{} : Tb67h450Output{static_cast<uint8_t>(duty), 0, mode};
            return true;
        case kTb67h450Reverse:
            output = duty == 0 ? Tb67h450Output{} : Tb67h450Output{0, static_cast<uint8_t>(duty), mode};
            return true;
        case kTb67h450Brake:
            if (duty != 0) return false;
            output = {255, 255, mode};
            return true;
    }
    return false;
}

inline bool tb67h450OutputForWheel(float wheel, uint16_t dutyLimit, Tb67h450Output& output) {
    if (!isfinite(wheel) || fabsf(wheel) > 1.0f || dutyLimit == 0 ||
        dutyLimit > kTb67h450MaxDuty) return false;
    const uint16_t duty = static_cast<uint16_t>(fabsf(wheel) * dutyLimit + 0.5f);
    if (duty == 0) return tb67h450Encode(kTb67h450Stop, 0, dutyLimit, output);
    return tb67h450Encode(wheel > 0 ? kTb67h450Forward : kTb67h450Reverse,
                          duty, dutyLimit, output);
}

inline bool validMotorPwmSettings(uint32_t frequencyHz, uint16_t range, uint32_t clockHz) {
    // The Arduino-Pico core reduces timer range when one PWM period has fewer source-clock ticks.
    return frequencyHz >= 100 && frequencyHz <= 1000000 && range == kMotorPwmRange &&
           static_cast<uint64_t>(frequencyHz) * range <= clockHz;
}

inline bool validRs485PinConfig(bool verified, uint8_t gpioCount, int8_t tx, int8_t rx, int8_t de) {
    if (!verified || gpioCount == 0 || gpioCount > 48 || tx < 0 || rx < 0 || de < 0 ||
        tx >= gpioCount || rx >= gpioCount || de >= gpioCount) return false;
    return tx != rx && tx != de && rx != de;
}

inline bool validMotorPinMap(const Tb67h450ChannelConfig driver[4], uint64_t pwmPinMask,
                             uint64_t reservedPinMask, uint8_t gpioCount = 30) {
    if (gpioCount == 0 || gpioCount > 48) return false;
    uint64_t used = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        if (driver[i].maxDuty == 0 || driver[i].maxDuty > kTb67h450MaxDuty) return false;
        const int8_t pins[2] = {driver[i].pinIn1, driver[i].pinIn2};
        // The paired brake transition uses one atomic SIO-bank write.
        if ((pins[0] < 32) != (pins[1] < 32)) return false;
        for (uint8_t j = 0; j < 2; ++j) {
            if (pins[j] < 0 || pins[j] >= gpioCount) return false;
            const uint64_t bit = uint64_t{1} << static_cast<uint8_t>(pins[j]);
            if (!(pwmPinMask & bit) || (reservedPinMask & bit) || (used & bit)) return false;
            used |= bit;
        }
    }
    return true;
}

inline bool validSafetyInputConfig(const int8_t pins[3], const int8_t activeLevels[3],
                                   bool biasVerified, uint64_t reservedPinMask, uint8_t gpioCount = 30) {
    if (!biasVerified || gpioCount == 0 || gpioCount > 48) return false;
    uint64_t used = 0;
    for (uint8_t i = 0; i < 3; ++i) {
        if (pins[i] < 0 || pins[i] >= gpioCount || (activeLevels[i] != 0 && activeLevels[i] != 1)) return false;
        const uint64_t bit = uint64_t{1} << static_cast<uint8_t>(pins[i]);
        if ((reservedPinMask & bit) || (used & bit)) return false;
        used |= bit;
    }
    return true;
}

inline bool validMotorHardwareConfig(bool outputEnabled, bool wheelsConfigured,
                                     bool driverTableConfigured, uint32_t pwmFrequency,
                                     uint16_t pwmRange, uint32_t pwmClockHz, const WheelConfig wheels[4],
                                     const Tb67h450ChannelConfig driver[4], uint64_t pwmPinMask,
                                     uint64_t reservedPinMask, uint8_t gpioCount = 30) {
    if (!outputEnabled || !wheelsConfigured || !driverTableConfigured ||
        !validMotorPwmSettings(pwmFrequency, pwmRange, pwmClockHz) ||
        !validMotorPinMap(driver, pwmPinMask, reservedPinMask, gpioCount)) return false;
    for (uint8_t i = 0; i < 4; ++i) {
        const WheelConfig& w = wheels[i];
        if (!isfinite(w.positionX) || !isfinite(w.positionY) || !isfinite(w.driveAngleDeg) ||
            (w.polarity != 1 && w.polarity != -1) ||
            (fabsf(w.positionX) + fabsf(w.positionY) < 0.001f)) return false;
        for (uint8_t previous = 0; previous < i; ++previous)
            if (w.positionX == wheels[previous].positionX && w.positionY == wheels[previous].positionY) return false;
    }
    return true;
}

class MotorWheelSlew {
public:
    bool update(float target, float maxDelta, uint32_t nowMs, uint32_t reversalDwellMs,
                float& output) {
        if (!isfinite(target) || fabsf(target) > 1.0f || !isfinite(output) ||
            fabsf(output) > 1.0f || !isfinite(maxDelta) || maxDelta <= 0.0f || maxDelta > 1.0f) {
            reset();
            output = 0.0f;
            return false;
        }
        const int8_t requestedDirection = target > 0.0f ? 1 : (target < 0.0f ? -1 : 0);
        if (direction_ == 0 && requestedDirection != 0) direction_ = requestedDirection;
        if (requestedDirection != 0 && requestedDirection != direction_) {
            if (output != 0.0f) {
                approach(0.0f, maxDelta, output);
                if (output == 0.0f) markStopped(nowMs);
                return true;
            }
            if (!stoppedSinceValid_) markStopped(nowMs);
            if (static_cast<uint32_t>(nowMs - stoppedSinceMs_) < reversalDwellMs) return true;
            direction_ = requestedDirection;
        } else if (requestedDirection == 0) {
            approach(0.0f, maxDelta, output);
            if (output == 0.0f && direction_ != 0 && !stoppedSinceValid_) markStopped(nowMs);
            return true;
        }
        stoppedSinceValid_ = false;
        approach(target, maxDelta, output);
        return true;
    }

    void reset() { direction_ = 0; stoppedSinceMs_ = 0; stoppedSinceValid_ = false; }

private:
    static void approach(float target, float maxDelta, float& output) {
        const float delta = target - output;
        if (delta > maxDelta) output += maxDelta;
        else if (delta < -maxDelta) output -= maxDelta;
        else output = target;
    }

    void markStopped(uint32_t nowMs) { stoppedSinceMs_ = nowMs; stoppedSinceValid_ = true; }

    int8_t direction_ = 0;
    uint32_t stoppedSinceMs_ = 0;
    bool stoppedSinceValid_ = false;
};

class MotorCommandPriority {
public:
    bool accept(const Motion& motion, bool enabled) {
        if (!enabled || (motion.x == 0.0f && motion.y == 0.0f && motion.turn == 0.0f)) {
            stopPending_ = true;
            return true;
        }
        return !stopPending_;
    }

    void latchStop() { stopPending_ = true; }
    bool stopPending() const { return stopPending_; }
    void completeOutputTick() { stopPending_ = false; }

private:
    bool stopPending_ = false;
};

enum MotorSafetyState : uint8_t { kMotorDisarmed, kMotorArmed, kMotorFault };

inline bool motorCommandExpired(uint32_t nowMs, uint32_t lastCommandMs, uint32_t timeoutMs) {
    return static_cast<uint32_t>(nowMs - lastCommandMs) >= timeoutMs;
}

struct MotorSafetyInput {
    bool inputsConfigured = false;
    bool hardwareReady = false;
    bool armPressed = false;
    bool emergencyStop = true;
    bool clearFaultPressed = false;
    bool commandFresh = false;
    bool commandEnabled = false;
    bool commandFault = false;
};

class MotorSafetyGate {
public:
    bool update(const MotorSafetyInput& input) {
        if (input.emergencyStop || input.commandFault ||
            (input.commandEnabled && !input.hardwareReady)) latchFault();
        if (state_ == kMotorFault) {
            if (input.inputsConfigured && input.hardwareReady && input.clearFaultPressed &&
                !input.emergencyStop && !input.armPressed && !input.commandEnabled) {
                state_ = kMotorDisarmed;
                armReleaseSeen_ = true;
                previousArmPressed_ = false;
            }
            return false;
        }
        if (!input.inputsConfigured || !input.hardwareReady) {
            if (state_ == kMotorArmed) latchFault();
            else state_ = kMotorDisarmed;
            armReleaseSeen_ = !input.armPressed;
            previousArmPressed_ = input.armPressed;
            return false;
        }
        if (state_ == kMotorArmed && (!input.commandFresh || !input.commandEnabled)) {
            latchFault();
            return false;
        }
        if (!input.commandFresh || !input.commandEnabled) {
            state_ = kMotorDisarmed;
            armReleaseSeen_ = !input.armPressed;
            previousArmPressed_ = input.armPressed;
            return false;
        }
        if (!armReleaseSeen_) {
            if (!input.armPressed) {
                armReleaseSeen_ = true;
                previousArmPressed_ = false;
            } else {
                previousArmPressed_ = true;
            }
            return false;
        }
        if (!input.armPressed) {
            previousArmPressed_ = false;
            return state_ == kMotorArmed;
        }
        if (!previousArmPressed_) state_ = kMotorArmed;
        previousArmPressed_ = true;
        return state_ == kMotorArmed;
    }

    void tripFault() { latchFault(); }
    MotorSafetyState state() const { return state_; }

private:
    void latchFault() { state_ = kMotorFault; }

    MotorSafetyState state_ = kMotorDisarmed;
    bool armReleaseSeen_ = false;
    bool previousArmPressed_ = false;
};

inline bool expireMotorCommand(MotorSafetyGate& gate, bool commandSeen, uint32_t nowMs,
                               uint32_t lastCommandMs, uint32_t timeoutMs) {
    if (!commandSeen || !motorCommandExpired(nowMs, lastCommandMs, timeoutMs)) return false;
    if (gate.state() == kMotorArmed) gate.tripFault();
    return true;
}

class ManualArmLatch {
public:
    bool update(bool inputConfigured, bool armPressed, bool fault) {
        if (!inputConfigured || fault) {
            reset();
            previousArmPressed_ = armPressed;
            return false;
        }
        if (!armPressed) {
            armReleaseSeen_ = true;
            previousArmPressed_ = false;
            return armed_;
        }
        if (armReleaseSeen_ && !previousArmPressed_) armed_ = true;
        previousArmPressed_ = true;
        return armed_;
    }

    void reset() {
        armed_ = false;
        armReleaseSeen_ = false;
        previousArmPressed_ = false;
    }

private:
    bool armed_ = false;
    bool armReleaseSeen_ = false;
    bool previousArmPressed_ = false;
};

class ManualActionLatch {
public:
    bool update(bool inputConfigured, bool buttonPressed, bool inhibited) {
        if (!inputConfigured || inhibited) {
            reset();
            previousPressed_ = buttonPressed;
            return false;
        }
        if (!buttonPressed) {
            releaseSeen_ = true;
            previousPressed_ = false;
            return latched_;
        }
        if (releaseSeen_ && !previousPressed_) latched_ = true;
        previousPressed_ = true;
        return latched_;
    }

    void reset() {
        releaseSeen_ = false;
        previousPressed_ = false;
        latched_ = false;
    }

private:
    bool releaseSeen_ = false;
    bool previousPressed_ = false;
    bool latched_ = false;
};

inline bool omniKinematics(const Motion& motion, const WheelConfig wheels[4], float output[4]) {
    if (!validMotion(motion)) return false;
    float largest = 0.0f;
    for (uint8_t i = 0; i < 4; ++i) {
        const WheelConfig& w = wheels[i];
        if (!isfinite(w.positionX) || !isfinite(w.positionY) || !isfinite(w.driveAngleDeg) ||
            (w.polarity != 1 && w.polarity != -1) ||
            (fabsf(w.positionX) + fabsf(w.positionY) < 0.001f)) return false;
        const float a = w.driveAngleDeg * 0.01745329252f;
        const float dx = cosf(a), dy = sinf(a);
        const float rotation = -w.positionY * dx + w.positionX * dy;
        output[i] = (motion.x * dx + motion.y * dy + motion.turn * rotation) * w.polarity;
        if (fabsf(output[i]) > largest) largest = fabsf(output[i]);
    }
    if (largest > 1.0f) for (uint8_t i = 0; i < 4; ++i) output[i] /= largest;
    return true;
}

inline bool slewWheels(const float target[4], float current[4], float maxDelta) {
    if (!isfinite(maxDelta) || maxDelta <= 0.0f) return false;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!isfinite(target[i]) || fabsf(target[i]) > 1.0f) return false;
        if (!isfinite(current[i]) || fabsf(current[i]) > 1.0f) return false;
        if (current[i] * target[i] < 0.0f) {
            current[i] = 0.0f;
            continue;
        }
        const float delta = target[i] - current[i];
        if (delta > maxDelta) current[i] += maxDelta;
        else if (delta < -maxDelta) current[i] -= maxDelta;
        else current[i] = target[i];
    }
    return true;
}

inline bool safeMotorUpdate(const Motion& command, bool commandFresh, bool enabled,
                            const WheelConfig wheels[4], MotorWheelSlew slew[4],
                            float current[4], float maxDelta, uint32_t nowMs,
                            uint32_t reversalDwellMs) {
    if (!commandFresh || !enabled) {
        for (uint8_t i = 0; i < 4; ++i) { current[i] = 0.0f; slew[i].reset(); }
        return true;
    }
    if (!validMotion(command)) {
        for (uint8_t i = 0; i < 4; ++i) { current[i] = 0.0f; slew[i].reset(); }
        return false;
    }
    if (command.x == 0.0f && command.y == 0.0f && command.turn == 0.0f) {
        bool valid = true;
        for (uint8_t i = 0; i < 4; ++i) {
            valid &= slew[i].update(0.0f, 1.0f, nowMs, reversalDwellMs, current[i]);
        }
        if (!valid) for (uint8_t i = 0; i < 4; ++i) { current[i] = 0.0f; slew[i].reset(); }
        if (!valid) return false;
        return true;
    }
    float target[4] = {};
    if (!omniKinematics(command, wheels, target)) {
        for (uint8_t i = 0; i < 4; ++i) { current[i] = 0.0f; slew[i].reset(); }
        return false;
    }
    for (uint8_t i = 0; i < 4; ++i) {
        if (!slew[i].update(target[i], maxDelta, nowMs, reversalDwellMs, current[i])) {
            for (uint8_t j = 0; j < 4; ++j) { current[j] = 0.0f; slew[j].reset(); }
            return false;
        }
    }
    return true;
}

inline float normalizeDegrees(float angle) {
    if (!isfinite(angle)) return NAN;
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f) angle += 360.0f;
    return angle;
}

inline float shortestAngleError(float target, float current) {
    if (!isfinite(target) || !isfinite(current)) return NAN;
    float error = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
    return error;
}

// Uses at most the newest three valid sample intervals and wraps the bearing across 0 degrees.
struct IrPrediction {
    float angleDeg = 0.0f;
    float relativeSignal = 0.0f;
    bool valid = false;
};

inline IrPrediction predictIr(const float* angleSamples, const float* signalSamples,
                              const uint32_t* timestampsUs, uint8_t head, uint8_t count,
                              uint8_t capacity, float horizonSeconds,
                              float maxAngularSpeedDegPerSec) {
    IrPrediction result{};
    if (!angleSamples || !signalSamples || !timestampsUs || capacity == 0 ||
        head >= capacity || count == 0 || count > capacity ||
        !isfinite(horizonSeconds) || horizonSeconds < 0.0f ||
        !isfinite(maxAngularSpeedDegPerSec) || maxAngularSpeedDegPerSec <= 0.0f) return result;

    const uint16_t latest = (static_cast<uint16_t>(head) + capacity - 1u) % capacity;
    if (!isfinite(angleSamples[latest]) || !isfinite(signalSamples[latest])) return result;
    result.angleDeg = normalizeDegrees(angleSamples[latest]);
    result.relativeSignal = signalSamples[latest];
    result.valid = isfinite(result.angleDeg);
    if (!result.valid || count < 2 || horizonSeconds == 0.0f) return result;

    const uint8_t segments = count - 1u < 3u ? count - 1u : 3u;
    float angularVelocitySum = 0.0f;
    float signalVelocitySum = 0.0f;
    uint8_t validSegments = 0;
    for (uint8_t i = 0; i < segments; ++i) {
        const uint16_t newer = (static_cast<uint16_t>(head) + capacity - 1u - i) % capacity;
        const uint16_t older = (static_cast<uint16_t>(head) + capacity - 2u - i) % capacity;
        const uint32_t deltaUs = timestampsUs[newer] - timestampsUs[older];
        if (deltaUs < 1000u || deltaUs > 200000u ||
            !isfinite(angleSamples[newer]) || !isfinite(angleSamples[older]) ||
            !isfinite(signalSamples[newer]) || !isfinite(signalSamples[older])) continue;

        float angleDelta = angleSamples[newer] - angleSamples[older];
        while (angleDelta > 180.0f) angleDelta -= 360.0f;
        while (angleDelta < -180.0f) angleDelta += 360.0f;
        const float intervalSeconds = deltaUs / 1000000.0f;
        const float angularVelocity = angleDelta / intervalSeconds;
        const float signalVelocity = (signalSamples[newer] - signalSamples[older]) / intervalSeconds;
        if (!isfinite(angularVelocity) || !isfinite(signalVelocity)) continue;
        angularVelocitySum += angularVelocity;
        signalVelocitySum += signalVelocity;
        ++validSegments;
    }
    if (validSegments == 0) return result;

    float angularVelocity = angularVelocitySum / validSegments;
    if (!isfinite(angularVelocity)) return result;
    if (angularVelocity > maxAngularSpeedDegPerSec) angularVelocity = maxAngularSpeedDegPerSec;
    if (angularVelocity < -maxAngularSpeedDegPerSec) angularVelocity = -maxAngularSpeedDegPerSec;
    const float predictedAngle = normalizeDegrees(result.angleDeg + angularVelocity * horizonSeconds);
    if (isfinite(predictedAngle)) result.angleDeg = predictedAngle;

    const float predictedSignal = result.relativeSignal +
                                 (signalVelocitySum / validSegments) * horizonSeconds;
    if (isfinite(predictedSignal)) result.relativeSignal = predictedSignal > 0.0f ? predictedSignal : 0.0f;
    return result;
}

class HeadingController {
public:
    bool update(float heading, float target, bool valid, float& turn) {
        turn = 0.0f;
        if (!valid || !isfinite(heading) || !isfinite(target)) {
            jumpPending_ = false;
            return false;
        }
        heading = normalizeDegrees(heading);
        target = normalizeDegrees(target);
        if (!initialized_ || fabsf(shortestAngleError(target, target_)) > 0.01f) {
            target_ = target;
            previous_ = heading;
            initialized_ = true;
            jumpPending_ = false;
        } else {
            if (fabsf(shortestAngleError(heading, previous_)) > 45.0f) {
                if (!jumpPending_ || fabsf(shortestAngleError(heading, jumpCandidate_)) > 5.0f) {
                    jumpCandidate_ = heading;
                    jumpPending_ = true;
                    return false;
                }
                jumpPending_ = false;
            } else {
                jumpPending_ = false;
            }
            previous_ = heading;
        }
        turn = constrainValue(shortestAngleError(target_, heading) * 0.012f, -0.35f, 0.35f);
        return true;
    }

    void reset() { initialized_ = false; jumpPending_ = false; target_ = 0.0f; previous_ = 0.0f; jumpCandidate_ = 0.0f; }

private:
    static float constrainValue(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
    bool initialized_ = false;
    bool jumpPending_ = false;
    float target_ = 0.0f;
    float previous_ = 0.0f;
    float jumpCandidate_ = 0.0f;
};

enum TacticalState : uint8_t { kDisarmed, kIdle, kSearchBall, kApproachBall, kAvoidLine, kFault };

struct MainInput {
    uint32_t nowMs = 0;
    bool armed = false;
    bool fault = false;
    bool emergencyStop = true;
    bool clearFault = false;
    bool lineFresh = false;
    LineObservation line{};
    bool irFresh = false;
    bool ballFound = false;
    uint16_t ballAngleDeg = 0;
    uint16_t relativeSignal = 0;
    bool predictedAngleValid = false;
    uint16_t predictedAngleDeg = 0;
    bool headingValid = false;
    bool headingRequired = false;
    float headingDeg = 0.0f;
    float headingTargetDeg = NAN;
};

struct MainOutput {
    TacticalState state = kDisarmed;
    Motion motion{};
};

class MainController {
public:
    MainOutput step(const MainInput& in) {
        MainOutput out{};
        if (!in.armed && !in.emergencyStop && !in.fault && !faultLatched_) {
            requiresRearm_ = false;
            clearMotionState();
            out.state = kDisarmed;
            return out;
        }
        float headingTurn = 0.0f;
        const bool headingOk = heading_.update(in.headingDeg, in.headingTargetDeg, in.headingValid, headingTurn);
        if (in.emergencyStop || in.fault ||
            (in.armed && (!in.lineFresh || !in.line.polarityValid || !in.irFresh ||
                          (in.headingRequired && !headingOk)))) {
            faultLatched_ = true;
            recoveryTiming_ = false;
        }
        if (faultLatched_) {
            const bool canRecover = !in.emergencyStop && !in.fault && in.clearFault && in.lineFresh &&
                                    in.line.polarityValid && in.irFresh && !in.line.detected &&
                                    (!in.headingRequired || headingOk);
            if (canRecover) {
                if (!recoveryTiming_) { recoveryTiming_ = true; recoverySinceMs_ = in.nowMs; }
                if (static_cast<uint32_t>(in.nowMs - recoverySinceMs_) >= 500) {
                    faultLatched_ = false;
                    recoveryTiming_ = false;
                    requiresRearm_ = true;
                    clearMotionState();
                }
            } else {
                recoveryTiming_ = false;
            }
            if (faultLatched_) {
                out.state = kFault;
                return out;
            }
        }
        if (requiresRearm_) {
            if (!in.armed) requiresRearm_ = false;
            else {
                out.state = kDisarmed;
                return out;
            }
        }
        if (!in.armed) {
            clearMotionState();
            out.state = kDisarmed;
            return out;
        }
        if (in.line.detected) {
            avoiding_ = true;
            lineClearTiming_ = false;
        } else if (avoiding_) {
            if (!lineClearTiming_) { lineClearSince_ = in.nowMs; lineClearTiming_ = true; }
            if (static_cast<uint32_t>(in.nowMs - lineClearSince_) >= 200) avoiding_ = false;
        }
        if (avoiding_) {
            out.state = kAvoidLine;
            if (in.line.detected && in.line.directionValid && !in.line.ambiguous &&
                in.line.angleCdeg != kNoLineAngle) {
                const float escape = normalizeDegrees(in.line.angleCdeg / 100.0f + 180.0f);
                out.motion.x = sinf(escape * 0.01745329252f) * 0.35f;
                out.motion.y = cosf(escape * 0.01745329252f) * 0.35f;
                out.motion.turn = headingOk ? headingTurn : 0.0f;
            }
            return out;
        }
        if (in.irFresh && in.ballFound && in.ballAngleDeg < 360) {
            out.state = kApproachBall;
            const float angle = in.predictedAngleValid && in.predictedAngleDeg < 360
                                    ? in.predictedAngleDeg : in.ballAngleDeg;
            float filtered = angle;
            if (ballAngleInitialized_) filtered = normalizeDegrees(ballAngle_ + shortestAngleError(angle, ballAngle_) * 0.25f);
            ballAngle_ = filtered;
            ballAngleInitialized_ = true;
            // shortcut: 200-count midpoint is provisional; tune from sensor logs before arming.
            const float magnitude = 0.2f + 0.35f * (in.relativeSignal / (in.relativeSignal + 200.0f));
            out.motion.x = sinf(filtered * 0.01745329252f) * magnitude;
            out.motion.y = cosf(filtered * 0.01745329252f) * magnitude;
            out.motion.turn = headingOk ? headingTurn : 0.0f;
            return out;
        }
        ballAngleInitialized_ = false;
        if (in.irFresh) {
            out.state = kSearchBall;
            if (headingOk) {
                out.motion.x = (in.nowMs % 1400 < 700) ? 0.15f : -0.15f;
                out.motion.turn = headingTurn;
            } else if (!in.headingRequired) {
                out.motion.turn = 0.12f;
            }
        } else {
            out.state = kIdle;
            out.motion.turn = headingOk ? headingTurn : 0.0f;
        }
        return out;
    }

private:
    void clearMotionState() {
        avoiding_ = false;
        lineClearTiming_ = false;
        ballAngleInitialized_ = false;
        heading_.reset();
    }
    HeadingController heading_{};
    bool faultLatched_ = false;
    bool recoveryTiming_ = false;
    bool requiresRearm_ = false;
    uint32_t recoverySinceMs_ = 0;
    bool avoiding_ = false;
    bool lineClearTiming_ = false;
    uint32_t lineClearSince_ = 0;
    bool ballAngleInitialized_ = false;
    float ballAngle_ = 0.0f;
};

}  // namespace soccer
