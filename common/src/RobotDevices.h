#pragma once

#include <math.h>
#include <stdint.h>

namespace soccer {

constexpr uint8_t kBnoAddressAuto = 0;
constexpr uint32_t kBnoRetryMs = 1000;

struct Bno055Io {
    void* context = nullptr;
    bool (*begin)(void*, uint8_t address) = nullptr;
    bool (*read)(void*, float& headingDeg, uint8_t calibration[4]) = nullptr;
};

struct Bno055Sample {
    bool connected = false;
    bool headingValid = false; // Requires a finite 0..360 reading and SYS calibration==3.
    float headingDeg = 0.0f;
    uint8_t calibration[4]{}; // SYS, gyro, accelerometer, magnetometer.
    uint8_t address = 0;
};

class Bno055Monitor {
public:
    bool poll(Bno055Io& io, uint8_t configuredAddress, uint32_t nowMs, Bno055Sample& sample) {
        sample = {};
        if (!io.context || !io.begin || !io.read ||
            (configuredAddress != kBnoAddressAuto && configuredAddress != 0x28 && configuredAddress != 0x29))
            return false;
        if (!initialized_) {
            if (attempted_ && static_cast<uint32_t>(nowMs - lastAttemptMs_) < kBnoRetryMs) return false;
            attempted_ = true;
            lastAttemptMs_ = nowMs;
            if (configuredAddress == kBnoAddressAuto) {
                if (io.begin(io.context, 0x28)) activeAddress_ = 0x28;
                else if (io.begin(io.context, 0x29)) activeAddress_ = 0x29;
                else return false;
            } else {
                if (!io.begin(io.context, configuredAddress)) return false;
                activeAddress_ = configuredAddress;
            }
            initialized_ = true;
        }
        sample.address = activeAddress_;
        float heading = 0.0f;
        if (!io.read(io.context, heading, sample.calibration)) {
            initialized_ = false;
            lastAttemptMs_ = nowMs;
            return false;
        }
        sample.connected = true;
        sample.headingDeg = heading;
        sample.headingValid = isfinite(heading) && heading >= 0.0f && heading < 360.0f &&
                              sample.calibration[0] == 3;
        return sample.headingValid;
    }

    bool initialized() const { return initialized_; }
    uint8_t activeAddress() const { return activeAddress_; }

private:
    bool initialized_ = false;
    bool attempted_ = false;
    uint8_t activeAddress_ = 0;
    uint32_t lastAttemptMs_ = 0;
};

// Host simulation only; never use these values as physical kicker timing.
constexpr uint32_t kKickerMockPulseMs = 60;
constexpr uint32_t kKickerMockCooldownMs = 1000;

enum KickerState : uint8_t { kKickerDisarmed, kKickerReady, kKickerFiring, kKickerCooldown, kKickerFault };
enum KickerEvent : uint8_t {
    kKickerNoEvent, kKickerBecameReady, kKickerPulseStarted, kKickerPulseCompleted,
    kKickerInhibited, kKickerFaultLatched, kKickerFaultCleared
};

struct KickerInput {
    bool request = false;
    bool commandFresh = false;
    bool armed = false;
    bool emergencyStop = true;
    bool fault = false;
    bool clearFault = false;
    bool hardwareConfigured = false;
    bool ballFresh = false;
    bool ballDetected = false;
    bool headingValid = false;
    bool headingAligned = false;
    bool lineClear = false;
};

inline bool kickerRequestAllowed(bool policyConfigured, bool approachingBall, const KickerInput& in) {
    return policyConfigured && approachingBall && in.commandFresh && in.armed && !in.emergencyStop &&
           !in.fault && in.hardwareConfigured && in.ballFresh && in.ballDetected && in.headingValid &&
           in.headingAligned && in.lineClear;
}

struct KickerEventRecord {
    uint32_t timestampMs = 0;
    KickerEvent event = kKickerNoEvent;
    KickerState state = kKickerDisarmed;
};

struct KickerOutput {
    KickerState state = kKickerDisarmed;
    bool mockPulseActive = false;
    KickerEvent event = kKickerNoEvent;
};

class KickerController {
public:
    KickerOutput update(const KickerInput& in, uint32_t nowMs) {
        const bool requestRising = in.request && !requestWasHigh_;
        requestWasHigh_ = in.request;
        KickerEvent event = kKickerNoEvent;

        if (in.emergencyStop || in.fault) {
            mockPulseActive_ = false;
            if (state_ != kKickerFault) event = transition(kKickerFault, kKickerFaultLatched, nowMs);
        } else if (state_ == kKickerFault) {
            mockPulseActive_ = false;
            if (in.clearFault && in.commandFresh && !in.request)
                event = transition(kKickerDisarmed, kKickerFaultCleared, nowMs);
        } else if (!interlocksReady(in)) {
            mockPulseActive_ = false;
            if (state_ != kKickerDisarmed) event = transition(kKickerDisarmed, kKickerInhibited, nowMs);
        } else {
            bool enteredReady = false;
            if (state_ == kKickerFiring &&
                static_cast<uint32_t>(nowMs - pulseStartedMs_) >= kKickerMockPulseMs) {
                mockPulseActive_ = false;
                cooldownStartedMs_ = nowMs;
                event = transition(kKickerCooldown, kKickerPulseCompleted, nowMs);
            }
            if (state_ == kKickerCooldown &&
                static_cast<uint32_t>(nowMs - cooldownStartedMs_) >= kKickerMockCooldownMs)
                event = transition(kKickerReady, kKickerBecameReady, nowMs);
            if (state_ == kKickerDisarmed) {
                event = transition(kKickerReady, kKickerBecameReady, nowMs);
                enteredReady = true;
            }
            if (state_ == kKickerReady && requestRising && !enteredReady) {
                mockPulseActive_ = true;
                pulseStartedMs_ = nowMs;
                event = transition(kKickerFiring, kKickerPulseStarted, nowMs);
            }
        }
        return {state_, mockPulseActive_, event};
    }

    KickerState state() const { return state_; }
    bool mockPulseActive() const { return mockPulseActive_; }
    uint8_t eventHistoryCount() const { return eventCount_; }

    bool eventHistoryAt(uint8_t oldestFirstIndex, KickerEventRecord& record) const {
        if (oldestFirstIndex >= eventCount_) return false;
        const uint8_t oldest = static_cast<uint8_t>((eventHead_ + kEventCapacity - eventCount_) % kEventCapacity);
        record = eventHistory_[(oldest + oldestFirstIndex) % kEventCapacity];
        return true;
    }

private:
    static constexpr uint8_t kEventCapacity = 8;

    static bool interlocksReady(const KickerInput& in) {
        return in.commandFresh && in.armed && in.hardwareConfigured && in.ballFresh && in.ballDetected &&
               in.headingValid && in.headingAligned && in.lineClear;
    }

    KickerEvent transition(KickerState state, KickerEvent event, uint32_t nowMs) {
        if (state_ == state) return kKickerNoEvent;
        state_ = state;
        if (event != kKickerNoEvent) {
            eventHistory_[eventHead_] = {nowMs, event, state_};
            eventHead_ = static_cast<uint8_t>((eventHead_ + 1) % kEventCapacity);
            if (eventCount_ < kEventCapacity) ++eventCount_;
        }
        return event;
    }

    KickerState state_ = kKickerDisarmed;
    bool requestWasHigh_ = false;
    bool mockPulseActive_ = false;
    uint32_t pulseStartedMs_ = 0;
    uint32_t cooldownStartedMs_ = 0;
    KickerEventRecord eventHistory_[kEventCapacity]{};
    uint8_t eventHead_ = 0;
    uint8_t eventCount_ = 0;
};

class DribblerController {
public:
    float update(float speed, bool commandFresh, bool armed, bool emergencyStop, bool fault,
                 bool hardwareConfigured) {
        if (!isfinite(speed) || fabsf(speed) > 1.0f || !commandFresh || !armed || emergencyStop ||
            fault || !hardwareConfigured) return 0.0f;
        return speed;
    }
};

} // namespace soccer
