#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>

// shortcut: shared modules are copied into four sketches; automate synchronization if updates start drifting.
// BEGIN INTEGRATED COMMON CODE
#include <stdint.h>

namespace soccer {

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kFrameStart0 = 0xA5;
constexpr uint8_t kFrameStart1 = 0x5A;
constexpr uint8_t kMaxPayload = 8;
constexpr uint8_t kMaxFrame = 2 + 4 + kMaxPayload + 2;
constexpr uint32_t kFrameTimeoutMs = 8;
constexpr uint8_t kMotorCommandStart = 0xAA;
constexpr uint8_t kMotorCommandWheelCount = 4;
constexpr uint8_t kMotorCommandEnd = 0x55;
constexpr uint8_t kMotorCommandFrameSize = 12;
constexpr int16_t kMotorBrakeCommand = 256;

enum MessageType : uint8_t { kLineStatus = 1, kLineRequest = 3 };

struct Packet {
    uint8_t type = 0;
    uint8_t sequence = 0;
    uint8_t length = 0;
    uint8_t payload[kMaxPayload] = {};
};

inline uint16_t crc16(const uint8_t* data, uint8_t length) {
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                  : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

inline uint8_t encodePacket(const Packet& packet, uint8_t* out, uint8_t capacity) {
    if ((packet.type != kLineStatus && packet.type != kLineRequest) ||
        packet.length > kMaxPayload ||
        capacity < static_cast<uint8_t>(8 + packet.length)) return 0;
    out[0] = kFrameStart0;
    out[1] = kFrameStart1;
    out[2] = kProtocolVersion;
    out[3] = packet.type;
    out[4] = packet.length;
    out[5] = packet.sequence;
    for (uint8_t i = 0; i < packet.length; ++i) out[6 + i] = packet.payload[i];
    const uint16_t crc = crc16(out + 2, static_cast<uint8_t>(4 + packet.length));
    out[6 + packet.length] = static_cast<uint8_t>(crc);
    out[7 + packet.length] = static_cast<uint8_t>(crc >> 8);
    return static_cast<uint8_t>(8 + packet.length);
}

struct ParserStats {
    uint16_t crcErrors = 0;
    uint16_t formatErrors = 0;
    uint16_t timeouts = 0;
};

class PacketParser {
public:
    bool feed(uint8_t byte, uint32_t nowMs, Packet& packet) {
        poll(nowMs);
        lastByteMs_ = nowMs;
        if (used_ == 0) {
            if (byte == kFrameStart0) buffer_[used_++] = byte;
            return false;
        }
        if (used_ == 1) {
            if (byte == kFrameStart1) buffer_[used_++] = byte;
            else if (byte != kFrameStart0) reset();
            return false;
        }
        if (used_ >= kMaxFrame) {
            ++stats_.formatErrors;
            reset();
            return false;
        }
        buffer_[used_++] = byte;
        if (used_ == 5 && (buffer_[2] != kProtocolVersion || buffer_[4] > kMaxPayload)) {
            ++stats_.formatErrors;
            reset();
            return false;
        }
        if (used_ < 5 || used_ < static_cast<uint8_t>(8 + buffer_[4])) return false;

        const uint8_t payloadLength = buffer_[4];
        const uint16_t expected = crc16(buffer_ + 2, static_cast<uint8_t>(4 + payloadLength));
        const uint16_t received = static_cast<uint16_t>(buffer_[6 + payloadLength]) |
                                  (static_cast<uint16_t>(buffer_[7 + payloadLength]) << 8);
        const uint8_t type = buffer_[3];
        if (expected != received) {
            ++stats_.crcErrors;
            reset();
            return false;
        }
        if (type != kLineStatus && type != kLineRequest) {
            ++stats_.formatErrors;
            reset();
            return false;
        }
        packet.type = type;
        packet.sequence = buffer_[5];
        packet.length = payloadLength;
        for (uint8_t i = 0; i < payloadLength; ++i) packet.payload[i] = buffer_[6 + i];
        reset();
        return true;
    }

    bool poll(uint32_t nowMs) {
        if (used_ && static_cast<uint32_t>(nowMs - lastByteMs_) > kFrameTimeoutMs) {
            ++stats_.timeouts;
            reset();
            return true;
        }
        return false;
    }

    const ParserStats& stats() const { return stats_; }

private:
    void reset() { used_ = 0; }
    uint8_t buffer_[kMaxFrame] = {};
    uint8_t used_ = 0;
    uint32_t lastByteMs_ = 0;
    ParserStats stats_{};
};

enum MotorFrameStatus : uint8_t { kMotorFramePending, kMotorFrameReady, kMotorFrameInvalid };

struct MotorFrameStats {
    uint16_t checksumErrors = 0;
    uint16_t formatErrors = 0;
    uint16_t rangeErrors = 0;
    uint16_t timeouts = 0;
};

inline bool validMotorCommandValue(int16_t value) {
    return (value >= -255 && value <= 255) || value == kMotorBrakeCommand;
}

inline bool parseMotorConsoleCommand(const char* line, int16_t values[kMotorCommandWheelCount]) {
    if (!line || !values) return false;
    const char* p = line;
    const auto skipSpace = [&p]() {
        while (*p == ' ' || *p == '\t') ++p;
    };
    skipSpace();
    if ((*p == 'S' || *p == 's') && (p[1] == '\0' || p[1] == ' ' || p[1] == '\t')) {
        ++p;
        skipSpace();
        if (*p != '\0') return false;
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) values[i] = 0;
        return true;
    }
    if (*p != 'M' && *p != 'm') return false;
    ++p;
    int16_t parsed[kMotorCommandWheelCount]{};
    for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) {
        skipSpace();
        int32_t sign = 1;
        if (*p == '+' || *p == '-') {
            if (*p++ == '-') sign = -1;
        }
        if (*p < '0' || *p > '9') return false;
        int32_t value = 0;
        do {
            value = value * 10 + (*p++ - '0');
            if (value > 256) return false;
        } while (*p >= '0' && *p <= '9');
        const int32_t signedValue = sign * value;
        if (signedValue < -255 || signedValue > 256) return false;
        parsed[i] = static_cast<int16_t>(signedValue);
    }
    skipSpace();
    if (*p != '\0') return false;
    for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) values[i] = parsed[i];
    return true;
}

enum MainConsoleCommand : uint8_t {
    kConsoleInvalid,
    kConsoleStop,
    kConsoleManualMotor,
    kConsoleAutoWheelLift,
    kConsoleAutoFloor,
    kConsoleAutoIrTest,
    kConsoleArmIrTest
};

inline void skipConsoleSpace(const char*& p) {
    while (*p == ' ' || *p == '\t') ++p;
}

inline bool consumeConsoleWord(const char*& p, const char* word) {
    const char* candidate = p;
    for (const char* expected = word; *expected; ++expected, ++candidate) {
        char actual = *candidate;
        if (actual >= 'a' && actual <= 'z') actual = static_cast<char>(actual - 'a' + 'A');
        if (actual != *expected) return false;
    }
    if (*candidate != '\0' && *candidate != ' ' && *candidate != '\t') return false;
    p = candidate;
    return true;
}

inline MainConsoleCommand parseMainConsoleCommand(const char* line,
                                                  int16_t values[kMotorCommandWheelCount]) {
    if (!line || !values) return kConsoleInvalid;
    const char* p = line;
    skipConsoleSpace(p);
    if (consumeConsoleWord(p, "AUTO")) {
        skipConsoleSpace(p);
        MainConsoleCommand command = kConsoleAutoWheelLift;
        if (*p && !consumeConsoleWord(p, "FLOAT")) {
            if (consumeConsoleWord(p, "FLOOR")) command = kConsoleAutoFloor;
            else if (consumeConsoleWord(p, "IR")) command = kConsoleAutoIrTest;
            else return kConsoleInvalid;
        }
        skipConsoleSpace(p);
        if (*p) return kConsoleInvalid;
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) values[i] = 0;
        return command;
    }
    if (consumeConsoleWord(p, "ARM")) {
        skipConsoleSpace(p);
        if (!consumeConsoleWord(p, "IR")) return kConsoleInvalid;
        skipConsoleSpace(p);
        if (*p) return kConsoleInvalid;
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) values[i] = 0;
        return kConsoleArmIrTest;
    }
    if (!parseMotorConsoleCommand(line, values)) return kConsoleInvalid;
    skipConsoleSpace(p);
    return (*p == 'S' || *p == 's') ? kConsoleStop : kConsoleManualMotor;
}

inline uint8_t encodeMotorCommandFrame(const int16_t values[kMotorCommandWheelCount],
                                       uint8_t* out, uint8_t capacity) {
    if (!values || !out || capacity < kMotorCommandFrameSize) return 0;
    out[0] = kMotorCommandStart;
    out[1] = kMotorCommandWheelCount;
    for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) {
        if (!validMotorCommandValue(values[i])) return 0;
        const uint16_t raw = static_cast<uint16_t>(values[i]);
        out[2 + 2 * i] = static_cast<uint8_t>(raw);
        out[3 + 2 * i] = static_cast<uint8_t>(raw >> 8);
    }
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < kMotorCommandFrameSize - 2; ++i) checksum ^= out[i];
    out[kMotorCommandFrameSize - 2] = checksum;
    out[kMotorCommandFrameSize - 1] = kMotorCommandEnd;
    return kMotorCommandFrameSize;
}

class MotorCommandFrameParser {
public:
    MotorFrameStatus feed(uint8_t byte, uint32_t nowMs,
                          int16_t values[kMotorCommandWheelCount]) {
        if (!values) {
            ++stats_.formatErrors;
            reset();
            return kMotorFrameInvalid;
        }
        if (poll(nowMs)) {
            if (byte == kMotorCommandStart) start(nowMs);
            return kMotorFrameInvalid;
        }

        // shortcut: skip LINE by header/length, not CRC; add CRC checks if MOTOR must own LINE integrity.
        if (lineFrameUsed_) {
            if (lineFrameUsed_ == 1 && byte != kFrameStart1) {
                resetLineFrame();
            } else {
                ++lineFrameUsed_;
                lineFrameLastByteMs_ = nowMs;
                if ((lineFrameUsed_ == 3 && byte != kProtocolVersion) ||
                    (lineFrameUsed_ == 4 && byte != kLineStatus && byte != kLineRequest) ||
                    (lineFrameUsed_ == 5 && byte > kMaxPayload)) {
                    ++stats_.formatErrors;
                    resetLineFrame();
                    return kMotorFrameInvalid;
                }
                if (lineFrameUsed_ == 5) lineFrameExpected_ = static_cast<uint8_t>(8 + byte);
                if (lineFrameExpected_ && lineFrameUsed_ >= lineFrameExpected_) resetLineFrame();
                return kMotorFramePending;
            }
        }
        if (!used_ && byte == kFrameStart0) {
            lineFrameUsed_ = 1;
            lineFrameLastByteMs_ = nowMs;
            return kMotorFramePending;
        }

        lastByteMs_ = nowMs;
        if (used_ == 0) {
            if (byte == kMotorCommandStart) buffer_[used_++] = byte;
            return kMotorFramePending;
        }
        if (used_ == 1 && byte != kMotorCommandWheelCount) {
            if (byte > 1) {
                ++stats_.formatErrors;
                reset();
                if (byte == kMotorCommandStart) start(nowMs);
                return kMotorFrameInvalid;
            }
        }
        buffer_[used_++] = byte;
        if (used_ == 6 && buffer_[1] <= 1) {
            const uint16_t direction = (static_cast<uint16_t>(buffer_[1]) << 8) | buffer_[2];
            const bool validIrResponse = buffer_[5] == kMotorCommandEnd && direction < 360;
            reset();
            if (!validIrResponse) {
                ++stats_.formatErrors;
                return kMotorFrameInvalid;
            }
            return kMotorFramePending;
        }
        if (used_ < kMotorCommandFrameSize) return kMotorFramePending;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < kMotorCommandFrameSize - 2; ++i) checksum ^= buffer_[i];
        if (buffer_[kMotorCommandFrameSize - 1] != kMotorCommandEnd) {
            ++stats_.formatErrors;
            const bool restart = byte == kMotorCommandStart;
            reset();
            if (restart) start(nowMs);
            return kMotorFrameInvalid;
        }
        if (checksum != buffer_[kMotorCommandFrameSize - 2]) {
            ++stats_.checksumErrors;
            reset();
            return kMotorFrameInvalid;
        }
        int16_t decoded[kMotorCommandWheelCount];
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) {
            const uint16_t raw = static_cast<uint16_t>(buffer_[2 + 2 * i]) |
                                 (static_cast<uint16_t>(buffer_[3 + 2 * i]) << 8);
            decoded[i] = raw < 0x8000 ? static_cast<int16_t>(raw)
                                      : static_cast<int16_t>(static_cast<int32_t>(raw) - 65536);
            if (!validMotorCommandValue(decoded[i])) {
                ++stats_.rangeErrors;
                reset();
                return kMotorFrameInvalid;
            }
        }
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) values[i] = decoded[i];
        reset();
        return kMotorFrameReady;
    }

    bool poll(uint32_t nowMs) {
        if (lineFrameUsed_ &&
            static_cast<uint32_t>(nowMs - lineFrameLastByteMs_) > kFrameTimeoutMs) {
            ++stats_.timeouts;
            resetLineFrame();
        }
        if (used_ && static_cast<uint32_t>(nowMs - lastByteMs_) > kFrameTimeoutMs) {
            ++stats_.timeouts;
            reset();
            return true;
        }
        return false;
    }

    const MotorFrameStats& stats() const { return stats_; }

private:
    void start(uint32_t nowMs) { buffer_[0] = kMotorCommandStart; used_ = 1; lastByteMs_ = nowMs; }
    void reset() { used_ = 0; }
    void resetLineFrame() { lineFrameUsed_ = 0; lineFrameExpected_ = 0; }
    uint8_t buffer_[kMotorCommandFrameSize]{};
    uint8_t used_ = 0;
    uint32_t lastByteMs_ = 0;
    MotorFrameStats stats_{};
    uint8_t lineFrameUsed_ = 0;
    uint8_t lineFrameExpected_ = 0;
    uint32_t lineFrameLastByteMs_ = 0;
};

// The legacy IR request is a single 0x01 byte; skip framed bus traffic so 0x01 inside a frame is not a request.
class SharedBusIrRequestParser {
public:
    bool feed(uint8_t byte, uint32_t nowMs) {
        poll(nowMs);
        lastByteMs_ = nowMs;
        if (motorSkipRemaining_) {
            --motorSkipRemaining_;
            return false;
        }
        if (motorStartPending_) {
            motorStartPending_ = false;
            if (byte == kMotorCommandWheelCount) {
                motorSkipRemaining_ = kMotorCommandFrameSize - 2;
                return false;
            }
            if (byte == kMotorCommandStart) {
                motorStartPending_ = true;
                return false;
            }
        }
        if (discard_) return false;
        if (used_ == 0) {
            if (byte == kMotorCommandStart) {
                motorStartPending_ = true;
                return false;
            }
            if (byte == kFrameStart0) {
                buffer_[used_++] = byte;
                return false;
            }
            return byte == 0x01;
        }
        if (used_ == 1) {
            if (byte == kFrameStart1) buffer_[used_++] = byte;
            else {
                reset();
                if (byte == kFrameStart0) buffer_[used_++] = byte;
            }
            return false;
        }
        if (used_ >= kMaxFrame) {
            ++stats_.formatErrors;
            discard_ = true;
            used_ = 0;
            return false;
        }
        buffer_[used_++] = byte;
        if (used_ == 5) {
            if (buffer_[2] != kProtocolVersion) ++stats_.formatErrors;
            if (buffer_[4] > kMaxPayload) {
                ++stats_.formatErrors;
                discard_ = true;
                used_ = 0;
                return false;
            }
            expected_ = static_cast<uint8_t>(8 + buffer_[4]);
        }
        if (expected_ && used_ >= expected_) reset();
        return false;
    }

    bool poll(uint32_t nowMs) {
        if ((used_ || discard_ || motorStartPending_ || motorSkipRemaining_) &&
            static_cast<uint32_t>(nowMs - lastByteMs_) > kFrameTimeoutMs) {
            ++stats_.timeouts;
            reset();
            return true;
        }
        return false;
    }

    const ParserStats& stats() const { return stats_; }

private:
    void reset() { used_ = 0; expected_ = 0; discard_ = false; motorStartPending_ = false; motorSkipRemaining_ = 0; }
    uint8_t buffer_[kMaxFrame]{};
    uint8_t used_ = 0;
    uint8_t expected_ = 0;
    uint32_t lastByteMs_ = 0;
    bool discard_ = false;
    bool motorStartPending_ = false;
    uint8_t motorSkipRemaining_ = 0;
    ParserStats stats_{};
};

class SequenceTracker {
public:
    bool accept(uint8_t sequence) {
        if (!seen_) {
            seen_ = true;
            last_ = sequence;
            return true;
        }
        const uint8_t delta = static_cast<uint8_t>(sequence - last_);
        if (delta == 0 || delta >= 128) {
            ++rejected_;
            return false;
        }
        if (delta != 1) ++gaps_;
        last_ = sequence;
        return true;
    }
    // Start a new peer baseline after a communication outage; keep lifetime diagnostics.
    void resetPeer() { seen_ = false; }
    uint16_t gaps() const { return gaps_; }
    uint16_t rejected() const { return rejected_; }

private:
    bool seen_ = false;
    uint8_t last_ = 0;
    uint16_t gaps_ = 0;
    uint16_t rejected_ = 0;
};

class LegacyIrParser {
public:
    bool feed(uint8_t byte, uint32_t nowMs, uint16_t& direction, uint16_t& strength) {
        poll(nowMs);
        lastByteMs_ = nowMs;
        if (state_ == 0) {
            if (byte == 0xAA) state_ = 1;
            return false;
        }
        if (state_ == 5) {
            state_ = 0;
            if (byte != 0x55) {
                ++errors_;
                if (byte == 0xAA) state_ = 1;
                return false;
            }
            direction = (static_cast<uint16_t>(data_[0]) << 8) | data_[1];
            strength = (static_cast<uint16_t>(data_[2]) << 8) | data_[3];
            if (direction >= 360) { ++errors_; return false; }
            return true;
        }
        data_[state_ - 1] = byte;
        ++state_;
        return false;
    }
    bool poll(uint32_t nowMs) {
        if (state_ && static_cast<uint32_t>(nowMs - lastByteMs_) > 10) {
            ++timeouts_;
            state_ = 0;
            return true;
        }
        return false;
    }
    uint16_t errors() const { return errors_; }
    uint16_t timeouts() const { return timeouts_; }

private:
    uint8_t state_ = 0;
    uint8_t data_[4] = {};
    uint32_t lastByteMs_ = 0;
    uint16_t errors_ = 0;
    uint16_t timeouts_ = 0;
};

inline uint16_t readU16LE(const uint8_t* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
inline int16_t readI16LE(const uint8_t* p) {
    const uint16_t v = readU16LE(p);
    return v < 0x8000 ? static_cast<int16_t>(v) : static_cast<int16_t>(static_cast<int32_t>(v) - 65536);
}
inline void writeU16LE(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}
inline void writeI16LE(uint8_t* p, int16_t v) { writeU16LE(p, static_cast<uint16_t>(v)); }

}  // namespace soccer

#include <math.h>

namespace soccer {

constexpr int16_t kNoLineAngle = static_cast<int16_t>(0x8000);
constexpr uint8_t kLineFlagDetected = 0x01;
constexpr uint8_t kLineFlagAmbiguous = 0x02;
constexpr uint8_t kLineFlagDirectionValid = 0x04;
constexpr uint8_t kLineFlagPolarityValid = 0x08;
constexpr uint8_t kLineFlagActiveHigh = 0x10;

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

constexpr uint8_t kIrTestPwmCap = 16;

// shortcut: wheel geometry is provisional; replace after a four-wheel direction check.
inline bool irTestMotionForBearing(uint16_t directionDeg, Motion& motion) {
    motion = {};
    if (directionDeg >= 360) return false;
    const uint16_t sector = static_cast<uint16_t>(((directionDeg + 15u) / 30u * 30u) % 360u);
    if (sector == 0) motion.y = 0.1f;
    else motion.turn = sector <= 180 ? -0.1f : 0.1f;
    return true;
}

class IrTestController {
public:
    bool arm(bool irFresh, bool ballFound, uint16_t directionDeg) {
        if (!irFresh || !ballFound || directionDeg >= 360) {
            disarm();
            return false;
        }
        armed_ = true;
        return true;
    }

    bool armOnFirstFreshBall(bool irFresh, bool ballFound, uint16_t directionDeg) {
        if (!autoStartPending_ || !irFresh || !ballFound) return false;
        autoStartPending_ = false;
        return arm(irFresh, ballFound, directionDeg);
    }

    void disableAutoStart() { autoStartPending_ = false; }

    bool update(bool irFresh, bool ballFound, uint16_t directionDeg, Motion& motion) {
        motion = {};
        if (!armed_) return false;
        if (!irFresh || !ballFound || !irTestMotionForBearing(directionDeg, motion)) {
            disarm();
            motion = {};
            return false;
        }
        return true;
    }

    void disarm() { armed_ = false; }
    bool armed() const { return armed_; }

private:
    bool autoStartPending_ = true;
    bool armed_ = false;
};

enum AutoMode : uint8_t { kAutoStopped, kAutoWheelLift, kAutoFloor, kAutoIrTest };

inline AutoMode autoModeForCommand(MainConsoleCommand command, bool wheelLiftReady, bool floorReady) {
    if (command == kConsoleAutoIrTest) return kAutoIrTest;
    if (command == kConsoleAutoWheelLift) return wheelLiftReady ? kAutoWheelLift : kAutoStopped;
    if (command == kConsoleAutoFloor) return floorReady ? kAutoFloor : kAutoStopped;
    return kAutoStopped;
}

inline void capMotorCommandValues(int16_t values[4], uint8_t maxPwm) {
    if (!values) return;
    for (uint8_t i = 0; i < 4; ++i) {
        if (values[i] > maxPwm) values[i] = maxPwm;
        else if (values[i] < -static_cast<int16_t>(maxPwm)) values[i] = -static_cast<int16_t>(maxPwm);
    }
}

inline bool validMotion(const Motion& m) {
    return isfinite(m.x) && isfinite(m.y) && isfinite(m.turn) &&
           fabsf(m.x) <= 1.0f && fabsf(m.y) <= 1.0f && fabsf(m.turn) <= 1.0f;
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
constexpr uint16_t kTb67h450MaxDuty = 255;
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

inline bool tb67h450RequiresNeutralDwell(Tb67h450Mode appliedMode, Tb67h450Mode requestedMode,
                                         bool neutralSinceValid, uint32_t nowMs,
                                         uint32_t neutralSinceMs, uint32_t dwellMs) {
    if ((requestedMode != kTb67h450Forward && requestedMode != kTb67h450Reverse) ||
        appliedMode == requestedMode) return false;
    return appliedMode != kTb67h450Stop || !neutralSinceValid ||
           static_cast<uint32_t>(nowMs - neutralSinceMs) < dwellMs;
}

inline bool validMotorPwmSettings(uint32_t frequencyHz, uint16_t range, uint32_t clockHz) {
    // The Arduino-Pico core reduces timer range when one PWM period has fewer source-clock ticks.
    return frequencyHz >= 100 && frequencyHz <= 1000000 && range == kMotorPwmRange &&
           static_cast<uint64_t>(frequencyHz) * range <= clockHz;
}

inline bool validRs485PinConfig(bool configured, uint8_t gpioCount, int8_t tx, int8_t rx, int8_t de) {
    if (!configured || gpioCount == 0 || gpioCount > 48 || tx < 0 || rx < 0 || de < 0 ||
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
    bool accept(bool enabled) {
        if (!enabled) {
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

inline bool motorCommandValuesForMotion(const Motion& motion, bool enabled,
                                        const WheelConfig wheels[4], int16_t values[4]) {
    if (!values) return false;
    for (uint8_t i = 0; i < 4; ++i) values[i] = 0;
    if (!enabled) return true;
    float wheel[4]{};
    if (!wheels || !omniKinematics(motion, wheels, wheel)) return false;
    for (uint8_t i = 0; i < 4; ++i) values[i] = static_cast<int16_t>(roundf(wheel[i] * 255.0f));
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

#include <stdint.h>

namespace soccer { namespace board {

// MAIN, IR, and LINE pin maps follow the current project configuration; verify physical continuity on the board.
// MOTOR routes below use the user's reference code; RS485 and safety wiring remain physically unverified.
constexpr int8_t kMainRs485TxPin = 0;
constexpr int8_t kMainRs485RxPin = 1;
constexpr int8_t kMainRs485DePin = 8;
constexpr int8_t kKickerPin = 11; // User-reported MAIN wiring; circuit review is still pending.

constexpr int8_t kIrRs485TxPin = 0;
constexpr int8_t kIrRs485RxPin = 1;
constexpr int8_t kIrRs485DePin = 2;

constexpr int8_t kLineRs485TxPin = 0;
constexpr int8_t kLineRs485RxPin = 1;
constexpr int8_t kLineRs485DePin = 27;
constexpr int8_t kLineSensorPins[24] = {
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25
};
constexpr bool kLineSensorEnabled[24] = {
    true, false, true, true,
    true, false, true, true,
    false, false, true, false,
    true, true, true, true,
    true, true, true, true,
    true, false, true, true
};
constexpr uint32_t kLineSensorEnabledMask = makeLineSensorEnabledMask(kLineSensorEnabled);

constexpr int8_t kMotorRs485TxPin = 0;
constexpr int8_t kMotorRs485RxPin = 1;
constexpr int8_t kMotorRs485DePin = 2;
constexpr int8_t kMotorRs485SharedDePin = -1;
constexpr int8_t kMotorIn1Pins[4] = {6, 12, 11, 8};
constexpr int8_t kMotorIn2Pins[4] = {7, 13, 10, 9};
// The user-reported J2 left-front pins are retained for test provenance.
constexpr int8_t kMotorTestIn1CandidatePin = 12;
constexpr int8_t kMotorTestIn2CandidatePin = 13;
constexpr uint8_t kMotorGpioCount = 30; // The final ZIP's MOTOR parent-board BOM specifies RP2354A.
constexpr bool kMotorRs485PinsConfigured = true; // From the reference code, not a hardware test.
constexpr uint64_t kMotorPwmPinMask = (1u << 6) | (1u << 7) | (1u << 8) | (1u << 9) |
                                      (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13);
constexpr uint64_t kMotorReservedPinMask = (1u << 0) | (1u << 1) | (1u << 2);
constexpr uint64_t kMotorSafetyReservedPinMask = kMotorPwmPinMask | kMotorReservedPinMask;

constexpr float kMotorWheelRadiusMm = 30.0f;
constexpr float kMotorCenterToWheelMm = 94.762f;
// Candidate mapping used by the host integration model; polarity and roller directions need a four-wheel test.
constexpr WheelConfig kMotorWheelConfig[4] = {
    {1, -1, 45, 1}, {-1, 1, 225, 1}, {1, 1, 135, 1}, {-1, -1, 315, 1}
};
constexpr bool kMotorWheelConfigConfigured = false;

// No MOTOR safety input pins or electrical bias have been confirmed on the manufactured board.
constexpr int8_t kMotorArmPin = -1;
constexpr int8_t kMotorEStopPin = -1;
constexpr int8_t kMotorClearFaultPin = -1;
constexpr int8_t kMotorArmActiveLevel = -1;
constexpr int8_t kMotorEStopActiveLevel = -1;
constexpr int8_t kMotorClearFaultActiveLevel = -1;
constexpr bool kMotorSafetyInputBiasVerified = false;

// MAIN has no physical E-stop; keep its safety GPIOs unassigned and competition AUTO fail-closed.
constexpr int8_t kMainArmPin = -1;
constexpr int8_t kMainEStopPin = -1;
constexpr int8_t kMainClearFaultPin = -1;
constexpr int8_t kMainArmActiveLevel = -1;
constexpr int8_t kMainEStopActiveLevel = -1;
constexpr int8_t kMainClearFaultActiveLevel = -1;
constexpr bool kMainSafetyInputBiasVerified = false;
constexpr uint32_t kMainSafetyReservedPinMask = (1u << 0) | (1u << 1) | (1u << 4) |
                                                (1u << 5) | (1u << 8) | (1u << 11);

// BNO055 signals are on the MAIN processor in the ir&main schematic.
constexpr int8_t kBnoSdaPin = 4;
constexpr int8_t kBnoSclPin = 5;
constexpr uint32_t kBnoI2cClockHz = 50000;
constexpr uint8_t kBnoAddress = 0x28;
constexpr bool kBnoOffsetsVerified = false; // Legacy offsets remain inactive until a measured calibration is recorded.
constexpr bool kAttackHeadingConfigured = false;
constexpr float kAttackHeadingDeg = 0.0f; // Relative to the first settled heading; ignored until configured.

constexpr bool kLineDebounceConfigured = false; // Candidate filter stays off until sensor edge timing is measured.
constexpr uint8_t kLineDebounceSamples = 2;
constexpr bool kLinePolarityConfigured = false;
constexpr bool kLineDetectHigh = true; // Candidate only; ignored until polarity is measured.
// 添付soccer.zipのsoccer/line2/line2.kicad_pcbにあるQ1..Q24の座標から算出した基板基準角度[1/100度]。中心(100,100)、基板上方向0°、時計回りで1°単位に丸めた値。
// PCBリビジョン、Q番号と実配線chの対応、機体前方オフセット、入力極性は未確認。故障6ch(1,5,8,9,11,21)は kLineSensorEnabled により制御には使われない。
constexpr int32_t kLineBoardAngleCdeg[24] = {
    1100, 2100, 4100, 5100, 7100, 8100, 10100, 11100, 13100, 14100, 16100, 17100,
    19100, 20100, 22100, 23100, 25100, 26100, 28100, 29100, 31100, 32100, 34100, 35100
};
// 機体前方 = 基板角度 - オフセット。基板上方向と機体前方の差は未確定(以前の「0番が正面」申告との11度差を解消しない)。
// 実測で確定するまで kLineFrontOffsetConfigured=false とし、角度は無効=方向推定せず停止側に倒す。
constexpr bool kLineFrontOffsetConfigured = false;
constexpr int16_t kLineFrontOffsetCdeg = 0;
struct LineAngleTable { int16_t v[24]; };
constexpr LineAngleTable makeLineAngleTable(int16_t offsetCdeg) {
    LineAngleTable t{};
    for (uint8_t i = 0; i < 24; ++i) {
        int32_t a = static_cast<int32_t>(kLineBoardAngleCdeg[i]) - offsetCdeg;
        while (a >= 18000) a -= 36000;
        while (a < -18000) a += 36000;
        t.v[i] = static_cast<int16_t>(a);
    }
    return t;
}
constexpr LineAngleTable kLineAngleTable = makeLineAngleTable(kLineFrontOffsetCdeg);
constexpr const int16_t (&kLineSensorAngleCdeg)[24] = kLineAngleTable.v;
constexpr bool kLineAnglesConfigured = false; // 前方オフセット実測後に kLineFrontOffsetConfigured と同時に true にする。
constexpr uint32_t kLineSensorAngleValidMask = kLineSensorEnabledMask;
static_assert(!kLineAnglesConfigured || kLineFrontOffsetConfigured, "LINE angles need a measured front offset");
constexpr bool kMotorHardwareConfigured = true; // Uses the user's four-wheel tested TB67H450 pin map.
constexpr bool kCompetitionArmed = false;
constexpr uint32_t kMotorPwmFrequencyHz = kTb67h450PwmFrequencyHz;
// Compile-time setup gates; runtime sensor freshness and the physical ARM/E-stop path are still required.
constexpr bool kAutoWheelLiftReady = kCompetitionArmed && kMotorWheelConfigConfigured &&
                                     kMainSafetyInputBiasVerified && kMotorSafetyInputBiasVerified &&
                                     kLinePolarityConfigured;
constexpr bool kAutoFloorReady = kAutoWheelLiftReady &&
                                 kLineSensorEnabledMask == 0x00FFFFFFu && kLineFrontOffsetConfigured &&
                                 kLineAnglesConfigured && kLineDebounceConfigured &&
                                 kBnoOffsetsVerified && kAttackHeadingConfigured;
static_assert(!kAutoFloorReady || kAutoWheelLiftReady, "floor AUTO needs wheel-lift AUTO prerequisites");
// Firmware gates enabled for the reported MAIN GPIO11 kicker connection.
// The high-voltage charge/discharge circuit still needs its separate hardware review.
constexpr bool kKickerHardwareConfigured = true;
constexpr bool kKickerRequestPolicyConfigured = true;
constexpr bool kKickerHeadingWindowConfigured = false; // BNO055 unverified; use IR bearing instead.
constexpr float kKickerHeadingToleranceDeg = 0.0f;
// Kick timing: the IR ball bearing must stay inside this front window while approaching.
constexpr float kKickerBallAngleToleranceDeg = 30.0f;
// Conservative unverified starting values for the physical GPIO pulse; measure on the bench.
constexpr uint32_t kKickerPulseMs = 30;
constexpr uint32_t kKickerCooldownMs = 2000;
constexpr bool kDribblerHardwareConfigured = false;

}} // namespace soccer::board

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
    bool ballAligned = false; // IR ball bearing is inside the front kick window.
};

inline bool kickerRequestAllowed(bool policyConfigured, bool approachingBall, const KickerInput& in) {
    return policyConfigured && approachingBall && in.commandFresh && in.armed && !in.emergencyStop &&
           !in.fault && in.hardwareConfigured && in.ballFresh && in.ballDetected && in.ballAligned &&
           in.headingValid && in.headingAligned && in.lineClear;
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
    bool pulseActive() const { return mockPulseActive; } // Physical GPIO adapter reads this.
};

class KickerController {
public:
    KickerOutput update(const KickerInput& in, uint32_t nowMs,
                        uint32_t pulseMs = kKickerMockPulseMs,
                        uint32_t cooldownMs = kKickerMockCooldownMs) {
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
                static_cast<uint32_t>(nowMs - pulseStartedMs_) >= pulseMs) {
                mockPulseActive_ = false;
                cooldownStartedMs_ = nowMs;
                event = transition(kKickerCooldown, kKickerPulseCompleted, nowMs);
            }
            if (state_ == kKickerCooldown &&
                static_cast<uint32_t>(nowMs - cooldownStartedMs_) >= cooldownMs)
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
    bool pulseActive() const { return mockPulseActive_; } // Name used by the physical GPIO adapter.
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
               in.ballAligned && in.headingValid && in.headingAligned && in.lineClear;
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

#include <stddef.h>
#include <stdint.h>

namespace soccer {

constexpr uint32_t kRs485QuietUs = 100; // One 115200-baud character time before the master starts another frame.

struct Rs485Io {
    void* context = nullptr;
    int (*available)(void*) = nullptr;
    int (*read)(void*) = nullptr;
    void (*setTransmit)(void*, bool) = nullptr; // true=DE high, false=receive (/RE low when tied to DE).
    size_t (*write)(void*, const uint8_t*, size_t) = nullptr;
    void (*flush)(void*) = nullptr; // Must wait for the UART shift register, not only the software queue.
    void (*delayMicros)(void*, uint32_t) = nullptr;
};

struct Rs485Diagnostics {
    uint32_t txFrames = 0;
    uint32_t txBytes = 0;
    uint32_t txErrors = 0;
    uint32_t txBusy = 0;
    uint32_t rxBytes = 0;
    uint32_t rxErrors = 0;
    uint32_t lastRxUs = 0;
    bool hasRx = false;
    uint32_t lastTxUs = 0;
    bool hasTx = false;
};

class Rs485Driver {
public:
    explicit Rs485Driver(const Rs485Io& io) : io_(io) {}

    bool ready() const {
        return io_.context && io_.available && io_.read && io_.setTransmit && io_.write &&
               io_.flush && io_.delayMicros;
    }

    bool canTransmit(uint32_t nowUs, bool replyingToRequest = false) const {
        if (!ready() || transmitting_ || io_.available(io_.context) > 0) return false;
        if (replyingToRequest) return true;
        return (!stats_.hasRx || static_cast<uint32_t>(nowUs - stats_.lastRxUs) >= kRs485QuietUs) &&
               (!stats_.hasTx || static_cast<uint32_t>(nowUs - stats_.lastTxUs) >= kRs485QuietUs);
    }

    bool send(const uint8_t* bytes, size_t length, uint32_t nowUs,
              bool replyingToRequest = false, uint32_t beforeTxUs = 20,
              uint32_t afterTxUs = 20, uint32_t afterRxUs = 20) {
        if (!ready() || !bytes || !length) {
            ++stats_.txErrors;
            return false;
        }
        if (!canTransmit(nowUs, replyingToRequest)) {
            ++stats_.txBusy;
            return false;
        }

        transmitting_ = true;
        io_.setTransmit(io_.context, true);
        io_.delayMicros(io_.context, beforeTxUs);
        const size_t written = io_.write(io_.context, bytes, length);
        io_.flush(io_.context);
        io_.delayMicros(io_.context, afterTxUs);
        io_.setTransmit(io_.context, false);
        io_.delayMicros(io_.context, afterRxUs);
        transmitting_ = false;
        stats_.lastTxUs = nowUs;
        stats_.hasTx = true;
        if (written != length) {
            ++stats_.txErrors;
            return false;
        }
        ++stats_.txFrames;
        stats_.txBytes += static_cast<uint32_t>(length);
        return true;
    }

    bool receiveByte(uint8_t& byte, uint32_t nowUs) {
        if (!ready() || io_.available(io_.context) <= 0) return false;
        const int value = io_.read(io_.context);
        if (value < 0) {
            ++stats_.rxErrors;
            return false;
        }
        byte = static_cast<uint8_t>(value);
        ++stats_.rxBytes;
        stats_.lastRxUs = nowUs;
        stats_.hasRx = true;
        return true;
    }

    bool linkFresh(uint32_t nowUs, uint32_t timeoutUs) const {
        return stats_.hasRx && static_cast<uint32_t>(nowUs - stats_.lastRxUs) <= timeoutUs;
    }

    const Rs485Diagnostics& diagnostics() const { return stats_; }

private:
    Rs485Io io_{};
    Rs485Diagnostics stats_{};
    bool transmitting_ = false;
};

enum Rs485Peer : uint8_t { kNoRs485Peer = 0, kIrRs485Peer = 1, kLineRs485Peer = 2 };

class Rs485MasterArbiter {
public:
    bool acquire(Rs485Peer peer) {
        if (peer == kNoRs485Peer || active_ != kNoRs485Peer) {
            ++collisionsPrevented_;
            return false;
        }
        active_ = peer;
        return true;
    }
    bool complete(Rs485Peer peer) {
        if (active_ != peer || peer == kNoRs485Peer) return false;
        active_ = kNoRs485Peer;
        return true;
    }
    Rs485Peer activePeer() const { return active_; }
    bool idle() const { return active_ == kNoRs485Peer; }
    uint32_t collisionsPrevented() const { return collisionsPrevented_; }

private:
    Rs485Peer active_ = kNoRs485Peer;
    uint32_t collisionsPrevented_ = 0;
};

} // namespace soccer
// END INTEGRATED COMMON CODE

using namespace soccer;

constexpr uint32_t kControlPeriodUs = 10000;
constexpr uint32_t kIrPollPeriodMs = 20;
constexpr uint32_t kIrReplyTimeoutUs = 15000;
constexpr uint32_t kLineReplyTimeoutUs = 8000;
constexpr uint32_t kIrFreshTimeoutMs = 60;
constexpr uint32_t kLineFreshTimeoutMs = 100;
constexpr uint32_t kHeadingPollPeriodMs = 20;
constexpr uint32_t kHeadingFreshTimeoutMs = 100;
constexpr uint32_t kDiagnosticPeriodMs = 250;
constexpr uint32_t kMotorCommandPeriodMs = 40;
constexpr uint32_t kMotorConsoleTimeoutMs = 300;
constexpr uint8_t kMotorConsoleLineCapacity = 48;
// shortcut: these are starting PWM ceilings, not measured wheel speeds; raise only after logged low-speed trials.
constexpr uint8_t kAutoWheelLiftPwmCap = 48;
constexpr uint8_t kAutoFloorPwmCap = 24;
constexpr int kBnoSensorId = 55;
constexpr float kBnoJumpThresholdDegrees = 30.0f;
constexpr float kBnoJumpConfirmToleranceDegrees = 5.0f;
constexpr uint32_t kBnoStartupSettleMs = 500;

constexpr int16_t kAccelOffsetX = 12;
constexpr int16_t kAccelOffsetY = -18;
constexpr int16_t kAccelOffsetZ = 3;
constexpr int16_t kMagOffsetX = -228;
constexpr int16_t kMagOffsetY = 1434;
constexpr int16_t kMagOffsetZ = 2039;
constexpr int16_t kGyroOffsetX = -1;
constexpr int16_t kGyroOffsetY = 0;
constexpr int16_t kGyroOffsetZ = -3;
constexpr int16_t kAccelRadius = 1000;
constexpr int16_t kMagRadius = 441;

PacketParser lineParser;
SequenceTracker lineSequence;
Rs485MasterArbiter busArbiter;
enum BusPhase : uint8_t { kBusIdle, kBusNeedLine };
BusPhase busPhase = kBusIdle;
MainController controller;
AutoMode autoMode = kAutoIrTest;
IrTestController irTestController;
KickerController kickerController;
ManualArmLatch operatorArm;
ManualActionLatch clearFaultAction;
TacticalState previousControllerState = kDisarmed;
LineObservation lineData{};
bool lineHasPacket = false;
uint32_t lastLinePacketMs = 0;
uint32_t linePacketPeriodMs = 0;
uint32_t linePeerRestartsInferred = 0;
bool linePeerResyncPending = false;
uint32_t lineRejectedPackets = 0;
uint32_t lineUnsolicitedPackets = 0;
uint32_t linePollTimeouts = 0;
uint8_t linePollSequence = 0;
uint8_t activeLinePollSequence = 0;
uint32_t busRequestStartedUs = 0;
uint32_t nextSensorPollMs = 0;
uint32_t motorFramesSent = 0;
uint32_t motorTxErrors = 0;
uint32_t lastMotorCommandMs = 0;
bool motorCommandSent = false;
bool motorStopPending = false;
Motion motorMotion{};
bool motorEnabled = false;
int16_t motorConsoleValues[kMotorCommandWheelCount]{};
char motorConsoleLine[kMotorConsoleLineCapacity]{};
uint8_t motorConsoleLineLength = 0;
bool motorConsoleOverflow = false;
bool bootAutoArmNoticeSent = false;
bool motorConsoleCommandSeen = false;
uint32_t lastMotorConsoleCommandMs = 0;
uint32_t lastIrPacketMs = 0;
uint32_t irRequests = 0;
uint32_t irMatchedResponses = 0;
uint32_t irResponseTimeouts = 0;
uint32_t irTxErrors = 0;
uint32_t irUnsolicitedResponses = 0;
uint32_t irRecoveriesInferred = 0;
uint32_t irLastLatencyUs = 0;
uint32_t irMaxLatencyUs = 0;
uint16_t irDirection = 0;
uint16_t irRelativeStrength = 0;
bool irPacketSeen = false;
bool irBallFound = false;
bool irLinkStale = true;
bool irPending = false;
LegacyIrParser irParser;
uint32_t lastHeadingPollMs = 0;
uint32_t lastHeadingSampleMs = 0;
Bno055Sample headingSample{};
uint32_t lastControlUs = 0;
uint32_t lastDebugMs = 0;
uint32_t maxControlUs = 0;
uint16_t controlOverruns = 0;
uint8_t diagnosticPhase = 0;

Adafruit_BNO055 bnoSensor(kBnoSensorId, board::kBnoAddress, &Wire);
struct ArduinoBnoDevices {
    Adafruit_BNO055* sensor;
    bool active = false;
    uint8_t activeAddress = 0;
    uint32_t settleStartedMs = 0;
    bool baseHeadingSet = false;
    float baseHeading = 0.0f;
    float previousHeading = 0.0f;
    bool jumpPending = false;
    float jumpCandidate = 0.0f;
    float previousSampleHeading = 0.0f;
    float previousSampleDelta = 0.0f;
    bool previousSampleSet = false;
    bool previousSampleDeltaSet = false;
};
ArduinoBnoDevices bnoDevices{&bnoSensor};
Bno055Monitor bnoMonitor;

int rs485Available(void*) { return Serial1.available(); }
int rs485Read(void*) { return Serial1.read(); }
void setRs485Tx(void*, bool tx) { digitalWrite(board::kMainRs485DePin, tx ? HIGH : LOW); }
size_t writeRs485(void*, const uint8_t* bytes, size_t length) { return Serial1.write(bytes, length); }
void flushRs485(void*) { Serial1.flush(); }
void waitRs485(void*, uint32_t us) { delayMicroseconds(us); }
Rs485Io rs485Io{&Serial1, rs485Available, rs485Read, setRs485Tx, writeRs485, flushRs485, waitRs485};
Rs485Driver rs485{rs485Io};

void stopIrTest() {
    irTestController.disarm();
    motorMotion = {};
    motorEnabled = false;
    motorStopPending = true;
}

constexpr int8_t kMainSafetyPins[3] = {board::kMainArmPin, board::kMainEStopPin,
                                       board::kMainClearFaultPin};
constexpr int8_t kMainSafetyActiveLevels[3] = {board::kMainArmActiveLevel, board::kMainEStopActiveLevel,
                                               board::kMainClearFaultActiveLevel};

bool mainSafetyInputsReady() {
    return validSafetyInputConfig(kMainSafetyPins, kMainSafetyActiveLevels,
                                  board::kMainSafetyInputBiasVerified,
                                  board::kMainSafetyReservedPinMask);
}

bool mainSafetyInputActive(uint8_t index) {
    return digitalRead(kMainSafetyPins[index]) == kMainSafetyActiveLevels[index];
}

uint32_t irSuccessPercent() {
    return irRequests ? static_cast<uint32_t>((static_cast<uint64_t>(irMatchedResponses) * 100u) / irRequests) : 0;
}

void finishIrCycle(uint32_t nowMs) {
    if (autoMode == kAutoIrTest) {
        busPhase = kBusIdle;
        nextSensorPollMs = nowMs + kIrPollPeriodMs;
    } else {
        busPhase = kBusNeedLine;
    }
}

bool readBnoRegister(uint8_t address, uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, static_cast<uint8_t>(1)) != 1) return false;
    value = static_cast<uint8_t>(Wire.read());
    return true;
}

bool bnoBegin(void* context, uint8_t address) {
    ArduinoBnoDevices* devices = static_cast<ArduinoBnoDevices*>(context);
    if (!devices || !devices->sensor || address != board::kBnoAddress) return false;
    devices->active = false;
    devices->activeAddress = 0;
    uint8_t chipId = 0;
    if (!readBnoRegister(address, 0x00, chipId) || chipId != 0xA0) return false;
    if (!devices->sensor->begin(OPERATION_MODE_IMUPLUS)) return false;

    adafruit_bno055_offsets_t offsets{};
    offsets.accel_offset_x = kAccelOffsetX;
    offsets.accel_offset_y = kAccelOffsetY;
    offsets.accel_offset_z = kAccelOffsetZ;
    offsets.gyro_offset_x = kGyroOffsetX;
    offsets.gyro_offset_y = kGyroOffsetY;
    offsets.gyro_offset_z = kGyroOffsetZ;
    offsets.mag_offset_x = kMagOffsetX;
    offsets.mag_offset_y = kMagOffsetY;
    offsets.mag_offset_z = kMagOffsetZ;
    offsets.accel_radius = kAccelRadius;
    offsets.mag_radius = kMagRadius;
    if (board::kBnoOffsetsVerified) devices->sensor->setSensorOffsets(offsets);
    devices->sensor->setExtCrystalUse(true);

    devices->active = true;
    devices->activeAddress = address;
    devices->settleStartedMs = millis();
    devices->baseHeadingSet = false;
    devices->jumpPending = false;
    devices->previousSampleSet = false;
    devices->previousSampleDeltaSet = false;
    return true;
}

bool bnoRead(void* context, float& heading, uint8_t calibration[4]) {
    ArduinoBnoDevices* devices = static_cast<ArduinoBnoDevices*>(context);
    if (!devices || !devices->active) return false;
    uint8_t chipId = 0;
    if (!readBnoRegister(devices->activeAddress, 0x00, chipId) || chipId != 0xA0) {
        devices->active = false;
        devices->activeAddress = 0;
        return false;
    }

    if (static_cast<uint32_t>(millis() - devices->settleStartedMs) < kBnoStartupSettleMs) {
        heading = NAN;
        for (uint8_t i = 0; i < 4; ++i) calibration[i] = 0;
        return true;
    }

    sensors_event_t event{};
    if (!devices->sensor->getEvent(&event, Adafruit_BNO055::VECTOR_EULER)) {
        devices->active = false;
        return false;
    }
    devices->sensor->getCalibration(&calibration[0], &calibration[1], &calibration[2], &calibration[3]);
    float rawHeading = normalizeDegrees(event.orientation.x);
    if (!isfinite(rawHeading)) {
        devices->active = false;
        return false;
    }

    if (!devices->baseHeadingSet) {
        devices->baseHeading = rawHeading;
        devices->previousHeading = rawHeading;
        devices->previousSampleHeading = rawHeading;
        devices->baseHeadingSet = true;
        devices->previousSampleSet = true;
        heading = 0.0f;
        return true;
    }

    const float acceptedDelta = shortestAngleError(rawHeading, devices->previousHeading);
    const float sampleDelta = devices->previousSampleSet
                                  ? shortestAngleError(rawHeading, devices->previousSampleHeading)
                                  : 0.0f;
    const bool continuingFastTurn = devices->previousSampleDeltaSet &&
                                    fabsf(sampleDelta) > 0.1f &&
                                    sampleDelta * devices->previousSampleDelta > 0.0f &&
                                    fabsf(fabsf(sampleDelta) - fabsf(devices->previousSampleDelta)) <=
                                        kBnoJumpConfirmToleranceDegrees;
    float filteredHeading = rawHeading;
    if (fabsf(acceptedDelta) > kBnoJumpThresholdDegrees) {
        // shortcut: 30 degrees per 20 ms; raise only after gyro logs confirm faster real turns.
        if (continuingFastTurn ||
            (devices->jumpPending &&
             fabsf(shortestAngleError(rawHeading, devices->jumpCandidate)) <=
                 kBnoJumpConfirmToleranceDegrees)) {
            const float step = acceptedDelta > 0.0f ? kBnoJumpThresholdDegrees : -kBnoJumpThresholdDegrees;
            filteredHeading = normalizeDegrees(devices->previousHeading + step);
            devices->previousHeading = filteredHeading;
            devices->jumpPending = fabsf(shortestAngleError(rawHeading, filteredHeading)) >
                                   kBnoJumpThresholdDegrees;
            if (devices->jumpPending) devices->jumpCandidate = rawHeading;
        } else {
            devices->jumpCandidate = rawHeading;
            devices->jumpPending = true;
            filteredHeading = devices->previousHeading;
        }
    } else {
        devices->previousHeading = rawHeading;
        devices->jumpPending = false;
    }
    devices->previousSampleHeading = rawHeading;
    devices->previousSampleSet = true;
    devices->previousSampleDelta = sampleDelta;
    devices->previousSampleDeltaSet = true;
    heading = normalizeDegrees(filteredHeading - devices->baseHeading);
    return true;
}

Bno055Io bnoIo{&bnoDevices, bnoBegin, bnoRead};

void pollHeading(uint32_t nowMs) {
    if (static_cast<uint32_t>(nowMs - lastHeadingPollMs) < kHeadingPollPeriodMs) return;
    lastHeadingPollMs = nowMs;
    bnoMonitor.poll(bnoIo, board::kBnoAddress, nowMs, headingSample);
    if (headingSample.connected) lastHeadingSampleMs = nowMs;
}

void acceptIrResponse(uint16_t direction, uint16_t strength, uint32_t nowMs) {
    if (busArbiter.activePeer() != kIrRs485Peer || !busArbiter.complete(kIrRs485Peer)) {
        ++irUnsolicitedResponses;
        return;
    }
    irLastLatencyUs = micros() - busRequestStartedUs;
    if (irLastLatencyUs > irMaxLatencyUs) irMaxLatencyUs = irLastLatencyUs;
    ++irMatchedResponses;
    irPending = false;
    irDirection = direction;
    irRelativeStrength = strength; // The legacy DIST field is a relative detector signal, not centimetres.
    irBallFound = strength != 0;
    if (irLinkStale) ++irRecoveriesInferred;
    irLinkStale = false;
    irPacketSeen = true;
    lastIrPacketMs = nowMs;
    finishIrCycle(nowMs);
}

void finishLineCycle(uint32_t nowMs) {
    busArbiter.complete(kLineRs485Peer);
    busPhase = kBusIdle;
    nextSensorPollMs = nowMs + kIrPollPeriodMs;
}

void acceptLineResponse(const Packet& packet, uint32_t nowMs) {
    if (packet.type != kLineStatus) return;
    if (busArbiter.activePeer() != kLineRs485Peer || packet.sequence != activeLinePollSequence) {
        ++lineUnsolicitedPackets;
        return;
    }
    if (lineHasPacket && !linePeerResyncPending &&
        static_cast<uint32_t>(nowMs - lastLinePacketMs) > kLineFreshTimeoutMs) {
        lineSequence.resetPeer();
        linePeerResyncPending = true;
        ++linePeerRestartsInferred;
    }
    if (!lineSequence.accept(packet.sequence)) {
        ++lineRejectedPackets;
        return;
    }
    LineObservation decoded{};
    if (!decodeLinePacket(packet, decoded, board::kLineSensorAngleCdeg,
                          board::kLineAnglesConfigured ? board::kLineSensorAngleValidMask : 0,
                          board::kLineSensorEnabledMask)) {
        ++lineRejectedPackets;
        return;
    }
    linePacketPeriodMs = lineHasPacket ? static_cast<uint32_t>(nowMs - lastLinePacketMs) : 0;
    lineData = decoded;
    lineHasPacket = true;
    linePeerResyncPending = false;
    lastLinePacketMs = nowMs;
    finishLineCycle(nowMs);
}

void receiveSharedBus(uint32_t nowMs) {
    uint8_t byte = 0;
    uint8_t consumed = 0;
    while (consumed++ < 64 && rs485.receiveByte(byte, micros())) {
        const uint16_t irErrors = irParser.errors();
        const uint16_t irTimeouts = irParser.timeouts();
        uint16_t direction = 0, strength = 0;
        if (irParser.feed(byte, nowMs, direction, strength)) acceptIrResponse(direction, strength, nowMs);
        if (autoMode == kAutoIrTest && irTestController.armed() &&
            (irParser.errors() != irErrors || irParser.timeouts() != irTimeouts)) stopIrTest();

        Packet packet{};
        const ParserStats before = lineParser.stats();
        const bool ready = lineParser.feed(byte, nowMs, packet);
        const ParserStats& after = lineParser.stats();
        if (after.crcErrors != before.crcErrors || after.formatErrors != before.formatErrors ||
            after.timeouts != before.timeouts) ++lineRejectedPackets;
        if (ready) acceptLineResponse(packet, nowMs);
    }
    const uint16_t irErrors = irParser.errors();
    const uint16_t irTimeouts = irParser.timeouts();
    irParser.poll(nowMs);
    if (autoMode == kAutoIrTest && irTestController.armed() &&
        (irParser.errors() != irErrors || irParser.timeouts() != irTimeouts)) stopIrTest();
    if (lineParser.poll(nowMs)) ++lineRejectedPackets;
}

bool sendIrPoll(uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs) || !busArbiter.acquire(kIrRs485Peer)) return false;
    const uint8_t request = 0x01;
    if (!rs485.send(&request, 1, nowUs)) {
        busArbiter.complete(kIrRs485Peer);
        ++irTxErrors;
        if (autoMode == kAutoIrTest && irTestController.armed()) stopIrTest();
        return false;
    }
    ++irRequests;
    irPending = true;
    busRequestStartedUs = micros();
    return true;
}

bool sendLinePoll(uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs) || !busArbiter.acquire(kLineRs485Peer)) return false;
    const Packet request = makeLineRequest(linePollSequence);
    uint8_t bytes[kMaxFrame]{};
    const uint8_t length = encodePacket(request, bytes, sizeof(bytes));
    if (!length || !rs485.send(bytes, length, nowUs)) {
        busArbiter.complete(kLineRs485Peer);
        ++lineRejectedPackets;
        return false;
    }
    activeLinePollSequence = linePollSequence++;
    busRequestStartedUs = micros();
    return true;
}

bool sendMotorCommand(uint32_t nowMs, uint32_t nowUs) {
    if (!rs485.canTransmit(nowUs)) return false;
    int16_t values[kMotorCommandWheelCount]{};
    if (!motorStopPending && motorConsoleCommandSeen && !motorCommandExpired(nowMs, lastMotorConsoleCommandMs,
                                                         kMotorConsoleTimeoutMs)) {
        for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) values[i] = motorConsoleValues[i];
    } else if (!motorStopPending) {
        if (motorConsoleCommandSeen) {
            motorConsoleCommandSeen = false;
            for (int16_t& value : motorConsoleValues) value = 0;
        }
        if (!motorCommandValuesForMotion(motorMotion, motorEnabled, board::kMotorWheelConfig, values)) {
            ++motorTxErrors;
        } else if (motorEnabled) {
            const uint8_t pwmCap = autoMode == kAutoIrTest ? kIrTestPwmCap
                                    : autoMode == kAutoFloor ? kAutoFloorPwmCap : kAutoWheelLiftPwmCap;
            capMotorCommandValues(values, pwmCap);
        }
    }
    uint8_t bytes[kMotorCommandFrameSize]{};
    const uint8_t length = encodeMotorCommandFrame(values, bytes, sizeof(bytes));
    if (!length || !rs485.send(bytes, length, nowUs)) {
        ++motorTxErrors;
        return false;
    }
    ++motorFramesSent;
    lastMotorCommandMs = nowMs;
    motorCommandSent = true;
    motorStopPending = false;
    return true;
}

void pollMotorConsole(uint32_t nowMs) {
    // Any USB byte cancels the one-shot boot AUTO IR auto-arm immediately.
    // Previously this only happened when a complete line ended with '\n'.
    if (Serial.available() > 0) {
        irTestController.disableAutoStart();
        if (!bootAutoArmNoticeSent) {
            bootAutoArmNoticeSent = true;
            if (Serial && Serial.availableForWrite() >= 64)
                Serial.println("USB input received; boot AUTO IR auto-arm disabled");
        }
    }
    while (Serial.available()) {
        const int readValue = Serial.read();
        if (readValue < 0) break;
        const char ch = static_cast<char>(readValue);
        if (ch == '\r') continue;
        if (ch != '\n') {
            if (motorConsoleLineLength + 1 < kMotorConsoleLineCapacity) {
                motorConsoleLine[motorConsoleLineLength++] = ch;
            } else {
                motorConsoleOverflow = true;
            }
            continue;
        }
        motorConsoleLine[motorConsoleLineLength] = '\0';
        int16_t parsed[kMotorCommandWheelCount]{};
        const MainConsoleCommand command = motorConsoleOverflow
                                               ? kConsoleInvalid
                                               : parseMainConsoleCommand(motorConsoleLine, parsed);
        // Boot auto-arm was already cancelled by the first received byte above.
        if (command == kConsoleAutoWheelLift || command == kConsoleAutoFloor ||
            command == kConsoleAutoIrTest) {
            autoMode = autoModeForCommand(command, board::kAutoWheelLiftReady, board::kAutoFloorReady);
            irTestController.disarm();
            motorConsoleCommandSeen = false;
            motorMotion = {};
            motorEnabled = false;
            motorStopPending = true;
            for (int16_t& value : motorConsoleValues) value = 0;
            if (Serial && Serial.availableForWrite() >= 72) {
                if (autoMode == kAutoStopped) Serial.println("AUTO_BLOCKED; hardware configuration gate closed");
                else if (autoMode == kAutoIrTest) Serial.println("AUTO IR selected; stopped; send ARM IR with a fresh ball");
                else Serial.println(autoMode == kAutoFloor ? "AUTO FLOOR selected; physical ARM still required"
                                                           : "AUTO FLOAT selected; physical ARM still required");
            }
        } else if (command == kConsoleArmIrTest) {
            const bool irFresh = irPacketSeen &&
                                 static_cast<uint32_t>(nowMs - lastIrPacketMs) <= kIrFreshTimeoutMs;
            if (autoMode == kAutoIrTest &&
                irTestController.arm(irFresh, irBallFound, irDirection)) {
                if (Serial && Serial.availableForWrite() >= 72)
                    Serial.println("IR TEST armed; IR loss/fault or S stops");
            } else {
                autoMode = kAutoStopped;
                motorConsoleCommandSeen = false;
                for (int16_t& value : motorConsoleValues) value = 0;
                stopIrTest();
                if (Serial && Serial.availableForWrite() >= 64)
                    Serial.println("IR TEST blocked; select AUTO IR and wait for a fresh ball");
            }
        } else {
            autoMode = kAutoStopped;
            irTestController.disarm();
            if (command == kConsoleManualMotor || command == kConsoleStop) {
                for (uint8_t i = 0; i < kMotorCommandWheelCount; ++i) motorConsoleValues[i] = parsed[i];
            } else {
                for (int16_t& value : motorConsoleValues) value = 0;
                if (Serial && Serial.availableForWrite() >= 32) Serial.println("MOTOR_CMD_INVALID; stopped");
            }
            motorConsoleCommandSeen = true;
            lastMotorConsoleCommandMs = nowMs;
            if (command == kConsoleStop || command == kConsoleInvalid) motorStopPending = true;
        }
        motorConsoleLineLength = 0;
        motorConsoleOverflow = false;
    }
    if (motorConsoleCommandSeen && motorCommandExpired(nowMs, lastMotorConsoleCommandMs,
                                                        kMotorConsoleTimeoutMs)) {
        motorConsoleCommandSeen = false;
        for (int16_t& value : motorConsoleValues) value = 0;
    }
}

void pollSharedBus(uint32_t nowMs) {
    receiveSharedBus(nowMs);
    const uint32_t nowUs = micros();
    if (busArbiter.activePeer() == kIrRs485Peer &&
        static_cast<uint32_t>(nowUs - busRequestStartedUs) >= kIrReplyTimeoutUs) {
        busArbiter.complete(kIrRs485Peer);
        irPending = false;
        ++irResponseTimeouts;
        if (autoMode == kAutoIrTest && irTestController.armed()) stopIrTest();
        finishIrCycle(nowMs);
    } else if (busArbiter.activePeer() == kLineRs485Peer &&
               static_cast<uint32_t>(nowUs - busRequestStartedUs) >= kLineReplyTimeoutUs) {
        ++linePollTimeouts;
        finishLineCycle(nowMs);
    }

    if (busArbiter.idle() && autoMode == kAutoIrTest && busPhase == kBusNeedLine) {
        busPhase = kBusIdle;
        nextSensorPollMs = nowMs + kIrPollPeriodMs;
    }
    if (busArbiter.idle() && motorStopPending) {
        sendMotorCommand(nowMs, nowUs);
    } else if (busArbiter.idle() && busPhase == kBusNeedLine) {
        sendLinePoll(nowUs);
    } else if (busArbiter.idle() && busPhase == kBusIdle &&
               static_cast<int32_t>(nowMs - nextSensorPollMs) >= 0) {
        sendIrPoll(nowUs);
    } else if (busArbiter.idle() && busPhase == kBusIdle &&
               (!motorCommandSent || static_cast<uint32_t>(nowMs - lastMotorCommandMs) >= kMotorCommandPeriodMs)) {
        sendMotorCommand(nowMs, nowUs);
    }

    if (irPacketSeen && static_cast<uint32_t>(nowMs - lastIrPacketMs) > kIrFreshTimeoutMs) {
        irBallFound = false;
        irLinkStale = true;
    }
}

void printLineLevels() {
    Serial.print("line_raw[0..23]=");
    for (uint8_t i = 0; i < 24; ++i) Serial.print((lineData.rawLevels & (1u << i)) ? 'H' : 'L');
}

void setup() {
    Serial.begin(115200);
    if (Serial && Serial.availableForWrite() >= 48)
        Serial.println("BOOT MAIN; AUTO IR wait; diagnostics active");
    if (Serial && Serial.availableForWrite() >= 64)
        Serial.println("USB: AUTO [FLOAT] / AUTO FLOOR / AUTO IR; ARM IR; S=stop; M <v0> <v1> <v2> <v3>");
    if (Serial && Serial.availableForWrite() >= 64)
        Serial.println("MOTOR M range -255..255, 256=brake; M command timeout 300ms");
    if (Serial && Serial.availableForWrite() >= 72)
        Serial.println("IR TEST: no LINE; auto-arm; PWM 16; stop on IR loss/S; wheels up");

    pinMode(board::kMainRs485DePin, OUTPUT);
    digitalWrite(board::kMainRs485DePin, LOW);
    pinMode(board::kKickerPin, OUTPUT);
    digitalWrite(board::kKickerPin, LOW);
    if (mainSafetyInputsReady())
        for (int8_t pin : kMainSafetyPins) pinMode(pin, INPUT);
    Serial1.setTX(board::kMainRs485TxPin);
    Serial1.setRX(board::kMainRs485RxPin);
    Serial1.begin(115200);

    Wire.setSDA(board::kBnoSdaPin);
    Wire.setSCL(board::kBnoSclPin);
    Wire.begin();
    Wire.setClock(board::kBnoI2cClockHz);
}

void loop() {
    const uint32_t nowUs = micros();
    const uint32_t nowMs = millis();
    pollMotorConsole(nowMs);
    pollSharedBus(nowMs);
    pollHeading(nowMs);
    if (static_cast<uint32_t>(nowUs - lastControlUs) < kControlPeriodUs) return;

    const uint32_t startedUs = micros();
    if (lastControlUs && static_cast<uint32_t>(nowUs - lastControlUs) > 2 * kControlPeriodUs)
        ++controlOverruns;
    lastControlUs = nowUs;

    MainInput input{};
    const bool safetyInputsReady = mainSafetyInputsReady();
    const bool emergencyStopActive = !safetyInputsReady || mainSafetyInputActive(1);
    const bool armPressed = safetyInputsReady && mainSafetyInputActive(0);
    const bool clearFaultPressed = safetyInputsReady && mainSafetyInputActive(2);
    const bool armPermitted = operatorArm.update(safetyInputsReady, armPressed,
                                                  !safetyInputsReady || emergencyStopActive);
    if (previousControllerState == kFault) {
        input.clearFault = clearFaultAction.update(safetyInputsReady, clearFaultPressed,
                                                    !safetyInputsReady || emergencyStopActive);
    } else {
        clearFaultAction.reset();
    }
    input.nowMs = nowMs;
    const bool autoConfigured = autoMode == kAutoWheelLift ? board::kAutoWheelLiftReady
                               : autoMode == kAutoFloor ? board::kAutoFloorReady : false;
    input.armed = autoConfigured && armPermitted;
    input.fault = !safetyInputsReady;
    input.lineFresh = lineHasPacket && static_cast<uint32_t>(nowMs - lastLinePacketMs) <= kLineFreshTimeoutMs;
    input.line = lineData;
    input.irFresh = irPacketSeen && static_cast<uint32_t>(nowMs - lastIrPacketMs) <= kIrFreshTimeoutMs;
    input.ballFound = input.irFresh && irBallFound;
    input.ballAngleDeg = irDirection;
    input.relativeSignal = irRelativeStrength; // Relative sensor signal; not a physical distance.
    input.headingValid = headingSample.headingValid &&
                         static_cast<uint32_t>(nowMs - lastHeadingSampleMs) <= kHeadingFreshTimeoutMs;
    input.headingRequired = board::kAttackHeadingConfigured;
    input.headingDeg = headingSample.headingDeg;
    input.headingTargetDeg = board::kAttackHeadingConfigured ? board::kAttackHeadingDeg : NAN;
    input.emergencyStop = emergencyStopActive;

    const MainOutput output = controller.step(input);
    previousControllerState = output.state;

    KickerInput kickerInput{};
    kickerInput.commandFresh = input.irFresh && input.lineFresh && (!input.headingRequired || input.headingValid);
    kickerInput.armed = input.armed;
    kickerInput.emergencyStop = input.emergencyStop;
    kickerInput.fault = input.fault || output.state == kFault;
    kickerInput.clearFault = input.clearFault;
    kickerInput.hardwareConfigured = board::kKickerHardwareConfigured;
    kickerInput.ballFresh = input.irFresh;
    kickerInput.ballDetected = input.ballFound;
    kickerInput.headingValid = input.headingValid;
    kickerInput.headingAligned = board::kKickerHeadingWindowConfigured && board::kAttackHeadingConfigured &&
        isfinite(board::kKickerHeadingToleranceDeg) && board::kKickerHeadingToleranceDeg > 0.0f &&
        fabsf(shortestAngleError(input.headingDeg, board::kAttackHeadingDeg)) <= board::kKickerHeadingToleranceDeg;
    kickerInput.lineClear = input.lineFresh && !input.line.detected;
    // Kick timing comes from the IR bearing, not the unverified BNO055: the ball must be in front.
    kickerInput.ballAligned = input.ballFound && input.ballAngleDeg < 360 &&
        fabsf(shortestAngleError(static_cast<float>(input.ballAngleDeg), 0.0f)) <=
            board::kKickerBallAngleToleranceDeg;
    kickerInput.request = kickerRequestAllowed(board::kKickerRequestPolicyConfigured,
                                                output.state == kApproachBall, kickerInput);
    const KickerOutput kickerOutput = kickerController.update(kickerInput, nowMs,
                                                              board::kKickerPulseMs,
                                                              board::kKickerCooldownMs);
    digitalWrite(board::kKickerPin, kickerOutput.pulseActive() ? HIGH : LOW);

    if (output.state == kFault) operatorArm.reset();
    else clearFaultAction.reset();
    motorMotion = output.motion;
    motorEnabled = autoConfigured && input.armed && output.state != kFault;
    if (autoMode == kAutoIrTest) {
        // Bench-only exception: no LINE guard or MAIN E-stop; keep the MOTOR power cutoff reachable.
        irTestController.armOnFirstFreshBall(input.irFresh, input.ballFound, irDirection);
        const bool wasArmed = irTestController.armed();
        motorEnabled = irTestController.update(input.irFresh, input.ballFound, irDirection, motorMotion);
        if (wasArmed && !motorEnabled) motorStopPending = true;
    }

    const uint32_t elapsed = micros() - startedUs;
    if (elapsed > maxControlUs) maxControlUs = elapsed;
    if (elapsed > kControlPeriodUs) ++controlOverruns;
    if (static_cast<uint32_t>(nowMs - lastDebugMs) >= kDiagnosticPeriodMs) {
        lastDebugMs = nowMs;
        if (Serial && Serial.availableForWrite() >= 240) {
            if (diagnosticPhase == 0) {
                Serial.print("DIAG IR req/match/success%/timeouts/recovery="); Serial.print(irRequests); Serial.print('/');
                Serial.print(irMatchedResponses); Serial.print('/'); Serial.print(irSuccessPercent()); Serial.print('/');
                Serial.print(irResponseTimeouts); Serial.print('/'); Serial.print(irRecoveriesInferred);
                Serial.print(" latency_us(last/max)=");
                Serial.print(irLastLatencyUs); Serial.print('/'); Serial.print(irMaxLatencyUs);
                Serial.print(" dir="); Serial.print(irDirection); Serial.print(" relative="); Serial.print(irRelativeStrength);
                Serial.print(" bad/timeout/tx/unmatched="); Serial.print(irParser.errors()); Serial.print('/');
                Serial.print(irParser.timeouts()); Serial.print('/'); Serial.print(irTxErrors); Serial.print('/');
                Serial.println(irUnsolicitedResponses);
            } else if (diagnosticPhase == 1) {
                Serial.print("DIAG LINE age/period_ms=");
                Serial.print(lineHasPacket ? static_cast<uint32_t>(nowMs - lastLinePacketMs) : 0xFFFFFFFFu);
                Serial.print('/'); Serial.print(linePacketPeriodMs);
                Serial.print(" crc/timeout/seq_gap/reject/unmatched/poll_timeout/resync="); Serial.print(lineParser.stats().crcErrors);
                Serial.print('/'); Serial.print(lineParser.stats().timeouts); Serial.print('/');
                Serial.print(lineSequence.gaps()); Serial.print('/'); Serial.print(lineSequence.rejected());
                Serial.print('/'); Serial.print(lineRejectedPackets); Serial.print('/'); Serial.print(lineUnsolicitedPackets);
                Serial.print('/'); Serial.print(linePollTimeouts); Serial.print('/'); Serial.print(linePeerRestartsInferred);
                Serial.print(" count/angle/ambiguous="); Serial.print(lineData.count); Serial.print('/');
                Serial.print(lineData.angleCdeg); Serial.print('/'); Serial.print(lineData.ambiguous);
                Serial.print(" polarity="); Serial.println(lineData.polarityValid ? (lineData.activeHigh ? "HIGH" : "LOW") : "UNKNOWN");
                printLineLevels(); Serial.println();
            } else if (diagnosticPhase == 2) {
                Serial.print("DIAG BNO connected/valid/address="); Serial.print(headingSample.connected); Serial.print('/');
                Serial.print(headingSample.headingValid); Serial.print("/0x"); Serial.print(headingSample.address, HEX);
                Serial.print(" age_ms="); Serial.print(headingSample.connected ? 0 : static_cast<uint32_t>(nowMs - lastHeadingSampleMs));
                Serial.print(" cal(S/G/A/M)="); Serial.print(headingSample.calibration[0]); Serial.print('/');
                Serial.print(headingSample.calibration[1]); Serial.print('/'); Serial.print(headingSample.calibration[2]);
                Serial.print('/'); Serial.print(headingSample.calibration[3]);
                Serial.print(" heading_rel="); Serial.println(headingSample.headingDeg);
            } else {
                Serial.print("DIAG auto/state=");
                Serial.print(autoMode == kAutoIrTest ? "IR_TEST" :
                             autoMode == kAutoWheelLift ? "FLOAT" : autoMode == kAutoFloor ? "FLOOR" : "OFF");
                Serial.print('/');
                Serial.print(static_cast<uint8_t>(output.state)); Serial.print(" MOTOR_frames_no_ACK/errors=");
                Serial.print(motorFramesSent); Serial.print('/'); Serial.print(motorTxErrors);
                Serial.print(" IR_test_armed="); Serial.print(irTestController.armed());
                Serial.print(" bus_rx/tx/busy/arbiter_denied=");
                Serial.print(rs485.diagnostics().rxBytes); Serial.print('/'); Serial.print(rs485.diagnostics().txFrames);
                Serial.print('/'); Serial.print(rs485.diagnostics().txBusy); Serial.print('/');
                Serial.print(busArbiter.collisionsPrevented());
                Serial.print(" control_max_us/overrun="); Serial.print(maxControlUs); Serial.print('/');
                Serial.print(controlOverruns); Serial.print(" kicker_state/mock_pulse/event=");
                Serial.print(static_cast<uint8_t>(kickerOutput.state)); Serial.print('/');
                Serial.print(kickerOutput.mockPulseActive); Serial.print('/');
                Serial.println(static_cast<uint8_t>(kickerOutput.event));
            }
            diagnosticPhase = static_cast<uint8_t>((diagnosticPhase + 1) & 0x03u);
        }
    }
}
