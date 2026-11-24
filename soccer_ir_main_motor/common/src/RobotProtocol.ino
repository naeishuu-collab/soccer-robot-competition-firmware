#pragma once

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

        // 暫定: LINEはヘッダ/長さで読み飛ばし（CRCは検査しない）。MOTOR側で完全性を担保するならCRC検査を追加。
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

// 従来のIR要求は1バイトの0x01。フレーム内の0x01を要求と誤認しないよう、フレーム形式の通信は読み飛ばす。
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
    // 通信途切れ後は相手側の連番基準をリセットする。累計診断値は維持する。
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
