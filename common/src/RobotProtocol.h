#pragma once

#include <stdint.h>

namespace soccer {

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kFrameStart0 = 0xA5;
constexpr uint8_t kFrameStart1 = 0x5A;
constexpr uint8_t kMaxPayload = 8;
constexpr uint8_t kMaxFrame = 2 + 4 + kMaxPayload + 2;
constexpr uint32_t kFrameTimeoutMs = 8;

enum MessageType : uint8_t { kLineStatus = 1, kMotorCommand = 2, kLineRequest = 3 };

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
    if ((packet.type != kLineStatus && packet.type != kMotorCommand && packet.type != kLineRequest) ||
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
        if (type != kLineStatus && type != kMotorCommand && type != kLineRequest) {
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

// The legacy IR request is a single 0x01 byte; skip framed bus traffic so 0x01 inside a frame is not a request.
class SharedBusIrRequestParser {
public:
    bool feed(uint8_t byte, uint32_t nowMs) {
        poll(nowMs);
        lastByteMs_ = nowMs;
        if (discard_) return false;
        if (used_ == 0) {
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
        if ((used_ || discard_) && static_cast<uint32_t>(nowMs - lastByteMs_) > kFrameTimeoutMs) {
            ++stats_.timeouts;
            reset();
            return true;
        }
        return false;
    }

    const ParserStats& stats() const { return stats_; }

private:
    void reset() { used_ = 0; expected_ = 0; discard_ = false; }
    uint8_t buffer_[kMaxFrame]{};
    uint8_t used_ = 0;
    uint8_t expected_ = 0;
    uint32_t lastByteMs_ = 0;
    bool discard_ = false;
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
