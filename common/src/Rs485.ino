#pragma once

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
