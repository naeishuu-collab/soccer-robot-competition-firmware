#pragma once

#include <stdint.h>

enum OpenMvMode : uint8_t {
    kOpenMvWait, kOpenMvAvoid, kOpenMvTrackIr, kOpenMvAlign, kOpenMvAimed
};

constexpr uint8_t kOpenMvNavHeader = 0x20;
constexpr uint8_t kOpenMvNavFrameSize = 8;
constexpr uint8_t kOpenMvCalibratedProtocol = 0x20;
constexpr uint8_t kOpenMvBoundary = 0x01;

inline bool validOpenMvFrame(const uint8_t* frame) {
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < kOpenMvNavFrameSize - 1; ++i) checksum ^= frame[i];
    if (frame[0] != kOpenMvNavHeader || frame[7] != checksum) return false;
    for (uint8_t i = 1; i <= 3; ++i) {
        if (frame[i] > 100 && frame[i] < 156) return false;
    }
    const uint8_t mode = frame[5];
    const uint8_t boundary = mode == kOpenMvAvoid ? kOpenMvBoundary : 0;
    if (frame[6] != (kOpenMvCalibratedProtocol | boundary) || frame[4] != (mode == kOpenMvAimed))
        return false;
    if (mode == kOpenMvAvoid) return frame[3] == 0;
    if (mode == kOpenMvAlign) return frame[1] == 0 && frame[2] == 0;
    return mode <= kOpenMvAimed && frame[1] == 0 && frame[2] == 0 && frame[3] == 0;
}

enum OpenMvControl { kCameraStop, kCameraAvoid, kIrFollow, kCameraAim, kSearchGoal };

struct OpenMvDecision {
    OpenMvControl control;
    bool kick;
};

inline OpenMvDecision decideOpenMvControl(bool armed, bool cameraFresh, bool irFresh,
                                        OpenMvMode mode, bool irBallCloseAndFront, bool aimed) {
    if (!armed || !cameraFresh || !irFresh || mode == kOpenMvWait) return {kCameraStop, false};
    if (mode == kOpenMvAvoid) return {kCameraAvoid, false};
    if (!irBallCloseAndFront) return {kIrFollow, false};
    if (mode == kOpenMvTrackIr) return {kSearchGoal, false};
    return {kCameraAim, mode == kOpenMvAimed && aimed};
}
