#pragma once
#include <chrono>
#include <cstdint>

namespace hybrid_qt {
// One steady-clock epoch for camera capture, USB arrivals and output on all threads.
inline int64_t hostNowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
