#include "ecv/core/profile.hpp"

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#else
#include <chrono>
#endif

namespace ecv {

uint64_t micros() {
#if defined(ESP_PLATFORM)
    return static_cast<uint64_t>(esp_timer_get_time());
#else
    using clock = std::chrono::steady_clock;
    static const clock::time_point t0 = clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(clock::now() - t0).count());
#endif
}

const char* stage_name(Stage s) {
    switch (s) {
        case kStageAcquire: return "acquire";
        case kStageBorder: return "border";
        case kStageThreshold: return "threshold";
        case kStageMorphology: return "morphology";
        case kStageBlobs: return "blobs";
        case kStageTrack: return "track";
        case kStageControl: return "control";
        default: return "?";
    }
}

}  // namespace ecv
