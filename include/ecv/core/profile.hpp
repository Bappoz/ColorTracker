// Instrumentação de latência por estágio. Sem alocação, sem string em runtime.
#pragma once

#include <cstdint>

namespace ecv {

/// Relógio monotônico em microssegundos (esp_timer no ESP-IDF, steady_clock no host).
uint64_t micros();

enum Stage : uint8_t {
    kStageAcquire = 0,
    kStageBorder,
    kStageThreshold,
    kStageMorphology,
    kStageBlobs,
    kStageTrack,
    kStageControl,
    kStageCount,
};

struct StageTimings {
    uint32_t us[kStageCount] = {};
    uint32_t total_us = 0;

    uint32_t sum() const {
        uint32_t s = 0;
        for (uint32_t v : us) s += v;
        return s;
    }
};

const char* stage_name(Stage s);

/// Acumula em `out.us[stage]` o tempo de vida do objeto.
class ScopedStage {
public:
    ScopedStage(StageTimings& out, Stage s) : out_(out), stage_(s), t0_(micros()) {}
    ~ScopedStage() { out_.us[stage_] += static_cast<uint32_t>(micros() - t0_); }

    ScopedStage(const ScopedStage&) = delete;
    ScopedStage& operator=(const ScopedStage&) = delete;

private:
    StageTimings& out_;
    Stage stage_;
    uint64_t t0_;
};

/// Estatística incremental (média/máx) sem guardar histórico — para o loop do robô.
struct LatencyStats {
    uint64_t sum_us = 0;
    uint32_t max_us = 0;
    uint32_t count = 0;

    void add(uint32_t us) {
        sum_us += us;
        if (us > max_us) max_us = us;
        ++count;
    }
    float mean_us() const {
        return count ? static_cast<float>(sum_us) / static_cast<float>(count) : 0.0f;
    }
};

}  // namespace ecv
