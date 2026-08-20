// Pipeline completo do robô de sumô — a BPMN executada uma vez por frame.
//
// Mapeamento BPMN -> código em docs/PIPELINE.md. Os dois desvios conscientes
// estão anotados no ponto exato de `SumoVision::process`.
#pragma once

#include <cstddef>
#include <cstdint>

#include "ecv/control/differential.hpp"
#include "ecv/control/pd.hpp"
#include "ecv/core/profile.hpp"
#include "ecv/core/types.hpp"
#include "ecv/track/kalman.hpp"
#include "ecv/vision/blobs.hpp"
#include "ecv/vision/linescan.hpp"
#include "ecv/vision/roi.hpp"
#include "ecv/vision/threshold.hpp"

namespace ecv {

enum class TrackState : uint8_t {
    kSearching,  ///< sem alvo: varre o dohyo girando no lugar
    kTracking,   ///< alvo medido neste frame
    kCoasting,   ///< alvo perdido, seguindo a predição do Kalman
};

const char* track_state_name(TrackState s);

struct SumoConfig {
    int16_t frame_w = 320;
    int16_t frame_h = 240;

    HsvRange target;          ///< cor do oponente (calibrar com apps/calibrate)
    uint32_t min_area = 150;  ///< área mínima do contorno (BPMN Gateway_Area)
    int16_t roi_padding = 40;
    int16_t roi_min_side = 32;
    int iterations_open = 1;
    int iterations_close = 1;

    float coast_timeout_s = 0.4f;  ///< quanto tempo confiar só na predição
    float aim_lead_s = 0.05f;      ///< mira antecipada: onde o alvo estará
    float accel_var = 4000.0f;     ///< (px/s²)² — agilidade esperada do oponente
    float meas_var = 9.0f;         ///< (px)² — ruído do centroide

    PdController<float>::Config pd{};
    DifferentialConfig drive{};
    float attack_speed = 0.6f;
    float search_turn = 0.35f;
    /// Quanto o desalinhamento freia o avanço: 1.0 = só avança alinhado.
    float align_gain = 0.9f;

    /// Detecção da borda do ringue (linha branca). Desligada por padrão porque
    /// exige calibrar o branco do dohyo.
    bool border_enabled = false;
    LineScanConfig border{};
    float retreat_speed = 0.7f;
};

struct SumoResult {
    TrackState state = TrackState::kSearching;
    bool detected = false;
    Point target;  ///< coordenada global no frame
    Rect box;      ///< bounding box global
    uint32_t area = 0;
    float error_norm = 0.0f;  ///< [-1, 1]; >0 = oponente à direita do centro
    /// Leitura da linha branca do ringue. `zone_hits` diz de que lado ela está,
    /// que é o que decide para onde recuar.
    LineScanResult border;
    MotorCommand cmd;
    Rect roi;  ///< região realmente processada neste frame
    StageTimings timings;

    bool border_hit() const { return border.found; }
};

class SumoVision {
public:
    /// Toda a memória de trabalho vem de fora: o pipeline nunca chama malloc.
    struct Buffers {
        MaskView mask;                     ///< frame_w x frame_h, 1 B/px
        uint8_t* morph_scratch = nullptr;  ///< >= frame_w bytes
        BlobWorkspace blobs;
        Blob* blob_out = nullptr;
        int32_t max_blobs = 0;
        /// Opcional (8 KB). Presente = limiarização por tabela; ausente = HSV
        /// calculado por pixel. Só faz diferença de velocidade, não de resultado.
        ColorLut565* lut = nullptr;
    };

    SumoVision(const SumoConfig& cfg, const Buffers& buf);

    /// Uma iteração da BPMN. `dt` em segundos desde o frame anterior.
    SumoResult process(const ImageView& frame, float dt);

    void reset();
    const SumoConfig& config() const { return cfg_; }
    /// Reconstrói a LUT quando existe — trocar a cor do alvo em runtime custa
    /// os 65.536 cálculos de HSV, então não fazer isso dentro do loop.
    void set_target_range(const HsvRange& r);

    // Ajustes de bancada: mexer nestes durante um teste manual evita reiniciar o
    // programa (e perder a exposição já estabilizada da câmera).
    void set_min_area(uint32_t area) { cfg_.min_area = area; }
    void set_border_enabled(bool on) { cfg_.border_enabled = on; }
    void set_border_config(const LineScanConfig& c) { cfg_.border = c; }

private:
    SumoConfig cfg_;
    Buffers buf_;
    RoiTracker roi_;
    Kalman2D kalman_;
    PdController<float> pd_;
    DifferentialMixer mixer_;
    float lost_time_s_ = 0.0f;
};

/// Aloca estaticamente tudo que o pipeline precisa para uma resolução fixa.
/// Em 320x240: 76.800 B de máscara + ~4 KB de workspace. Cabe na PSRAM de um
/// ESP32-CAM e sobra folga na RAM interna de um RPi.
template <int W, int H, int MaxRunsPerRow = W / 4, int MaxLabels = 512, int MaxBlobs = 16>
struct SumoStorage {
    uint8_t mask[static_cast<size_t>(W) * H];
    uint8_t scratch[W];
    StaticBlobWorkspace<MaxRunsPerRow, MaxLabels> ws;
    Blob blobs[MaxBlobs];
    ColorLut565 lut;

    SumoVision::Buffers view() {
        SumoVision::Buffers b;
        b.mask = MaskView{mask, W, H, W};
        b.morph_scratch = scratch;
        b.blobs = ws.view();
        b.blob_out = blobs;
        b.max_blobs = MaxBlobs;
        b.lut = &lut;
        return b;
    }
};

}  // namespace ecv
