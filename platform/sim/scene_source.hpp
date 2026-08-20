// Gerador de cena sintética: um dohyo com oponente colorido, sem hardware.
//
// É o que permite testar e medir o pipeline inteiro num laptop, com verdade de
// campo exata (a posição do alvo é conhecida por construção) e em qualquer um
// dos formatos de pixel que as câmeras reais entregam.
#pragma once

#include <cstdint>
#include <vector>

#include "ecv/hal/frame_source.hpp"
#include "ecv/vision/color.hpp"

namespace ecv::sim {

struct SceneConfig {
    int32_t width = 320;
    int32_t height = 240;
    PixelFormat format = PixelFormat::kBgr888;

    Rgb background{28, 28, 32};  ///< superfície escura do dohyo
    Rgb opponent{205, 35, 35};   ///< marcador do oponente
    Rgb line{240, 240, 240};     ///< linha branca do ringue

    int16_t opponent_radius = 26;
    bool opponent_visible = true;
    Rect white_band{};  ///< faixa branca desenhada (vazia = sem borda)

    uint8_t noise = 6;  ///< amplitude do ruído uniforme por canal
    uint32_t seed = 0x1234567u;
};

/// Fonte de frames determinística. `advance` move o alvo; `set_opponent` fixa a
/// posição quando o teste precisa de controle total.
class SyntheticSource : public FrameSource {
public:
    explicit SyntheticSource(const SceneConfig& cfg);

    bool open() override;
    void close() override {}
    bool next(ImageView& out) override;

    int32_t width() const override { return cfg_.width; }
    int32_t height() const override { return cfg_.height; }
    PixelFormat format() const override { return cfg_.format; }
    const char* name() const override { return "synthetic"; }

    void set_opponent(Point p) { pos_ = p; }
    Point opponent() const { return pos_; }
    void set_visible(bool v) { cfg_.opponent_visible = v; }
    void set_white_band(const Rect& r) { cfg_.white_band = r; }

    /// Move o alvo em movimento harmônico horizontal, quicando nas bordas.
    void advance(float dt);

    SceneConfig& config() { return cfg_; }

private:
    void render();

    SceneConfig cfg_;
    Point pos_;
    float vx_ = 140.0f;  ///< px/s
    std::vector<uint8_t> rgb_;
    std::vector<uint8_t> out_;
    uint32_t rng_ = 1;
};

/// Codifica um buffer RGB888 no formato de pixel alvo.
void encode_frame(const uint8_t* rgb, int32_t w, int32_t h, PixelFormat fmt, uint8_t* out);

/// Bytes por linha de um frame no formato dado.
int32_t stride_for(PixelFormat fmt, int32_t width);

}  // namespace ecv::sim
