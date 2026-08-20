// Fonte de frames sobre o driver espressif/esp32-camera (OV2640 / OV3660).
//
// RGB565 direto do sensor é o formato de escolha: é o único que a limiarização
// por LUT consome sem nenhuma conversão — o pixel É o índice da tabela. Pedir
// JPEG obrigaria a decodificar 76.800 pixels por frame antes de qualquer coisa.
#pragma once

#include "ecv/hal/frame_source.hpp"

namespace ecv::esp32 {

/// Pinagem padrão do módulo AI-Thinker ESP32-CAM. Trocar por placa.
struct CamPins {
    int pwdn = 32;
    int reset = -1;
    int xclk = 0;
    int sccb_sda = 26;
    int sccb_scl = 27;
    int d7 = 35, d6 = 34, d5 = 39, d4 = 36, d3 = 21, d2 = 19, d1 = 18, d0 = 5;
    int vsync = 25;
    int href = 23;
    int pclk = 22;
};

struct CamConfig {
    CamPins pins;
    int xclk_freq_hz = 20000000;
    /// QVGA (320x240) é o teto prático: em QQVGA o alvo fica pequeno demais
    /// para o limiar de área, e acima de QVGA o tempo por frame estoura.
    int32_t width = 320;
    int32_t height = 240;
    /// 2 buffers + GRAB_LATEST: o pipeline sempre pega o frame mais recente.
    /// Com 1 buffer o driver bloqueia enquanto a visão processa.
    int fb_count = 2;
    bool use_psram = true;

    /// Trava de exposição/ganho/AWB. Com automático ligado, o oponente entrando
    /// no quadro muda o brilho da cena inteira e a faixa HSV calibrada some.
    bool lock_exposure = true;
    int aec_value = 300;  ///< 0..1200, só usado quando lock_exposure
    int agc_gain = 4;     ///< 0..30
};

class Esp32CamSource : public FrameSource {
public:
    explicit Esp32CamSource(const CamConfig& cfg) : cfg_(cfg) {}
    ~Esp32CamSource() override { close(); }

    bool open() override;
    void close() override;
    bool next(ImageView& out) override;

    int32_t width() const override { return cfg_.width; }
    int32_t height() const override { return cfg_.height; }
    PixelFormat format() const override { return PixelFormat::kRgb565; }
    const char* name() const override { return "esp32-cam"; }

    int last_error() const { return last_error_; }

private:
    void apply_sensor_controls();

    CamConfig cfg_;
    void* pending_fb_ = nullptr;  ///< camera_fb_t* do frame anterior
    bool open_ = false;
    int last_error_ = 0;
};

}  // namespace ecv::esp32
