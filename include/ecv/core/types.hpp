// Tipos base do ECV: views sem dono sobre memória alheia.
// Nenhuma estrutura aqui aloca — o chamador é dono de todos os buffers.
#pragma once

#include <cstdint>

namespace ecv {

struct Point {
    int16_t x = 0;
    int16_t y = 0;
};

struct Rect {
    int16_t x = 0;
    int16_t y = 0;
    int16_t w = 0;
    int16_t h = 0;

    constexpr int16_t right() const { return static_cast<int16_t>(x + w); }
    constexpr int16_t bottom() const { return static_cast<int16_t>(y + h); }
    constexpr bool empty() const { return w <= 0 || h <= 0; }
    constexpr Point center() const {
        return Point{static_cast<int16_t>(x + w / 2), static_cast<int16_t>(y + h / 2)};
    }
};

/// Expande `r` por `pad` em todas as direções e corta no retângulo (0,0,w,h).
Rect expand_and_clamp(const Rect& r, int16_t pad, int16_t frame_w, int16_t frame_h);

enum class PixelFormat : uint8_t {
    kBgr888 = 0,  ///< 3 bytes/px, ordem B,G,R (padrão OpenCV / PPM invertido)
    kRgb888,      ///< 3 bytes/px, ordem R,G,B (PPM P6)
    kRgb565,      ///< 2 bytes/px, big-endian na saída do OV2640 (ESP32-CAM)
    kYuyv,        ///< 4 bytes / 2 px, YUV 4:2:2 (formato mais comum em V4L2)
};

constexpr int32_t bytes_per_pixel_x2(PixelFormat f) {
    // *2 para representar o YUYV (2 bytes/px) sem fração.
    return f == PixelFormat::kBgr888 || f == PixelFormat::kRgb888 ? 6 : 4;
}

/// View somente-leitura sobre um frame já capturado. `stride` em bytes.
struct ImageView {
    const uint8_t* data = nullptr;
    int32_t width = 0;
    int32_t height = 0;
    int32_t stride = 0;
    PixelFormat format = PixelFormat::kBgr888;

    const uint8_t* row(int32_t y) const { return data + static_cast<int64_t>(y) * stride; }
    bool valid() const { return data != nullptr && width > 0 && height > 0; }
};

/// Máscara binária de 1 byte/px, valores 0 ou 255.
struct MaskView {
    uint8_t* data = nullptr;
    int32_t width = 0;
    int32_t height = 0;
    int32_t stride = 0;

    uint8_t* row(int32_t y) { return data + static_cast<int64_t>(y) * stride; }
    const uint8_t* row(int32_t y) const { return data + static_cast<int64_t>(y) * stride; }
    bool valid() const { return data != nullptr && width > 0 && height > 0; }
};

/// Recorte de ROI sem cópia: só aritmética de ponteiro + stride preservado.
/// É o que substitui o `crop` da BPMN — copiar 40 KB por frame seria o gargalo.
ImageView crop(const ImageView& src, const Rect& r);
MaskView crop(const MaskView& src, const Rect& r);

}  // namespace ecv
