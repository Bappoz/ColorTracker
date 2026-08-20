// Decodificação de pixel e conversão para HSV — tudo em inteiro.
//
// Convenção de faixa igual à do OpenCV (H em 0..179, S e V em 0..255) para que
// valores calibrados em ferramentas de desktop sirvam direto no firmware.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"

namespace ecv {

struct Rgb {
    uint8_t r = 0, g = 0, b = 0;
};

struct Hsv {
    uint8_t h = 0;  ///< 0..179
    uint8_t s = 0;  ///< 0..255
    uint8_t v = 0;  ///< 0..255
};

inline uint8_t clamp_u8(int32_t v) {
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

/// Converte YUV (BT.601, faixa TV) para RGB com aritmética inteira.
inline Rgb yuv_to_rgb(int32_t y, int32_t u, int32_t v) {
    const int32_t c = y - 16;
    const int32_t d = u - 128;
    const int32_t e = v - 128;
    return Rgb{clamp_u8((298 * c + 409 * e + 128) >> 8),
               clamp_u8((298 * c - 100 * d - 208 * e + 128) >> 8),
               clamp_u8((298 * c + 516 * d + 128) >> 8)};
}

/// Decodifica o pixel `x` de uma linha já posicionada. Inline e sem branch no
/// caso comum porque isto roda width*height vezes por frame.
inline Rgb decode_pixel(const uint8_t* row, int32_t x, PixelFormat fmt) {
    switch (fmt) {
        case PixelFormat::kBgr888: {
            const uint8_t* p = row + x * 3;
            return Rgb{p[2], p[1], p[0]};
        }
        case PixelFormat::kRgb888: {
            const uint8_t* p = row + x * 3;
            return Rgb{p[0], p[1], p[2]};
        }
        case PixelFormat::kRgb565: {
            const uint8_t* p = row + x * 2;
            const uint16_t px = static_cast<uint16_t>((p[0] << 8) | p[1]);  // big-endian (OV2640)
            const uint8_t r5 = static_cast<uint8_t>((px >> 11) & 0x1F);
            const uint8_t g6 = static_cast<uint8_t>((px >> 5) & 0x3F);
            const uint8_t b5 = static_cast<uint8_t>(px & 0x1F);
            // Replica os bits altos nos baixos: mantém 0->0 e 31->255.
            return Rgb{static_cast<uint8_t>((r5 << 3) | (r5 >> 2)),
                       static_cast<uint8_t>((g6 << 2) | (g6 >> 4)),
                       static_cast<uint8_t>((b5 << 3) | (b5 >> 2))};
        }
        case PixelFormat::kYuyv: {
            const uint8_t* pair = row + (x & ~1) * 2;  // início do par Y0 U Y1 V
            const uint8_t y = (x & 1) ? pair[2] : pair[0];
            return yuv_to_rgb(y, pair[1], pair[3]);
        }
    }
    return Rgb{};
}

/// RGB -> HSV inteiro. Erro máximo de 1 unidade em H comparado ao cálculo em float.
inline Hsv rgb_to_hsv(Rgb c) {
    const int32_t r = c.r, g = c.g, b = c.b;
    const int32_t max = r > g ? (r > b ? r : b) : (g > b ? g : b);
    const int32_t min = r < g ? (r < b ? r : b) : (g < b ? g : b);
    const int32_t delta = max - min;

    Hsv out;
    out.v = static_cast<uint8_t>(max);
    if (max == 0 || delta == 0) return out;  // preto ou cinza: H e S ficam 0

    out.s = static_cast<uint8_t>((255 * delta + max / 2) / max);

    int32_t h;
    if (max == r) {
        h = (30 * (g - b) + delta / 2) / delta;
    } else if (max == g) {
        h = 60 + (30 * (b - r) + delta / 2) / delta;
    } else {
        h = 120 + (30 * (r - g) + delta / 2) / delta;
    }
    if (h < 0) h += 180;
    if (h >= 180) h -= 180;
    out.h = static_cast<uint8_t>(h);
    return out;
}

/// Caminho de depuração: converte o frame inteiro para um buffer HSV (3 B/px).
/// Não usar no loop do robô — o pipeline funde conversão e limiarização.
void convert_to_hsv(const ImageView& src, uint8_t* dst, int32_t dst_stride);

/// Converte um frame para RGB888 empacotado (usado por dump de debug e testes).
void convert_to_rgb888(const ImageView& src, uint8_t* dst, int32_t dst_stride);

}  // namespace ecv
