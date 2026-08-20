#include "ecv/vision/threshold.hpp"

namespace ecv {
namespace {

/// Corpo único das duas variantes: `Counting` some no branch em tempo de compilação.
template <bool Counting>
uint32_t threshold_impl(const ImageView& src, const HsvRange& range, MaskView& dst) {
    const int32_t w = src.width < dst.width ? src.width : dst.width;
    const int32_t h = src.height < dst.height ? src.height : dst.height;
    uint32_t hits = 0;

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* srow = src.row(y);
        uint8_t* drow = dst.row(y);
        for (int32_t x = 0; x < w; ++x) {
            const Hsv p = rgb_to_hsv(decode_pixel(srow, x, src.format));
            const bool hit = in_range(p, range);
            drow[x] = hit ? 255 : 0;
            if (Counting && hit) ++hits;
        }
    }
    return hits;
}

}  // namespace

void threshold_hsv(const ImageView& src, const HsvRange& range, MaskView& dst) {
    threshold_impl<false>(src, range, dst);
}

uint32_t threshold_hsv_count(const ImageView& src, const HsvRange& range, MaskView& dst) {
    return threshold_impl<true>(src, range, dst);
}

void ColorLut565::build(const HsvRange& range) {
    for (int32_t i = 0; i < 8192; ++i) bits_[i] = 0;

    for (uint32_t code = 0; code < 65536u; ++code) {
        const uint8_t r5 = static_cast<uint8_t>((code >> 11) & 0x1F);
        const uint8_t g6 = static_cast<uint8_t>((code >> 5) & 0x3F);
        const uint8_t b5 = static_cast<uint8_t>(code & 0x1F);
        const Rgb c{static_cast<uint8_t>((r5 << 3) | (r5 >> 2)),
                    static_cast<uint8_t>((g6 << 2) | (g6 >> 4)),
                    static_cast<uint8_t>((b5 << 3) | (b5 >> 2))};
        if (in_range(rgb_to_hsv(c), range))
            bits_[code >> 3] |= static_cast<uint8_t>(1u << (code & 7));
    }
    built_ = true;
}

void threshold_lut(const ImageView& src, const ColorLut565& lut, MaskView& dst) {
    const int32_t w = src.width < dst.width ? src.width : dst.width;
    const int32_t h = src.height < dst.height ? src.height : dst.height;

    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* srow = src.row(y);
        uint8_t* drow = dst.row(y);
        if (src.format == PixelFormat::kRgb565) {
            // Caminho nativo do OV2640: o pixel JÁ é o índice da tabela.
            for (int32_t x = 0; x < w; ++x) {
                const uint16_t px = static_cast<uint16_t>((srow[x * 2] << 8) | srow[x * 2 + 1]);
                drow[x] = lut.contains(px) ? 255 : 0;
            }
        } else {
            for (int32_t x = 0; x < w; ++x) {
                drow[x] = lut.contains(decode_pixel(srow, x, src.format)) ? 255 : 0;
            }
        }
    }
}

}  // namespace ecv
