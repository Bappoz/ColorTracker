#include "ecv/core/types.hpp"

namespace ecv {
namespace {

int16_t clamp16(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return static_cast<int16_t>(v);
}

}  // namespace

Rect expand_and_clamp(const Rect& r, int16_t pad, int16_t frame_w, int16_t frame_h) {
    const int32_t x0 = clamp16(r.x - pad, 0, frame_w);
    const int32_t y0 = clamp16(r.y - pad, 0, frame_h);
    const int32_t x1 = clamp16(r.right() + pad, 0, frame_w);
    const int32_t y1 = clamp16(r.bottom() + pad, 0, frame_h);
    return Rect{static_cast<int16_t>(x0), static_cast<int16_t>(y0), static_cast<int16_t>(x1 - x0),
                static_cast<int16_t>(y1 - y0)};
}

ImageView crop(const ImageView& src, const Rect& r) {
    Rect c = r;
    c.x = clamp16(c.x, 0, src.width);
    c.y = clamp16(c.y, 0, src.height);
    c.w = clamp16(c.w, 0, src.width - c.x);
    c.h = clamp16(c.h, 0, src.height - c.y);
    // YUYV compartilha croma entre pares de pixels: começar em x ímpar trocaria U por V.
    if (src.format == PixelFormat::kYuyv && (c.x & 1)) {
        c.x = static_cast<int16_t>(c.x - 1);
        c.w = static_cast<int16_t>(c.w + 1);
    }

    ImageView out = src;
    out.width = c.w;
    out.height = c.h;
    out.data = src.row(c.y) + static_cast<int64_t>(c.x) * (bytes_per_pixel_x2(src.format) / 2);
    return out;
}

MaskView crop(const MaskView& src, const Rect& r) {
    Rect c = r;
    c.x = clamp16(c.x, 0, src.width);
    c.y = clamp16(c.y, 0, src.height);
    c.w = clamp16(c.w, 0, src.width - c.x);
    c.h = clamp16(c.h, 0, src.height - c.y);

    MaskView out = src;
    out.width = c.w;
    out.height = c.h;
    out.data = src.data + static_cast<int64_t>(c.y) * src.stride + c.x;
    return out;
}

}  // namespace ecv
