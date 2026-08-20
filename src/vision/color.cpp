#include "ecv/vision/color.hpp"

namespace ecv {

void convert_to_hsv(const ImageView& src, uint8_t* dst, int32_t dst_stride) {
    for (int32_t y = 0; y < src.height; ++y) {
        const uint8_t* srow = src.row(y);
        uint8_t* drow = dst + static_cast<int64_t>(y) * dst_stride;
        for (int32_t x = 0; x < src.width; ++x) {
            const Hsv p = rgb_to_hsv(decode_pixel(srow, x, src.format));
            drow[x * 3 + 0] = p.h;
            drow[x * 3 + 1] = p.s;
            drow[x * 3 + 2] = p.v;
        }
    }
}

void convert_to_rgb888(const ImageView& src, uint8_t* dst, int32_t dst_stride) {
    for (int32_t y = 0; y < src.height; ++y) {
        const uint8_t* srow = src.row(y);
        uint8_t* drow = dst + static_cast<int64_t>(y) * dst_stride;
        for (int32_t x = 0; x < src.width; ++x) {
            const Rgb c = decode_pixel(srow, x, src.format);
            drow[x * 3 + 0] = c.r;
            drow[x * 3 + 1] = c.g;
            drow[x * 3 + 2] = c.b;
        }
    }
}

}  // namespace ecv
