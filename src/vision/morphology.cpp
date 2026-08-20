#include "ecv/vision/morphology.hpp"

namespace ecv {
namespace {

inline uint8_t min3(uint8_t a, uint8_t b, uint8_t c) {
    const uint8_t m = a < b ? a : b;
    return m < c ? m : c;
}
inline uint8_t max3(uint8_t a, uint8_t b, uint8_t c) {
    const uint8_t m = a > b ? a : b;
    return m > c ? m : c;
}

template <bool Erode>
void separable_3x3(MaskView& mask, uint8_t* scratch) {
    const int32_t w = mask.width;
    const int32_t h = mask.height;
    if (w <= 0 || h <= 0) return;

    // Passada horizontal, in-place: `left` guarda o valor original do vizinho.
    for (int32_t y = 0; y < h; ++y) {
        uint8_t* r = mask.row(y);
        uint8_t left = r[0];
        for (int32_t x = 0; x < w; ++x) {
            const uint8_t cur = r[x];
            const uint8_t right = (x + 1 < w) ? r[x + 1] : cur;
            r[x] = Erode ? min3(left, cur, right) : max3(left, cur, right);
            left = cur;
        }
    }

    // Passada vertical, in-place: `scratch` guarda a linha y-1 antes de ser
    // sobrescrita; a linha y+1 ainda está intacta.
    for (int32_t y = 0; y < h; ++y) {
        uint8_t* r = mask.row(y);
        const uint8_t* below = (y + 1 < h) ? mask.row(y + 1) : r;
        for (int32_t x = 0; x < w; ++x) {
            const uint8_t cur = r[x];
            const uint8_t above = (y == 0) ? cur : scratch[x];
            scratch[x] = cur;
            r[x] = Erode ? min3(above, cur, below[x]) : max3(above, cur, below[x]);
        }
    }
}

}  // namespace

void erode3(MaskView& mask, uint8_t* scratch) {
    separable_3x3<true>(mask, scratch);
}
void dilate3(MaskView& mask, uint8_t* scratch) {
    separable_3x3<false>(mask, scratch);
}

void open3(MaskView& mask, uint8_t* scratch, int iterations) {
    for (int i = 0; i < iterations; ++i) erode3(mask, scratch);
    for (int i = 0; i < iterations; ++i) dilate3(mask, scratch);
}

void close3(MaskView& mask, uint8_t* scratch, int iterations) {
    for (int i = 0; i < iterations; ++i) dilate3(mask, scratch);
    for (int i = 0; i < iterations; ++i) erode3(mask, scratch);
}

}  // namespace ecv
