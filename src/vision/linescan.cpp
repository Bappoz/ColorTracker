#include "ecv/vision/linescan.hpp"

namespace ecv {

LineScanResult scan_lines(const ImageView& frame, const LineScanConfig& cfg) {
    LineScanResult out;
    if (!frame.valid() || cfg.rows == 0) return out;

    int16_t y_top = cfg.y_top;
    int16_t y_bottom = cfg.y_bottom;
    if (y_bottom <= y_top) y_bottom = static_cast<int16_t>(frame.height - 1);
    if (y_top < 0) y_top = 0;
    if (y_bottom > frame.height - 1) y_bottom = static_cast<int16_t>(frame.height - 1);

    const int32_t span = y_bottom - y_top;
    const int32_t third = frame.width / 3;
    int64_t sum_x = 0;

    for (uint8_t i = 0; i < cfg.rows; ++i) {
        const int32_t y = cfg.rows == 1 ? y_top : y_top + span * i / (cfg.rows - 1);
        const uint8_t* row = frame.row(y);
        for (int32_t x = 0; x < frame.width; ++x) {
            if (!in_range(rgb_to_hsv(decode_pixel(row, x, frame.format)), cfg.target)) continue;
            ++out.hits;
            sum_x += x;
            const int32_t zone = x < third ? 0 : (x < 2 * third ? 1 : 2);
            ++out.zone_hits[zone];
        }
    }

    out.found = out.hits >= cfg.min_hits;
    if (out.hits > 0) out.center_x = static_cast<int16_t>(sum_x / out.hits);
    return out;
}

float line_error(const LineScanResult& r, int32_t frame_width) {
    if (!r.found || frame_width <= 0) return 0.0f;
    const float half = static_cast<float>(frame_width) * 0.5f;
    return (static_cast<float>(r.center_x) - half) / half;
}

}  // namespace ecv
