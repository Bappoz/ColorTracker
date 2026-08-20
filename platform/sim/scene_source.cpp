#include "scene_source.hpp"

namespace ecv::sim {
namespace {

inline uint8_t add_noise(uint8_t base, int32_t n) {
    const int32_t v = static_cast<int32_t>(base) + n;
    return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

inline void rgb_to_yuv(Rgb c, uint8_t& y, uint8_t& u, uint8_t& v) {
    const int32_t r = c.r, g = c.g, b = c.b;
    y = clamp_u8(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
    u = clamp_u8(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
    v = clamp_u8(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
}

}  // namespace

int32_t stride_for(PixelFormat fmt, int32_t width) {
    return width * (bytes_per_pixel_x2(fmt) / 2);
}

void encode_frame(const uint8_t* rgb, int32_t w, int32_t h, PixelFormat fmt, uint8_t* out) {
    const int32_t stride = stride_for(fmt, w);
    for (int32_t y = 0; y < h; ++y) {
        const uint8_t* src = rgb + static_cast<int64_t>(y) * w * 3;
        uint8_t* dst = out + static_cast<int64_t>(y) * stride;
        switch (fmt) {
            case PixelFormat::kRgb888:
                for (int32_t i = 0; i < w * 3; ++i) dst[i] = src[i];
                break;
            case PixelFormat::kBgr888:
                for (int32_t x = 0; x < w; ++x) {
                    dst[x * 3 + 0] = src[x * 3 + 2];
                    dst[x * 3 + 1] = src[x * 3 + 1];
                    dst[x * 3 + 2] = src[x * 3 + 0];
                }
                break;
            case PixelFormat::kRgb565:
                for (int32_t x = 0; x < w; ++x) {
                    const uint16_t px = static_cast<uint16_t>(((src[x * 3 + 0] & 0xF8) << 8) |
                                                              ((src[x * 3 + 1] & 0xFC) << 3) |
                                                              (src[x * 3 + 2] >> 3));
                    dst[x * 2 + 0] = static_cast<uint8_t>(px >> 8);
                    dst[x * 2 + 1] = static_cast<uint8_t>(px & 0xFF);
                }
                break;
            case PixelFormat::kYuyv:
                for (int32_t x = 0; x + 1 < w; x += 2) {
                    uint8_t y0, u0, v0, y1, u1, v1;
                    rgb_to_yuv(Rgb{src[x * 3], src[x * 3 + 1], src[x * 3 + 2]}, y0, u0, v0);
                    rgb_to_yuv(Rgb{src[(x + 1) * 3], src[(x + 1) * 3 + 1], src[(x + 1) * 3 + 2]},
                               y1, u1, v1);
                    dst[x * 2 + 0] = y0;
                    dst[x * 2 + 1] = static_cast<uint8_t>((u0 + u1) / 2);
                    dst[x * 2 + 2] = y1;
                    dst[x * 2 + 3] = static_cast<uint8_t>((v0 + v1) / 2);
                }
                break;
        }
    }
}

SyntheticSource::SyntheticSource(const SceneConfig& cfg) : cfg_(cfg) {
    pos_ = Point{static_cast<int16_t>(cfg.width / 2), static_cast<int16_t>(cfg.height / 2)};
    rng_ = cfg.seed ? cfg.seed : 1u;
}

bool SyntheticSource::open() {
    rgb_.assign(static_cast<size_t>(cfg_.width) * cfg_.height * 3, 0);
    out_.assign(static_cast<size_t>(stride_for(cfg_.format, cfg_.width)) * cfg_.height, 0);
    return true;
}

void SyntheticSource::advance(float dt) {
    float x = static_cast<float>(pos_.x) + vx_ * dt;
    const float lo = static_cast<float>(cfg_.opponent_radius);
    const float hi = static_cast<float>(cfg_.width - cfg_.opponent_radius);
    if (x < lo) {
        x = lo;
        vx_ = -vx_;
    } else if (x > hi) {
        x = hi;
        vx_ = -vx_;
    }
    pos_.x = static_cast<int16_t>(x);
}

void SyntheticSource::render() {
    const int32_t w = cfg_.width;
    const int32_t h = cfg_.height;
    const int32_t r2 = static_cast<int32_t>(cfg_.opponent_radius) * cfg_.opponent_radius;

    for (int32_t y = 0; y < h; ++y) {
        uint8_t* row = rgb_.data() + static_cast<int64_t>(y) * w * 3;
        for (int32_t x = 0; x < w; ++x) {
            Rgb c = cfg_.background;

            if (!cfg_.white_band.empty() && x >= cfg_.white_band.x && x < cfg_.white_band.right() &&
                y >= cfg_.white_band.y && y < cfg_.white_band.bottom()) {
                c = cfg_.line;
            }
            if (cfg_.opponent_visible) {
                const int32_t dx = x - pos_.x;
                const int32_t dy = y - pos_.y;
                if (dx * dx + dy * dy <= r2) c = cfg_.opponent;
            }

            if (cfg_.noise) {
                // LCG de Numerical Recipes: barato e reprodutível entre plataformas.
                rng_ = rng_ * 1664525u + 1013904223u;
                const int32_t n =
                    static_cast<int32_t>((rng_ >> 16) % (2u * cfg_.noise + 1u)) - cfg_.noise;
                c.r = add_noise(c.r, n);
                c.g = add_noise(c.g, n);
                c.b = add_noise(c.b, n);
            }

            row[x * 3 + 0] = c.r;
            row[x * 3 + 1] = c.g;
            row[x * 3 + 2] = c.b;
        }
    }
    encode_frame(rgb_.data(), w, h, cfg_.format, out_.data());
}

bool SyntheticSource::next(ImageView& out) {
    if (out_.empty() && !open()) return false;
    render();
    out = ImageView{out_.data(), cfg_.width, cfg_.height, stride_for(cfg_.format, cfg_.width),
                    cfg_.format};
    return true;
}

}  // namespace ecv::sim
