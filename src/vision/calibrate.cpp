#include "ecv/vision/calibrate.hpp"

namespace ecv {
namespace {

/// Menor arco circular do histograma de matiz que cobre `target` amostras.
/// Circular porque o vermelho — a cor mais usada em marcador de sumô — fica
/// exatamente na descontinuidade 179/0.
uint32_t hue_arc(const uint32_t hist[180], uint32_t target, uint8_t& out_min, uint8_t& out_max) {
    uint32_t best_len = 181;
    int best_start = 0;
    for (int start = 0; start < 180; ++start) {
        uint32_t acc = 0;
        for (int len = 1; len <= 180 && static_cast<uint32_t>(len) < best_len; ++len) {
            acc += hist[(start + len - 1) % 180];
            if (acc >= target) {
                best_len = static_cast<uint32_t>(len);
                best_start = start;
                break;
            }
        }
    }
    if (best_len > 180) best_len = 180;  // nenhuma amostra: arco degenerado
    out_min = static_cast<uint8_t>(best_start);
    out_max = static_cast<uint8_t>((best_start + best_len - 1) % 180);
    return best_len;
}

/// Percentil sem ordenar: soma acumulada do histograma de 256 posições.
uint8_t percentile(const uint32_t hist[256], uint32_t total, float p) {
    if (total == 0) return 0;
    const uint32_t target = static_cast<uint32_t>(p * static_cast<float>(total));
    uint32_t acc = 0;
    for (int i = 0; i < 256; ++i) {
        acc += hist[i];
        if (acc > target) return static_cast<uint8_t>(i);
    }
    return 255;
}

Rect clamp_roi(const ImageView& img, const Rect& r) {
    Rect c = r;
    if (c.x < 0) c.x = 0;
    if (c.y < 0) c.y = 0;
    if (c.x > img.width) c.x = static_cast<int16_t>(img.width);
    if (c.y > img.height) c.y = static_cast<int16_t>(img.height);
    if (c.x + c.w > img.width) c.w = static_cast<int16_t>(img.width - c.x);
    if (c.y + c.h > img.height) c.h = static_cast<int16_t>(img.height - c.y);
    return c;
}

}  // namespace

CalibrationResult estimate_hsv_range(const ImageView& img, const Rect& region,
                                     CalibrationWorkspace& ws, const CalibrationParams& params) {
    CalibrationResult out;
    if (!img.valid()) return out;

    const Rect roi = clamp_roi(img, region);
    if (roi.empty()) return out;

    for (int i = 0; i < 180; ++i) ws.hue[i] = 0;

    // Passada 1: histograma de matiz, só com pixels de cor definida.
    for (int32_t y = roi.y; y < roi.bottom(); ++y) {
        const uint8_t* row = img.row(y);
        for (int32_t x = roi.x; x < roi.right(); ++x) {
            const Hsv p = rgb_to_hsv(decode_pixel(row, x, img.format));
            ++out.samples;
            if (p.s < params.chroma_floor) continue;
            ++ws.hue[p.h];
            ++out.chromatic;
        }
    }
    if (out.samples == 0) return out;

    // Menos de um quarto do recorte com cor: cai para o modo degradado, contando
    // tudo, e avisa. Melhor um resultado ruim declarado que um silencioso.
    out.low_chroma = out.chromatic * 4 < out.samples;
    uint32_t hue_total = out.chromatic;
    if (out.low_chroma) {
        for (int i = 0; i < 180; ++i) ws.hue[i] = 0;
        for (int32_t y = roi.y; y < roi.bottom(); ++y) {
            const uint8_t* row = img.row(y);
            for (int32_t x = roi.x; x < roi.right(); ++x) {
                ++ws.hue[rgb_to_hsv(decode_pixel(row, x, img.format)).h];
            }
        }
        hue_total = out.samples;
    }

    out.hue_span =
        hue_arc(ws.hue, static_cast<uint32_t>(params.coverage * static_cast<float>(hue_total)),
                out.range.h_min, out.range.h_max);

    // Passada 2: S e V só dos pixels que caíram no arco.
    for (int i = 0; i < 256; ++i) ws.sat[i] = ws.val[i] = 0;

    HsvRange hue_only = out.range;
    hue_only.s_min = 0;
    hue_only.s_max = 255;
    hue_only.v_min = 0;
    hue_only.v_max = 255;

    uint32_t in_arc = 0;
    for (int32_t y = roi.y; y < roi.bottom(); ++y) {
        const uint8_t* row = img.row(y);
        for (int32_t x = roi.x; x < roi.right(); ++x) {
            const Hsv p = rgb_to_hsv(decode_pixel(row, x, img.format));
            if (!out.low_chroma && p.s < params.chroma_floor) continue;
            if (!in_range(p, hue_only)) continue;
            ++ws.sat[p.s];
            ++ws.val[p.v];
            ++in_arc;
        }
    }

    const float tail = (1.0f - params.coverage) * 0.5f;
    out.range.s_min = percentile(ws.sat, in_arc, tail);
    out.range.s_max = 255;
    out.range.v_min = percentile(ws.val, in_arc, tail);
    out.range.v_max = 255;

    if (out.hue_span + 2u * params.hue_margin < 180u) {
        out.range.h_min = static_cast<uint8_t>((out.range.h_min + 180 - params.hue_margin) % 180);
        out.range.h_max = static_cast<uint8_t>((out.range.h_max + params.hue_margin) % 180);
    }
    return out;
}

float false_positive_rate(const ImageView& img, const Rect& region, const HsvRange& range) {
    if (!img.valid()) return 0.0f;
    const Rect roi = clamp_roi(img, region);

    uint32_t outside = 0, total = 0;
    for (int32_t y = 0; y < img.height; ++y) {
        const uint8_t* row = img.row(y);
        const bool row_in_roi = y >= roi.y && y < roi.bottom();
        for (int32_t x = 0; x < img.width; ++x) {
            if (row_in_roi && x >= roi.x && x < roi.right()) continue;
            ++total;
            if (in_range(rgb_to_hsv(decode_pixel(row, x, img.format)), range)) ++outside;
        }
    }
    return total ? static_cast<float>(outside) / static_cast<float>(total) : 0.0f;
}

}  // namespace ecv
