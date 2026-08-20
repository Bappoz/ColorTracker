#include "ecv/vision/blobs.hpp"

namespace ecv {
namespace {

int16_t find_root(int16_t* parent, int16_t l) {
    while (parent[l] != l) {
        parent[l] = parent[parent[l]];  // compressão de caminho pela metade
        l = parent[l];
    }
    return l;
}

void absorb(LabelStats& dst, const LabelStats& src) {
    dst.area += src.area;
    dst.sum_x += src.sum_x;
    dst.sum_y += src.sum_y;
    if (src.x0 < dst.x0) dst.x0 = src.x0;
    if (src.y0 < dst.y0) dst.y0 = src.y0;
    if (src.x1 > dst.x1) dst.x1 = src.x1;
    if (src.y1 > dst.y1) dst.y1 = src.y1;
}

/// Une duas raízes mantendo o índice menor como raiz (estável e determinístico).
int16_t merge(int16_t* parent, LabelStats* stats, int16_t a, int16_t b) {
    if (a == b) return a;
    if (b < a) {
        const int16_t t = a;
        a = b;
        b = t;
    }
    parent[b] = a;
    absorb(stats[a], stats[b]);
    stats[b] = LabelStats{};
    return a;
}

}  // namespace

int32_t find_blobs(const MaskView& mask, BlobWorkspace& ws, uint32_t min_area, Blob* out,
                   int32_t max_out) {
    if (!mask.valid() || !ws.valid() || out == nullptr || max_out <= 0) return 0;

    Run* prev = ws.runs_prev;
    Run* cur = ws.runs_cur;
    int32_t prev_count = 0;
    int32_t next_label = 1;  // 0 é reservado para "sem rótulo"

    ws.parent[0] = 0;

    for (int32_t y = 0; y < mask.height; ++y) {
        const uint8_t* row = mask.row(y);
        int32_t cur_count = 0;

        int32_t x = 0;
        while (x < mask.width && cur_count < ws.max_runs_per_row) {
            if (!row[x]) {
                ++x;
                continue;
            }
            const int32_t start = x;
            while (x < mask.width && row[x]) ++x;
            cur[cur_count].y = static_cast<int16_t>(y);
            cur[cur_count].x0 = static_cast<int16_t>(start);
            cur[cur_count].x1 = static_cast<int16_t>(x - 1);
            cur[cur_count].label = 0;
            ++cur_count;
        }

        // Casa cada run com os runs da linha anterior (conectividade 8: tocar
        // na diagonal conta, por isso o ±1 na comparação).
        int32_t p = 0;
        for (int32_t i = 0; i < cur_count; ++i) {
            Run& r = cur[i];
            while (p < prev_count && prev[p].x1 < r.x0 - 1) ++p;

            int16_t label = 0;
            for (int32_t q = p; q < prev_count && prev[q].x0 <= r.x1 + 1; ++q) {
                const int16_t root = find_root(ws.parent, prev[q].label);
                label = label == 0 ? root : merge(ws.parent, ws.stats, label, root);
            }

            if (label == 0) {
                if (next_label >= ws.max_labels) continue;  // degrada: run é descartado
                label = static_cast<int16_t>(next_label++);
                ws.parent[label] = label;
                ws.stats[label] = LabelStats{0, 0, 0, r.x0, r.y, r.x1, r.y};
            }
            r.label = label;

            const int32_t n = r.x1 - r.x0 + 1;
            LabelStats& s = ws.stats[label];
            s.area += static_cast<uint32_t>(n);
            s.sum_x += (r.x0 + r.x1) * n / 2;  // soma exata: (x0+x1)*n é sempre par
            s.sum_y += y * n;
            if (r.x0 < s.x0) s.x0 = r.x0;
            if (r.x1 > s.x1) s.x1 = r.x1;
            if (r.y < s.y0) s.y0 = r.y;
            if (r.y > s.y1) s.y1 = r.y;
        }

        Run* swap = prev;
        prev = cur;
        cur = swap;
        prev_count = cur_count;
    }

    int32_t count = 0;
    for (int32_t l = 1; l < next_label && count < max_out; ++l) {
        if (ws.parent[l] != l) continue;  // rótulo absorvido por outro
        const LabelStats& s = ws.stats[l];
        if (s.area < min_area) continue;

        Blob& b = out[count++];
        b.area = s.area;
        b.box = Rect{s.x0, s.y0, static_cast<int16_t>(s.x1 - s.x0 + 1),
                     static_cast<int16_t>(s.y1 - s.y0 + 1)};
        b.centroid = Point{static_cast<int16_t>(s.sum_x / static_cast<int32_t>(s.area)),
                           static_cast<int16_t>(s.sum_y / static_cast<int32_t>(s.area))};
    }
    return count;
}

int32_t largest_blob(const Blob* blobs, int32_t count) {
    int32_t best = -1;
    uint32_t best_area = 0;
    for (int32_t i = 0; i < count; ++i) {
        if (blobs[i].area > best_area) {
            best_area = blobs[i].area;
            best = i;
        }
    }
    return best;
}

}  // namespace ecv
