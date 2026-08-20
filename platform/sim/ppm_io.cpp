#include "ppm_io.hpp"

#include <cstdio>

#include "ecv/vision/color.hpp"

namespace ecv::sim {
namespace {

/// Pula espaços em branco e comentários (# até fim de linha) do cabeçalho PPM.
int next_token_int(std::FILE* f, bool& ok) {
    int c = 0;
    do {
        c = std::fgetc(f);
        if (c == '#') {
            while (c != '\n' && c != EOF) c = std::fgetc(f);
        }
    } while (c == ' ' || c == '\t' || c == '\n' || c == '\r');

    if (c == EOF) {
        ok = false;
        return 0;
    }
    int value = 0;
    while (c >= '0' && c <= '9') {
        value = value * 10 + (c - '0');
        c = std::fgetc(f);
    }
    return value;
}

}  // namespace

bool read_ppm(const std::string& path, PpmImage& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;

    char magic[3] = {};
    if (std::fread(magic, 1, 2, f) != 2 || magic[0] != 'P' || magic[1] != '6') {
        std::fclose(f);
        return false;
    }

    bool ok = true;
    const int w = next_token_int(f, ok);
    const int h = next_token_int(f, ok);
    const int maxval = next_token_int(f, ok);
    if (!ok || w <= 0 || h <= 0 || maxval != 255) {
        std::fclose(f);
        return false;
    }

    out.width = w;
    out.height = h;
    out.pixels.resize(static_cast<size_t>(w) * h * 3);
    const size_t got = std::fread(out.pixels.data(), 1, out.pixels.size(), f);
    std::fclose(f);
    return got == out.pixels.size();
}

bool write_ppm(const std::string& path, const ImageView& img) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", img.width, img.height);

    std::vector<uint8_t> row(static_cast<size_t>(img.width) * 3);
    for (int32_t y = 0; y < img.height; ++y) {
        const uint8_t* src = img.row(y);
        for (int32_t x = 0; x < img.width; ++x) {
            const Rgb c = decode_pixel(src, x, img.format);
            row[x * 3 + 0] = c.r;
            row[x * 3 + 1] = c.g;
            row[x * 3 + 2] = c.b;
        }
        std::fwrite(row.data(), 1, row.size(), f);
    }
    std::fclose(f);
    return true;
}

bool write_mask_ppm(const std::string& path, const MaskView& mask) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    std::fprintf(f, "P6\n%d %d\n255\n", mask.width, mask.height);

    std::vector<uint8_t> row(static_cast<size_t>(mask.width) * 3);
    for (int32_t y = 0; y < mask.height; ++y) {
        const uint8_t* src = mask.row(y);
        for (int32_t x = 0; x < mask.width; ++x) {
            row[x * 3 + 0] = row[x * 3 + 1] = row[x * 3 + 2] = src[x];
        }
        std::fwrite(row.data(), 1, row.size(), f);
    }
    std::fclose(f);
    return true;
}

bool PpmSequenceSource::open() {
    index_ = 0;
    return !paths_.empty() && read_ppm(paths_[0], img_);
}

bool PpmSequenceSource::next(ImageView& out) {
    if (paths_.empty()) return false;
    if (index_ >= paths_.size()) {
        if (!loop_) return false;
        index_ = 0;
    }
    if (!read_ppm(paths_[index_++], img_)) return false;
    out = img_.view();
    return true;
}

}  // namespace ecv::sim
