// Mede a latência de cada estágio na CPU atual.
//
// Serve para responder a única pergunta que importa no robô: cabe no orçamento
// de tempo entre dois frames? Rodar no alvo real (RPi, ESP32), não no laptop —
// aqui o número só vale como referência relativa entre estágios.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "ecv/app/sumo_vision.hpp"
#include "ecv/core/profile.hpp"
#include "ecv/vision/morphology.hpp"
#include "sim/scene_source.hpp"

using namespace ecv;

namespace {

struct Args {
    int32_t width = 320;
    int32_t height = 240;
    int32_t frames = 300;
    PixelFormat format = PixelFormat::kRgb565;
};

PixelFormat parse_format(const char* s) {
    if (std::strcmp(s, "rgb565") == 0) return PixelFormat::kRgb565;
    if (std::strcmp(s, "yuyv") == 0) return PixelFormat::kYuyv;
    if (std::strcmp(s, "bgr888") == 0) return PixelFormat::kBgr888;
    return PixelFormat::kRgb888;
}

HsvRange red_marker() {
    HsvRange r;
    r.h_min = 170;
    r.h_max = 10;
    r.s_min = 120;
    r.v_min = 60;
    return r;
}

void report(const char* label, uint64_t total_us, int32_t iterations) {
    const double per = static_cast<double>(total_us) / iterations;
    std::printf("  %-28s %8.1f us   %8.0f Hz\n", label, per, per > 0 ? 1e6 / per : 0.0);
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--width") == 0)
            args.width = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--height") == 0)
            args.height = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--frames") == 0)
            args.frames = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--format") == 0)
            args.format = parse_format(argv[++i]);
    }

    sim::SceneConfig scene;
    scene.width = args.width;
    scene.height = args.height;
    scene.format = args.format;
    scene.opponent_radius = static_cast<int16_t>(args.width / 12);
    sim::SyntheticSource src(scene);
    if (!src.open()) return 1;

    ImageView frame{};
    if (!src.next(frame)) return 1;

    std::vector<uint8_t> mask_buf(static_cast<size_t>(args.width) * args.height);
    std::vector<uint8_t> scratch(static_cast<size_t>(args.width));
    MaskView mask{mask_buf.data(), args.width, args.height, args.width};

    std::vector<Run> runs(static_cast<size_t>(args.width));
    std::vector<int16_t> parent(1024);
    std::vector<LabelStats> stats(1024);
    BlobWorkspace ws;
    ws.runs_prev = runs.data();
    ws.runs_cur = runs.data() + args.width / 2;
    ws.max_runs_per_row = args.width / 2;
    ws.parent = parent.data();
    ws.stats = stats.data();
    ws.max_labels = 1024;
    Blob blobs[16];

    const HsvRange range = red_marker();

    std::printf("ECV bench — %dx%d, %d frames, formato %d\n", args.width, args.height, args.frames,
                static_cast<int>(args.format));
    std::printf(
        "Footprint estático de SumoStorage<320,240>: %zu B "
        "(máscara %d B + LUT %zu B + workspace %zu B)\n",
        sizeof(SumoStorage<320, 240>), 320 * 240, sizeof(ColorLut565),
        sizeof(SumoStorage<320, 240>) - 320 * 240 - sizeof(ColorLut565));
    std::printf("\nEstágios isolados (frame cheio):\n");

    uint64_t t = micros();
    for (int32_t i = 0; i < args.frames; ++i) threshold_hsv(frame, range, mask);
    const uint64_t direct_us = micros() - t;
    report("convert+threshold (HSV)", direct_us, args.frames);

    static ColorLut565 lut;
    t = micros();
    lut.build(range);
    const uint64_t build_us = micros() - t;
    t = micros();
    for (int32_t i = 0; i < args.frames; ++i) threshold_lut(frame, lut, mask);
    const uint64_t lut_us = micros() - t;
    report("convert+threshold (LUT)", lut_us, args.frames);
    std::printf("  %-28s %8.1fx  (tabela montada em %llu us, uma vez)\n", "ganho da LUT",
                lut_us ? static_cast<double>(direct_us) / static_cast<double>(lut_us) : 0.0,
                static_cast<unsigned long long>(build_us));

    t = micros();
    for (int32_t i = 0; i < args.frames; ++i) open3(mask, scratch.data());
    report("morfologia (abertura)", micros() - t, args.frames);

    t = micros();
    for (int32_t i = 0; i < args.frames; ++i) find_blobs(mask, ws, 100, blobs, 16);
    report("blobs (CCL)", micros() - t, args.frames);

    // Pipeline completo: primeiro frame varre tudo, os seguintes usam a ROI.
    SumoConfig cfg;
    cfg.frame_w = static_cast<int16_t>(args.width);
    cfg.frame_h = static_cast<int16_t>(args.height);
    cfg.target = range;
    cfg.min_area = 150;

    SumoVision::Buffers buf;
    buf.mask = mask;
    buf.morph_scratch = scratch.data();
    buf.blobs = ws;
    buf.blob_out = blobs;
    buf.max_blobs = 16;
    buf.lut = &lut;
    SumoVision vision(cfg, buf);

    StageTimings acc;
    LatencyStats full_frame, tracked;
    for (int32_t i = 0; i < args.frames; ++i) {
        src.advance(1.0f / 60.0f);
        src.next(frame);
        const SumoResult r = vision.process(frame, 1.0f / 60.0f);
        for (int s = 0; s < kStageCount; ++s) acc.us[s] += r.timings.us[s];
        (r.roi.w >= args.width ? full_frame : tracked).add(r.timings.total_us);
    }

    std::printf("\nPipeline completo:\n");
    for (int s = 0; s < kStageCount; ++s) {
        if (acc.us[s] == 0) continue;
        report(stage_name(static_cast<Stage>(s)), acc.us[s], args.frames);
    }
    std::printf("\n  frames em varredura total : %u (média %.1f us)\n", full_frame.count,
                full_frame.mean_us());
    std::printf("  frames com ROI dinâmica   : %u (média %.1f us, pior caso %u us)\n",
                tracked.count, tracked.mean_us(), tracked.max_us);
    if (full_frame.count && tracked.count) {
        std::printf("  ganho da ROI              : %.1fx\n",
                    full_frame.mean_us() / tracked.mean_us());
    }
    return 0;
}
