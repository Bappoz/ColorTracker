// Soak do robô de sumô completo: roda a BPMN inteira por fases e mede tudo.
//
// O `ecv_bench` mede estágio isolado; aqui o interesse é o oposto — o robô
// inteiro, passando por busca, perseguição, oclusão, borda e recalibração,
// para responder: quantos Hz esta placa sustenta e qual é o pior frame.
// A saída em CSV/JSON é o insumo do relatório de desempenho.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ecv/app/sumo_vision.hpp"
#include "sim/log_motor_sink.hpp"
#include "sim/scene_source.hpp"

#if defined(ECV_HAS_SYSMETRICS)
#include "linux/sysmetrics.hpp"
#endif

using namespace ecv;

namespace {

// ---------------------------------------------------------------- cenário ---

enum PhaseId {
    kBusca = 0,     ///< dohyo vazio: o robô gira procurando
    kAquisicao,     ///< oponente entra em cena
    kPerseguicao,   ///< alvo em movimento, ROI travada
    kOclusao,       ///< alvo some: Kalman em coasting e depois desistência
    kReaquisicao,   ///< alvo volta: ROI reabre para o frame cheio
    kBorda,         ///< linha branca do ringue à vista: recuo
    kRecalibracao,  ///< troca da cor do alvo em runtime (reconstrói a LUT)
    kPhaseCount,
};

const char* phase_name(int p) {
    switch (p) {
        case kBusca: return "busca";
        case kAquisicao: return "aquisicao";
        case kPerseguicao: return "perseguicao";
        case kOclusao: return "oclusao";
        case kReaquisicao: return "reaquisicao";
        case kBorda: return "borda";
        case kRecalibracao: return "recalibracao";
        default: return "?";
    }
}

struct FrameRecord {
    uint16_t phase = 0;
    uint32_t total_us = 0;
    uint32_t stage_us[kStageCount] = {};
    uint8_t state = 0;
    uint8_t detected = 0;
    uint8_t border = 0;
    int16_t roi_w = 0;
    int16_t roi_h = 0;
    float err = 0.0f;
    float left = 0.0f;
    float right = 0.0f;
};

// ------------------------------------------------------------ estatística ---

/// Percentil por interpolação linear sobre a amostra já ordenada.
double percentile(const std::vector<uint32_t>& sorted, double p) {
    if (sorted.empty()) return 0.0;
    const double idx = p / 100.0 * static_cast<double>(sorted.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = std::min(lo + 1, sorted.size() - 1);
    const double frac = idx - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

struct Summary {
    uint32_t count = 0;
    uint32_t min_us = 0;
    uint32_t max_us = 0;
    double mean_us = 0.0;
    double stddev_us = 0.0;
    double p50 = 0.0, p90 = 0.0, p95 = 0.0, p99 = 0.0;
    double detect_rate = 0.0;
    uint32_t n_state[3] = {};  ///< searching, tracking, coasting
    uint32_t border_hits = 0;

    double fps_mean() const { return mean_us > 0 ? 1e6 / mean_us : 0.0; }
    double fps_worst() const { return max_us > 0 ? 1e6 / max_us : 0.0; }
};

Summary summarize(const std::vector<FrameRecord>& recs, int phase_filter) {
    std::vector<uint32_t> v;
    v.reserve(recs.size());
    uint32_t detected = 0;
    uint32_t n_state[3] = {};
    uint32_t border_hits = 0;
    for (const FrameRecord& r : recs) {
        if (phase_filter >= 0 && r.phase != phase_filter) continue;
        v.push_back(r.total_us);
        detected += r.detected;
        border_hits += r.border;
        if (r.state < 3) ++n_state[r.state];
    }
    Summary s;
    if (v.empty()) return s;
    for (int i = 0; i < 3; ++i) s.n_state[i] = n_state[i];
    s.border_hits = border_hits;
    std::sort(v.begin(), v.end());
    s.count = static_cast<uint32_t>(v.size());
    s.min_us = v.front();
    s.max_us = v.back();
    double sum = 0.0;
    for (uint32_t x : v) sum += x;
    s.mean_us = sum / static_cast<double>(v.size());
    double acc = 0.0;
    for (uint32_t x : v) {
        const double d = static_cast<double>(x) - s.mean_us;
        acc += d * d;
    }
    s.stddev_us = std::sqrt(acc / static_cast<double>(v.size()));
    s.p50 = percentile(v, 50);
    s.p90 = percentile(v, 90);
    s.p95 = percentile(v, 95);
    s.p99 = percentile(v, 99);
    s.detect_rate = 100.0 * detected / static_cast<double>(v.size());
    return s;
}

// ------------------------------------------------------------- argumentos ---

struct Args {
    int32_t width = 320;
    int32_t height = 240;
    int32_t per_phase = 300;
    PixelFormat format = PixelFormat::kRgb565;
    std::string csv_path;
    std::string json_path;
    bool sweep = false;
    bool quiet = false;
    // Modelo de energia — dois pontos da curva potência x utilização da placa.
    // Não é medição: são constantes de entrada, impressas junto do resultado.
    double p_idle_w = -1.0;
    double p_busy_w = -1.0;
};

PixelFormat parse_format(const char* s) {
    if (std::strcmp(s, "rgb565") == 0) return PixelFormat::kRgb565;
    if (std::strcmp(s, "yuyv") == 0) return PixelFormat::kYuyv;
    if (std::strcmp(s, "bgr888") == 0) return PixelFormat::kBgr888;
    return PixelFormat::kRgb888;
}

const char* format_name(PixelFormat f) {
    switch (f) {
        case PixelFormat::kRgb565: return "rgb565";
        case PixelFormat::kYuyv: return "yuyv";
        case PixelFormat::kBgr888: return "bgr888";
        default: return "rgb888";
    }
}

// -------------------------------------------------------------- execução ---

/// Buffers para a maior resolução suportada por esta build. Estático: todo o
/// custo de memória é conhecido em tempo de link, como no robô real.
constexpr int kMaxW = 640;
constexpr int kMaxH = 480;
SumoStorage<kMaxW, kMaxH> g_storage;

struct SysSample {
    double t_s = 0.0;
    double cpu_all = 0.0;
    double cpu_proc = 0.0;
    double temp_c = -1.0;
    double freq_mhz = -1.0;
    double rss_kb = -1.0;
};

struct RunOutput {
    std::vector<FrameRecord> frames;
    std::vector<SysSample> sys;
    uint64_t lut_build_us = 0;
    uint64_t wall_us = 0;
    double cpu_proc_avg = 0.0;
};

HsvRange red_marker() {
    HsvRange r;
    r.h_min = 170;
    r.h_max = 10;
    r.s_min = 120;
    r.v_min = 60;
    return r;
}

/// Segundo alvo usado na fase de recalibração: um oponente azul.
HsvRange blue_marker() {
    HsvRange r;
    r.h_min = 100;
    r.h_max = 130;
    r.s_min = 120;
    r.v_min = 60;
    return r;
}

RunOutput run_scenario(const Args& args, int32_t w, int32_t h) {
    RunOutput out;
    out.frames.reserve(static_cast<size_t>(args.per_phase) * kPhaseCount);

    sim::SceneConfig scene;
    scene.width = w;
    scene.height = h;
    scene.format = args.format;
    scene.opponent_radius = static_cast<int16_t>(w / 12);
    sim::SyntheticSource src(scene);
    if (!src.open()) return out;

    SumoConfig cfg;
    cfg.frame_w = static_cast<int16_t>(w);
    cfg.frame_h = static_cast<int16_t>(h);
    cfg.target = red_marker();
    cfg.min_area = 150;
    cfg.pd.kp = 0.9f;
    cfg.pd.kd = 0.06f;
    cfg.attack_speed = 0.6f;
    cfg.drive.deadband = 0.2f;
    // Branco do dohyo: saturação baixa, brilho alto — vale para qualquer matiz.
    cfg.border.target.s_max = 60;
    cfg.border.target.v_min = 200;
    cfg.border.y_top = static_cast<int16_t>(h * 3 / 4);
    cfg.border.y_bottom = static_cast<int16_t>(h - 1);
    cfg.border.rows = 4;
    cfg.border.min_hits = 12;

    SumoVision::Buffers buf = g_storage.view();
    buf.mask.width = w;
    buf.mask.height = h;
    buf.mask.stride = w;
    SumoVision vision(cfg, buf);
    sim::LogMotorSink motors;
    motors.open();

    ImageView frame{};
    const float dt = 1.0f / 60.0f;

#if defined(ECV_HAS_SYSMETRICS)
    linux_platform::SysMonitor mon;
    mon.open();
#endif

    const uint64_t t_start = micros();
    uint64_t next_sys_us = t_start;

    for (int phase = 0; phase < kPhaseCount; ++phase) {
        // Cada fase reconfigura a cena e o pipeline antes de rodar seus frames.
        switch (phase) {
            case kBusca:
                src.set_visible(false);
                src.set_white_band(Rect{});
                vision.set_border_enabled(false);
                vision.reset();
                break;
            case kAquisicao:
            case kPerseguicao:
            case kReaquisicao: src.set_visible(true); break;
            case kOclusao: src.set_visible(false); break;
            case kBorda: {
                src.set_visible(true);
                // Faixa branca no quarto inferior: é o que o linescan varre.
                src.set_white_band(Rect{0, static_cast<int16_t>(h * 7 / 8), static_cast<int16_t>(w),
                                        static_cast<int16_t>(h / 8)});
                vision.set_border_enabled(true);
                break;
            }
            case kRecalibracao: {
                // O oponente vira azul junto com a faixa HSV: a fase só tem
                // valor se provar a re-aquisição, não apenas a perda do alvo.
                src.config().opponent = Rgb{35, 60, 205};
                const uint64_t t0 = micros();
                vision.set_target_range(blue_marker());
                out.lut_build_us = micros() - t0;
                break;
            }
            default: break;
        }

        for (int i = 0; i < args.per_phase; ++i) {
            src.advance(dt);
            if (!src.next(frame)) break;
            const SumoResult r = vision.process(frame, dt);
            motors.write(r.cmd);

            FrameRecord rec;
            rec.phase = static_cast<uint16_t>(phase);
            rec.total_us = r.timings.total_us;
            for (int s = 0; s < kStageCount; ++s) rec.stage_us[s] = r.timings.us[s];
            rec.state = static_cast<uint8_t>(r.state);
            rec.detected = r.detected ? 1u : 0u;
            rec.border = r.border_hit() ? 1u : 0u;
            rec.roi_w = r.roi.w;
            rec.roi_h = r.roi.h;
            rec.err = r.error_norm;
            rec.left = r.cmd.left;
            rec.right = r.cmd.right;
            out.frames.push_back(rec);

#if defined(ECV_HAS_SYSMETRICS)
            const uint64_t now = micros();
            if (now >= next_sys_us) {  // ~20 Hz de amostragem do sistema
                const linux_platform::SysSnapshot s = mon.sample();
                SysSample ss;
                ss.t_s = static_cast<double>(now - t_start) / 1e6;
                ss.cpu_all = s.cpu_all_pct;
                ss.cpu_proc = s.cpu_proc_pct;
                ss.temp_c = s.temp_milli_c >= 0 ? s.temp_milli_c / 1000.0 : -1.0;
                ss.freq_mhz = s.freq_khz >= 0 ? s.freq_khz / 1000.0 : -1.0;
                ss.rss_kb = static_cast<double>(s.rss_kb);
                out.sys.push_back(ss);
                next_sys_us = now + 50000;
            }
#endif
        }
    }

    motors.stop();
    out.wall_us = micros() - t_start;
    double acc = 0.0;
    int n = 0;
    for (const SysSample& s : out.sys) {
        if (s.cpu_proc > 0.0) {
            acc += s.cpu_proc;
            ++n;
        }
    }
    out.cpu_proc_avg = n ? acc / n : 0.0;
    return out;
}

// ---------------------------------------------------------------- saídas ---

void write_csv(const std::string& path, const RunOutput& run) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::fprintf(stderr, "não consegui escrever %s\n", path.c_str());
        return;
    }
    std::fprintf(f, "frame,phase,state,detected,total_us");
    for (int s = 0; s < kStageCount; ++s) {
        std::fprintf(f, ",%s_us", stage_name(static_cast<Stage>(s)));
    }
    std::fprintf(f, ",roi_w,roi_h,border,err,left,right\n");
    for (size_t i = 0; i < run.frames.size(); ++i) {
        const FrameRecord& r = run.frames[i];
        std::fprintf(f, "%zu,%s,%s,%u,%u", i, phase_name(r.phase),
                     track_state_name(static_cast<TrackState>(r.state)), r.detected, r.total_us);
        for (int s = 0; s < kStageCount; ++s) std::fprintf(f, ",%u", r.stage_us[s]);
        std::fprintf(f, ",%d,%d,%u,%.4f,%.3f,%.3f\n", r.roi_w, r.roi_h, r.border,
                     static_cast<double>(r.err), static_cast<double>(r.left),
                     static_cast<double>(r.right));
    }
    std::fclose(f);
}

struct SweepPoint {
    int32_t w = 0, h = 0;
    Summary s;
    double cpu_proc = 0.0;
};

void write_json(const std::string& path, const Args& args, const RunOutput& run,
                const std::vector<SweepPoint>& sweep, const char* model, int32_t ncpu) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::fprintf(stderr, "não consegui escrever %s\n", path.c_str());
        return;
    }
    const Summary all = summarize(run.frames, -1);
    std::fprintf(f, "{\n");
    std::fprintf(f, "  \"host\": {\"model\": \"%s\", \"ncpu\": %d},\n", model, ncpu);
    std::fprintf(f,
                 "  \"config\": {\"width\": %d, \"height\": %d, \"format\": \"%s\", "
                 "\"frames_per_phase\": %d, \"total_frames\": %zu},\n",
                 args.width, args.height, format_name(args.format), args.per_phase,
                 run.frames.size());
    std::fprintf(f, "  \"footprint_bytes\": %zu,\n", sizeof(SumoStorage<320, 240>));
    std::fprintf(f, "  \"lut_build_us\": %llu,\n",
                 static_cast<unsigned long long>(run.lut_build_us));
    std::fprintf(f, "  \"wall_s\": %.3f,\n", run.wall_us / 1e6);
    std::fprintf(f, "  \"cpu_proc_pct\": %.2f,\n", run.cpu_proc_avg);
    std::fprintf(f,
                 "  \"overall\": {\"count\": %u, \"min_us\": %u, \"max_us\": %u, "
                 "\"mean_us\": %.2f, \"stddev_us\": %.2f, \"p50\": %.2f, \"p90\": %.2f, "
                 "\"p95\": %.2f, \"p99\": %.2f, \"fps_mean\": %.1f, \"fps_worst\": %.1f},\n",
                 all.count, all.min_us, all.max_us, all.mean_us, all.stddev_us, all.p50, all.p90,
                 all.p95, all.p99, all.fps_mean(), all.fps_worst());

    std::fprintf(f, "  \"stages\": [");
    double stage_sum = 0.0;
    double stage_mean[kStageCount] = {};
    uint32_t stage_max[kStageCount] = {};
    for (const FrameRecord& r : run.frames) {
        for (int s = 0; s < kStageCount; ++s) {
            stage_mean[s] += r.stage_us[s];
            if (r.stage_us[s] > stage_max[s]) stage_max[s] = r.stage_us[s];
        }
    }
    for (int s = 0; s < kStageCount; ++s) {
        stage_mean[s] /= static_cast<double>(run.frames.size() ? run.frames.size() : 1);
        stage_sum += stage_mean[s];
    }
    for (int s = 0; s < kStageCount; ++s) {
        std::fprintf(f,
                     "%s\n    {\"name\": \"%s\", \"mean_us\": %.2f, \"max_us\": %u, "
                     "\"share_pct\": %.1f}",
                     s ? "," : "", stage_name(static_cast<Stage>(s)), stage_mean[s], stage_max[s],
                     stage_sum > 0 ? 100.0 * stage_mean[s] / stage_sum : 0.0);
    }
    std::fprintf(f, "\n  ],\n");

    std::fprintf(f, "  \"phases\": [");
    for (int p = 0; p < kPhaseCount; ++p) {
        const Summary s = summarize(run.frames, p);
        std::fprintf(f,
                     "%s\n    {\"name\": \"%s\", \"count\": %u, \"min_us\": %u, \"max_us\": %u, "
                     "\"mean_us\": %.2f, \"p95\": %.2f, \"detect_pct\": %.1f, \"fps_mean\": %.1f, "
                     "\"searching\": %u, \"tracking\": %u, \"coasting\": %u, \"border_hits\": %u}",
                     p ? "," : "", phase_name(p), s.count, s.min_us, s.max_us, s.mean_us, s.p95,
                     s.detect_rate, s.fps_mean(), s.n_state[0], s.n_state[1], s.n_state[2],
                     s.border_hits);
    }
    std::fprintf(f, "\n  ],\n");

    std::fprintf(f, "  \"sys\": [");
    for (size_t i = 0; i < run.sys.size(); ++i) {
        const SysSample& s = run.sys[i];
        std::fprintf(f,
                     "%s\n    {\"t_s\": %.3f, \"cpu_all\": %.2f, \"cpu_proc\": %.2f, "
                     "\"temp_c\": %.1f, \"freq_mhz\": %.1f, \"rss_kb\": %.0f}",
                     i ? "," : "", s.t_s, s.cpu_all, s.cpu_proc, s.temp_c, s.freq_mhz, s.rss_kb);
    }
    std::fprintf(f, "\n  ],\n");

    std::fprintf(f, "  \"sweep\": [");
    for (size_t i = 0; i < sweep.size(); ++i) {
        const SweepPoint& p = sweep[i];
        std::fprintf(f,
                     "%s\n    {\"width\": %d, \"height\": %d, \"pixels\": %d, \"mean_us\": %.2f, "
                     "\"p95_us\": %.2f, \"max_us\": %u, \"fps_mean\": %.1f, \"fps_worst\": %.1f, "
                     "\"cpu_proc\": %.2f}",
                     i ? "," : "", p.w, p.h, p.w * p.h, p.s.mean_us, p.s.p95, p.s.max_us,
                     p.s.fps_mean(), p.s.fps_worst(), p.cpu_proc);
    }
    std::fprintf(f, "\n  ]");

    if (args.p_idle_w > 0.0 && args.p_busy_w > 0.0) {
        // Interpolação linear entre dois pontos declarados pelo usuário.
        const double util = run.cpu_proc_avg / 100.0 / (ncpu > 0 ? ncpu : 1);
        const double watts = args.p_idle_w + (args.p_busy_w - args.p_idle_w) * util;
        std::fprintf(f,
                     ",\n  \"energy_model\": {\"p_idle_w\": %.2f, \"p_busy_w\": %.2f, "
                     "\"util\": %.4f, \"watts\": %.2f, \"joules\": %.1f, \"mwh_per_min\": %.1f}",
                     args.p_idle_w, args.p_busy_w, util, watts, watts * run.wall_us / 1e6,
                     watts * 1000.0 / 60.0);
    }
    std::fprintf(f, "\n}\n");
    std::fclose(f);
}

void usage(const char* prog) {
    std::printf(
        "uso: %s [opções]\n"
        "  --width N --height N   resolução do cenário (padrão 320x240)\n"
        "  --frames N             frames por fase (padrão 300; 7 fases)\n"
        "  --format F             rgb565|yuyv|bgr888|rgb888\n"
        "  --sweep                repete o cenário em 160x120..640x480\n"
        "  --csv ARQ              grava uma linha por frame\n"
        "  --json ARQ             grava o resumo (insumo do relatório)\n"
        "  --power-idle W --power-busy W   modelo de energia (constantes suas)\n"
        "  --quiet                só o resumo\n",
        prog);
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_next = i + 1 < argc;
        if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else if (a == "--width" && has_next) {
            args.width = std::atoi(argv[++i]);
        } else if (a == "--height" && has_next) {
            args.height = std::atoi(argv[++i]);
        } else if (a == "--frames" && has_next) {
            args.per_phase = std::atoi(argv[++i]);
        } else if (a == "--format" && has_next) {
            args.format = parse_format(argv[++i]);
        } else if (a == "--csv" && has_next) {
            args.csv_path = argv[++i];
        } else if (a == "--json" && has_next) {
            args.json_path = argv[++i];
        } else if (a == "--power-idle" && has_next) {
            args.p_idle_w = std::atof(argv[++i]);
        } else if (a == "--power-busy" && has_next) {
            args.p_busy_w = std::atof(argv[++i]);
        } else if (a == "--sweep") {
            args.sweep = true;
        } else if (a == "--quiet") {
            args.quiet = true;
        } else {
            std::fprintf(stderr, "opção desconhecida: %s\n", a.c_str());
            usage(argv[0]);
            return 2;
        }
    }
    if (args.width > kMaxW || args.height > kMaxH) {
        std::fprintf(stderr, "esta build reserva buffers para %dx%d\n", kMaxW, kMaxH);
        return 1;
    }

    const char* model = "desconhecido";
    int32_t ncpu = 0;
#if defined(ECV_HAS_SYSMETRICS)
    linux_platform::SysMonitor probe;
    probe.open();
    model = probe.model()[0] ? probe.model() : "desconhecido";
    ncpu = probe.ncpu();
#endif

    std::printf("ECV soak — robô de sumô completo, %d frames por fase, %d fases\n", args.per_phase,
                kPhaseCount);
    std::printf("alvo: %s · %d núcleos · cenário %dx%d %s\n", model, ncpu, args.width, args.height,
                format_name(args.format));
    std::printf("footprint estático SumoStorage<320,240>: %zu B\n\n",
                sizeof(SumoStorage<320, 240>));

    const RunOutput run = run_scenario(args, args.width, args.height);
    if (run.frames.empty()) {
        std::fprintf(stderr, "cenário não produziu frames\n");
        return 1;
    }

    std::printf("Por fase (latência do frame inteiro):\n");
    std::printf("  %-14s %6s %8s %8s %8s %8s %7s  %s\n", "fase", "frames", "min us", "média", "p95",
                "max us", "detec%", "busca/trava/coast borda");
    for (int p = 0; p < kPhaseCount; ++p) {
        const Summary s = summarize(run.frames, p);
        if (!s.count) continue;
        std::printf("  %-14s %6u %8u %8.1f %8.1f %8u %6.1f%%  %5u/%5u/%5u %5u\n", phase_name(p),
                    s.count, s.min_us, s.mean_us, s.p95, s.max_us, s.detect_rate, s.n_state[0],
                    s.n_state[1], s.n_state[2], s.border_hits);
    }

    const Summary all = summarize(run.frames, -1);
    std::printf("\nAgregado (%u frames):\n", all.count);
    std::printf("  mínimo   %8u us   →  %7.1f Hz  (pico de melhor caso)\n", all.min_us,
                all.min_us ? 1e6 / all.min_us : 0.0);
    std::printf("  média    %8.1f us   →  %7.1f Hz  (capacidade sustentada)\n", all.mean_us,
                all.fps_mean());
    std::printf("  p95      %8.1f us   →  %7.1f Hz\n", all.p95, all.p95 > 0 ? 1e6 / all.p95 : 0.0);
    std::printf("  p99      %8.1f us   →  %7.1f Hz\n", all.p99, all.p99 > 0 ? 1e6 / all.p99 : 0.0);
    std::printf("  máximo   %8u us   →  %7.1f Hz  (pior caso garantido)\n", all.max_us,
                all.fps_worst());
    std::printf("  desvio   %8.1f us   (jitter)\n", all.stddev_us);
    std::printf("  LUT reconstruída em %llu us (fase de recalibração)\n",
                static_cast<unsigned long long>(run.lut_build_us));

    std::printf("\nEstágios (média por frame, sobre todas as fases):\n");
    double stage_mean[kStageCount] = {};
    uint32_t stage_max[kStageCount] = {};
    for (const FrameRecord& r : run.frames) {
        for (int s = 0; s < kStageCount; ++s) {
            stage_mean[s] += r.stage_us[s];
            if (r.stage_us[s] > stage_max[s]) stage_max[s] = r.stage_us[s];
        }
    }
    double stage_sum = 0.0;
    for (int s = 0; s < kStageCount; ++s) {
        stage_mean[s] /= static_cast<double>(run.frames.size());
        stage_sum += stage_mean[s];
    }
    for (int s = 0; s < kStageCount; ++s) {
        if (stage_mean[s] <= 0.0) continue;
        std::printf("  %-14s %9.1f us  %5.1f%%  (pior %u us)\n", stage_name(static_cast<Stage>(s)),
                    stage_mean[s], 100.0 * stage_mean[s] / stage_sum, stage_max[s]);
    }

    if (!run.sys.empty()) {
        double cpu_all = 0.0, cpu_proc = 0.0, temp_max = -1.0, freq_sum = 0.0;
        double rss = 0.0;
        int nfreq = 0;
        for (const SysSample& s : run.sys) {
            cpu_all += s.cpu_all;
            cpu_proc += s.cpu_proc;
            if (s.temp_c > temp_max) temp_max = s.temp_c;
            if (s.freq_mhz > 0) {
                freq_sum += s.freq_mhz;
                ++nfreq;
            }
            rss = s.rss_kb;
        }
        const double n = static_cast<double>(run.sys.size());
        std::printf("\nSistema (%zu amostras em %.2f s):\n", run.sys.size(), run.wall_us / 1e6);
        std::printf("  CPU do processo   %6.1f%% de um núcleo (%.1f%% da máquina toda)\n",
                    cpu_proc / n, cpu_proc / n / (ncpu > 0 ? ncpu : 1));
        std::printf("  CPU da máquina    %6.1f%%\n", cpu_all / n);
        if (temp_max >= 0) std::printf("  temperatura máx   %6.1f °C\n", temp_max);
        if (nfreq) std::printf("  frequência média  %6.0f MHz\n", freq_sum / nfreq);
        std::printf("  RSS final         %6.0f kB\n", rss);

        if (args.p_idle_w > 0.0 && args.p_busy_w > 0.0) {
            const double util = (cpu_proc / n) / 100.0 / (ncpu > 0 ? ncpu : 1);
            const double watts = args.p_idle_w + (args.p_busy_w - args.p_idle_w) * util;
            std::printf(
                "\nEnergia (MODELO, não medição — interpola entre %.2f W ocioso e %.2f W "
                "saturado):\n",
                args.p_idle_w, args.p_busy_w);
            std::printf("  utilização %.1f%% → %.2f W → %.1f J em %.1f s → %.0f mWh/h\n",
                        util * 100.0, watts, watts * run.wall_us / 1e6, run.wall_us / 1e6,
                        watts * 1000.0);
        }
    }

    std::vector<SweepPoint> sweep;
    if (args.sweep) {
        const int32_t dims[][2] = {{160, 120}, {320, 240}, {480, 360}, {640, 480}};
        std::printf("\nVarredura de resolução (mesmo cenário, %d frames por fase):\n",
                    args.per_phase);
        std::printf("  %-11s %10s %10s %10s %10s\n", "resolução", "média us", "p95 us", "max us",
                    "Hz médio");
        for (const auto& d : dims) {
            const RunOutput r = run_scenario(args, d[0], d[1]);
            if (r.frames.empty()) continue;
            SweepPoint pt;
            pt.w = d[0];
            pt.h = d[1];
            pt.s = summarize(r.frames, -1);
            pt.cpu_proc = r.cpu_proc_avg;
            sweep.push_back(pt);
            char label[32];
            std::snprintf(label, sizeof(label), "%dx%d", d[0], d[1]);
            std::printf("  %-11s %10.1f %10.1f %10u %10.1f\n", label, pt.s.mean_us, pt.s.p95,
                        pt.s.max_us, pt.s.fps_mean());
        }
    }

    if (!args.csv_path.empty()) {
        write_csv(args.csv_path, run);
        std::printf("\nCSV por frame  → %s\n", args.csv_path.c_str());
    }
    if (!args.json_path.empty()) {
        write_json(args.json_path, args, run, sweep, model, ncpu);
        std::printf("JSON do resumo → %s\n", args.json_path.c_str());
    }
    return 0;
}
