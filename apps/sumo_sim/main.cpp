// Roda o pipeline de sumô sem hardware: cena sintética ou sequência de PPMs.
//
// É o "cargo run" do projeto — cada mudança no pipeline tem que continuar
// produzindo comando de motor coerente aqui antes de subir para o robô.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ecv/app/sumo_vision.hpp"
#include "sim/log_motor_sink.hpp"
#include "sim/ppm_io.hpp"
#include "sim/scene_source.hpp"

using namespace ecv;

namespace {

void usage(const char* prog) {
    std::printf(
        "uso: %s [opções] [arquivo.ppm ...]\n"
        "  --frames N     quantidade de frames sintéticos (padrão 120)\n"
        "  --occlude A,B  esconde o oponente entre os frames A e B\n"
        "  --dump DIR     grava frame e máscara de cada iteração em DIR\n"
        "  --quiet        só o resumo final\n",
        prog);
}

}  // namespace

int main(int argc, char** argv) {
    int frames = 120;
    int occlude_from = -1, occlude_to = -1;
    std::string dump_dir;
    bool quiet = false;
    std::vector<std::string> ppm_files;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else if (a == "--frames" && i + 1 < argc) {
            frames = std::atoi(argv[++i]);
        } else if (a == "--dump" && i + 1 < argc) {
            dump_dir = argv[++i];
        } else if (a == "--quiet") {
            quiet = true;
        } else if (a == "--occlude" && i + 1 < argc) {
            std::sscanf(argv[++i], "%d,%d", &occlude_from, &occlude_to);
        } else {
            ppm_files.push_back(a);
        }
    }

    SumoConfig cfg;
    cfg.target.h_min = 170;  // marcador vermelho
    cfg.target.h_max = 10;
    cfg.target.s_min = 120;
    cfg.target.v_min = 60;
    cfg.min_area = 150;
    cfg.pd.kp = 0.9f;
    cfg.pd.kd = 0.06f;
    cfg.attack_speed = 0.6f;
    cfg.drive.deadband = 0.2f;

    sim::SceneConfig scene;
    sim::SyntheticSource synthetic(scene);
    sim::PpmSequenceSource replay(ppm_files, false);
    FrameSource* source = ppm_files.empty() ? static_cast<FrameSource*>(&synthetic)
                                            : static_cast<FrameSource*>(&replay);
    if (!source->open()) {
        std::fprintf(stderr, "falha ao abrir a fonte de frames (%s)\n", source->name());
        return 1;
    }
    cfg.frame_w = static_cast<int16_t>(source->width());
    cfg.frame_h = static_cast<int16_t>(source->height());
    if (!ppm_files.empty()) frames = static_cast<int>(ppm_files.size());

    static SumoStorage<320, 240> storage;
    if (cfg.frame_w > 320 || cfg.frame_h > 240) {
        std::fprintf(stderr, "esta build reserva buffers para 320x240; frame é %dx%d\n",
                     cfg.frame_w, cfg.frame_h);
        return 1;
    }
    SumoVision::Buffers buf = storage.view();
    buf.mask.width = cfg.frame_w;
    buf.mask.height = cfg.frame_h;
    SumoVision vision(cfg, buf);
    sim::LogMotorSink motors;
    motors.open();

    const float dt = 1.0f / 60.0f;
    LatencyStats latency;
    int detections = 0;
    ImageView frame{};

    for (int i = 0; i < frames; ++i) {
        if (ppm_files.empty()) {
            synthetic.advance(dt);
            synthetic.set_visible(!(i >= occlude_from && i <= occlude_to && occlude_from >= 0));
        }
        if (!source->next(frame)) break;

        const SumoResult r = vision.process(frame, dt);
        motors.write(r.cmd);
        latency.add(r.timings.total_us);
        detections += r.detected ? 1 : 0;

        if (!quiet) {
            std::printf(
                "%4d %-9s alvo=(%3d,%3d) area=%5u erro=%+.3f roi=%3dx%-3d L=%+.2f R=%+.2f %5u us\n",
                i, track_state_name(r.state), r.target.x, r.target.y, r.area, r.error_norm, r.roi.w,
                r.roi.h, r.cmd.left, r.cmd.right, r.timings.total_us);
        }

        if (!dump_dir.empty()) {
            char path[512];
            std::snprintf(path, sizeof(path), "%s/frame_%04d.ppm", dump_dir.c_str(), i);
            sim::write_ppm(path, frame);
            std::snprintf(path, sizeof(path), "%s/mask_%04d.ppm", dump_dir.c_str(), i);
            MaskView m = buf.mask;
            m.width = r.roi.w;
            m.height = r.roi.h;
            sim::write_mask_ppm(path, m);
        }
    }

    motors.stop();
    std::printf("\n%d frames · %d com detecção (%.0f%%) · latência média %.1f us (pior %u us)\n",
                frames, detections, frames ? 100.0 * detections / frames : 0.0, latency.mean_us(),
                latency.max_us);
    return 0;
}
