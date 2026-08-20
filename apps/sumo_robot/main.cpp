// Loop do robô no Raspberry Pi: V4L2 -> pipeline -> PWM.
//
// Invariante de segurança: existe exatamente um ponto de saída, e ele para os
// motores. Qualquer erro (câmera sumiu, sinal recebido, frame atrasado) cai
// nele. Robô que perde o controlador com PWM travado não para sozinho.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "ecv/app/sumo_vision.hpp"
#include "linux/pwm_sink.hpp"
#include "linux/v4l2_source.hpp"
#include "sim/log_motor_sink.hpp"

using namespace ecv;

namespace {

volatile std::sig_atomic_t g_running = 1;
void on_signal(int) {
    g_running = 0;
}

}  // namespace

int main(int argc, char** argv) {
    linux_hal::V4l2Config cam;
    bool dry_run = false;
    float duration_s = 0.0f;  // 0 = até Ctrl-C

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--device" && i + 1 < argc)
            cam.device = argv[++i];
        else if (a == "--dry-run")
            dry_run = true;
        else if (a == "--seconds" && i + 1 < argc)
            duration_s = static_cast<float>(std::atof(argv[++i]));
        else if (a == "--help") {
            std::printf("uso: %s [--device /dev/video0] [--dry-run] [--seconds N]\n", argv[0]);
            return 0;
        }
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    linux_hal::V4l2Source camera(cam);
    if (!camera.open()) {
        std::fprintf(stderr, "câmera: %s\n", camera.last_error().c_str());
        return 1;
    }

    linux_hal::PwmSinkConfig pwm;
    pwm.left_a.index = 0;
    pwm.left_b.index = 1;
    pwm.right_a.index = 2;
    pwm.right_b.index = 3;
    pwm.max_duty = 0.8f;  // margem de segurança nos primeiros testes

    linux_hal::SysfsPwmSink motors_real(pwm);
    sim::LogMotorSink motors_log(true);
    MotorSink* motors =
        dry_run ? static_cast<MotorSink*>(&motors_log) : static_cast<MotorSink*>(&motors_real);
    if (!motors->open()) {
        std::fprintf(stderr, "pwm: %s\n", motors_real.last_error().c_str());
        camera.close();
        return 1;
    }

    SumoConfig cfg;
    cfg.frame_w = static_cast<int16_t>(camera.width());
    cfg.frame_h = static_cast<int16_t>(camera.height());
    cfg.target.h_min = 170;  // trocar pelo resultado do ecv_calibrate
    cfg.target.h_max = 10;
    cfg.target.s_min = 120;
    cfg.target.v_min = 60;
    cfg.pd.kp = 0.9f;
    cfg.pd.kd = 0.06f;
    cfg.drive.deadband = 0.2f;
    cfg.attack_speed = 0.6f;

    static SumoStorage<320, 240> storage;
    if (cfg.frame_w > 320 || cfg.frame_h > 240) {
        std::fprintf(stderr, "buffers reservados para 320x240, câmera entregou %dx%d\n",
                     cfg.frame_w, cfg.frame_h);
        camera.close();
        motors->close();
        return 1;
    }
    SumoVision::Buffers buf = storage.view();
    buf.mask.width = cfg.frame_w;
    buf.mask.height = cfg.frame_h;
    SumoVision vision(cfg, buf);

    std::printf("rodando %dx%d em %s (%s)\n", cfg.frame_w, cfg.frame_h, cam.device.c_str(),
                motors->name());

    LatencyStats latency;
    uint64_t previous = micros();
    const uint64_t started = previous;
    ImageView frame{};
    int exit_code = 0;
    uint32_t frames = 0;

    while (g_running) {
        if (!camera.next(frame)) {
            std::fprintf(stderr, "captura falhou: %s\n", camera.last_error().c_str());
            exit_code = 1;
            break;
        }
        const uint64_t now = micros();
        const float dt = static_cast<float>(now - previous) * 1e-6f;
        previous = now;

        const SumoResult r = vision.process(frame, dt);
        motors->write(r.cmd);
        latency.add(r.timings.total_us);
        ++frames;

        if (duration_s > 0.0f && static_cast<float>(now - started) * 1e-6f >= duration_s) break;
    }

    motors->stop();
    motors->close();
    camera.close();
    std::printf("\n%u frames · latência de visão média %.1f us (pior %u us)\n", frames,
                latency.mean_us(), latency.max_us);
    return exit_code;
}
