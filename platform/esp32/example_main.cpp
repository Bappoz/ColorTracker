// Exemplo de app_main para ESP32-CAM: copiar para `main/` do projeto IDF.
//
// Não é compilado pelo componente — está aqui como referência de fiação entre
// câmera, pipeline e ponte H.
#include "ecv/app/sumo_vision.hpp"
#include "esp32_cam_source.hpp"
#include "esp_log.h"
#include "ledc_motor_sink.hpp"

using namespace ecv;

namespace {

constexpr const char* kTag = "sumo";

// 320x240: máscara de 76.800 B + LUT de 8 KB + workspace. Fica em .bss, alocado
// no link — nada de heap depois do boot.
SumoStorage<320, 240> g_storage;

SumoConfig make_config() {
    SumoConfig cfg;
    cfg.frame_w = 320;
    cfg.frame_h = 240;
    cfg.target.h_min = 170;  // saída do ecv_calibrate
    cfg.target.h_max = 10;
    cfg.target.s_min = 120;
    cfg.target.v_min = 60;
    cfg.min_area = 150;
    cfg.pd.kp = 0.9f;
    cfg.pd.kd = 0.06f;
    cfg.drive.deadband = 0.2f;
    cfg.attack_speed = 0.6f;
    return cfg;
}

}  // namespace

extern "C" void app_main() {
    esp32::Esp32CamSource camera{esp32::CamConfig{}};
    esp32::LedcMotorSink motors{esp32::LedcMotorConfig{}};

    if (!camera.open()) {
        ESP_LOGE(kTag, "câmera não inicializou (erro %d)", camera.last_error());
        return;
    }
    if (!motors.open()) {
        ESP_LOGE(kTag, "LEDC não inicializou");
        camera.close();
        return;
    }

    SumoVision vision(make_config(), g_storage.view());
    ImageView frame{};
    uint64_t previous = micros();
    uint32_t frames = 0;
    LatencyStats latency;

    while (true) {
        if (!camera.next(frame)) {
            motors.stop();
            ESP_LOGE(kTag, "frame perdido");
            continue;
        }
        const uint64_t now = micros();
        const float dt = static_cast<float>(now - previous) * 1e-6f;
        previous = now;

        const SumoResult r = vision.process(frame, dt);
        motors.write(r.cmd);
        latency.add(r.timings.total_us);

        if (++frames % 60 == 0) {
            ESP_LOGI(kTag, "%s alvo=(%d,%d) erro=%+.2f L=%+.2f R=%+.2f visão=%.0f us",
                     track_state_name(r.state), r.target.x, r.target.y, r.error_norm, r.cmd.left,
                     r.cmd.right, latency.mean_us());
        }
    }
}
