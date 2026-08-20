// Teste de ponta a ponta: cena sintética -> comando de motor.
// A verdade de campo é exata porque a cena é gerada por construção.
#include "ecv/app/sumo_vision.hpp"
#include "ecv/vision/calibrate.hpp"
#include "sim/scene_source.hpp"
#include "test_harness.hpp"

using namespace ecv;

namespace {

constexpr int kW = 320;
constexpr int kH = 240;
constexpr float kDt = 1.0f / 60.0f;

SumoConfig make_config() {
    SumoConfig cfg;
    cfg.frame_w = kW;
    cfg.frame_h = kH;
    cfg.target.h_min = 170;  // vermelho com wrap
    cfg.target.h_max = 10;
    cfg.target.s_min = 120;
    cfg.target.v_min = 60;
    cfg.min_area = 150;
    cfg.pd.kp = 0.9f;
    cfg.pd.kd = 0.05f;
    cfg.attack_speed = 0.6f;
    cfg.search_turn = 0.35f;
    return cfg;
}

sim::SceneConfig make_scene() {
    sim::SceneConfig s;
    s.width = kW;
    s.height = kH;
    s.opponent_radius = 24;
    s.noise = 4;
    return s;
}

}  // namespace

ECV_TEST(pipeline_aponta_para_o_oponente_a_direita) {
    static SumoStorage<kW, kH> storage;
    SumoVision vision(make_config(), storage.view());
    sim::SyntheticSource src(make_scene());
    CHECK(src.open());
    src.set_opponent(Point{240, 120});  // à direita do centro (160)

    ImageView frame{};
    CHECK(src.next(frame));
    const SumoResult r = vision.process(frame, kDt);

    CHECK(r.detected);
    CHECK(r.state == TrackState::kTracking);
    CHECK_NEAR(r.target.x, 240, 4.0);
    CHECK_NEAR(r.target.y, 120, 4.0);
    CHECK(r.error_norm > 0.4f);
    CHECK(r.cmd.left > r.cmd.right);  // vira para a direita
    CHECK(r.area > 1500u);
}

ECV_TEST(pipeline_inverte_o_comando_com_o_oponente_a_esquerda) {
    static SumoStorage<kW, kH> storage;
    SumoVision vision(make_config(), storage.view());
    sim::SyntheticSource src(make_scene());
    CHECK(src.open());
    src.set_opponent(Point{60, 120});

    ImageView frame{};
    CHECK(src.next(frame));
    const SumoResult r = vision.process(frame, kDt);

    CHECK(r.detected);
    CHECK(r.error_norm < -0.4f);
    CHECK(r.cmd.right > r.cmd.left);
}

ECV_TEST(pipeline_avanca_mais_quando_esta_alinhado) {
    static SumoStorage<kW, kH> storage;
    sim::SyntheticSource src(make_scene());
    CHECK(src.open());
    ImageView frame{};

    SumoVision centered(make_config(), storage.view());
    src.set_opponent(Point{160, 120});
    CHECK(src.next(frame));
    const SumoResult aligned = centered.process(frame, kDt);

    SumoVision skewed(make_config(), storage.view());
    src.set_opponent(Point{280, 120});
    CHECK(src.next(frame));
    const SumoResult off = skewed.process(frame, kDt);

    const float forward_aligned = (aligned.cmd.left + aligned.cmd.right) * 0.5f;
    const float forward_off = (off.cmd.left + off.cmd.right) * 0.5f;
    CHECK(forward_aligned > forward_off);
    CHECK(forward_aligned > 0.4f);
}

ECV_TEST(pipeline_estreita_a_roi_depois_de_travar_no_alvo) {
    static SumoStorage<kW, kH> storage;
    SumoVision vision(make_config(), storage.view());
    sim::SyntheticSource src(make_scene());
    CHECK(src.open());
    src.set_opponent(Point{160, 120});

    ImageView frame{};
    CHECK(src.next(frame));
    const SumoResult first = vision.process(frame, kDt);
    CHECK_EQ(static_cast<int>(first.roi.w), kW);  // primeiro frame varre tudo

    CHECK(src.next(frame));
    const SumoResult second = vision.process(frame, kDt);
    CHECK(second.detected);
    CHECK(second.roi.w < kW);
    CHECK(second.roi.h < kH);
    // Continua acertando a posição global mesmo processando só a ROI.
    CHECK_NEAR(second.target.x, 160, 5.0);

    // O trabalho por frame cai junto com a área processada.
    const int32_t full_px = kW * kH;
    const int32_t roi_px = second.roi.w * second.roi.h;
    CHECK(roi_px * 2 < full_px);
}

ECV_TEST(pipeline_passa_por_coasting_antes_de_desistir) {
    SumoConfig cfg = make_config();
    cfg.coast_timeout_s = 0.2f;
    static SumoStorage<kW, kH> storage;
    SumoVision vision(cfg, storage.view());
    sim::SyntheticSource src(make_scene());
    CHECK(src.open());
    src.set_opponent(Point{200, 120});

    ImageView frame{};
    for (int i = 0; i < 5; ++i) {
        CHECK(src.next(frame));
        CHECK(vision.process(frame, kDt).detected);
    }

    src.set_visible(false);  // oponente sumiu (oclusão / marcador virado)
    CHECK(src.next(frame));
    const SumoResult coasting = vision.process(frame, kDt);
    CHECK(!coasting.detected);
    CHECK(coasting.state == TrackState::kCoasting);
    CHECK(coasting.error_norm > 0.0f);  // ainda mira onde o alvo estava

    // O reset da ROI vale para o frame seguinte: este ainda foi processado na
    // ROI estreita herdada do último acerto.
    CHECK(src.next(frame));
    CHECK_EQ(static_cast<int>(vision.process(frame, kDt).roi.w), kW);

    for (int i = 0; i < 30; ++i) {
        CHECK(src.next(frame));
        vision.process(frame, kDt);
    }
    CHECK(src.next(frame));
    const SumoResult lost = vision.process(frame, kDt);
    CHECK(lost.state == TrackState::kSearching);
    CHECK(lost.cmd.left > 0.0f);
    CHECK(lost.cmd.right < 0.0f);  // gira no lugar procurando
}

ECV_TEST(pipeline_recua_quando_ve_a_borda_do_ringue) {
    SumoConfig cfg = make_config();
    cfg.border_enabled = true;
    cfg.border.target.s_max = 60;
    cfg.border.target.v_min = 200;
    cfg.border.y_top = 210;
    cfg.border.y_bottom = 235;
    cfg.border.rows = 4;
    cfg.border.min_hits = 20;
    cfg.retreat_speed = 0.7f;

    static SumoStorage<kW, kH> storage;
    SumoVision vision(cfg, storage.view());

    sim::SceneConfig scene = make_scene();
    scene.noise = 0;
    scene.white_band = Rect{0, 200, 120, 40};  // borda entrando pela esquerda
    sim::SyntheticSource src(scene);
    CHECK(src.open());
    src.set_opponent(Point{240, 100});

    ImageView frame{};
    CHECK(src.next(frame));
    const SumoResult r = vision.process(frame, kDt);

    CHECK(r.border_hit());
    CHECK(r.border.left());  // e sabe de que lado ela está
    CHECK(!r.border.right());
    CHECK(r.detected);         // ainda enxerga o oponente
    CHECK(r.cmd.left < 0.0f);  // mas a prioridade é não sair do dohyo
    CHECK(r.cmd.right < 0.0f);
}

ECV_TEST(recalibrar_em_runtime_troca_o_alvo_sem_reconstruir_o_pipeline) {
    // É o que a tecla `c` do ecv_probe faz: apontar a câmera para outro alvo e
    // recalibrar sem reiniciar (e sem perder a exposição já estabilizada).
    static SumoStorage<kW, kH> storage;
    SumoConfig cfg = make_config();
    SumoVision vision(cfg, storage.view());

    sim::SceneConfig scene = make_scene();
    scene.opponent = Rgb{40, 60, 210};  // alvo azul: fora da faixa vermelha
    scene.noise = 0;
    sim::SyntheticSource src(scene);
    CHECK(src.open());
    src.set_opponent(Point{200, 120});

    ImageView frame{};
    CHECK(src.next(frame));
    CHECK(!vision.process(frame, kDt).detected);

    static CalibrationWorkspace ws;
    const CalibrationResult c = estimate_hsv_range(frame, Rect{176, 96, 48, 48}, ws);
    vision.set_target_range(c.range);

    CHECK(src.next(frame));
    const SumoResult after = vision.process(frame, kDt);
    CHECK(after.detected);
    CHECK_NEAR(after.target.x, 200, 6.0);
}

ECV_TEST(pipeline_rastreia_alvo_em_movimento_sem_perder) {
    static SumoStorage<kW, kH> storage;
    SumoVision vision(make_config(), storage.view());
    sim::SyntheticSource src(make_scene());
    CHECK(src.open());
    src.set_opponent(Point{40, 120});

    ImageView frame{};
    int detections = 0;
    float worst_error_px = 0.0f;
    for (int i = 0; i < 120; ++i) {
        src.advance(kDt);
        CHECK(src.next(frame));
        const SumoResult r = vision.process(frame, kDt);
        if (!r.detected) continue;
        ++detections;
        const float err = static_cast<float>(r.target.x - src.opponent().x);
        const float abs_err = err < 0 ? -err : err;
        if (abs_err > worst_error_px) worst_error_px = abs_err;
    }

    CHECK(detections >= 118);
    CHECK(worst_error_px < 15.0f);  // inclui a mira antecipada de 50 ms
}

ECV_TEST(pipeline_funciona_igual_nos_quatro_formatos_de_pixel) {
    const PixelFormat formats[] = {PixelFormat::kRgb888, PixelFormat::kBgr888, PixelFormat::kRgb565,
                                   PixelFormat::kYuyv};
    for (PixelFormat fmt : formats) {
        static SumoStorage<kW, kH> storage;
        SumoVision vision(make_config(), storage.view());

        sim::SceneConfig scene = make_scene();
        scene.format = fmt;
        scene.noise = 0;
        sim::SyntheticSource src(scene);
        CHECK(src.open());
        src.set_opponent(Point{200, 130});

        ImageView frame{};
        CHECK(src.next(frame));
        const SumoResult r = vision.process(frame, kDt);
        CHECK(r.detected);
        CHECK_NEAR(r.target.x, 200, 6.0);
        CHECK_NEAR(r.target.y, 130, 6.0);
    }
}
