// Testes do filtro de Kalman desacoplado por eixo.
#include <cmath>

#include "ecv/track/kalman.hpp"
#include "test_harness.hpp"

using namespace ecv;

ECV_TEST(kalman_converge_para_a_velocidade_real_do_alvo) {
    Kalman1D k;
    k.configure(500.0f, 4.0f);
    k.reset(0.0f);

    const float dt = 1.0f / 60.0f;
    const float velocity = 120.0f;  // px/s
    float truth = 0.0f;
    for (int i = 0; i < 90; ++i) {
        truth += velocity * dt;
        k.predict(dt);
        k.update(truth);
    }

    CHECK_NEAR(k.velocity(), velocity, 8.0);
    CHECK_NEAR(k.position(), truth, 2.0);
}

ECV_TEST(kalman_filtra_ruido_de_medida) {
    Kalman1D k;
    k.configure(50.0f, 25.0f);
    k.reset(100.0f);

    // Alvo parado em 100 px com medidas oscilando ±5 px.
    const float dt = 1.0f / 60.0f;
    float worst_measurement_error = 0.0f;
    for (int i = 0; i < 120; ++i) {
        const float noise = (i % 2 == 0) ? 5.0f : -5.0f;
        k.predict(dt);
        k.update(100.0f + noise);
        worst_measurement_error = 5.0f;
    }
    const float filtered_error = std::fabs(k.position() - 100.0f);
    CHECK(filtered_error < worst_measurement_error);
    CHECK(filtered_error < 2.0f);
}

ECV_TEST(kalman_extrapola_durante_oclusao) {
    Kalman2D k;
    k.configure(500.0f, 4.0f);
    k.reset(Point{50, 120});

    const float dt = 1.0f / 60.0f;
    float truth = 50.0f;
    for (int i = 0; i < 60; ++i) {
        truth += 100.0f * dt;
        k.predict(dt);
        k.update(Point{static_cast<int16_t>(truth), 120});
    }

    const float before = k.uncertainty();
    // 10 frames sem medida: a posição continua avançando na direção certa.
    for (int i = 0; i < 10; ++i) {
        truth += 100.0f * dt;
        k.predict(dt);
    }
    CHECK_NEAR(k.position().x, truth, 6.0);
    CHECK(k.uncertainty() > before);  // e a incerteza cresce, como deve
}

ECV_TEST(kalman_invalidado_reinicializa_na_proxima_medida) {
    Kalman2D k;
    k.configure(500.0f, 4.0f);
    k.reset(Point{10, 10});
    k.update(Point{10, 10});
    CHECK(k.initialized());

    k.invalidate();
    CHECK(!k.initialized());

    k.update(Point{200, 30});
    CHECK(k.initialized());
    CHECK_EQ(static_cast<int>(k.position().x), 200);
}

ECV_TEST(project_antecipa_a_posicao_pela_velocidade_estimada) {
    Kalman1D k;
    k.configure(500.0f, 4.0f);
    k.reset(0.0f);

    const float dt = 1.0f / 60.0f;
    float truth = 0.0f;
    for (int i = 0; i < 120; ++i) {
        truth += 200.0f * dt;
        k.predict(dt);
        k.update(truth);
    }
    // 50 ms à frente com ~200 px/s = ~10 px adiante da posição atual.
    CHECK_NEAR(k.project(0.05f) - k.position(), 10.0f, 1.5);
}
