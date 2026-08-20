// Testes de controle: ponto fixo, PD (float e Q16.16) e mixer diferencial.
#include "ecv/control/differential.hpp"
#include "ecv/control/pd.hpp"
#include "ecv/core/fixed.hpp"
#include "test_harness.hpp"

using namespace ecv;

ECV_TEST(fixed_q16_mantem_precisao_e_satura_em_vez_de_estourar) {
    const Fixed a = Fixed::from_float(2.5f);
    const Fixed b = Fixed::from_float(-0.75f);

    CHECK_NEAR((a + b).to_float(), 1.75f, 1e-4);
    CHECK_NEAR((a * b).to_float(), -1.875f, 1e-4);
    CHECK_NEAR((a / b).to_float(), -3.3333f, 1e-3);
    CHECK_NEAR(abs(b).to_float(), 0.75f, 1e-4);

    // Divisão por zero e overflow saturam nos extremos, nunca invertem o sinal.
    CHECK(Fixed::from_float(1.0f) / Fixed::from_float(0.0f) > Fixed::from_int(30000));
    const Fixed big = Fixed::from_float(30000.0f);
    CHECK((big * big) > Fixed::from_int(32000));
}

ECV_TEST(fixed_e_float_produzem_o_mesmo_pd_dentro_da_tolerancia) {
    PdController<float>::Config cfg;
    cfg.kp = 0.8f;
    cfg.kd = 0.05f;
    cfg.d_alpha = 0.5f;

    PdController<float> pd_float;
    pd_float.configure(cfg);

    PdController<Fixed>::Config cfg_fx;
    cfg_fx.kp = cfg.kp;
    cfg_fx.kd = cfg.kd;
    cfg_fx.d_alpha = cfg.d_alpha;
    PdController<Fixed> pd_fixed;
    pd_fixed.configure(cfg_fx);

    const float dt = 1.0f / 60.0f;
    const float errors[] = {0.5f, 0.4f, 0.2f, -0.1f, -0.3f, 0.0f};
    for (float e : errors) {
        const float out_f = pd_float.update(e, dt);
        const float out_x = pd_fixed.update(Fixed::from_float(e), Fixed::from_float(dt)).to_float();
        CHECK_NEAR(out_f, out_x, 2e-3);
    }
}

ECV_TEST(pd_reage_ao_sinal_do_erro_e_respeita_o_clamp) {
    PdController<float>::Config cfg;
    cfg.kp = 2.0f;
    cfg.kd = 0.0f;
    cfg.out_min = -0.5f;
    cfg.out_max = 0.5f;
    PdController<float> pd;
    pd.configure(cfg);

    CHECK_NEAR(pd.update(0.1f, 0.016f), 0.2f, 1e-5);
    CHECK_NEAR(pd.update(-0.1f, 0.016f), -0.2f, 1e-5);
    CHECK_NEAR(pd.update(1.0f, 0.016f), 0.5f, 1e-5);  // saturado
    CHECK_NEAR(pd.update(-1.0f, 0.016f), -0.5f, 1e-5);
}

ECV_TEST(derivada_filtrada_amortece_salto_de_um_frame) {
    PdController<float>::Config unfiltered;
    unfiltered.kp = 0.0f;
    unfiltered.kd = 0.01f;
    unfiltered.d_alpha = 1.0f;
    unfiltered.out_min = -10.0f;
    unfiltered.out_max = 10.0f;

    PdController<float>::Config filtered = unfiltered;
    filtered.d_alpha = 0.3f;

    PdController<float> raw, smooth;
    raw.configure(unfiltered);
    smooth.configure(filtered);

    const float dt = 0.016f;
    raw.update(0.0f, dt);
    smooth.update(0.0f, dt);
    const float spike_raw = raw.update(0.5f, dt);  // salto de centroide
    const float spike_smooth = smooth.update(0.5f, dt);

    CHECK(spike_smooth < spike_raw);
    CHECK(spike_smooth > 0.0f);
}

ECV_TEST(mixer_satura_preservando_a_razao_entre_as_rodas) {
    DifferentialConfig cfg;
    cfg.max_speed = 1.0f;
    DifferentialMixer mixer;
    mixer.configure(cfg);

    const MotorCommand c = mixer.mix(0.9f, 0.6f);  // pediria 1.5 / 0.3
    CHECK_NEAR(c.left, 1.0f, 1e-5);
    CHECK_NEAR(c.right, 0.2f, 1e-5);
    CHECK_NEAR(c.left / c.right, 1.5f / 0.3f, 1e-3);
}

ECV_TEST(mixer_gira_no_lugar_e_aplica_deadband) {
    DifferentialConfig cfg;
    cfg.max_speed = 1.0f;
    cfg.deadband = 0.25f;
    DifferentialMixer mixer;
    mixer.configure(cfg);

    const MotorCommand spin = mixer.mix(0.0f, 0.4f);
    CHECK(spin.left > 0.0f);
    CHECK(spin.right < 0.0f);
    CHECK_NEAR(spin.left, -spin.right, 1e-5);
    CHECK(spin.left >= cfg.deadband);  // acima do atrito estático

    const MotorCommand idle = mixer.mix(0.0f, 0.0f);
    CHECK_NEAR(idle.left, 0.0f, 1e-6);
    CHECK_NEAR(idle.right, 0.0f, 1e-6);
}

ECV_TEST(to_pwm_converte_para_duty_e_sentido) {
    const PwmDuty forward = to_pwm(0.5f, 1023);
    CHECK_EQ(static_cast<int>(forward.magnitude), 512);
    CHECK(!forward.reverse);

    const PwmDuty back = to_pwm(-1.5f, 1023);  // além do limite: satura
    CHECK_EQ(static_cast<int>(back.magnitude), 1023);
    CHECK(back.reverse);
}
