// Testes da aritmética de duty da ponte H. Só a parte pura: abrir sysfs exige
// hardware, mas o que decide quanta potência vai para o motor cabe aqui.
#include "linux/pwm_sink.hpp"
#include "test_harness.hpp"

using namespace ecv;
using namespace ecv::linux_hal;

namespace {
constexpr uint32_t kPeriod = 50000;  // 20 kHz
}

ECV_TEST(duty_satura_no_maximo_configurado_em_vez_de_estourar_o_periodo) {
    // O limite existe para proteger bateria e ponte nos primeiros testes:
    // um comando de 100% com max_duty 0,8 tem de virar 80%, não 100%.
    CHECK_EQ(duty_ns(1.0f, kPeriod, 0.8f), 40000u);
    CHECK_EQ(duty_ns(0.5f, kPeriod, 0.8f), 25000u);
    CHECK_EQ(duty_ns(0.8f, kPeriod, 0.8f), 40000u);

    // Nunca passa do período, aconteça o que acontecer com a entrada.
    CHECK(duty_ns(5.0f, kPeriod, 1.0f) <= kPeriod);
    CHECK(duty_ns(1.0f, kPeriod, 2.0f) <= kPeriod);
}

ECV_TEST(duty_trata_negativo_zero_e_nan_como_parado) {
    CHECK_EQ(duty_ns(-0.5f, kPeriod, 1.0f), 0u);
    CHECK_EQ(duty_ns(0.0f, kPeriod, 1.0f), 0u);
    // NaN não pode virar duty gigante: um robô com PWM travado não para sozinho.
    const float nan_value = 0.0f / 0.0f;
    CHECK_EQ(duty_ns(nan_value, kPeriod, 1.0f), 0u);
}

ECV_TEST(sentido_do_motor_sai_de_qual_entrada_da_ponte_recebe_o_duty) {
    // Frente: entrada A recebe, B fica em zero.
    MotorCommand fwd{};
    fwd.left = 0.5f;
    fwd.right = 0.5f;
    BridgeDuties d = duties_for(fwd, kPeriod, 1.0f);
    CHECK_EQ(d.left_a, 25000u);
    CHECK_EQ(d.left_b, 0u);
    CHECK_EQ(d.right_a, 25000u);
    CHECK_EQ(d.right_b, 0u);

    // Ré: espelha, e a entrada A tem de zerar.
    MotorCommand back{};
    back.left = -0.5f;
    back.right = -0.5f;
    d = duties_for(back, kPeriod, 1.0f);
    CHECK_EQ(d.left_a, 0u);
    CHECK_EQ(d.left_b, 25000u);
    CHECK_EQ(d.right_a, 0u);
    CHECK_EQ(d.right_b, 25000u);
}

ECV_TEST(giro_no_lugar_manda_sentidos_opostos_nos_dois_lados) {
    MotorCommand spin{};
    spin.left = 0.35f;
    spin.right = -0.35f;
    const BridgeDuties d = duties_for(spin, kPeriod, 1.0f);

    CHECK_EQ(d.left_a, 17500u);
    CHECK_EQ(d.left_b, 0u);
    CHECK_EQ(d.right_a, 0u);
    CHECK_EQ(d.right_b, 17500u);
}

ECV_TEST(nenhum_comando_liga_as_duas_entradas_do_mesmo_motor) {
    // Invariante da ponte: as duas entradas com duty ao mesmo tempo significa
    // freio, nunca avanço. Nenhum comando pode produzir isso.
    for (int i = -20; i <= 20; ++i) {
        MotorCommand cmd{};
        cmd.left = static_cast<float>(i) * 0.05f;
        cmd.right = static_cast<float>(-i) * 0.05f;
        const BridgeDuties d = duties_for(cmd, kPeriod, 1.0f);
        CHECK(d.left_a == 0 || d.left_b == 0);
        CHECK(d.right_a == 0 || d.right_b == 0);
    }
}

ECV_TEST(comando_parado_zera_os_quatro_canais) {
    const BridgeDuties d = duties_for(MotorCommand::stopped(), kPeriod, 1.0f);
    CHECK_EQ(d.left_a, 0u);
    CHECK_EQ(d.left_b, 0u);
    CHECK_EQ(d.right_a, 0u);
    CHECK_EQ(d.right_b, 0u);
}

ECV_TEST(abrir_sem_pwmchip_falha_com_mensagem_acionavel_em_vez_de_travar) {
    // É exatamente o estado do Pi 3 sem `dtoverlay=pwm-2chan`: o diretório não
    // existe. O sink tem de recusar na hora e dizer o que fazer.
    PwmSinkConfig cfg;
    cfg.left_a.chip = cfg.left_b.chip = "/sys/class/pwm/pwmchip-inexistente";
    cfg.right_a.chip = cfg.right_b.chip = "/sys/class/pwm/pwmchip-inexistente";
    SysfsPwmSink sink(cfg);

    CHECK(!sink.open());
    CHECK(sink.last_error().find("dtoverlay=pwm-2chan") != std::string::npos);
    // Sink fechado ignora comandos em vez de escrever em fd inválido.
    sink.write(MotorCommand::stopped());
    CHECK_EQ(sink.write_failures(), 0u);
    CHECK_EQ(sink.writes_issued(), 0u);
}
