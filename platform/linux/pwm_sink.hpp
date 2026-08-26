// PWM por sysfs para ponte H de duas entradas por motor (DRV8833, TB6612).
//
// Quatro canais de PWM e nenhum GPIO de direção: o sentido é escolhido por qual
// das duas entradas recebe o duty (a outra fica em 0). Isso evita depender de
// libgpiod e é o esquema que essas pontes esperam.
//
// ATENÇÃO: exige quatro canais de PWM expostos em /sys/class/pwm. Um Raspberry
// Pi 3 tem só dois canais de hardware, e mesmo esses só aparecem com
// `dtoverlay=pwm-2chan` no config.txt. Ver docs/MOTORES.md antes de ligar fio.
#pragma once

#include <cstdint>
#include <string>

#include "ecv/hal/motor_sink.hpp"

namespace ecv::linux_hal {

struct PwmChannel {
    std::string chip = "/sys/class/pwm/pwmchip0";
    int index = 0;
};

struct PwmSinkConfig {
    PwmChannel left_a, left_b, right_a, right_b;
    uint32_t period_ns = 50000;  ///< 20 kHz: acima da audição e dentro do slew da ponte
    /// Duty máximo aplicado (0..1). Limitar aqui protege a bateria e o ESC nos
    /// primeiros testes — subir só depois que o controle estiver estável.
    float max_duty = 1.0f;
};

/// Duty de um canal em nanossegundos, saturado em [0, max_duty]. Puro: é aqui
/// que mora a aritmética, para poder ser testada sem hardware nenhum.
uint32_t duty_ns(float normalized, uint32_t period_ns, float max_duty);

/// Os quatro duties que um comando gera. Em cada motor só uma das entradas
/// recebe duty; a outra fica em zero, e é isso que define o sentido.
struct BridgeDuties {
    uint32_t left_a = 0, left_b = 0, right_a = 0, right_b = 0;
};
BridgeDuties duties_for(const MotorCommand& cmd, uint32_t period_ns, float max_duty);

class SysfsPwmSink : public MotorSink {
public:
    explicit SysfsPwmSink(const PwmSinkConfig& cfg) : cfg_(cfg) {}
    ~SysfsPwmSink() override { close(); }

    bool open() override;
    void close() override;
    void write(const MotorCommand& cmd) override;
    void stop() override;
    const char* name() const override { return "sysfs-pwm"; }

    const std::string& last_error() const { return error_; }
    /// Escritas de duty que o kernel recusou. Diferente de zero significa motor
    /// sem comando — para um robô isso é falha de segurança, não estatística.
    uint32_t write_failures() const { return write_failures_; }
    /// Escritas efetivamente enviadas ao kernel (as redundantes são puladas).
    uint32_t writes_issued() const { return writes_issued_; }

private:
    static constexpr uint32_t kDutyUnknown = 0xFFFFFFFFu;

    struct Channel {
        int duty_fd = -1;
        std::string path;
        uint32_t last_ns = kDutyUnknown;  ///< força a primeira escrita
    };

    bool export_channel(const PwmChannel& ch, Channel& out);
    void write_duty_ns(Channel& ch, uint32_t ns, bool force);

    PwmSinkConfig cfg_;
    Channel la_, lb_, ra_, rb_;
    std::string error_;
    uint32_t write_failures_ = 0;
    uint32_t writes_issued_ = 0;
    bool open_ = false;
};

}  // namespace ecv::linux_hal
