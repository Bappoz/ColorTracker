// PWM por sysfs para ponte H de duas entradas por motor (DRV8833, TB6612).
//
// Quatro canais de PWM e nenhum GPIO de direção: o sentido é escolhido por qual
// das duas entradas recebe o duty (a outra fica em 0). Isso evita depender de
// libgpiod e é o esquema que essas pontes esperam.
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

private:
    struct Channel {
        int duty_fd = -1;
        std::string path;
    };

    bool export_channel(const PwmChannel& ch, Channel& out);
    void write_duty(Channel& ch, float normalized);

    PwmSinkConfig cfg_;
    Channel la_, lb_, ra_, rb_;
    std::string error_;
    bool open_ = false;
};

}  // namespace ecv::linux_hal
