// Mixer diferencial: (avanço, giro) -> (esquerda, direita).
#pragma once

#include <cstdint>

#include "ecv/core/fixed.hpp"

namespace ecv {

/// Comandos normalizados em [-1, 1]. Negativo = ré.
struct MotorCommand {
    float left = 0.0f;
    float right = 0.0f;

    static MotorCommand stopped() { return MotorCommand{}; }
};

/// Sinal pronto para a ponte H: magnitude no domínio do timer + sentido.
struct PwmDuty {
    uint16_t magnitude = 0;
    bool reverse = false;
};

inline PwmDuty to_pwm(float normalized, uint16_t max_duty) {
    const bool reverse = normalized < 0.0f;
    float m = reverse ? -normalized : normalized;
    if (m > 1.0f) m = 1.0f;
    return PwmDuty{static_cast<uint16_t>(m * max_duty + 0.5f), reverse};
}

struct DifferentialConfig {
    float max_speed = 1.0f;
    /// Duty mínimo que vence o atrito estático do conjunto motor+redução.
    /// Sem isso, todo comando pequeno vira apenas motor zumbindo parado, e o
    /// PD nunca fecha o erro residual. Medir na bancada, não chutar.
    float deadband = 0.0f;
    float turn_gain = 1.0f;
};

class DifferentialMixer {
public:
    void configure(const DifferentialConfig& c) { cfg_ = c; }
    const DifferentialConfig& config() const { return cfg_; }

    MotorCommand mix(float forward, float turn) const {
        float l = forward + cfg_.turn_gain * turn;
        float r = forward - cfg_.turn_gain * turn;

        // Satura escalando os dois lados juntos: cortar só o lado saturado
        // mudaria o raio da curva justamente quando o giro é mais agressivo.
        const float al = l < 0 ? -l : l;
        const float ar = r < 0 ? -r : r;
        const float peak = al > ar ? al : ar;
        if (peak > cfg_.max_speed && peak > 0.0f) {
            const float k = cfg_.max_speed / peak;
            l *= k;
            r *= k;
        }
        return MotorCommand{apply_deadband(l), apply_deadband(r)};
    }

private:
    float apply_deadband(float v) const {
        if (cfg_.deadband <= 0.0f) return v;
        const float a = v < 0 ? -v : v;
        if (a < 1e-3f) return 0.0f;
        const float scaled = cfg_.deadband + (cfg_.max_speed - cfg_.deadband) * a / cfg_.max_speed;
        return v < 0 ? -scaled : scaled;
    }

    DifferentialConfig cfg_;
};

}  // namespace ecv
