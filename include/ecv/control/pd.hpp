// Controlador PD genérico no tipo escalar (float ou ecv::Fixed).
//
// PD e não PID de propósito: o termo integral acumula erro enquanto o robô está
// fisicamente travado contra o oponente — situação normal em sumô — e o windup
// resultante faz o robô girar sozinho quando o contato acaba.
#pragma once

#include "ecv/core/fixed.hpp"

namespace ecv {

template <typename T = float>
class PdController {
public:
    struct Config {
        float kp = 1.0f;
        float kd = 0.0f;
        float out_min = -1.0f;
        float out_max = 1.0f;
        /// Suavização exponencial da derivada em (0, 1]. 1 = sem filtro. A
        /// derivada de um centroide de visão é ruidosa: o centroide salta alguns
        /// pixels por frame só por causa do limiar, e kd amplifica isso.
        float d_alpha = 0.4f;
    };

    void configure(const Config& c) {
        using S = ScalarTraits<T>;
        kp_ = S::from_float(c.kp);
        kd_ = S::from_float(c.kd);
        out_min_ = S::from_float(c.out_min);
        out_max_ = S::from_float(c.out_max);
        d_alpha_ = S::from_float(c.d_alpha);
        one_minus_alpha_ = S::from_float(1.0f - c.d_alpha);
        reset();
    }

    void reset() {
        prev_error_ = T{};
        d_filtered_ = T{};
        primed_ = false;
    }

    /// `dt` em segundos. No primeiro frame a derivada é ignorada (não há passado).
    T update(T error, T dt) {
        T derivative = T{};
        if (primed_ && dt > T{}) {
            const T raw = (error - prev_error_) / dt;
            d_filtered_ = d_alpha_ * raw + one_minus_alpha_ * d_filtered_;
            derivative = d_filtered_;
        }
        prev_error_ = error;
        primed_ = true;

        const T out = kp_ * error + kd_ * derivative;
        return clamp(out, out_min_, out_max_);
    }

    T derivative() const { return d_filtered_; }

private:
    T kp_{}, kd_{}, out_min_{}, out_max_{}, d_alpha_{}, one_minus_alpha_{};
    T prev_error_{}, d_filtered_{};
    bool primed_ = false;
};

}  // namespace ecv
