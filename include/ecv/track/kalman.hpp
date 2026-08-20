// Filtro de Kalman de velocidade constante, desacoplado por eixo.
//
// Por que não uma matriz 4x4: com Q e R diagonais (ruído de X independente do de
// Y, que é a hipótese razoável para um oponente visto de cima), a matriz de
// covariância 4x4 é bloco-diagonal e o filtro se fatora em dois filtros de 2
// estados. Cada um tem 3 elementos únicos de P por simetria. Resultado: ~20
// operações de ponto flutuante por frame em vez de duas inversões 4x4 e nenhuma
// biblioteca de álgebra linear no firmware.
#pragma once

#include "ecv/core/types.hpp"

namespace ecv {

/// Estado [posição, velocidade] de um eixo. Unidades: pixels e pixels/s.
class Kalman1D {
public:
    /// `accel_var`: variância do ruído de aceleração (px/s²)² — quão brusco o
    /// alvo pode mudar de velocidade. `meas_var`: variância da medida (px²).
    void configure(float accel_var, float meas_var) {
        accel_var_ = accel_var;
        meas_var_ = meas_var;
    }

    void reset(float pos) {
        x_ = pos;
        v_ = 0.0f;
        p00_ = 1000.0f;  // desconfia da posição inicial
        p01_ = 0.0f;
        p11_ = 1000.0f;
        initialized_ = true;
    }

    /// Descarta o estado: a próxima medida reinicializa o filtro. Usar quando o
    /// alvo ficou perdido tempo demais — extrapolar velocidade antiga a partir
    /// daí só produz um alvo fantasma.
    void invalidate() { initialized_ = false; }

    bool initialized() const { return initialized_; }
    float position() const { return x_; }
    float velocity() const { return v_; }
    float variance() const { return p00_; }

    /// Modelo de aceleração contínua branca: Q = accel_var * [[dt³/3, dt²/2],
    ///                                                        [dt²/2, dt   ]].
    void predict(float dt) {
        if (!initialized_ || dt <= 0.0f) return;
        x_ += v_ * dt;

        const float p00 = p00_ + dt * (2.0f * p01_ + dt * p11_);
        const float p01 = p01_ + dt * p11_;
        const float p11 = p11_;

        const float dt2 = dt * dt;
        p00_ = p00 + accel_var_ * dt2 * dt / 3.0f;
        p01_ = p01 + accel_var_ * dt2 / 2.0f;
        p11_ = p11 + accel_var_ * dt;
    }

    void update(float z) {
        if (!initialized_) {
            reset(z);
            return;
        }
        const float s = p00_ + meas_var_;
        const float k0 = p00_ / s;
        const float k1 = p01_ / s;
        const float innovation = z - x_;

        x_ += k0 * innovation;
        v_ += k1 * innovation;

        const float p01_old = p01_;
        p00_ -= k0 * p00_;
        p01_ -= k0 * p01_;
        p11_ -= k1 * p01_old;
    }

    /// Extrapola `dt` segundos à frente sem alterar o estado (para mira preditiva).
    float project(float dt) const { return x_ + v_ * dt; }

private:
    float x_ = 0.0f, v_ = 0.0f;
    float p00_ = 1000.0f, p01_ = 0.0f, p11_ = 1000.0f;
    float accel_var_ = 2000.0f;
    float meas_var_ = 9.0f;
    bool initialized_ = false;
};

class Kalman2D {
public:
    void configure(float accel_var, float meas_var) {
        x_.configure(accel_var, meas_var);
        y_.configure(accel_var, meas_var);
    }
    void reset(Point p) {
        x_.reset(static_cast<float>(p.x));
        y_.reset(static_cast<float>(p.y));
    }
    bool initialized() const { return x_.initialized(); }
    void invalidate() {
        x_.invalidate();
        y_.invalidate();
    }

    void predict(float dt) {
        x_.predict(dt);
        y_.predict(dt);
    }
    void update(Point z) {
        x_.update(static_cast<float>(z.x));
        y_.update(static_cast<float>(z.y));
    }

    Point position() const {
        return Point{static_cast<int16_t>(x_.position()), static_cast<int16_t>(y_.position())};
    }
    Point project(float dt) const {
        return Point{static_cast<int16_t>(x_.project(dt)), static_cast<int16_t>(y_.project(dt))};
    }
    /// Incerteza posicional agregada — cresce durante oclusão e é o gatilho
    /// natural para desistir da predição e voltar a varrer o frame inteiro.
    float uncertainty() const { return x_.variance() + y_.variance(); }

    const Kalman1D& axis_x() const { return x_; }
    const Kalman1D& axis_y() const { return y_; }

private:
    Kalman1D x_, y_;
};

}  // namespace ecv
