// Ponto fixo Q16.16 sobre int32.
//
// Por que existe: ESP32-C3/C6 e a maioria dos Cortex-M0/M3 não têm FPU — cada
// operação float vira chamada de biblioteca (~50-100 ciclos). O ESP32 clássico e
// o S3 têm FPU de precisão simples, então lá `float` é aceitável. O controle PD é
// templated no escalar justamente para trocar um pelo outro sem tocar na lógica.
//
// Faixa: ±32767.999985, resolução 1/65536.
#pragma once

#include <cstdint>

namespace ecv {

class Fixed {
public:
    static constexpr int kShift = 16;
    static constexpr int32_t kOne = 1 << kShift;

    constexpr Fixed() = default;
    static constexpr Fixed from_raw(int32_t raw) {
        Fixed f;
        f.raw_ = raw;
        return f;
    }
    static constexpr Fixed from_int(int32_t v) { return from_raw(v << kShift); }
    static constexpr Fixed from_float(float v) {
        return from_raw(static_cast<int32_t>(v * kOne + (v >= 0 ? 0.5f : -0.5f)));
    }

    constexpr int32_t raw() const { return raw_; }
    constexpr float to_float() const { return static_cast<float>(raw_) / kOne; }
    constexpr int32_t to_int() const { return raw_ >> kShift; }  // floor

    constexpr Fixed operator-() const { return from_raw(-raw_); }
    constexpr Fixed operator+(Fixed o) const { return from_raw(raw_ + o.raw_); }
    constexpr Fixed operator-(Fixed o) const { return from_raw(raw_ - o.raw_); }
    constexpr Fixed operator*(Fixed o) const {
        // O produto intermediário precisa de 64 bits: Q16.16 * Q16.16 = Q32.32.
        return from_raw(saturate((static_cast<int64_t>(raw_) * o.raw_ + (kOne / 2)) >> kShift));
    }
    constexpr Fixed operator/(Fixed o) const {
        // Satura em vez de estourar: num laço de controle, saturar vira comando
        // no limite do motor; estourar vira comando no sentido oposto.
        if (o.raw_ == 0) return from_raw(raw_ >= 0 ? INT32_MAX : INT32_MIN);
        return from_raw(saturate((static_cast<int64_t>(raw_) << kShift) / o.raw_));
    }

    Fixed& operator+=(Fixed o) { return *this = *this + o; }
    Fixed& operator-=(Fixed o) { return *this = *this - o; }
    Fixed& operator*=(Fixed o) { return *this = *this * o; }

    constexpr bool operator<(Fixed o) const { return raw_ < o.raw_; }
    constexpr bool operator>(Fixed o) const { return raw_ > o.raw_; }
    constexpr bool operator<=(Fixed o) const { return raw_ <= o.raw_; }
    constexpr bool operator>=(Fixed o) const { return raw_ >= o.raw_; }
    constexpr bool operator==(Fixed o) const { return raw_ == o.raw_; }
    constexpr bool operator!=(Fixed o) const { return raw_ != o.raw_; }

private:
    static constexpr int32_t saturate(int64_t v) {
        if (v > INT32_MAX) return INT32_MAX;
        if (v < INT32_MIN) return INT32_MIN;
        return static_cast<int32_t>(v);
    }

    int32_t raw_ = 0;
};

constexpr Fixed abs(Fixed v) {
    return v < Fixed() ? -v : v;
}

template <typename T>
constexpr T clamp(T v, T lo, T hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/// Traits para o código genérico (PD, mixer) construir constantes sem saber o tipo.
template <typename T>
struct ScalarTraits {
    static constexpr T from_float(float v) { return static_cast<T>(v); }
    static constexpr float to_float(T v) { return static_cast<float>(v); }
};

template <>
struct ScalarTraits<Fixed> {
    static constexpr Fixed from_float(float v) { return Fixed::from_float(v); }
    static constexpr float to_float(Fixed v) { return v.to_float(); }
};

}  // namespace ecv
