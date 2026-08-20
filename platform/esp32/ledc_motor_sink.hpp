// PWM por LEDC para ponte H de duas entradas por motor (DRV8833, TB6612).
//
// Timer 1 e canais 2..5 por padrão: o driver da câmera ocupa o timer 0 e o
// canal 0 gerando o XCLK do sensor. Reutilizar o timer 0 aqui derruba a câmera.
#pragma once

#include "driver/ledc.h"
#include "ecv/hal/motor_sink.hpp"

namespace ecv::esp32 {

struct LedcMotorConfig {
    int gpio_left_a = 12;
    int gpio_left_b = 13;
    int gpio_right_a = 14;
    int gpio_right_b = 15;

    ledc_timer_t timer = LEDC_TIMER_1;
    ledc_channel_t first_channel = LEDC_CHANNEL_2;
    /// 20 kHz fica acima da faixa audível e ainda dá 12 bits de resolução com
    /// o clock de 80 MHz do periférico (80e6 / 2^12 ≈ 19,5 kHz).
    uint32_t freq_hz = 19531;
    ledc_timer_bit_t resolution = LEDC_TIMER_12_BIT;
    float max_duty = 1.0f;
};

class LedcMotorSink : public MotorSink {
public:
    explicit LedcMotorSink(const LedcMotorConfig& cfg) : cfg_(cfg) {}
    ~LedcMotorSink() override { close(); }

    bool open() override;
    void close() override;
    void write(const MotorCommand& cmd) override;
    void stop() override;
    const char* name() const override { return "ledc"; }

private:
    bool configure_channel(int index, int gpio);
    void set_duty(int index, float normalized);

    LedcMotorConfig cfg_;
    uint32_t max_ticks_ = 0;
    bool open_ = false;
};

}  // namespace ecv::esp32
