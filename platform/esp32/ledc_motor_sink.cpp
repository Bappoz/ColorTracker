#include "ledc_motor_sink.hpp"

#include "esp_log.h"

namespace ecv::esp32 {
namespace {
constexpr const char* kTag = "ecv.ledc";
}

bool LedcMotorSink::configure_channel(int index, int gpio) {
    ledc_channel_config_t ch = {};
    ch.gpio_num = gpio;
    ch.speed_mode = LEDC_LOW_SPEED_MODE;  // único modo presente no S3/C3
    ch.channel = static_cast<ledc_channel_t>(cfg_.first_channel + index);
    ch.timer_sel = cfg_.timer;
    ch.duty = 0;
    ch.hpoint = 0;
    ch.intr_type = LEDC_INTR_DISABLE;

    const esp_err_t err = ledc_channel_config(&ch);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "ledc_channel_config(gpio %d) falhou: 0x%x", gpio, err);
        return false;
    }
    return true;
}

bool LedcMotorSink::open() {
    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = cfg_.resolution;
    timer.timer_num = cfg_.timer;
    timer.freq_hz = cfg_.freq_hz;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "ledc_timer_config falhou: 0x%x", err);
        return false;
    }

    max_ticks_ = (1u << static_cast<uint32_t>(cfg_.resolution)) - 1u;
    const int gpios[4] = {cfg_.gpio_left_a, cfg_.gpio_left_b, cfg_.gpio_right_a, cfg_.gpio_right_b};
    for (int i = 0; i < 4; ++i) {
        if (!configure_channel(i, gpios[i])) return false;
    }

    open_ = true;
    stop();
    return true;
}

void LedcMotorSink::close() {
    if (!open_) return;
    stop();
    open_ = false;
}

void LedcMotorSink::set_duty(int index, float normalized) {
    if (!open_) return;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > cfg_.max_duty) normalized = cfg_.max_duty;

    const auto channel = static_cast<ledc_channel_t>(cfg_.first_channel + index);
    const uint32_t ticks = static_cast<uint32_t>(normalized * static_cast<float>(max_ticks_));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, ticks);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void LedcMotorSink::write(const MotorCommand& cmd) {
    // Sentido = qual entrada da ponte recebe o duty; a outra vai a zero.
    set_duty(0, cmd.left > 0 ? cmd.left : 0.0f);
    set_duty(1, cmd.left < 0 ? -cmd.left : 0.0f);
    set_duty(2, cmd.right > 0 ? cmd.right : 0.0f);
    set_duty(3, cmd.right < 0 ? -cmd.right : 0.0f);
}

void LedcMotorSink::stop() {
    for (int i = 0; i < 4; ++i) set_duty(i, 0.0f);
}

}  // namespace ecv::esp32
