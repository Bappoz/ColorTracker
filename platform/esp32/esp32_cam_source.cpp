#include "esp32_cam_source.hpp"

#include "esp_camera.h"
#include "esp_log.h"

namespace ecv::esp32 {
namespace {

constexpr const char* kTag = "ecv.cam";

framesize_t frame_size_for(int32_t w, int32_t h) {
    if (w <= 96 && h <= 96) return FRAMESIZE_96X96;
    if (w <= 160 && h <= 120) return FRAMESIZE_QQVGA;
    if (w <= 240 && h <= 176) return FRAMESIZE_HQVGA;
    if (w <= 320 && h <= 240) return FRAMESIZE_QVGA;
    return FRAMESIZE_VGA;
}

}  // namespace

void Esp32CamSource::apply_sensor_controls() {
    sensor_t* s = esp_camera_sensor_get();
    if (!s) return;

    if (cfg_.lock_exposure) {
        s->set_whitebal(s, 0);
        s->set_awb_gain(s, 0);
        s->set_exposure_ctrl(s, 0);
        s->set_aec2(s, 0);
        s->set_aec_value(s, cfg_.aec_value);
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, cfg_.agc_gain);
    }
    // Correção gama e saturação altas distorcem o matiz; manter neutro para a
    // faixa HSV calibrada continuar valendo.
    s->set_saturation(s, 0);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
}

bool Esp32CamSource::open() {
    camera_config_t config = {};
    config.pin_pwdn = cfg_.pins.pwdn;
    config.pin_reset = cfg_.pins.reset;
    config.pin_xclk = cfg_.pins.xclk;
    config.pin_sccb_sda = cfg_.pins.sccb_sda;
    config.pin_sccb_scl = cfg_.pins.sccb_scl;
    config.pin_d7 = cfg_.pins.d7;
    config.pin_d6 = cfg_.pins.d6;
    config.pin_d5 = cfg_.pins.d5;
    config.pin_d4 = cfg_.pins.d4;
    config.pin_d3 = cfg_.pins.d3;
    config.pin_d2 = cfg_.pins.d2;
    config.pin_d1 = cfg_.pins.d1;
    config.pin_d0 = cfg_.pins.d0;
    config.pin_vsync = cfg_.pins.vsync;
    config.pin_href = cfg_.pins.href;
    config.pin_pclk = cfg_.pins.pclk;

    config.xclk_freq_hz = cfg_.xclk_freq_hz;
    // Timer/canal do XCLK: o driver ocupa estes. O LedcMotorSink usa timer 1 e
    // canais 2..5 justamente para não colidir.
    config.ledc_timer = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;

    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = frame_size_for(cfg_.width, cfg_.height);
    config.fb_count = static_cast<size_t>(cfg_.fb_count);
    config.fb_location = cfg_.use_psram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.sccb_i2c_port = -1;

    const esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        last_error_ = static_cast<int>(err);
        ESP_LOGE(kTag, "esp_camera_init falhou: 0x%x", err);
        return false;
    }

    apply_sensor_controls();
    open_ = true;
    return true;
}

void Esp32CamSource::close() {
    if (pending_fb_) {
        esp_camera_fb_return(static_cast<camera_fb_t*>(pending_fb_));
        pending_fb_ = nullptr;
    }
    if (open_) {
        esp_camera_deinit();
        open_ = false;
    }
}

bool Esp32CamSource::next(ImageView& out) {
    if (!open_) return false;

    // Devolve o buffer anterior só agora: enquanto a visão processava, o driver
    // seguiu preenchendo o outro buffer.
    if (pending_fb_) {
        esp_camera_fb_return(static_cast<camera_fb_t*>(pending_fb_));
        pending_fb_ = nullptr;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        last_error_ = -1;
        return false;
    }
    pending_fb_ = fb;

    out = ImageView{fb->buf, static_cast<int32_t>(fb->width), static_cast<int32_t>(fb->height),
                    static_cast<int32_t>(fb->width) * 2, PixelFormat::kRgb565};
    return true;
}

}  // namespace ecv::esp32
