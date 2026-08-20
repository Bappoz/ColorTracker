#include "ecv/app/sumo_vision.hpp"

#include "ecv/vision/morphology.hpp"

namespace ecv {

const char* track_state_name(TrackState s) {
    switch (s) {
        case TrackState::kSearching: return "SEARCHING";
        case TrackState::kTracking: return "TRACKING";
        case TrackState::kCoasting: return "COASTING";
    }
    return "?";
}

namespace {

float fabs_f(float v) {
    return v < 0.0f ? -v : v;
}

}  // namespace

SumoVision::SumoVision(const SumoConfig& cfg, const Buffers& buf)
    : cfg_(cfg), buf_(buf), roi_(cfg.frame_w, cfg.frame_h, cfg.roi_padding, cfg.roi_min_side) {
    kalman_.configure(cfg_.accel_var, cfg_.meas_var);
    pd_.configure(cfg_.pd);
    mixer_.configure(cfg_.drive);
    if (buf_.lut) buf_.lut->build(cfg_.target);
    reset();
}

void SumoVision::set_target_range(const HsvRange& r) {
    cfg_.target = r;
    if (buf_.lut) buf_.lut->build(r);
}

void SumoVision::reset() {
    roi_.reset();
    kalman_.invalidate();
    pd_.reset();
    lost_time_s_ = 0.0f;
}

SumoResult SumoVision::process(const ImageView& frame, float dt) {
    SumoResult r;
    const uint64_t t_start = micros();
    if (!frame.valid() || !buf_.mask.valid()) return r;

    // --- Gateway_Tracking + Task_CropROI / Task_FullFrame -------------------
    // O recorte é aritmética de ponteiro: nenhum pixel é copiado.
    const Rect roi = roi_.current();
    r.roi = roi;
    const ImageView view = crop(frame, roi);
    MaskView mask = crop(
        buf_.mask, Rect{0, 0, static_cast<int16_t>(view.width), static_cast<int16_t>(view.height)});

    // --- Borda do ringue ----------------------------------------------------
    // Sempre no frame cheio: a linha branca pode aparecer fora da ROI do alvo, e
    // sair do dohyo perde a luta independentemente de onde está o oponente.
    if (cfg_.border_enabled) {
        ScopedStage s(r.timings, kStageBorder);
        r.border = scan_lines(frame, cfg_.border);
    }

    // --- Task_ColorConvert + Task_Threshold (fundidos) ----------------------
    {
        ScopedStage s(r.timings, kStageThreshold);
        if (buf_.lut && buf_.lut->built()) {
            threshold_lut(view, *buf_.lut, mask);
        } else {
            threshold_hsv(view, cfg_.target, mask);
        }
    }

    // --- Task_Morph ---------------------------------------------------------
    {
        ScopedStage s(r.timings, kStageMorphology);
        if (cfg_.iterations_open > 0) open3(mask, buf_.morph_scratch, cfg_.iterations_open);
        if (cfg_.iterations_close > 0) close3(mask, buf_.morph_scratch, cfg_.iterations_close);
    }

    // --- Task_FindContours + Gateway_Area + Task_Centroid -------------------
    int32_t best = -1;
    {
        ScopedStage s(r.timings, kStageBlobs);
        const int32_t n =
            find_blobs(mask, buf_.blobs, cfg_.min_area, buf_.blob_out, buf_.max_blobs);
        best = largest_blob(buf_.blob_out, n);
    }

    // --- Task_CoordConvert / Task_UpdateROI / Task_Kalman -------------------
    {
        ScopedStage s(r.timings, kStageTrack);
        if (best >= 0) {
            const Blob& b = buf_.blob_out[best];
            r.detected = true;
            r.area = b.area;
            r.box = roi_.to_global(b.box);  // Task_CoordConvert
            const Point measured = roi_.to_global(b.centroid);

            // Desvio consciente da BPMN: o Kalman também é alimentado quando HÁ
            // detecção. Na BPMN ele só aparece no ramo de falha, mas um filtro
            // que nunca recebe medida não tem estado nenhum para prever — a
            // velocidade estimada vem justamente da sequência de acertos.
            if (!kalman_.initialized()) {
                kalman_.reset(measured);
            } else {
                kalman_.predict(dt);
                kalman_.update(measured);
            }

            roi_.update(r.box);  // Task_UpdateROI
            lost_time_s_ = 0.0f;
            r.state = TrackState::kTracking;
            r.target =
                cfg_.aim_lead_s > 0.0f ? kalman_.project(cfg_.aim_lead_s) : kalman_.position();
        } else {
            roi_.reset();  // Task_ResetTracking
            lost_time_s_ += dt;
            if (kalman_.initialized() && lost_time_s_ <= cfg_.coast_timeout_s) {
                kalman_.predict(dt);
                r.state = TrackState::kCoasting;
                r.target = kalman_.project(cfg_.aim_lead_s);
            } else {
                kalman_.invalidate();
                r.state = TrackState::kSearching;
            }
        }
    }

    // --- Task_ErrorCalc + Task_PDControl ------------------------------------
    {
        ScopedStage s(r.timings, kStageControl);
        const float half = static_cast<float>(cfg_.frame_w) * 0.5f;
        float forward = 0.0f;
        float turn = 0.0f;

        if (r.state == TrackState::kSearching) {
            pd_.reset();  // o histórico da derivada não vale mais nada
            turn = cfg_.search_turn;
        } else {
            r.error_norm = (static_cast<float>(r.target.x) - half) / half;
            turn = pd_.update(r.error_norm, dt);
            float align = 1.0f - cfg_.align_gain * fabs_f(r.error_norm);
            if (align < 0.0f) align = 0.0f;
            forward = cfg_.attack_speed * align;
        }

        if (r.border_hit()) {
            // Sobrepõe qualquer decisão de ataque: recua e gira para o lado
            // oposto ao da borda detectada.
            forward = -cfg_.retreat_speed;
            turn =
                r.border.left() ? cfg_.search_turn : (r.border.right() ? -cfg_.search_turn : 0.0f);
        }

        r.cmd = mixer_.mix(forward, turn);
    }

    r.timings.total_us = static_cast<uint32_t>(micros() - t_start);
    return r;
}

}  // namespace ecv
