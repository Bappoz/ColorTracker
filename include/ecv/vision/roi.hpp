// ROI dinâmica: processa só a vizinhança do alvo enquanto ele está rastreado.
//
// É o maior ganho de latência do pipeline inteiro. Com o alvo ocupando ~60x60 px
// e padding de 40, a ROI tem ~140x140 = 19.600 px contra 76.800 do frame cheio:
// ~4x menos trabalho em limiarização, morfologia e rotulagem.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"

namespace ecv {

class RoiTracker {
public:
    RoiTracker() = default;
    RoiTracker(int16_t frame_w, int16_t frame_h, int16_t padding, int16_t min_side = 24)
        : frame_w_(frame_w), frame_h_(frame_h), padding_(padding), min_side_(min_side) {}

    bool active() const { return active_; }
    const Rect& roi() const { return roi_; }

    /// ROI a usar neste frame: a rastreada, ou o frame inteiro se perdeu o alvo.
    Rect current() const { return active_ ? roi_ : Rect{0, 0, frame_w_, frame_h_}; }

    /// Recentra a ROI na bounding box do alvo (coordenadas globais).
    void update(const Rect& target_box) {
        Rect r = target_box;
        if (r.w < min_side_) {
            r.x = static_cast<int16_t>(r.x - (min_side_ - r.w) / 2);
            r.w = min_side_;
        }
        if (r.h < min_side_) {
            r.y = static_cast<int16_t>(r.y - (min_side_ - r.h) / 2);
            r.h = min_side_;
        }
        roi_ = expand_and_clamp(r, padding_, frame_w_, frame_h_);
        active_ = !roi_.empty();
    }

    /// Perdeu o alvo: próximo frame volta a varrer o frame inteiro.
    void reset() {
        active_ = false;
        roi_ = Rect{0, 0, frame_w_, frame_h_};
    }

    /// Coordenada local da ROI -> coordenada global do frame (BPMN: Task_CoordConvert).
    Point to_global(Point local) const {
        const Rect r = current();
        return Point{static_cast<int16_t>(local.x + r.x), static_cast<int16_t>(local.y + r.y)};
    }
    Rect to_global(const Rect& local) const {
        const Rect r = current();
        return Rect{static_cast<int16_t>(local.x + r.x), static_cast<int16_t>(local.y + r.y),
                    local.w, local.h};
    }

private:
    Rect roi_;
    int16_t frame_w_ = 0;
    int16_t frame_h_ = 0;
    int16_t padding_ = 40;
    int16_t min_side_ = 24;
    bool active_ = false;
};

}  // namespace ecv
