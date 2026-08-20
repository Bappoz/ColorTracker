// Varredura de poucas linhas horizontais — a primitiva mais barata do repositório.
//
// Serve a dois sistemas com o mesmo código e a mesma passada:
//  * borda do dohyo (linha branca): quantos pixels acendem em cada terço da
//    imagem -> de que lado o robô está saindo do ringue;
//  * seguidor de linha: centroide horizontal dos pixels acesos -> erro lateral.
//
// Custo: `rows` * width pixels (ex.: 4*320 = 1280) contra 76.800 do frame cheio.
// Cabe folgado entre dois frames mesmo num ESP32-C3 e não depende da máscara.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"
#include "ecv/vision/threshold.hpp"

namespace ecv {

struct LineScanConfig {
    HsvRange target;         ///< branco típico do dohyo: s_max baixo, v_min alto
    int16_t y_top = 0;       ///< primeira linha amostrada
    int16_t y_bottom = 0;    ///< última linha amostrada (inclusiva)
    uint8_t rows = 4;        ///< quantas linhas entre y_top e y_bottom
    uint16_t min_hits = 12;  ///< pixels acesos para considerar detecção válida
};

struct LineScanResult {
    uint32_t hits = 0;
    uint32_t zone_hits[3] = {0, 0, 0};  ///< esquerda, centro, direita
    int16_t center_x = 0;               ///< centroide horizontal dos pixels acesos
    bool found = false;

    bool left() const { return zone_hits[0] * 3 > hits && found; }
    bool center() const { return zone_hits[1] * 3 > hits && found; }
    bool right() const { return zone_hits[2] * 3 > hits && found; }
};

/// Varre `cfg.rows` linhas igualmente espaçadas em [y_top, y_bottom].
LineScanResult scan_lines(const ImageView& frame, const LineScanConfig& cfg);

/// Erro lateral normalizado em [-1, 1] (negativo = linha à esquerda do centro).
/// Retorna 0 quando não houve detecção.
float line_error(const LineScanResult& r, int32_t frame_width);

}  // namespace ecv
