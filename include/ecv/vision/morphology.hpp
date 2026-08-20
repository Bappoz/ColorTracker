// Morfologia binária 3x3, in-place, separável.
//
// Duas escolhas que importam no embarcado:
//  1. Kernel retangular é separável para min/max -> 2*3 comparações por pixel em
//     vez de 9. Para 320x240 isso corta ~350k comparações por operação.
//  2. Nenhuma imagem temporária: a passada horizontal guarda o vizinho da
//     esquerda em registrador e a vertical guarda uma única linha original em
//     `scratch` (width bytes), não um frame inteiro.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"

namespace ecv {

/// `scratch` precisa ter pelo menos `mask.width` bytes. Borda replicada.
void erode3(MaskView& mask, uint8_t* scratch);
void dilate3(MaskView& mask, uint8_t* scratch);

/// Abertura: remove ruído sal (pixels isolados) sem encolher o alvo.
void open3(MaskView& mask, uint8_t* scratch, int iterations = 1);

/// Fechamento: preenche buracos internos do alvo (reflexo especular, etc).
void close3(MaskView& mask, uint8_t* scratch, int iterations = 1);

}  // namespace ecv
