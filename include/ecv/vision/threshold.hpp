// Limiarização de cor em HSV, fundida com a conversão de espaço de cor.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"
#include "ecv/vision/color.hpp"

namespace ecv {

/// Faixa HSV inclusiva. Se `h_min > h_max` a faixa dá a volta em 180 (vermelho),
/// que é o caso mais comum em robô de sumô com marcador vermelho.
struct HsvRange {
    uint8_t h_min = 0, h_max = 179;
    uint8_t s_min = 0, s_max = 255;
    uint8_t v_min = 0, v_max = 255;

    bool wraps() const { return h_min > h_max; }
};

inline bool in_range(Hsv p, const HsvRange& r) {
    const bool h_ok =
        r.wraps() ? (p.h >= r.h_min || p.h <= r.h_max) : (p.h >= r.h_min && p.h <= r.h_max);
    return h_ok && p.s >= r.s_min && p.s <= r.s_max && p.v >= r.v_min && p.v <= r.v_max;
}

/// Conversão de espaço de cor + limiarização em uma única passada.
///
/// A BPMN separa "Converter Espaço de Cor" de "Limiarização"; manter os dois
/// estágios separados exigiria um buffer HSV intermediário de 3 bytes/px
/// (225 KB em 320x240) e uma segunda varredura da imagem. Fundindo, o HSV vive
/// só em registrador e a máscara é o único buffer. `dst` precisa ter as mesmas
/// dimensões de `src`.
void threshold_hsv(const ImageView& src, const HsvRange& range, MaskView& dst);

/// Variante que também devolve quantos pixels passaram — útil para auto-exposição
/// e para detectar limiar mal calibrado sem uma segunda varredura.
uint32_t threshold_hsv_count(const ImageView& src, const HsvRange& range, MaskView& dst);

/// Tabela de decisão indexada pelo pixel RGB565 inteiro: 65.536 entradas de 1
/// bit = 8 KB.
///
/// A conversão RGB->HSV por pixel custa duas divisões inteiras. Num Cortex-A é
/// caro; num ESP32 é proibitivo. Como o espaço RGB565 tem só 65.536 valores
/// possíveis, dá para responder "esse pixel é do alvo?" com um deslocamento e
/// um teste de bit — o HSV é calculado 65.536 vezes na inicialização e nunca
/// mais. Formatos de 24 bits são quantizados para 565 antes da consulta; o erro
/// de quantização é menor que o ruído do sensor.
class ColorLut565 {
public:
    void build(const HsvRange& range);

    static constexpr uint16_t quantize(Rgb c) {
        return static_cast<uint16_t>(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
    }

    bool contains(uint16_t px565) const { return (bits_[px565 >> 3] >> (px565 & 7)) & 1u; }
    bool contains(Rgb c) const { return contains(quantize(c)); }
    bool built() const { return built_; }

private:
    uint8_t bits_[8192] = {};
    bool built_ = false;
};

/// Mesma saída de `threshold_hsv`, sem nenhuma divisão no laço de pixel.
void threshold_lut(const ImageView& src, const ColorLut565& lut, MaskView& dst);

}  // namespace ecv
