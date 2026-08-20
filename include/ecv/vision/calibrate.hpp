// Estimativa da faixa HSV de um alvo a partir de um recorte da imagem.
//
// Vive no núcleo (e não numa ferramenta de desktop) porque roda por histograma,
// sem ordenar e sem alocar: dá para calibrar no próprio robô, com a câmera dele
// e a luz do dia da competição, que é quando a calibração vale de fato.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"
#include "ecv/vision/threshold.hpp"

namespace ecv {

struct CalibrationParams {
    /// Fração das amostras que a faixa precisa cobrir.
    float coverage = 0.9f;
    /// Saturação mínima para um pixel entrar no histograma de matiz. O matiz de
    /// um pixel quase-cinza é ruído de divisão — contá-lo espalha a faixa por
    /// cores que o alvo não tem. Todo recorte feito à mão pega fundo nos cantos.
    uint8_t chroma_floor = 40;
    /// Folga aplicada ao matiz nas duas pontas: a foto é uma amostra de uma
    /// iluminação só, e o matiz medido desloca alguns graus com a luz.
    uint8_t hue_margin = 3;
};

struct CalibrationResult {
    HsvRange range;
    uint32_t samples = 0;    ///< pixels no recorte
    uint32_t chromatic = 0;  ///< pixels acima de `chroma_floor`
    uint32_t hue_span = 0;   ///< largura do arco de matiz antes da margem
    /// Recorte pegou mais fundo que alvo: a estimativa caiu no modo degradado
    /// (todos os pixels) e provavelmente não presta.
    bool low_chroma = false;
};

/// Histogramas de trabalho — 2,7 KB que o chamador é dono, como o resto.
struct CalibrationWorkspace {
    uint32_t hue[180];
    uint32_t sat[256];
    uint32_t val[256];
};

/// Duas passadas sobre `roi`: a primeira acha o menor arco circular de matiz que
/// cobre `coverage` das amostras cromáticas; a segunda mede S e V apenas nos
/// pixels desse arco — é o que separa o alvo do fundo que sobrou no recorte.
CalibrationResult estimate_hsv_range(const ImageView& img, const Rect& roi,
                                     CalibrationWorkspace& ws,
                                     const CalibrationParams& params = {});

/// Fração dos pixels FORA de `roi` que também casam com `range`. Alta significa
/// que a cor do alvo não é separável da cena e nenhuma morfologia resolve.
float false_positive_rate(const ImageView& img, const Rect& roi, const HsvRange& range);

}  // namespace ecv
