// Calibra a faixa HSV do alvo a partir de uma foto e um retângulo.
//
// Fluxo real: aponta a câmera do robô para o oponente, salva um frame em PPM
// (`ecv_probe --snapshot` faz isso com a câmera de verdade), mede o retângulo do
// marcador num visualizador e roda este programa. A saída é o literal HsvRange
// pronto para colar — em vez de girar sliders até "parecer bom".
//
// A estimativa em si vive em `ecv/vision/calibrate.hpp`, para o mesmo código
// poder rodar no robô.
#include <cstdio>
#include <cstdlib>

#include "ecv/vision/calibrate.hpp"
#include "sim/ppm_io.hpp"

using namespace ecv;

namespace {
CalibrationWorkspace g_workspace;
}

int main(int argc, char** argv) {
    if (argc < 6) {
        std::printf("uso: %s <imagem.ppm> <x> <y> <w> <h> [cobertura=0.9]\n", argv[0]);
        return 1;
    }
    sim::PpmImage img;
    if (!sim::read_ppm(argv[1], img)) {
        std::fprintf(stderr, "não consegui ler %s (só PPM P6 de 8 bits)\n", argv[1]);
        return 1;
    }

    Rect roi;
    roi.x = static_cast<int16_t>(std::atoi(argv[2]));
    roi.y = static_cast<int16_t>(std::atoi(argv[3]));
    roi.w = static_cast<int16_t>(std::atoi(argv[4]));
    roi.h = static_cast<int16_t>(std::atoi(argv[5]));

    CalibrationParams params;
    if (argc > 6) params.coverage = static_cast<float>(std::atof(argv[6]));

    if (roi.x < 0 || roi.y < 0 || roi.right() > img.width || roi.bottom() > img.height ||
        roi.empty()) {
        std::fprintf(stderr, "retângulo fora da imagem %dx%d\n", img.width, img.height);
        return 1;
    }

    const ImageView view = img.view();
    const CalibrationResult r = estimate_hsv_range(view, roi, g_workspace, params);
    const float fp = false_positive_rate(view, roi, r.range);

    if (r.low_chroma) {
        std::printf(
            "aviso: só %.0f%% do retângulo tem cor definida (S >= %u). Usando todos os pixels — "
            "reveja o recorte.\n",
            100.0 * r.chromatic / r.samples, params.chroma_floor);
    }
    std::printf("pixels no retângulo: %u   com cor definida: %u   arco de matiz: %u graus/2\n",
                r.samples, r.chromatic, r.hue_span);
    std::printf("falsos positivos fora do alvo: %.2f%%%s\n", 100.0 * fp,
                fp > 0.01f ? "  <-- alto: a cor do alvo não separa da cena" : "");
    if (r.range.wraps()) std::printf("faixa dá a volta em 180 (cor perto do vermelho)\n");

    std::printf(
        "\nHsvRange range;\n"
        "range.h_min = %u; range.h_max = %u;\n"
        "range.s_min = %u; range.s_max = %u;\n"
        "range.v_min = %u; range.v_max = %u;\n"
        "\nou direto no probe:  --range %u,%u,%u,%u,%u,%u\n",
        r.range.h_min, r.range.h_max, r.range.s_min, r.range.s_max, r.range.v_min, r.range.v_max,
        r.range.h_min, r.range.h_max, r.range.s_min, r.range.s_max, r.range.v_min, r.range.v_max);
    return 0;
}
