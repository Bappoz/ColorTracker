// Extração de regiões conexas por run-length + union-find.
//
// Substitui `findContours` + `moments` da BPMN. Motivo: o contorno em si nunca é
// usado no robô — só área, bounding box e centroide. Rotular componentes por
// runs entrega os três exatamente, em UMA passada sobre a máscara, sem alocar
// vetor de pontos por contorno (o que o findContours faz e é o que estoura a
// heap de um ESP32).
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"

namespace ecv {

struct Blob {
    uint32_t area = 0;  ///< pixels
    Rect box;           ///< bounding box
    Point centroid;     ///< centroide geométrico (momentos de ordem 1 / área)
};

/// Segmento horizontal contíguo de pixels acesos.
struct Run {
    int16_t y = 0;
    int16_t x0 = 0;  ///< inclusivo
    int16_t x1 = 0;  ///< inclusivo
    int16_t label = 0;
};

/// Estatísticas acumuladas por rótulo provisório.
struct LabelStats {
    uint32_t area;
    int32_t sum_x, sum_y;
    int16_t x0, y0, x1, y1;
};

/// Memória de trabalho fornecida pelo chamador — nada aqui aloca.
struct BlobWorkspace {
    Run* runs_prev = nullptr;
    Run* runs_cur = nullptr;
    int32_t max_runs_per_row = 0;
    int16_t* parent = nullptr;    ///< union-find, `max_labels` entradas
    LabelStats* stats = nullptr;  ///< `max_labels` entradas
    int32_t max_labels = 0;

    bool valid() const {
        return runs_prev && runs_cur && parent && stats && max_runs_per_row > 0 && max_labels > 1;
    }
};

/// Aloca estaticamente a workspace. `MaxRunsPerRow` ~ width/4 já cobre máscara
/// bem ruidosa; `MaxLabels` limita quantas componentes provisórias cabem.
template <int MaxRunsPerRow, int MaxLabels>
struct StaticBlobWorkspace {
    Run runs_a[MaxRunsPerRow];
    Run runs_b[MaxRunsPerRow];
    int16_t parent[MaxLabels];
    LabelStats stats[MaxLabels];

    BlobWorkspace view() {
        BlobWorkspace ws;
        ws.runs_prev = runs_a;
        ws.runs_cur = runs_b;
        ws.max_runs_per_row = MaxRunsPerRow;
        ws.parent = parent;
        ws.stats = stats;
        ws.max_labels = MaxLabels;
        return ws;
    }
};

/// Rotula a máscara e escreve em `out` os blobs com área >= `min_area`.
/// Conectividade 8. Retorna quantos blobs foram escritos (<= `max_out`).
/// As coordenadas são locais à `mask` — converter para o frame é responsabilidade
/// do chamador quando a máscara é uma ROI (ver `RoiTracker::to_global`).
int32_t find_blobs(const MaskView& mask, BlobWorkspace& ws, uint32_t min_area, Blob* out,
                   int32_t max_out);

/// Índice do blob de maior área, ou -1 se `count == 0`.
int32_t largest_blob(const Blob* blobs, int32_t count);

}  // namespace ecv
