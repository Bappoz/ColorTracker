// Testes das primitivas de imagem: cor, limiar, morfologia, blobs, linescan.
#include <cmath>
#include <vector>

#include "ecv/vision/blobs.hpp"
#include "ecv/vision/calibrate.hpp"
#include "ecv/vision/color.hpp"
#include "ecv/vision/linescan.hpp"
#include "ecv/vision/morphology.hpp"
#include "ecv/vision/roi.hpp"
#include "ecv/vision/threshold.hpp"
#include "sim/scene_source.hpp"
#include "test_harness.hpp"

using namespace ecv;

namespace {

/// Referência em ponto flutuante para validar a conversão inteira.
Hsv rgb_to_hsv_reference(Rgb c) {
    const float r = c.r / 255.0f, g = c.g / 255.0f, b = c.b / 255.0f;
    const float mx = std::fmax(r, std::fmax(g, b));
    const float mn = std::fmin(r, std::fmin(g, b));
    const float d = mx - mn;

    float h = 0.0f;
    if (d > 0.0f) {
        if (mx == r)
            h = 60.0f * std::fmod((g - b) / d, 6.0f);
        else if (mx == g)
            h = 60.0f * (((b - r) / d) + 2.0f);
        else
            h = 60.0f * (((r - g) / d) + 4.0f);
        if (h < 0.0f) h += 360.0f;
    }
    const float s = mx <= 0.0f ? 0.0f : d / mx;
    return Hsv{static_cast<uint8_t>(h / 2.0f + 0.5f), static_cast<uint8_t>(s * 255.0f + 0.5f),
               static_cast<uint8_t>(mx * 255.0f + 0.5f)};
}

MaskView make_mask(std::vector<uint8_t>& buf, int w, int h) {
    buf.assign(static_cast<size_t>(w) * h, 0);
    return MaskView{buf.data(), w, h, w};
}

}  // namespace

ECV_TEST(rgb_to_hsv_bate_com_a_referencia_em_float) {
    int max_h_err = 0, max_s_err = 0;
    for (int r = 0; r <= 255; r += 17) {
        for (int g = 0; g <= 255; g += 17) {
            for (int b = 0; b <= 255; b += 17) {
                const Rgb c{static_cast<uint8_t>(r), static_cast<uint8_t>(g),
                            static_cast<uint8_t>(b)};
                const Hsv got = rgb_to_hsv(c);
                const Hsv want = rgb_to_hsv_reference(c);
                CHECK_EQ(static_cast<int>(got.v), static_cast<int>(want.v));

                int dh = std::abs(static_cast<int>(got.h) - static_cast<int>(want.h));
                if (dh > 90) dh = 180 - dh;  // distância circular no matiz
                if (dh > max_h_err) max_h_err = dh;
                const int ds = std::abs(static_cast<int>(got.s) - static_cast<int>(want.s));
                if (ds > max_s_err) max_s_err = ds;
            }
        }
    }
    CHECK(max_h_err <= 1);
    CHECK(max_s_err <= 1);
}

ECV_TEST(decode_pixel_sobrevive_ao_roundtrip_de_cada_formato) {
    const Rgb original{200, 40, 40};
    const PixelFormat formats[] = {PixelFormat::kRgb888, PixelFormat::kBgr888, PixelFormat::kRgb565,
                                   PixelFormat::kYuyv};
    for (PixelFormat fmt : formats) {
        uint8_t rgb[6] = {original.r, original.g, original.b, original.r, original.g, original.b};
        uint8_t encoded[8] = {};
        sim::encode_frame(rgb, 2, 1, fmt, encoded);

        const Rgb got = decode_pixel(encoded, 0, fmt);
        // RGB565 descarta bits e YUYV subamostra croma: tolerância de 8 níveis.
        CHECK(std::abs(static_cast<int>(got.r) - original.r) <= 8);
        CHECK(std::abs(static_cast<int>(got.g) - original.g) <= 8);
        CHECK(std::abs(static_cast<int>(got.b) - original.b) <= 8);
    }
}

ECV_TEST(faixa_de_matiz_com_wrap_pega_vermelho_dos_dois_lados) {
    HsvRange red;
    red.h_min = 170;
    red.h_max = 10;  // dá a volta em 180
    red.s_min = 100;
    red.v_min = 60;

    CHECK(red.wraps());
    CHECK(in_range(Hsv{175, 200, 200}, red));
    CHECK(in_range(Hsv{5, 200, 200}, red));
    CHECK(!in_range(Hsv{90, 200, 200}, red));
    CHECK(!in_range(Hsv{175, 50, 200}, red));  // saturação baixa demais
}

ECV_TEST(limiarizacao_marca_apenas_o_disco_colorido) {
    sim::SceneConfig cfg;
    cfg.width = 160;
    cfg.height = 120;
    cfg.opponent_radius = 20;
    cfg.noise = 0;
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    src.set_opponent(Point{80, 60});

    ImageView frame{};
    CHECK(src.next(frame));

    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 160, 120);
    HsvRange range;
    range.h_min = 170;
    range.h_max = 10;
    range.s_min = 120;
    range.v_min = 60;

    const uint32_t hits = threshold_hsv_count(frame, range, mask);
    const double expected = 3.14159 * 20 * 20;
    CHECK_NEAR(hits, expected, expected * 0.1);
    CHECK_EQ(static_cast<int>(mask.row(60)[80]), 255);
    CHECK_EQ(static_cast<int>(mask.row(5)[5]), 0);
}

ECV_TEST(lut565_reproduz_exatamente_o_limiar_calculado_por_pixel) {
    HsvRange range;
    range.h_min = 170;
    range.h_max = 10;
    range.s_min = 120;
    range.v_min = 60;

    static ColorLut565 lut;
    lut.build(range);
    CHECK(lut.built());

    sim::SceneConfig cfg;
    cfg.width = 160;
    cfg.height = 120;
    cfg.format = PixelFormat::kRgb565;
    cfg.noise = 8;
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    ImageView frame{};
    CHECK(src.next(frame));

    std::vector<uint8_t> a, b;
    MaskView direct = make_mask(a, 160, 120);
    MaskView table = make_mask(b, 160, 120);
    threshold_hsv(frame, range, direct);
    threshold_lut(frame, lut, table);

    // Em RGB565 a tabela é indexada pelo próprio pixel: tem que bater bit a bit.
    int diffs = 0;
    for (int y = 0; y < 120; ++y)
        for (int x = 0; x < 160; ++x)
            if (direct.row(y)[x] != table.row(y)[x]) ++diffs;
    CHECK_EQ(diffs, 0);
}

ECV_TEST(lut565_quantiza_formato_de_24_bits_com_erro_desprezivel) {
    HsvRange range;
    range.h_min = 170;
    range.h_max = 10;
    range.s_min = 120;
    range.v_min = 60;

    static ColorLut565 lut;
    lut.build(range);

    sim::SceneConfig cfg;
    cfg.width = 160;
    cfg.height = 120;
    cfg.format = PixelFormat::kRgb888;
    cfg.noise = 8;
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    ImageView frame{};
    CHECK(src.next(frame));

    std::vector<uint8_t> a, b;
    MaskView direct = make_mask(a, 160, 120);
    MaskView table = make_mask(b, 160, 120);
    threshold_hsv(frame, range, direct);
    threshold_lut(frame, lut, table);

    int diffs = 0;
    for (int y = 0; y < 120; ++y)
        for (int x = 0; x < 160; ++x)
            if (direct.row(y)[x] != table.row(y)[x]) ++diffs;
    CHECK(diffs * 100 < 160 * 120);  // menos de 1% dos pixels
}

ECV_TEST(calibracao_recupera_a_faixa_do_alvo_com_recorte_folgado) {
    sim::SceneConfig cfg;
    cfg.width = 320;
    cfg.height = 240;
    cfg.opponent_radius = 30;
    cfg.noise = 6;
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    src.set_opponent(Point{160, 120});

    ImageView frame{};
    CHECK(src.next(frame));

    // Recorte deliberadamente folgado: o quadrado circunscrito ao disco tem 21%
    // de fundo nos cantos, que é o erro que se comete medindo à mão.
    static CalibrationWorkspace ws;
    const Rect roi{130, 90, 60, 60};
    const CalibrationResult r = estimate_hsv_range(frame, roi, ws);

    CHECK(!r.low_chroma);
    CHECK_EQ(r.samples, 3600u);
    CHECK(r.range.wraps());  // alvo vermelho: faixa dá a volta em 180
    CHECK(r.range.s_min > 100);
    CHECK(r.range.v_min > 100);
    CHECK(false_positive_rate(frame, roi, r.range) < 0.01f);

    // E a faixa estimada precisa realmente segmentar o alvo.
    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 320, 240);
    const uint32_t hits = threshold_hsv_count(frame, r.range, mask);
    const double area = 3.14159 * 30 * 30;
    CHECK_NEAR(hits, area, area * 0.2);
}

ECV_TEST(calibracao_avisa_quando_o_recorte_pegou_so_fundo) {
    sim::SceneConfig cfg;
    cfg.width = 320;
    cfg.height = 240;
    cfg.noise = 6;
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    src.set_opponent(Point{160, 120});

    ImageView frame{};
    CHECK(src.next(frame));

    static CalibrationWorkspace ws;
    const CalibrationResult r = estimate_hsv_range(frame, Rect{10, 10, 40, 40}, ws);
    CHECK(r.low_chroma);
    CHECK(r.chromatic * 4 < r.samples);
}

ECV_TEST(abertura_remove_pixel_isolado_e_preserva_bloco) {
    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 32, 32);
    mask.row(3)[3] = 255;  // ruído sal
    for (int y = 10; y < 20; ++y)
        for (int x = 10; x < 20; ++x) mask.row(y)[x] = 255;  // alvo

    std::vector<uint8_t> scratch(32);
    open3(mask, scratch.data());

    CHECK_EQ(static_cast<int>(mask.row(3)[3]), 0);
    CHECK_EQ(static_cast<int>(mask.row(15)[15]), 255);
    CHECK_EQ(static_cast<int>(mask.row(10)[10]), 255);  // canto do bloco preservado
}

ECV_TEST(fechamento_preenche_buraco_interno) {
    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 32, 32);
    for (int y = 10; y < 20; ++y)
        for (int x = 10; x < 20; ++x) mask.row(y)[x] = 255;
    mask.row(15)[15] = 0;  // reflexo especular no meio do alvo

    std::vector<uint8_t> scratch(32);
    close3(mask, scratch.data());
    CHECK_EQ(static_cast<int>(mask.row(15)[15]), 255);
}

ECV_TEST(blobs_separam_componentes_e_medem_area_e_centroide) {
    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 64, 64);
    for (int y = 5; y < 15; ++y)
        for (int x = 5; x < 15; ++x) mask.row(y)[x] = 255;  // 10x10 em (9.5, 9.5)
    for (int y = 30; y < 50; ++y)
        for (int x = 30; x < 50; ++x) mask.row(y)[x] = 255;  // 20x20 em (39.5, 39.5)

    StaticBlobWorkspace<32, 128> ws_storage;
    BlobWorkspace ws = ws_storage.view();
    Blob blobs[8];
    const int32_t n = find_blobs(mask, ws, 10, blobs, 8);
    CHECK_EQ(n, 2);

    const int32_t big = largest_blob(blobs, n);
    CHECK(big >= 0);
    CHECK_EQ(blobs[big].area, 400u);
    CHECK_EQ(static_cast<int>(blobs[big].centroid.x), 39);
    CHECK_EQ(static_cast<int>(blobs[big].centroid.y), 39);
    CHECK_EQ(static_cast<int>(blobs[big].box.x), 30);
    CHECK_EQ(static_cast<int>(blobs[big].box.w), 20);

    const int32_t small = big == 0 ? 1 : 0;
    CHECK_EQ(blobs[small].area, 100u);
    CHECK_EQ(static_cast<int>(blobs[small].centroid.x), 9);
}

ECV_TEST(blobs_unem_forma_em_u_num_unico_componente) {
    // Forma que só é uma componente se o union-find funcionar: dois braços
    // verticais ligados por baixo.
    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 32, 32);
    for (int y = 4; y < 20; ++y) {
        mask.row(y)[5] = 255;
        mask.row(y)[6] = 255;
        mask.row(y)[15] = 255;
        mask.row(y)[16] = 255;
    }
    for (int x = 5; x <= 16; ++x) {
        mask.row(20)[x] = 255;
        mask.row(21)[x] = 255;
    }

    StaticBlobWorkspace<16, 64> ws_storage;
    BlobWorkspace ws = ws_storage.view();
    Blob blobs[4];
    const int32_t n = find_blobs(mask, ws, 1, blobs, 4);
    CHECK_EQ(n, 1);
    CHECK_EQ(blobs[0].area, 16u * 4 + 12u * 2);
    CHECK_EQ(static_cast<int>(blobs[0].box.x), 5);
    CHECK_EQ(static_cast<int>(blobs[0].box.w), 12);
}

ECV_TEST(blobs_conectam_na_diagonal) {
    std::vector<uint8_t> buf;
    MaskView mask = make_mask(buf, 16, 16);
    mask.row(4)[4] = 255;
    mask.row(5)[5] = 255;
    mask.row(6)[6] = 255;

    StaticBlobWorkspace<8, 32> ws_storage;
    BlobWorkspace ws = ws_storage.view();
    Blob blobs[4];
    CHECK_EQ(find_blobs(mask, ws, 1, blobs, 4), 1);
    CHECK_EQ(blobs[0].area, 3u);
}

ECV_TEST(roi_recorta_sem_copiar_e_converte_coordenadas) {
    sim::SceneConfig cfg;
    cfg.width = 320;
    cfg.height = 240;
    cfg.noise = 0;
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    ImageView frame{};
    CHECK(src.next(frame));

    RoiTracker roi(320, 240, 40);
    CHECK(!roi.active());
    CHECK_EQ(static_cast<int>(roi.current().w), 320);

    roi.update(Rect{100, 80, 40, 40});
    CHECK(roi.active());
    CHECK_EQ(static_cast<int>(roi.roi().x), 60);
    CHECK_EQ(static_cast<int>(roi.roi().w), 120);

    const ImageView sub = crop(frame, roi.roi());
    CHECK_EQ(sub.width, 120);
    CHECK_EQ(sub.stride, frame.stride);  // mesma imagem, só ponteiro deslocado
    CHECK(sub.data == frame.row(roi.roi().y) + roi.roi().x * 3);

    const Point global = roi.to_global(Point{10, 10});
    CHECK_EQ(static_cast<int>(global.x), 70);

    // Alvo colado na borda: a ROI é cortada, não sai do frame.
    roi.update(Rect{300, 220, 40, 40});
    CHECK_EQ(static_cast<int>(roi.roi().right()), 320);
    CHECK_EQ(static_cast<int>(roi.roi().bottom()), 240);
}

ECV_TEST(linescan_localiza_a_faixa_branca_e_o_lado) {
    sim::SceneConfig cfg;
    cfg.width = 320;
    cfg.height = 240;
    cfg.noise = 0;
    cfg.opponent_visible = false;
    cfg.white_band = Rect{0, 200, 100, 40};  // borda do ringue à esquerda
    sim::SyntheticSource src(cfg);
    CHECK(src.open());
    ImageView frame{};
    CHECK(src.next(frame));

    LineScanConfig scan;
    scan.target.s_max = 60;
    scan.target.v_min = 200;
    scan.y_top = 210;
    scan.y_bottom = 235;
    scan.rows = 4;
    scan.min_hits = 20;

    const LineScanResult r = scan_lines(frame, scan);
    CHECK(r.found);
    CHECK(r.left());
    CHECK(!r.right());
    CHECK(line_error(r, 320) < 0.0f);  // linha à esquerda do centro
}
