// Sonda a detecção com a câmera de verdade, desenhando no próprio terminal.
//
// Sem GUI: cada célula do terminal vira dois pixels verticais com o caractere
// meio-bloco (▀) e cor de frente/fundo em truecolor. Isso mantém a promessa do
// repositório — nenhuma dependência — e ainda funciona por SSH, que é como se
// olha para a câmera de um Raspberry Pi montado no robô.
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "ecv/app/sumo_vision.hpp"
#include "ecv/vision/calibrate.hpp"
#include "linux/v4l2_source.hpp"
#include "sim/ppm_io.hpp"

using namespace ecv;

namespace {

volatile std::sig_atomic_t g_running = 1;
void on_signal(int) {
    g_running = 0;
}

void restore_terminal_at_exit();

constexpr const char* kReset = "\x1b[0m";
constexpr const char* kHideCursor = "\x1b[?25l";
constexpr const char* kShowCursor = "\x1b[?25h";
constexpr const char* kClear = "\x1b[2J\x1b[H";
constexpr const char* kHome = "\x1b[H";

struct TermSize {
    int cols = 80;
    int rows = 24;
};

TermSize term_size() {
    TermSize t;
    winsize ws{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0) {
        t.cols = ws.ws_col;
        t.rows = ws.ws_row;
    }
    return t;
}

/// Buffer de preview em espaço próprio: a imagem é subamostrada uma vez e todo
/// o desenho (caixa, centroide, ROI) acontece aqui. Desenhar no espaço do frame
/// e só depois subamostrar apagaria qualquer linha de 1 px.
class Preview {
public:
    void resize(int w, int h) {
        w_ = w;
        h_ = h;
        px_.assign(static_cast<size_t>(w) * h, Rgb{});
    }
    int width() const { return w_; }
    int height() const { return h_; }

    /// Média de caixa em vez de vizinho mais próximo. Cada célula do preview
    /// cobre ~3x3 pixels do frame; pegar só um deles descarta o resto e serrilha
    /// a imagem, o que faz parecer que a detecção enxerga menos do que enxerga.
    /// Limitado a 3x3 amostras por célula: o resto da melhora não paga o custo.
    void fill(const ImageView& frame) {
        for (int y = 0; y < h_; ++y) {
            const int sy0 = static_cast<int>(static_cast<int64_t>(y) * frame.height / h_);
            const int sy1 = static_cast<int>(static_cast<int64_t>(y + 1) * frame.height / h_);
            for (int x = 0; x < w_; ++x) {
                const int sx0 = static_cast<int>(static_cast<int64_t>(x) * frame.width / w_);
                const int sx1 = static_cast<int>(static_cast<int64_t>(x + 1) * frame.width / w_);
                px_[static_cast<size_t>(y) * w_ + x] = average(frame, sx0, sx1, sy0, sy1);
            }
        }
    }

    /// A máscara só é válida dentro da ROI processada; fora dela pinta escuro
    /// para deixar visível que aquela área não foi olhada neste frame.
    void fill_mask(const MaskView& mask, const Rect& roi, int fw, int fh) {
        for (int y = 0; y < h_; ++y) {
            const int fy = static_cast<int>(static_cast<int64_t>(y) * fh / h_);
            for (int x = 0; x < w_; ++x) {
                const int fx = static_cast<int>(static_cast<int64_t>(x) * fw / w_);
                Rgb c{20, 20, 28};
                const int mx = fx - roi.x;
                const int my = fy - roi.y;
                if (mx >= 0 && my >= 0 && mx < mask.width && my < mask.height) {
                    const uint8_t v = mask.row(my)[mx];
                    c = v ? Rgb{235, 235, 235} : Rgb{45, 45, 55};
                }
                px_[static_cast<size_t>(y) * w_ + x] = c;
            }
        }
    }

    void set(int x, int y, Rgb c) {
        if (x < 0 || y < 0 || x >= w_ || y >= h_) return;
        px_[static_cast<size_t>(y) * w_ + x] = c;
    }

    void rect(const Rect& r, int fw, int fh, Rgb c) {
        const int x0 = map(r.x, fw, w_), x1 = map(r.right() - 1, fw, w_);
        const int y0 = map(r.y, fh, h_), y1 = map(r.bottom() - 1, fh, h_);
        for (int x = x0; x <= x1; ++x) {
            set(x, y0, c);
            set(x, y1, c);
        }
        for (int y = y0; y <= y1; ++y) {
            set(x0, y, c);
            set(x1, y, c);
        }
    }

    /// Mistura `c` sobre a região — usado para marcar a faixa varrida pela
    /// detecção de borda sem esconder a imagem embaixo.
    void blend_rect(const Rect& r, int fw, int fh, Rgb c) {
        const int x0 = map(r.x, fw, w_), x1 = map(r.right() - 1, fw, w_);
        const int y0 = map(r.y, fh, h_), y1 = map(r.bottom() - 1, fh, h_);
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                Rgb& d = px_[static_cast<size_t>(y) * w_ + x];
                d = Rgb{static_cast<uint8_t>((d.r + c.r) / 2),
                        static_cast<uint8_t>((d.g + c.g) / 2),
                        static_cast<uint8_t>((d.b + c.b) / 2)};
            }
        }
    }

    void cross(Point p, int fw, int fh, Rgb c) {
        const int x = map(p.x, fw, w_), y = map(p.y, fh, h_);
        for (int d = -2; d <= 2; ++d) {
            set(x + d, y, c);
            set(x, y + d, c);
        }
    }

    void vline(int fx, int fw, Rgb c) {
        const int x = map(static_cast<int16_t>(fx), fw, w_);
        for (int y = 0; y < h_; ++y) {
            Rgb& dst = px_[static_cast<size_t>(y) * w_ + x];
            dst = Rgb{static_cast<uint8_t>((dst.r + c.r) / 2),
                      static_cast<uint8_t>((dst.g + c.g) / 2),
                      static_cast<uint8_t>((dst.b + c.b) / 2)};
        }
    }

    /// Emite tudo de uma vez, repetindo escape só quando a cor muda: sem isso
    /// são ~40 bytes por célula e o terminal vira o gargalo do laço.
    void render(std::string& out) const {
        out.clear();
        out += kHome;
        char buf[64];
        int last_fg = -1, last_bg = -1;
        for (int y = 0; y + 1 < h_; y += 2) {
            for (int x = 0; x < w_; ++x) {
                const Rgb top = px_[static_cast<size_t>(y) * w_ + x];
                const Rgb bottom = px_[static_cast<size_t>(y + 1) * w_ + x];
                const int fg = (top.r << 16) | (top.g << 8) | top.b;
                const int bg = (bottom.r << 16) | (bottom.g << 8) | bottom.b;
                if (fg != last_fg) {
                    std::snprintf(buf, sizeof(buf), "\x1b[38;2;%u;%u;%um", top.r, top.g, top.b);
                    out += buf;
                    last_fg = fg;
                }
                if (bg != last_bg) {
                    std::snprintf(buf, sizeof(buf), "\x1b[48;2;%u;%u;%um", bottom.r, bottom.g,
                                  bottom.b);
                    out += buf;
                    last_bg = bg;
                }
                out += "▀";  // meio-bloco superior
            }
            out += kReset;
            out += "\x1b[K\n";
            last_fg = last_bg = -1;
        }
    }

    /// Maior preview que cabe em `cols` x `char_rows` sem distorcer a imagem.
    /// Cada célula do terminal vale 2 pixels na vertical e 1 na horizontal, e a
    /// célula em si tem ~2:1 (altura:largura) na maioria das fontes mono — então
    /// o pixel do preview sai quadrado e basta casar as proporções.
    static void fit(int cols, int char_rows, int fw, int fh, int& out_w, int& out_h) {
        int w = cols;
        int h = char_rows * 2;
        if (fw <= 0 || fh <= 0) {
            out_w = w;
            out_h = h;
            return;
        }
        if (w * fh > h * fw) {
            w = h * fw / fh;  // sobra largura: a altura é quem limita
        } else {
            h = w * fh / fw;
        }
        out_w = w > 0 ? w : 1;
        out_h = (h > 1 ? h : 2) & ~1;  // par: cada linha do terminal são 2 pixels
    }

private:
    static Rgb average(const ImageView& frame, int x0, int x1, int y0, int y1) {
        if (x1 <= x0) x1 = x0 + 1;
        if (y1 <= y0) y1 = y0 + 1;
        const int step_x = (x1 - x0) > 3 ? (x1 - x0) / 3 : 1;
        const int step_y = (y1 - y0) > 3 ? (y1 - y0) / 3 : 1;

        uint32_t r = 0, g = 0, b = 0, n = 0;
        for (int y = y0; y < y1 && y < frame.height; y += step_y) {
            const uint8_t* row = frame.row(y);
            for (int x = x0; x < x1 && x < frame.width; x += step_x) {
                const Rgb c = decode_pixel(row, x, frame.format);
                r += c.r;
                g += c.g;
                b += c.b;
                ++n;
            }
        }
        if (n == 0) return Rgb{};
        return Rgb{static_cast<uint8_t>(r / n), static_cast<uint8_t>(g / n),
                   static_cast<uint8_t>(b / n)};
    }

    static int map(int16_t v, int from, int to) {
        if (from <= 0) return 0;
        int r = static_cast<int>(static_cast<int64_t>(v) * to / from);
        if (r < 0) r = 0;
        if (r >= to) r = to - 1;
        return r;
    }

    int w_ = 0, h_ = 0;
    std::vector<Rgb> px_;
};

/// Terminal em modo cru para ler tecla sem Enter e sem bloquear o laço.
/// Restaura no destrutor e no sinal — deixar o terminal em raw depois de um
/// Ctrl-C é o tipo de coisa que faz a pessoa fechar o emulador na marra.
class RawTerminal {
public:
    void enable() {
        if (!isatty(STDIN_FILENO)) return;
        if (tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        termios raw = saved_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
        active_ = true;
    }

    void restore() {
        if (!active_) return;
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        active_ = false;
    }

    ~RawTerminal() { restore(); }

    /// -1 quando não há tecla pendente.
    int poll() const {
        if (!active_) return -1;
        char c = 0;
        return ::read(STDIN_FILENO, &c, 1) == 1 ? static_cast<unsigned char>(c) : -1;
    }

private:
    termios saved_{};
    bool active_ = false;
};

RawTerminal g_terminal;

/// Faixa branca do dohyo vista de frente pelo robô: as linhas de baixo do quadro.
LineScanConfig default_border(int32_t width, int32_t height) {
    (void)width;
    LineScanConfig c;
    c.target.s_max = 60;   // branco é pouco saturado
    c.target.v_min = 190;  // e claro
    c.y_top = static_cast<int16_t>(height * 82 / 100);
    c.y_bottom = static_cast<int16_t>(height - 1);
    c.rows = 4;
    c.min_hits = 12;
    return c;
}

PixelFormat parse_format(const char* s) {
    if (std::strcmp(s, "yuyv") == 0) return PixelFormat::kYuyv;
    if (std::strcmp(s, "rgb565") == 0) return PixelFormat::kRgb565;
    if (std::strcmp(s, "bgr888") == 0) return PixelFormat::kBgr888;
    return PixelFormat::kRgb888;
}

bool parse_range(const char* s, HsvRange& r) {
    unsigned v[6];
    if (std::sscanf(s, "%u,%u,%u,%u,%u,%u", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) {
        return false;
    }
    r.h_min = static_cast<uint8_t>(v[0]);
    r.h_max = static_cast<uint8_t>(v[1]);
    r.s_min = static_cast<uint8_t>(v[2]);
    r.s_max = static_cast<uint8_t>(v[3]);
    r.v_min = static_cast<uint8_t>(v[4]);
    r.v_max = static_cast<uint8_t>(v[5]);
    return true;
}

void usage(const char* prog) {
    std::printf(
        "uso: %s [opções]\n"
        "  --list                 formatos que a câmera aceita, e sai\n"
        "  --device CAMINHO       padrão /dev/video0\n"
        "  --width N --height N   padrão 320x240\n"
        "  --format F             yuyv (padrão) | rgb565 | bgr888 | rgb888\n"
        "  --range h0,h1,s0,s1,v0,v1   faixa HSV do alvo (h0>h1 = dá a volta no vermelho)\n"
        "  --min-area N           área mínima do blob, padrão 150\n"
        "  --calibrate            estima a faixa no retângulo central e sai\n"
        "  --rect x,y,w,h         retângulo usado por --calibrate\n"
        "  --snapshot ARQ.ppm     grava um frame e sai\n"
        "  --mask                 preview mostra a máscara em vez da imagem\n"
        "  --border               liga a detecção da linha branca do ringue\n"
        "  --lock-exposure        trava exposição/ganho/AWB (o que o robô usa)\n"
        "  --log ARQ.csv          registra uma linha por frame, para analisar depois\n"
        "  --no-preview           só estatísticas, para terminal sem truecolor\n"
        "  --seconds N            encerra sozinho depois de N segundos\n"
        "\n"
        "durante a execução (não precisa reiniciar):\n"
        "  c  calibra a faixa no retângulo central e aplica na hora\n"
        "  m  alterna imagem / máscara      b  liga/desliga detecção de borda\n"
        "  e  trava/destrava exposição      r  reseta o rastreio\n"
        "  +/- ajusta a área mínima         s  salva um PPM do frame atual\n"
        "  q  encerra\n",
        prog);
}

void restore_terminal_at_exit() {
    g_terminal.restore();
}

CalibrationWorkspace g_calibration;
SumoStorage<640, 480> g_storage;

}  // namespace

int main(int argc, char** argv) {
    linux_hal::V4l2Config cam;
    cam.lock_exposure = false;  // explorando: deixa o automático ajudar
    cam.format = PixelFormat::kYuyv;

    HsvRange range;
    range.h_min = 170;  // marcador vermelho por padrão
    range.h_max = 10;
    range.s_min = 120;
    range.v_min = 60;

    bool list_only = false, calibrate = false, show_mask = false, preview_on = true;
    bool border_on = false;
    uint32_t min_area = 150;
    float duration_s = 0.0f;
    std::string snapshot, log_path;
    Rect calib_rect{};

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--help" || a == "-h") {
            usage(argv[0]);
            return 0;
        } else if (a == "--list") {
            list_only = true;
        } else if (a == "--device" && i + 1 < argc) {
            cam.device = argv[++i];
        } else if (a == "--width" && i + 1 < argc) {
            cam.width = std::atoi(argv[++i]);
        } else if (a == "--height" && i + 1 < argc) {
            cam.height = std::atoi(argv[++i]);
        } else if (a == "--format" && i + 1 < argc) {
            cam.format = parse_format(argv[++i]);
        } else if (a == "--range" && i + 1 < argc) {
            if (!parse_range(argv[++i], range)) {
                std::fprintf(stderr, "faixa inválida: use --range h0,h1,s0,s1,v0,v1\n");
                return 1;
            }
        } else if (a == "--min-area" && i + 1 < argc) {
            min_area = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (a == "--calibrate") {
            calibrate = true;
        } else if (a == "--rect" && i + 1 < argc) {
            int x, y, w, h;
            if (std::sscanf(argv[++i], "%d,%d,%d,%d", &x, &y, &w, &h) == 4) {
                calib_rect = Rect{static_cast<int16_t>(x), static_cast<int16_t>(y),
                                  static_cast<int16_t>(w), static_cast<int16_t>(h)};
            }
        } else if (a == "--snapshot" && i + 1 < argc) {
            snapshot = argv[++i];
        } else if (a == "--mask") {
            show_mask = true;
        } else if (a == "--border") {
            border_on = true;
        } else if (a == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (a == "--lock-exposure") {
            cam.lock_exposure = true;
        } else if (a == "--no-preview") {
            preview_on = false;
        } else if (a == "--seconds" && i + 1 < argc) {
            duration_s = static_cast<float>(std::atof(argv[++i]));
        } else {
            std::fprintf(stderr, "opção desconhecida: %s\n", a.c_str());
            usage(argv[0]);
            return 1;
        }
    }

    if (list_only) {
        linux_hal::V4l2Mode modes[128];
        const int n = linux_hal::list_modes(cam.device, modes, 128);
        if (n < 0) {
            std::fprintf(stderr, "não consegui abrir %s\n", cam.device.c_str());
            return 1;
        }
        std::printf("%s — %d modos (marcados com * o núcleo decodifica)\n", cam.device.c_str(), n);
        for (int i = 0; i < n; ++i) {
            std::printf("  %s %4dx%-4d %3u fps %s\n", modes[i].fourcc, modes[i].width,
                        modes[i].height, modes[i].fps, modes[i].supported_by_ecv ? "*" : "");
        }
        std::printf(
            "\nMJPG não aparece marcado: é fluxo comprimido e decodificar JPEG por frame\n"
            "custaria mais que o pipeline inteiro. Use um modo YUYV.\n");
        return 0;
    }

    if (cam.width > 640 || cam.height > 480) {
        std::fprintf(stderr, "esta build reserva buffers para até 640x480\n");
        return 1;
    }

    linux_hal::V4l2Source camera(cam);
    if (!camera.open()) {
        std::fprintf(stderr, "câmera: %s\nrode com --list para ver os modos aceitos\n",
                     camera.last_error().c_str());
        return 1;
    }

    ImageView frame{};
    // Descarta os primeiros frames: o automático da webcam leva alguns quadros
    // para estabilizar e calibrar no primeiro frame dá faixa errada.
    for (int i = 0; i < 12 && camera.next(frame); ++i) {
    }

    if (calibrate || !snapshot.empty()) {
        if (!camera.next(frame)) {
            std::fprintf(stderr, "captura falhou: %s\n", camera.last_error().c_str());
            camera.close();
            return 1;
        }
        if (!snapshot.empty()) {
            if (sim::write_ppm(snapshot, frame)) {
                std::printf("frame %dx%d salvo em %s\n", frame.width, frame.height,
                            snapshot.c_str());
            } else {
                std::fprintf(stderr, "não consegui escrever %s\n", snapshot.c_str());
            }
        }
        if (calibrate) {
            Rect roi = calib_rect;
            if (roi.empty()) {  // quarto central: onde se põe o alvo para calibrar
                roi = Rect{
                    static_cast<int16_t>(frame.width / 4), static_cast<int16_t>(frame.height / 4),
                    static_cast<int16_t>(frame.width / 2), static_cast<int16_t>(frame.height / 2)};
            }
            const CalibrationResult r = estimate_hsv_range(frame, roi, g_calibration);
            const float fp = false_positive_rate(frame, roi, r.range);
            std::printf("retângulo %d,%d %dx%d · %u pixels, %u com cor definida\n", roi.x, roi.y,
                        roi.w, roi.h, r.samples, r.chromatic);
            if (r.low_chroma) {
                std::printf("aviso: quase nada com cor no retângulo — o alvo está mesmo ali?\n");
            }
            std::printf("falsos positivos fora do retângulo: %.2f%%\n", 100.0 * fp);
            std::printf("\n  --range %u,%u,%u,%u,%u,%u\n", r.range.h_min, r.range.h_max,
                        r.range.s_min, r.range.s_max, r.range.v_min, r.range.v_max);
        }
        camera.close();
        return 0;
    }

    SumoConfig cfg;
    cfg.frame_w = static_cast<int16_t>(camera.width());
    cfg.frame_h = static_cast<int16_t>(camera.height());
    cfg.target = range;
    cfg.min_area = min_area;
    cfg.border = default_border(camera.width(), camera.height());
    cfg.border_enabled = border_on;

    SumoVision::Buffers buf = g_storage.view();
    buf.mask.width = cfg.frame_w;
    buf.mask.height = cfg.frame_h;
    SumoVision vision(cfg, buf);

    const Rect center_rect{
        static_cast<int16_t>(cfg.frame_w / 4), static_cast<int16_t>(cfg.frame_h / 4),
        static_cast<int16_t>(cfg.frame_w / 2), static_cast<int16_t>(cfg.frame_h / 2)};

    std::FILE* log = nullptr;
    if (!log_path.empty()) {
        log = std::fopen(log_path.c_str(), "w");
        if (log) {
            std::fprintf(log,
                         "frame,t_s,estado,detectado,alvo_x,alvo_y,area,erro,roi_w,roi_h,"
                         "cmd_l,cmd_r,borda,borda_esq,borda_centro,borda_dir,visao_us,fps\n");
        } else {
            std::fprintf(stderr, "não consegui abrir %s para escrita\n", log_path.c_str());
        }
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::atexit(restore_terminal_at_exit);
    g_terminal.enable();

    Preview preview;
    std::string out;
    out.reserve(1 << 18);
    if (preview_on) std::fputs(kClear, stdout), std::fputs(kHideCursor, stdout);

    LatencyStats latency;
    uint64_t previous = micros();
    const uint64_t started = previous;
    uint32_t frames = 0, detections = 0, snapshots = 0;
    float fps = 0.0f;
    const char* notice = "";

    while (g_running) {
        if (!camera.next(frame)) {
            std::fprintf(stderr, "captura falhou: %s\n", camera.last_error().c_str());
            break;
        }
        const uint64_t now = micros();
        const float dt = static_cast<float>(now - previous) * 1e-6f;
        previous = now;
        fps = dt > 0.0f ? 0.9f * fps + 0.1f / dt : fps;

        switch (g_terminal.poll()) {
            case 'q': g_running = 0; break;
            case 'm': show_mask = !show_mask; break;
            case 'b':
                border_on = !border_on;
                vision.set_border_enabled(border_on);
                notice = border_on ? "borda ligada" : "borda desligada";
                break;
            case 'e': {
                const bool locked = !camera.exposure_locked();
                camera.set_exposure_lock(locked);
                notice = locked ? "exposição travada" : "exposição automática";
                break;
            }
            case 'r':
                vision.reset();
                notice = "rastreio resetado";
                break;
            case 'c': {
                // Calibra no que está no centro do quadro AGORA e já aplica.
                const CalibrationResult k = estimate_hsv_range(frame, center_rect, g_calibration);
                range = k.range;
                vision.set_target_range(range);
                if (k.low_chroma) {
                    notice = "calibrado, mas o centro tem pouca cor — aproxime o alvo";
                } else if (k.hue_span > 60) {
                    // Arco largo = o centro tem várias cores, não um alvo só.
                    notice = "calibrado, mas o centro tem cores demais — enquadre só o alvo";
                } else {
                    notice = "calibrado no centro do quadro";
                }
                break;
            }
            case '+':
            case '=':
                min_area = min_area < 40000u ? min_area * 2 : min_area;
                vision.set_min_area(min_area);
                break;
            case '-':
            case '_':
                min_area = min_area > 20u ? min_area / 2 : min_area;
                vision.set_min_area(min_area);
                break;
            case 's': {
                char path[64];
                std::snprintf(path, sizeof(path), "probe_%03u.ppm", snapshots++);
                notice = sim::write_ppm(path, frame) ? "frame salvo em probe_NNN.ppm"
                                                     : "falhou ao salvar o PPM";
                break;
            }
            default: break;
        }
        if (!g_running) break;

        const SumoResult r = vision.process(frame, dt);
        latency.add(r.timings.total_us);
        ++frames;
        detections += r.detected ? 1 : 0;

        if (log) {
            std::fprintf(log, "%u,%.3f,%s,%d,%d,%d,%u,%.4f,%d,%d,%.3f,%.3f,%d,%u,%u,%u,%u,%.1f\n",
                         frames, static_cast<double>(now - started) * 1e-6,
                         track_state_name(r.state), r.detected ? 1 : 0, r.target.x, r.target.y,
                         r.area, static_cast<double>(r.error_norm), r.roi.w, r.roi.h,
                         static_cast<double>(r.cmd.left), static_cast<double>(r.cmd.right),
                         r.border_hit() ? 1 : 0, r.border.zone_hits[0], r.border.zone_hits[1],
                         r.border.zone_hits[2], r.timings.total_us, static_cast<double>(fps));
        }

        if (preview_on) {
            const TermSize t = term_size();
            const int cols = t.cols < 160 ? t.cols : 160;
            const int rows = (t.rows > 9 ? t.rows - 6 : 4);
            int pw = 0, ph = 0;
            Preview::fit(cols, rows, frame.width, frame.height, pw, ph);
            if (preview.width() != pw || preview.height() != ph) preview.resize(pw, ph);

            if (show_mask) {
                preview.fill_mask(buf.mask, r.roi, frame.width, frame.height);
            } else {
                preview.fill(frame);
            }
            if (border_on) {
                // Faixa varrida em cinza; o terço que acendeu, em vermelho.
                const LineScanConfig& bc = vision.config().border;
                const int16_t band_h = static_cast<int16_t>(bc.y_bottom - bc.y_top + 1);
                preview.blend_rect(Rect{0, bc.y_top, static_cast<int16_t>(frame.width), band_h},
                                   frame.width, frame.height, Rgb{70, 70, 90});
                const int16_t third = static_cast<int16_t>(frame.width / 3);
                const bool zones[3] = {r.border.left(), r.border.center(), r.border.right()};
                for (int z = 0; z < 3; ++z) {
                    if (!zones[z]) continue;
                    preview.blend_rect(
                        Rect{static_cast<int16_t>(z * third), bc.y_top, third, band_h}, frame.width,
                        frame.height, Rgb{230, 40, 40});
                }
            }
            preview.vline(frame.width / 2, frame.width, Rgb{90, 90, 90});
            if (r.roi.w < frame.width) {
                preview.rect(r.roi, frame.width, frame.height, Rgb{40, 130, 160});
            }
            if (r.detected) {
                preview.rect(r.box, frame.width, frame.height, Rgb{250, 200, 40});
            }
            if (r.state != TrackState::kSearching) {
                preview.cross(r.target, frame.width, frame.height,
                              r.detected ? Rgb{255, 60, 60} : Rgb{160, 60, 200});
            }

            preview.render(out);
            std::fwrite(out.data(), 1, out.size(), stdout);
        }

        if (preview_on) {
            std::printf(
                "%-9s alvo=(%3d,%3d) area=%6u erro=%+.3f  L=%+.2f R=%+.2f  borda:%s\x1b[K\n"
                "roi=%3dx%-3d  visão=%4u us (média %.0f)  câmera=%.1f fps  detecção=%.0f%%\x1b[K\n"
                "processa %dx%d · mostra %dx%d (o preview é só desenho)\x1b[K\n"
                "H[%u..%u] S[%u..%u] V[%u..%u]  area>=%u  %s\x1b[K\n"
                "c calibra · m máscara · b borda · e exposição · r reset · +/- área · s salva · q "
                "sai\x1b[K",
                track_state_name(r.state), r.target.x, r.target.y, r.area, r.error_norm, r.cmd.left,
                r.cmd.right,
                !border_on
                    ? "off"
                    : (r.border.left()
                           ? "ESQUERDA"
                           : (r.border.right() ? "DIREITA" : (r.border_hit() ? "FRENTE" : "-"))),
                r.roi.w, r.roi.h, r.timings.total_us, latency.mean_us(), fps,
                100.0 * detections / frames, frame.width, frame.height, preview.width(),
                preview.height(), range.h_min, range.h_max, range.s_min, range.s_max, range.v_min,
                range.v_max, min_area, notice);
        } else {
            // Uma linha só, reescrita no lugar: sem preview o terminal pode não
            // entender nem cursor addressing.
            std::printf(
                "\r%-9s alvo=(%3d,%3d) area=%6u erro=%+.3f L=%+.2f R=%+.2f  %4u us  %.1f fps  ",
                track_state_name(r.state), r.target.x, r.target.y, r.area, r.error_norm, r.cmd.left,
                r.cmd.right, r.timings.total_us, fps);
        }
        std::fflush(stdout);

        if (duration_s > 0.0f && static_cast<float>(now - started) * 1e-6f >= duration_s) break;
    }

    g_terminal.restore();
    if (preview_on) std::fputs(kShowCursor, stdout), std::fputs(kReset, stdout);
    camera.close();
    if (log) {
        std::fclose(log);
        std::printf("\nlog gravado em %s", log_path.c_str());
    }
    std::printf("\n%u frames · %u com detecção (%.0f%%) · latência média %.1f us (pior %u us)\n",
                frames, detections, frames ? 100.0 * detections / frames : 0.0, latency.mean_us(),
                latency.max_us);
    std::printf("faixa final:  --range %u,%u,%u,%u,%u,%u  --min-area %u\n", range.h_min,
                range.h_max, range.s_min, range.s_max, range.v_min, range.v_max, min_area);
    return 0;
}
