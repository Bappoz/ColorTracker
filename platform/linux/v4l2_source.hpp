// Captura V4L2 com buffers mapeados em memória (Raspberry Pi, webcam USB).
//
// Duas decisões que definem a latência:
//  * MMAP em vez de read(): o driver escreve direto no buffer do processo, sem
//    uma cópia por frame;
//  * fila curta (2 buffers): com fila longa o robô processa um frame antigo —
//    em sumô, agir sobre onde o oponente estava há 100 ms é agir errado.
#pragma once

#include <cstdint>
#include <string>

#include "ecv/hal/frame_source.hpp"

namespace ecv::linux_hal {

struct V4l2Config {
    std::string device = "/dev/video0";
    int32_t width = 320;
    int32_t height = 240;
    uint32_t fps = 60;
    PixelFormat format = PixelFormat::kYuyv;  ///< YUYV ou RGB565, sem MJPEG
    uint32_t buffer_count = 2;

    /// Exposição e balanço de branco fixos. Com automático, o alvo muda de cor
    /// quando o oponente entra no quadro e o limiar calibrado deixa de valer.
    bool lock_exposure = true;
    int32_t exposure_absolute = 100;  ///< unidades de 100 µs (driver-dependente)
};

/// Um formato/resolução que o dispositivo aceita.
struct V4l2Mode {
    char fourcc[5] = {};
    int32_t width = 0;
    int32_t height = 0;
    uint32_t fps = 0;
    bool supported_by_ecv = false;  ///< o núcleo decodifica este formato?
};

/// Enumera o que a câmera oferece. Escreve até `max_modes` entradas e devolve
/// quantas. Sem isso, descobrir por que a captura falhou vira tentativa e erro.
int list_modes(const std::string& device, V4l2Mode* out, int max_modes);

class V4l2Source : public FrameSource {
public:
    explicit V4l2Source(const V4l2Config& cfg) : cfg_(cfg) {}
    ~V4l2Source() override { close(); }

    bool open() override;
    void close() override;
    bool next(ImageView& out) override;

    int32_t width() const override { return cfg_.width; }
    int32_t height() const override { return cfg_.height; }
    PixelFormat format() const override { return cfg_.format; }
    const char* name() const override { return "v4l2"; }

    const std::string& last_error() const { return error_; }

    /// Liga/desliga a trava de exposição com a câmera já aberta. Serve para ver
    /// na bancada o que o automático faz com a máscara quando a cena muda.
    void set_exposure_lock(bool locked);
    bool exposure_locked() const { return cfg_.lock_exposure; }

private:
    struct Buffer {
        void* start = nullptr;
        size_t length = 0;
    };

    bool set_format();
    bool request_buffers();
    bool start_streaming();
    void apply_camera_controls();
    bool fail(const char* what);

    V4l2Config cfg_;
    int fd_ = -1;
    Buffer buffers_[8];
    uint32_t buffer_count_ = 0;
    int32_t stride_ = 0;
    int queued_index_ = -1;
    std::string error_;
};

}  // namespace ecv::linux_hal
