// Leitura/escrita de PPM binário (P6). Formato de imagem mais simples que
// existe: sem dependência, abre em qualquer visualizador e serve de "dataset"
// versionável para os testes.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ecv/core/types.hpp"
#include "ecv/hal/frame_source.hpp"

namespace ecv::sim {

struct PpmImage {
    std::vector<uint8_t> pixels;  ///< RGB888 empacotado
    int32_t width = 0;
    int32_t height = 0;

    ImageView view() const {
        return ImageView{pixels.data(), width, height, width * 3, PixelFormat::kRgb888};
    }
};

bool read_ppm(const std::string& path, PpmImage& out);
bool write_ppm(const std::string& path, const ImageView& img);

/// Grava uma máscara binária como PPM cinza — para inspecionar limiarização.
bool write_mask_ppm(const std::string& path, const MaskView& mask);

/// Reproduz uma sequência de PPMs como se fosse a câmera. Fecha o ciclo entre
/// captura real e teste offline: grava no robô, depura no laptop.
class PpmSequenceSource : public FrameSource {
public:
    PpmSequenceSource(std::vector<std::string> paths, bool loop)
        : paths_(std::move(paths)), loop_(loop) {}

    bool open() override;
    void close() override {}
    bool next(ImageView& out) override;

    int32_t width() const override { return img_.width; }
    int32_t height() const override { return img_.height; }
    PixelFormat format() const override { return PixelFormat::kRgb888; }
    const char* name() const override { return "ppm-sequence"; }

    size_t size() const { return paths_.size(); }

private:
    std::vector<std::string> paths_;
    bool loop_ = false;
    size_t index_ = 0;
    PpmImage img_;
};

}  // namespace ecv::sim
