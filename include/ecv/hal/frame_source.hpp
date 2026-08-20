// Fronteira de aquisição. Virtual aqui é aceitável: uma chamada indireta por
// frame (~2 ns) contra 76.800 pixels de trabalho. Dentro do loop de pixel não
// existe nenhuma função virtual — lá tudo é template ou inline.
#pragma once

#include <cstdint>

#include "ecv/core/types.hpp"

namespace ecv {

class FrameSource {
public:
    virtual ~FrameSource() = default;

    virtual bool open() = 0;
    virtual void close() = 0;

    /// Entrega o próximo frame. A view aponta para o buffer interno da fonte e
    /// vale até a chamada seguinte — nenhuma cópia acontece aqui.
    virtual bool next(ImageView& out) = 0;

    virtual int32_t width() const = 0;
    virtual int32_t height() const = 0;
    virtual PixelFormat format() const = 0;
    virtual const char* name() const = 0;
};

}  // namespace ecv
