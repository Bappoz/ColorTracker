// Fronteira de atuação: o pipeline produz MotorCommand normalizado e não sabe
// se do outro lado tem L298N, DRV8833 ou um log de simulação.
#pragma once

#include "ecv/control/differential.hpp"

namespace ecv {

class MotorSink {
public:
    virtual ~MotorSink() = default;

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual void write(const MotorCommand& cmd) = 0;

    /// Parada segura. Chamar em qualquer caminho de erro e no encerramento —
    /// um robô que perde o controlador com PWM travado não para sozinho.
    virtual void stop() { write(MotorCommand::stopped()); }

    virtual const char* name() const = 0;
};

}  // namespace ecv
