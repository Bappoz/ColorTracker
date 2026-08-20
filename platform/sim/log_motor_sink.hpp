// Atuador de mentira: imprime o que os motores fariam. Deixa o loop de controle
// inteiro rodar e ser verificado antes de existir hardware.
#pragma once

#include <cstdio>

#include "ecv/hal/motor_sink.hpp"

namespace ecv::sim {

class LogMotorSink : public MotorSink {
public:
    explicit LogMotorSink(bool verbose = false) : verbose_(verbose) {}

    bool open() override { return true; }
    void close() override { stop(); }

    void write(const MotorCommand& cmd) override {
        last_ = cmd;
        ++writes_;
        if (verbose_) std::printf("[motor] L=%+.3f R=%+.3f\n", cmd.left, cmd.right);
    }

    const char* name() const override { return "log"; }

    const MotorCommand& last() const { return last_; }
    uint32_t writes() const { return writes_; }

private:
    bool verbose_;
    MotorCommand last_;
    uint32_t writes_ = 0;
};

}  // namespace ecv::sim
