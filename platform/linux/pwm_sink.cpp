#include "pwm_sink.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ecv::linux_hal {
namespace {

bool write_file(const std::string& path, const char* value) {
    const int fd = ::open(path.c_str(), O_WRONLY);
    if (fd < 0) return false;
    const ssize_t n = ::write(fd, value, std::strlen(value));
    ::close(fd);
    return n > 0;
}

bool exists(const std::string& path) {
    return ::access(path.c_str(), F_OK) == 0;
}

}  // namespace

uint32_t duty_ns(float normalized, uint32_t period_ns, float max_duty) {
    if (!(normalized > 0.0f)) return 0;  // pega NaN junto, que viraria duty absurdo
    if (normalized > max_duty) normalized = max_duty;
    if (normalized > 1.0f) normalized = 1.0f;
    return static_cast<uint32_t>(normalized * static_cast<float>(period_ns));
}

BridgeDuties duties_for(const MotorCommand& cmd, uint32_t period_ns, float max_duty) {
    BridgeDuties d;
    d.left_a = duty_ns(cmd.left, period_ns, max_duty);
    d.left_b = duty_ns(-cmd.left, period_ns, max_duty);
    d.right_a = duty_ns(cmd.right, period_ns, max_duty);
    d.right_b = duty_ns(-cmd.right, period_ns, max_duty);
    return d;
}

bool SysfsPwmSink::export_channel(const PwmChannel& ch, Channel& out) {
    if (!exists(ch.chip)) {
        error_ = ch.chip +
                 " não existe: nenhum canal de PWM exposto. Num Raspberry Pi, "
                 "adicione `dtoverlay=pwm-2chan` ao config.txt e reinicie.";
        return false;
    }

    char buf[64];
    std::snprintf(buf, sizeof(buf), "%d", ch.index);
    const std::string dir = ch.chip + "/pwm" + buf;

    if (!exists(dir) && !write_file(ch.chip + "/export", buf)) {
        error_ = "não consegui exportar " + dir + ": " + std::strerror(errno) +
                 " (o chip expõe canais suficientes? confira " + ch.chip + "/npwm)";
        return false;
    }

    std::snprintf(buf, sizeof(buf), "%u", cfg_.period_ns);
    write_file(dir + "/duty_cycle", "0");  // duty > período novo faria o write falhar
    if (!write_file(dir + "/period", buf)) {
        error_ = "não consegui configurar o período de " + dir;
        return false;
    }
    write_file(dir + "/enable", "1");

    out.path = dir + "/duty_cycle";
    out.last_ns = kDutyUnknown;
    out.duty_fd = ::open(out.path.c_str(), O_WRONLY);
    if (out.duty_fd < 0) {
        error_ = "não consegui abrir " + out.path + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

bool SysfsPwmSink::open() {
    write_failures_ = 0;
    writes_issued_ = 0;
    if (!export_channel(cfg_.left_a, la_) || !export_channel(cfg_.left_b, lb_) ||
        !export_channel(cfg_.right_a, ra_) || !export_channel(cfg_.right_b, rb_)) {
        close();
        return false;
    }
    open_ = true;
    stop();
    return true;
}

void SysfsPwmSink::close() {
    if (open_) stop();
    for (Channel* ch : {&la_, &lb_, &ra_, &rb_}) {
        if (ch->duty_fd >= 0) ::close(ch->duty_fd);
        ch->duty_fd = -1;
        ch->last_ns = kDutyUnknown;
        // Desliga o canal: duty zero já para, mas um canal habilitado que
        // ninguém mais controla é uma saída de potência sem dono.
        if (!ch->path.empty()) {
            const std::string dir = ch->path.substr(0, ch->path.rfind('/'));
            write_file(dir + "/enable", "0");
        }
    }
    open_ = false;
}

void SysfsPwmSink::write_duty_ns(Channel& ch, uint32_t ns, bool force) {
    if (ch.duty_fd < 0) return;
    // sysfs não tem estado escondido: reescrever o mesmo duty não muda nada no
    // hardware e custa uma syscall. Em regime, dois dos quatro canais estão
    // sempre em zero e o comando muda pouco entre frames.
    if (!force && ns == ch.last_ns) return;

    char buf[32];
    const int n = std::snprintf(buf, sizeof(buf), "%u", ns);
    if (::pwrite(ch.duty_fd, buf, static_cast<size_t>(n), 0) < 0) {
        ++write_failures_;
        ch.last_ns = kDutyUnknown;  // não confiar no cache depois de uma falha
        return;
    }
    ++writes_issued_;
    ch.last_ns = ns;
}

void SysfsPwmSink::write(const MotorCommand& cmd) {
    if (!open_) return;
    const BridgeDuties d = duties_for(cmd, cfg_.period_ns, cfg_.max_duty);

    // Zera antes de levantar. Numa inversão de sentido a ordem inversa deixaria,
    // por um instante, as duas entradas da ponte com duty — que nessas pontes
    // significa freio, não avanço.
    Channel* const chans[4] = {&la_, &lb_, &ra_, &rb_};
    const uint32_t want[4] = {d.left_a, d.left_b, d.right_a, d.right_b};
    for (int i = 0; i < 4; ++i)
        if (want[i] == 0) write_duty_ns(*chans[i], 0, false);
    for (int i = 0; i < 4; ++i)
        if (want[i] != 0) write_duty_ns(*chans[i], want[i], false);
}

void SysfsPwmSink::stop() {
    // Parada é segurança: ignora o cache e escreve zero de verdade nos quatro.
    for (Channel* ch : {&la_, &lb_, &ra_, &rb_}) write_duty_ns(*ch, 0, true);
}

}  // namespace ecv::linux_hal
