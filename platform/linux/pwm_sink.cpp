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

bool SysfsPwmSink::export_channel(const PwmChannel& ch, Channel& out) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%d", ch.index);
    const std::string dir = ch.chip + "/pwm" + buf;

    if (!exists(dir) && !write_file(ch.chip + "/export", buf)) {
        error_ = "não consegui exportar " + dir + ": " + std::strerror(errno);
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
    out.duty_fd = ::open(out.path.c_str(), O_WRONLY);
    if (out.duty_fd < 0) {
        error_ = "não consegui abrir " + out.path;
        return false;
    }
    return true;
}

bool SysfsPwmSink::open() {
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
    }
    open_ = false;
}

void SysfsPwmSink::write_duty(Channel& ch, float normalized) {
    if (ch.duty_fd < 0) return;
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > cfg_.max_duty) normalized = cfg_.max_duty;

    char buf[32];
    const int n =
        std::snprintf(buf, sizeof(buf), "%u", static_cast<uint32_t>(normalized * cfg_.period_ns));
    ::pwrite(ch.duty_fd, buf, static_cast<size_t>(n), 0);
}

void SysfsPwmSink::write(const MotorCommand& cmd) {
    if (!open_) return;
    // Sentido = qual entrada da ponte recebe o duty; a outra vai a zero.
    write_duty(la_, cmd.left > 0 ? cmd.left : 0.0f);
    write_duty(lb_, cmd.left < 0 ? -cmd.left : 0.0f);
    write_duty(ra_, cmd.right > 0 ? cmd.right : 0.0f);
    write_duty(rb_, cmd.right < 0 ? -cmd.right : 0.0f);
}

void SysfsPwmSink::stop() {
    for (Channel* ch : {&la_, &lb_, &ra_, &rb_}) write_duty(*ch, 0.0f);
}

}  // namespace ecv::linux_hal
