#include "linux/sysmetrics.hpp"

#include <sched.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "ecv/core/profile.hpp"

namespace ecv::linux_platform {
namespace {

/// Lê um único inteiro de um arquivo de uma linha. -1 quando o arquivo não existe.
int64_t read_int_file(const char* path) {
    std::FILE* f = std::fopen(path, "r");
    if (!f) return -1;
    long long v = -1;
    const int n = std::fscanf(f, "%lld", &v);
    std::fclose(f);
    return n == 1 ? static_cast<int64_t>(v) : -1;
}

/// Agregado de /proc/stat: jiffies ocupados e totais desde o boot.
bool read_cpu_jiffies(uint64_t& busy, uint64_t& total) {
    std::FILE* f = std::fopen("/proc/stat", "r");
    if (!f) return false;
    unsigned long long v[10] = {};
    const int n = std::fscanf(f, "cpu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu", &v[0],
                              &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9]);
    std::fclose(f);
    if (n < 4) return false;
    total = 0;
    for (int i = 0; i < n; ++i) total += v[i];
    // v[3] = idle, v[4] = iowait: nenhum dos dois é trabalho útil.
    const uint64_t idle = v[3] + (n > 4 ? v[4] : 0);
    busy = total > idle ? total - idle : 0;
    return true;
}

/// utime+stime do processo. O comm entre parênteses pode conter espaço, então
/// a varredura começa depois do último ')'.
uint64_t read_proc_ticks() {
    std::FILE* f = std::fopen("/proc/self/stat", "r");
    if (!f) return 0;
    char buf[1024];
    const size_t got = std::fread(buf, 1, sizeof(buf) - 1, f);
    std::fclose(f);
    buf[got] = '\0';
    const char* p = std::strrchr(buf, ')');
    if (!p) return 0;
    unsigned long long utime = 0, stime = 0;
    // Depois de ')' vem o campo 3 (state); utime é o 14, stime o 15.
    if (std::sscanf(p + 1, " %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %llu %llu", &utime,
                    &stime) != 2) {
        return 0;
    }
    return utime + stime;
}

int64_t read_rss_kb() {
    std::FILE* f = std::fopen("/proc/self/status", "r");
    if (!f) return -1;
    char line[256];
    int64_t rss = -1;
    while (std::fgets(line, sizeof(line), f)) {
        long long v = 0;
        if (std::sscanf(line, "VmRSS: %lld kB", &v) == 1) {
            rss = v;
            break;
        }
    }
    std::fclose(f);
    return rss;
}

/// Frequência do núcleo onde este processo está rodando agora. Ler sempre a
/// cpu0 mente quando o processo está fixado em outro núcleo (`taskset`) ou
/// quando o governor escala cada núcleo por conta própria.
int32_t read_cur_freq_khz() {
    const int cpu = sched_getcpu();
    char path[128];
    std::snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq",
                  cpu >= 0 ? cpu : 0);
    int64_t khz = read_int_file(path);
    if (khz < 0 && cpu > 0) {
        khz = read_int_file("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq");
    }
    return static_cast<int32_t>(khz);
}

void read_model(char* out, size_t cap) {
    out[0] = '\0';
    std::FILE* f = std::fopen("/proc/cpuinfo", "r");
    if (!f) return;
    char line[512];
    char fallback[128] = {};
    while (std::fgets(line, sizeof(line), f)) {
        const char* colon = std::strchr(line, ':');
        if (!colon) continue;
        // "Model" só aparece no Raspberry Pi e nomeia a placa, não o núcleo.
        if (std::strncmp(line, "Model\t", 6) == 0 || std::strncmp(line, "Model ", 6) == 0) {
            std::snprintf(out, cap, "%s", colon + 2);
            break;
        }
        if (!fallback[0] && std::strncmp(line, "model name", 10) == 0) {
            std::snprintf(fallback, sizeof(fallback), "%s", colon + 2);
        }
    }
    std::fclose(f);
    if (!out[0]) std::snprintf(out, cap, "%s", fallback);
    char* nl = std::strchr(out, '\n');
    if (nl) *nl = '\0';
}

}  // namespace

bool SysMonitor::open() {
    ncpu_ = static_cast<int32_t>(sysconf(_SC_NPROCESSORS_ONLN));
    if (ncpu_ < 1) ncpu_ = 1;
    ticks_per_s_ = sysconf(_SC_CLK_TCK);
    if (ticks_per_s_ < 1) ticks_per_s_ = 100;
    read_model(model_, sizeof(model_));
    sample();  // prime: descarta a primeira janela
    return true;
}

SysSnapshot SysMonitor::sample() {
    SysSnapshot s;
    s.ncpu = ncpu_;
    s.temp_milli_c = static_cast<int32_t>(read_int_file("/sys/class/thermal/thermal_zone0/temp"));
    s.freq_khz = read_cur_freq_khz();
    s.rss_kb = read_rss_kb();

    uint64_t busy = 0, total = 0;
    const bool have_cpu = read_cpu_jiffies(busy, total);
    const uint64_t proc = read_proc_ticks();
    const uint64_t now_us = micros();

    if (primed_ && have_cpu) {
        const uint64_t d_total = total - prev_total_;
        if (d_total > 0) s.cpu_all_pct = 100.0 * static_cast<double>(busy - prev_busy_) / d_total;
        const uint64_t d_wall = now_us - prev_wall_us_;
        if (d_wall > 0) {
            const double proc_s = static_cast<double>(proc - prev_proc_) / ticks_per_s_;
            s.cpu_proc_pct = 100.0 * proc_s / (static_cast<double>(d_wall) / 1e6);
        }
    }

    prev_busy_ = busy;
    prev_total_ = total;
    prev_proc_ = proc;
    prev_wall_us_ = now_us;
    primed_ = true;
    return s;
}

}  // namespace ecv::linux_platform
