// Amostragem de CPU, temperatura, frequência e memória via sysfs/procfs.
//
// Fica em platform/ porque lê arquivo do kernel: o núcleo em include/ecv não
// pode saber que Linux existe. Serve ao relatório de desempenho — sem isto a
// latência sozinha não diz se o robô está com CPU sobrando ou já em throttling.
#pragma once

#include <cstdint>

namespace ecv::linux_platform {

/// Estado instantâneo da máquina. Campos com -1 = indisponível neste alvo
/// (thermal_zone e cpufreq não existem em todo kernel/container).
struct SysSnapshot {
    double cpu_all_pct = 0.0;   ///< uso somado de todos os núcleos, 0..100
    double cpu_proc_pct = 0.0;  ///< uso do próprio processo, 100 = um núcleo cheio
    int32_t temp_milli_c = -1;  ///< thermal_zone0, milésimos de °C
    int32_t freq_khz = -1;      ///< scaling_cur_freq do núcleo em uso agora
    int64_t rss_kb = -1;        ///< VmRSS do processo
    int32_t ncpu = 0;
};

/// Diferencial entre duas chamadas de `sample()`. A primeira chamada devolve
/// percentuais zerados porque não existe janela anterior para comparar.
class SysMonitor {
public:
    bool open();
    SysSnapshot sample();

    int32_t ncpu() const { return ncpu_; }
    /// Nome do modelo lido de /proc/cpuinfo ("Model" no Raspberry Pi,
    /// "model name" no x86). Vazio quando não há.
    const char* model() const { return model_; }

private:
    uint64_t prev_busy_ = 0;
    uint64_t prev_total_ = 0;
    uint64_t prev_proc_ = 0;
    uint64_t prev_wall_us_ = 0;
    int32_t ncpu_ = 0;
    long ticks_per_s_ = 100;
    bool primed_ = false;
    char model_[128] = {};
};

}  // namespace ecv::linux_platform
