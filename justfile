build_dir := "build"
# Alvo do Raspberry Pi: alias de ~/.ssh/config. Sobrescreva com ECV_PI=usuario@ip
pi := env_var_or_default("ECV_PI", "sumo")

default:
    @just --list

# Configura o build (Release por padrão; `just configure Debug` para depurar)
configure type="Release":
    cmake -S . -B {{build_dir}} -DCMAKE_BUILD_TYPE={{type}}

build:
    cmake --build {{build_dir}} -j

# Compila e roda a suíte de testes
test: build
    ./{{build_dir}}/tests/ecv_tests

# Latência por estágio na máquina atual
bench frames="400" format="rgb565":
    ./{{build_dir}}/apps/ecv_bench --frames {{frames}} --format {{format}}

# Pipeline completo sobre cena sintética (sem hardware)
sim *ARGS:
    ./{{build_dir}}/apps/ecv_sumo_sim {{ARGS}}

# Faixa HSV a partir de uma foto: just calibrate frame.ppm 120 90 60 60
calibrate image x y w h:
    ./{{build_dir}}/apps/ecv_calibrate {{image}} {{x}} {{y}} {{w}} {{h}}

# Modos que a câmera aceita (só os marcados com * o núcleo decodifica)
cam-list device="/dev/video0":
    ./{{build_dir}}/apps/ecv_probe --list --device {{device}}

# Detecção ao vivo com preview no terminal. `just probe --mask` mostra a máscara.
probe *ARGS:
    ./{{build_dir}}/apps/ecv_probe {{ARGS}}

# Põe o alvo no centro do quadro e mede a faixa HSV dele
cam-calibrate *ARGS:
    ./{{build_dir}}/apps/ecv_probe --calibrate {{ARGS}}

# Loop do robô no Raspberry Pi (--dry-run não aciona os motores)
robot *ARGS:
    ./{{build_dir}}/apps/ecv_sumo_robot {{ARGS}}

# Robô inteiro por fases (busca->oclusao->borda->recalibracao) com estatística
soak *ARGS:
    ./{{build_dir}}/apps/ecv_soak {{ARGS}}

# Build cruzado para Raspberry Pi 3 com SO 64 bits (binário estático)
rpi-build:
    cmake -S . -B build-rpi3 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-rpi3.cmake \
      -DCMAKE_BUILD_TYPE=Release
    cmake --build build-rpi3 -j

# Copia para a placa, roda testes + soak + bench e traz CSV/JSON de volta
rpi-bench target frames="400":
    ./scripts/rpi_bench.sh {{target}} {{frames}}

# "Grava" na placa: build cruzado + envio + reinício do serviço (o flash da ESP32)
rpi-flash:
    ./scripts/rpi_deploy.sh {{pi}} --restart

# Primeira vez numa placa nova: envia tudo e instala o serviço de boot
rpi-install:
    ./scripts/rpi_deploy.sh {{pi}} --service

# Bateria extrema na placa (telemetria + fases + pior caso + câmera)
rpi-stress *ARGS:
    ssh {{pi}} "~/ecv/ecv-stress.sh {{ARGS}}"

# Traz tudo que a placa produziu para bench-out/pi/
rpi-pull:
    mkdir -p bench-out/pi
    scp -q {{pi}}:ecv/out/'*' bench-out/pi/ && ls -la bench-out/pi/

# Câmera na placa sem preview: throughput real de captura + visão
rpi-cam seconds="30":
    ssh {{pi}} "~/ecv/bin/ecv_probe --no-preview --seconds {{seconds}} --log ~/ecv/out/cam.csv"

# Log do serviço ao vivo
rpi-logs:
    ssh {{pi}} "journalctl --user -u ecv-sumo -f"

rpi-shell:
    ssh {{pi}}

# Acha o IP da placa na rede atual (mDNS, e se falhar varre procurando o MAC)
rpi-find:
    ./scripts/find_pi.sh

fmt:
    fd -e cpp -e hpp . include src platform apps tests -x clang-format -i

fmt-check:
    fd -e cpp -e hpp . include src platform apps tests -x clang-format --dry-run --Werror

# Build limpo com avisos como erro — o que o CI deveria rodar
strict:
    cmake -S . -B {{build_dir}}-strict -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic -Werror"
    cmake --build {{build_dir}}-strict -j
    ./{{build_dir}}-strict/tests/ecv_tests

# Núcleo só pode incluir <cstdint>/<cstddef> (exceto o relógio em profile.cpp)
check-portability:
    #!/usr/bin/env bash
    set -uo pipefail
    hits=$(rg -n '#include <' include/ecv src --glob '!src/core/profile.cpp' \
      | rg -v 'cstdint|cstddef' || true)
    if [ -n "$hits" ]; then
      echo "$hits"
      echo "^ includes fora do subconjunto portátil"
      exit 1
    fi
    echo "núcleo portátil: só <cstdint>/<cstddef> (relógio em src/core/profile.cpp é a exceção)"

clean:
    rm -rf {{build_dir}} {{build_dir}}-strict
