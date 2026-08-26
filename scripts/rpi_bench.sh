#!/usr/bin/env bash
# Compila cruzado, copia para o Raspberry Pi, roda a bateria e traz os dados.
#
# Uso: scripts/rpi_bench.sh usuario@host [frames_por_fase]
# Requer apenas SSH na placa — o binário vai estático, sem instalar nada lá.
set -euo pipefail

TARGET="${1:?uso: $0 usuario@host [frames_por_fase]}"
FRAMES="${2:-400}"
REMOTE_DIR="/tmp/ecv"
OUT_DIR="${ECV_OUT_DIR:-bench-out}"

here() { cd "$(dirname "$0")/.."; }
here

echo "==> arquitetura da placa"
ARCH="$(ssh "$TARGET" 'uname -m')"
echo "    $TARGET => $ARCH"
if [ "$ARCH" != "aarch64" ]; then
  echo "    ERRO: este toolchain gera aarch64; a placa reporta '$ARCH'."
  echo "    Num Raspberry Pi OS 32 bits (armv7l) instale arm-linux-gnueabihf-gcc"
  echo "    e crie o toolchain equivalente antes de rodar de novo."
  exit 1
fi

echo "==> build cruzado"
cmake -S . -B build-rpi3 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-rpi3.cmake \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build-rpi3 -j >/dev/null

echo "==> copiando"
ssh "$TARGET" "mkdir -p $REMOTE_DIR"
scp -q build-rpi3/tests/ecv_tests build-rpi3/apps/ecv_soak build-rpi3/apps/ecv_bench \
  "$TARGET:$REMOTE_DIR/"

echo "==> contexto da placa"
ssh "$TARGET" "
  echo '--- modelo ---'; cat /proc/device-tree/model 2>/dev/null; echo
  echo '--- kernel ---'; uname -a
  echo '--- governor ---'; cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null
  echo '--- throttled ---'; vcgencmd get_throttled 2>/dev/null || echo 'vcgencmd indisponível'
  echo '--- volts ---'; vcgencmd measure_volts core 2>/dev/null || true
  echo '--- temp inicial ---'; cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null
" | tee "$OUT_DIR-context.txt" 2>/dev/null || true

echo "==> suíte de testes na placa"
ssh "$TARGET" "$REMOTE_DIR/ecv_tests" | tail -3

echo "==> soak ($FRAMES frames por fase, com varredura de resolução)"
ssh "$TARGET" "$REMOTE_DIR/ecv_soak --frames $FRAMES --sweep \
  --csv $REMOTE_DIR/frames.csv --json $REMOTE_DIR/summary.json" | tee "$OUT_DIR-soak.txt"

echo "==> bench por estágio"
ssh "$TARGET" "$REMOTE_DIR/ecv_bench --frames $FRAMES" | tee "$OUT_DIR-bench.txt"

mkdir -p "$OUT_DIR"
scp -q "$TARGET:$REMOTE_DIR/frames.csv" "$TARGET:$REMOTE_DIR/summary.json" "$OUT_DIR/"
echo "==> dados em $OUT_DIR/"
