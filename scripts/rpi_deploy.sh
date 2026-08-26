#!/usr/bin/env bash
# "Grava" o código na placa: compila cruzado, sincroniza os binários e, opcionalmente,
# reinstala o serviço. É o equivalente ao flash de uma ESP32 — depois disso a Pi roda
# sozinha, sem monitor nem teclado.
#
# Uso: scripts/rpi_deploy.sh [alvo] [--service] [--restart]
#   alvo      usuario@host ou alias de ~/.ssh/config (padrão: $ECV_PI, senão "sumo")
#   --service (re)instala a unit systemd de usuário na placa
#   --restart reinicia o serviço depois de sincronizar
set -euo pipefail

TARGET="${ECV_PI:-sumo}"
INSTALL_SERVICE=0
RESTART=0
for a in "$@"; do
  case "$a" in
  --service) INSTALL_SERVICE=1 ;;
  --restart) RESTART=1 ;;
  -*)
    echo "opção desconhecida: $a" >&2
    exit 2
    ;;
  *) TARGET="$a" ;;
  esac
done

cd "$(dirname "$0")/.."
REMOTE="ecv"

echo "==> placa"
ARCH="$(ssh "$TARGET" 'uname -m')"
echo "    $TARGET => $ARCH"
if [ "$ARCH" != "aarch64" ]; then
  echo "    ERRO: este toolchain gera aarch64; a placa reporta '$ARCH'." >&2
  exit 1
fi

echo "==> build cruzado"
cmake -S . -B build-rpi3 -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-rpi3.cmake \
  -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build build-rpi3 -j >/dev/null

echo "==> sincronizando para ~/$REMOTE"
ssh "$TARGET" "mkdir -p ~/$REMOTE/bin ~/$REMOTE/out"

BINS=(
  build-rpi3/apps/ecv_probe
  build-rpi3/apps/ecv_soak
  build-rpi3/apps/ecv_bench
  build-rpi3/apps/ecv_sumo_robot
  build-rpi3/apps/ecv_sumo_sim
  build-rpi3/apps/ecv_calibrate
  build-rpi3/tests/ecv_tests
)

# rsync manda só o delta; sem ele na placa, scp resolve (binário estático ~1 MB).
if ssh "$TARGET" 'command -v rsync >/dev/null'; then
  rsync -az --info=stats0 "${BINS[@]}" "$TARGET:$REMOTE/bin/"
  rsync -az scripts/pi/ecv-stress.sh scripts/pi/RUNBOOK.md "$TARGET:$REMOTE/"
else
  scp -q "${BINS[@]}" "$TARGET:$REMOTE/bin/"
  scp -q scripts/pi/ecv-stress.sh scripts/pi/RUNBOOK.md "$TARGET:$REMOTE/"
fi
ssh "$TARGET" "chmod +x ~/$REMOTE/bin/* ~/$REMOTE/ecv-stress.sh"

if [ "$INSTALL_SERVICE" -eq 1 ]; then
  echo "==> instalando serviço de usuário"
  ssh "$TARGET" "mkdir -p ~/.config/systemd/user"
  scp -q scripts/pi/ecv-sumo.service "$TARGET:.config/systemd/user/"
  # Sem sobrescrever a configuração já ajustada na placa.
  ssh "$TARGET" "[ -f ~/$REMOTE/robot.env ] || printf 'ECV_ROBOT_ARGS=--dry-run\n' > ~/$REMOTE/robot.env"
  ssh "$TARGET" "systemctl --user daemon-reload && systemctl --user enable ecv-sumo.service"
  RESTART=1
fi

if [ "$RESTART" -eq 1 ]; then
  echo "==> reiniciando serviço"
  ssh "$TARGET" "systemctl --user restart ecv-sumo.service" || true
  ssh "$TARGET" "systemctl --user --no-pager status ecv-sumo.service | head -12" || true
fi

echo "==> pronto. Na placa: ~/$REMOTE/bin/  ·  runbook em ~/$REMOTE/RUNBOOK.md"
