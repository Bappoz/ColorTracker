#!/usr/bin/env bash
# Bateria extrema que roda NA PLACA. Amostra telemetria (temperatura, frequência,
# throttling, carga) em segundo plano enquanto empurra o pipeline pelos piores casos,
# inclusive com a câmera real e com os outros núcleos ocupados.
#
# Uso: ~/ecv/ecv-stress.sh [--device /dev/video0] [--out DIR] [--quick]
#
# Saída em $OUT: telemetry.csv (série temporal), events.csv (marcadores de fase),
# frames.csv + summary.json (soak), cam.csv (câmera) e stress.log (tudo).
set -uo pipefail

DEVICE=/dev/video0
OUT="$HOME/ecv/out"
QUICK=0
while [ $# -gt 0 ]; do
  case "$1" in
  --device)
    DEVICE="$2"
    shift 2
    ;;
  --out)
    OUT="$2"
    shift 2
    ;;
  --quick)
    QUICK=1
    shift
    ;;
  *)
    echo "opção desconhecida: $1" >&2
    exit 2
    ;;
  esac
done

BIN="$HOME/ecv/bin"
mkdir -p "$OUT"
LOG="$OUT/stress.log"
: >"$LOG"
exec > >(tee -a "$LOG") 2>&1

T0=$(date +%s)
rel() { echo "$(($(date +%s) - T0))"; }

# ---------------------------------------------------------------- telemetria
TELE="$OUT/telemetry.csv"
EVENTS="$OUT/events.csv"
echo "t_s,temp_c,freq_mhz,volts_core,throttled,load1,mem_free_mb" >"$TELE"
echo "t_s,phase,event" >"$EVENTS"

mark() {
  echo "$(rel),$1,$2" >>"$EVENTS"
  echo ""
  echo "===== [$(rel)s] $1 — $2"
}

sampler() {
  while :; do
    local temp freq volts thr load memf
    temp=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
    freq=$(awk '{printf "%d", $1/1000}' /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null)
    volts=$(vcgencmd measure_volts core 2>/dev/null | tr -dc '0-9.')
    thr=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
    load=$(awk '{print $1}' /proc/loadavg)
    memf=$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo)
    echo "$(rel),${temp:-},${freq:-},${volts:-},${thr:-},${load:-},${memf:-}" >>"$TELE"
    sleep 1
  done
}
sampler &
SAMPLER_PID=$!
# Garante que nenhum processo de carga sobreviva a um Ctrl-C.
cleanup() {
  kill "$SAMPLER_PID" 2>/dev/null
  pkill -P $$ 2>/dev/null
}
trap cleanup EXIT INT TERM

burn_start() {
  BURN_PIDS=""
  for _ in $(seq 1 "$1"); do
    (while :; do :; done) &
    BURN_PIDS="$BURN_PIDS $!"
  done
}
burn_stop() {
  for p in $BURN_PIDS; do kill "$p" 2>/dev/null; done
  # `wait` sem argumento também esperaria o amostrador, que é laço infinito:
  # colhe só os processos de carga, um a um.
  for p in $BURN_PIDS; do wait "$p" 2>/dev/null; done
  BURN_PIDS=""
}

secs() { if [ "$QUICK" -eq 1 ]; then echo "$(($1 / 3 + 5))"; else echo "$1"; fi; }
frames() { if [ "$QUICK" -eq 1 ]; then echo "$(($1 / 4))"; else echo "$1"; fi; }

# ------------------------------------------------------------------ contexto
mark setup contexto
echo "--- modelo ---"
tr -d '\0' </proc/device-tree/model 2>/dev/null
echo
echo "--- kernel ---"
uname -a
echo "--- governor ---"
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null
echo "--- throttled inicial ---"
vcgencmd get_throttled 2>/dev/null || echo "vcgencmd indisponível"
echo "--- câmeras ---"
ls -1 /dev/video* 2>/dev/null || echo "nenhum /dev/video*"
"$BIN/ecv_probe" --list --device "$DEVICE" 2>&1 | head -30 || true

# ------------------------------------------------------------------- T0 base
mark T0 "baseline ocioso $(secs 20)s"
sleep "$(secs 20)"

# ------------------------------- T1 robô inteiro sintético, todas as fases
mark T1 "soak completo por fases + varredura de resolução"
"$BIN/ecv_soak" --frames "$(frames 400)" --sweep \
  --csv "$OUT/frames.csv" --json "$OUT/summary.json"

# ------------------------------------------------ T2 pior caso sustentado
mark T2 "pior caso 640x480 sustentado (alvo perdido = varredura total)"
"$BIN/ecv_soak" --frames "$(frames 600)" --width 640 --height 480 \
  --json "$OUT/summary-worst.json" --quiet

# ------------------------------- T3 visão disputando CPU com 3 núcleos ocupados
mark T3 "visão com os outros 3 núcleos saturados"
burn_start 3
sleep 3
"$BIN/ecv_soak" --frames "$(frames 400)" --width 320 --height 240 \
  --json "$OUT/summary-contended.json" --quiet
burn_stop
sleep 3

# ------------------------------------------------------- T4 câmera real
if [ -e "$DEVICE" ]; then
  mark T4 "câmera real headless por $(secs 45)s"
  "$BIN/ecv_probe" --device "$DEVICE" --no-preview --seconds "$(secs 45)" \
    --log "$OUT/cam.csv" || echo "probe falhou (ver acima)"

  mark T5 "robô completo com câmera (dry-run) por $(secs 45)s"
  "$BIN/ecv_sumo_robot" --device "$DEVICE" --dry-run --seconds "$(secs 45)" ||
    echo "sumo_robot falhou (ver acima)"
else
  mark T4 "PULADO — $DEVICE não existe"
fi

# ---------------------------------------------------------------- T6 cooldown
mark T6 "resfriamento $(secs 30)s"
sleep "$(secs 30)"

mark fim "encerrado"
echo "--- throttled final ---"
vcgencmd get_throttled 2>/dev/null || true
echo "--- temperatura final ---"
awk '{printf "%.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null

echo ""
echo "==> dados em $OUT/"
ls -la "$OUT/"
