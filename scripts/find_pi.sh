#!/usr/bin/env bash
# Acha a Pi na rede em que este notebook está agora, mesmo que o IP tenha mudado
# (hotspot novo, DHCP diferente, outra faculdade). Imprime só o IP, para poder
# ser usado direto: ECV_PI=pedro@$(scripts/find_pi.sh) just rpi-flash
#
# Ordem: mDNS primeiro (barato); se a rede bloquear multicast, varre o /24 atual
# procurando o OUI da Raspberry Pi Foundation na tabela ARP.
set -uo pipefail

MDNS="${ECV_PI_HOST:-pi-pedro.local}"
# OUIs da Raspberry Pi Foundation/Trading. b8:27:eb cobre Pi 1 a 3.
OUI='b8:27:eb|dc:a6:32|e4:5f:01|d8:3a:dd|2c:cf:67'
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1
say() { [ "$QUIET" -eq 1 ] || echo "$@" >&2; }

say "==> tentando mDNS ($MDNS)"
if command -v avahi-resolve >/dev/null; then
  # -4: sem isso o avahi pode devolver o IPv6 link-local, que o scp não usa.
  ip=$(timeout 5 avahi-resolve -4 -n "$MDNS" 2>/dev/null | awk '{print $2}' | head -1)
  if [ -n "${ip:-}" ]; then
    say "    achei por mDNS"
    echo "$ip"
    exit 0
  fi
fi
say "    mDNS não respondeu"

# Descobre o /24 da interface que tem a rota padrão.
cidr=$(ip -4 -o route get 1.1.1.1 2>/dev/null |
  grep -oE 'src [0-9.]+' | awk '{print $2}')
if [ -z "${cidr:-}" ]; then
  say "ERRO: não consegui descobrir o endereço local"
  exit 1
fi
base="${cidr%.*}"
say "==> varrendo ${base}.0/24 procurando MAC de Raspberry Pi"

# Popula a tabela ARP: um ping por host, todos em paralelo.
for i in $(seq 1 254); do
  ping -c1 -W1 "${base}.${i}" >/dev/null 2>&1 &
done
wait

found=$(ip neigh | grep -iE "$OUI" | awk '{print $1}' | head -1)
if [ -n "${found:-}" ]; then
  mac=$(ip neigh | grep -iE "$OUI" | awk '{print $5}' | head -1)
  say "    achei $found ($mac)"
  echo "$found"
  exit 0
fi

say "ERRO: nenhuma Raspberry Pi encontrada em ${base}.0/24."
say "      Confira se a placa entrou na mesma rede. Alguns hotspots isolam"
say "      clientes entre si — nesse caso nenhum host vê nenhum outro."
exit 1
