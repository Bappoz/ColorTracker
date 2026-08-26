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

# Descobre a sub-rede real da interface com rota padrão. Não dá para assumir /24:
# hotspot de iPhone entrega /28, e varrer .1–.254 ali é faixa errada e lenta.
dev=$(ip -4 -o route get 1.1.1.1 2>/dev/null | grep -oE 'dev [^ ]+' | awk '{print $2}')
cidr=$(ip -4 -o addr show dev "${dev:-lo}" 2>/dev/null | awk '{print $4}' | head -1)
if [ -z "${cidr:-}" ]; then
  say "ERRO: não consegui descobrir a sub-rede local"
  exit 1
fi

# network+1 .. broadcast-1, a partir de IP e prefixo.
range=$(awk -v cidr="$cidr" 'BEGIN{
  split(cidr, p, "/"); split(p[1], o, ".");
  ip = o[1]*16777216 + o[2]*65536 + o[3]*256 + o[4];
  bits = p[2] + 0;
  mask = (bits == 0) ? 0 : (4294967295 - (2 ^ (32 - bits) - 1));
  net = int(ip / 1) ; net = and_net(ip, mask);
  print net + 1, net + (2 ^ (32 - bits)) - 2;
}
function and_net(a, m,   i, r, bit) {
  r = 0; bit = 2147483648;
  for (i = 0; i < 32; i++) {
    if (int(a / bit) % 2 == 1 && int(m / bit) % 2 == 1) r += bit;
    bit /= 2;
  }
  return r;
}')
first=${range%% *}
last=${range##* }
hosts=$((last - first + 1))
if [ "$hosts" -gt 1024 ]; then
  say "ERRO: sub-rede $cidr tem $hosts hosts — grande demais para varrer."
  exit 1
fi
say "==> varrendo $cidr ($hosts hosts) procurando MAC de Raspberry Pi"

# Popula a tabela ARP: um ping por host, todos em paralelo.
i=$first
while [ "$i" -le "$last" ]; do
  a=$((i / 16777216 % 256)); b=$((i / 65536 % 256))
  c=$((i / 256 % 256)); d=$((i % 256))
  ping -c1 -W1 "$a.$b.$c.$d" >/dev/null 2>&1 &
  i=$((i + 1))
done
wait

found=$(ip neigh | grep -iE "$OUI" | awk '{print $1}' | head -1)
if [ -n "${found:-}" ]; then
  mac=$(ip neigh | grep -iE "$OUI" | awk '{print $5}' | head -1)
  say "    achei $found ($mac)"
  echo "$found"
  exit 0
fi

say "ERRO: nenhuma Raspberry Pi encontrada em $cidr."
say "      Confira se a placa entrou na mesma rede. Alguns hotspots isolam"
say "      clientes entre si — nesse caso nenhum host vê nenhum outro."
exit 1
