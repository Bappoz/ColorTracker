# RUNBOOK — ECV no Raspberry Pi 3

Este arquivo fica **na própria placa**, em `~/ecv/RUNBOOK.md`. Tudo aqui roda sem
monitor e sem teclado: basta energia, a câmera USB e a Wi-Fi da placa.
Do notebook: `ssh sumo` (ou `ssh sumo "comando"` para rodar direto).

```
~/ecv/
  bin/            binários estáticos (não dependem de nada instalado aqui)
  out/            saída dos testes: CSV, JSON, logs
  robot.env       argumentos do serviço
  ecv-stress.sh   bateria extrema
  RUNBOOK.md      este arquivo
```

## 1. Está tudo são?

```bash
~/ecv/bin/ecv_tests                 # suíte de testes; termina com "todos os testes passaram"
vcgencmd get_throttled              # 0x0 = ideal. Ver tabela abaixo.
vcgencmd measure_temp
ls /dev/video*                      # a câmera precisa aparecer aqui
~/ecv/bin/ecv_probe --list          # modos da câmera; só os marcados com * o núcleo decodifica
```

`get_throttled` — cada bit acende por um motivo:

| valor | significa |
|---|---|
| `0x0` | tudo certo |
| `0x1` | subtensão **agora** — troque a fonte antes de medir qualquer coisa |
| `0x4` | throttling **agora** — o clock caiu, os números não valem |
| `0x50000` | só histórico desde o boot; o estado atual está limpo |

## 2. Ver a visão funcionando

```bash
# Preview no terminal (use uma janela PEQUENA — o desenho é que limita o FPS, não a visão)
~/ecv/bin/ecv_probe

# Só a máscara binária, para ajustar a faixa de cor
~/ecv/bin/ecv_probe --mask

# Throughput real, sem o custo do desenho
~/ecv/bin/ecv_probe --no-preview --seconds 30 --log ~/ecv/out/cam.csv

# Recalibrar a cor do alvo: ponha o marcador no centro do quadro
~/ecv/bin/ecv_probe --calibrate
```

O preview é opcional e caro: ele redesenha o quadro inteiro em blocos coloridos a cada
frame, e o custo cresce com o tamanho da janela do terminal. No robô real não existe
preview — o limite é a câmera e a visão, não o terminal.

## 3. O robô inteiro

```bash
# Processa a câmera e decide, sem acionar motor. Sempre comece por aqui.
~/ecv/bin/ecv_sumo_robot --dry-run --seconds 60

# Com motores. Só com o robô no chão e espaço livre em volta.
~/ecv/bin/ecv_sumo_robot --seconds 60
```

## 4. Bateria extrema

```bash
~/ecv/ecv-stress.sh              # ~6 min: baseline, fases, pior caso, CPU saturada, câmera
~/ecv/ecv-stress.sh --quick      # versão curta para conferir que está tudo de pé
```

Deixa em `~/ecv/out/`: `telemetry.csv` (temperatura/frequência/throttling por segundo),
`events.csv` (marcadores de fase), `frames.csv` + `summary.json` (latência por frame),
`cam.csv` (câmera) e `stress.log`.

Do notebook, para puxar os dados: `just rpi-pull`.

## 5. Serviço no boot

O robô sobe sozinho quando a Pi liga — sem login, sem monitor.

```bash
systemctl --user status ecv-sumo      # está rodando?
systemctl --user restart ecv-sumo
systemctl --user stop ecv-sumo        # PARE isto antes de rodar o probe à mão:
                                      # duas coisas não abrem a mesma câmera
journalctl --user -u ecv-sumo -f      # log ao vivo
```

Argumentos ficam em `~/ecv/robot.env` (padrão: `--dry-run`). Depois de editar:
`systemctl --user restart ecv-sumo`.

Para o serviço subir no boot **sem ninguém logar**, o linger precisa estar ligado —
uma vez só, com sudo: `sudo loginctl enable-linger $USER`.

## 6. Atualizar o código

Nada é compilado aqui. Do notebook, no repositório:

```bash
just rpi-flash        # compila cruzado + envia + reinicia o serviço
```

Os binários são estáticos: não há dependência para instalar na placa.

## 7. Quando algo dá errado

| sintoma | causa provável |
|---|---|
| FPS baixíssimo no `ecv_probe` | é o preview do terminal. Use `--no-preview` ou diminua a janela |
| `/dev/video0` não existe | câmera não enumerou — reconecte e veja `dmesg \| tail` |
| `Permission denied` no `/dev/video0` | falta o grupo: `sudo usermod -aG video $USER` e religue a sessão |
| câmera ocupada | o serviço está segurando: `systemctl --user stop ecv-sumo` |
| latência disparou | veja `vcgencmd get_throttled`; `0x4` = clock caiu por calor/tensão |
| `ssh sumo` não resolve | do notebook: `just rpi-find` acha o IP na rede atual |
| trocou de rede/hotspot | a placa reconecta sozinha em qualquer SSID já salvo (`nmcli con show`); o IP muda, o nome `pi-pedro.local` não |
