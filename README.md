# embedded_computer_vision_models

Blocos de visão computacional em C++17 para robôs pequenos — sem OpenCV, sem
alocação dinâmica no laço, mesmo código no laptop, no Raspberry Pi e no ESP32.

O primeiro sistema montado sobre eles é o do **robô de sumô**: achar o oponente
por cor, rastrear com ROI dinâmica e Kalman, e fechar a malha num PD que sai em
PWM de ponte H. Rastreabilidade completa com o diagrama BPMN do projeto em
[`docs/PIPELINE.md`](docs/PIPELINE.md).

## Por que não OpenCV

Num Raspberry Pi o OpenCV resolveria tudo isso em 30 linhas de Python. Num
ESP32 ele simplesmente não cabe — e o objetivo é justamente ter o mesmo pipeline
nos dois. O núcleo aqui usa só `<cstdint>`: compila como componente do ESP-IDF
sem nenhuma modificação. O preço é reimplementar limiarização, morfologia e
rotulagem de componentes; o ganho é enxergar (e poder otimizar) cada passada
sobre os pixels.

## Estado

Roda de ponta a ponta em cena sintética e com webcam USB de verdade (YUYV 320x240
a 30 fps, 722 µs de visão por frame com a ROI travada), com 35 testes verdes. O
PWM por sysfs compila mas nunca acionou uma ponte H; a camada ESP32 nunca passou
por um `idf.py build` — ver o aviso em
[`platform/esp32/README.md`](platform/esp32/README.md).

## Uso

```bash
just configure && just build && just test
just bench                        # latência por estágio nesta máquina
just sim --frames 60 --occlude 20,30   # pipeline completo, sem hardware
```

### Com sua câmera

`ecv_probe` roda o pipeline sobre uma câmera V4L2 e desenha o resultado no
terminal (meio-bloco + truecolor) — caixa no alvo, cruz no centroide, ROI. Sem
GUI, então serve também por SSH num Raspberry Pi montado no robô.

```bash
just cam-list                # modos aceitos; só os marcados com * servem
just probe                   # aperte `c` com o alvo no centro e ele calibra
just probe --border --log ensaio.csv    # borda do ringue + registro por frame
just robot --dry-run         # loop do robô sem acionar os motores
```

Dentro do probe, sem reiniciar: `c` calibra no centro do quadro e aplica na
hora · `m` alterna imagem/máscara · `b` liga a detecção de borda · `e` trava a
exposição · `+`/`-` ajusta a área mínima · `s` salva um PPM · `q` sai.

Roteiro de bancada com critério de passa/falha para cada funcionalidade:
[`docs/TESTES_MANUAIS.md`](docs/TESTES_MANUAIS.md). MJPG não serve (decodificar
JPEG por frame custa mais que o pipeline inteiro); use um modo YUYV.

`just --list` mostra o resto. Cada app é um binário isolado em `apps/`.

## Arquitetura

```
include/ecv/          núcleo portátil (header) — nada além de <cstdint>
  core/               views de imagem, ponto fixo Q16.16, cronometragem
  vision/             cor, limiar+LUT, morfologia, componentes conexas, ROI, linescan
  track/              Kalman de velocidade constante, desacoplado por eixo
  control/            PD (float ou ponto fixo) e mixer diferencial
  hal/                interfaces FrameSource / MotorSink
  app/                pipelines prontos (sumo_vision)
src/                  implementação do núcleo
platform/sim/         cena sintética, PPM, atuador de log — testar sem hardware
platform/linux/       V4L2 (mmap) e PWM por sysfs
platform/esp32/       componente ESP-IDF: esp32-camera e LEDC
apps/                 bench, sumo_sim, probe, calibrate, sumo_robot
```

Regra de fronteira: `include/ecv/**` e `src/**` só incluem `<cstdint>` e
`<cstddef>` — nada de STL, nada de `platform/**`. A única exceção é o relógio em
`src/core/profile.cpp`, que é plataforma por definição. `just check-portability`
verifica isso.

## Blocos reaproveitáveis

| Bloco | Para que serve fora do sumô |
|---|---|
| `vision/threshold.hpp` + `ColorLut565` | qualquer detecção por cor; a LUT vale para qualquer faixa HSV |
| `vision/blobs.hpp` | contar/medir objetos numa máscara binária |
| `vision/linescan.hpp` | seguidor de linha e detecção de borda com 4 linhas varridas |
| `vision/roi.hpp` | reduzir custo de qualquer detector que já sabe onde o alvo estava |
| `track/kalman.hpp` | suavizar e extrapolar posição de qualquer alvo 2D |
| `control/pd.hpp` + `differential.hpp` | qualquer robô de tração diferencial |
| `hal/` + `platform/sim/` | rodar e testar o robô inteiro sem montar o robô |

## Portar para outro robô

1. Implementar `FrameSource` e `MotorSink` para o hardware novo (referências em
   `platform/linux/` e `platform/esp32/`).
2. Calibrar a faixa HSV com `ecv_calibrate` numa foto do alvo real.
3. Ajustar `SumoConfig` — ou compor os blocos direto, se o comportamento não for
   perseguir um alvo.

## Números medidos

320x240 em x86_64 (`just bench`): pipeline completo em **156 µs** por frame com
ROI ativa, **645 µs** varrendo o frame inteiro. A LUT de cor é **11,9x** mais
rápida que calcular HSV por pixel, e a ROI dinâmica dá **4,1x**. Memória
estática total: **98 KB**. Detalhes e ressalvas em
[`docs/PIPELINE.md`](docs/PIPELINE.md#orçamento-de-tempo-e-memória) — esses
números são do laptop, não do alvo embarcado.
