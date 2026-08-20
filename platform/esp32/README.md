# Camada ESP32 (ESP-IDF)

Componente que expõe o núcleo `ecv` mais a câmera OV2640 e o PWM por LEDC.

> Nada aqui foi compilado neste repositório — não há ESP-IDF instalado na
> máquina de desenvolvimento. O núcleo `ecv` é o mesmo binário-fonte validado
> pelos 33 testes do host; esta camada é a fiação com o IDF e precisa de um
> `idf.py build` antes de ser considerada pronta.

## Usar como componente

No `CMakeLists.txt` do projeto IDF, antes do `include(...project.cmake)`:

```cmake
set(EXTRA_COMPONENT_DIRS "<caminho>/embedded_computer_vision_models/platform/esp32")
```

Depois:

```bash
idf.py add-dependency "espressif/esp32-camera^2.0.0"
idf.py set-target esp32     # ou esp32s3
idf.py menuconfig           # habilitar PSRAM
idf.py build flash monitor
```

`example_main.cpp` é o ponto de partida do `app_main` — copiar para `main/` do
projeto, não é compilado pelo componente.

## Configuração obrigatória no menuconfig

| Opção | Valor | Motivo |
|---|---|---|
| `CONFIG_SPIRAM` | habilitado | os frame buffers da câmera vivem na PSRAM |
| `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ` | 240 | o laço de pixel é o orçamento inteiro |
| `CONFIG_COMPILER_OPTIMIZATION` | `Release (-O2)` | `-Og` chega a triplicar o tempo do laço |
| `CONFIG_FREERTOS_HZ` | 1000 | tick de 1 ms para o controle não granular demais |

## Memória

`SumoStorage<320,240>` ocupa **98.116 B** de `.bss` (máscara 76.800 + LUT 8.193
+ workspace 13.123). Num ESP32 com 320 KB de DRAM interna isso cabe, e deve
ficar na **RAM interna**: a máscara é escrita e lida a cada frame e a PSRAM é
várias vezes mais lenta. Os frame buffers da câmera é que vão para a PSRAM
(`fb_location = CAMERA_FB_IN_PSRAM`).

Sem PSRAM não dá para usar 320x240 em RGB565 (150 KB só de frame buffer duplo);
cair para QQVGA (160x120) resolve, ao custo de o alvo ficar pequeno demais para
o limiar de área — ajustar `min_area` proporcionalmente (÷4).

## Conflitos de periférico

O driver da câmera gera o XCLK do sensor com **LEDC timer 0, canal 0**. O
`LedcMotorSink` usa por padrão **timer 1, canais 2 a 5** para não colidir.
Trocar esses valores sem conferir derruba a câmera de forma difícil de depurar.

## Pinagem

`CamPins` traz o mapa do AI-Thinker ESP32-CAM. Nessa placa os GPIOs livres para
os motores são poucos (o cartão SD ocupa vários); os padrões de
`LedcMotorConfig` (12, 13, 14, 15) conflitam com o slot SD — se for usar SD,
mudar. Em ESP32-S3 com câmera dedicada há bem mais folga.

## ESP32-C3 e outros sem FPU

O núcleo faz o processamento de pixel em inteiro, mas Kalman e mistura de
motores usam `float`. No ESP32 clássico e no S3 existe FPU de precisão simples;
no C3/C6 não, e cada operação vira chamada de biblioteca. O controlador PD já é
template no tipo escalar e roda com `ecv::Fixed` (Q16.16) — ver
`tests/test_control.cpp:24`. O Kalman ainda não tem versão em ponto fixo.
