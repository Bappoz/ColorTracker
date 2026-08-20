# embedded_computer_vision_models — blocos de visão em C++17 para robôs pequenos

## Stack
C++17 sem dependência externa · CMake 3.16+ · `just` como runner · testes com harness
próprio em `tests/test_harness.hpp` · alvos: host (x86_64), Raspberry Pi (V4L2), ESP32
(componente ESP-IDF).

## Comandos
```bash
just configure      # cmake -S . -B build
just build
just test           # ./build/tests/ecv_tests
just bench          # latência por estágio
just sim --frames 60   # pipeline sobre cena sintética
just fmt-check      # clang-format
just strict         # build com -Werror + testes
just check-portability
```
Antes de concluir qualquer mudança: `just fmt-check && just strict`.

## Arquitetura
- `include/ecv/` + `src/` — núcleo portátil. Só `<cstdint>`/`<cstddef>`; exceção única
  é o relógio em `src/core/profile.cpp`.
- `platform/{sim,linux,esp32}/` — aquisição e atuação. Só esta camada usa STL/syscalls.
- `apps/` — um binário por ferramenta, sem lógica de visão própria.
- Pipeline do sumô e rastreabilidade com a BPMN: `docs/PIPELINE.md`.

## Convenções (não-negociáveis)
- **Nenhuma alocação depois da inicialização.** Buffers vêm do chamador (`Buffers`,
  `BlobWorkspace`); `StaticBlobWorkspace`/`SumoStorage` alocam em `.bss`.
- Nada de `virtual` dentro do laço de pixel — só nas fronteiras `FrameSource`/`MotorSink`,
  que são chamadas uma vez por frame.
- Núcleo não inclui `platform/`. A dependência é sempre platform → core.
- Identificador em inglês; comentário e docstring em PT-BR, e só onde o *porquê* não
  é óbvio.
- Otimização entra com número medido junto (`just bench`), nunca por intuição.
- Commits: Conventional Commits.

## Armadilhas
- ROI é view por ponteiro sobre o frame: coordenada de blob é **local**, converter com
  `RoiTracker::to_global` antes de usar. Converter depois de `update()` dá offset errado.
- `SumoResult::roi` é a ROI usada *naquele* frame; o reset após perder o alvo só vale
  para o frame seguinte.
- RGB565 do OV2640 é big-endian — `decode_pixel` já trata; não "consertar".
- LEDC timer 0 / canal 0 é do XCLK da câmera no ESP32; motores usam timer 1, canais 2-5.
- `ColorLut565::build` custa 65.536 conversões HSV: chamar na inicialização, nunca no laço.

## Evitar
- OpenCV, Eigen ou qualquer lib de álgebra — inviabiliza o alvo ESP32, que é o ponto.
- Ponto flutuante no laço de pixel (o processamento de imagem é inteiro de propósito).
- Escrever número de desempenho no README/docs sem ter rodado `just bench`.
- Marcar a camada ESP32 como validada: ela nunca passou por `idf.py build` nesta máquina.

## Estado
Fonte da verdade = git (`git log --oneline -5`, branch atual).
