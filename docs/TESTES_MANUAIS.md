# Roteiro de testes manuais com a câmera

Nove testes de bancada, na ordem em que fazem sentido: cada um só vale se o
anterior passou. Todos rodam com `ecv_probe`, sem motores e sem robô montado.

Antes de começar: `just build` e um marcador colorido no oponente — papel ou
fita fosca, cor saturada e que não exista no resto do dohyo. Vermelho e verde
funcionam bem; evite branco (confunde com a linha do ringue), preto (confunde
com o dohyo) e qualquer coisa brilhante (reflexo especular vira buraco na
máscara).

O que aparece na tela do probe:

| Elemento | Significado |
|---|---|
| Caixa amarela | bounding box do blob detectado |
| Cruz vermelha | centroide medido (alvo em `TRACKING`) |
| Cruz roxa | posição predita pelo Kalman (alvo em `COASTING`) |
| Retângulo azul | ROI processada neste frame |
| Linha cinza vertical | centro do quadro, onde o erro é zero |
| Faixa cinza embaixo | linhas varridas pela detecção de borda (tecla `b`) |
| Terço vermelho | lado em que a linha branca foi vista |

Teclas durante a execução: `c` calibra no centro · `m` máscara · `b` borda ·
`e` exposição · `r` reset · `+`/`-` área mínima · `s` salva PPM · `q` sai.

---

## 1. A câmera entrega um formato que serve

```bash
just cam-list
```

**Passa** se aparecer pelo menos um modo marcado com `*` (YUYV, RGB565, RGB24 ou
BGR24) na resolução que você quer. **Falhou** se só houver MJPG: essa câmera não
serve sem um decodificador JPEG, que custaria mais que o pipeline inteiro.

Anote a resolução. Daqui para a frente, `--width`/`--height` se não for 320x240.

## 2. Calibrar a cor do marcador

```bash
just probe
```

Ponha o marcador ocupando o centro do quadro e aperte `c`. O rodapé responde
uma de três coisas:

- *"calibrado no centro do quadro"* → seguir.
- *"pouca cor"* → o marcador está longe ou escuro demais; aproxime.
- *"cores demais"* → o centro pegou fundo junto; enquadre só o marcador.

**Passa** quando, afastando o marcador para o canto do quadro, a caixa amarela
continua nele e a área fica estável. Anote o `--range` impresso ao sair (`q`).

## 3. A máscara está limpa

Com o marcador no quadro, aperte `m` para ver o que a limiarização enxerga.

**Passa** se o alvo aparecer como uma mancha branca sólida e o resto for quase
todo preto. Problemas típicos:

| O que se vê | Causa | O que fazer |
|---|---|---|
| Alvo esburacado | reflexo especular ou `s_min` alto | fita fosca; recalibrar com o alvo mais iluminado |
| Manchas brancas espalhadas | a cor do alvo existe na cena | trocar a cor do marcador |
| Alvo some nas bordas do quadro | vinheta da lente escurece os cantos | baixar `v_min` alguns pontos |
| Tudo branco | faixa larga demais (calibração pegou fundo) | repetir o teste 2 |

Guarde a evidência com `s` (grava `probe_NNN.ppm` no diretório atual).

## 4. Área mínima × distância

Ponha o marcador na distância máxima em que o robô ainda precisa reagir — em
sumô, a diagonal do dohyo. Leia a `area` no rodapé.

Ajuste com `+`/`-` até a área mínima ficar em torno de **metade** da área lida
nessa distância. Sobra margem para o alvo de perfil (que projeta menos área) e
ainda corta o ruído.

**Passa** se, cobrindo o marcador com a mão, o estado sai de `TRACKING` na hora,
e destapando volta.

## 5. Rastreio com alvo em movimento

Mova o marcador de um lado ao outro do quadro, devagar e depois rápido.

**Passa** se:
- o retângulo azul (ROI) acompanha o alvo e fica bem menor que o quadro;
- `detecção` no rodapé fica acima de ~95%;
- o `erro` troca de sinal ao cruzar a linha cinza central;
- `L` e `R` respondem: alvo à direita → `L` maior que `R`.

Se a ROI ficar piscando entre estreita e 320x240, o alvo está saindo dela entre
frames: aumente `roi_padding` em `SumoConfig` ou reduza a velocidade do teste.

## 6. Oclusão e predição (Kalman)

Com o alvo em movimento constante, tape a câmera por meio segundo e destape.

**Passa** se, durante a oclusão, o estado for `COASTING` e a cruz roxa continuar
andando na direção em que o alvo ia — e não congelar no último ponto. Passado o
`coast_timeout_s` (0,4 s por padrão) o estado vira `SEARCHING` e o comando passa
a girar no lugar (`L` positivo, `R` negativo).

Esse é o teste que separa "detector" de "rastreador". Se a cruz roxa não se
mexe, o Kalman não está recebendo medida — ver `docs/PIPELINE.md`, desvio 3.

## 7. Borda do ringue

Aperte `b`. A faixa cinza embaixo do preview mostra as linhas varridas.

Aponte a câmera para a linha branca do dohyo (ou uma folha branca no chão)
entrando pela esquerda, pelo centro e pela direita.

**Passa** se o terço correspondente acende vermelho, o rodapé mostra
`borda: ESQUERDA/FRENTE/DIREITA`, e os comandos ficam **os dois negativos**
(recuo) — a borda sobrepõe qualquer decisão de ataque, mesmo com o oponente à
vista.

Se acender sem linha nenhuma, o `v_min` de 190 está pegando o fundo claro do
laboratório: suba, ou baixe o `s_max`.

## 8. Exposição travada, do jeito que vai competir

Aperte `e` para travar exposição/ganho/AWB. Agora repita o teste 3 movendo algo
grande e escuro (ou claro) pelo quadro, simulando o oponente entrando em cena.

**Passa** se a máscara do alvo não mudar. Com o automático ligado (o padrão do
probe), a cena inteira muda de brilho quando algo grande entra no quadro e a
faixa calibrada deixa de valer — é o modo de falha mais comum em competição.

**Calibre com a trava ligada**, na luz do local onde vai competir. Uma faixa
calibrada com AE ligado não vale para o robô.

## 9. Latência e taxa

```bash
just probe --range <sua faixa> --log ensaio.csv --seconds 60
```

Mexa o alvo durante o minuto inteiro, incluindo oclusões.

**Passa** se `visão` ficar bem abaixo do intervalo entre frames da câmera
(33 ms a 30 fps) e o pior caso não explodir. O CSV tem uma linha por frame
(`estado, area, erro, roi_w, cmd_l, cmd_r, visao_us, …`) — dá para plotar área
contra distância, ver quanto tempo ficou em `COASTING` e anexar no relatório.

---

## Depois da bancada

Com a faixa validada, o mesmo teste roda no robô montado:

```bash
just probe --device /dev/video0 --range <faixa> --border --lock-exposure   # por SSH no RPi
just robot --dry-run                                                       # loop real, motores mudos
```

O `--dry-run` usa o atuador de log: o loop de controle roda inteiro e imprime o
PWM que mandaria, sem energizar a ponte H. Só depois disso vale ligar o motor —
e com `max_duty` baixo no `PwmSinkConfig` para o primeiro teste com roda no ar.
