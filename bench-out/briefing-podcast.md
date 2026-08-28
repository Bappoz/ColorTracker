# Visão computacional embarcada num robô de sumô — o projeto inteiro

Documento de contexto para gerar um resumo em áudio. Reúne o que o código não conta:
as decisões, os números medidos em hardware, as descobertas e o que ainda não funciona.
Todos os números aqui foram medidos numa Raspberry Pi 3 real em 26 de agosto de 2026.

---

## 1. O que é o projeto

Uma biblioteca de visão computacional em **C++17 puro, sem nenhuma dependência externa**,
que faz um robô de sumô enxergar o oponente e decidir para onde ir. Roda em três alvos:
um PC de desenvolvimento, um Raspberry Pi 3 (o robô de verdade) e um ESP32.

O objetivo declarado não é terminar rápido: é **entender o mecanismo**. É projeto de
aprendizado e portfólio. Isso explica quase todas as decisões que parecem estranhas à
primeira vista.

### A regra que define tudo

**Nada de OpenCV, Eigen, ou qualquer biblioteca de álgebra.**

Não é purismo. É que o alvo ESP32 tem 520 KB de RAM e nenhum sistema operacional
completo. OpenCV sozinho não cabe. No momento em que você aceita a dependência, perde o
alvo mais restrito — e o alvo mais restrito é justamente o que força o código a ser
honesto sobre custo.

O núcleo inteiro só pode incluir dois cabeçalhos: `<cstdint>` e `<cstddef>`. Há uma
única exceção, o relógio usado para medir tempo. Existe um teste automatizado que falha
se alguém incluir qualquer outra coisa.

### As outras restrições autoimpostas

**Nenhuma alocação de memória depois da inicialização.** Todos os buffers vêm de quem
chama a função. Num robô, uma alocação no meio do laço é uma pausa imprevisível — e
imprevisibilidade num sistema que decide 30 vezes por segundo é o que faz o robô perder
o oponente. O consumo total ficou em **96 KB de memória estática**.

**Nada de ponto flutuante no laço de pixel.** Todo o processamento de imagem é aritmética
inteira. O Cortex-A53 do Pi tem unidade de ponto flutuante, mas o ESP32 tem uma bem mais
fraca, e a diferença aparece quando você multiplica por 76 800 pixels.

**Nada de `virtual` dentro do laço de pixel.** Chamada virtual impede o compilador de
fazer inline e quebra a previsão de desvio. Só existem duas fronteiras virtuais no
projeto — a fonte de imagem e o destino dos motores — e ambas são chamadas uma vez por
quadro, não uma vez por pixel.

**Otimização só entra com número medido junto.** Não se otimiza por intuição.

---

## 2. A arquitetura, em três camadas

**O núcleo** é portátil e não sabe onde está rodando. Recebe uma imagem, devolve uma
decisão. Não conhece câmera, motor, sistema operacional.

**A camada de plataforma** faz a ponte com o hardware: captura de câmera por V4L2 no
Linux, geração de PWM para os motores, e as versões equivalentes no ESP32 e no simulador.
Só essa camada tem permissão para usar a biblioteca padrão e chamadas de sistema.

**As aplicações** são um binário por ferramenta: o robô, o simulador, o medidor de
latência, o calibrador de cor, a sonda de câmera ao vivo, e a bateria de testes.

A dependência é sempre plataforma → núcleo, nunca o contrário. O núcleo não sabe que a
camada de plataforma existe.

---

## 3. O pipeline, estágio a estágio

Cada quadro que chega passa por seis estágios. A porcentagem é o custo médio medido em
320×240 no Pi 3.

**Threshold — 29% do tempo.** Decide, pixel a pixel, se aquele ponto é da cor do alvo.
Converte a cor para o espaço HSV (matiz, saturação, valor) e compara com uma faixa. O
truque de desempenho: em vez de converter cada pixel na hora, existe uma tabela pré
calculada com todas as 65 536 cores possíveis do formato RGB565. Consultar a tabela é uma
leitura de memória em vez de uma conversão. A tabela custa 2,5 milissegundos para
construir e é construída uma vez, na inicialização — nunca dentro do laço.

**Morphology — 60% do tempo, o estágio mais caro.** A máscara que sai do threshold é
suja: pixels isolados de ruído acesos, buracos dentro do alvo. A morfologia faz duas
passadas — erosão seguida de dilatação — que apagam ruído pequeno e fecham buracos.
É o maior alvo de otimização do projeto e ninguém mexeu nele ainda.

**Blobs — 10% do tempo.** Agrupa os pixels acesos em regiões conectadas e mede cada uma:
área, centro, caixa delimitadora. A maior região acima de um limiar de área vira o alvo.

**Border — 1,3% do tempo.** Procura a borda branca do ringue, para o robô não se
autoeliminar saindo da área.

**Track e Control — juntos, 2 microssegundos.** O rastreador guarda onde o alvo estava e
prevê onde vai estar. O controlador transforma o erro de posição em comando de motor,
usando um controlador proporcional-derivativo. É a lógica "inteligente" do robô, e ela é
essencialmente gratuita perto do custo de processar a imagem.

### A ideia mais importante do pipeline: a ROI

ROI significa região de interesse. Quando o robô já achou o alvo, não faz sentido varrer
a imagem inteira no quadro seguinte — o alvo não teleportou. O rastreador recorta uma
janela em volta da última posição conhecida e processa **só aquilo**.

Numa captura real com câmera, a diferença medida foi:

- varrendo a imagem inteira: **8 050 microssegundos**
- com a região de interesse travada: **1 453 microssegundos**

**Uma diferença de 5,5 vezes.** É a otimização mais importante do projeto e não envolve
instrução nenhuma de baixo nível: é só não processar o que não precisa.

---

## 4. Como se mede isso

O robô foi exercitado por uma bateria de testes que reproduz sete fases de uma luta:
busca, aquisição do alvo, perseguição, oclusão (o alvo some atrás de algo), reaquisição,
detecção de borda, e recalibração de cor em tempo de execução.

Durante 345 segundos, um amostrador registrou a cada segundo: temperatura do processador,
frequência de clock, tensão do núcleo, estado de limitação térmica, carga do sistema e
memória livre.

Os testes rodaram do mais leve ao mais brutal: ocioso, fases completas com varredura de
resolução, pior caso sustentado em 640×480, visão disputando processador com os outros
três núcleos saturados, câmera real, robô completo, e resfriamento.

### Um detalhe de método que quase estragou tudo

A alimentação. O Raspberry Pi tem um registrador que diz se está sofrendo subtensão. Três
fontes diferentes falharam nesse teste — a USB do notebook, uma de 2,1 amperes e um
carregador portátil de 3 amperes. Sob subtensão o clock cai sozinho e **todos os números
mentem**. Só com uma fonte de 5 volts e 5,1 amperes de cabo fixo o registrador zerou.

A lição: o gargalo não era amperagem nominal, era **queda de tensão no cabo**. Cabo micro
USB fino derruba tensão o suficiente para acionar a proteção mesmo com fonte generosa.

---

## 5. Os resultados na placa

Raspberry Pi 3 Model B, processador de quatro núcleos a 1,2 GHz, Debian 13, 64 bits.

### O veredito

Com a webcam ligada e sem desenhar nada na tela, o robô fechou **30,1 quadros por
segundo sustentados** durante 45 segundos, processando 1 351 quadros. Trinta é
exatamente o teto da câmera. **Não é a placa que limita, é a câmera.**

O orçamento de tempo de um quadro a 30 quadros por segundo é de 33 333 microssegundos.
A visão consome:

- 24% desse orçamento no modo mais caro, varrendo a imagem toda
- 4% quando a região de interesse está travada no alvo

Sobram entre 76% e 96% do quadro para controle de motores, telemetria e o que mais o robô
precisar.

### A distribuição é bimodal, não uma curva

Esse é um resultado que muda como se pensa o sistema. Não existe "caso médio". O pipeline
está sempre num de dois modos: ou rastreando com região de interesse, custando cerca de
1 500 microssegundos, ou varrendo tudo, custando cerca de 8 000. A média entre os dois não
descreve nenhum quadro real.

Para dimensionar um prazo de resposta, o número que importa é o da varredura total, nunca
a média.

---

## 6. As três descobertas

### Descoberta 1: o formato de pixel custa 20% do tempo

A câmera entrega os pixels num formato chamado YUYV. Testando os quatro formatos que o
núcleo aceita, na mesma cena e mesma resolução:

| formato | estágio threshold | média total |
|---|---|---|
| RGB565 | 454 microssegundos | 1 567 |
| RGB24 | 726 | 1 830 |
| BGR24 | 752 | 1 853 |
| YUYV (o atual) | **1 184** | **2 297** |

O threshold em YUYV custa **161% a mais** que em RGB565, porque converter do espaço de
cor YUV para RGB e depois para HSV é matemática por pixel, enquanto RGB565 é só desempacotar
bits e consultar a tabela.

E a prova de que a atribuição está certa: o estágio de morfologia ficou em **934
microssegundos nos quatro formatos**. Ele opera na máscara binária já pronta, então não
depende de como ela foi feita. Os estágios de baixo são desacoplados.

Consequência prática: o relatório anterior media RGB565 sintético, um formato que **esta
câmera não oferece**. O número honesto de orçamento é o do YUYV.

### Descoberta 2: saturar os outros núcleos quase não muda nada

Teste: rodar o pipeline enquanto três dos quatro núcleos estão 100% ocupados com trabalho
inútil.

- média: sobe **1,8%**
- percentil 95: sobe 0,8%
- percentil 99: sobe 20,7%
- máximo: sobe 12,7%

O pipeline é de uma thread só e a placa tem quatro núcleos: o escalonador entrega um
núcleo inteiro para a visão e o resto do sistema se vira com os outros três.

Isso significa que dá para rodar controle de motores, telemetria, gravação de log e rede
em paralelo **sem tirar o prazo da visão**. Só a cauda da distribuição sente.

### Descoberta 3: a escala com resolução é sublinear

Multiplicando a área da imagem por 16 (de 160×120 para 640×480), o tempo multiplica por
**8,4**, não por 16. O custo por pixel cai de 32,3 nanossegundos para 17,0.

Motivo: existe um custo fixo por quadro que se dilui em imagens grandes, e a região de
interesse trabalha mais a favor quando a imagem é maior.

Consequência: **640×480 cabe no orçamento.** Mesmo no pior caso sustentado, o pior quadro
usou 47% do tempo disponível. Resolução não é o limite desta placa.

---

## 7. Térmica e energia

Durante os 345 segundos de bateria completa, com pico de carga em todos os núcleos:

- temperatura partiu de 53,7 °C e chegou a **73,1 °C** no pico
- o limiar em que o Pi começa a reduzir clock é 80 °C — sobraram **6,9 °C de folga**
- o registrador de limitação reportou **zero em todas as 345 amostras**: nenhuma
  subtensão, nenhuma redução de clock, nem uma vez
- a frequência ficou cravada em 1200 MHz durante todo trabalho real, caindo para 600 a
  800 MHz só nas fases ociosas

Sobre energia: não houve medição com wattímetro, então qualquer número absoluto seria
inventado. O que foi medido é o ciclo de trabalho — o processo ocupou 99,9% de um núcleo,
ou 25% da máquina inteira — e a tensão, cravada em 1,325 volts.

A leitura que importa não é o valor absoluto: **manter o alvo travado corta o custo por
5,5 vezes**. Um robô que perde o oponente gasta 5,5 vezes mais energia de processador por
quadro. Campo de visão maior e detecção estável são economia de energia, não só
desempenho.

---

## 8. A pergunta da câmera

Vale trocar por uma câmera de 120 graus com saída RGB? A resposta tem três partes, em
ordem de impacto medido.

**Primeiro: taxa de quadros.** A câmera atual trava em 30 quadros por segundo no formato
não comprimido. Como a visão usa só 24% do orçamento, a placa aguentaria 60 quadros por
segundo folgada — usaria 48%. Isso **dobra a taxa do laço de controle**: a reação do robô
cai de 33 para 17 milissegundos. É o maior ganho disponível.

**Segundo: campo de visão.** Cento e vinte graus fazem o robô achar o oponente antes, e
portanto passar menos tempo na fase cara de varredura. Ganho duplo: desempenho e energia.

Mas cobra pedágio, e o pedágio é real. Espalhar o mesmo sensor em 120 graus dá **menos
pixels no alvo** à mesma distância. Na captura real, o alvo já estava oscilando em 164
pixels de área contra um limiar de 150 — com 120 graus, aquele marcador não seria
detectado. E há distorção de barril, que torna a relação entre posição na imagem e ângulo
real **não-linear**: o ganho do controlador passa a variar conforme o alvo esteja no
centro ou na borda. Corrigir exigiria ponto flutuante no laço de pixel, que é justamente
o que o projeto proíbe.

Para sumô isso costuma ser aceitável, porque basta saber para que lado virar. Precisão
angular na borda não decide luta.

**Terceiro: formato.** Vinte por cento de ganho medido. Real, mas o menor dos três. E
cuidado com a palavra "RGB": em câmeras USB, RGB quase nunca significa RGB565, o formato
barato. Significa RGB24, que é o ganho de 20%, não de 32%.

**A recomendação foi não comprar nada ainda.** A placa tem 76% do quadro sobrando. O
próximo gargalo do robô quase certamente não é a câmera.

---

## 9. Os bugs encontrados no caminho

### O robô saía com erro toda vez que era desligado

O serviço que roda o robô no boot ficava marcado como "falhou" toda vez que era parado.

Causa raiz: quando um sinal de encerramento chega, chamadas de sistema que estavam
esperando são interrompidas e retornam um código específico chamado EINTR, que significa
"fui interrompido, tente de novo". O código tratava corretamente esse caso numa das
chamadas, mas não na outra — a que espera a câmera ter um quadro pronto. Então o sinal de
desligamento virava "a captura falhou" e o programa saía com código de erro.

Correção: repetir a chamada quando o retorno for EINTR, como já era feito na outra. Depois
disso o serviço encerra limpo.

### A bateria de testes travou no meio

O script de testes travou logo antes das fases de câmera. Causa: uma instrução que espera
processos filhos terminarem foi usada sem argumento, e sem argumento ela espera **todos**
os filhos — incluindo o amostrador de telemetria, que é um laço infinito de propósito.

Correção: esperar só os processos específicos.

O detalhe honesto é que esse bug estava num script escrito para *medir* o sistema, não no
sistema. Ferramenta de medição também tem bug, e um bug na medição contamina tudo que ela
produz.

---

## 10. A camada de motores: o risco que continua aberto

Este é o ponto mais importante do projeto, e é uma ausência.

**Nenhum motor girou. Nunca.** Todo o robô foi exercitado em modo seco, que troca a ponte
H por um registro em log. A aritmética tem testes; nenhum elétron passou por um motor.

E ao investigar por quê, apareceu que não era falta de teste.

**A placa não expõe nenhum canal de PWM.** PWM, modulação por largura de pulso, é como se
controla velocidade de motor: em vez de variar tensão, você liga e desliga muito rápido e
varia a proporção de tempo ligado. O sistema de arquivos que deveria expor esses canais
está vazio, porque falta uma linha de configuração no arquivo de inicialização da placa.

**Mas o problema de fundo é pior.** O código foi escrito esperando **quatro** canais de
PWM — duas entradas por motor, sem pino separado de direção. O processador do Pi 3 tem
**dois**. E esses dois estão ocupados pela saída de áudio analógica da placa, que usa
exatamente o mesmo hardware.

Ou seja: hoje não existe nem um canal livre, e mesmo liberando, faltariam dois.

### As três saídas

**PCA9685 por I²C** é a recomendada: uma placa de dezesseis canais de PWM que se comunica
por dois fios e gera os pulsos sozinha, tirando o trabalho do processador. Custa um driver
novo em código.

**TB6612FNG** usa um canal de PWM mais dois pinos digitais de direção por motor — cabe nos
dois canais do Pi. Exige reescrever a camada para outro modelo de acionamento.

**Dois canais só** serve para provar que a cadeia software-ponte-motor funciona, com um
motor. Não serve para o robô.

### O que foi corrigido no código enquanto isso

Sete testes novos cobrem a aritmética, incluindo um invariante importante: **nenhum
comando pode energizar as duas entradas do mesmo motor ao mesmo tempo**, porque nessas
pontes isso significa freio, não avanço.

Correções de segurança: valores inválidos como "não é um número" agora viram parada em vez
de largura de pulso arbitrária; a inversão de sentido zera uma entrada antes de levantar a
outra; e a função de parada de emergência ignora qualquer otimização e escreve zero de
verdade.

Também entrou um cache que evita reescrever a mesma largura de pulso a cada quadro.
Honestamente: isso é meio por cento do quadro, não é ganho de desempenho relevante. O
ganho é não martelar o driver do kernel à toa.

### A lacuna que continua

Se o processo morrer sem passar pela rotina de parada — uma queda de energia, uma
finalização forçada — **o PWM fica travado na última largura de pulso**. O hardware não
sabe que o controlador morreu, e o robô sai andando sozinho.

Para competição isso não basta. Falta um watchdog na ponte: um circuito que precisa ser
reafirmado periodicamente e corta a potência se o software parar de reafirmar. **Não está
implementado.**

---

## 11. Como o robô é operado hoje

A placa roda sem monitor nem teclado. O robô sobe automaticamente quando ela liga, como
serviço do sistema, com reinício automático em caso de falha e com o modo seco como padrão
— processa a câmera e decide, mas não aciona motor até você mandar.

Os binários vão compilados de forma estática: não há nada para instalar na placa, e ela
não compila nada. Um comando no notebook compila para a arquitetura ARM, envia pela rede e
reinicia o serviço. O ciclo é o mesmo de gravar um microcontrolador.

Um manual de operação fica gravado **na própria placa** e viaja junto com o binário a cada
atualização, então nunca fica desatualizado em relação ao código que está rodando.

Isso foi testado de ponta a ponta, inclusive trocando de rede: o endereço da placa mudou,
a ferramenta de descoberta achou por nome de rede local, e o robô **continuou rodando sem
reiniciar durante a troca** — ele não depende de rede, só quem fala com ele depende.

---

## 12. O que este projeto ensina

**Medir antes de otimizar não é slogan.** A intuição dizia que o gargalo seria a
detecção de cor. Era a morfologia, com 60% do tempo. E o segundo maior custo não era
algoritmo nenhum: era o **formato em que a câmera entrega os pixels**, algo que nem
aparece no código do pipeline.

**A restrição é a professora.** Proibir bibliotecas externas, alocação dinâmica e ponto
flutuante parece masoquismo. O resultado é um sistema de 96 KB que roda a 30 quadros por
segundo numa placa de 2016 usando um quarto do processador, e cujo custo é inteiramente
explicável estágio a estágio.

**A diferença entre "compila" e "funciona" é hardware.** A camada de motores passava em
todos os testes e não conseguiria acionar um motor nem se quisesse, porque o hardware
necessário não estava configurado — e, mais fundo, porque a placa não tem canais
suficientes. Isso só aparece quando você tenta ligar de verdade.

**A ferramenta de medição também tem bug.** O script que mede o sistema travou por um erro
sutil de espera de processos. Se ninguém tivesse olhado o relógio, os dados teriam saído
incompletos e ninguém saberia.
