# Camada de motores — estado e o que falta para validar

**Estado em 2026-08-26: `SysfsPwmSink` nunca acionou um motor.** Todo o robô foi
exercitado com `--dry-run`, que troca a ponte H por um log. A aritmética de duty tem
testes (`tests/test_motors.cpp`), mas nenhum elétron passou por um motor.

Este documento existe porque a distância entre "compila e passa nos testes" e "gira o
motor certo no sentido certo" é onde robô quebra.

## Por que ainda não rodou

A Pi 3 desta bancada **não expõe nenhum canal de PWM**:

```console
$ ls /sys/class/pwm/
(vazio)
```

`SysfsPwmSink::open()` falha na primeira linha e devolve mensagem acionável. Faltam
duas coisas, e a segunda é a que dói.

### 1. O overlay não está habilitado

`/boot/firmware/config.txt` não tem nenhum `dtoverlay=pwm*`. Sem isso o kernel não
cria `pwmchip0`. Corrigir exige editar o arquivo com sudo e **reiniciar**.

### 2. O Pi 3 tem dois canais de PWM, e o código quer quatro

O BCM2837 tem **dois** canais de PWM por hardware (PWM0 e PWM1). `SysfsPwmSink` foi
escrito para quatro — duas entradas por motor, sem GPIO de direção. **Esse esquema não
cabe num Pi 3 sem hardware extra.**

Pior: o `config.txt` desta placa tem `dtparam=audio=on`, e a saída de áudio analógica
do Pi **usa exatamente esses dois canais de PWM**. Enquanto o áudio estiver ligado, nem
os dois canais existem para o robô.

## As três saídas

| opção | canais | mudança de código | veredito |
|---|---|---|---|
| **A — PCA9685 por I²C** | 16 | trocar o sink por um driver I²C | **recomendada** |
| **B — TB6612FNG** | 2 PWM + 4 GPIO | sink novo em modo PWM+DIR | viável |
| **C — 2 canais só** | 2 | um motor só | serve para o primeiro teste de bancada |

**A — PCA9685.** Placa de 16 canais de PWM por I²C, ~10 reais. Resolve o número de
canais, tira o PWM da CPU inteiramente (o chip gera os pulsos sozinho), e o grupo `i2c`
já está liberado para o usuário `pedro`. Custa um driver novo em `platform/linux/`, mas
é o caminho que não esbarra em limite nenhum depois.

**B — TB6612FNG.** Usa 1 PWM + 2 GPIO de direção por motor: 2 PWM + 4 GPIO no total,
que cabe no Pi 3. Exige reescrever o sink para o modelo PWM+DIR e mexer em GPIO (sysfs
ou libgpiod). Mais fiação, menos peça.

**C — dois canais.** Com `dtoverlay=pwm-2chan` e `dtparam=audio=off` você ganha PWM0 no
GPIO18 e PWM1 no GPIO19. Dá para acionar **um** motor no esquema atual de duas entradas.
Não serve para o robô, serve para provar que a cadeia sysfs→ponte→motor funciona.

### Habilitando a opção C (proposta — rode você, exige sudo e reboot)

```bash
sudo sed -i 's/^dtparam=audio=on/dtparam=audio=off/' /boot/firmware/config.txt
echo 'dtoverlay=pwm-2chan' | sudo tee -a /boot/firmware/config.txt
sudo reboot
# depois do boot, confirmar:
ls /sys/class/pwm/ && cat /sys/class/pwm/pwmchip0/npwm
```

Isso **desliga a saída de áudio analógica** da placa. HDMI continua com som.

## Procedimento de bancada — antes de pôr o robô no chão

Nesta ordem, sem pular:

1. **Motor desconectado da ponte.** Rode `ecv_sumo_robot --dry-run` e confira no log que
   os comandos fazem sentido: alvo à direita ⇒ `L` maior que `R`.
2. **Ponte alimentada, motores ainda desconectados.** Meça as saídas da ponte com
   multímetro ou osciloscópio. Confirme que só uma das duas entradas de cada motor tem
   duty por vez — é o invariante que `nenhum_comando_liga_as_duas_entradas_do_mesmo_motor`
   garante no código, mas a fiação pode desmentir.
3. **Um motor, robô suspenso, rodas no ar.** `max_duty` em **0,3**. Confirme o sentido.
   Se estiver invertido, troque os fios do motor — não o código.
4. **Dois motores, ainda no ar.** Confirme que o robô "gira" na direção certa quando
   você move o alvo.
5. **Chão, `max_duty` 0,3, espaço livre.** Só então suba para 0,8.

`max_duty` está em `apps/sumo_robot/main.cpp` (hoje 0,8). **Baixe para 0,3 antes do
primeiro teste com motor** e suba depois que o controle estiver estável.

## Parada de emergência

O laço tem um único ponto de saída e ele chama `stop()`, que escreve zero nos quatro
canais **ignorando o cache de duty**. SIGINT (Ctrl-C) e SIGTERM (`systemctl stop`) caem
nele. Se o processo morrer de forma que não passe por ali (SIGKILL, queda de energia da
Pi), **o PWM fica travado no último duty** — o hardware não sabe que o controlador
morreu.

Para um robô de competição isso não basta. Um watchdog na ponte (capacitor + enable, ou
o `nSLEEP` da DRV8833 pilotado por um GPIO que precisa ser reafirmado) é o que garante
parada em queda de software. **Não está implementado.**

## O que os testes cobrem, e o que não

`tests/test_motors.cpp` (7 testes) cobre a parte pura: saturação por `max_duty`,
negativo/zero/NaN virando parada, mapeamento de sentido, giro no lugar, o invariante de
nunca energizar as duas entradas do mesmo motor, e a mensagem de erro quando não há
`pwmchip`.

**Não cobre**, e só hardware cobre: fiação certa, sentido físico dos motores, resposta
do conjunto ponte+motor ao duty, comportamento sob corrente de partida, e se 20 kHz é
mesmo a frequência certa para os motores que você tem.
