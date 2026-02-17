# REV Hub ROS2 Driver
this is an ROS Driver that uses [librhsp](https://github.com/REVrobotics/node-rhsplib/tree/main/packages/rhsplib/librhsp)

the library was obtained by studying the Node-JS implementation of the Rev Hub Serial Protocol (RHSP). 
turns out, there is a C library that does the actual communication. (librhsp)

all the code inside the librhsp folder belongs to REV and have their respective authors and licences.

## Usage

Todas as funcionalidades do pacote podem ser iniciadas através do arquivo launch principal:

```bash
ros2 launch work_expansion_hub_driver expansion.launch.py
```

## Odometry

O driver inclui um nó de Odometria Mecanum `encoder_odometry.py` que processa os dados dos encoders para estimar a posição (x,y) e a orientação (θ) do robô.

A odometria pode ser ativada ou desativada diretamente pelo arquivo de launch principal:

| Argumento | Default Value|
| :--- | :--- |
| `use_enc_odom` | `true` |

## Config File

Para garantir a consistência entre o driver C++ e o nó de odometria em Python, ambos compartilham o arquivo `src/config.hpp`. Este arquivo centraliza as constantes físicas do robô.

| Parametro | Descrição |
| :--- | :--- |
| `WHEEL_RADIUS` | Raio da roda (em metros). |
| `TICK_PER_ROT` | Quantidade de pulsos do encoder por rotação completa da roda. |
| `WHEEL_SEPARATION_WIDTH` | Distância entre as rodas direitas e esquerdas. |
| `WHEEL_SEPARATION_LENGTH` | Distância entre as rodas traseiras e dianteiras. |
| `Kp, Ki, Kd, Kf` | Ganhos do controle PIDF dos motores. |
	
### Inversão de Motores:

Dependendo da montagem física e da orientação dos motores no chassi, os encoders podem contar de forma invertida (diminuir quando o robô vai para frente).

*Atenção:* Caso a odometria apresente comportamento invertido (ex: robô gira mas o log diz que ele vai para frente), deve-se ajustar os sinais dos multiplicadores no `encoder_callback` dentro do arquivo `encoder_odometry.py` para normalizar o sentido de rotação.

## Calibração de PIDF

Para calibração de PIDF recomenda-se o uso das seguintes ferramentas:

Para visualizar o comportamento do motor em tempo real, utilize o ```rqt_plot```. Isso permite comparar a velocidade alvo com a velocidade atual e observar oscilações ou atrasos. Lembre-se de verificar a escala do gráfico para não esconder dados.

```bash
ros2 run rqt_plot rqt_plot /m0/speed/data /m0/speed_alvo/data
```

Como o expansion_hub_node utiliza o ciclo de vida do ROS 2 (Lifecycle Nodes), é necessário desativar o nó antes de aplicar novos ganhos para garantir que a mudança seja processada corretamente.

*Fluxo de Trabalho*:
1. Desativar o nó de controle.
2. Alterar o parâmetro desejado (kP, kI, kD ou kF).
3. Reativar o nó para testar o novo valor.

```bash
ros2 lifecycle set /expansion_hub_node deactivate
ros2 param set /expansion_hub_node kf 20.0
ros2 lifecycle set /expansion_hub_node activate
```
Para testar a resposta do sistema aos comandos, utilize o pacote de teleoperação via teclado para enviar diferentes velocidades ao robô:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
