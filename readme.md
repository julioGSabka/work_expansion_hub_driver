# REV Hub ROS2 Driver
this is an ROS Driver that uses [librhsp](https://github.com/REVrobotics/node-rhsplib/tree/main/packages/rhsplib/librhsp)

the library was obtained by studying the Node-JS implementation of the Rev Hub Serial Protocol (RHSP). 
turns out, there is a C library that does the actual communication. (librhsp)

all the code inside the librhsp folder belongs to REV and have their respective authors and licences.


# Calibração de PIDF

Para calibração de PIDF recomenda-se o uso das seguintes ferramentas:

Para visualizar o comportamento do motor em tempo real, utilize o ´´´rqt_plot´´. Isso permite comparar a velocidade alvo com a velocidade atual e observar oscilações ou atrasos. Lembre-se de verificar a escala do gráfico para não esconder dados.

´´´
ros2 run rqt_plot rqt_plot /m0/speed/data /m0/speed_alvo/data
´´´

Como o expansion_hub_node utiliza o ciclo de vida do ROS 2 (Lifecycle Nodes), é necessário desativar o nó antes de aplicar novos ganhos para garantir que a mudança seja processada corretamente.
Fluxo de Trabalho:

### 1. Desativar o nó de controle.
### 2. Alterar o parâmetro desejado (kP, kI, kD ou kF).
### 3. Reativar o nó para testar o novo valor.

´´´
ros2 lifecycle set /expansion_hub_node deactivate
ros2 param set /expansion_hub_node kf 20.0
ros2 lifecycle set /expansion_hub_node activate
´´´
Para testar a resposta do sistema aos comandos, utilize o pacote de teleoperação via teclado para enviar diferentes velocidades ao robô:

´´´
ros2 run teleop_twist_keyboard teleop_twist_keyboard
´´´
