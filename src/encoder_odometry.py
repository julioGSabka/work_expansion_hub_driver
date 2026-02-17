#!/usr/bin/env python3

import rclpy
import re
import numpy as np
import os
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class EncoderOdometry(Node):
    
    # Nó de Odometria para base Mecanum.
    # Calcula a posição e orientação baseada nos encoders dos motores.
    
    def __init__(self):
        super().__init__('encoder_odometry')

        # Parâmetros de caminho
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if config_path == '':
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.abspath(os.path.join(current_dir, 'config.hpp'))

        self.get_logger().info(f"Tentando carregar config em: {config_path}")
        self.constants = self.parse_hpp_for_defines(config_path)
        self.get_logger().info(f"Constantes carregadas: {self.constants}")

        # Inscrição nos dados de encoder
        self.encoder_sub = self.create_subscription(
            Int32MultiArray, 'encoder', self.encoder_callback, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
             
        # Estado inicial
        self.last_clock = self.get_clock().now()
        self.last_encoders = None  # Inicializado no primeiro callback para evitar saltos
        
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        self.xMultiplier: float = 1.0
        self.yMultiplier: float = 1.0
        self.headingMultiplier: float = 1.0

    def encoder_callback(self, msg: Int32MultiArray):

        now = self.get_clock().now()

        # Inicialização no primeiro frame para evitar delta espúrio
        if self.last_encoders is None:
            self.last_encoders = list(msg.data)
            self.last_clock = now
            return
        
        # Cálculo do deslocamento dos encoders (Delta Ticks)
        raw_deltas = [curr - last for curr, last in zip(msg.data, self.last_encoders)]
        delta_time = (now - self.last_clock).nanoseconds / 1e9

        if delta_time <= 0:
            return

        # --- AJUSTE DE INVERSÃO DOS MOTORES DA DIREITA ---
        # Multiplicamos por -1 os deltas dos motores 0 (FR) e 1 (BR)
        # para que o sentido "frente" seja positivo para todos.
        deltas = [
            -raw_deltas[0], # Inverte Motor 0 (FR)
            -raw_deltas[1], # Inverte Motor 1 (BR)
             raw_deltas[2], # Mantém Motor 2 (FL)
             raw_deltas[3]  # Mantém Motor 3 (BL)
        ]

        # Conversão Ticks -> Metros (Distância linear percorrida por cada roda)
        meters_per_tick = (2 * np.pi * self.constants['WHEEL_RADIUS']) / self.constants['TICK_PER_ROT']
        
        # Distâncias lineares por roda
        dFR = deltas[0] * meters_per_tick
        dBR = deltas[1] * meters_per_tick
        dFL = deltas[2] * meters_per_tick
        dBL = deltas[3] * meters_per_tick

        # Cinemática Mecanum (Velocidade no referencial do Robô)
        # Nota: Ajustar sinais conforme a orientação física dos seus motores
        lx = self.constants['WHEEL_SEPARATION_WIDTH']
        ly = self.constants['WHEEL_SEPARATION_LENGTH']
        
        # Deslocamento local (Body Frame)
        dx_b = (dFL + dFR + dBL + dBR) / 4.0
        dy_b = (-dFL + dFR + dBL - dBR) / 4.0
        dtheta = (-dFL + dFR - dBL + dBR) / (4.0 * (lx + ly))

        # Aplicação de multiplicadores de calibração
        dx_b *= self.xMultiplier
        dy_b *= self.yMultiplier
        dtheta *= self.headingMultiplier

        # Integração Espacial (Projeção no referencial Global/Odom)
        # Usamos a média do heading para melhorar a precisão em trajetórias curvas
        avg_heading = self.heading + (dtheta / 2.0)
        
        self.x += np.cos(avg_heading) * dx_b - np.sin(avg_heading) * dy_b
        self.y += np.sin(avg_heading) * dx_b + np.cos(avg_heading) * dy_b
        self.heading += dtheta

        # Publicação da TF
        self.broadcast_odometry(now)

        # Atualização de estado para o próximo ciclo
        self.last_clock = now
        self.last_encoders = list(msg.data)
    
    def broadcast_odometry(self, timestamp):
        tf = TransformStamped()
        tf.header.stamp = timestamp.to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_footprint"
        
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        
        quat = quaternion_from_euler(0, 0, self.heading)
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(tf)

    def parse_hpp_for_defines(self, hpp_file_path):
        """Extrai constantes de um arquivo C++ header."""
        values = {}
        try:
            with open(hpp_file_path, 'r') as f:
                content = f.read()
                matches = re.findall(r'#define\s+(\w+)\s+([-\w.]+)', content)
                for name, value_str in matches:
                    try:
                        # Prioriza conversão para float para manter precisão
                        values[name] = float(value_str) if '.' in value_str else int(value_str)
                    except ValueError:
                        values[name] = value_str
        except FileNotFoundError:
            self.get_logger().error(f"Arquivo de configuração não encontrado em: {hpp_file_path}")
        return values

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()