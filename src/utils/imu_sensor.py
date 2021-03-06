"""Definição da classe ImuSensor

File
-------
src/utils/imu_sensor.py

Authors
-------
    ThundeRatz Team <comp@thunderatz.org>
"""

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class ImuSensor:
    """Classe para lidar com um sensor Imu"""

    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura da Imu

        Args:
            topic_name (string): Nome do tópico da Imu
        """
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()

        rospy.Subscriber(topic_name, Imu, self._callback)
        rospy.loginfo(f"Inicializando a Imu {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback que atualiza os valores dos atributos

        Args:
            data (sensor_msgs.msg.Imu): Dados da leitura da Imu
        """
        self.angular_velocity = data.angular_velocity
        self.linear_acceleration = data.linear_acceleration

    def get_angular_velocity(self):
        """Método para obter o valor da velocidade angular do robô

        Returns:
            geometry_msgs.msg.Vector3: Velocidade angular nos 3 eixos
        """
        return self.angular_velocity

    def get_linear_acceleration(self):
        """Método para obter o valor da aceleração linear do robô

        Returns:
            geometry_msgs.msg.Vector3: Aceleração linear nos 3 eixos
        """
        return self.linear_acceleration
