"""Definition of the Motor class.

File
-------
src/utils/motors.py

Authors
-------
    ThundeRatz Team <comp@thunderatz.org>
"""

import rospy
from std_msgs.msg import Float64

MAX_VEL = 30  # rad/s

class Motors: # pylint: disable=too-few-public-methods
    """Class to control the motors."""

    def __init__(self, left_motor_topic, right_motor_topic):
        """Cria um novo objeto para controlar os dois motores do robô

        Args:
            left_motor_topic (string): Nome do tópico do motor esquerdo
            right_motor_topic (string): Nome do tópico do motor direito
        """
        self.left_pub = rospy.Publisher(left_motor_topic, Float64, queue_size=1)
        self.right_pub = rospy.Publisher(right_motor_topic, Float64, queue_size=1)
        rospy.loginfo(f"Inicializando motores {rospy.get_time()}")

    def drive(self, left_command, right_command):
        """Envia um comando para os dois motores

        Args:
            left_command (int): Comando a ser enviado ao motor esquerdo,
                de -100 (velocidade máxima de ré) até 100 (velocidade máxima para frente)
            right_command (int): Comando a ser enviado ao motor direito,
                de -100 (velocidade máxima de ré) até 100 (velocidade máxima para frente)

        Raises:
            ValueError: Se qualquer um dos comandos for maior em módulo do que 100
        """
        if abs(left_command) > 100:
            raise ValueError(f"Comando de velocidade maior que 100! Recebido {left_command}")

        if abs(right_command) > 100:
            raise ValueError(f"Comando de velocidade maior que 100! Recebido {right_command}")

        left_vel = MAX_VEL * int(left_command) / 100
        right_vel = MAX_VEL * int(right_command) / 100

        self.left_pub.publish(Float64(left_vel))
        self.right_pub.publish(Float64(right_vel))
