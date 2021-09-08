"""Definição da classe LineSensor

File
-------
src/utils/line_sensor.py

Authors
-------
    ThundeRatz Team <comp@thunderatz.org>
"""

import rospy
from std_msgs.msg import UInt32

class LineSensor: # pylint: disable=too-few-public-methods
    """Classe para lidar com um sensor de linha """

    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura do sensor de linha

        Args:
            topic_name (string): Nome do tópico do sensor de linha
        """
        white_line = rospy.get_param("white_line")
        black_line = rospy.get_param("black_line")
        threshold = (white_line - black_line) / 3

        self.upper_bound = white_line - threshold
        self.lower_bound = black_line + threshold
        self.on_line = False

        rospy.Subscriber(topic_name, UInt32, self._callback)
        rospy.loginfo(f"Inicializando sensor de linha {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback que atualiza os valores dos atributos

        Args:
            data (std_msgs.msg.UInt32): Dados da leitura do sensor de linha
        """
        if not self.on_line:
            self.on_line = data.data > self.upper_bound
        else:
            self.on_line = data.data > self.lower_bound

    def is_on_line(self):
        """ Checa se o sensor está lendo uma linha

        Returns:
            Bool: True se o sensor está lendo uma linha branca e False caso contrário
        """
        return self.on_line
