"""Definition of the DistanceSensor class

File
-------
src/utils/distance_sensor.py

Authors
-------
    ThundeRatz Team <comp@thunderatz.org>
"""

import rospy
from sensor_msgs.msg import Range

class DistanceSensor:
    """Class to hanlde a distance sensor"""

    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura do sensor de distância

        Args:
            topic_name (string): Nome do tópico do sensor de distância
        """
        self.range = 0
        self.min_range = 0
        self.max_range = 0

        rospy.Subscriber(topic_name, Range, self._callback)
        rospy.loginfo(f"Inicializando sensor de distância {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback que atualiza os valores dos atributos

        Args:
            data (sensor_msgs.msg.Range): Dados da leitura do sensor
        """
        self.range = data.range
        self.min_range = data.min_range
        self.max_range = data.max_range

    def get_range(self):
        """Método para obter o valor da última leitura de distância

        Returns:
            float: Distância em metros até o objeto mais próximo
        """
        return self.range

    def get_limits(self):
        """Método para obter o limite inferior e superior da leitura

        Returns:
            (float, float): Limite inferior e superior da leitura
        """
        return (self.min_range, self.max_range)
