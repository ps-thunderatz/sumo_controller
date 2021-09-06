"""Definition of the MatchState class.

File
-------
src/utils/match_state.py

Authors
-------
    ThundeRatz Team <comp@thunderatz.org>
"""

import rospy
from std_msgs.msg import Bool

class MatchState: # pylint: disable=too-few-public-methods
    """Class to handle the match state receiver."""

    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura do estado da partida

        Args:
            topic_name (string): Nome do tópico do estado da partida
        """
        self.state = False

        rospy.Subscriber(topic_name, Bool, self._callback)
        rospy.loginfo(f"Inicializando leitor do estado da partida {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback que atualiza os valores dos atributos

        Args:
            data (std_msgs.msg.Bool): Dados da leitura do estado da partida
        """
        self.state = data.data

    def started(self):
        """ Método para obter o estado da partida

        Returns:
            Bool: True se a partida já tiver começado e False caso contrário
        """
        return self.state
