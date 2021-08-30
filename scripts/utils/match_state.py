import rospy
from std_msgs.msg import Bool

class MatchState:
    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura do estado da partida

        Args:
            topic_name (string): Nome do tópico do estado da partida
        """
        self.state = False
        self.topic_name = topic_name

    def initialise(self):
        """Inicializa o subscriber para leitura do estado da partida
        """
        rospy.Subscriber(self.topic_name, Bool, self._callback)
        rospy.loginfo(f"Inicializando leitor do estado da partida {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback necessária para receber informação do subscriber

        Args:
            data (std_msgs/UInt32): Dados da leitura do estado da partida
        """
        self.state = data.data

    def started(self):
        """ Método para obter o estado da partida
            O estado será verdeiro se já tiver começado e falso caso contrário
        Returns:
            Bool : retorna o estado da partida
        """
        return self.state
