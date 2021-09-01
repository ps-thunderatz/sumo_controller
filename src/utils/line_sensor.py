import rospy
from std_msgs.msg import UInt32

class LineSensor:
    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura do sensor de linha

        Args:
            topic_name (string): Nome do tópico do sensor de linha
        """
        self.brightness = 1023 # Inicializa como branco
        self.topic_name = topic_name
        self.white_line = rospy.get_param('white_line')
        self.black_line = rospy.get_param('black_line')

    def initialise(self):
        """Inicializa o subscriber para leitura do sensor
        """
        rospy.Subscriber(self.topic_name, UInt32, self._callback)
        rospy.loginfo(f"Inicializando sensor de linha {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback necessária para receber informação do subscriber

        Args:
            data (std_msgs/UInt32): Dados da leitura do sensor
        """
        self.brightness = data.data

    @property
    def get_brightness(self):
        """ Método para obter o valor da última leitura de luz refletida
            A leitura varia de 0 (mais escura possível) para 1023 (mais clara possível)
        Returns:
            uint32 : retorna o valor da última leitura de luz
        """
        return self.brightness

    def is_on_line(self):
        """ Checa se o sensor de linha está lendo uma linha ou não
        """
        threshold = (self.white_line - self.black_line)

        on_line = False

        if on_line is False and self.brightness < self.black_line + (threshold * 0.3):
            on_line = True

        if on_line is True and self.brightness > self.white_line - (threshold * 0.6):
            on_line = False

        return on_line
