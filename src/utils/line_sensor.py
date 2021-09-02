import rospy
from std_msgs.msg import UInt32

class LineSensor:
    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura do sensor de linha

        Args:
            topic_name (string): Nome do tópico do sensor de linha
        """
        self._brightness = 1023 # Inicializa como branco
        self.topic_name = topic_name

        white_line = rospy.get_param('white_line')
        black_line = rospy.get_param('black_line')
        threshold = (white_line - black_line) / 3
        
        self.upper_bound = white_line - threshold
        self.lower_bound = black_line + threshold
        self.on_line = False

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
        self._brightness = data.data

        if not self.on_line:
            self.on_line = self.brightness < self.lower_bound
        else:
            self.on_line = self.brightness < self.upper_bound

    @property
    def brightness(self):
        return self._brightness

    def is_on_line(self):
        """ Checa se o sensor de linha está lendo uma linha ou não
        """
        return self.on_line
