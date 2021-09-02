import rospy
from sensor_msgs.msg import Imu

class ImuSensor:
    def __init__(self, topic_name):
        """Cria um novo objeto para fazer a leitura da Imu

        Args:
            topic_name (string): Nome do tópico da Imu
        """
        self.angular_velocity = 0
        self.linear_acceleration = 0
        self.topic_name = topic_name

    def initialise(self):
        """Inicializa o subscriber para leitura da Imu
        """
        rospy.Subscriber(self.topic_name, Imu, self._callback)
        rospy.loginfo(f"Inicializando a IMU {rospy.get_time()}")

    def _callback(self, data):
        """Função de callback necessária para inicializar o subscriber

        Args:
            data (sensor_msgs.msg.Imu): Dados da leitura da Imu
        """
        self.angular_velocity = data.angular_velocity
        self.linear_acceleration = data.linear_acceleration

    def get_angular_velocity(self):
        """Método para obter o valor da variação da velocidade angular

        Returns:
            float: Variação da velocidade angular nos 3 eixos
        """
        return self.angular_velocity

    def get_linear_acceleration(self):
        """Método para obter o valor da variação da aceleração linear

        Returns:
            float: Variação da aceleração linear
        """
        return self.linear_acceleration
