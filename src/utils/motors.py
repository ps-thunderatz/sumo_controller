import rospy
from std_msgs.msg import Float64

MAX_VEL = 30  # rad/s 

class Motors:
    def __init__(self, left_motor_topic, right_motor_topic):
        """Cria um novo objeto para controlar os dois motores de um carrinho

        Args:
            left_motor_topic (string): Nome do tópico do motor esquerdo
            right_motor_topic (string): Nome do tópico do motor direito
        """
        self.left_motor_topic = left_motor_topic
        self.right_motor_topic = right_motor_topic

    def initialise(self):
        """Inicializa o controlador dos motores
        """
        self.left_pub = rospy.Publisher(self.left_motor_topic, Float64, queue_size=1)
        self.right_pub = rospy.Publisher(self.right_motor_topic, Float64, queue_size=1)
        rospy.loginfo(f"Inicializando motores {rospy.get_time()}")

    def drive(self, left_command, right_command):
        """Envia um comando para os dois motores

        Args:
            left_command (int): Comando a ser enviado ao motor esquerdo,
                de -100 (velocidade máxima, ré) até 100 (velocidade máxima, para frente)
            right_command (int): Comando a ser enviado ao motor direito,
                de -100 (velocidade máxima, ré) até 100 (velocidade máxima, para frente)

        Raises:
            ValueError: Se qualquer um dos comandos for maior em módulo do que 100
        """
        if abs(left_command) > 100:
            raise ValueError(f"Comando de velocidade não pode ser maior que 100! Recebido {left_command}")

        if abs(right_command) > 100:
            raise ValueError(f"Comando de velocidade não pode ser maior que 100! Recebido {right_command}")

        left_vel = int(left_command)/100*MAX_VEL
        right_vel = int(right_command)/100*MAX_VEL
        
        self.left_pub.publish(Float64(left_vel))
        self.right_pub.publish(Float64(right_vel))
