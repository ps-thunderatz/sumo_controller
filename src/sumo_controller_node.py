#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

CONTROL_RATE = 60  # Hz

def main():
    """ Lógica principal do node de controle
    """
    rospy.init_node("sumo_controller", disable_signals=True, anonymous=True)
    rospy.loginfo(f"Node de controle iniciado {rospy.get_time()}")
    rate = rospy.Rate(CONTROL_RATE)

    # Inicialize os sensores e motores aqui

    while not rospy.is_shutdown():
        # Escreva aqui seu código para controlar o sumô
        
        rate.sleep()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        left_motor_pub = rospy.Publisher('/modelo_carrinho/robot_left_controller/command', Float64, queue_size=1)
        right_motor_pub = rospy.Publisher('/modelo_carrinho/robot_right_controller/command', Float64, queue_size=1)
        left_motor_pub.publish(Float64(0))
        right_motor_pub.publish(Float64(0))
