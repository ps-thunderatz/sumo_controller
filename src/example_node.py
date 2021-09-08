#!/usr/bin/env python3

# pylint: disable-all

"""Simples node de exemplo para realização de testes

File
-------
sumo_controller/src/example_node.py

Authors
-------
    ThundeRatz Team <comp@thunderatz.org>
"""

import rospy
from std_msgs.msg import Float64
from utils.motors import Motors
from utils.match_state import MatchState

CONTROL_RATE = 60  # Hz

def main():
    """ Lógica principal do node de controle
    """
    rospy.init_node("sumo_controller", disable_signals=True, anonymous=True)
    rospy.loginfo(f"Node de controle iniciado {rospy.get_time()}")
    rate = rospy.Rate(CONTROL_RATE)

    # Corrija o nome dos tópicos!!!
    motors = Motors("robot_left_controller/command", "robot_right_controller/command")
    match_state = MatchState("/sumo/start")

    while not rospy.is_shutdown():
        if match_state.started():
            motors.drive(100, 100)
        else:
            motors.drive(0, 0)

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        # Corrija o nome dos tópicos!!!
        left_motor_pub = rospy.Publisher("robot_left_controller/command", Float64, queue_size=1)
        right_motor_pub = rospy.Publisher("robot_right_controller/command", Float64, queue_size=1)

        left_motor_pub.publish(Float64(0))
        right_motor_pub.publish(Float64(0))
