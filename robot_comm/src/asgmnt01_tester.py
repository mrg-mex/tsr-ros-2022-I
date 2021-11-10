#!/usr/bin/env python

import rospy
from robot_comm.msg import RoboComm
import math


def main():
    try:
        rospy.init_node('cmd_talker')
        command_pub = rospy.Publisher("/cmd_robot", RoboComm, queue_size=1)
        rate = rospy.Rate(1)
        command = RoboComm()
        command.comando = 'IDLE'
        command.valor = 0.0
        while not rospy.is_shutdown():
            comando = input("Comando: ")
            if str(comando).lower() == 'exit':
                break
            else:
                command.comando = str(comando)
            valor = input("valor: ")
            if valor > 0.5:
                command.valor = 0.05
                print('Valor ({}) too high, using 0.05 instead.'.format(valor))
            else:
                command.valor = valor
            command_pub.publish(command)
            rate.sleep()
    except rospy.ROSInterruptException as e:
        print(e)

if __name__ == '__main__':
    main()