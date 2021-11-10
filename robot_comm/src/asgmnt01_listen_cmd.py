#!/usr/bin/env python

import rospy
from robot_comm.msg import RoboComm
from geometry_msgs.msg import Twist


class CommandListener(object):
    def __init__(self):
        self._icommand = RoboComm()
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.distance_moved_sub = rospy.Subscriber('/cmd_robot', RoboComm, self._on_robot_comand)

    def _on_robot_comand(self, data):
        self._icommand = data
        robot_state = Twist()
        if not self._icommand is None:
            if self._icommand.comando.lower() == 'avanza':
                robot_state.linear.x = self._icommand.valor
                robot_state.angular.z = 0.0
                self._cmd_vel_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando: ({}, {}), enviado.'.format(self._icommand.comando, self._icommand.valor))
            elif self._icommand.comando.lower() == 'gira':
                robot_state.linear.x = 0.0
                robot_state.angular.z = self._icommand.valor
                self._cmd_vel_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando: ({}, {}), enviado.'.format(self._icommand.comando, self._icommand.valor))
            elif self._icommand.comando.lower() == 'detente':
                robot_state.linear.x = 0.0
                robot_state.angular.z = 0.0
                self._cmd_vel_pub.publish(robot_state)
                rospy.loginfo('Recibi el comando: ({}, {}), enviado'.format(self._icommand.comando, self._icommand.valor))
            else:
                print('Recibi el comando: ({}, {}), descartado.'.format(self._icommand.comando, self._icommand.valor))

    def loop(self):
        rospy.spin()    

def main():
    try:
        rospy.init_node('asgmnt01')
        cmd_listener = CommandListener()
        cmd_listener.loop()
    except rospy.ROSInterruptException as e:
        print(str(e))

if __name__ == '__main__':
    main()