#! /usr/bin/env python

import sys
import rospy
import actionlib
from battery_simulator.msg import Battery_SimAction, Battery_SimGoal


def battery_state(charge_condition):
    goal = Battery_SimGoal()
    goal.charge_state = charge_condition
    client.send_goal(goal)


if __name__ == '__main__':
    rospy.init_node('BatterySimClient')
    client = actionlib.SimpleActionClient('battery_simulator', Battery_SimAction)
    client.wait_for_server()
    param = int(sys.argv[1])
    # print(str(sys.argv[0]))
    print('recibi param: %s', param)
    battery_state(param)
    client.wait_for_result()