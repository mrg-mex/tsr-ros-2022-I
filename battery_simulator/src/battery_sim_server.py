#! /usr/bin/env python

import rospy
import time
from multiprocessing import Process
import actionlib
from battery_simulator.msg import Battery_SimAction, Battery_SimGoal, Battery_SimResult, Battery_SimFeedback
from rospy.core import rospywarn

def batterySim():
    battery_level = 100
    result = Battery_SimResult()
    feedback = Battery_SimFeedback()
    while not rospy.is_shutdown():
        if rospy.has_param('/battery_sim/BatteryStatus'):
            feedback.battery_percentage = battery_level / 100.0
            time.sleep(1)
            param = rospy.get_param('/battery_sim/BatteryStatus')
            if param == 1:
                if battery_level == 100:
                    result.battery_status = 'Cargada'
                    server.set_succeeded(result)
                    break
                else:
                    battery_level += 1
                    # result.battery_status = 'Cargando'
                    rospy.loginfo('Cargando... actualmente %s', battery_level)
                    time.sleep(4)
            elif param == 0:
                battery_level -= 1
                rospy.logwarn('Descargando... actualmente %s', battery_level)
                time.sleep(2)
            server.publish_feedback(feedback)
                

def goal_clbck(goal):
    rate = rospy.Rate(2)
    proceso = Process(target = batterySim)
    proceso.start()
    time.sleep(1)
    if goal.charge_state == 0:
        rospy.set_param('/battery_sim/BatteryStatus', goal.charge_state)
    elif goal.charge_state == 1:
        rospy.set_param('/battery_sim/BatteryStatus', goal.charge_state)

if __name__ == '__main__':
    rospy.init_node('BatterySimServer')
    server = actionlib.SimpleActionServer('battery_simulator', Battery_SimAction, goal_clbck, False)
    server.start()
    rospy.spin()