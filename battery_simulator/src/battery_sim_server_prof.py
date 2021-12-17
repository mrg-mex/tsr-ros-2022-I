#! /usr/bin/env python

import rospy
import time
from multiprocessing import Process
import actionlib
from battery_simulator.msg import battery_simAction, battery_simResult, battery_simFeedback


def batterySim():
    battery_level = 100
    result = battery_simResult()
    feedback = battery_simFeedback()
    while not rospy.is_shutdown():
        if rospy.has_param('/tb3/BatteryStatus'):
            time.sleep(1)
            param = rospy.get_param('/tb3/BatteryStatus')
            feedback.battery_percentage = battery_level/100
            if param == 1:
                if battery_level == 100:
                    result.battery_status = 'Full'
                    #server.publish_feedback(feedback)
                    server.set_succeeded(result)
                    break
                else:
                    battery_level += 1
                    #result.battery_status = 'Charging'
                    #server.publish_feedback(feedback)
                    rospy.loginfo('Cargando... actualmente, %s', battery_level)
                    time.sleep(4)
            elif param == 0:
                #if battery_level == 0:
                #    result.battery_status = 'Discharged'
                #    server.set_aborted(result, 'Battery discharged')
                battery_level -= 1
                # server.publish_feedback(feedback)
                rospy.logwarn('Descargando... actualmente, %s', battery_level)
                time.sleep(2)

def goal_clbck(goal):
    rate = rospy.Rate(2)
    process = Process(target = batterySim)
    process.start()
    time.sleep(1)
    if goal.charge_state == 0:
        rospy.set_param('/tb3/BatteryStatus', goal.charge_state)
    elif goal.charge_state == 1:
        rospy.set_param('/tb3/BatteryStatus', goal.charge_state)


if __name__ == '__main__':
    rospy.init_node('BatterySimServer')
    server = actionlib.SimpleActionServer('battery_simulator', battery_simAction, goal_clbck, False)
    server.start()
    rospy.spin()