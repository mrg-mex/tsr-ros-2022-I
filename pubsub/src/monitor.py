#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def init_monitor():
    sub = rospy.Subscriber('odom', Odometry, process_msg_callback)
    rospy.init_node('monitor')
    rospy.spin()

def process_msg_callback(msg):
    dx = msg.twist.twist.linear.x
    dy = msg.twist.twist.linear.y
    theta = msg.twist.twist.angular.z
    rospy.loginfo('Actualmente el robot tiene dx={}, dy={}, theta={}'.format(dx, dy, theta))




if __name__=='__main__':
    init_monitor()