#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def callback(msg):
    print('[{}] -> [0:{:.6f}], [90:{:.6f}], [180:{:.6f}], [270:{:.6f}]'
    .format(len(msg.ranges), msg.ranges[0], msg.ranges[90], msg.ranges[180], msg.ranges[270]))

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()