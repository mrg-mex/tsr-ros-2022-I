#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2


x = 0.0
y = 0.0
theta = 0.0
l = 0.0

def sensor_callback(msg):
    global l 

    s = msg.ranges
    l = s[len(s)/2]


def controll_callback(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    oq = msg.pose.pose.orientation

    (roll, pitch, theta) = euler_from_quaternion([oq.x, oq.y, oq.z, oq.w])

def main():
    rospy.init_node('speed_controller')

    sub = rospy.Subscriber('/odom', Odometry, controll_callback)
    laser_sub = rospy.Subscriber('/scan', LaserScan, sensor_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    goal = Point()
    goal.x = 5
    goal.y = 5

    speed = Twist()
    r = rospy.Rate(4)

    while not rospy.is_shutdown():
        inc_x = goal.x - x
        inc_y = goal.y - y
        angle_to_go = atan2(inc_y, inc_x)
        rospy.loginfo('dist:{} dx: {} dy: {} alpha: {}'.format(l, inc_x, inc_y, angle_to_go))

        # angle in radians, ang speed rad/s, lin m/s
        if abs(angle_to_go - theta) > 0.1:     
            speed.linear.x = 0.0
            speed.angular.z = 0.13
        else:
            if l > 1.0:
                speed.linear.x = 0.25
            else:
                speed.linear.x = 0.0
            speed.linear.y = 0.0

        pub.publish(speed)
        r.sleep()            

if __name__ == '__main__':
    main()