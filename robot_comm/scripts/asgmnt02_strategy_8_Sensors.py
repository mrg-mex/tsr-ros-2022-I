#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance():
    def __init__(self, range_max = 10, threshold_dist=1, linear_speed = 0.26, angular_speed = 0.1):
        self.robot_command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._laser_scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self._range_max = range_max
        self._threshold_dist = threshold_dist
        self.current_state = ''
        self.choice = 'STOP'
        self._base_linear_speed = linear_speed 
        self._base_angular_speed = angular_speed
        self._heading = 0


    def _scan_callback(self, scan_reads):
        # 360 degrees sensor divided into 8 sensors (45 degrees)
        sensors = {
            'FRONT' : min(min(scan_reads.ranges[0:22] + scan_reads.ranges[338:359:-1]), self._range_max),
            'FT-LF' : min(min(scan_reads.ranges[23:67]), self._range_max),
            'LEFT' : min(min(scan_reads.ranges[68:112]), self._range_max),
            'LF-BK' : min(min(scan_reads.ranges[113:157]), self._range_max),
            'BACK' : min(min(scan_reads.ranges[158:202]), self._range_max),
            'RT-BK' : min(min(scan_reads.ranges[203:247]), self._range_max),
            'RIGHT' : min(min(scan_reads.ranges[248:292]), self._range_max),
            'FT-RT' : min(min(scan_reads.ranges[293:337]), self._range_max)
        }

        self._motion(sensors)

    def _motion(self, sensors):
        command = Twist()
        lineal_vel = 0.0
        angular_vel = 0.0
        # Case 1: NO OBSTACLE
        # 0 0 0
        # # R #
        # # # #  
        if sensors['FT-LFT'] > self._threshold_dist and sensors['FRONT'] > self._threshold_distand and sensors['FT-RT'] > self._threshold_dist:
            self.current_state = '--- NO OBSTACLES ---'
            lineal_vel = self._base_linear_speed
            angular_vel = 0
            self.robot_choice = 'GO'
        # Case 2: OBSTACLE AT FRONT
        # 0 1 0
        # # R #
        # # # #  
        elif sensors['FRONT'] < self._threshold_dist and sensors['FT-LFT'] > self._threshold_dist and sensors['FT-RT'] > self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT FRONT ---'
            lineal_vel = 0
            angular_vel = self._base_angular_speed
            self.robot_choice = 'TURNING LEFT'
        # Case 3: OBSTACLE AT FRONT-RIGHT
        # 0 0 1
        # # R #
        # # # #  
        elif sensors['FRONT'] > self._threshold_dist and sensors['FT-LFT'] > self._threshold_dist and sensors['FT-RT'] < self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT FRONT-RIGHT ---'
            lineal_vel = 0
            angular_vel = self._base_angular_speed
            self.robot_choice = 'TURNING LEFT'
        # Case 4: OBSTACLE AT FRONT-LEFT
        # 1 0 0
        # # R #
        # # # #  
        elif sensors['FRONT'] > self._threshold_dist and sensors['FT-LFT'] > self._threshold_dist and sensors['FT-RT'] < self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT FRONT-LEFT ---'
            lineal_vel = 0
            angular_vel = -self._base_angular_speed
            self.robot_choice = 'TURNING RIGHT'
        

