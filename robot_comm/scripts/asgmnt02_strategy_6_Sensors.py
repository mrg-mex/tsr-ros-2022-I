#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point

class ObstacleAvoidance():
    def __init__(self, range_max = 10, threshold_dist = 1.0, linear_speed = 0.26, angular_speed = 0.1):
        self.robot_command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._laser_scan_sub = rospy.Subscriber('/scan', LaserScan, self._scan_callback)
        self._range_max = range_max
        self._threshold_dist = threshold_dist
        self.current_state = ''
        self.robot_choice = 'STOP'
        self._current_lineal_vel = 0.0
        self._current_angular_vel = 0.0
        self._base_linear_speed = linear_speed 
        self._base_angular_speed = angular_speed
        self._goal = Point()
        self._heading = 0

    def _scan_callback(self, scan_reads):
        # 360 degrees sensor divided into 6 sensors (60 degrees)
        sensors = {
            'FRONT' : min(min(scan_reads.ranges[0:29] + scan_reads.ranges[330:359:-1]), self._range_max),
            'LEFT' : min(min(scan_reads.ranges[30:89]), self._range_max),
            'BK-LF' : min(min(scan_reads.ranges[90:149]), self._range_max),
            'BACK' : min(min(scan_reads.ranges[150:209]), self._range_max),
            'BK-RT' : min(min(scan_reads.ranges[210:269]), self._range_max),
            'RIGHT' : min(min(scan_reads.ranges[270:329]), self._range_max),
        }
        min_read = min(scan_reads.ranges, self._range_max)
        self._heading = scan_reads.index(min_read)
        self._motion_plan(sensors)

    def _motion_plan(self, sensors):
        command = Twist()
        # Condition: ROBOT STOPPED
        # # # #
        # # R #
        # # # #
        if self.robot_choice == 'STOP':
            self.current_state = '--- ROBOT STOPPED ---'
            self._current_lineal_vel = 0.0
            self._current_angular_vel = 0.0
        # Case 0: NO OBSTACLE
        # 0 0 0
        # # R #
        # 0 0 0
        elif sensors['FRONT'] > self._threshold_dist and sensors['LEFT'] > self._threshold_dist and sensors['RIGHT'] > self._threshold_dist:
            self.current_state = '--- NO OBSTACLES ---'
            self._current_lineal_vel = self._base_linear_speed
            self._current_angular_vel = 0.0
            self.robot_choice = 'GO'
        # Case 1: OBSTACLE AT FRONT
        # 0 1 0
        # # R #
        # 0 0 0
        elif sensors['FRONT'] < self._threshold_dist and sensors['LEFT'] > self._threshold_dist and sensors['RIGHT'] > self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT FRONT ---'
            self._current_lineal_vel = 0.0
            self._current_angular_vel = self._base_angular_speed
            self.robot_choice = 'TURNING LEFT'
        # Case 2: OBSTACLE AT RIGHT
        # 0 0 1
        # # R #
        # 0 0 0
        elif sensors['FRONT'] > self._threshold_dist and sensors['LEFT'] > self._threshold_dist and sensors['RIGHT'] < self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT FRONT-RIGHT ---'
            self._current_lineal_vel = 0.0
            self._current_angular_vel = self._base_angular_speed
            self.robot_choice = 'TURNING LEFT'
        # Case 3: OBSTACLE AT LEFT
        # 1 0 0
        # # R #
        # 0 0 0
        elif sensors['FRONT'] > self._threshold_dist and sensors['LEFT'] > self._threshold_dist and sensors['RIGHT'] < self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT FRONT-LEFT ---'
            self._current_lineal_vel = 0.0
            self._current_angular_vel = -self._base_angular_speed
            self.robot_choice = 'TURNING RIGHT'
        # Case 4: OBSTACLE AT FRONT AND RIGHT
        # 0 1 1
        # # R #
        # 0 0 0
        elif sensors['FRONT'] < self._threshold_dist and sensors['LEFT'] > self._threshold_dist and sensors['RIGHT'] < self._threshold_dist:
            self.current_state = '--- OBSTACLE AT FRONT AND RIGHT ---'
            self._current_linear_x = 0.0
            self._current_angular_z = self._base_angular_speed
            self.robot_choice = 'TURNING LEFT'
        # Case 5: OBSTACLE AT LEFT AND FRONT
        # 1 1 0
        # # R #
        # 0 0 0
        elif sensors['FRONT'] < self._threshold_dist and sensors['LEFT'] < self._threshold_dist and sensors['RIGHT'] > self._threshold_dist:
            self.current_state = '--- OBSTACLE AT FRONT AND LEFT ---'
            self._current_linear_x = 0.0
            self._current_angular_z = -self._base_angular_speed
            self.robot_choice = 'TURNING RIGHT'
        # Case 6: OBSTACLE AT LEFT, FRONT AND RIGHT
        # 1 1 1
        # # R #
        # 0 0 0
        elif sensors['FRONT'] < self._threshold_dist and sensors['LEFT'] < self._threshold_dist and sensors['RIGHT'] < self._threshold_dist:
            self.current_state = '--- OBSTACLE AT FRONT, LEFT AND RIGHT ---'
            self._current_linear_x = 0.0
            self._current_angular_z = self._base_angular_speed
            self.robot_choice = 'TURNING LEFT'
        # Case 7: OBSTACLE AT LEFT AND RIGHT
        # 1 0 1
        # # R #
        # 0 0 0
        elif sensors['FRONT'] > self._threshold_dist and sensors['LEFT'] < self._threshold_dist and sensors['RIGHT'] < self._threshold_dist:
            self.current_state = '--- OBSTACLE AT LEFT AND RIGHT ---'
            self._current_linear_x = self._base_linear_speed
            self._current_angular_z = 0.0
            self.robot_choice = 'GO'
        # SOLO POR LAS RISAS    
        # Case 8: OBSTACLE AT BACK-LEFT AND BACK-RIGHT
        # 0 0 0
        # # R #
        # 1 0 1
        elif sensors['BACK'] > self._threshold_dist and sensors['BK-LF'] < self._threshold_dist and sensors['BK-RT'] < self._threshold_dist:
            self.current_state = '--- OBSTACLE AT BACK-LEFT AND BACK-RIGHT ---'
            self.robot_choice = 'GO'
        # Case 9: OBSTACLE AT BACK-LEFT AND BACK
        # 0 0 0
        # # R #
        # 1 1 0
        elif sensors['BACK'] < self._threshold_dist and sensors['BK-LF'] < self._threshold_dist and sensors['BK-RT'] > self._threshold_dist:
            self.current_state = '--- OBSTACLE AT BACK-LEFT AND BACK ---'
            self.robot_choice = 'GO'
        # Case 10: OBSTACLE AT BACK-LEFT AND BACK
        # 0 0 0
        # # R #
        # 0 1 1
        elif sensors['BACK'] < self._threshold_dist and sensors['BK-LF'] > self._threshold_dist and sensors['BK-RT'] < self._threshold_dist:
            self.current_state = '--- OBSTACLE AT BACK AND BACK-RIGHT ---'
            self.robot_choice = 'GO'
        # Case 11: OBSTACLE AT BACK-LEFT
        # 0 0 0
        # # R #
        # 1 0 0
        elif sensors['BACK'] > self._threshold_dist and sensors['BK-LF'] < self._threshold_dist and sensors['BK-RT'] > self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT BACK-LEFT ---'
            self.robot_choice = 'GO'
        # Case 12: OBSTACLE AT BACK-RIGHT
        # 0 0 0
        # # R #
        # 0 0 1
        elif sensors['BACK'] > self._threshold_dist and sensors['BK-LF'] > self._threshold_dist and sensors['BK-RT'] < self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT BACK-RIGHT ---'
            self.robot_choice = 'GO'
        # Case 13: OBSTACLE AT BACK
        # 0 0 0
        # # R #
        # 0 1 0
        elif sensors['BACK'] < self._threshold_dist and sensors['BK-LF'] > self._threshold_dist and sensors['BK-RT'] > self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT BACK ---'
            self.robot_choice = 'GO'
        # Case 14: OBSTACLE AT BACK, BACK-LEFT AND BACK-LEFT
        # 0 0 0
        # # R #
        # 1 1 1
        elif sensors['BACK'] < self._threshold_dist and sensors['BK-LF'] < self._threshold_dist and sensors['BK-RT'] < self._threshold_dist:  
            self.current_state = '--- OBSTACLE AT BACK ---'
            self.robot_choice = 'GO'
        # WARNING
        else:
            self.current_state = 'UNKNOWN CASE'
            rospy.logwarn(sensors)        

        rospy.loginfo(self.current_state)    

    def set_goal(self, x, y):
        self._goal.x = x
        self._goal.y = y
        self._goal.z = 0

    def start(self):
        self.current_state = 'GO'

    def stop(self):
        self.current_state = 'STOP'

def main(x= 5, y = -5):
    control = ObstacleAvoidance()
    control.set_goal(x, y)
    control.start()

if __name__ == '__main__':
    main()