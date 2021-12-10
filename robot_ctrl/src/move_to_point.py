#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose
from tf import transformations
import math

# Abbreviations:
# i - current
# orient - orientation
# rpy - roll, pitch, yaw
class RobotMotion():
    
    def __init__(self, min_dist = 0.1):
        self._ipose = Pose()
        self._iorient_rpy = Point()
        self._irobot_state = 'STOP'
        self._idist_to_goal = 0.0
        self._iyaw_error = 0.0
        self._base_ang_vel = 0.06
        self._base_lin_vel = 0.1
        self._min_dist_to_goal = min_dist
        self._yaw_tolerance = math.pi/90 # 2 degrees
        self._goal = Point()
        self._command_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._odometry_sub = rospy.Subscriber('/odom', Odometry, self._on_odometry_update)

    def set_goal(self, x, y, z):
        self._goal.x = x
        self._goal.y = y
        self._goal.z = z

    def _on_odometry_update(self, data):
        self._ipose = data.pose.pose
        quaternion = [
            self._ipose.orientation.x, 
            self._ipose.orientation.y,
            self._ipose.orientation.z, 
            self._ipose.orientation.w
        ]
        euler = transformations.euler_from_quaternion(quaternion)
        self._iorient_rpy.x = euler[0]
        self._iorient_rpy.y = euler[1]
        self._iorient_rpy.z = euler[2]

    def _head_towards_goal(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        self._iyaw_error = goal_yaw - self._iorient_rpy.z
        rospy.loginfo('Status: {}. Yaw-Error: {:.6f}, dx-to-goal: {:.6f}'.format(self._irobot_state, self._iyaw_error, dist_to_goal))
        if math.fabs(self._iyaw_error) > self._yaw_tolerance:
            angular_velocity = self._base_ang_vel if self._iyaw_error > 0 else -self._base_ang_vel
            self._send_command_to_robot('TWIST', 0, angular_velocity)
        if math.fabs(self._iyaw_error) <= self._yaw_tolerance:
            self._irobot_state = 'GO'

    def _go_straight(self):
        goal_yaw, dist_to_goal = self._compute_goal()
        self._iyaw_error = goal_yaw - self._iorient_rpy.z
        rospy.loginfo('Status: {}. Yaw-Error: {:.6f}, dx-to-goal: {:.6f}'.format(self._irobot_state, self._iyaw_error, dist_to_goal))
        if self._irobot_state not in ['GOAL', 'STOP']:
            if dist_to_goal > self._min_dist_to_goal:
                self._send_command_to_robot('GO', self._base_lin_vel, 0)
            else:
                self._irobot_state = 'GOAL'
                rospy.loginfo('Goal reached! Dist Diff {:.6f}, Yaw Err: {:.6f}'.format(dist_to_goal, self._iyaw_error))
            if self._irobot_state != 'GOAL' and math.fabs(self._iyaw_error) > self._yaw_tolerance:
                self._irobot_state = 'TWIST'

    def _compute_goal(self):
        yaw_diff = math.atan2(self._goal.y - self._ipose.position.y, self._goal.x - self._ipose.position.x)
        dist_to_goal = math.sqrt(
            pow(self._goal.y - self._ipose.position.y, 2) +
            pow(self._goal.x - self._ipose.position.x, 2) 
        )
        return yaw_diff, dist_to_goal

    def _send_command_to_robot(self, robot_state = 'STOP', vel_lineal = 0.0, vel_angular = 0.0):
        self._irobot_state = robot_state
        command = Twist()
        command.linear.x = vel_lineal
        command.angular.z = vel_angular
        self._command_pub.publish(command)

    def stop_robot(self):    
        self._send_command_to_robot()

    def set_yaw_tolerance(self, new_yaw_tolerance):
        self._yaw_tolerance = new_yaw_tolerance

    def is_goal_reached(self):
        return self._idist_to_goal <= self._min_dist_to_goal

    def get_robot_state(self):
        return self._irobot_state

    def start(self):    
        self._irobot_state = 'GO'

def main():
    robot_control = RobotMotion(0.25)
    robot_control.set_goal(5, -5, 0)
    rospy.init_node('move_to_point', anonymous=True)
    rate = rospy.Rate(1)
    robot_control._irobot_state = 'TWIST'
    if robot_control.get_robot_state() == 'STOP':
        robot_control.start()
    while (not rospy.is_shutdown()):
        if robot_control.get_robot_state() == 'TWIST':
            robot_control._head_towards_goal()
        elif robot_control.get_robot_state() == 'GO':
            robot_control._go_straight()
        elif robot_control.get_robot_state == 'GOAL':
            robot_control.stop_robot()
            rospy.loginfo('Goal reached, current position: {:.6f}, {:.6f}'
            .format(robot_control._ipose.position.x, robot_control._ipose.position.y))
            break
        elif robot_control.get_robot_state() == 'STOP':
            rospy.logwarn('Robot stoped, waiting for resume.')    
            pass
        else:
            if robot_control.get_robot_state() != 'GOAL':
                rospy.logwarn('Unknown State: {}, position: {}'.format(robot_control.get_robot_state(), robot_control._ipose.position))
            robot_control.stop_robot()
            break
        rate.sleep()

if __name__ == '__main__':
    main()