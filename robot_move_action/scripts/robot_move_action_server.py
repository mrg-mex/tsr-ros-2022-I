#! /usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import Twist, Point, PointStamped
from nav_msgs.msg import Odometry
from robot_move_action.msg import Robot_Move_Action, Robot_Move_Feedback, Robot_Move_Result
from tf import transformations
import math


class RobotMoveActionServer():

    def __init__(self, base_lin_vel = 0.2, base_ang_vel = 0.2, min_dist = 0.05, yaw_tolerance = math.pi/90):
        # Current robot description (Position, orientation)
        self._ipose = PointStamped()
        self._iorient_rpy = Point()
        # State machine control variables
        self._irobot_state_code = 0
        self._irobot_states = ['STOP','TWIST','GO','GOAL']
        # Goal and move control variables
        self._goal = None
        self._idist_to_goal = 0.0
        self._iyaw_error = 0.0
        self._base_ang_vel = base_ang_vel   # Valor anterior: 0.06
        self._base_lin_vel = base_lin_vel   # Valor anterior: 0.1
        self._min_dist_to_goal = min_dist
        self._yaw_tolerance = yaw_tolerance
        # Publish and subscribers variables
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._odometry_sub = rospy.Subscriber('/odom', Odometry, self._on_odometry_update)
        # Action message variables
        self._feedback = Robot_Move_Feedback()
        # Action Server initialization
        self._server = actionlib.SimpleActionServer('Robot_Move_Server', Robot_Move_Action, self._execute, False)
        print('class initialized')

    # 'odom' callback function
    def _on_odometry_update(self, odometry_msg):
        self._ipose.point = odometry_msg.pose.pose.position
        self._ipose.header.seq = odometry_msg.header.seq
        self._ipose.header.stamp = odometry_msg.header.stamp
        self._ipose.header.frame_id = odometry_msg.header.frame_id
        quaternion = [
            odometry_msg.pose.pose.orientation.x,
            odometry_msg.pose.pose.orientation.y,
            odometry_msg.pose.pose.orientation.z,
            odometry_msg.pose.pose.orientation.w
        ]
        euler_angles = transformations.euler_from_quaternion(quaternion)
        self._iorient_rpy.x = euler_angles[0] # roll - alfa
        self._iorient_rpy.y = euler_angles[1] # pitch - beta
        self._iorient_rpy.z = euler_angles[2] # yaw - gama

    def _update_goal_vectors(self):
        dx = self._goal.point.x - self._ipose.point.x
        dy = self._goal.point.y - self._ipose.point.y
        self._idist_to_goal = math.hypot(dx, dy)
        goal_yaw = math.atan2(dy, dx)
        self._iyaw_error = goal_yaw - self._iorient_rpy.z

    def _head_towards_goal(self):
        self._update_goal_vectors()

        # If yaw error magnitude is greater than yaw tolerance
        if math.fabs(self._iyaw_error) > self._yaw_tolerance:
            twist_msg = Twist()
            twist_msg.angular.z = self._base_ang_vel if self._iyaw_error > 0 else -self._base_ang_vel
            self._cmd_vel_pub.publish(twist_msg)
        # If yaw error magnitude is less than yaw tolerance
        else:
            # Twist is no longer necessary
            # then go straight
            self._irobot_state_code = 2

    def _go_staight(self):
        self._update_goal_vectors()
        # If distance to goal is greater than distance tolerance
        if self._idist_to_goal > self._min_dist_to_goal:
            twist_msg = Twist()
            twist_msg.linear.x = self._base_lin_vel
            self._cmd_vel_pub.publish(twist_msg)
        # If distance to goal is less than distance tolerance
        # Goal is reached
        else:
            self._irobot_state_code = 3
        
        # Validating yaw error
        if math.fabs(self._iyaw_error) > self._yaw_tolerance:
            self._irobot_state_code = 1


    def _stop_robot(self):
        rospy.loginfo("Stopping robot...")
        self._cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        self._irobot_state_code = 0

    # Action Server callback function
    def _execute(self, goal):
        success = True
        # Try new GOAL
        rospy.loginfo('NEW GOAL received!')
        if not self._accept_goal(goal):
            rospy.logerr("NEW GOAL aborted, turtlebot on '%s' state.", self._irobot_states[self._irobot_state_code]) #robot_state_code
            self._server.set_aborted()
            return
        # New GOAL accepted
        rospy.loginfo("NEW GOAL accepted, heading to (%s, %s) with '%s'.", self._goal.point.x, self._goal.point.y, self._irobot_states[self._irobot_state_code])

        while not self._irobot_state_code == 3:
            if self._server.is_preempt_requested():
                success = False
                rospy.logwarn("PREEMPT request received!")
                break
            if self._irobot_state_code == 0:
                rospy.logwarn('Robot stopped, waiting for resume.')
                # wait some time
                rospy.sleep(3)
            elif self._irobot_state_code == 1:
                self._head_towards_goal()
            elif self._irobot_state_code == 2:
                self._go_staight()
            else:
                success = False
                rospy.logerr("Assert error, state code (%s) '%s' ", self._irobot_state_code, self._irobot_states[self._irobot_state_code])
                break

            self._publish_feedback()

        result = self._get_result_msg(success)
        self._stop_robot()
        if success:
            self._server.set_succeeded(result)
            rospy.loginfo("Goal reached!")
        else:
            self._server.set_preempted(result)
            rospy.logwarn("Goal preempted!")


    def _accept_goal(self, goal):
        if self._irobot_state_code == 0 or self._irobot_state_code == 3:
            self._irobot_state_code = 2
            self._goal = goal.target
            return True
        
        rospy.logwarn("Goal rejected!")
        return False

    def _publish_feedback(self):
        self._feedback.i_position = PointStamped()
        self._feedback.i_position.header.frame_id = self._goal.header.frame_id
        self._feedback.i_position.header.stamp = rospy.Time.now()
        self._feedback.i_position.header.seq += 1 
        self._feedback.i_position.point = self._ipose.point
        self._feedback.i_state_code = self._irobot_state_code 
        self._feedback.i_state_name = self._irobot_states[self._irobot_state_code]
        self._feedback.i_distance_error = self._idist_to_goal
        self._feedback.i_yaw_error = self._iyaw_error
        self._server.publish_feedback(self._feedback)

    def _get_result_msg(self, success):
        result = Robot_Move_Result()
        result.i_position = PointStamped()
        result.i_position.header.frame_id = self._goal.header.frame_id
        result.i_position.header.stamp = rospy.Time.now()
        result.i_position.header.seq = self._ipose.header.seq 
        result.i_position.point = self._ipose.point
        result.i_position
        result.i_state_code = self._irobot_state_code 
        result.i_state_name = self._irobot_states[self._irobot_state_code]
        result.i_distance_error = self._idist_to_goal
        result.i_yaw_error = self._iyaw_error
        result.success = success
        if success:
            result.goal_message = 'Goal complete successfully!'
        else:
            result.goal_message = 'Goal failed!'

        return result
    
def main():
    rospy.init_node('robot_move_server_node')
    robot_move_server = RobotMoveActionServer()
    robot_move_server._server.start()
    rospy.spin()


if __name__ == '__main__':
    main()