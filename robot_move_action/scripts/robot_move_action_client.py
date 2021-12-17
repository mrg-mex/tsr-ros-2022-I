#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robot_move_action.msg import Robot_Move_Goal, Robot_Move_Action
import sys

class RobotMoveClient():

    def __init__(self):
        self._goal = Robot_Move_Goal()
        self._client = actionlib.SimpleActionClient('Robot_Move_Server', Robot_Move_Action)
                
    def _set_goal(self, goal_x, goal_y):

        self._goal.target.header.stamp = rospy.Time.now()
        self._goal.target.header.frame_id = 'map'
        self._goal.target.header.seq = 1
        self._goal.target.point.x = goal_x
        self._goal.target.point.y = goal_y
        self._goal.target.point.z = 0

    def send_goal(self, goal_x, goal_y, time_out = 0.0):
        self._set_goal(goal_x, goal_y)
        rospy.loginfo('Waiting for server.')
        self._client.wait_for_server()
        rospy.loginfo('Sending goal...')
        self._client.send_goal(self._goal, self._on_done, self._on_active, self._on_feedback)
        rospy.loginfo(self._goal)
        return self._client.wait_for_result(rospy.Duration(time_out))
        # return self._client.wait_for_result()
        

    def _on_active(self):
        rospy.loginfo('Goal active')
        state = self._client.get_state()
        rospy.loginfo("Goal '%s', %s", self.get_state_name(state), self._client.get_goal_status_text())

    def _on_done(self, success_code, msg):
        elapsed_secs = msg.i_position.header.stamp.secs - self._goal.target.header.stamp.secs
        formatted_msg = ("{}, [{}]'{}': iposition({:.6f}, {:.6f}), Derr({:.6f}), Yerr({:.6f})"
            .format(
                self.get_state_name(success_code),
                msg.i_state_code, 
                msg.i_state_name, 
                msg.i_position.point.x, 
                msg.i_position.point.y,
                msg.i_distance_error,
                msg.i_yaw_error))
        formatted_status_msg = ('{} Finished with state \'{}\', elapsed time {} secs'
            .format(msg.goal_message, self.get_state_name(success_code), elapsed_secs))
            
        if success_code == GoalStatus.SUCCEEDED:        
            rospy.loginfo(formatted_status_msg)
            rospy.loginfo(formatted_msg)
        else:    
            rospy.logerr(formatted_status_msg)
            rospy.logerr(formatted_msg)

    def _on_feedback(self, feedback_msg):
        state = self._client.get_state()
        formatted_msg = ("Goal['{}'] - '{}'[{}]: iposition({:.6f}, {:.6f}), Derr({:.6f}), Yerr({:.6f})"
            .format(
                self.get_state_name(state),
                feedback_msg.i_state_name, 
                feedback_msg.i_state_code, 
                feedback_msg.i_position.point.x, 
                feedback_msg.i_position.point.y,
                feedback_msg.i_distance_error,
                feedback_msg.i_yaw_error))
        rospy.loginfo(formatted_msg)

    def cancel_goal(self):
        self._client.cancel_goal()

    def get_client_status(self):
        return self._client.get_state(), self._client.get_goal_status_text()    

    def get_state_name(self, state):
        state_name = ''
        if state == GoalStatus.PENDING: state_name = 'PENDING'
        if state == GoalStatus.ACTIVE : state_name = 'ACTIVE'
        if state == GoalStatus.PREEMPTED : state_name = 'PREEMPTED'
        if state == GoalStatus.SUCCEEDED : state_name = 'SUCCEEDED'
        if state == GoalStatus.ABORTED : state_name = 'ABORTED'
        if state == GoalStatus.REJECTED : state_name = 'REJECTED'
        if state == GoalStatus.PREEMPTING : state_name = 'PREEMPTING'
        if state == GoalStatus.RECALLING : state_name = 'RECALLING'
        if state == GoalStatus.RECALLED : state_name = 'RECALLED'
        if state == GoalStatus.LOST : state_name = 'LOST'
        return state_name

def main():
    param_x = float(sys.argv[1])
    param_y = float(sys.argv[2])
    rospy.init_node('robot_move_client_node')
    robot_move_client = RobotMoveClient()
    
    goal_completed = robot_move_client.send_goal(param_x, param_y, 60.0)
    if not goal_completed:
        rospy.logerr("Timed out waiting for goal complete exceeded!")
        robot_move_client.cancel_goal()
        rospy.logerr("Goal cancelled.")
        sys.exit(1) 

if __name__ == '__main__':
    main()