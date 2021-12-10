#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from robot_move.msg import RobotMoveMsg_Goal, RobotMoveMsg_Action
import sys

class RobotMoveClient():

    def __init__(self):
        self._goal = RobotMoveMsg_Goal()
        self._client = actionlib.SimpleActionClient('Robot_Move_Server', RobotMoveMsg_Action)

    def _set_goal(self, goal_x, goal_y):
        self._goal.target.header.stamp = rospy.Time().now()
        self._goal.target.header.frame_id = 'map'
        self._goal.target.header.seq = 1
        self._goal.target.point.x = goal_x
        self._goal.target.point.y = goal_y
        self._goal.target.point.z = 0

    def send_goal(self, goal_x, goal_y, time_out=0.0):
        self._set_goal(goal_x, goal_y)
        rospy.loginfo("waiting for server")
        self._client.wait_for_server()
        rospy.loginfo("Enviando nuevo GOAL...")
        self._client.send_goal(self._goal,self._on_done, self._on_active, self._on_feedback)
        rospy.loginfo(self._goal)
        return self._client.wait_for_result(rospy.Duration(time_out))
    
    def _on_active(self):
        rospy.loginfo('GOAL activa')
        state = self._client.get_state()
        # rospy.loginfo("GOAL '%s', %s", self.get)

    def _on_done(self, success_code, msg):
        # elapsed = msg
        if msg.success:
            rospy.loginfo("Meta exitosa, mensaje '%s'", msg.goal_message)
        else:    
            rospy.loginfo("Meta fallida, mensaje '%s'", msg.goal_message)
        if success_code == GoalStatus.SUCCEEDED:
            rospy.loginfo('El servidor respondio OK.')
        else:
            rospy.logwarn('El servidor respondio %s', success_code)

    def _on_feedback(self, feedback_msg):
        state = self._client.get_state()
        format_msg = ("GOAL ['{}']({}), DXerr: {:.6f}, YAWerr: {:6f}"
            .format(
                feedback_msg.i_state_name,
                feedback_msg.i_state_code,
                feedback_msg.i_distance_error,
                feedback_msg.i_yaw_error,
                ))
        rospy.loginfo(format_msg)

def main():
    param_x = float(sys.argv[1])
    param_y = float(sys.argv[2])
    rospy.init_node('robot_move_client_node')
    robot_move_client = RobotMoveClient()
    goal_completed = robot_move_client.send_goal(param_x, param_y)
    if goal_completed:
        rospy.loginfo("Proceso completado con exito")
    else:
        rospy.logwarn("Proceso no completado con exito")

    sys.exit(1)


if __name__ == '__main__':
    main()