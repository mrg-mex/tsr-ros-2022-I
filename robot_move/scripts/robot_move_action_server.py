#! /usr/bin/env python

import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import PointStamped, Point, Twist
from nav_msgs.msg import Odometry
from robot_move.msg import RobotMoveMsg_Action, RobotMoveMsg_Feedback, RobotMoveMsg_Result
from tf import transformations
import math

class RobotMoveActionServer():

    def __init__(self, base_lin_vel = 0.2, base_ang_vel = 0.2, min_dist = 0.05, yaw_tolerance = math.pi/90):
        # Descripcion actual del robot (Posicion y orientacion)
        self._ipose = PointStamped()
        self._iorient_rpy = Point()
        # Variables de control para la maquina de estados del robot
        self._state_code = 0
        self._code_states = ['STOP', 'TWIST', 'GO', 'GOAL']
        # Variables de control para mover y alcanzar la meta
        self._goal = None
        self._idist_to_goal = 0.0
        self._iyaw_error = 0.0
        self._base_ang_vel = base_ang_vel
        self._base_lin_vel = base_lin_vel
        self._min_dist_to_goal = min_dist
        self._yaw_tolerance = yaw_tolerance
        # Variables para publicador y subscriptor
        self._cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._odometry_sub = rospy.Subscriber('/odom', Odometry, self._on_odometry_update)
        # Implementacion de actionlib
        self._feedback = RobotMoveMsg_Feedback()
        # Declaramos el servidor en una variable de clase
        self._server = actionlib.SimpleActionServer('Robot_Move_Server', RobotMoveMsg_Action, self._execute, False)

    def _on_odometry_update(self, odom_msg):
        self._ipose.point = odom_msg.pose.pose.position
        self._ipose.header.seq = odom_msg.header.seq
        self._ipose.header.stamp = odom_msg.header.stamp
        self._ipose.header.frame_id = odom_msg.header.frame_id
        quaternion = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ]
        euler_angles = transformations.euler_from_quaternion(quaternion)
        self._iorient_rpy.x = euler_angles[0] # roll - alfa
        self._iorient_rpy.y = euler_angles[1] # pitch - beta
        self._iorient_rpy.z = euler_angles[2] # yaw - gama

    ##########################################
    ## Metodos para el movimiento del Robot ##
    ##########################################

    def _head_towards_goal(self):
        self._update_goal_vector()

        # Si yaw_error es mayor que yaw_tolerance
        if math.fabs(self._iyaw_error) > self._yaw_tolerance:
            twist_msg = Twist()
            twist_msg.angular.z = self._base_ang_vel if self._iyaw_error > 0 else -self._base_ang_vel
            self._cmd_vel_pub.publish(twist_msg)
        else: 
            # Si ya no es necesario girar
            # avanza hacia la meta
            self._state_code = 2   

    def _go_staight(self):
        self._update_goal_vector()

        # Si la distacia hacia la meta es mayor a la tolerancia
        if self._idist_to_goal > self._min_dist_to_goal:
            twist_msg = Twist()
            twist_msg.linear.x = self._base_lin_vel
            self._cmd_vel_pub.publish(twist_msg)
        else:
            # Si la distancia la meta es menor o igual a la tolerancia
            # Llegamos a la meta
            self._state_code = 3

        # Validar si existe error en la orientacion
        if math.fabs(self._iyaw_error) > self._yaw_tolerance:
            self._state_code = 1

    def _update_goal_vector(self):
        dx = self._goal.point.x - self._ipose.point.x
        dy = self._goal.point.y - self._ipose.point.y
        self._idist_to_goal = math.hypot(dx, dy)
        goal_yaw = math.atan2(dy, dx)
        self._iyaw_error = goal_yaw -self._iorient_rpy.z

    def _stop_robot(self):
        rospy.loginfo("Deteniendo al robot.")
        stop_msg = Twist()
        stop_msg.linear.x = 0
        stop_msg.linear.y = 0
        stop_msg.linear.z = 0
        stop_msg.angular.x = 0
        stop_msg.angular.y = 0
        stop_msg.angular.z = 0
        self._cmd_vel_pub.publish(stop_msg)
        rospy.sleep(1)
        self._state_code = 0

    ####################################################
    ## Metodos para el control del flujo de Actionlib ##
    ####################################################

    def _execute (self, goal):
        success = True
        rospy.loginfo('Recibi una meta!')
        # Validamos la nueva GOAL
        if not self._accept_goal(goal):
            rospy.logerr('NEW GOAL rechazada, el robot esta en estado \'%s\'.', self._code_states[self._state_code])
            self._server.set_aborted()
            return

        rospy.loginfo("NEW GOAL aceptada, dirigiendome a (%s, %s) con estado '%s'", self._goal.point.x, self._goal.point.y, self._code_states[self._state_code])
        self._state_code = 2

        while not self._state_code == 3:
            # Tomamos una accion de acuerdo al estado de 
            # la maquina de estados de nuestro robot
            if self._server.is_preempt_requested():
                success = False
                rospy.logwarn("PREEMPT signal received!")
                break

            if self._state_code == 0:
               success == False
               rospy.logwarn("Robot detenido, esperando para reanudar")
            elif self._state_code == 1:
                self._head_towards_goal()                
            elif self._state_code == 2:
                self._go_staight()
            else:
                success = False
                rospy.logerr("Assert error: No se que hago aqui, estatus (%s) '%s'", 
                    self._state_code, self._code_states[self._state_code])
                break

            self._publish_feedback()

        result = self._get_result_msg(success)
        self._stop_robot()
        if success:
            self._server.set_succeeded(result, "GOAL reached!")
        else:
            self._server.set_preempted(result)
            rospy.logwarn("GOAL PREEMPTED")

    def _accept_goal(self, goal):
        if self._state_code == 0 or self._state_code == 3:
            self._state_code == 2 # <- error '==' , solo '='
            self._goal = goal.target
            return True

        rospy.logwarn('GOAL rechazada.')    
        return False

    def _publish_feedback(self):
        self._feedback.i_state_code = self._state_code
        self._feedback.i_state_name = self._code_states[self._state_code]
        self._feedback.i_distance_error = self._idist_to_goal
        self._feedback.i_yaw_error = self._iyaw_error
        self._server.publish_feedback(self._feedback)

    def _get_result_msg(self, success):
        result = RobotMoveMsg_Result()
        result.state_code = self._state_code
        result.state_name = self._code_states[self._state_code]
        result.distance_error = self._idist_to_goal
        result.yaw_error = self._iyaw_error
        result.success = success
        if success:
            result.goal_message = "Goal completed! Posicion actual ({:.6f}, {:.6f})".format(self._ipose.point.x, self._ipose.point.y)
        else:
            result.goal_message = "Goal failed! Posicion actual ({:.6f}, {:.6f})".format(self._ipose.point.x, self._ipose.point.y)
        
        return result


def main():
    rospy.init_node('robot_move_server_node')
    robot_move_server = RobotMoveActionServer()
    robot_move_server._server.start()
    rospy.spin()

if __name__ == '__main__':
    main()