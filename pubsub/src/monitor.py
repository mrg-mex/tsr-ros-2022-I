#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from pubsub.msg import Estatus

def process_msg_callback(msg):
    dx = round(msg.twist.twist.linear.x, 2)
    # Debido a que nuestro robot es (2,0) no puede moverse sobre el eje Y
    # dy = msg.twist.twist.linear.y
    theta = round(msg.twist.twist.angular.z, 2)
    rospy.loginfo('Actualmente el robot tiene dx={:.2f} m/s, theta={:.2f} radianes'.format(dx, theta))
    if dx == 0.0 and theta == 0.0:
        pubmsg.codigo = 0
        pubmsg.estado = 'Detenido'
    elif dx != 0.0 and theta == 0.0:
        pubmsg.codigo = 100
        pubmsg.estado = 'Solo vel lineal: {} m'.format(dx)    
    elif dx == 0.0 and theta != 0.0:
        pubmsg.codigo = 200
        pubmsg.estado = 'Solo vel angular: {} rads'.format(theta)    
    elif dx != 0.0 and theta != 0.0:
        pubmsg.codigo = 300
        pubmsg.estado = 'movimiento lineal: {} m y angular: {} rads'.format(dx, theta)
    else:
        pubmsg.codigo = 1000
        pubmsg.estado = 'Error'        
    pub.publish(pubmsg)

rospy.init_node('monitor')
sub = rospy.Subscriber('odom', Odometry, process_msg_callback)
pub = rospy.Publisher('estatus',Estatus, queue_size=2)
rate = rospy.Rate(2)
pubmsg = Estatus()
rospy.spin()



# if __name__=='__main__':
#    init_monitor()