#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from pubsub.msg import Estatus

dx = 0.0
theta = 0.0

def init_monitor():
    rospy.init_node('monitor')
    sub = rospy.Subscriber('odom', Odometry, process_msg_callback)
    pub = rospy.Publisher('estatus',Estatus, queue_size=1)
    rate = rospy.Rate(2)
    pubmsg = Estatus()
    while not rospy.is_shutdown():
        if dx == 0.0 and theta == 0.0:
            pubmsg.codigo = 0
            pubmsg.estado = 'Detenido'
        elif dx != 0.0 and theta == 0.0:
            pubmsg.codigo = 100
            pubmsg.estado = 'Solo vel lineal'    
        elif dx == 0.0 and theta != 0.0:
            pubmsg.codigo = 200
            pubmsg.estado = 'Solo vel angular'    
        elif dx != 0.0 and theta != 0.0:
            pubmsg.codigo = 300
            pubmsg.estado = 'movimiento lineal y angular'
        else:
            pubmsg.codigo = 1000
            pubmsg.estado = 'Error'        
        pub.publish(pubmsg)
        rate.sleep()

    # rospy.spin()

def process_msg_callback(msg):
    dx = msg.twist.twist.linear.x
    # Debido a que nuestro robot es (2,0) no puede moverse sobre el eje Y
    # dy = msg.twist.twist.linear.y
    theta = msg.twist.twist.angular.z
    rospy.loginfo('Actualmente el robot tiene dx={:.2f} m/s, theta={:.2f} radianes'.format(dx, theta))




if __name__=='__main__':
    init_monitor()