#!/usr/bin/env python

import rospy
from pubsub.msg import Estatus

def publisher():
    pub = rospy.Publisher('pubtopic', Estatus, queue_size=10)
    rospy.init_node('pubnode', anonymous=False)  
    r = rospy.Rate(10)
    msg = Estatus()
    msg.estado = 'OK'
    msg.codigo = 0

    while not rospy.is_shutdown():
        msg.codigo = msg.codigo + 1
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__ == "__main__":
    publisher()