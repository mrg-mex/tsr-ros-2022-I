#!/usr/bin/env python

import rospy
from pubsub.msg import Estatus


def subscriber():
    rospy.init_node('subscriber')
    sub = rospy.Subscriber('pubtopic', Estatus, func_callback)
    rospy.spin()

def func_callback(msg):
    rospy.loginfo('La informacion recibida fue codigo:{}-{}'.format(msg.codigo, msg.estado))

if __name__ == '__main__':
    subscriber()