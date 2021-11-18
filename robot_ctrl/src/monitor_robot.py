#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetWorldProperties, GetModelState
from robot_ctrl.srv import GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse

class GazeboUtils(object):
    def __init__(self):
        pass

    def getWorldProperties(self):
        try:
            get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            wp = get_world_properties()
            if wp.success:
                return wp
            else:
                rospy.logwarn(wp.status_message)
                return None    
        except rospy.ServiceException as e:
            rospy.logerr('Al llamar /gazebo/get_world_properties: %s'%e)

    def getModelState(self, model_name, relative_entity_name='world'):
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            ms = get_model_state(model_name, relative_entity_name)
            if ms.success:
                return ms
            else:
                rospy.logwarn(ms.status_message)
                return None
        except rospy.ServiceException as e:
            rospy.logerr('Al llamar /gazebo/get_model_state: %s'%e)

    def get_model_Pose(self, model_name):
        ms = self.getModelState(model_name)
        if ms:
            return ms.pose
        else:
            return None

class DistanceMonitor():
    def __init__(self):
        self._landmarks = {}
        self._exclude = ['ground_plane', 'turtlebot3_waffle']
        self._gazebo_utils = GazeboUtils()
        self._position = Point()
        self._odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self._getClosestSrv = rospy.Service('/get_closest', GetClosest, self.get_closest_srv)
        self._getDistanceSrv = rospy.Service('/get_distance', GetDistance, self.get_distance_srv)
        self._init() # Esta linea faltaba

    def _init(self):
        wp = self._gazebo_utils.getWorldProperties()
        if wp:
            for model in wp.model_names:
                if model not in self._exclude:
                    ms = self._gazebo_utils.get_model_Pose(model)
                    position = (ms.position.x, ms.position.y)
                    self._landmarks.update({model: position})
            
        rospy.loginfo('{} landmarks encontradas.'.format(len(self._landmarks)))

    def _odom_callback(self, msg):
        self._position = msg.pose.pose.position

    def get_closest_srv(self, request):
        rospy.loginfo('GetClosestService called')
        closest_landmark = ''
        closest_distance = -1
        for model_name, (x, y) in self._landmarks.items():
            dx = x - self._position.x
            dy = y - self._position.y
            sqr_dist = (dx * dx) + (dy * dy)
            if closest_distance == -1 or sqr_dist < closest_distance:
                closest_distance = sqr_dist
                closest_landmark = model_name

        response = GetClosestResponse()
        response.name = closest_landmark
        return response

    def get_distance_srv(self, request):
        rospy.loginfo('GetDistanceService called')
        if request.nombre not in self._landmarks:
            rospy.logerr('\'{}\' no encontrada en landmarks'.format(request.nombre))
            return None
        x, y = self._landmarks[request.nombre]
        dx = x - self._position.x
        dy = y - self._position.y
        response = GetDistanceResponse()
        response.distancia = math.hypot(dx, dy)
        return response

def test_services():
    gazebo_utils = GazeboUtils()
    exclude = ['ground_plane', 'turtlebot3_waffle']
    landmarks = {}
    print('Probando GazeboUtils, get_world_properties service:')
    wp = gazebo_utils.getWorldProperties()
    if wp:
        for model in wp.model_names:
            if model not in exclude:
                ms = gazebo_utils.get_model_Pose(model)
                if ms:
                    position = (ms.position.x, ms.position.y)
                    landmarks.update({model: position})
                else:
                    print('Fallo la llamada al servicio /gazebo/get_model_state')
        return landmarks
    else:
        print('Fallo la llamada al servicio /gazebo/get_world_properties')
        return None

def main():
    rospy.init_node('distance_monitor_server')
    monitor = DistanceMonitor()
    rospy.spin()

if __name__ == '__main__':
    main()

#    landmarks = test_services()
#    if landmarks:
#        print(landmarks)
#    else:
#        print('Fallo al final')    