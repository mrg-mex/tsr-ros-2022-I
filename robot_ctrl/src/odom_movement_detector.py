#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point


class MovementDetector(object):
    def __init__(self):
        # Inicializamos los valores de la clase MovementDetector 
        # variable para almacenar el valor de la distancia recorrida
        self._moved_distance = Float64()
        # inicializamos el valor con cero
        self._moved_distance.data = 0.0
        # variable para almacenar la posicion actual del robot
        self._current_position = Point()
        # funcion para inicializar la posicion actual
        # debe ejecutarse antes de declarar el subscriptor para determinar el punto inicial
        self.get_init_position()
        # declaracion del publicador para el topico donde se muestra el valor de la distancia
        self.distance_moved_pub = rospy.Publisher('/moved_distance', Float64, queue_size=1)
        # declaracion del subscriptor al topico de odometria 
        self.distance_moved_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def get_init_position(self):
        # Obtiene la posición inicial del robot.
        #
        # Estructura del mensaje Odometry que describe la posicion 
        # geometry_msgs/Pose pose
        # geometry_msgs/Point position
        #     float64 x
        #     float64 y
        #     float64 z
        # geometry_msgs/Quaternion orientation
        #     float64 x
        #     float64 y
        #     float64 z
        #     float64 w
        # float64[36] covariance

        # Variable para obtener el valor inicial de la odometria
        data_odom = None
        # Obtencion del valor inicial en un bloque de que valida la existencia del topico
        # o de lo contrario espera que sea publicado 
        while data_odom is None:
            try:
                # rospy.wait_for_message, es una funcion que crea un
                # nuevo subscriptor al topico, recibe un mensaje y se desubscribe.  
                data_odom = rospy.wait_for_message('/odom', timeout=1)
            except:
                rospy.loginfo("El topico odom no se encuentra activo, esperando.")
        # Asignación de los valores iniciales obtenidos del topico de odometria
        self._current_position.x = data_odom.pose.pose.position.x
        self._current_position.y = data_odom.pose.pose.position.y
        self._current_position.z = data_odom.pose.pose.position.z

    def odom_callback(self, msg):
        # Obtiene la información de la posición del mensaje de odommetria
        new_position = msg.pose.pose.position
        # Calcula el incremento de la distancia recorrida y se almacena el la variable _mved_distance
        self._moved_distance.data += self.calculate_distance(new_position, self._current_position)
        # Actualiza la posicion actual del robot al nuevo punto de referencia 
        # para los calculos siguientes
        self.update_current_position(new_position)

    def update_current_position(self, new_position):
        # funcion para asignar la nueva posicion a la posicion actual
        self._current_position.x = new_position.x
        self._current_position.y = new_position.y
        self._current_position.z = new_position.z

    def calculate_distance(self, new_position, old_position):
        # Obtiene los puntos la posicion nueva y la anterior para calcular 
        # la distancia entre ellos
        x2 = new_position.x
        x1 = old_position.x
        y2 = new_position.y
        y1 = old_position.y
        # funcion para calcular la distancia usando math.hypot
        dist = math.hypot(x2-x1, y2-y1)
        return dist


if __name__ == '__main__':
    pass