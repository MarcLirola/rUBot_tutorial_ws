#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Dimensiones de la ventana de turtlesim
AREA_X_MIN = 0.5
AREA_X_MAX = 10.5
AREA_Y_MIN = 0.5
AREA_Y_MAX = 10.5

robot_x = None
robot_y = None

def pose_callback(pose):
    """ Callback para actualizar la posición del robot """
    global robot_x, robot_y
    robot_x = pose.x
    robot_y = pose.y
    rospy.loginfo("Posición actual - X: %f, Y: %f", robot_x, robot_y)

def move_turtle(lin_vel, ang_vel):
    """ Mueve la tortuga hasta que choque con una pared """
    global robot_x, robot_y

    # Esperar hasta recibir la primera posición
    while robot_x is None or robot_y is None:
        rospy.sleep(0.1)

    rospy.loginfo("Posición inicial - X: %f, Y: %f", robot_x, robot_y)

    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)  # 10 Hz
    vel = Twist()

    while not rospy.is_shutdown():
        # Configurar velocidades
        vel.linear.x = lin_vel
        vel.angular.z = ang_vel

        # Verificar si la tortuga toca una pared
        if (robot_x <= AREA_X_MIN or robot_x >= AREA_X_MAX or
            robot_y <= AREA_Y_MIN or robot_y >= AREA_Y_MAX):
            rospy.loginfo("El Robot ha chocado con una pared. Deteniendo...")
            break  # Salir del bucle

        # Publicar velocidad y esperar
        pub.publish(vel)
        rate.sleep()

    # Detener la tortuga al finalizar
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)

if __name__ == '__main__':
    try:
        rospy.init_node('move_turtle', anonymous=False)  # Iniciar nodo
        rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

        v = rospy.get_param("~v")  # Obtener velocidad lineal
        w = rospy.get_param("~w")  # Obtener velocidad angular

        move_turtle(v, w)
    except rospy.ROSInterruptException:
        pass
