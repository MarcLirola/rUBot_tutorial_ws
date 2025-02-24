#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_robot(lin_velx, ang_vel, time_duration):
    """Mueve la tortuga con una velocidad dada durante un tiempo determinado."""
    rospy.init_node('move_turtle', anonymous=False)  # Iniciar el nodo
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    vel = Twist()
    vel.linear.x = lin_velx  # Velocidad lineal
    vel.angular.z = ang_vel  # Velocidad angular

    time_begin = rospy.Time.now()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        time_elapsed = (rospy.Time.now() - time_begin).to_sec()

        if time_elapsed <= time_duration:
            pub.publish(vel)
            rospy.loginfo(f"Robot en movimiento. Tiempo transcurrido: {time_elapsed:.2f}s")
        else:
            rospy.logwarn("Deteniendo el robot")
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel)
            break  # Salir del bucle

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("Esperando parámetros...")
        v = rospy.get_param("~v", 1.0)  # Velocidad lineal, con valor por defecto
        w = rospy.get_param("~w", 0.5)  # Velocidad angular, con valor por defecto
        t = rospy.get_param("~t", 5.0)  # Duración, con valor por defecto

        rospy.loginfo(f"Parámetros recibidos: v={v}, w={w}, t={t}")
        move_robot(v, w, t)
    except rospy.ROSInterruptException:
        rospy.logwarn("Nodo interrumpido antes de completarse")
