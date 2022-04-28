#!/usr/bin/env python

#  realizamos los imports
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math


def calcular_yaw(measured_odom):
    # obtenemos la orientacion actual de lrobot a partir del quaternion
    # construymos un quaternion a partir de la posicion actual x,y,z,w
    quaternion = (
        measured_odom.pose.pose.orientation.x,
        measured_odom.pose.pose.orientation.y,
        measured_odom.pose.pose.orientation.z,
        measured_odom.pose.pose.orientation.w,
    )
    # se realiza la tranformacion del quaternion a euler
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # obtenemos la orientacion
    yaw = euler[2]

    return yaw


def calcular_angle(yaw, error_x, error_y, dist_euclidea):
    # calculamos el error angular entre dos vectores a partir de la orientacion actual, dist_euclidea, error_x y error_y
    # se ha aplicado la formula de la documentacion
    angle = math.acos(
        ((math.cos(yaw) * error_x) + (math.sin(yaw) * error_y)) / dist_euclidea
    )
    return angle


def exec_pid_linear(error, v_linear):

    # TODO calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # hacemos referencia a las variables globales error_acum_linear y max_integ_error_linear
    global error_acum_linear
    global max_integ_error_linear

    # sumamos el error actual en el error total acumulado
    error_acum_linear += error

    #  para limitar el efecto de la parte integral, una vez alcanzados los valores maximos y minimos, se les asigna el valor limite
    if error_acum_linear > max_integ_error_linear:
        error_acum_linear = max_integ_error_linear
    elif error_acum_linear < -max_integ_error_linear:
        error_acum_linear = -max_integ_error_linear

    # calculamos la consigna mediante la formula con la que calculabamos PID
    output = Kp_linear * error + Ki_linear * error_acum_linear + Kd_linear * (-v_linear)

    return output


def exec_pid_angular(error, v_angular):

    # TODO calcula la sortida del PID fent servir els parametres Kp, Kd, Ki i max_integ_error
    # hacemos referencia a las variables globales error_acum_angular y max_integ_error_angular
    global error_acum_angular
    global max_integ_error_angular

    # sumamos el error actual en el error total acumulado
    error_acum_angular += error

    #  para limitar el efecto de la parte integral, una vez alcanzados los valores maximos y minimos, se les asigna el valor limite
    if error_acum_angular > max_integ_error_angular:
        error_acum_angular = max_integ_error_angular
    elif error_acum_angular < -max_integ_error_angular:
        error_acum_angular = -max_integ_error_angular

    # calculamos la consigna mediante la formula con la que calculabamos PID
    output = (
        Kp_angular * error + Ki_angular * error_acum_angular + Kd_angular * (-v_angular)
    )

    return output


def callbackPose(pos_objectiu):
    # guardamos en la variable global PO la posicion deseada
    global PO
    PO = pos_objectiu


def callbackOdom(pos_actual):
    # hacemos referencia q la variable global que contiene la posicion deseada
    global PO

    # declaramos y inicializamos a cero las consignas de velocidad lineal y angular
    c_lin = 0.0
    c_ang = 0.0

    # guardamos la posicion X,Y actual (del simulador)
    PA_x = pos_actual.pose.pose.position.x
    PA_y = pos_actual.pose.pose.position.y

    # calculamos la diferencia o el error entre la posicion actual y deseada
    error_x = PO.position.x - PA_x
    error_y = PO.position.y - PA_y

    # calculamos la distancia euclidea a partir del error
    dist_euclidea = math.sqrt((error_x) ** 2 + (error_y) ** 2)

    # si la distancia es mayor a 0.05m, entonces es cuando el robot tiene que mover
    if dist_euclidea > 0.05:
        # calculamos la orientacion actual del robot
        orientacion_actual = calcular_yaw(pos_actual)
        # calculamos la diferencia del angulo a partir de la orientacion actual y la posicion deseada
        error_angle = calcular_angle(
            orientacion_actual, error_x, error_y, dist_euclidea
        )
        # decidimos si el robot tiene que girar hacia derecha o izquierda
        if (
            math.cos(orientacion_actual) * error_y
            - math.sin(orientacion_actual) * error_x
        ) < 0:
            error_angle = -error_angle

        # obtenemos la velocidad angular actual y calculamos la consigna angular para luego publicarlo
        v_angular = pos_actual.twist.twist.angular.z
        c_ang = exec_pid_angular(error_angle, v_angular)

        # si el error angular es menor a 10 grados, entonces el robot puede realizar un movimiento lineal
        if abs(error_angle) < math.radians(10):
            # obtenemos la velocidad lineal
            v_linear = pos_actual.twist.twist.linear.x
            # calculamos la consigna lineal para publicarlo
            c_lin = exec_pid_linear(dist_euclidea, v_linear)

    # declaramos un variable del tipo Twist
    msg = Twist()
    # guardamos la consigna lineal y angular en en mensaje
    msg.linear.x = c_lin
    msg.angular.z = c_ang
    # publicamos el mensaje al topic
    pub.publish(msg)


def PID():

    # declaración de las variables globales
    # declaramos el publisher
    global pub

    # declaración de las constantes para PID_linear y max_integ_error_linear
    global Kp_linear
    global Kd_linear
    global Ki_linear
    global max_integ_error_linear
    global error_acum_linear
    error_acum_linear = 0.0
    max_integ_error_linear = 0.0

    # declaración de las constantes para PID_angular y max_integ_error_angular
    global Kp_angular
    global Kd_angular
    global Ki_angular
    global max_integ_error_angular
    global error_acum_angular
    error_acum_angular = 0.0
    max_integ_error_angular = 0.0

    # declaramos e inicializamos una variable del tipo Pose()
    global PO
    PO = Pose()

    # inicializamos el nodo PID_ctrl
    rospy.init_node("PID_ctr")

    # guardamos las constantes PID_linear y el termino maximo de la integral, recibiendo valores desde el launch file
    Kp_linear = rospy.get_param("~Kp_linear", 0.0)
    Kd_linear = rospy.get_param("~Kd_linear", 0.0)
    Ki_linear = rospy.get_param("~Ki_linear", 0.0)
    max_integ_term_linear = rospy.get_param("~max_integ_term", 0.0)

    # guardamos las constantes PID_angular y el termino maximo de la integral, recibiendo valores desde el launch file
    Kp_angular = rospy.get_param("~Kp_angular", 0.0)
    Kd_angular = rospy.get_param("~Kd_angular", 0.0)
    Ki_angular = rospy.get_param("~Ki_angular", 0.0)
    max_integ_term_angular = rospy.get_param("~max_integ_term_angular", 0.0)

    # calculamos el error maximo para la integral
    if Ki_linear > 0.0:
        max_integ_error_linear = max_integ_term_linear / Ki_linear

    # calculamos el error maximo para la integral
    if Ki_angular > 0.0:
        max_integ_error_angular = max_integ_term_angular / Ki_angular

    # el nodo actual se suscribira al topic odom (posicion del robot en el simulador) que es del tipo Odometry, y al recibir ejecutará la funcion del callbackOdom
    rospy.Subscriber("~odom", Odometry, callbackOdom)
    # el nodo actual se suscribira al topic pose (posicion a la que queremos que se vaya) que es del tipo Pose, y al recibir ejecutará la funcion del callbackPose
    rospy.Subscriber("~pose", Pose, callbackPose)
    # el nodo publicara en el topic twist las velocidades lineaels y angulares, cada vez que se tiene que mover
    pub = rospy.Publisher("~twist", Twist, queue_size=10)
    # se informa que el nodo se esta ejecutando
    rospy.loginfo("PID running")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":

    PID()
