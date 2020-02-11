#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Twist, Quaternion,TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64,Float32, Int16
from sensor_msgs.msg import Joy
import tf

import random
import time
import math

import filtro
import ajusteTanque
import fuzzy, fuzzy_autonomy
import time

import os

d = 0
yaw = -10
gravidade_x = 0
gravidade_y = 0
gravidade_z = 1

start_time = time.time()

lado = [2,1,2,1] # sequencia de acoes a serem seguidas em cruzamentos: 0 - reto / 1 - esquerda / 2 - direita
lado_joy = 0 # lado vindo do joystick
index_sequence = 0 # index_sequence que percorre o array 'lado'
autonomy = 4 # 1 - manual, 2 - controle de vel linear, 3 - controle de direcao, 4 - autonomo
espera_joy = 0 # 1 - esperando, 0 - pronto
erroacumulado = 0 # 

def laserCallback(data):
    global d
    d = data

    return

def joyCallback(data):
    global lado_joy
    global espera_joy
    if data.axes[0] > 0: #esquerda
        lado_joy = 1
        espera_joy = 0
    elif data.axes[0] < 0: #direita
        lado_joy = 2
        espera_joy = 0
    if data.axes[1] > 0: # ir reto
        lado_joy = 0
        espera_joy = 0

    return

def autonomyCallback(data):
    global autonomy
    autonomy = data.data

    return


def odonCallback(data):
    global yaw
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    explicit_quat = [x,y,z,w]

    (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(explicit_quat)
    # print yaw
    return

def twistCallback(data):
    global gravidade_x
    global gravidade_y
    global gravidade_z
    if data != 0:
        gravidade_x = data.twist.linear.x/(math.fabs(data.twist.linear.x) + math.fabs(data.twist.linear.y) + math.fabs(data.twist.linear.z))
        gravidade_y = data.twist.linear.y/(math.fabs(data.twist.linear.x) + math.fabs(data.twist.linear.y) + math.fabs(data.twist.linear.z))
        gravidade_z = data.twist.linear.z/(math.fabs(data.twist.linear.x) + math.fabs(data.twist.linear.y) + math.fabs(data.twist.linear.z))
    #print gravidade_x,gravidade_y,gravidade_z
    return

def listener():
    global d
    global yaw
    global gravidade_x
    global gravidade_y
    global gravidade_z
    global espera_joy
    global autonomy
    global erroacumulado
    global index_sequence

    # filtro.calcularFiltro()
    #filtro.criarGraficos()

    fuzzy.inicializaFuzzy()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    pubFiltro = rospy.Publisher('filtro', PointCloud, queue_size=10)
    pubTanque = rospy.Publisher('tanque', PointCloud, queue_size=10)
    pubPointCloud  = rospy.Publisher('pointLRS36', PointCloud, queue_size=10)
    pubVel = rospy.Publisher('air1/cmd_vel',Twist,queue_size=10) #alterado 
    #pubVel = rospy.Publisher('sim/cmd_vel',Twist,queue_size=10)
    pubErro  = rospy.Publisher('air1/erro', Float32, queue_size=10)

    rospy.init_node('processamento_node', anonymous=True)

    rospy.Subscriber('air1/lrs36', LaserScan, laserCallback)
    rospy.Subscriber('air1/odon', Odometry, odonCallback)
    rospy.Subscriber('air1/twist', TwistStamped, twistCallback)
    rospy.Subscriber('joy', Joy, joyCallback)
    rospy.Subscriber('autonomy_level', Int16, autonomyCallback)

    rate = rospy.Rate(1000)
    orie_desejada = 0
    erro_x = 0

    # os.system("rqt_plot /air1/erro &")

    while not rospy.is_shutdown():
        data = d
        if data != 0:
            erro_x = 0
            scan = ajusteTanque.laserScanToPointCloud(data)
            tanque = ajusteTanque.calcularPosicaoTanque(scan)

            #filtrado = filtro.aplicarFiltro(scan)
            #derivada = ajusteTanque.determinarDerivada(scan)
            if tanque != -1:
                # print 'a'
                sinal = ajusteTanque.determinarSinal(scan,tanque)

                #    pubPointCloud.publish(scan)
                #    pubTanque.publish(tanque)
                #
                #pubPointCloud.publish(scan)
                #pubTanque.publish(ajusteTanque.testecalcularCentroSolda(scan))
                #    pubFiltro.publish(filtrado)
                erro_x = ajusteTanque.calcularCentroSolda(sinal)
                pubErro.publish(erro_x)
                erro_orientacao =  0
                erroacumulado = erroacumulado + erro_x*erro_x
                print("ErroAcumulado:")
                print erroacumulado
                elapsed_time = time.time() - start_time
                print("Tempo:")
                print elapsed_time
                erro_por_tempo = math.sqrt(erroacumulado)/elapsed_time
                print("metrica:")
                print erro_por_tempo


                vang,vlin=fuzzy.calculaVelocidade(erro_x,erro_orientacao,3,gravidade_x,gravidade_y,gravidade_z)
                msg = Twist()
                msg.linear.x=0.3*vlin
                msg.angular.z= vang
                if espera_joy == 1:
                    msg.linear.x = 0
                    msg.angular.z = 0
                # print erro_x, erro_orientacao, msg.linear.x, msg.angular.z
                pubVel.publish(msg)


            else:
                
                # print scan.points
                iMin,iMax,estado = ajusteTanque.calcularLimitesSolda(scan)
                #ladoRotacao = random.randint(0,1)
                if autonomy == 3:
                    lado[index_sequence] = lado_joy
                ladoRotacao = lado[index_sequence]
                print("estado:")
                print estado
                print("autonomylevel:")
                print autonomy

                if gravidade_z > 0.4:
                    gPos = 0
                    angulo = -yaw

                elif gravidade_z < -0.4:
                    gPos = 1
                    angulo = yaw
                else:
                    gPos = 2
                    angulo = math.atan2(-gravidade_y,gravidade_x)
                orie_desejada = angulo

                # cruzamento em t
                if estado == 0:
                    if autonomy == 3:
                        espera_joy = 1
                    if ladoRotacao == 2:
                        orie_desejada = orie_desejada - math.pi/2 # direita
                    elif ladoRotacao == 1:
                        orie_desejada = orie_desejada + math.pi/2 # esquerda

                # reto ou curva a direita
                if estado == 1:
                    if autonomy == 3:
                        espera_joy = 1
                    if ladoRotacao == 2:
                        orie_desejada = orie_desejada - math.pi/2

                # reto ou curva a esquerda
                if estado == 2:
                    if autonomy == 3:
                        espera_joy = 1
                    if ladoRotacao == 1:
                        orie_desejada = orie_desejada + math.pi/2

                index_sequence+=1
                if index_sequence >= len(lado):
                    index_sequence = 0

                if orie_desejada > math.pi:
                    orie_desejada = orie_desejada - 2*math.pi
                elif orie_desejada < -1*math.pi:
                    orie_desejada = orie_desejada + 2*math.pi

                erro_orientacao = orie_desejada - angulo

                if erro_orientacao  > math.pi:
                    erro_orientacao  = erro_orientacao  - 2*math.pi
                elif erro_orientacao  < -1*math.pi:
                    erro_orientacao  = erro_orientacao  + 2*math.pi

                vang = 0
                vlin = 0

                # print orie_desejada,yaw,erro_orientacao

                while math.fabs(erro_orientacao) > 0.05:
                    vang,vlin=fuzzy.calculaVelocidade(erro_x,erro_orientacao,estado,gravidade_x,gravidade_y,gravidade_z)
                    pubErro.publish(erro_x)
                    # print erro_orientacao
                    # pubErro.publish(math.fabs(erro_orientacao))
                    msg = Twist()
                    msg.angular.z= 1.25*vang
                    msg.linear.x=0.1285*vlin
                    if espera_joy == 1:
                        msg.linear.x = 0
                        msg.angular.z = 0
                    # print erro_x, erro_orientacao, msg.linear.x, msg.angular.z
                    pubVel.publish(msg)
                    if gPos == 0:
                        erro_orientacao = orie_desejada -(-yaw)
                    elif gPos == 1:
                        erro_orientacao = orie_desejada - yaw
                    else:
                        erro_orientacao = orie_desejada - math.atan2(-gravidade_y,gravidade_x)

                    if erro_orientacao  > math.pi:
                        erro_orientacao  = erro_orientacao  - 2*math.pi
                    elif erro_orientacao  < -1*math.pi:
                        erro_orientacao  = erro_orientacao  + 2*math.pi
                    rate.sleep()
                scan = ajusteTanque.laserScanToPointCloud(d)
                iMax,iMin,tipoSolda = ajusteTanque.calcularLimitesSolda(scan)
                # pubPointCloud.publish(scan)
                # pubTanque.publish(ajusteTanque.testecalcularCentroSolda(scan))
                # print tipoSolda
                while  tipoSolda != 3:
                    msg = Twist()
                    msg.linear.x =0.01 * vlin
                    msg.angular.z = vang
                    if espera_joy == 1:
                        msg.linear.x = 0
                        msg.angular.z = 0
                    pubVel.publish(msg)
                    # print ajusteTanque.calcularLimitesSolda(scan)
                    scan = ajusteTanque.laserScanToPointCloud(d)
                    iMax,iMin,tipoSolda = ajusteTanque.calcularLimitesSolda(scan)

                    rate.sleep();
                print estado


        rate.sleep()


if __name__ == '__main__':
    listener()