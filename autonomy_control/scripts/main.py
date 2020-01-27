#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud

import filtro
import ajusteTanque

pubFiltro = 0
pubPointCloud = 0
pubTanque = 0

def callback(data):

    scan = ajusteTanque.laserScanToPointCloud(data)
    tanque = ajusteTanque.calcularPosicaoTanque(scan)

    filtrado = filtro.aplicarFiltro(scan)
    derivada = ajusteTanque.determinarDerivada(filtrado)

    sinal = ajusteTanque.determinarSinal(filtrado,tanque)

#    pubPointCloud.publish(scan)
#    pubTanque.publish(tanque)
#    pubTanque.publish(derivada)
    pubPointCloud.publish(sinal)
    pubTanque.publish(ajusteTanque.testecalcularCentroSolda(sinal))
#    pubFiltro.publish(filtrado)

    return

def listener():
    global pubFiltro
    global pubTanque
    global pubPointCloud
    filtro.calcularFiltro()
    #filtro.criarGraficos()

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.

    pubFiltro = rospy.Publisher('filtro', PointCloud, queue_size=10)
    pubTanque = rospy.Publisher('tanque', PointCloud, queue_size=10)
    pubPointCloud  = rospy.Publisher('pointLRS36', PointCloud, queue_size=10)

    rospy.init_node('processamento_node', anonymous=True)

    rospy.Subscriber('/air1/lrs36', LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
