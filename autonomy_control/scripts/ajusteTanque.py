#!/usr/bin/env python
from numpy import cos, sin, pi, absolute, arange

import filtro
import math

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point


def determinarCoeficientes(data):
    A = [[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]
    B = [[0.0],[0.0],[0.0]]

    A[0][0] = len(data.points)
    for i in range(0,len(data.points)):
        A[0][1] += data.points[i].x
        A[0][2] += data.points[i].x**2
        A[1][0] += data.points[i].x
        A[1][1] += data.points[i].x**2
        A[1][2] += data.points[i].x**3
        A[2][0] += data.points[i].x**2
        A[2][1] += data.points[i].x**3
        A[2][2] += data.points[i].x**4

        B[0][0] += data.points[i].z
        B[1][0] += data.points[i].x*data.points[i].z
        B[2][0] += (data.points[i].x**2)*data.points[i].z

    return (A,B)

def calcularParametroTanque(A,B):
    Xanterior = [[0.0],[0.0],[0.0]]
    Xatual = [[0.0],[0.0],[0.0]]
    buffer = [[0.0],[0.0],[0.0]]

    Xatual[0][0] = (B[0][0] - A[0][1]*Xatual[1][0] - A[0][2]*Xatual[2][0]) / A[0][0]
    Xatual[1][0] = (B[1][0] - A[1][0]*Xatual[0][0] - A[1][2]*Xatual[2][0]) / A[1][1]
    Xatual[2][0] = (B[2][0] - A[2][0]*Xatual[0][0] - A[2][1]*Xatual[1][0]) / A[2][2]

    iteracao = 0
    while variancia(Xanterior,Xatual) >= 10**-7 and iteracao < 1000:
        Xanterior[0][0] = Xatual[0][0]
        Xanterior[1][0] = Xatual[1][0]
        Xanterior[2][0] = Xatual[2][0]

        Xatual[0][0] = (B[0][0] - A[0][1]*Xatual[1][0] - A[0][2]*Xatual[2][0]) / A[0][0]
        Xatual[1][0] = (B[1][0] - A[1][0]*Xatual[0][0] - A[1][2]*Xatual[2][0]) / A[1][1]
        Xatual[2][0] = (B[2][0] - A[2][0]*Xatual[0][0] - A[2][1]*Xatual[1][0]) / A[2][2]
        iteracao +=1

    return Xatual

def variancia(Xanterior,Xatual):
    var = [0.0,0.0,0.0]
    for i in range(3):
        if Xatual[i][0] != 0:
            var[i] = math.fabs((Xatual[i][0]-Xanterior[i][0])/Xatual[i][0])
        elif Xanterior[i][0] == 0:
            var[i] = 0
        else:
            var[i] = 1
    return max(var)

def laserScanToPointCloud(data):
    dadoPointCloud = PointCloud()
    kIni,kFin = filtro.calcularIndices(data)
    dadoPointCloud.header = data.header
    dadoPointCloud.points = [None]*(kFin+1-kIni)
    j=0
    referencia = 0
    for i in range(kIni,kFin+1,1):
        if data.ranges[i] > data.range_max:
            print "Erro medicao"
        ponto_x = data.ranges[i]*sin(data.angle_min + i*data.angle_increment)
        ponto_y = 0
        ponto_z = data.ranges[i]*cos(data.angle_min + i*data.angle_increment)
        if referencia < ponto_z:
            referencia = ponto_z
        dadoPointCloud.points[j] = Point(ponto_x,ponto_y,ponto_z)
        j = j+1
    i=0
    for i in range(0,len(dadoPointCloud.points)):
        dadoPointCloud.points[i].z = -1*(dadoPointCloud.points[i].z - referencia)
    return dadoPointCloud

def calcularPosicaoTanque(data):

    iMin,iMax, estado = calcularLimitesSolda(data)

    if estado != 3:
        return -1

    dado = PointCloud()
    dado.header = data.header

    parteEsquerda = PointCloud()
    parteEsquerda.header = data.header

    for i in range (0,min(iMax,iMin)+1):
        parteEsquerda.points.append(Point(data.points[i].x,0,data.points[i].z))

    A,B = determinarCoeficientes(parteEsquerda)
    C = calcularParametroTanque(A,B)

    for i in range(0,len(parteEsquerda.points)):
        ponto_x = data.points[i].x
        ponto_z = C[0][0] + C[1][0]*data.points[i].x + C[2][0]*(data.points[i].x**2)
        dado.points.append(Point(ponto_x,0,ponto_z))

    if min(iMin,iMax)>4 and max(iMin,iMax) < len(data.points)-4:
        a,b = estimarLinha(data.points[min(iMin,iMax)-4],data.points[max(iMin,iMax)+4])
    else:
        a,b = estimarLinha(data.points[min(iMin,iMax)],data.points[max(iMin,iMax)])

    for i in range (min(iMin,iMax),max(iMin,iMax)):
        dado.points.append(Point(data.points[i].x,0,a*data.points[i].x+b))
    parteDireita = PointCloud()
    parteDireita.header = data.header

    for i in range (max(iMax,iMin),len(data.points)):
        parteDireita.points.append(Point(data.points[i].x,0,data.points[i].z))

    A,B = determinarCoeficientes(parteDireita)
    C = calcularParametroTanque(A,B)

    for i in range(max(iMax,iMin),len(data.points)):
        ponto_x = data.points[i].x
        ponto_z = C[0][0] + C[1][0]*data.points[i].x + C[2][0]*(data.points[i].x**2)
        dado.points.append(Point(ponto_x,0,ponto_z))

    return dado

def determinarSinal(scan,tanque):
    dado = PointCloud()
    dado.header = scan.header
    for i in range(0,len(scan.points)):
        ponto_x = scan.points[i].x
        ponto_z = (scan.points[i].z - tanque.points[i].z)
        dado.points.append(Point(ponto_x,0,ponto_z))
    return dado

def determinarDerivada(scan):
    derivada = PointCloud()
    derivada.header = scan.header
    derivada.points = [Point(0,0,0)]*(len(scan.points)-1)
    for i in range(0,len(derivada.points)):
        ponto_x = scan.points[i].x
        ponto_y = 0
        if math.fabs(scan.points[i+1].x-scan.points[i].x) > 0.000001:
            ponto_z = (scan.points[i+1].z-scan.points[i].z)/(scan.points[i+1].x-scan.points[i].x)
        else:
            ponto_z = 0
        derivada.points[i] = Point(ponto_x,ponto_y,ponto_z)
    return derivada

def calcularLimitesSolda(scan):
    # filtrado = filtro.aplicarFiltro(scan)
    derivada = determinarDerivada(scan)
    maximo = -100
    iMax=0

    minimo = 100
    iMin=0
    for i in range(0,len(derivada.points)):
        if maximo < derivada.points[i].z:
            maximo = derivada.points[i].z
            iMax = i
        if minimo > derivada.points[i].z:
            minimo = derivada.points[i].z
            iMin = i


    if math.fabs(derivada.points[iMax].z) > 1.2 and math.fabs(derivada.points[iMin].z) > 1.2:
        return iMin,iMax,3
    elif math.fabs(derivada.points[iMax].z) > 1.2:
        return iMin,iMax,2
    elif math.fabs(derivada.points[iMin].z) > 1.2:
        # print scan.points
        # print derivada.points
        # print derivada.points[iMin].z
        #print derivada.points[iMax].z
        return iMin,iMax,1
    else:
        return iMin,iMax,0


def estimarLinha(ponto1,ponto2):
    a = (ponto2.z - ponto1.z)/(ponto2.x - ponto1.x)
    b = ponto1.z - a*ponto1.x
    return a,b


def calcularCentroSolda(scan):
    iMin,iMax,estado = calcularLimitesSolda(scan)
    if estado != 3:
        return -1
    numero = int(round((iMin+iMax)/2))
    return scan.points[numero].x

def testecalcularCentroSolda(scan):
    iMin,iMax,estado = calcularLimitesSolda(scan)
    if estado != 3:
        return -1
    centro = PointCloud()
    centro.header = scan.header
    centro.points.append(Point(scan.points[int(round((iMin+iMax)/2))].x,0,0))
    return centro
