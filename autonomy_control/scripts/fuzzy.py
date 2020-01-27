#!/usr/bin/env python

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np

sang = 0
slin = 0

def inicializaFuzzy():
    global sang
    global slin

    TipoSolda = ctrl.Antecedent(np.arange(0,3,0.005),'TipoSolda')
    ErroX = ctrl.Antecedent(np.arange(-1,1,0.005),'ErroX')
    ErroYaw = ctrl.Antecedent(np.arange(-3.5,3.5,0.005),'ErroYaw')
    GravidadeY = ctrl.Antecedent(np.arange(-1,1,0.005),'GravidadeY')
    GravidadeX = ctrl.Antecedent(np.arange(-1,1,0.005),'GravidadeX')
    GravidadeZ = ctrl.Antecedent(np.arange(-1,1,0.005),'GravidadeZ')

    Vangular = ctrl.Consequent(np.arange(-1,1,0.005),'Vangular')
    Vlinear = ctrl.Consequent(np.arange(0,2,0.005),'Vlinear')

    TipoSolda['TS0'] = fuzz.trapmf(TipoSolda.universe,[0,0,0.5,0.5])
    TipoSolda['TS12'] = fuzz.trapmf(TipoSolda.universe,[0.5,0.5,2.5,2.5])
    TipoSolda['TS3'] = fuzz.trapmf(TipoSolda.universe,[2.5,2.5,3,3])

    ErroX['EX0'] = fuzz.trapmf(ErroX.universe,[-1, -1, -0.600, -0.100])
    ErroX['EX1'] = fuzz.trimf(ErroX.universe,[-0.600, -0.100, 0.000])
    ErroX['EX2'] = fuzz.trimf(ErroX.universe,[-0.100, 0.000, 0.100])
    ErroX['EX3'] = fuzz.trimf(ErroX.universe,[0, 0.100, 0.600])
    ErroX['EX4'] = fuzz.trapmf(ErroX.universe,[0.100, 0.600, 1, 1])

    ErroYaw['EY0'] = fuzz.trapmf(ErroYaw.universe,[-3.5, -3.5, -1.5, -1])
    ErroYaw['EY1'] = fuzz.trimf(ErroYaw.universe,[-1.5, -1, -0.1])
    ErroYaw['EY2'] = fuzz.trimf(ErroYaw.universe,[-1, -0.1, 0])
    ErroYaw['EY3'] = fuzz.trimf(ErroYaw.universe,[-0.1, 0.000, 0.1])
    ErroYaw['EY4'] = fuzz.trimf(ErroYaw.universe,[0, 0.1, 1])
    ErroYaw['EY5'] = fuzz.trimf(ErroYaw.universe,[0.1, 1, 1.5])
    ErroYaw['EY6'] = fuzz.trapmf(ErroYaw.universe,[1, 1.5, 3.5, 3.5])

    GravidadeX['GX0'] = fuzz.trimf(GravidadeX.universe,[-1,-1,0])
    GravidadeX['GX1'] = fuzz.trimf(GravidadeX.universe,[-1,0,1])
    GravidadeX['GX2'] = fuzz.trimf(GravidadeX.universe,[0,1,1])

    GravidadeY['GY0'] = fuzz.trimf(GravidadeY.universe,[-1,-1,0])
    GravidadeY['GY1'] = fuzz.trimf(GravidadeY.universe,[-1,0,1])
    GravidadeY['GY2'] = fuzz.trimf(GravidadeY.universe,[0,1,1])

    GravidadeZ['GZ0'] = fuzz.trimf(GravidadeZ.universe,[-1,-1,0])
    GravidadeZ['GZ1'] = fuzz.trimf(GravidadeZ.universe,[-1,0,1])
    GravidadeZ['GZ2'] = fuzz.trimf(GravidadeZ.universe,[0,1,1])

    Vangular['VA0'] = fuzz.trapmf(Vangular.universe,[-1, -1, -0.600, -0.300])
    Vangular['VA1'] = fuzz.trimf(Vangular.universe,[-0.600, -0.300, 0.000])
    Vangular['VA2'] = fuzz.trimf(Vangular.universe,[-0.300, 0.000, 0.300])
    Vangular['VA3'] = fuzz.trimf(Vangular.universe,[0.000, 0.300, 0.600])
    Vangular['VA4'] = fuzz.trapmf(Vangular.universe,[0.300, 0.600, 1, 1])

    Vlinear['VL0'] = fuzz.trimf(Vlinear.universe,[0, 0, 0.1])
    Vlinear['VL1'] = fuzz.trimf(Vlinear.universe,[0, 0.1, 0.5])
    #Vlinear['VL2'] = fuzz.trapmf(Vlinear.universe,[0.1, 0.5, 1,1])
    Vlinear['VL2'] = fuzz.trimf(Vlinear.universe,[0.1, 0.5, 1])
    Vlinear['VL3'] = fuzz.trimf(Vlinear.universe,[0.5, 1, 1.5])
    Vlinear['VL4'] = fuzz.trapmf(Vlinear.universe,[1, 1.5, 2, 2])

    # TipoSolda.view()
    # ErroX.view()
    # ErroYaw.view()
    # GravidadeY.view()
    # GravidadeX.view()
    # GravidadeZ.view()
    # Vangular.view()
    # Vlinear.view()


    ruleA1 = ctrl.Rule(ErroX['EX0'] & ErroYaw['EY3'],Vangular['VA4'])
    ruleA2 = ctrl.Rule(ErroX['EX1'] & ErroYaw['EY3'],Vangular['VA3'])
    ruleA3 = ctrl.Rule(ErroX['EX2'] & ErroYaw['EY3'],Vangular['VA2'])
    ruleA4 = ctrl.Rule(ErroX['EX3'] & ErroYaw['EY3'],Vangular['VA1'])
    ruleA5 = ctrl.Rule(ErroX['EX4'] & ErroYaw['EY3'],Vangular['VA0'])

    ruleA6 = ctrl.Rule(ErroYaw['EY0'],Vangular['VA4'])
    ruleA7 = ctrl.Rule(ErroYaw['EY1'],Vangular['VA4'])
    ruleA8 = ctrl.Rule(ErroYaw['EY2'],Vangular['VA3'])
    ruleA9 = ctrl.Rule(ErroYaw['EY4'],Vangular['VA1'])
    ruleA10 = ctrl.Rule(ErroYaw['EY5'],Vangular['VA0'])
    ruleA11 = ctrl.Rule(ErroYaw['EY6'],Vangular['VA0'])

    ruleA12 = ctrl.Rule(GravidadeY['GY1'],Vangular['VA2'])

    ruleA13 = ctrl.Rule(GravidadeY['GY0'],Vangular['VA1'])
    ruleA14 = ctrl.Rule(GravidadeY['GY2'],Vangular['VA3'])

    ruleL1 = ctrl.Rule(ErroX['EX0'] & ErroYaw['EY3'],Vlinear['VL0'])
    ruleL2 = ctrl.Rule(ErroX['EX1'] & ErroYaw['EY3'],Vlinear['VL1'])
    ruleL3 = ctrl.Rule(ErroX['EX2'] & ErroYaw['EY3'],Vlinear['VL3'])
    ruleL4 = ctrl.Rule(ErroX['EX3'] & ErroYaw['EY3'],Vlinear['VL1'])
    ruleL5 = ctrl.Rule(ErroX['EX4'] & ErroYaw['EY3'],Vlinear['VL0'])

    ruleL6 = ctrl.Rule(ErroYaw['EY0'] & TipoSolda['TS0'],Vlinear['VL3'])
    ruleL7 = ctrl.Rule(ErroYaw['EY1'] & TipoSolda['TS0'],Vlinear['VL1'])
    ruleL8 = ctrl.Rule(ErroYaw['EY2'] & TipoSolda['TS0'],Vlinear['VL0'])
    ruleL9 = ctrl.Rule(ErroYaw['EY4'] & TipoSolda['TS0'],Vlinear['VL0'])
    ruleL10 = ctrl.Rule(ErroYaw['EY5'] & TipoSolda['TS0'],Vlinear['VL1'])
    ruleL11 = ctrl.Rule(ErroYaw['EY6'] & TipoSolda['TS0'],Vlinear['VL3'])

    ruleL12 = ctrl.Rule(ErroYaw['EY0'] & TipoSolda['TS12'],Vlinear['VL0'])
    ruleL13 = ctrl.Rule(ErroYaw['EY1'] & TipoSolda['TS12'],Vlinear['VL2'])
    ruleL14 = ctrl.Rule(ErroYaw['EY2'] & TipoSolda['TS12'],Vlinear['VL2'])
    ruleL15 = ctrl.Rule(ErroYaw['EY4'] & TipoSolda['TS12'],Vlinear['VL2'])
    ruleL16 = ctrl.Rule(ErroYaw['EY5'] & TipoSolda['TS12'],Vlinear['VL2'])
    ruleL17 = ctrl.Rule(ErroYaw['EY6'] & TipoSolda['TS12'],Vlinear['VL0'])

    ruleL18 = ctrl.Rule(GravidadeX['GX0'] & TipoSolda['TS3'],Vlinear['VL3']) # - -
    ruleL18 = ctrl.Rule(GravidadeX['GX0'] & TipoSolda['TS12'],Vlinear['VL3']) # - -
    ruleL19 = ctrl.Rule(GravidadeX['GX0'] & TipoSolda['TS0'],Vlinear['VL2'])

    ruleL20 = ctrl.Rule(GravidadeX['GX1'] & TipoSolda['TS3'],Vlinear['VL2']) # - -
    ruleL21 = ctrl.Rule(GravidadeX['GX1'] & TipoSolda['TS12'],Vlinear['VL2']) # 0
    ruleL22 = ctrl.Rule(GravidadeX['GX1'] & TipoSolda['TS0'],Vlinear['VL2']) # 0

    ruleL23 = ctrl.Rule(GravidadeX['GX2'] & TipoSolda['TS3'],Vlinear['VL1']) # -
    ruleL24 = ctrl.Rule(GravidadeX['GX2'] & TipoSolda['TS12'],Vlinear['VL1']) # +
    ruleL25 = ctrl.Rule(GravidadeX['GX2'] & TipoSolda['TS0'],Vlinear['VL3']) # +

    # ruleL18 = ctrl.Rule(GravidadeZ['GZ0'] & GravidadeX['GX0'],Vlinear['VL2']) # - -
    # ruleL18 = ctrl.Rule(GravidadeZ['GZ0'] & GravidadeX['GX1'],Vlinear['VL2']) # - 0
    # ruleL18 = ctrl.Rule(GravidadeZ['GZ0'] & GravidadeX['GX2'],Vlinear['VL2']) # - +
    #
    # ruleL20 = ctrl.Rule(GravidadeZ['GZ1'] & GravidadeX['GX0'],Vlinear['VL2']) # 0 -
    # ruleL21 = ctrl.Rule(GravidadeZ['GZ1'] & GravidadeX['GX1'],Vlinear['VL2']) # 0 0
    # ruleL22 = ctrl.Rule(GravidadeZ['GZ1'] & GravidadeX['GX2'],Vlinear['VL2']) # 0 +
    #
    # ruleL19 = ctrl.Rule(GravidadeZ['GZ2'] & GravidadeX['GX0'],Vlinear['VL2']) # + -
    # ruleL19 = ctrl.Rule(GravidadeZ['GZ2'] & GravidadeX['GX1'],Vlinear['VL2']) # + 0
    # ruleL19 = ctrl.Rule(GravidadeZ['GZ2'] & GravidadeX['GX2'],Vlinear['VL2']) # + +

    sang_ctrl = ctrl.ControlSystem([ruleA1,ruleA2,ruleA3,ruleA4,ruleA5,ruleA6,ruleA7,ruleA8,ruleA9,ruleA10,
                ruleA11#,ruleA12,ruleA13,ruleA14
                ])
    sang = ctrl.ControlSystemSimulation(sang_ctrl)

    slin_ctrl = ctrl.ControlSystem([ruleL1,ruleL2,ruleL3,ruleL4,ruleL5,ruleL6,ruleL7,ruleL8,ruleL9,ruleL10,
        ruleL11,ruleL12,ruleL13,ruleL14,ruleL15,ruleL16,ruleL17,ruleL18,ruleL19,ruleL20,ruleL21,ruleL22,ruleL23,ruleL24,ruleL25
        ])
    slin = ctrl.ControlSystemSimulation(slin_ctrl)

    return

def calculaVelocidade(erro,orientacao,estado,gravidade_x,gravidade_y,gravidade_z):
    global sang
    global slin

    sang.input['ErroX'] = erro
    sang.input['ErroYaw'] = orientacao
    # sang.input['GravidadeY'] = gravidade_y
    sang.compute()

    slin.input['ErroX'] = erro
    slin.input['ErroYaw'] = orientacao
    slin.input['TipoSolda'] = estado
    slin.input['GravidadeX'] = gravidade_x
    #slin.input['GravidadeY'] = gravidade_y
    # slin.input['GravidadeZ'] = gravidade_z
    slin.compute()

    vang = sang.output['Vangular']
    vlin = slin.output['Vlinear']

    return vang,vlin
