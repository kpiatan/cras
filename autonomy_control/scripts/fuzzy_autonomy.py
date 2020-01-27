#!/usr/bin/env python

import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np
import matplotlib.pyplot as plt

autonomy = 0

def inicializaFuzzy():
    global autonomy

    #inputs
    FrontSensor = ctrl.Antecedent(np.arange(0,3,0.005),'FrontSensor')
    BackSensor = ctrl.Antecedent(np.arange(0,3,0.005),'BackSensor')
    JoyLinear = ctrl.Antecedent(np.arange(-1,1,0.005),'JoyLinear')
    JoyAngular = ctrl.Antecedent(np.arange(-2,2,0.005),'JoyTheta')
    WeldPos = ctrl.Antecedent(np.arange(-1,1,0.005),'WeldPos')

    #outputs
    LoA = ctrl.Consequent(np.arange(1,4,1),'LoA')


    #membership functions
    #front sensor
    FrontSensor['Close'] = fuzz.trimf(FrontSensor.universe,[0, 0, 0.5])
    FrontSensor['Medium'] = fuzz.trimf(FrontSensor.universe,[0, 0.5, 1])
    FrontSensor['Far'] = fuzz.trimf(FrontSensor.universe,[1, 3, 3])

    #back sensor
    BackSensor['Close'] = fuzz.trimf(BackSensor.universe,[0, 0, 0.5])
    BackSensor['Medium'] = fuzz.trimf(BackSensor.universe,[0, 0.5, 1])
    BackSensor['Far'] = fuzz.trimf(BackSensor.universe,[1, 3, 3])

    #velocity linear - joystick
    JoyLinear['NegHigh'] = fuzz.trimf(JoyLinear.universe,[-1, -1, -0.5])    
    JoyLinear['NegMedium'] = fuzz.trimf(JoyLinear.universe,[-1, -0.5, -0.1])
    JoyLinear['NegLow'] = fuzz.trimf(JoyLinear.universe,[-0.5, -0.1, 0])
    JoyLinear['Zero'] = fuzz.trimf(JoyLinear.universe,[-0.1, 0, 0.1])
    JoyLinear['PosLow'] = fuzz.trimf(JoyLinear.universe,[0, 0.1, 0.5])
    JoyLinear['PosMedium'] = fuzz.trimf(JoyLinear.universe,[0.1, 0.5, 1])
    JoyLinear['PosHigh'] = fuzz.trimf(JoyLinear.universe,[0.5, 1, 1])

    #velocity angular - joystick
    JoyAngular['LeftHigh'] = fuzz.trimf(JoyAngular.universe,[-2, -2, -1])    
    JoyAngular['LeftMedium'] = fuzz.trimf(JoyAngular.universe,[-2, -1, -0.5])
    JoyAngular['LeftLow'] = fuzz.trimf(JoyAngular.universe,[-1, -0.5, 0])
    JoyAngular['Center'] = fuzz.trimf(JoyAngular.universe,[-0.5, 0, 0.5])
    JoyAngular['RightLow'] = fuzz.trimf(JoyAngular.universe,[0, 0.5, 1])
    JoyAngular['RightMedium'] = fuzz.trimf(JoyAngular.universe,[0.5, 1, 2])
    JoyAngular['RightHigh'] = fuzz.trimf(JoyAngular.universe,[1, 2, 2])

    #weld position
    WeldPos['LeftHigh'] = fuzz.trimf(WeldPos.universe,[-1, -1, -0.5])    
    WeldPos['LeftMedium'] = fuzz.trimf(WeldPos.universe,[-1, -0.5, -0.1])
    WeldPos['LeftLow'] = fuzz.trimf(WeldPos.universe,[-0.5, -0.1, 0])
    WeldPos['Center'] = fuzz.trimf(WeldPos.universe,[-0.5, 0, 0.5])
    WeldPos['RightLow'] = fuzz.trimf(WeldPos.universe,[0, 0.1, 0.5])
    WeldPos['RightMedium'] = fuzz.trimf(WeldPos.universe,[0.1, 0.5, 1])
    WeldPos['RightHigh'] = fuzz.trimf(WeldPos.universe,[0.5, 1, 1])

    #Level of Autonomy - LoA
    LoA['Manual'] = fuzz.trimf(LoA.universe,[1, 1, 2])
    LoA['Shared'] = fuzz.trimf(LoA.universe,[1, 2, 3])
    LoA['Supervisory'] = fuzz.trimf(LoA.universe,[2, 3, 4])
    LoA['Autonomous'] = fuzz.trimf(LoA.universe,[3, 4, 4])


    #fuzzy rules
    rule1 = ctrl.Rule(FrontSensor['Close'] & JoyLinear['PosHigh'],LoA['Autonomous'])
    rule2 = ctrl.Rule(FrontSensor['Close'] & JoyLinear['PosMedium'],LoA['Supervisory'])
    rule3 = ctrl.Rule(FrontSensor['Medium'] & JoyLinear['PosMedium'],LoA['Shared'])
    rule4 = ctrl.Rule(FrontSensor['Far'] & JoyLinear['PosLow'],LoA['Manual'])

    rule5 = ctrl.Rule(BackSensor['Close'] & JoyLinear['NegHigh'],LoA['Autonomous'])
    rule6 = ctrl.Rule(BackSensor['Close'] & JoyLinear['NegMedium'],LoA['Supervisory'])
    rule7 = ctrl.Rule(BackSensor['Medium'] & JoyLinear['NegMedium'],LoA['Shared'])
    rule8 = ctrl.Rule(BackSensor['Far'] & JoyLinear['NegLow'],LoA['Manual'])

    rule9 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftHigh'],LoA['Manual'])
    rule10 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftMedium'],LoA['Manual'])
    rule11 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['LeftLow'],LoA['Shared'])
    rule12 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['Center'],LoA['Shared'])
    rule13 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightLow'],LoA['Supervisory'])
    rule14 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightMedium'],LoA['Supervisory'])
    rule15 = ctrl.Rule(JoyAngular['LeftHigh'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule16 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftHigh'],LoA['Manual'])
    rule17 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftMedium'],LoA['Manual'])
    rule18 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['LeftLow'],LoA['Manual'])
    rule19 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['Center'],LoA['Shared'])
    rule20 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightLow'],LoA['Shared'])
    rule21 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightMedium'],LoA['Supervisory'])
    rule22 = ctrl.Rule(JoyAngular['LeftMedium'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule23 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftHigh'],LoA['Shared'])
    rule24 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftMedium'],LoA['Manual'])
    rule25 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['LeftLow'],LoA['Manual'])
    rule26 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['Center'],LoA['Manual'])
    rule27 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightLow'],LoA['Shared'])
    rule28 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightMedium'],LoA['Supervisory'])
    rule29 = ctrl.Rule(JoyAngular['LeftLow'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule30 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule31 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule32 = ctrl.Rule(JoyAngular['Center'] & WeldPos['LeftLow'],LoA['Shared'])
    rule33 = ctrl.Rule(JoyAngular['Center'] & WeldPos['Center'],LoA['Manual'])
    rule34 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightLow'],LoA['Shared'])
    rule35 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightMedium'],LoA['Supervisory'])
    rule36 = ctrl.Rule(JoyAngular['Center'] & WeldPos['RightHigh'],LoA['Autonomous'])

    rule37 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule38 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule39 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['LeftLow'],LoA['Shared'])
    rule40 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['Center'],LoA['Manual'])
    rule41 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightLow'],LoA['Manual'])
    rule42 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightMedium'],LoA['Manual'])
    rule43 = ctrl.Rule(JoyAngular['RightLow'] & WeldPos['RightHigh'],LoA['Shared'])

    rule44 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule45 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule46 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['LeftLow'],LoA['Shared'])
    rule47 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['Center'],LoA['Shared'])
    rule48 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['RightLow'],LoA['Manual'])
    rule49 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['RightMedium'],LoA['Manual'])
    rule50 = ctrl.Rule(JoyAngular['RightMedium'] & WeldPos['RightHigh'],LoA['Manual'])

    rule51 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftHigh'],LoA['Autonomous'])
    rule52 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftMedium'],LoA['Supervisory'])
    rule53 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['LeftLow'],LoA['Supervisory'])
    rule54 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['Center'],LoA['Shared'])
    rule55 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightLow'],LoA['Shared'])
    rule56 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightMedium'],LoA['Manual'])
    rule57 = ctrl.Rule(JoyAngular['RightHigh'] & WeldPos['RightHigh'],LoA['Manual'])


    #fuzzy control
    autonomy_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
                rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18, rule19, rule20,
                rule21, rule22, rule23, rule24, rule25, rule26, rule27, rule28, rule29, rule30,
                rule31, rule32, rule33, rule34, rule35, rule36, rule37, rule38, rule39, rule40,
                rule41, rule42, rule43, rule44, rule45, rule46, rule47, rule48, rule49, rule50,
                rule51, rule52, rule53, rule54, rule55, rule56, rule57
                ])
    autonomy = ctrl.ControlSystemSimulation(autonomy_ctrl)

    return

def calculateAutonomy(fsensor,bsensor,jlinear,jangular,wpos):
    global autonomy

    autonomy.input['FrontSensor'] = fsensor
    autonomy.input['BackSensor'] = bsensor
    autonomy.input['JoyLinear'] = jlinear
    autonomy.input['JoyAngular'] = jangular
    autonomy.input['WeldPos'] = wpos
    autonomy.compute()

    autonomy_level = autonomy.output['LoA']

    return autonomy_level