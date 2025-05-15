# wczytanie potrzebnych podczas zajęć bibliotek:
import roboticstoolbox as rtb
import numpy as np
import math
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
import time
# ...

# definicje funkcji:
def przyklad_1():
    robot = rtb.DHRobot(
        [
            rtb.RevoluteDH(alpha= np.pi / 2),
            rtb.RevoluteDH(a=0.4318),
            rtb.RevoluteDH(d=0.15005, a=0.0203, alpha= -np.pi / 2),
            rtb.RevoluteDH(d=0.4318, alpha= np.pi / 2),
            rtb.RevoluteDH(alpha= -np.pi / 2),
            rtb.RevoluteDH()
        ], name = "My_Robot")
    

    robot = rtb.models.DH.Puma560()
    robot.addconfiguration_attr('conf', [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    print(robot)
    #robot.plot(robot.conf, block=True, limits=None)
    #robot.teach(robot.qn)

    #Kinematyka prosta (współrzędne konfiguracyjne -> położenie końcówi)
    T = robot.fkine(robot.conf)
    print(T)
    robot.base = SE3(0, 0, 3)
    T=robot.fkine(robot.conf)
    print(T)

    #Kinematyka odworotna (położenie końcówki -> współrzędne konfiguracyjne)
    robot = rtb.models.DH.Puma560()
    T = robot.fkine(robot.qn)
    ik_solution = robot.ikine_a(T=T, config="rd")
    print(ik_solution.q)
    #robot.plot(ik_solution.q, block=True)

    #Jakobian
    robot = rtb.models.DH.Puma560()
    J = robot.jacob0(robot.q)
    np.set_printoptions(precision=3, suppress=True)
    print(J)

    q_tmp = [0.5, 0., 0.9, 1.2, 0.2, 0.7]
    J = robot.jacob0(q_tmp)
    print(J)

    
def zadanie_1():
    robot = None # zastąp tę linię swoim kodem
    return robot
    
def zadanie_2(robot):
    pass # zastąp tę linię swoim kodem

def zadanie_3(robot):
    pass # zastąp tę linię swoim kodem

def zadanie_4():
    pass # zastąp tę linię swoim kodem

def zadanie_5():
    pass # zastąp tę linię swoim kodem

# ...

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    przyklad_1()
    #robot = zadanie_1()
    #zadanie_2(robot)
    #zadanie_3(robot)
    #zadanie_4()
    #zadanie_5()

