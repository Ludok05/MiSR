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
    robot = rtb.models.DH.Puma560()
    #robot.base = SE3.Rt(SO3.Rx(np.pi), np.array([0, 0, 3]))
    #robot.plot(robot.qn, block=True, limits=None)

    T = robot.fkine(robot.qn)
    ik_solution = robot.ikine_LM(T, q0 = robot.qr)
    robot.plot(ik_solution.q, block=True)

def zadanie_1():
    #Zadanie 1
    l1,l2 = symbol('l1, l2')

    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=l1, alpha=pi()/2),
        rtb.RevoluteDH(offset=pi()/2, alpha=pi()/2),
        rtb.PrismaticDH(offset=l2)
    ], name='MyRobot')
    
    for i in range(3):
        print(robot.links[i])

    #Zadanie 2
    q1, q2, d3 = symbol('q1, q2, d3')
    T=robot.fkine([q1, q2, d3])
    print(T)

    #Zadanie 3
    J = robot.jacob0([q1, q2, d3])
    np.set_printoptions(precision=3, suppress=True)
    print(simplify(J))

    #Zadanie 4
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=1, alpha=np.pi/2),
        rtb.RevoluteDH(offset=np.pi/2, alpha=np.pi/2),
        rtb.PrismaticDH(offset=0.4)
    ], name='MyRobot')

    T=robot.fkine([0.1, 1, 0.4])
    print(T)
    


    
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
    #przyklad_1()
    zadanie_1()
    #zadanie_2(robot)
    #zadanie_3(robot)
    #zadanie_4()
    #zadanie_5()