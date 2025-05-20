import roboticstoolbox as rtb
import numpy as np
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *


def zadanie_3():
    # wykorzystaj zmienne symboliczne o nazwach:  l1, l2, t1, t2, d3
    l1,l2 = symbol('l1, l2')
    t1, t2, d3 = symbol('t1, t2, d3')
    robot = rtb.DHRobot([
        rtb.RevoluteDH(d=l1, alpha=pi()/2),
        rtb.RevoluteDH(offset=pi()/2, alpha=pi()/2),
        rtb.PrismaticDH(offset=l2)
    ], name='MyRobot')
    
    J = robot.jacob0([t1, t2, d3])
    print(simplify(J))
    # nie umieszczaj w kodzie innych funkcji print

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    zadanie_3()