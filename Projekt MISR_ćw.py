# wczytanie potrzebnych podczas zajęć bibliotek:
import roboticstoolbox as rtb
import numpy as np
import math
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
import time
# ...

d1, d2, d3, q1, q2 = symbol('d1, d2, d3, q1, q2')
robot1 = rtb.DHRobot(
    [
        rtb.RevoluteDH(d = d1, alpha = -1*pi()/2),
        rtb.RevoluteDH(d = d2, alpha = pi()/2),
        rtb.PrismaticDH(),
    ], name="My_Robot2")
T2 = robot1.fkine([q1, q2, d3])
J = robot1.jacob0([q1,q2,d3])
#ik_solution = robot1.ikine_LM(T2, joint_limits = [-100, 100, -100, 100, -100, 100])
print(T2)
print(simplify(J))
#print(simplify(ik_solution))