# wczytanie potrzebnych podczas zajęć bibliotek:
import roboticstoolbox as rtb
import numpy as np
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
from matplotlib import pyplot as plt
# ...

# definicje funkcji:
def przyklad_1():
    R2 = SO3.Rz(30, 'deg')
    print(R2)


def zadanie_1():
    pass # zastąp tę linię swoim kodem

def zadanie_2():
    pass # zastąp tę linię swoim kodem

def zadanie_3():
    d_p = np.array([1, -2, 2])
    t_bs = np.array([2, 5, 0])
    t_ps = np.array([3, -6, 4])
    R_bs = SO3(np.array([[0, -1, 0], [1, 0, 0],[0, 0, 1]]))
    T_bs = SE3.Rt(R_bs , t_bs)
    R_ps = SO3(np.array([[-1, 0, 0], [0, 1, 0],[0, 0, -1]]))
    T_ps = SE3.Rt(R_ps , t_ps)
    T_sp = T_ps.inv()
    d_s = T_sp * d_p
    d_b = T_bs * d_s
    print(d_b.T)

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    #przyklad_1()
    zadanie_3()