# wczytanie potrzebnych podczas zajęć bibliotek:
import roboticstoolbox as rtb
import numpy as np
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
from matplotlib import pyplot as plt

# UWAGA! DOSTOSUJ NAZWY ZMIENNYCH DO PODANYCH PONIZEJ.
# Nalezy pozostawic nazwy ponizszych zmiennych bez zmian!
def zadanie_3_1():
    pD = np.array([1, -2, 2])
    pTs = SE3.Rt(SO3.Ry(np.pi), np.array([3,-6,4])) #APS
    bTs = SE3.Rt(SO3.Rz(np.pi/2), np.array([2,5,0])) #ABS
    ASP = pTs.inv()
    sD = ASP * SE3(pD)
    bD = bTs * sD
    # nie umieszczaj w kodzie innych funkcji print
    print(f'pTs:\n{pTs}\nbTs:\n{bTs}\nsD.t:\n{sD.t}\nbD.t:\n{bD.t}') # pozostaw ta linie bez zmian

# wykonywanie wybranej funkcji
if __name__ == '__main__':
    zadanie_3_1()