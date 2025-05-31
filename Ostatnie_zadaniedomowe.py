import roboticstoolbox as rtb
import numpy as np
from matplotlib.pyplot import figure
from spatialmath import *
from spatialmath.base import *
from matplotlib import pyplot as plt
from roboticstoolbox.tools.trajectory import *
from roboticstoolbox.backends.swift import Swift

def sphere_param(theta = 0 ,r = 1, z = 0 ,x0 = 0, y0 = 0):
    return [r*np.cos(theta)+x0,r*np.sin(theta)+y0,z]


def check_smooth_traj(sol, jump_threshold=0.15):
    dists = []
    for current_q, next_q in zip(sol.q[1:], sol.q[2:]):
        for j1, j2 in zip(current_q, next_q):
            dist = np.fabs(j1 - j2)
            dists.append(dist)
    print(max(dists))
    return max(dists) < jump_threshold


def zadanie_3():
    points_number = 50
    eef_height = 0.15
    r = 0.1
    x0 = 0.65
    y0 = 0.2

    angles =  np.linspace( 0, 2*np.pi, points_number)# TODO: utwórz listę kątów od 0 do 2pi o długości points_number

    Pt_list = []
    for angle in angles:
        Pt_list.append(sphere_param(angle,r,eef_height,x0,y0))  # TODO: do listy Pt_list dla każdego kąta dodaj punkt [x,y,z] leżący na okręgu (wykorzystaj równanie parametryczne okręgu)

    Pt_list = np.asarray(Pt_list)
    x_toplot = Pt_list[:, 0]
    y_toplot = Pt_list[:, 1]

    # TODO: wykreśl wykres punktów o współrzędnych x_toplot, y_toplot
    rtb.xplot(x_toplot,y_toplot)

    robot = rtb.models.DH.Panda()  # TODO: załaduj robota Panda


    T_list = robot.fkine(robot.qz) # TODO: do listy dodaj pierwszą macierz 4x4 - pozycję końcówki dla konfiguracji robot.qz
    T_list.extend(SE3(Pt_list)*SE3.Rx(180,'deg'))  # TODO: rozszerz listę o listę macierzy 4x4 leżących na okręgu (do utworzenia listy macierzy użyj listy punktów Pt_list, pamiętaj o zadaniu orientacji - chwytak w dół)

    smooth_traj = False
    while not smooth_traj:
        sol = robot.ikine_LM(T_list)  # TODO: oblicz kinematykę odwrotną dla listy macierzy transformacji
        if not sol.success:
            print("IK failed!\n")
            return
        smooth_traj = check_smooth_traj(sol)

    traj = mstraj(sol.q,dt=0.02,qdmax = 2.0,tacc=0.2)  # TODO: utwórz trajektorię o wielu odcinkach - lista waypointów to lista konfiguracji z rozwiązania kin. odwr.
    robot.plot(traj.q, backend = 'pyplot', loop = True, movie='panda_pyplot.gif')

if __name__ == '__main__':
    zadanie_3()
