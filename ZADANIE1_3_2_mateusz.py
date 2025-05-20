import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
from spatialmath.base.symbolic import *
import numpy as np
import matplotlib.pyplot as plt


# Twoj kod wprowadz ponizej
dp = np.array([1, -2, 2])

ABS = SE3.Rt(SO3.Rz(np.pi/2), np.array([2,5,0]))
APS = SE3.Rt(SO3.Ry(np.pi), np.array([3,-6,4]))
ASP = APS.inv()

ds = ASP*dp
db = ABS*ds

#Wykres
s = ABS         #Układ S względem układu B
p = ABS*ASP     #Układ P względem układu B
b = SE3(0,0,0)  #Układ B względem układu B

s.plot(frame='S')
p.plot(frame='P')
b.plot(frame='B')
plt.show()

# Nie zmieniaj ponizszych linii, nie umieszczaj w kodzie zadnych funkcji print
plt.quiver(0, 0, 0, -2, 7, 2)
plt.savefig('outputimage.png')