import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys
path_root = Path(__file__).parents[2]
sys.path.append(str(path_root))
from David_VTOL_LSDO.CarpetPlots import pintref
from math import pi
print("Wahoo.wav")

clmax = 1.45
rho = 1.225414729 #kg/m**3
vstall = 10.3820976 #m/s
vcruise = 20.98227588 #m/s
q = 0.5*rho*vcruise
cdo = 0.0429
e = 0.75
AR = 5.3333
LDmax = 5.7
G = 3.048/(vcruise) #m/s corresponds to 10 ft/s
n = 2

WS_upperlim = 12
WS = np.linspace(1,WS_upperlim,WS_upperlim-1) #kg/m^3
#Stall
WS_stall = clmax*0.5*rho*vstall
#Climb
TW_climb = (cdo*q)/(WS) + WS/(pi*e*AR*q)+G
#Maneuver
TW_Man = (cdo*q)/(WS)+n**2*(WS)/(pi*e*AR*q)

#Ceiling
TW_Ciel = 1/LDmax
plt.figure()
plt.plot(WS,TW_climb,'b')
plt.plot(WS,TW_Man,'g')
plt.axhline(TW_Ciel)
plt.axvline(WS_stall)
plt.ylabel("T/W")
plt.xlabel("W/S")
plt.legend(["Climb at 10ft/s close to stall speed", "Maneuver at 2g","Ceiling","Stall"])
plt.title("T/W vs W/S")
plt.show()

array = np.array([
        [-3.53927E+00,2.96057E-05,-2.63734E+00],
        [-3.55440E+00,9.24183E-01,-2.60233E+00],
        [-3.57151E+00,1.88572E+00,-2.56586E+00],
        [-3.58867E+00,2.84852E+00,-2.52935E+00],
        [-3.60592E+00,3.81277E+00,-2.49278E+00],
        [-3.62204E+00,4.77378E+00,-2.45601E+00],
        [-3.64281E+00,5.73453E+00,-2.41846E+00],
        [-3.67890E+00,6.69733E+00,-2.38051E+00],
        [-3.92518E+00,7.69578E+00,-2.32876E+00],
    ])

plt.figure()
ax = plt.axes(projection ='3d')
ax.plot3D(array[:,0],array[:,1],array[:,2])
plt.show()

print("test")
