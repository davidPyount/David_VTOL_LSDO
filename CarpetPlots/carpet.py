import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys
path_root = Path(__file__).parents[2]
sys.path.append(str(path_root))
from David_VTOL_LSDO.CarpetPlots import pintref
from math import pi
from math import sqrt
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

nmax = 5
nmin = -2
ns = np.linspace(0,6,100)
#V-n diagram
upper = vstall*ns**2
lower = -1*abs(nmin)*ns**2

plt.figure()
ax = plt.gca()
ax.set_xlim([0, vstall+5])
ax.set_ylim([-4, 8])
plt.plot(ns,upper)
plt.axhline(nmax)
plt.axhline(nmin)
plt.axvline(vstall)
plt.plot(ns,lower)
plt.ylabel("Load Factor")
plt.xlabel("Airspeed")
plt.show()
print("test")
