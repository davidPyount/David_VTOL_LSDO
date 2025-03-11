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
plt.legend(["Climb at 10ft/s", "Maneuver at 2g","Ceiling","Stall"])
plt.title("T/W vs W/S")

#P/W
T_climb = 8 #Newtons
V_climb = vcruise #m/s
P_climb = 152.4 #W
eta_climb = T_climb*V_climb/P_climb
print(f"Climb efficiency is {eta_climb}")
PW_climb = TW_climb*eta_climb

T_man = 3.656 #N
V_man = vcruise
P_man = 69.437 #W
eta_man = T_man*V_man/P_climb
print(f"Maneuver efficiency is {eta_man}")
PW_man = TW_Man*eta_man
plt.figure()
plt.plot(WS,PW_climb)
plt.plot(WS,PW_man)




#V-n diagram
nmaxcritical = 5
nmax = 5
nmin = -3

vs = np.linspace(0,35,100)
upper = (vs/vstall)**2 #This output is ns
lower = -(vs/vstall)**2 #This output is ns
vne = 30

SFy = 1.25
SFu = 1.5

plt.figure()
ax = plt.gca()
ax.set_xlim([0, vne+5])
ax.set_ylim([-6, 8])

idxu = int(np.argwhere(np.diff(np.sign(upper - nmax*SFu))).flatten()[0]) + 1
upper = upper[:idxu]
idxl = int(np.argwhere(np.diff(np.sign(lower - nmin*SFu))).flatten()[0]) + 1
lower = lower[:idxl]

#Corner speed
idxc = int(np.argwhere(np.diff(np.sign(upper - nmax))).flatten()[0]) + 1
vcorner = vs[idxc]
print(f"Corner speed is {vcorner}")
plt.plot(vs[idxc],upper[idxc],'o')

plt.plot(vs[:idxu],upper,label ="Positive Stall Limit", color = 'b')
plt.hlines(nmax,23,vne, label ="Max allowable n",color = 'm')
plt.hlines(nmin,18,vne,label = "Max allowable -n", color = 'g')
plt.axvline(vne,label = "Never Exceed Speed", color = 'r')
plt.plot(vs[:idxl],lower,label = "Negative Stall Limit", color = 'c')
plt.hlines(SFy*nmax,26,vne,label = "Yield Limit Upper", color = 'y')
plt.hlines(SFu*nmax,28,vne,label = "Ultimate Limit Upper",color = 'r')
plt.hlines(SFy*nmin,20,vne,label = "Yield Limit Lower", color = 'y')
plt.hlines(SFu*nmin,22,vne,label = "Ultimate Limit Lower", color = 'r')
plt.title("Performance Envelope")
plt.ylabel("Load Factor [g]")
plt.xlabel("Airspeed [m/s]")
plt.legend(loc="upper left")
plt.show()

print("test")
