import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys
path_root = Path(__file__).parents[2]
sys.path.append(str(path_root))
from David_VTOL_LSDO.CarpetPlots import pintref
from math import pi
from math import sqrt
import math as m
import sympy as sym

g = 9.81 #m/s**2
clmax = 1.45
rho = 1.225414729 #kg/m**3
vstall = 10.3820976 #m/s
vcruise = 20.98227588 #m/s
q = 0.5*rho*vcruise**2
cdo = 0.0429
e = 0.75
AR = 5.3333
LDmax = 5.7
dhdt = 3.048 #m/s
G = dhdt/(vcruise) #m/s corresponds to 10 ft/s
n = 1.5
W = 26.6704 #N
S = 0.2787 #m^2


k = 1/(pi*e*AR)

## Finding climb velocity based on excess power and predicted Cl and climb
#from following link https://aerodynamics4students.com/aircraft-performance/climb-and-descent.php
x = sym.Symbol('x')
T_climb = 9 #13.5 #N, max available thrust
cubicV = (-0.5*rho*x**3*(cdo+k*W/(0.5*rho*x**2*S)**2))/W + (T_climb*x)/W - dhdt
V_climb = np.array(sym.solve(cubicV,x))
V_climb = abs(V_climb[0]) #If you change things look at this to make sure it makes sense.
print(f"Climb speed is {V_climb} m/s")
cl_climb = W/(0.5*rho*V_climb**2*S)
print(f"CLimb Cl is {cl_climb}")

## Finding turning performance

#Specify turn radius, and either bank angle (load factor) or rate of turn.
r = 30.48 #m
turnChoice = "Specify Bank Angle" #"Specify Omega"

if turnChoice == "Specify Bank Angle":
    phi = 30 #turn angle in degrees
    V_man2 = m.sqrt(r*m.tan(m.radians(phi))*g)
    omega = g/V_man2*m.tan(m.degrees(phi))
    print(F"Turn airspeed is {V_man2} m/s.")
    print(f"Turn rate is {omega} degrees/second")
elif turnChoice == "Specify Omega":
    omega = m.radians(3) #deg/s
    V_man2 = r*omega
    print(F"Turn airspeed is {V_man2} m/s.")
    phi = m.degrees(m.atan(omega*V_man2/g))
    print(f"Bank angle is {phi} degrees")
else:
    print("Incorrect turnChoice")

#Load factor
n_new = 1/(m.cos(m.radians(phi)))

#Check that velocity results in positive or zero excess specific power
#Same thing as before but this time the excess power dh/dt goes into making the turn happen, not gaining altitude.
#T_man = 8.031820154 #N max thrust available
eqTW = (-0.5*rho*V_man2**3*(cdo+k*(n_new*W/(0.5*rho*V_man2**2*S))**2))/W + x*V_man2
TW_Man2 = sym.solve(eqTW,x)
TW_Man2 = float(TW_Man2[0])
#Solve inherently equates eqTW to 0, meaning no excess power, which should mean minimum TW for this maneuver
b = type(TW_Man2)
print(f"The maneuver T/W is {TW_Man2}")
#check, cl cannot be greater than cl_max which is 1.5. This is still useful I guess
cl_check = (n_new*W)/(0.5*rho*V_man2**2*S)
print(f"The reported cl is {cl_check}")

P_man2 = 355.5 #[W] Hard to know exactly
eta_man2 = (TW_Man2*W*V_man2)/P_man2 #Minimizing eta maximises PW, so taking a conservatibe (large) P_man results in conservative PW
PW_man2 = TW_Man2/0.008553936
print(f"Maneuver power efficiency is {eta_man2}.")
print(f"P/W required for maneuver is {PW_man2} W/lb.")

#T/W Stuf =================================================================================

#Wing loading we chose
loading = 95 #pascals
n_new = 2.25 #########

WS_upperlim = 100
WS = np.linspace(0.5,WS_upperlim,WS_upperlim-1) #kg/m^3
#Stall
WS_stall = clmax*0.5*rho*vstall**2
#Climb
TW_climb = (cdo*0.5*rho*V_climb**2)/(WS) + (WS)/(pi*e*AR*0.5*rho*V_climb**2)+dhdt/V_climb
#Maneuver
TW_Man = (cdo*0.5*rho*V_man2**2)/(WS) + n_new**2*(WS)/(pi*e*AR*0.5*rho*V_man2**2)

#Ceiling
TW_Ciel = 1/LDmax
plt.figure()
ax = plt.gca()
ax.set_xlim([0, 100])
ax.set_ylim([0, 2])
plt.plot(WS,TW_climb,'b')
plt.plot(WS,TW_Man,'g')
plt.axhline(TW_Ciel)
plt.axvline(WS_stall,color='r')
plt.axvline(loading,label = "Design Loading", color = 'c')
TW_design = 0.506666667
plt.plot(loading,TW_design,'o')
plt.ylabel("T/W")
plt.xlabel("W/S [Pa]")
n_new = round(n_new,3)
plt.legend(["Climb at 10ft/s", f"Maneuver at {n_new}g","Ceiling","Stall","Design Loading","Design T/W"])
plt.title("T/W vs W/S")

#P/W Ratio ==============================================================

P_climb = 355.5 #W assuming full throttle
eta_climb1 = (T_climb*V_climb)/P_climb
eta_climb = 0.008553936 #lb/W thrust efficiency derived from cobramotors static Grams/W
print(f"Climb efficiency calculated from the Hwang method is {eta_climb1} compared to {eta_climb}")
PW_climb = TW_climb/eta_climb


P_man = 355.5 #W
T_man = 13.5 #N
eta_man1 = (T_man*V_man2)/P_man
eta_man = 0.008553936 #lb/W thrust efficiency derived from cobramotors static Grams/W
PW_man = TW_Man/eta_man
print(f"Maneuver efficiency calculated from the Hwang method is {eta_man1} compared to {eta_man}")
plt.figure()
ax = plt.gca()
ax.set_xlim([0, 100])
ax.set_ylim([0, 100])
plt.plot(WS,PW_climb,label = "Climb at 10ft/s")
plt.plot(WS,PW_man, label = f"Maneuver at {n_new}g")
plt.axvline(WS_stall, label = "Stall",color='r')
plt.axvline(loading,label = "Design Loading", color = 'c')
PW_design = 59.25 #W/lb
plt.plot(loading,PW_design,'o',label = "Design P/W")
plt.ylabel("P/W [W/lb]")
plt.xlabel("W/S [Pa]")
plt.title("P/W vs W/S")
plt.legend()









#V-n diagram =======================================================
nmaxcritical = 5
nmax = 5
nmin = -3

vs = np.linspace(0,35,100)
upper = (vs/vstall)**2 #This output is ns
lower = -(vs/vstall)**2 #This output is ns
vne = 60

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
print(f"Corner speed is {vcorner} m/s")
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

