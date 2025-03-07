import matplotlib.pyplot as plt
import numpy as np
from pintref import u,Q_

g = 32.174 #ft/s^2
rho = 0.002378 #slug/ft^3
mu = 3.737E-7 #slug/ft/s

# Gross weight
print("The non-optimized baseline Lift+Cruise eVTOL was designed with given requirements for wingspan, payload weight, and gross weight. \
    Best practices are used to determine parameters such as wing loading from statistical regressions (RC Aircraft wisdown). \
    It is important to note that conventional aircraft statistical regressions often do not apply or do not apply well at the RC scale. \
    In general, most RC planes are designed with an overpowered engine as a safety factor, as oversupplying engine power does not come \
    at such a substantial weight disadvantage as in full scale aircraft")

#I think we might need to input a sizing phase here

wg = int(input("Enter gross weight [lbs]:"))
wp = int(input("Enter payload weight {lbs]: "))

wempt = wg-wp
wstruct_batt = wempt #Because at this point we don't know what our battery weight is

wl = int(input("Enter a chosen wing loading [lb/ft^2]"))
b = int(input("Enter a chosen wingspan"))
clmax = int(input("What is the max lift coefficient of your airfoil (XLR5)"))
clcruise = int(input("What is the lift coefficient at cruise angle of attack (XLRF5)"))
e = int(input("What is your chosen oswald efficiency factor"))




