import numpy as np
import sympy as sym
import math as m
import matplotlib.pyplot as plt

#This script is used to find the constant of proportionalty relating airfoil area to wing chord
#Ok, so the area of an airfoil is proportional to the chord, with constant of proportionality determined by aifroil shape.
#To determine airfoil area, integrate oh I could just find this with solidworks omgfg

def Four_Dig_Airfoil_Area(num,c,steps):
    if steps % 2 != 0:
        print("Please use even numbers for the amount of grid points")
        quit()

    m = num[0]/100
    p = num[1]/10

    x_bp = np.linspace(0,p,int(steps/2))
    x_up = np.linspace(p,1,int(steps/2))

    yc_bp = m/p**2*(2*p*x_bp-x_bp**2)
    yc_up = m/(1-p**2)*((1-2*p)+2*p*x_up-x_up**2)
    
    #Weird testing
    x = p
    yc_bp_test = (m/p**2)*(2*p*x-x**2)
    yc_up_test = m/(1-p**2)*((1-2*p)+2*p*x-x**2)
    #These two are not equal and that shouldn't be possible.
    print(yc_bp_test)
    print(yc_up_test)

    t = (num[2]*10+num[3])/100
    yt_bp = 5*t*(0.2969*np.sqrt(x_bp)-0.1260*x_bp-0.3516*x_bp**2+0.2843*x_bp**3-0.1015*x_bp**4)
    yt_up = 5*t*(0.2969*np.sqrt(x_up)-0.1260*x_up-0.3516*x_up**2+0.2843*x_up**3-0.1015*x_up**4)
 
    dycdx_bp = (2*m)/(p**2)*(p-x_bp)
    dycdx_up = (2*m)/((1-p)**2)*(p-x_up)
    theta_bp = np.arctan(dycdx_bp)
    theta_up = np.arctan(dycdx_up)
    #Final coordinate generation
    xu_bp = x_bp - yt_bp*np.sin(theta_bp)
    xu_up = x_up - yt_up*np.sin(theta_up)

    yu_bp = yc_bp + yt_bp*np.cos(theta_bp)  #suspect
    yu_up = yc_up + yt_up*np.cos(theta_up)

    xl_bp = x_bp + yt_bp*np.sin(theta_bp)
    xl_up = x_up + yt_up*np.sin(theta_up)
    
    yl_bp = yc_bp - yt_bp*np.cos(theta_bp) #suspect
    yl_up = yc_up - yt_up*np.cos(theta_up)

    plt.plot(xu_bp,yu_bp, label = "Upper Curve Before p") #suspect
    plt.plot(xu_up,yu_up, label = "Upper Curve After p")
    plt.plot(xl_bp,yl_bp, label = "Lower Curve Before p") #suspect
    plt.plot(xl_up,yl_up, label = "Lower Curve After p")
    plt.plot(x_bp,yc_bp)
    plt.plot(x_up,yc_up)
    plt.legend()
    plt.show()




area = Four_Dig_Airfoil_Area(np.array([4,4,1,2]),3,1000)