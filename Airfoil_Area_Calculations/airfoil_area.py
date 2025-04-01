import numpy as np
import matplotlib.pyplot as plot
import math as mt

#https://web.itu.edu.tr/~atares/courses/CA/3.1.1_NACA4.html
# NACA 4 Digit Airfoil Generation

#INPUTS ARE AR AND WING AREA S
def wing_weight_model(AR,S,m,p,t,spar_outer_diameter):
    b = mt.sqrt(AR*S) #ft
    c = b/AR #ft

    m = 0.01*m  # maximum camber in % of chord
    p = 0.10*p  # maximum camber position in tenths of chord
    t = 0.01*t  # thickness in % of chord

    # Coefficients for 4 digit series
    a0 =  1.4845
    a1 = -0.6300
    a2 = -1.7580
    a3 =  1.4215
    a4 = -0.5075

    n = 1000 # number of points along the chord
    x = np.linspace(0,c,n) # x coordinate of points along the chord
    y   = np.zeros(n) # x coordinate of points along the chord
    yc  = np.zeros(n) # y coordinate of the camber line
    dyc = np.zeros(n) # gradient of the camber line
    yt  = np.zeros(n) # thickness distribution
    xu  = np.zeros(n) # x coordinate of the upper surface
    yu  = np.zeros(n) # y coordinate of the upper surface
    xl  = np.zeros(n) # x coordinate of the lower surface
    yl  = np.zeros(n) # y coordinate of the lower surface
    for i in range(n):
        if  (x[i]/c < p):
            yc[i]  = (c*m/p**2)*(2*p*(x[i]/c)-(x[i]/c)**2)
            dyc[i] = ((2*m)/p**2)*(p-(x[i]/c))
        else:
            yc[i]  = (c*m/(1-p)**2)*(1-2*p+2*p*(x[i]/c)-(x[i]/c)**2)
            dyc[i] = ((2*m)/(1-p)**2)*(p-(x[i]/c))
            
    for i in range(n):
        yt[i] = (t*c)*(a0*mt.sqrt(x[i]/c)+a1*(x[i]/c)+a2*(x[i]/c)**2+a3*(x[i]/c)**3+a4*(x[i]/c)**4)
        teta  = mt.atan(dyc[i])
        xu[i] = x[i]  - yt[i]*mt.sin(teta)
        xl[i] = x[i]  + yt[i]*mt.sin(teta)
        yu[i] = yc[i] + yt[i]*mt.cos(teta)
        yl[i] = yc[i] - yt[i]*mt.cos(teta)


    # plot.xlim(-0.2,c+0.2)
    # plot.ylim(-c/3,c/3)
    # plot.plot(xu,yu,color='deepskyblue')   
    # plot.plot(xl,yl,color='deepskyblue')
    # plot.plot(x,yc,'g--') 

    upper = np.trapz(yu,xu)
    lower = np.trapz(yl,xl)

    total_area = upper + -(lower)

    foam_density = 1.5 #lb/ft**3
    volume_wing = total_area * b
    volume_spars = 2*mt.pi*(spar_outer_diameter/2/12)**2*b #Volume of both spars in ft^3
    volume_total = volume_wing-volume_spars
    weight = volume_total*foam_density
AR = 5.3333
S = 3 #ft^2
wing_weight_model(AR,S,4,4,12,0.5)